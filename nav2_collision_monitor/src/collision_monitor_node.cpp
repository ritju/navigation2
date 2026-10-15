// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{
namespace
{

/** Longer side of axis-aligned bounding box of footprint (robot "length" scale). */
double footprint_bbox_max_extent(const std::vector<Point>& fp)
{
  if (fp.size() < 2)
  {
    return 0.0;
  }
  double min_x = fp[0].x;
  double max_x = fp[0].x;
  double min_y = fp[0].y;
  double max_y = fp[0].y;
  for (size_t i = 1; i < fp.size(); ++i)
  {
    min_x = std::min(min_x, fp[i].x);
    max_x = std::max(max_x, fp[i].x);
    min_y = std::min(min_y, fp[i].y);
    max_y = std::max(max_y, fp[i].y);
  }
  return std::max(max_x - min_x, max_y - min_y);
}

/** Ray-crossing point-in-polygon test (same algorithm as Polygon::isPointInside). */
bool isPointInsidePolygon(const Point& point, const std::vector<Point>& poly)
{
  const int poly_size = static_cast<int>(poly.size());
  if (poly_size < 3)
  {
    return false;
  }
  int i = poly_size - 1;
  bool res = false;
  for (int j = 0; j < poly_size; ++j)
  {
    if ((point.y <= poly[i].y) == (point.y > poly[j].y))
    {
      const double x_inter =
          poly[i].x + (point.y - poly[i].y) * (poly[j].x - poly[i].x) / (poly[j].y - poly[i].y);
      if (x_inter > point.x)
      {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

int countPointsInsidePolygon(const std::vector<Point>& points, const std::vector<Point>& poly)
{
  int num = 0;
  for (const Point& point : points)
  {
    if (isPointInsidePolygon(point, poly))
    {
      ++num;
    }
  }
  return num;
}

}  // namespace

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions& options)
  : nav2_util::LifecycleNode("collision_monitor", "", options)
  , process_active_(false)
  , robot_action_prev_{ DO_NOTHING, { -1.0, -1.0, -1.0 } }
  , stop_stamp_{ 0, 0, get_clock()->get_clock_type() }
  , stop_pub_timeout_(1.0, 0.0)
  , recover_stop_timeout_(10.0, 0.0)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn CollisionMonitor::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic, 1, std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, 1);

  if (use_local_plan_collision_check_)
  {
    local_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        local_plan_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&CollisionMonitor::localPlanCallback, this, std::placeholders::_1));
    local_plan_footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), local_plan_footprint_topic_, *tf_buffer_, base_frame_id_,
        tf2::durationToSec(transform_tolerance_));
    latest_local_plan_receive_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = { DO_NOTHING, { -1.0, -1.0, -1.0 } };

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  local_plan_sub_.reset();
  local_plan_footprint_sub_.reset();

  {
    std::lock_guard<std::mutex> lk(local_plan_mutex_);
    latest_local_plan_.poses.clear();
    latest_local_plan_receive_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  local_plan_collision_stop_latched_ = false;

  polygons_.clear();
  sources_.clear();
  ignore_manager_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  process({ msg->linear.x, msg->linear.y, msg->angular.z });
}

void CollisionMonitor::localPlanCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!use_local_plan_collision_check_)
  {
    return;
  }
  std::lock_guard<std::mutex> lk(local_plan_mutex_);
  latest_local_plan_ = *msg;
  latest_local_plan_receive_time_ = get_clock()->now();
}

void CollisionMonitor::publishVelocity(const Action& robot_action)
{
  if (robot_action.req_vel.isZero())
  {
    if (local_plan_collision_stop_latched_)
    {
      // Keep publishing zero while local-plan collision stop is latched.
      stop_stamp_ = this->now();
    }
    else
    {
      if (!robot_action_prev_.req_vel.isZero())
      {
        // Robot just stopped: saving stop timestamp and continue
        stop_stamp_ = this->now();
      }
      else if (this->now() - stop_stamp_ > stop_pub_timeout_)
      {
        // More than stop_pub_timeout_ passed after robot has been stopped.
        // Cease publishing output cmd_vel.
        stop_stamp_ = this->now();
        return;
      }
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "[CollisionMonitor] publishVelocity: publishing cmd_vel: %f, %f", cmd_vel_out_msg->linear.x,
                       cmd_vel_out_msg->angular.z);
  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(std::string& cmd_vel_in_topic, std::string& cmd_vel_out_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance = tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout = rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());

  nav2_util::declare_parameter_if_not_declared(node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ = rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  nav2_util::declare_parameter_if_not_declared(node, "recover_stop_timeout", rclcpp::ParameterValue(10.0));
  recover_stop_timeout_ = rclcpp::Duration::from_seconds(get_parameter("recover_stop_timeout").as_double());

  base_frame_id_ = base_frame_id;
  global_frame_id_ = "map";
  transform_tolerance_ = transform_tolerance;

  nav2_util::declare_parameter_if_not_declared(node, "use_velocity_approach_prediction", rclcpp::ParameterValue(true));
  use_velocity_approach_prediction_ = get_parameter("use_velocity_approach_prediction").as_bool();

  nav2_util::declare_parameter_if_not_declared(node, "use_local_plan_collision_check", rclcpp::ParameterValue(false));
  use_local_plan_collision_check_ = get_parameter("use_local_plan_collision_check").as_bool();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_topic",
                                               rclcpp::ParameterValue(std::string("local_plan")));
  local_plan_topic_ = get_parameter("local_plan_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_footprint_topic",
                                               rclcpp::ParameterValue("/local_costmap/published_footprint"));
  local_plan_footprint_topic_ = get_parameter("local_plan_footprint_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_collision_max_points", rclcpp::ParameterValue(0));
  local_plan_collision_max_points_ = get_parameter("local_plan_collision_max_points").as_int();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_lookahead_distance", rclcpp::ParameterValue(5.0));
  local_plan_lookahead_distance_ = get_parameter("local_plan_lookahead_distance").as_double();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_sample_spacing_ratio", rclcpp::ParameterValue(0.5));
  local_plan_sample_spacing_ratio_ = get_parameter("local_plan_sample_spacing_ratio").as_double();

  nav2_util::declare_parameter_if_not_declared(node, "local_plan_validity_timeout", rclcpp::ParameterValue(0.0));
  local_plan_validity_timeout_ =
      rclcpp::Duration::from_seconds(get_parameter("local_plan_validity_timeout").as_double());

  if (!configurePolygons(base_frame_id, transform_tolerance))
  {
    return false;
  }

  if (!configureSources(base_frame_id, odom_frame_id, transform_tolerance, source_timeout))
  {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(const std::string& base_frame_id, const tf2::Duration& transform_tolerance)
{
  try
  {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names)
    {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon")
      {
        polygons_.push_back(
            std::make_shared<Polygon>(node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      }
      else if (polygon_type == "circle")
      {
        polygons_.push_back(
            std::make_shared<Circle>(node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      }
      else
      {  // Error if something else
        RCLCPP_ERROR(get_logger(), "[%s]: Unknown polygon type: %s", polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure())
      {
        return false;
      }
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(const std::string& base_frame_id, const std::string& odom_frame_id,
                                        const tf2::Duration& transform_tolerance,
                                        const rclcpp::Duration& source_timeout)
{
  try
  {
    auto node = shared_from_this();

    // Create and configure ignore polygon manager shared across all sources
    if (!ignore_manager_)
    {
      ignore_manager_ = std::make_shared<nav2_ignore_polygon_manager::IgnorePolygonManager>(node);
    }

    ignore_manager_->configure();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names)
    {
      nav2_util::declare_parameter_if_not_declared(node, source_name + ".type",
                                                   rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan")
      {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
                                                         transform_tolerance, source_timeout);

        s->configure();
        s->setIgnoreManager(ignore_manager_);

        sources_.push_back(s);
      }
      else if (source_type == "pointcloud")
      {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
            node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance, source_timeout);

        p->configure();
        p->setIgnoreManager(ignore_manager_);

        sources_.push_back(p);
      }
      else if (source_type == "range")
      {
        std::shared_ptr<Range> r = std::make_shared<Range>(node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
                                                           transform_tolerance, source_timeout);

        r->configure();
        r->setIgnoreManager(ignore_manager_);

        sources_.push_back(r);
      }
      else
      {  // Error if something else
        RCLCPP_ERROR(get_logger(), "[%s]: Unknown source type: %s", source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity& cmd_vel_in)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->get_clock()->now();

  // Do nothing if main worker in non-active state
  if (!process_active_)
  {
    return;
  }

  nav_msgs::msg::Path path_this_cycle;
  rclcpp::Time local_plan_receive_time(0, 0, get_clock()->get_clock_type());
  if (use_local_plan_collision_check_)
  {
    std::lock_guard<std::mutex> lk(local_plan_mutex_);
    path_this_cycle = latest_local_plan_;
    local_plan_receive_time = latest_local_plan_receive_time_;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Update ignore polygon manager with current robot position in global frame
  if (ignore_manager_ && ignore_manager_->getIgnoreWidth() != 0)
  {
    try
    {
      geometry_msgs::msg::TransformStamped tf_base_to_global =
          tf_buffer_->lookupTransform("map", base_frame_id_, curr_time,
                                      rclcpp::Duration::from_seconds(tf2::durationToSec(transform_tolerance_)));
      ignore_manager_->update(tf_base_to_global.transform.translation.x, tf_base_to_global.transform.translation.y, 0.0);
    }
    catch (const tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to get base->global transform for ignore manager: %s; "
                           "skip ignore update and continue collision check.",
                           e.what());
    }
  }

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_)
  {
    source->getData(curr_time, collision_points);
  }
  
  // Debug: publish collision points as PointCloud2 when debug_mode is enabled
  if (ignore_manager_ && ignore_manager_->getDebugMode())
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(collision_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
    for (const auto& p : collision_points)
    {
      *iter_x = static_cast<float>(p.x);
      *iter_y = static_cast<float>(p.y);
      *iter_z = 0.0f;
      ++iter_x; ++iter_y; ++iter_z;
    }

    cloud->header.stamp = curr_time;
    cloud->header.frame_id = base_frame_id_;
    ignore_manager_->publishCollisionPoints(std::move(cloud));
  }

  // By default - there is no action
  Action robot_action{ DO_NOTHING, cmd_vel_in };
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;
  bool has_stop_or_slowdown_polygon = false;

  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    if (robot_action.action_type == STOP)
    {
      // If robot already should stop, do nothing
      break;
    }

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN)
    {
      has_stop_or_slowdown_polygon = true;
    }

    if (at == STOP || at == SLOWDOWN)
    {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action))
      {
        action_polygon = polygon;
      }
    }
    else if (at == APPROACH)
    {
      if (use_velocity_approach_prediction_)
      {
        // Process APPROACH for the selected polygon
        if (processApproach(polygon, collision_points, cmd_vel_in, robot_action))
        {
          action_polygon = polygon;
        }
      }
    }
  }

  if (use_local_plan_collision_check_)
  {
    if (!path_this_cycle.poses.empty())
    {
      if (!collision_points.empty())
      {
        const std::optional<bool> lp_res = processLocalPlanCollision(
            path_this_cycle, collision_points, curr_time, local_plan_receive_time, robot_action);
        if (lp_res.has_value())
        {
          if (lp_res.value())
          {
            local_plan_collision_stop_latched_ = true;
          }
          else
          {
            local_plan_collision_stop_latched_ = false;
          }
        }
        else if (local_plan_collision_stop_latched_)
        {
          local_plan_collision_stop_latched_ = false;
        }
      }
      else
      {
        if (local_plan_collision_stop_latched_)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Local plan collision latch released: no collision points available.");
        }
        local_plan_collision_stop_latched_ = false;
      }
    }
    else
    {
      if (local_plan_collision_stop_latched_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Local plan collision latch released: local plan is empty.");
      }
      local_plan_collision_stop_latched_ = false;
    }

    if (local_plan_collision_stop_latched_)
    {
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Execute stop cmd due to local plan collision!");
    }
  }

  // Defensive guard for local-plan-only mode:
  // when velocity-based APPROACH and STOP/SLOWDOWN polygons are disabled,
  // only local_plan_collision_stop_latched_ is allowed to enforce a full stop.
  if (use_local_plan_collision_check_ && !use_velocity_approach_prediction_ && !has_stop_or_slowdown_polygon &&
      !local_plan_collision_stop_latched_)
  {
    if (robot_action.req_vel.isZero() && !cmd_vel_in.isZero())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unexpected zero cmd in local-plan-only mode (latch=false), passing through input cmd_vel.");
    }
    robot_action.action_type = DO_NOTHING;
    robot_action.req_vel = cmd_vel_in;
  }

  if (robot_action.action_type != robot_action_prev_.action_type)
  {
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
  }

  // Publish requred robot velocity
  publishVelocity(robot_action);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

std::optional<bool> CollisionMonitor::processLocalPlanCollision(
    const nav_msgs::msg::Path& path, const std::vector<Point>& collision_points, const rclcpp::Time& curr_time,
    const rclcpp::Time& plan_receive_time, Action& robot_action)
{
  if (path.poses.empty() || path.header.frame_id.empty())
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Local plan collision check skipped: path is empty or frame id is empty.");
    return std::nullopt;
  }

  if (local_plan_validity_timeout_.nanoseconds() > 0)
  {
    const bool stamp_ok = path.header.stamp.sec != 0 || path.header.stamp.nanosec != 0;
    const rclcpp::Time ref_time =
        stamp_ok ? rclcpp::Time(path.header.stamp, get_clock()->get_clock_type()) : plan_receive_time;
    const rclcpp::Duration age = curr_time - ref_time;
    if (age > local_plan_validity_timeout_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Local plan expired (age %.3f s > timeout %.3f s); skipping local plan collision check.",
                           age.seconds(), local_plan_validity_timeout_.seconds());
      return std::nullopt;
    }
  }

  if (collision_points.empty())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Local plan collision check skipped: no collision points available.");
    return std::nullopt;
  }

  std::vector<Point> points_path;
  if (path.header.frame_id != base_frame_id_)
  {
    tf2::Transform t_base_to_path;
    try
    {
      const geometry_msgs::msg::TransformStamped tf_base_to_path =
          tf_buffer_->lookupTransform(path.header.frame_id, base_frame_id_, curr_time,
                                      rclcpp::Duration::from_seconds(tf2::durationToSec(transform_tolerance_)));
      tf2::fromMsg(tf_base_to_path.transform, t_base_to_path);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Local plan collision check skipped: %s", ex.what());
      return std::nullopt;
    }
    points_path.reserve(collision_points.size());
    for (const Point& p : collision_points)
    {
      const tf2::Vector3 v_in(p.x, p.y, 0.0);
      const tf2::Vector3 v_out = t_base_to_path * v_in;
      points_path.push_back({ v_out.x(), v_out.y() });
    }
  }
  else
  {
    points_path = collision_points;
  }

  std::vector<Point> footprint_poly;
  if (!updateLocalPlanFootprint(footprint_poly))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Local plan collision check skipped: no footprint from topic '%s'.",
                         local_plan_footprint_topic_.c_str());
    return std::nullopt;
  }

  const double fp_extent = footprint_bbox_max_extent(footprint_poly);
  static constexpr double k_fallback_footprint_extent = 0.5;
  const double char_len = (fp_extent > 1.0e-6) ? fp_extent : k_fallback_footprint_extent;
  const double spacing = std::max(1.0e-3, local_plan_sample_spacing_ratio_ * char_len);
  double arc_length = 0.0;
  double last_sample_arc = -spacing;

  // 原地转 local_plan：xy 弧长≈0，需对每个 yaw 采样点做 footprint 检查
  double total_xy_arc = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const geometry_msgs::msg::Point & prev = path.poses[i - 1].pose.position;
    const geometry_msgs::msg::Point & curr = path.poses[i].pose.position;
    total_xy_arc += std::hypot(curr.x - prev.x, curr.y - prev.y);
  }
  static constexpr double kRotationDominantXYArcThreshold = 0.05;
  const bool rotation_dominant_path =
    path.poses.size() > 1 && total_xy_arc < kRotationDominantXYArcThreshold;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::msg::Pose & pose = path.poses[i].pose;
    const double wx = pose.position.x;
    const double wy = pose.position.y;
    const double yaw = tf2::getYaw(pose.orientation);

    if (i > 0)
    {
      const geometry_msgs::msg::Point& prev = path.poses[i - 1].pose.position;
      arc_length += std::hypot(pose.position.x - prev.x, pose.position.y - prev.y);
    }

    if (arc_length > local_plan_lookahead_distance_)
    {
      break;
    }

    // 强制检查路径首尾点，避免采样间距跳过端点
    const bool is_endpoint = (i == 0) || (i + 1 == path.poses.size());
    const bool should_check =
      rotation_dominant_path ||
      is_endpoint || (arc_length - last_sample_arc >= spacing);
    if (!should_check) {
      continue;
    }
    last_sample_arc = arc_length;

    std::vector<Point> points_robot_frame;
    points_robot_frame.reserve(points_path.size());
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    for (const Point& pf : points_path)
    {
      const double dx = pf.x - wx;
      const double dy = pf.y - wy;
      points_robot_frame.push_back({ dx * c - dy * s, dx * s + dy * c });
    }

    const int inside_cnt = countPointsInsidePolygon(points_robot_frame, footprint_poly);
    const int max_pts = local_plan_collision_max_points_;
    if (inside_cnt > max_pts)
    {
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "STOP due to local plan footprint collision ahead (footprint: %s): "
                           "inside=%d max_allowed=%d path_pose_index=%zu arc_length=%.3f m",
                           local_plan_footprint_topic_.c_str(), inside_cnt, max_pts, i, arc_length);
      return true;
    }
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Local plan collision check passed!");
  return false;
}

bool CollisionMonitor::updateLocalPlanFootprint(std::vector<Point>& footprint_poly)
{
  if (!local_plan_footprint_sub_)
  {
    return false;
  }

  std::vector<geometry_msgs::msg::Point> footprint_vec;
  std_msgs::msg::Header footprint_header;
  if (!local_plan_footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header))
  {
    return false;
  }

  footprint_poly.resize(footprint_vec.size());
  for (size_t i = 0; i < footprint_vec.size(); ++i)
  {
    footprint_poly[i] = { footprint_vec[i].x, footprint_vec[i].y };
  }
  return footprint_poly.size() >= 3;
}

bool CollisionMonitor::processStopSlowdown(const std::shared_ptr<Polygon> polygon,
                                           const std::vector<Point>& collision_points, const Velocity& velocity,
                                           Action& robot_action) const
{
  using namespace std::chrono_literals;
  auto now = rclcpp::Clock();
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints())
  {
    if (polygon->getActionType() == STOP &&
        ((this->now() - stop_stamp_) < stop_pub_timeout_ || (this->now() - stop_stamp_) > recover_stop_timeout_))
    {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      RCLCPP_INFO_STREAM_THROTTLE(get_logger(), now, 1000, "Execute stop cmd due to collison monitor!");
      return true;
    }
    else
    {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel)
      {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), now, 5000,
                                    "Execute SLOWDOWN cmd at safe vel due to collison monitor!");
        return true;
      }
      else
      {
        robot_action.action_type = SLOWDOWN;
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), now, 5000,
                                    "Execute SLOWDOWN cmd at original vel due to collison monitor!");
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(const std::shared_ptr<Polygon> polygon,
                                       const std::vector<Point>& collision_points, const Velocity& velocity,
                                       Action& robot_action) const
{
  polygon->updatePolygon();

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0)
  {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel)
    {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::printAction(const Action& robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP)
  {
    if (action_polygon)
    {
      RCLCPP_INFO(get_logger(), "Robot to stop due to %s polygon", action_polygon->getName().c_str());
    }
    else if (local_plan_collision_stop_latched_)
    {
      RCLCPP_INFO(get_logger(), "Robot to stop due to local plan footprint collision");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Robot to stop");
    }
  }
  else if (robot_action.action_type == SLOWDOWN)
  {
    if (action_polygon)
    {
      RCLCPP_INFO(get_logger(), "Robot to slowdown for %f percents due to %s polygon",
                  action_polygon->getSlowdownRatio() * 100, action_polygon->getName().c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Robot to slowdown");
    }
  }
  else if (robot_action.action_type == APPROACH)
  {
    if (action_polygon)
    {
      RCLCPP_INFO(get_logger(), "Robot to approach for %f seconds away from collision",
                  action_polygon->getTimeBeforeCollision());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Robot to approach");
    }
  }
  else
  {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(get_logger(), "Robot to continue normal operation");
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->publish();
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
