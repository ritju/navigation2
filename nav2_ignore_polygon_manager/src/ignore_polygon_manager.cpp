#include "nav2_ignore_polygon_manager/ignore_polygon_manager.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"

namespace nav2_ignore_polygon_manager
{

IgnorePolygonManager::IgnorePolygonManager(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node,
                                           const std::string& param_prefix)
  : node_(node), logger_(rclcpp::get_logger("ignore_polygon_manager")), param_prefix_(param_prefix)
{
}

IgnorePolygonManager::~IgnorePolygonManager()
{
  lane_center_paths_sub_.reset();
  collision_points_pub_.reset();
}

void IgnorePolygonManager::configure()
{
  auto node_ptr = node_.lock();
  if (!node_ptr)
  {
    throw std::runtime_error{ "Failed to lock node in IgnorePolygonManager::configure()" };
  }

  const std::string prefix = param_prefix_.empty() ? "" : param_prefix_ + ".";

  nav2_util::declare_parameter_if_not_declared(node_ptr, prefix + "ignore_width", rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(node_ptr, prefix + "ignore_range", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(node_ptr, prefix + "debug_mode", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node_ptr, prefix + "ignore_height_above", rclcpp::ParameterValue(0.0));

  ignore_width_.store(node_ptr->get_parameter(prefix + "ignore_width").as_int());
  ignore_range_.store(node_ptr->get_parameter(prefix + "ignore_range").as_double());
  debug_mode_.store(node_ptr->get_parameter(prefix + "debug_mode").as_bool());
  ignore_height_above_.store(node_ptr->get_parameter(prefix + "ignore_height_above").as_double());

  RCLCPP_INFO(logger_, "ignore_width=%d, ignore_range=%.2f, debug_mode=%d, ignore_height_above=%.2f",
              ignore_width_.load(), ignore_range_.load(), debug_mode_.load(), ignore_height_above_.load());

  rclcpp::QoS lane_qos(10);
  lane_qos.transient_local();
  lane_qos.reliable();

  lane_center_paths_sub_ = node_ptr->create_subscription<capella_ros_msg::msg::LaneCenterPaths>(
      "/edge_reference_paths_no_collision_check", lane_qos,
      std::bind(&IgnorePolygonManager::laneCenterPathsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Subscribed to edge_reference_paths_no_collision_check");

  if (debug_mode_.load())
  {
    std::string topic = node_ptr->get_name();
    if (!param_prefix_.empty())
    {
      topic += "/" + param_prefix_;
    }
    topic += "/collision_points";

    collision_points_pub_ = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS());
    collision_points_pub_->on_activate();
    RCLCPP_INFO(logger_, "Debug mode enabled: publishing collision points to '%s' topic", topic.c_str());
  }
}

void IgnorePolygonManager::laneCenterPathsCallback(capella_ros_msg::msg::LaneCenterPaths::ConstSharedPtr msg)
{
  std::vector<nav_msgs::msg::Path> paths;
  paths.reserve(msg->paths.size());

  for (const auto& path : msg->paths)
  {
    if (path.poses.size() >= 2)
    {
      paths.push_back(path);
    }
  }

  {
    std::lock_guard<std::mutex> lock(ignore_paths_mutex_);
    ignore_paths_ = std::move(paths);
  }

  RCLCPP_DEBUG(logger_, "Received %zu paths, stored %zu valid paths.", msg->paths.size(), paths.size());
}

void IgnorePolygonManager::update(const double& robot_x, const double& robot_y, const double& robot_z)
{
  robot_z_.store(robot_z);

  const int width = ignore_width_.load();
  if (width == 0)
  {
    std::lock_guard<std::mutex> lock(active_ignore_rects_mutex_);
    active_ignore_rects_.clear();
    return;
  }

  const double offset = std::abs(static_cast<double>(width)) / 100.0;  // convert cm to meters
  const double range = ignore_range_.load();
  const double range_sq = range * range;

  std::vector<nav_msgs::msg::Path> paths_snapshot;
  {
    std::lock_guard<std::mutex> lock(ignore_paths_mutex_);
    paths_snapshot = ignore_paths_;
  }

  std::vector<geometry_msgs::msg::PolygonStamped> new_rects;

  for (const auto& path : paths_snapshot)
  {
    if (path.poses.size() < 2)
    {
      continue;
    }

    // Step 1: Find segments whose minimum distance to robot is within range
    std::vector<size_t> nearby_segments;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i)
    {
      const double ax = path.poses[i].pose.position.x;
      const double ay = path.poses[i].pose.position.y;
      const double bx = path.poses[i + 1].pose.position.x;
      const double by = path.poses[i + 1].pose.position.y;

      const double seg_dx = bx - ax;
      const double seg_dy = by - ay;
      const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

      double min_dist_sq;
      if (seg_len_sq < 1e-18)
      {
        const double dx = ax - robot_x;
        const double dy = ay - robot_y;
        min_dist_sq = dx * dx + dy * dy;
      }
      else
      {
        const double to_ax = robot_x - ax;
        const double to_ay = robot_y - ay;
        double t = (to_ax * seg_dx + to_ay * seg_dy) / seg_len_sq;
        t = std::max(0.0, std::min(1.0, t));

        const double closest_x = ax + t * seg_dx;
        const double closest_y = ay + t * seg_dy;
        const double dx = closest_x - robot_x;
        const double dy = closest_y - robot_y;
        min_dist_sq = dx * dx + dy * dy;
      }

      if (min_dist_sq <= range_sq)
      {
        nearby_segments.push_back(i);
      }
    }

    if (nearby_segments.empty())
    {
      continue;
    }

    // Step 2: For each nearby segment, generate a fine-grained rectangle on robot's side
    for (const auto i : nearby_segments)
    {
      const size_t j = i + 1;
      const auto& p0 = path.poses[i].pose.position;
      const auto& p1 = path.poses[j].pose.position;

      double dx0, dy0;
      if (i == 0)
      {
        dx0 = path.poses[1].pose.position.x - p0.x;
        dy0 = path.poses[1].pose.position.y - p0.y;
      }
      else
      {
        dx0 = p0.x - path.poses[i - 1].pose.position.x;
        dy0 = p0.y - path.poses[i - 1].pose.position.y;
      }

      double dx1, dy1;
      if (j == path.poses.size() - 1)
      {
        dx1 = p1.x - path.poses[j - 1].pose.position.x;
        dy1 = p1.y - path.poses[j - 1].pose.position.y;
      }
      else
      {
        dx1 = path.poses[j + 1].pose.position.x - p1.x;
        dy1 = path.poses[j + 1].pose.position.y - p1.y;
      }

      const double len0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
      const double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      if (len0 < 1e-9 || len1 < 1e-9)
      {
        continue;
      }

      const double nx0 = -dy0 / len0;
      const double ny0 = dx0 / len0;
      const double nx1 = -dy1 / len1;
      const double ny1 = dx1 / len1;

      // Determine robot side via cross product at segment midpoint
      const double mid_x = (p0.x + p1.x) * 0.5;
      const double mid_y = (p0.y + p1.y) * 0.5;
      const double dir_x = p1.x - p0.x;
      const double dir_y = p1.y - p0.y;
      const double to_robot_x = robot_x - mid_x;
      const double to_robot_y = robot_y - mid_y;
      const double cross = dir_x * to_robot_y - dir_y * to_robot_x;

      // If cross >= 0, robot is on the left → ignore the left side
      // If cross < 0, robot is on the right → ignore the right side
      const double sign = (cross >= 0) ? 1.0 : -1.0;

      const double ox0 = p0.x + sign * nx0 * offset;
      const double oy0 = p0.y + sign * ny0 * offset;
      const double ox1 = p1.x + sign * nx1 * offset;
      const double oy1 = p1.y + sign * ny1 * offset;

      double min_x = std::min({ p0.x, p1.x, ox0, ox1 });
      double max_x = std::max({ p0.x, p1.x, ox0, ox1 });
      double min_y = std::min({ p0.y, p1.y, oy0, oy1 });
      double max_y = std::max({ p0.y, p1.y, oy0, oy1 });

      geometry_msgs::msg::PolygonStamped rect;
      rect.header = path.header;

      geometry_msgs::msg::Point32 c0, c1, c2, c3;
      c0.x = static_cast<float>(min_x);
      c0.y = static_cast<float>(min_y);
      c0.z = 0.0f;
      c1.x = static_cast<float>(max_x);
      c1.y = static_cast<float>(min_y);
      c1.z = 0.0f;
      c2.x = static_cast<float>(max_x);
      c2.y = static_cast<float>(max_y);
      c2.z = 0.0f;
      c3.x = static_cast<float>(min_x);
      c3.y = static_cast<float>(max_y);
      c3.z = 0.0f;

      rect.polygon.points = { c0, c1, c2, c3 };
      new_rects.push_back(rect);
    }
  }

  {
    std::lock_guard<std::mutex> lock(active_ignore_rects_mutex_);
    active_ignore_rects_ = std::move(new_rects);
  }
}

bool IgnorePolygonManager::isPointIgnored(const double& px, const double& py, const double& pz) const
{
  const double height_threshold = robot_z_.load() + ignore_height_above_.load();
  if (pz <= height_threshold)
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(active_ignore_rects_mutex_);
  for (const auto& rect : active_ignore_rects_)
  {
    if (isPointInAABB(px, py, rect.polygon.points))
    {
      return true;
    }
  }
  return false;
}

bool IgnorePolygonManager::isPointInAABB(const double& px, const double& py,
                                         const std::vector<geometry_msgs::msg::Point32>& pts) const
{
  if (pts.size() < 4)
  {
    return false;
  }
  // pts[0] = (min_x, min_y), pts[2] = (max_x, max_y)
  return px >= pts[0].x && px <= pts[2].x && py >= pts[0].y && py <= pts[2].y;
}

void IgnorePolygonManager::setIgnoreWidth(int width)
{
  ignore_width_.store(width);
}

int IgnorePolygonManager::getIgnoreWidth() const
{
  return ignore_width_;
}

void IgnorePolygonManager::setIgnoreRange(double range)
{
  ignore_range_.store(range);
}

double IgnorePolygonManager::getIgnoreRange() const
{
  return ignore_range_.load();
}

bool IgnorePolygonManager::getDebugMode() const
{
  return debug_mode_.load() && ignore_width_.load()!=0;
}

void IgnorePolygonManager::publishCollisionPoints(std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud)
{
  if (!collision_points_pub_)
  {
    return;
  }
  collision_points_pub_->publish(std::move(cloud));
}

}  // namespace nav2_ignore_polygon_manager
