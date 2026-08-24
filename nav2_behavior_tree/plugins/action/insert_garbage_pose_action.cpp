#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_behavior_tree/plugins/action/insert_garbage_pose_action.hpp"

namespace nav2_behavior_tree
{

// 构造：创建垃圾、禁扫区、footprint 话题订阅
InsertGarbagePose::InsertGarbagePose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  garbage_topic_("/garbage_cord1"),
  special_terrain_topic_("/cleaning_tool_retraction_areas"),
  footprint_topic_("local_costmap/published_footprint"),
  local_costmap_topic_("local_costmap/costmap"),
  visualization_topic_("insert_garbage_pose/markers"),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  arrived_radius_(0.5),       // footprint 进入垃圾附近半径，停止再插入
  clip_extend_m_(1.0),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(30.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0),  // 离队头超过该距离就不删点
  max_garbage_robot_dist_m_(5.0),  // 垃圾离机器人超过该距离则忽略
  min_garbage_obstacle_clearance_m_(0.7),  // 垃圾周围该半径内有障碍则丢弃
  garbage_merge_radius_m_(1.0),    // 到种子小于该距离合为一堆
  garbage_extend_m_(2.0),          // 沿扫向相对垃圾再插一点，默认 2.0m；见 GARBAGE_EXTEND_M
  work_circle_radius_m_(10.0)
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("local_costmap_topic", local_costmap_topic_);
  getInput("visualization_topic", visualization_topic_);
  getInput("arrived_radius", arrived_radius_);
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("min_garbage_obstacle_clearance_m", min_garbage_obstacle_clearance_m_);
  getInput("garbage_merge_radius_m", garbage_merge_radius_m_);
  getInput("work_circle_radius_m", work_circle_radius_m_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  // 环境变量 GARBAGE_EXTEND_M：沿 path_yaw 相对垃圾再插一点
  if (const char * extend_env = std::getenv("GARBAGE_EXTEND_M")) {
    char * end = nullptr;
    const double parsed = std::strtod(extend_env, &end);
    if (end != extend_env && std::isfinite(parsed)) {
      garbage_extend_m_ = parsed;
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: invalid GARBAGE_EXTEND_M='%s', keep %.2f m",
        extend_env, garbage_extend_m_);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: garbage_extend_m: %.2f m",
    garbage_extend_m_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  garbage_sub_ = node_->create_subscription<capella_ros_msg::msg::GarbageDetect>(
    garbage_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&InsertGarbagePose::garbageDetectCallback, this, std::placeholders::_1),
    sub_option);

  // TRANSIENT_LOCAL
  rclcpp::QoS special_terrain_qos(rclcpp::KeepLast(1));
  special_terrain_qos.transient_local().reliable();
  special_terrain_sub_ = node_->create_subscription<garage_utils_msgs::msg::Polygons>(
    special_terrain_topic_,
    special_terrain_qos,
    std::bind(&InsertGarbagePose::special_terrain_callback, this, std::placeholders::_1),
    sub_option);

  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&InsertGarbagePose::footprintCallback, this, std::placeholders::_1),
    sub_option);

  // 局部代价图：TRANSIENT_LOCAL
  rclcpp::QoS local_costmap_qos(rclcpp::KeepLast(1));
  local_costmap_qos.transient_local().reliable();
  local_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_,
    local_costmap_qos,
    std::bind(&InsertGarbagePose::localCostmapCallback, this, std::placeholders::_1),
    sub_option);

  // RViz Marker，默认 VOLATILE，和 RViz 订阅对齐
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_, 10);
}

// 垃圾检测话题回调：转到 map；合堆半径内已有则不进 history
void InsertGarbagePose::garbageDetectCallback(
  const capella_ros_msg::msg::GarbageDetect::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  capella_ros_msg::msg::GarbageDetect item = *msg;
  const std::string & detect_frame = item.pose.header.frame_id;
  if (detect_frame.empty() || detect_frame == robot_base_frame_) {
    const double bx = item.pose.pose.position.x;
    const double by = item.pose.pose.position.y;
    const double origin_r2 =
      kInvalidGarbageOriginRadiusM * kInvalidGarbageOriginRadiusM;
    if (bx * bx + by * by < origin_r2) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "InsertGarbagePose: drop garbage at robot origin in '%s' (%.3f, %.3f)",
        detect_frame.empty() ? robot_base_frame_.c_str() : detect_frame.c_str(),
        bx, by);
      return;
    }
  }

  if (!transformGarbageToMap(item)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: drop garbage, transform to map failed, wait for next frame");
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: drop garbage, getCurrentPose failed, wait for next frame");
    return;
  }
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  const double gx = item.pose.pose.position.x;
  const double gy = item.pose.pose.position.y;
  const double merge_r = std::max(0.0, garbage_merge_radius_m_);
  const double merge_r2 = merge_r * merge_r;

  auto near_xy = [&](double x, double y) {
    return squaredDistanceXY(gx, gy, x, y) < merge_r2;
  };

  for (const auto & reached : reached_garbage_xy_) {
    if (near_xy(reached.first, reached.second)) {
      return;
    }
  }
  for (const auto & kept : garbage_list_) {
    if (near_xy(kept.pose.pose.position.x, kept.pose.pose.position.y)) {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(history_mutex_);
  for (const auto & existing : history_list_) {
    if (near_xy(existing.pose.pose.position.x, existing.pose.pose.position.y)) {
      return;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 新发现一堆垃圾, 坐标=(%.2f, %.2f)", gx, gy);

  history_list_.push_back(std::move(item));
  while (history_list_.size() > kMaxHistorySize) {
    auto farthest_it = history_list_.begin();
    double farthest_d2 = -1.0;
    for (auto it = history_list_.begin(); it != history_list_.end(); ++it) {
      const double d2 = squaredDistanceXY(
        it->pose.pose.position.x, it->pose.pose.position.y, robot_x, robot_y);
      if (d2 > farthest_d2) {
        farthest_d2 = d2;
        farthest_it = it;
      }
    }
    history_list_.erase(farthest_it);
  }
}

// 特殊清扫/禁扫区域话题回调
void InsertGarbagePose::special_terrain_callback(
  const garage_utils_msgs::msg::Polygons::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(special_terrain_mutex_);
  special_terrain_polygons_ = msg->polygons;
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: received %zu special terrain polygon(s)",
    special_terrain_polygons_.size());
}

// footprint 话题回调
void InsertGarbagePose::footprintCallback(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  latest_footprint_ = msg;
}

// 局部代价图话题回调
void InsertGarbagePose::localCostmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(local_costmap_mutex_);
  latest_local_costmap_ = msg;
}

// 有没有图    点在图内吗    格子可读吗，是否是栅格值 < 0
bool InsertGarbagePose::isObstacleInfoReadable(
  double x, double y, std::string * reason) const
{
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    costmap = latest_local_costmap_;
  }

  if (!costmap || costmap->info.width == 0 || costmap->info.height == 0 ||
    costmap->data.empty())
  {
    if (reason) {
      *reason = "no local costmap received";
    }
    return false;
  }

  double px = x;
  double py = y;
  const std::string & costmap_frame = costmap->header.frame_id;
  if (!costmap_frame.empty() && costmap_frame != global_frame_) {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header.frame_id = global_frame_;
    in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    in.point.x = x;
    in.point.y = y;
    in.point.z = 0.0;
    try {
      tf_->transform(in, out, costmap_frame, tf2::durationFromSec(transform_tolerance_));
      px = out.point.x;
      py = out.point.y;
    } catch (const tf2::TransformException & ex) {
      if (reason) {
        *reason = std::string("tf to costmap frame failed: ") + ex.what();
      }
      return false;
    }
  }

  const auto & info = costmap->info;
  if (info.resolution <= 0.0) {
    if (reason) {
      *reason = "invalid costmap resolution";
    }
    return false;
  }

  const double dx = px - info.origin.position.x;
  const double dy = py - info.origin.position.y;
  const double yaw = tf2::getYaw(info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  // 转到代价图栅格坐标系  含 origin 朝向
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  const int mx = static_cast<int>(std::floor(local_x / info.resolution));
  const int my = static_cast<int>(std::floor(local_y / info.resolution));
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
  {
    if (reason) {
      *reason = "outside local costmap";
    }
    return false;
  }

  const std::size_t idx =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
    static_cast<std::size_t>(mx);
  if (idx >= costmap->data.size()) {
    if (reason) {
      *reason = "costmap index out of range";
    }
    return false;
  }

  // OccupancyGrid: -1 = unknown，此时障碍物情况不可读
  if (costmap->data[idx] < 0) {
    if (reason) {
      *reason = "costmap cell unknown";
    }
    return false;
  }

  return true;
}

bool InsertGarbagePose::isMapPointPassableOnLocalCostmap(double x, double y) const
{
  // 取最新局部代价图
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    costmap = latest_local_costmap_;
  }

  if (!costmap || costmap->info.width == 0 || costmap->info.height == 0 ||
    costmap->data.empty())
  {
    return false;
  }

  // map 点转到代价图坐标系
  double px = x;
  double py = y;
  const std::string & costmap_frame = costmap->header.frame_id;
  if (!costmap_frame.empty() && costmap_frame != global_frame_) {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header.frame_id = global_frame_;
    in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    in.point.x = x;
    in.point.y = y;
    in.point.z = 0.0;
    try {
      tf_->transform(in, out, costmap_frame, tf2::durationFromSec(transform_tolerance_));
      px = out.point.x;
      py = out.point.y;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  const auto & info = costmap->info;
  if (info.resolution <= 0.0) {
    return false;
  }

  // 相对 origin 的偏移，再按 origin 朝向转到栅格局部坐标
  const double dx = px - info.origin.position.x;
  const double dy = py - info.origin.position.y;
  const double yaw = tf2::getYaw(info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  // 栅格下标，越界不可过
  const int mx = static_cast<int>(std::floor(local_x / info.resolution));
  const int my = static_cast<int>(std::floor(local_y / info.resolution));
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
  {
    return false;
  }

  const std::size_t idx =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
    static_cast<std::size_t>(mx);
  if (idx >= costmap->data.size()) {
    return false;
  }

  // OccupancyGrid 发布后：-1 对应 255，>=100 对应 254；99 对应 253 可通过
  const int8_t cell = costmap->data[idx];
  if (cell < 0 || cell >= 100) {
    return false;
  }
  return true;
}

bool InsertGarbagePose::hasObstacleWithinRadius(
  double x, double y, double radius_m) const
{
  if (radius_m <= 0.0 || !tf_) {
    return false;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    costmap = latest_local_costmap_;
  }
  if (!costmap || costmap->info.width == 0 || costmap->info.height == 0 ||
    costmap->data.empty() || costmap->info.resolution <= 0.0)
  {
    return false;
  }

  const auto & info = costmap->info;
  const std::string & costmap_frame = costmap->header.frame_id;
  double px = x;
  double py = y;
  if (!costmap_frame.empty() && costmap_frame != global_frame_) {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header.frame_id = global_frame_;
    in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    in.point.x = x;
    in.point.y = y;
    in.point.z = 0.0;
    try {
      tf_->transform(in, out, costmap_frame, tf2::durationFromSec(transform_tolerance_));
      px = out.point.x;
      py = out.point.y;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  const double origin_yaw = tf2::getYaw(info.origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double dx = px - info.origin.position.x;
  const double dy = py - info.origin.position.y;
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const int gx = static_cast<int>(std::floor(local_x / info.resolution));
  const int gy = static_cast<int>(std::floor(local_y / info.resolution));
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
    return false;
  }

  const int r_cells = std::max(1, static_cast<int>(std::ceil(radius_m / info.resolution)));
  const double r2 = radius_m * radius_m;
  const int mx0 = std::max(0, gx - r_cells);
  const int mx1 = std::min(width - 1, gx + r_cells);
  const int my0 = std::max(0, gy - r_cells);
  const int my1 = std::min(height - 1, gy + r_cells);
  for (int my = my0; my <= my1; ++my) {
    for (int mx = mx0; mx <= mx1; ++mx) {
      const std::size_t idx =
        static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(mx);
      if (idx >= costmap->data.size()) {
        continue;
      }
      // OccupancyGrid：>=100 是障碍；膨胀层 253 等 <100 不当硬障碍
      if (costmap->data[idx] < 100) {
        continue;
      }
      const double clx = (static_cast<double>(mx) + 0.5) * info.resolution;
      const double cly = (static_cast<double>(my) + 0.5) * info.resolution;
      const double wx = info.origin.position.x + cos_yaw * clx - sin_yaw * cly;
      const double wy = info.origin.position.y + sin_yaw * clx + cos_yaw * cly;
      const double d2 = (wx - px) * (wx - px) + (wy - py) * (wy - py);
      if (d2 <= r2) {
        return true;
      }
    }
  }
  return false;
}

// 在局部代价图上找离查询点最近的占用格，写出该格中心的 map 坐标
bool InsertGarbagePose::findNearestObstaclePixel(
  double x, double y, double * ox, double * oy)
{
  if (!ox || !oy || !tf_) {
    return false;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    costmap = latest_local_costmap_;
  }
  if (!costmap || costmap->info.width == 0 || costmap->info.height == 0 ||
    costmap->data.empty() || costmap->info.resolution <= 0.0)
  {
    return false;
  }

  const auto & info = costmap->info;
  const std::string & costmap_frame = costmap->header.frame_id;
  // 查询点先变到代价图坐标系，后面按格子扫
  double px = x;
  double py = y;
  if (!costmap_frame.empty() && costmap_frame != global_frame_) {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header.frame_id = global_frame_;
    in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    in.point.x = x;
    in.point.y = y;
    in.point.z = 0.0;
    try {
      tf_->transform(in, out, costmap_frame, tf2::durationFromSec(transform_tolerance_));
      px = out.point.x;
      py = out.point.y;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // 世界点 → 栅格下标  含 origin 朝向
  const double origin_yaw = tf2::getYaw(info.origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double dx = px - info.origin.position.x;
  const double dy = py - info.origin.position.y;
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const int gx = static_cast<int>(std::floor(local_x / info.resolution));
  const int gy = static_cast<int>(std::floor(local_y / info.resolution));
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
    return false;
  }

  // 搜索上限 = E 延伸距离 + 0.5m：挡住 2m 的墙应在 2m 内，不必扫到 4m 外另一面墙
  const double search_radius_m = std::max(1.0, std::fabs(garbage_extend_m_) + 0.5);
  const int r_cells = std::max(1, static_cast<int>(std::ceil(search_radius_m / info.resolution)));
  const double search_r2 = search_radius_m * search_radius_m;
  bool found = false;
  double best_d2 = std::numeric_limits<double>::infinity();
  double best_cx = 0.0;
  double best_cy = 0.0;

  const int mx0 = std::max(0, gx - r_cells);
  const int mx1 = std::min(width - 1, gx + r_cells);
  const int my0 = std::max(0, gy - r_cells);
  const int my1 = std::min(height - 1, gy + r_cells);
  for (int my = my0; my <= my1; ++my) {
    for (int mx = mx0; mx <= mx1; ++mx) {
      const std::size_t idx =
        static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(mx);
      if (idx >= costmap->data.size()) {
        continue;
      }
      // OccupancyGrid：>=100 是障碍；<100 含自由和膨胀层 253，不当墙
      const int8_t cell = costmap->data[idx];
      if (cell < 100) {
        continue;
      }
      // 格子中心变回代价图世界坐标，再和查询点比距离
      const double clx = (static_cast<double>(mx) + 0.5) * info.resolution;
      const double cly = (static_cast<double>(my) + 0.5) * info.resolution;
      const double wx = info.origin.position.x + cos_yaw * clx - sin_yaw * cly;
      const double wy = info.origin.position.y + sin_yaw * clx + cos_yaw * cly;
      const double d2 = (wx - px) * (wx - px) + (wy - py) * (wy - py);
      if (d2 > search_r2 || d2 >= best_d2) {
        continue;
      }
      best_d2 = d2;
      best_cx = wx;
      best_cy = wy;
      found = true;
    }
  }
  if (!found) {
    return false;
  }

  viz_obstacle_cell_m_ = std::max(0.08, static_cast<double>(info.resolution));

  *ox = best_cx;
  *oy = best_cy;
  // 代价图不在 map 时，把最近格中心转回 map，给后面 G-P 垂线用
  if (!costmap_frame.empty() && costmap_frame != global_frame_) {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header.frame_id = costmap_frame;
    in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    in.point.x = best_cx;
    in.point.y = best_cy;
    in.point.z = 0.0;
    try {
      tf_->transform(in, out, global_frame_, tf2::durationFromSec(transform_tolerance_));
      *ox = out.point.x;
      *oy = out.point.y;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  return true;
}

bool InsertGarbagePose::isStraightCorridorClear(
  double start_x, double start_y,
  double end_x, double end_y,
  std::string * reason) const
{
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    if (!latest_local_costmap_ || latest_local_costmap_->data.empty()) {
      if (reason) {
        *reason = "no local costmap";
      }
      return false;
    }
  }

  std::vector<std::pair<double, double>> local_xy;
  if (!getRobotFootprintInBase(local_xy) || local_xy.empty()) {
    if (reason) {
      *reason = "no footprint";
    }
    return false;
  }

  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double len = std::sqrt(dx * dx + dy * dy);
  const double travel_yaw = (len < 1e-6) ? 0.0 : std::atan2(dy, dx);
  const double c_yaw = std::cos(travel_yaw);
  const double s_yaw = std::sin(travel_yaw);

  auto cornerWorld = [&](double cx, double cy, double lx, double ly) {
    return std::pair<double, double>{
      cx + lx * c_yaw - ly * s_yaw,
      cy + lx * s_yaw + ly * c_yaw};
  };

  auto cornersClearAt = [&](double cx, double cy, const char * fail_reason) -> bool {
    for (const auto & off : local_xy) {
      const auto q = cornerWorld(cx, cy, off.first, off.second);
      if (!isMapPointPassableOnLocalCostmap(q.first, q.second)) {
        if (reason) {
          *reason = fail_reason;
        }
        return false;
      }
    }
    return true;
  };

  if (len < 1e-6) {
    if (!isMapPointPassableOnLocalCostmap(start_x, start_y)) {
      if (reason) {
        *reason = "lethal at robot";
      }
      return false;
    }
    return cornersClearAt(start_x, start_y, "lethal at footprint corner");
  }

  const double ux = dx / len;
  const double uy = dy / len;
  const double nx = -uy;
  const double ny = ux;

  double half_w = 0.0;
  for (const auto & off : local_xy) {
    half_w = std::max(half_w, std::fabs(off.second));
  }
  if (half_w < 1e-3) {
    half_w = 0.2;
  }

  constexpr double kStep = 0.1;
  for (double s = 0.0; s <= len + 1e-9; s += kStep) {
    const double cx = start_x + ux * std::min(s, len);
    const double cy = start_y + uy * std::min(s, len);
    for (double t = -half_w; t <= half_w + 1e-9; t += kStep) {
      const double qx = cx + nx * t;
      const double qy = cy + ny * t;
      if (!isMapPointPassableOnLocalCostmap(qx, qy)) {
        if (reason) {
          *reason = "lethal on corridor";
        }
        return false;
      }
    }
    if (!cornersClearAt(cx, cy, "lethal at footprint corner")) {
      return false;
    }
  }

  if (!isMapPointPassableOnLocalCostmap(end_x, end_y)) {
    if (reason) {
      *reason = "lethal at garbage";
    }
    return false;
  }

  return true;
}

// 判断 看垃圾点是否在禁行区里面的函数
bool InsertGarbagePose::isPointInPolygon(
  double x, double y, const geometry_msgs::msg::Polygon & polygon)
{
  const auto & pts = polygon.points;
  if (pts.size() < 3) {
    return false;
  }

  // 射线法
  bool inside = false;
  for (std::size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
    const double xi = static_cast<double>(pts[i].x);
    const double yi = static_cast<double>(pts[i].y);
    const double xj = static_cast<double>(pts[j].x);
    const double yj = static_cast<double>(pts[j].y);
    const bool intersect =
      ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

// 判断点是否在禁扫区域内
bool InsertGarbagePose::isPointInSpecialTerrain(double x, double y) const
{
  std::lock_guard<std::mutex> lock(special_terrain_mutex_);
  if (special_terrain_polygons_.empty()) {
    return false;
  }

  for (const auto & polygon : special_terrain_polygons_) {
    if (isPointInPolygon(x, y, polygon)) {
      return true;
    }
  }
  return false;
}

// 把单个垃圾从 base_link 转到 map
bool InsertGarbagePose::transformGarbageToMap(
  capella_ros_msg::msg::GarbageDetect & garbage) const
{
  if (!tf_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped pose_in = garbage.pose;
  if (pose_in.header.frame_id.empty()) {
    pose_in.header.frame_id = robot_base_frame_;
  }

  // 已经在 map 下，不用再转
  if (pose_in.header.frame_id == global_frame_) {
    return true;
  }

  geometry_msgs::msg::PoseStamped pose_out;
  if (!nav2_util::transformPoseInTargetFrame(
      pose_in, pose_out, *tf_, global_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: failed to transform garbage pose from '%s' to '%s'",
      pose_in.header.frame_id.c_str(), global_frame_.c_str());
    return false;
  }

  // bbox 角点一并转到 map
  std::vector<geometry_msgs::msg::Point> corners_map;
  corners_map.reserve(garbage.bbox_corner_points.size());
  for (const auto & corner : garbage.bbox_corner_points) {
    geometry_msgs::msg::PointStamped pin;
    pin.header = pose_in.header;
    pin.point = corner;
    try {
      geometry_msgs::msg::PointStamped pout = tf_->transform(
        pin, global_frame_, tf2::durationFromSec(transform_tolerance_));
      corners_map.push_back(pout.point);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: failed to transform bbox corner: %s", ex.what());
      return false;
    }
  }

  garbage.pose = pose_out;
  garbage.bbox_corner_points = std::move(corners_map);
  return true;
}

// 是否与已保留垃圾过近  去重
bool InsertGarbagePose::isDuplicateOfKept(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const GarbageList & kept)
{
  const double x = garbage.pose.pose.position.x;
  const double y = garbage.pose.pose.position.y;
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;

  for (const auto & existing : kept) {
    const double dx = x - existing.pose.pose.position.x;
    const double dy = y - existing.pose.pose.position.y;
    if (dx * dx + dy * dy < thresh2) {
      return true;
    }
  }
  return false;
}

double InsertGarbagePose::squaredDistanceXY(
  double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

// 队列未满直接加；满了则用更近的新垃圾替换离机器人最远的
bool InsertGarbagePose::tryInsertPreferCloserToRobot(
  capella_ros_msg::msg::GarbageDetect garbage,
  double robot_x, double robot_y)
{
  if (garbage_list_.size() < kMaxGarbageSize) {
    garbage_list_.push_back(std::move(garbage));
    return true;
  }

  const double new_dist2 = squaredDistanceXY(
    garbage.pose.pose.position.x, garbage.pose.pose.position.y,
    robot_x, robot_y);

  // 找队列中离机器人最远的
  std::size_t farthest_idx = 0;
  double farthest_dist2 = -1.0;
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
    if (d2 > farthest_dist2) {
      farthest_dist2 = d2;
      farthest_idx = i;
    }
  }

  // 新垃圾不比最远的更近 → 不加
  if (new_dist2 >= farthest_dist2) {
    return false;
  }

  garbage_list_.erase(garbage_list_.begin() + static_cast<std::ptrdiff_t>(farthest_idx));
  garbage_list_.push_back(std::move(garbage));
  return true;
}

// 距机器人最近的点当种子，到种子小于 merge_radius 的并入同一堆；返回各堆代表点
InsertGarbagePose::GarbageList InsertGarbagePose::mergeGarbagePiles(
  const GarbageList & candidates,
  double robot_x, double robot_y,
  double merge_radius_m)
{
  GarbageList merged_garbage_list;
  if (candidates.empty()) {
    return merged_garbage_list;
  }

  const double radius = std::max(0.0, merge_radius_m);
  const double radius2 = radius * radius;
  // remaining：还没分堆的候选下标
  std::vector<std::size_t> remaining(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    remaining[i] = i;
  }

  while (!remaining.empty()) {
    // 在剩余点里找离机器人最近的，作为本堆种子
    std::size_t seed_pos = 0;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < remaining.size(); ++p) {
      const auto & g = candidates[remaining[p]];
      const double d2 = squaredDistanceXY(
        g.pose.pose.position.x, g.pose.pose.position.y, robot_x, robot_y);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        seed_pos = p;
      }
    }

    const std::size_t seed_idx = remaining[seed_pos];
    const double sx = candidates[seed_idx].pose.pose.position.x;
    const double sy = candidates[seed_idx].pose.pose.position.y;
    merged_garbage_list.push_back(candidates[seed_idx]);

    // 到种子距离 < 半径的算本堆已消化，否则留到下一轮
    std::vector<std::size_t> next;
    next.reserve(remaining.size());
    for (std::size_t p = 0; p < remaining.size(); ++p) {
      if (p == seed_pos) {
        continue;
      }
      const std::size_t idx = remaining[p];
      const double d2 = squaredDistanceXY(
        candidates[idx].pose.pose.position.x,
        candidates[idx].pose.pose.position.y,
        sx, sy);
      if (d2 >= radius2) {
        next.push_back(idx);
      }
    }
    remaining = std::move(next);
  }

  return merged_garbage_list;
}

namespace
{

double wrapAngleRad(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 某一访问顺序的累计转角：起点用 robot_yaw，每到一堆更新朝向
double routeTotalTurnRad(
  const InsertGarbagePose::GarbageList & garbage_list,
  const std::vector<std::size_t> & order,
  double robot_x, double robot_y,
  double robot_yaw)
{
  double heading = robot_yaw;   //当前朝向取机器人的初始朝向 和位置
  double px = robot_x;
  double py = robot_y;
  double total = 0.0;
  for (const std::size_t idx : order) {      // 按照order顺序走
    const double qx = garbage_list[idx].pose.pose.position.x;
    const double qy = garbage_list[idx].pose.pose.position.y;
    const double target_yaw = std::atan2(qy - py, qx - px);
    total += std::fabs(wrapAngleRad(target_yaw - heading));   // 要转的角度
    heading = target_yaw;    //走到这里以后朝向和位置更新
    px = qx;
    py = qy;
  }
  return total;     // 总角度
}

double routeTotalDistM(
  const InsertGarbagePose::GarbageList & garbage_list,
  const std::vector<std::size_t> & order,
  double robot_x, double robot_y)
{
  double px = robot_x;
  double py = robot_y;
  double total = 0.0;
  for (const std::size_t idx : order) {
    const double qx = garbage_list[idx].pose.pose.position.x;
    const double qy = garbage_list[idx].pose.pose.position.y;
    const double dx = qx - px;
    const double dy = qy - py;
    total += std::sqrt(dx * dx + dy * dy);
    px = qx;
    py = qy;
  }
  return total;
}

double weightedSweepScore(double turn_rad, double dist_m, double max_turn, double max_dist,
  double w_turn, double w_dist)
{
  const double turn_n = max_turn > 1e-9 ? turn_rad / max_turn : 0.0;
  const double dist_n = max_dist > 1e-9 ? dist_m / max_dist : 0.0;
  return w_turn * turn_n + w_dist * dist_n;
}

double pointDist2(double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

std::vector<std::size_t> findOrderMinTurn(
  const InsertGarbagePose::GarbageList & garbage_list,
  double robot_x, double robot_y, double robot_yaw)
{
  const std::size_t n = garbage_list.size();
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    order[i] = i;
  }
  if (n <= 1) {
    return order;
  }
  if (n <= InsertGarbagePose::kSweepBruteMaxN) {
    std::vector<std::size_t> best = order;
    double best_turn = std::numeric_limits<double>::infinity();
    std::vector<std::size_t> perm = order;
    do {
      const double turn = routeTotalTurnRad(
        garbage_list, perm, robot_x, robot_y, robot_yaw);
      if (turn < best_turn - 1e-9) {
        best_turn = turn;
        best = perm;
      }
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
  }
  std::vector<std::size_t> remaining = order;
  std::vector<std::size_t> greedy;
  greedy.reserve(n);
  double cur_x = robot_x;
  double cur_y = robot_y;
  double cur_yaw = robot_yaw;
  while (!remaining.empty()) {
    std::size_t best_pos = 0;
    double best_angle = std::numeric_limits<double>::infinity();
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < remaining.size(); ++p) {
      const auto & g = garbage_list[remaining[p]];
      const double qx = g.pose.pose.position.x;
      const double qy = g.pose.pose.position.y;
      const double target_yaw = std::atan2(qy - cur_y, qx - cur_x);
      const double angle = std::fabs(wrapAngleRad(target_yaw - cur_yaw));
      const double d2 = pointDist2(qx, qy, cur_x, cur_y);
      if (angle < best_angle - 1e-6 ||
        (std::fabs(angle - best_angle) < 1e-6 && d2 < best_d2))
      {
        best_angle = angle;
        best_d2 = d2;
        best_pos = p;
      }
    }
    const std::size_t chosen = remaining[best_pos];
    greedy.push_back(chosen);
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_pos));
    const double nx = garbage_list[chosen].pose.pose.position.x;
    const double ny = garbage_list[chosen].pose.pose.position.y;
    cur_yaw = std::atan2(ny - cur_y, nx - cur_x);
    cur_x = nx;
    cur_y = ny;
  }
  return greedy;
}

std::vector<std::size_t> findOrderMinDist(
  const InsertGarbagePose::GarbageList & garbage_list,
  double robot_x, double robot_y)
{
  const std::size_t n = garbage_list.size();
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    order[i] = i;
  }
  if (n <= 1) {
    return order;
  }
  if (n <= InsertGarbagePose::kSweepBruteMaxN) {
    std::vector<std::size_t> best = order;
    double best_dist = std::numeric_limits<double>::infinity();
    std::vector<std::size_t> perm = order;
    do {
      const double dist = routeTotalDistM(garbage_list, perm, robot_x, robot_y);
      if (dist < best_dist - 1e-9) {
        best_dist = dist;
        best = perm;
      }
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
  }
  std::vector<std::size_t> remaining = order;
  std::vector<std::size_t> greedy;
  greedy.reserve(n);
  double cur_x = robot_x;
  double cur_y = robot_y;
  while (!remaining.empty()) {
    std::size_t best_pos = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < remaining.size(); ++p) {
      const auto & g = garbage_list[remaining[p]];
      const double d2 = pointDist2(
        g.pose.pose.position.x, g.pose.pose.position.y, cur_x, cur_y);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_pos = p;
      }
    }
    const std::size_t chosen = remaining[best_pos];
    greedy.push_back(chosen);
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_pos));
    cur_x = garbage_list[chosen].pose.pose.position.x;
    cur_y = garbage_list[chosen].pose.pose.position.y;
  }
  return greedy;
}

}  // namespace

// 返回清扫先后下标，[0] 对应下一堆；输入只读
std::vector<std::size_t> InsertGarbagePose::computeSweepOrder(
  const GarbageList & garbage_list,
  double robot_x, double robot_y,
  double robot_yaw)
{
  const std::size_t n = garbage_list.size();
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {   // order = [0, 1, 2, 3] 
    order[i] = i;
  }
  if (n <= 1) {
    return order;
  }

  std::vector<std::size_t> best = order;

  // 把能走的顺序全试一遍
  if (n <= kSweepBruteMaxN) {
    std::vector<std::size_t> perm = order;
    std::vector<double> turns;
    std::vector<double> dists;
    std::vector<std::vector<std::size_t>> perms;
    do {
      perms.push_back(perm);
      turns.push_back(routeTotalTurnRad(
        garbage_list, perm, robot_x, robot_y, robot_yaw));
      dists.push_back(routeTotalDistM(garbage_list, perm, robot_x, robot_y));
    } while (std::next_permutation(perm.begin(), perm.end()));

    double max_turn = 0.0;
    double max_dist = 0.0;
    for (std::size_t i = 0; i < turns.size(); ++i) {
      max_turn = std::max(max_turn, turns[i]);
      max_dist = std::max(max_dist, dists[i]);
    }

    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < perms.size(); ++i) {
      const double score = weightedSweepScore(
        turns[i], dists[i], max_turn, max_dist,
        sweep_turn_weight_, sweep_dist_weight_);
      if (score < best_score - 1e-9) {
        best_score = score;
        best = perms[i];
      }
    }
  } else {
    // 点数过多 每步按加权得分选下一个
    std::vector<std::size_t> remaining = order;
    std::vector<std::size_t> greedy;
    greedy.reserve(n);

    double cur_x = robot_x;
    double cur_y = robot_y;
    double cur_yaw = robot_yaw;

    while (!remaining.empty()) {
      std::vector<double> step_turn(remaining.size(), 0.0);
      std::vector<double> step_dist(remaining.size(), 0.0);
      double max_turn = 0.0;
      double max_dist = 0.0;
      for (std::size_t p = 0; p < remaining.size(); ++p) {
        const auto & g = garbage_list[remaining[p]];
        const double qx = g.pose.pose.position.x;
        const double qy = g.pose.pose.position.y;
        const double target_yaw = std::atan2(qy - cur_y, qx - cur_x);
        step_turn[p] = std::fabs(wrapAngleRad(target_yaw - cur_yaw));
        step_dist[p] = std::sqrt(squaredDistanceXY(qx, qy, cur_x, cur_y));
        max_turn = std::max(max_turn, step_turn[p]);
        max_dist = std::max(max_dist, step_dist[p]);
      }

      std::size_t best_pos = 0;
      double best_step = std::numeric_limits<double>::infinity();
      for (std::size_t p = 0; p < remaining.size(); ++p) {
        const double score = weightedSweepScore(
          step_turn[p], step_dist[p], max_turn, max_dist,
          sweep_turn_weight_, sweep_dist_weight_);
        if (score < best_step - 1e-9) {
          best_step = score;
          best_pos = p;
        }
      }

      const std::size_t chosen = remaining[best_pos];
      greedy.push_back(chosen);
      remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_pos));
      const double nx = garbage_list[chosen].pose.pose.position.x;
      const double ny = garbage_list[chosen].pose.pose.position.y;
      cur_yaw = std::atan2(ny - cur_y, nx - cur_x);
      cur_x = nx;
      cur_y = ny;
    }
    best = std::move(greedy);
  }

  return best;
}

void InsertGarbagePose::reorderNearestFirstThenSweep(
  double robot_x, double robot_y, double robot_yaw)
{
  if (garbage_list_.size() <= 1) {
    if (!garbage_list_.empty()) {
      syncLastSweepXyFromList();
    }
    return;
  }

  std::size_t nearest_idx = 0;
  double best_d2 = squaredDistanceXY(
    garbage_list_[0].pose.pose.position.x,
    garbage_list_[0].pose.pose.position.y,
    robot_x, robot_y);
  for (std::size_t i = 1; i < garbage_list_.size(); ++i) {
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest_idx = i;
    }
  }

  GarbageList rest;
  rest.reserve(garbage_list_.size() - 1);
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    if (i != nearest_idx) {
      rest.push_back(garbage_list_[i]);
    }
  }

  GarbageList reordered;
  reordered.reserve(garbage_list_.size());
  reordered.push_back(garbage_list_[nearest_idx]);

  const double nx = reordered.front().pose.pose.position.x;
  const double ny = reordered.front().pose.pose.position.y;
  double nyaw = robot_yaw;
  const double dx = nx - robot_x;
  const double dy = ny - robot_y;
  if (dx * dx + dy * dy > 1e-6) {
    nyaw = std::atan2(dy, dx);
  }

  if (!rest.empty()) {
    const auto sub = computeSweepOrder(rest, nx, ny, nyaw);
    for (const std::size_t k : sub) {
      if (k < rest.size()) {
        reordered.push_back(rest[k]);
      }
    }
  }

  garbage_list_ = std::move(reordered);
  syncLastSweepXyFromList();
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: nearest-first then sweep, next (%.2f, %.2f), n=%zu",
    garbage_list_.front().pose.pose.position.x,
    garbage_list_.front().pose.pose.position.y,
    garbage_list_.size());
}

// 按下标顺序重建 garbage_list_，[0] 即下一堆（必须是离车最近的）
void InsertGarbagePose::reorderGarbageListBySweep(
  double robot_x, double robot_y, double robot_yaw)
{
  reorderNearestFirstThenSweep(robot_x, robot_y, robot_yaw);
}

void InsertGarbagePose::syncLastSweepXyFromList() // 把排好的顺序保存下来
{
  last_sweep_xy_.clear();
  last_sweep_xy_.reserve(garbage_list_.size());
  for (const auto & g : garbage_list_) {
    last_sweep_xy_.emplace_back(
      g.pose.pose.position.x, g.pose.pose.position.y);
  }
}

bool InsertGarbagePose::findNewGarbageIndex(   //找新垃圾的
  const GarbageList & before,
  double robot_x, double robot_y,
  std::size_t & new_idx) const
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  std::vector<std::size_t> news;
  news.reserve(garbage_list_.size());
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    const double x = garbage_list_[i].pose.pose.position.x;
    const double y = garbage_list_[i].pose.pose.position.y;
    bool existed = false;
    for (const auto & b : before) {
      if (squaredDistanceXY(
          x, y, b.pose.pose.position.x, b.pose.pose.position.y) < thresh2)
      {
        existed = true;
        break;
      }
    }
    if (!existed) {
      news.push_back(i);
    }
  }
  if (news.empty()) {
    return false;
  }

  new_idx = news.front();
  double best_d2 = squaredDistanceXY(
    garbage_list_[new_idx].pose.pose.position.x,
    garbage_list_[new_idx].pose.pose.position.y,
    robot_x, robot_y);
  for (std::size_t k = 1; k < news.size(); ++k) {
    const std::size_t i = news[k];
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
    if (d2 < best_d2) {
      best_d2 = d2;
      new_idx = i;
    }
  }
  return true;
}

bool InsertGarbagePose::reorderGarbageListWithNewPile(   // 新来的那一个新垃圾插到一个合理的位置
  double robot_x, double robot_y, double robot_yaw,
  std::size_t new_idx)
{
  const std::size_t n = garbage_list_.size();
  if (n == 0 || new_idx >= n) {
    return false;
  }

  // 除新堆外离机器人最近的一堆；正在扫的 pending 也算旧堆
  std::size_t nearest_idx = n;
  bool nearest_is_pending = false;
  double nearest_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    if (i == new_idx) {
      continue;
    }
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
    if (d2 < nearest_d2) {
      nearest_d2 = d2;
      nearest_idx = i;
    }
  }
  if (has_pending_garbage_) {
    const double d2 = squaredDistanceXY(
      pending_garbage_xy_.first, pending_garbage_xy_.second,
      robot_x, robot_y);
    if (d2 < nearest_d2) {
      nearest_d2 = d2;
      nearest_is_pending = true;
      nearest_idx = n;
    }
  }
  if (!nearest_is_pending && nearest_idx >= n) {
    return false;
  }

  const double nx = garbage_list_[new_idx].pose.pose.position.x;
  const double ny = garbage_list_[new_idx].pose.pose.position.y;
  const double nrx = nearest_is_pending ?
    pending_garbage_xy_.first : garbage_list_[nearest_idx].pose.pose.position.x;
  const double nry = nearest_is_pending ?
    pending_garbage_xy_.second : garbage_list_[nearest_idx].pose.pose.position.y;

  double foot_x = 0.0;
  double foot_y = 0.0;
  projectPointToInfiniteLine(
    nx, ny, robot_x, robot_y, nrx, nry, foot_x, foot_y);
  const double t = lineParameterT(
    foot_x, foot_y, robot_x, robot_y, nrx, nry);

  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  GarbageList reordered;
  reordered.reserve(n);

  if (t >= 0.0 && t <= 1.0) {
    // 仅新堆在队列里：不用重排，4-1 插到队首并破 pending
    if (n <= 1) {
      syncLastSweepXyFromList();
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 新堆插队(4-1) t=%.2f, next (%.2f, %.2f)",
        t, nx, ny);
      return true;
    }
    // 新垃圾插到队首，其余尽量保持上次顺序
    reordered.push_back(garbage_list_[new_idx]);
    std::vector<bool> used(n, false);
    used[new_idx] = true;

    for (const auto & xy : last_sweep_xy_) {
      for (std::size_t i = 0; i < n; ++i) {
        if (used[i]) {
          continue;
        }
        if (squaredDistanceXY(
            garbage_list_[i].pose.pose.position.x,
            garbage_list_[i].pose.pose.position.y,
            xy.first, xy.second) < thresh2)
        {
          reordered.push_back(garbage_list_[i]);
          used[i] = true;
          break;
        }
      }
    }

    GarbageList missing;
    missing.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (!used[i]) {
        missing.push_back(garbage_list_[i]);
      }
    }
    if (!missing.empty()) {
      const auto sub = computeSweepOrder(missing, robot_x, robot_y, robot_yaw);
      for (const std::size_t k : sub) {
        if (k < missing.size()) {
          reordered.push_back(missing[k]);
        }
      }
    }

    garbage_list_ = std::move(reordered);
    reorderNearestFirstThenSweep(robot_x, robot_y, robot_yaw);
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 新堆插队(4-1) t=%.2f, next (%.2f, %.2f)",
      t, garbage_list_.front().pose.pose.position.x,
      garbage_list_.front().pose.pose.position.y);
    return squaredDistanceXY(
      garbage_list_.front().pose.pose.position.x,
      garbage_list_.front().pose.pose.position.y, nx, ny) < thresh2;
  }

  // 仅新堆：不在半路上，等当前堆扫完再插
  if (n <= 1 || nearest_is_pending) {
    if (n > 1) {
      reorderGarbageListBySweep(robot_x, robot_y, robot_yaw);
    } else {
      syncLastSweepXyFromList();
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 新堆重排(4-2) t=%.2f, next (%.2f, %.2f)",
      t, garbage_list_.front().pose.pose.position.x,
      garbage_list_.front().pose.pose.position.y);
    return false;
  }

  // 最近堆仍第一，其余按最小转角重排
  reordered.push_back(garbage_list_[nearest_idx]);
  GarbageList rest;
  rest.reserve(n - 1);
  for (std::size_t i = 0; i < n; ++i) {
    if (i == nearest_idx) {
      continue;
    }
    rest.push_back(garbage_list_[i]);
  }
  if (!rest.empty()) {
    const auto sub = computeSweepOrder(rest, robot_x, robot_y, robot_yaw);
    for (const std::size_t k : sub) {
      if (k < rest.size()) {
        reordered.push_back(rest[k]);
      }
    }
  }

  garbage_list_ = std::move(reordered);
  reorderNearestFirstThenSweep(robot_x, robot_y, robot_yaw);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 新堆重排(4-2) t=%.2f, next (%.2f, %.2f)",
    t, garbage_list_.front().pose.pose.position.x,
    garbage_list_.front().pose.pose.position.y);
  return false;
}

// 接收到垃圾后的后处理函数，返回处理后的 garbage_list

InsertGarbagePose::GarbageList InsertGarbagePose::postProcessHistory()
{
  std::deque<capella_ros_msg::msg::GarbageDetect> snapshot;
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    snapshot = history_list_;
  } // 加锁

  if (snapshot.empty()) {
    return garbage_list_;
  }

  // base_link到map
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: getCurrentPose failed, keep history for next tick");
    return garbage_list_;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("garbage_merge_radius_m", garbage_merge_radius_m_);
  getInput("work_circle_radius_m", work_circle_radius_m_);

  auto sameDetect =     // id，stamp，xy 都相同则认为是同一条垃圾
    [](const capella_ros_msg::msg::GarbageDetect & a,
      const capella_ros_msg::msg::GarbageDetect & b) {
      return a.class_id == b.class_id &&
             a.pose.header.stamp == b.pose.header.stamp &&
             std::fabs(a.pose.pose.position.x - b.pose.pose.position.x) < 1e-6 &&
             std::fabs(a.pose.pose.position.y - b.pose.pose.position.y) < 1e-6;
    };

  // 从 history 里删掉已处理完的那一条
  auto eraseFromHistory =
    [this, &sameDetect](const capella_ros_msg::msg::GarbageDetect & original) {
      std::lock_guard<std::mutex> lock(history_mutex_);
      for (auto it = history_list_.begin(); it != history_list_.end(); ++it) {
        if (sameDetect(*it, original)) {
          history_list_.erase(it);
          return;
        }
      }
    };

  // 清掉合进该代表点的那一堆在 history 里的原始检测
  auto erasePileAroundSeed =
    [&](const capella_ros_msg::msg::GarbageDetect & seed,
      const GarbageList & map_pts,
      const std::vector<capella_ros_msg::msg::GarbageDetect> & originals)
    {
      const double sx = seed.pose.pose.position.x;
      const double sy = seed.pose.pose.position.y;
      const double radius2 = std::max(0.0, garbage_merge_radius_m_) *
        std::max(0.0, garbage_merge_radius_m_);
      for (std::size_t i = 0; i < map_pts.size(); ++i) {
        const double d2 = squaredDistanceXY(
          map_pts[i].pose.pose.position.x, map_pts[i].pose.pose.position.y,
          sx, sy);
        // 种子自身 d2==0，半径为 0 时也要清掉
        if (d2 < radius2 || d2 < 1e-12) {
          eraseFromHistory(originals[i]);
        }
      }
    };

  GarbageList candidates;
  std::vector<capella_ros_msg::msg::GarbageDetect> candidate_originals;
  candidates.reserve(snapshot.size());
  candidate_originals.reserve(snapshot.size());

  for (const auto & original : snapshot) {
    capella_ros_msg::msg::GarbageDetect garbage = original;

    // 转到 map，失败则留在 history
    if (!transformGarbageToMap(garbage)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为转到map失败, 暂不处理",
        original.pose.pose.position.x, original.pose.pose.position.y);
      continue;
    }

    const double gx = garbage.pose.pose.position.x;
    const double gy = garbage.pose.pose.position.y;

    // 落在禁扫区则丢弃
    if (isPointInSpecialTerrain(gx, gy)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为在禁扫区, 丢弃", gx, gy);
      eraseFromHistory(original);
      continue;
    }

    // 离机器人太远：视为误识别，丢弃
    if (max_garbage_robot_dist_m_ > 0.0) {
      const double dist_robot = std::sqrt(squaredDistanceXY(gx, gy, robot_x, robot_y));
      if (dist_robot > max_garbage_robot_dist_m_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为离机器人过远(%.2fm>%.2fm), 丢弃",
          gx, gy, dist_robot, max_garbage_robot_dist_m_);
        eraseFromHistory(original);
        continue;
      }
    }

    // 局部代价图障碍物情况不可读：无图 / 图外 / unknown，丢弃
    {
      std::string costmap_reason;
      if (!isObstacleInfoReadable(gx, gy, &costmap_reason)) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为局部代价图不可读(%s), 丢弃",
          gx, gy, costmap_reason.c_str());
        eraseFromHistory(original);
        continue;
      }
    }

    // 垃圾周围 clearance 内有硬障碍：贴墙扫不了，直接丢弃，不进后续规划
    getInput("min_garbage_obstacle_clearance_m", min_garbage_obstacle_clearance_m_);
    if (min_garbage_obstacle_clearance_m_ > 0.0 &&
      hasObstacleWithinRadius(gx, gy, min_garbage_obstacle_clearance_m_))
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为太靠近障碍物(%.2fm内), 丢弃",
        gx, gy, min_garbage_obstacle_clearance_m_);
      eraseFromHistory(original);
      continue;
    }

    // 机器人到垃圾直线走廊有 lethal 则丢弃
    {
      std::string corridor_reason;
      if (!isStraightCorridorClear(robot_x, robot_y, gx, gy, &corridor_reason)) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为走廊不通(%s), 丢弃",
          gx, gy, corridor_reason.c_str());
        eraseFromHistory(original);
        continue;
      }
    }

    if (work_circle_radius_m_ > 0.0) {
      if (!has_work_circle_ && garbage_list_.empty() && active_piles_.empty()) {
        has_work_circle_ = true;
        work_circle_x_ = robot_x;
        work_circle_y_ = robot_y;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: 生成工作圈 圆心=(%.2f, %.2f) 半径=%.2f m",
          work_circle_x_, work_circle_y_, work_circle_radius_m_);
        publishRangeCircles(robot_x, robot_y);
      }
      if (has_work_circle_) {
        const double d_circle = std::sqrt(squaredDistanceXY(
            gx, gy, work_circle_x_, work_circle_y_));
        if (d_circle > work_circle_radius_m_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为在工作圈外(%.2fm>%.2fm), 丢弃",
            gx, gy, d_circle, work_circle_radius_m_);
          eraseFromHistory(original);
          continue;
        }
      }
    }

    candidates.push_back(std::move(garbage));
    candidate_originals.push_back(original);
  }

  if (candidates.empty()) {
    return garbage_list_;
  }

  // 合堆后的代表点，再写入成员 garbage_list_
  GarbageList merged_garbage_list = mergeGarbagePiles(
    candidates, robot_x, robot_y, garbage_merge_radius_m_);

  for (auto & seed : merged_garbage_list) {
    const double sx = seed.pose.pose.position.x;
    const double sy = seed.pose.pose.position.y;

    // 已插入过的堆不再进候选，避免持续发布反复占队首
    bool near_reached = false;
    constexpr double kReachedThreshM = 0.5;
    const double reached_thresh2 = kReachedThreshM * kReachedThreshM;
    for (const auto & reached : reached_garbage_xy_) {
      if (squaredDistanceXY(sx, sy, reached.first, reached.second) < reached_thresh2) {
        near_reached = true;
        break;
      }
    }
    if (near_reached) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为已到达/已插入过, 丢弃", sx, sy);
      erasePileAroundSeed(seed, candidates, candidate_originals);
      continue;
    }

    if (isDuplicateOfKept(seed, garbage_list_)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为与已规划堆重复, 丢弃", sx, sy);
      erasePileAroundSeed(seed, candidates, candidate_originals);
      continue;
    }

    if (tryInsertPreferCloserToRobot(seed, robot_x, robot_y)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 进入清扫规划", sx, sy);
      erasePileAroundSeed(seed, candidates, candidate_originals);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为队列已满且更远, 丢弃",
        sx, sy);
    }
  }

  return garbage_list_;
}

// 接收完整的 {goals} 路径点
InsertGarbagePose::Goals InsertGarbagePose::receiveGoals()
{
  Goals goals;
  if (!getInput("input_goals", goals)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: failed to get input_goals");
    received_goals_.clear();
    return received_goals_;
  }

  received_goals_ = goals;
  return received_goals_;
}

// 对比 goals 时间戳，外部重发任务时清空 history 和 garbage
// 看当前是不是一次新的导航任务；如果是，就把上一任务留下的垃圾缓存清掉，免得串到新任务里
void InsertGarbagePose::checkAndResetOnNewMission()
{
  const Goals goals = receiveGoals();
  if (goals.empty()) {
    return;
  }
  // 取第一个goals
  const rclcpp::Time current_stamp = goals.front().header.stamp;   
  if (!has_mission_stamp_) {
    mission_stamp_record_ = current_stamp;
    has_mission_stamp_ = true;
    return;
  }

  if (current_stamp == mission_stamp_record_) {  // 时间戳相同，说明是同一任务
    return;
  }

  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    history_list_.clear();
  }
  garbage_list_.clear();
  active_piles_.clear();
  reached_garbage_xy_.clear();
  viz_obstacle_pixels_.clear();
  viz_obstacle_marker_count_ = 0;
  has_pending_garbage_ = false;
  bypass_pending_insert_ = false;
  last_sweep_xy_.clear();
  has_last_sweep_arrive_ = false;
  last_sweep_arrive_xy_ = {0.0, 0.0};
  viz_pile_count_ = 0;
  footprint_stripped_viz_.clear();
  g_num_xy_.clear();
  next_g_num_ = 1;
  has_last_viz_time_ = false;
  has_work_circle_ = false;
  mission_stamp_record_ = current_stamp;
  clearMissionVisualization();

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 更新扫地任务");
}

// 获取机器人当前 footprint，并转到 map
bool InsertGarbagePose::getRobotFootprintInMap(
  std::vector<geometry_msgs::msg::Point> & footprint_map) const
{
  footprint_map.clear();
  if (!tf_) {
    return false;
  }

  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    footprint_msg = latest_footprint_;
  }
  if (!footprint_msg || footprint_msg->polygon.points.empty()) {
    return false;
  }

  std::string source_frame = footprint_msg->header.frame_id;
  if (source_frame.empty()) {
    source_frame = robot_base_frame_;
  }

  for (const auto & pt32 : footprint_msg->polygon.points) {
    geometry_msgs::msg::PointStamped pin;
    pin.header.frame_id = source_frame;
    pin.header.stamp = footprint_msg->header.stamp;
    pin.point.x = pt32.x;
    pin.point.y = pt32.y;
    pin.point.z = pt32.z;
    try {
      geometry_msgs::msg::PointStamped pout = tf_->transform(
        pin, global_frame_, tf2::durationFromSec(transform_tolerance_));
      footprint_map.push_back(pout.point);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: failed to transform footprint point: %s", ex.what());
      footprint_map.clear();
      return false;
    }
  }
  return !footprint_map.empty();
}

bool InsertGarbagePose::getRobotFootprintInBase(
  std::vector<std::pair<double, double>> & local_xy) const
{
  local_xy.clear();
  if (!tf_) {
    return false;
  }

  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    footprint_msg = latest_footprint_;
  }
  if (!footprint_msg || footprint_msg->polygon.points.empty()) {
    return false;
  }

  std::string source_frame = footprint_msg->header.frame_id;
  if (source_frame.empty()) {
    source_frame = robot_base_frame_;
  }

  const bool already_base = (source_frame == robot_base_frame_);
  for (const auto & pt32 : footprint_msg->polygon.points) {
    if (already_base) {
      local_xy.emplace_back(static_cast<double>(pt32.x), static_cast<double>(pt32.y));
      continue;
    }
    geometry_msgs::msg::PointStamped pin;
    pin.header.frame_id = source_frame;
    pin.header.stamp = footprint_msg->header.stamp;
    pin.point.x = pt32.x;
    pin.point.y = pt32.y;
    pin.point.z = pt32.z;
    try {
      geometry_msgs::msg::PointStamped pout = tf_->transform(
        pin, robot_base_frame_, tf2::durationFromSec(transform_tolerance_));
      local_xy.emplace_back(pout.point.x, pout.point.y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: failed to transform footprint to base: %s", ex.what());
      local_xy.clear();
      return false;
    }
  }
  return !local_xy.empty();
}

// 判断 footprint 是否已进入垃圾附近
bool InsertGarbagePose::shouldStopInsertingGarbage(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double arrived_radius,
  double robot_x, double robot_y, double robot_yaw) const
{
  (void)arrived_radius;
  const double gx = garbage.pose.pose.position.x;
  const double gy = garbage.pose.pose.position.y;
  return isGarbageCoveredByFootprint(
    gx, gy, footprint_map, robot_x, robot_y, robot_yaw);
}

int InsertGarbagePose::lookupStableGNum(double x, double y) const
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (const auto & item : g_num_xy_) {
    if (squaredDistanceXY(item.first.first, item.first.second, x, y) < thresh2) {
      return item.second;
    }
  }
  return 0;
}

int InsertGarbagePose::assignStableGNum(double x, double y)
{
  const int existing = lookupStableGNum(x, y);
  if (existing > 0) {
    return existing;
  }
  const int num = next_g_num_++;
  g_num_xy_.push_back({{x, y}, num});
  return num;
}

bool InsertGarbagePose::isGarbageCoveredByFootprint(
  double gx, double gy,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double robot_x, double robot_y, double robot_yaw,
  double * dist_robot_m,
  double * base_x,
  double * base_y) const
{
  const double dx = gx - robot_x;
  const double dy = gy - robot_y;
  const double dist = std::hypot(dx, dy);
  if (dist_robot_m != nullptr) {
    *dist_robot_m = dist;
  }

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  const double bx = dx * c + dy * s;
  const double by = -dx * s + dy * c;
  if (base_x != nullptr) {
    *base_x = bx;
  }
  if (base_y != nullptr) {
    *base_y = by;
  }

  // 车头可到 1.25m，只靠多边形会在车体还没到时就删；必须车体中心也到
  if (arrived_radius_ > 0.0 && dist > arrived_radius_) {
    return false;
  }

  geometry_msgs::msg::Polygon footprint_poly;
  std::vector<std::pair<double, double>> local_xy;
  if (getRobotFootprintInBase(local_xy) && local_xy.size() >= 3) {
    footprint_poly.points.reserve(local_xy.size());
    for (const auto & xy : local_xy) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(xy.first);
      p32.y = static_cast<float>(xy.second);
      footprint_poly.points.push_back(p32);
    }
    return isPointInPolygon(bx, by, footprint_poly);
  }

  if (footprint_map.size() < 3) {
    return false;
  }
  footprint_poly.points.reserve(footprint_map.size());
  for (const auto & pt : footprint_map) {
    const double pdx = pt.x - robot_x;
    const double pdy = pt.y - robot_y;
    geometry_msgs::msg::Point32 p32;
    p32.x = static_cast<float>(pdx * c + pdy * s);
    p32.y = static_cast<float>(-pdx * s + pdy * c);
    footprint_poly.points.push_back(p32);
  }
  return isPointInPolygon(bx, by, footprint_poly);
}

InsertGarbagePose::SentinelArrivalDetail InsertGarbagePose::probeSentinelArrival(
  double gx, double gy,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double arrived_radius) const
{
  SentinelArrivalDetail detail;
  if (footprint_map.empty() || arrived_radius <= 0.0) {
    return detail;
  }

  const double r2 = arrived_radius * arrived_radius;
  for (const auto & pt : footprint_map) {
    const double dist = std::sqrt(squaredDistanceXY(pt.x, pt.y, gx, gy));
    if (dist < detail.min_vertex_dist_m) {
      detail.min_vertex_dist_m = dist;
    }
    if (squaredDistanceXY(pt.x, pt.y, gx, gy) < r2) {
      detail.by_vertex_radius = true;
      detail.arrived = true;
    }
  }

  if (!detail.arrived) {
    geometry_msgs::msg::Polygon footprint_poly;
    footprint_poly.points.reserve(footprint_map.size());
    for (const auto & pt : footprint_map) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(pt.x);
      p32.y = static_cast<float>(pt.y);
      p32.z = static_cast<float>(pt.z);
      footprint_poly.points.push_back(p32);
    }
    detail.by_inside_polygon = isPointInPolygon(gx, gy, footprint_poly);
    detail.arrived = detail.by_inside_polygon;
  }
  return detail;
}

// 判断 goals 里某点是否为本节点写入的 G/E，而不是编号途经点
bool InsertGarbagePose::isUnindexedSentinelPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  // 与写入端统一：圆整后等于 kGarbageSentinelPoseZ 即为 G/E
  return std::lround(pose_stamped_goal.pose.position.z) ==
         std::lround(kGarbageSentinelPoseZ);
}

// 每 tick 只检查队首：是 G/E 且 footprint 到了才删这一个点
std::size_t InsertGarbagePose::stripReachedZNeg1Goals(
  Goals & goals,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double robot_x, double robot_y, double robot_yaw,
  std::string * deleted_summary)
{
  if (footprint_map.empty() || goals.empty()) {
    return 0;
  }

  const geometry_msgs::msg::PoseStamped & current = goals.front();
  if (!isUnindexedSentinelPoseZ(current)) {
    return 0;
  }

  const double gx = current.pose.position.x;
  const double gy = current.pose.position.y;
  double dist_robot = 0.0;
  double base_x = 0.0;
  double base_y = 0.0;
  if (!isGarbageCoveredByFootprint(
      gx, gy, footprint_map, robot_x, robot_y, robot_yaw,
      &dist_robot, &base_x, &base_y))
  {
    return 0;
  }

  // 稳定 G 编号；对不上已登记堆则视为延伸点 E
  std::string point_label = "E";
  const int g_num = lookupStableGNum(gx, gy);
  if (g_num > 0) {
    point_label = "G" + std::to_string(g_num);
  }

  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  goals.erase(goals.begin());
  addProtectedGarbageXy(gx, gy);

  for (auto it = active_piles_.begin(); it != active_piles_.end(); ) {
    const double ax = it->pose.pose.position.x;
    const double ay = it->pose.pose.position.y;
    if (squaredDistanceXY(ax, ay, gx, gy) < thresh2) {
      it = active_piles_.erase(it);
    } else {
      ++it;
    }
  }

  footprint_stripped_viz_.push_back({gx, gy, point_label});
  publishFootprintStrippedMarkers();

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: footprint删点 %s (%.2f, %.2f) robot=(%.2f, %.2f) "
    "dist=%.2fm base=(%.2f, %.2f) 已覆盖删除",
    point_label.c_str(), gx, gy, robot_x, robot_y, dist_robot, base_x, base_y);

  if (deleted_summary != nullptr) {
    std::ostringstream deleted_oss;
    deleted_oss << point_label << " (" << gx << "," << gy << ")";
    *deleted_summary = deleted_oss.str();
  }
  return 1;
}

// 真正写黑板前打一条日志，便于观察 output 时机和频率
void InsertGarbagePose::emitOutputGoals(const Goals & goals, const char * reason)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: output_goals emit reason=%s goals=%zu",
    reason, goals.size());
  setOutput("output_goals", goals);
}

bool InsertGarbagePose::isNearReachedGarbage(double x, double y) const
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (const auto & reached : reached_garbage_xy_) {
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}

// 该点是否已在已插入垃圾附近
bool InsertGarbagePose::isProtectedGarbageXy(double x, double y) const
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (const auto & reached : reached_garbage_xy_) {
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}

// 记录已插入的垃圾点，防重复添加
void InsertGarbagePose::addProtectedGarbageXy(double x, double y)
{
  if (isProtectedGarbageXy(x, y)) {
    return;
  }
  reached_garbage_xy_.emplace_back(x, y);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: diag protected+ (%.2f, %.2f), protected_n=%zu",
    x, y, reached_garbage_xy_.size());
}

void InsertGarbagePose::publishProtectedGarbage()
{
  Goals protected_goals;
  protected_goals.reserve(reached_garbage_xy_.size());
  for (const auto & xy : reached_garbage_xy_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    protected_goals.push_back(std::move(pose));
  }
  setOutput("protected_garbage", protected_goals);
}

bool InsertGarbagePose::isPendingGarbageInGoals(const Goals & goals) const
{
  if (!has_pending_garbage_) {
    return false;
  }
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (const auto & g : goals) {
    if (squaredDistanceXY(
        g.pose.position.x, g.pose.position.y,
        pending_garbage_xy_.first, pending_garbage_xy_.second) < thresh2)
    {
      return true;
    }
  }
  return false;
}

namespace
{

// 点到有限线段 AB 的距离平方；t_out 为夹在 [0,1] 的投影参数
double squaredDistancePointToSegment(
  double px, double py,
  double ax, double ay,
  double bx, double by,
  double * t_out = nullptr)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab_len2 = abx * abx + aby * aby;
  double t = 0.0;
  if (ab_len2 > 1e-12) {
    t = (apx * abx + apy * aby) / ab_len2;
    t = std::clamp(t, 0.0, 1.0);
  }
  if (t_out) {
    *t_out = t;
  }
  const double qx = ax + t * abx;
  const double qy = ay + t * aby;
  const double dx = px - qx;
  const double dy = py - qy;
  return dx * dx + dy * dy;
}

}  // namespace

// 点到无限直线 AB 的垂足
void InsertGarbagePose::projectPointToInfiniteLine(
  double px, double py,
  double ax, double ay,
  double bx, double by,
  double & out_x, double & out_y)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab_len2 = abx * abx + aby * aby;
  if (ab_len2 < 1e-12) {
    out_x = ax;
    out_y = ay;
    return;
  }
  const double apx = px - ax;
  const double apy = py - ay;
  const double t = (apx * abx + apy * aby) / ab_len2;
  out_x = ax + t * abx;
  out_y = ay + t * aby;
}

// 点在无限直线 AB 上的参数 t
double InsertGarbagePose::lineParameterT(
  double px, double py,
  double ax, double ay,
  double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab_len2 = abx * abx + aby * aby;
  if (ab_len2 < 1e-12) {
    return 0.0;
  }
  return ((px - ax) * abx + (py - ay) * aby) / ab_len2;
}

// true=非角点，false=角点：这个点的前面一个点指向自己的方向，和自己指向下一个点的方向大于一个corner_angle_deg_，就算角点
bool InsertGarbagePose::isGoalNotCorner(
  const Goals & goals,
  std::size_t idx,
  double robot_x, double robot_y) const
{
  // 最后一个点不做角点判断
  if (goals.empty() || idx >= goals.size() - 1) {
    return true;
  }

  // 前驱：idx>0 用上一 goal；队首用机器人
  double prev_x = robot_x;
  double prev_y = robot_y;
  if (idx > 0) {
    prev_x = goals[idx - 1].pose.position.x;
    prev_y = goals[idx - 1].pose.position.y;
  }

  const double cx = goals[idx].pose.position.x;
  const double cy = goals[idx].pose.position.y;
  const double nx = goals[idx + 1].pose.position.x;
  const double ny = goals[idx + 1].pose.position.y;

  // v1：前一个点 → 当前点；v2：当前点 → 后一个点
  const double v1x = cx - prev_x;
  const double v1y = cy - prev_y;
  const double v2x = nx - cx;
  const double v2y = ny - cy;

  // 段长过短时不做角度判断
  const double len1_sq = v1x * v1x + v1y * v1y;
  const double len2_sq = v2x * v2x + v2y * v2y;
  constexpr double kMinSegLenSq = 1e-6;
  if (len1_sq < kMinSegLenSq || len2_sq < kMinSegLenSq) {
    return true;
  }

  // 两段方向夹角 θ：直线约 0°，直角约 90°
  const double dot = v1x * v2x + v1y * v2y;
  const double cos_theta = std::clamp(
    dot / std::sqrt(len1_sq * len2_sq), -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  const double corner_angle_rad = corner_angle_deg_ * M_PI / 180.0;

  return theta <= corner_angle_rad;
}

// 从机器人前方沿路径找第一个角点
bool InsertGarbagePose::findFirstCornerFromRobot(
  const Goals & goals,
  double robot_x, double robot_y,
  std::size_t & corner_idx) const
{
  if (goals.size() < 2) {
    return false;
  }

  // 从离机器人最近的原路径点的后一个点开始找角点
  std::size_t goala_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  bool have_path_goal = false;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const double px = goals[i].pose.position.x;
    const double py = goals[i].pose.position.y;
    if (isProtectedGarbageXy(px, py)) {
      continue;
    }
    const double d2 = squaredDistanceXY(px, py, robot_x, robot_y);
    if (d2 < best_d2) {
      best_d2 = d2;
      goala_idx = i;
      have_path_goal = true;
    }
  }
  if (!have_path_goal) {
    return false;
  }

  const std::size_t start_idx = goala_idx + 1;
  if (start_idx >= goals.size()) {
    return false;
  }

  std::size_t range_end = start_idx;
  double accumulated = 0.0;
  for (std::size_t i = start_idx; i + 1 < goals.size(); ++i) {
    accumulated += std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    if (accumulated > goaltotal_range_m_) {
      break;
    }
    range_end = i + 1;
  }

  for (std::size_t i = start_idx; i <= range_end && i < goals.size(); ++i) {
    if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
      continue;
    }
    if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
      corner_idx = i;
      return true;
    }
  }
  return false;
}

// 从 after_idx 之后找下一个角点
bool InsertGarbagePose::findNextCornerAfter(
  const Goals & goals,
  std::size_t after_idx,
  double robot_x, double robot_y,
  std::size_t & corner_idx) const
{
  if (goals.size() < 2 || after_idx + 1 >= goals.size()) {
    return false;
  }

  // 从 after_idx 起沿路径累加 range，得到搜索上界
  std::size_t range_end = after_idx;
  double accumulated = 0.0;
  for (std::size_t i = after_idx; i + 1 < goals.size(); ++i) {
    accumulated += std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    if (accumulated > goaltotal_range_m_) {
      break;
    }
    range_end = i + 1;
  }
  if (range_end <= after_idx) {
    return false;
  }

  for (std::size_t i = after_idx + 1; i <= range_end; ++i) {
    if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
      continue;
    }
    if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
      corner_idx = i;
      return true;
    }
  }
  return false;
}

// 根据累计路径 range_m ，去找最后一个点的下标
bool InsertGarbagePose::findLastGoalWithinPathRange(
  const Goals & goals,
  double robot_x, double robot_y,
  double range_m,
  std::size_t & out_idx,
  std::size_t * nearest_seg_out,
  std::size_t * start_idx_out) const
{
  if (goals.size() < 2 || range_m <= 0.0) {
    return false;
  }

  // 起点取离机器人最近的路径段，避免范围锚定到导航起点
  std::size_t start_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
    double t_seg = 0.0;
    const double d2 = squaredDistancePointToSegment(
      robot_x, robot_y,
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y,
      &t_seg);
    if (d2 < best_d2) {
      best_d2 = d2;
      start_idx = i;
    }
  }

  double accumulated = 0.0;
  out_idx = start_idx;
  for (std::size_t i = start_idx; i + 1 < goals.size(); ++i) {
    const double seg_len = std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    accumulated += seg_len;
    if (accumulated > range_m) {
      break;
    }
    out_idx = i + 1;
  }

  if (nearest_seg_out) {
    *nearest_seg_out = start_idx;
  }
  if (start_idx_out) {
    *start_idx_out = start_idx;
  }
  return true;
}

// 获取插入所需的全部信息并返回
InsertGarbagePose::InsertInfo InsertGarbagePose::gatherInsertInfo(
  const Goals & goals,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double garbage_x, double garbage_y)
{
  InsertInfo info;
  info.goals = goals;
  info.robot_pose = robot_pose;
  info.garbage.pose.pose.position.x = garbage_x;
  info.garbage.pose.pose.position.y = garbage_y;

  if (info.goals.size() < 2) {
    info.invalid_reason = "goals size < 2";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(),
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
    return info;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  // 投影线起点取机器人前方的原路径点，已插入的 G/E 不参与
  std::size_t nearest_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  bool have_path_goal = false;
  for (std::size_t i = 0; i < info.goals.size(); ++i) {
    const double px = info.goals[i].pose.position.x;
    const double py = info.goals[i].pose.position.y;
    if (isProtectedGarbageXy(px, py)) {
      continue;
    }
    const double d2 = squaredDistanceXY(px, py, rx, ry);
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest_idx = i;
      have_path_goal = true;
    }
  }
  if (!have_path_goal) {
    info.invalid_reason = "no original path goal for A-C";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(),
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
    return info;
  }
  info.goala = info.goals[nearest_idx];
 // 给垃圾点和机器人点各起一对短名字，写起来方便
  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;

  // 半径 R
  info.radius_m = std::sqrt(squaredDistanceXY(rx, ry, gx, gy));
  if (info.radius_m < 1e-6) {
    info.invalid_reason = "radius ~ 0";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(), gx, gy);
    return info;
  }

  // 这段是在定投影线的终点 goalc   在队头起约 10m 路径里找第一个角点
  if (!findFirstCornerFromRobot(info.goals, rx, ry, info.goalc_idx)) {
    if (!findLastGoalWithinPathRange(
        info.goals, rx, ry, goaltotal_range_m_, info.goalc_idx))
    {
      info.invalid_reason = "no goalc fallback within range";
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
        info.invalid_reason.c_str(), gx, gy);
      return info;
    }
    RCLCPP_DEBUG(
      node_->get_logger(),
      "InsertGarbagePose: no corner, use last goal within %.2fm as goalc (idx %zu)",
      goaltotal_range_m_, info.goalc_idx);
  }
  info.goalc = info.goals[info.goalc_idx];

  // 这段是在处理一个异常：goala 和 goalc 几乎是同一个点，没法定投影直线    A、C 撞成同一个点时，强制换一个更后面的点当 C
  if (squaredDistanceXY(
      info.goala.pose.position.x, info.goala.pose.position.y,
      info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
  {
    std::size_t next_c = 0;
    if (findNextCornerAfter(info.goals, 0, rx, ry, next_c)) {
      info.goalc_idx = next_c;
    } else if (findLastGoalWithinPathRange(
        info.goals, rx, ry, goaltotal_range_m_, next_c) &&
      next_c > 0)
    {
      info.goalc_idx = next_c;
    } else if (info.goals.size() >= 2) {
      info.goalc_idx = 1;
    } else {
      info.invalid_reason = "goala and goalc too close";
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
        info.invalid_reason.c_str(), gx, gy);
      return info;
    }
    info.goalc = info.goals[info.goalc_idx];
    if (squaredDistanceXY(
        info.goala.pose.position.x, info.goala.pose.position.y,
        info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
    {
      info.invalid_reason = "goala and goalc too close";
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
        info.invalid_reason.c_str(), gx, gy);
      return info;
    }
  }

  // goald：垃圾投影到 goala-goalc 无限直线的垂足
  projectPointToInfiniteLine(
    gx, gy,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    info.goald_x, info.goald_y);

  // 插入朝向 / 默认伸 E：首堆用当前车；其后用上一堆到达点
  double from_x = rx;
  double from_y = ry;
  const char * yaw_src = "robot->G";
  if (has_last_sweep_arrive_) {
    from_x = last_sweep_arrive_xy_.first;
    from_y = last_sweep_arrive_xy_.second;
    yaw_src = "arrive->G";
  }
  const double min_from_m = std::max(kMinExtendFromDistM, arrived_radius_);
  const double dx_from = gx - from_x;
  const double dy_from = gy - from_y;
  const double from_dist = std::hypot(dx_from, dy_from);
  if (from_dist < min_from_m) {
    // 车已在 G 上：真车不能当来向。沿车头在 G 后方虚设 from，E 只向前
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double back_m = std::max(std::fabs(garbage_extend_m_), min_from_m * 2.0);
    from_x = gx - back_m * std::cos(robot_yaw);
    from_y = gy - back_m * std::sin(robot_yaw);
    info.path_yaw = robot_yaw;
    yaw_src = "on-G, forward=robot_yaw";
  } else {
    info.path_yaw = std::atan2(dy_from, dx_from);
  }
  info.extend_from_x = from_x;
  info.extend_from_y = from_y;
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: path_yaw=%.3f rad (%s) from=(%.2f, %.2f) garbage=(%.2f, %.2f)",
    info.path_yaw, yaw_src, from_x, from_y, gx, gy);

  info.valid = true;
  return info;
}
//  重新检查被保护的角点还是不是角点
void InsertGarbagePose::refreshCornersOnRemaining(
  const Goals & goals,
  double robot_x, double robot_y,
  std::set<std::size_t> & delete_idx,
  std::set<std::size_t> & protected_corners,
  std::vector<std::pair<double, double>> & corners_kept_xy,
  const std::size_t * keep_idx,
  const std::size_t * also_keep_idx) const
{

  std::vector<std::size_t> remaining_idx;
  remaining_idx.reserve(goals.size());
  //先收集还没删除的点
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (delete_idx.count(i) == 0) {
      remaining_idx.push_back(i);
    }
  }
  if (remaining_idx.size() < 2) {
    return;
  }

  Goals remaining_pts;
  remaining_pts.reserve(remaining_idx.size());
  std::map<std::size_t, std::size_t> old_to_new;
  for (std::size_t new_i = 0; new_i < remaining_idx.size(); ++new_i) {
    const std::size_t old_i = remaining_idx[new_i];
    remaining_pts.push_back(goals[old_i]);
    old_to_new[old_i] = new_i;     // 删点后路径变短，角点判定要用新序列里的下标
  }

  auto is_kept = [&](std::size_t idx) {
    return (keep_idx != nullptr && idx == *keep_idx) ||
           (also_keep_idx != nullptr && idx == *also_keep_idx);
  };
  // 边遍历边删的 iterator 写法
  std::vector<std::size_t> stale;
  for (auto it = protected_corners.begin(); it != protected_corners.end(); ) {
    const std::size_t old_i = *it;
    if (delete_idx.count(old_i) != 0) {
      it = protected_corners.erase(it);
      continue;
    }
    if (is_kept(old_i)) {
      ++it;
      continue;
    }
    const auto map_it = old_to_new.find(old_i);
    if (map_it == old_to_new.end()) {
      it = protected_corners.erase(it);
      continue;
    }
    // 判定“过期角点”
    const std::size_t new_i = map_it->second;
    const bool lost_inbound = (old_i == 0) || (delete_idx.count(old_i - 1) != 0);
    const bool became_head = (new_i == 0);
    // 队首 + 原前驱已断 + 后方还有要保留的角点 → 过期肘点
    // 当前 A/C 已在 is_kept，不会误伤下一轮投影边
    if (became_head && lost_inbound && keep_idx != nullptr &&
      delete_idx.count(*keep_idx) == 0)
    {
      stale.push_back(old_i);
      ++it;
      continue;
    }
    if (isGoalNotCorner(remaining_pts, new_i, robot_x, robot_y)) {
      stale.push_back(old_i);
    }
    ++it;
  }

  constexpr double kMatchTol = 0.08;
  constexpr double kMatchTol2 = kMatchTol * kMatchTol;
  //  真正删 + 擦可视化坐标
  for (const std::size_t old_i : stale) {
    protected_corners.erase(old_i);
    delete_idx.insert(old_i);
    const double px = goals[old_i].pose.position.x;
    const double py = goals[old_i].pose.position.y;
    corners_kept_xy.erase(
      std::remove_if(
        corners_kept_xy.begin(), corners_kept_xy.end(),
        [px, py](const std::pair<double, double> & c) {
          const double dx = c.first - px;
          const double dy = c.second - py;
          return dx * dx + dy * dy < kMatchTol2;
        }),
      corners_kept_xy.end());
    RCLCPP_DEBUG(
      node_->get_logger(),
      "InsertGarbagePose: stale corner idx=%zu (%.2f, %.2f) deleted on remaining",
      old_i, px, py);
  }
}

// 按角点链从队头往后有序删点；角点保留。不按机器人欧氏最近段跳删
InsertGarbagePose::Goals InsertGarbagePose::clipGoalsNearGarbage(InsertInfo & info)
{
  const Goals & goals = info.goals;
  info.goaltotal.clear();
  info.clip_rounds.clear();
  if (goals.size() < 2) {
    return goals;
  }

  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;
  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;

  // 各 goal 折线弧长
  std::vector<double> arc_s(goals.size(), 0.0);
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
    arc_s[i + 1] = arc_s[i] + std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
  }

  // goalc：优先用 gather 已算的；否则从队头找角点，再否则 range 内末点
  std::size_t c_idx = info.goalc_idx;
  if (c_idx >= goals.size()) {
    if (!findFirstCornerFromRobot(goals, rx, ry, c_idx)) {
      if (!findLastGoalWithinPathRange(goals, rx, ry, goaltotal_range_m_, c_idx)) {
        return goals;
      }
    }
  }

  // 机器人到 A-C 直线的垂直距离超过阈值时不删点
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  double ac_foot_x = 0.0;
  double ac_foot_y = 0.0;
  projectPointToInfiniteLine(
    rx, ry,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    ac_foot_x, ac_foot_y);
  const double dist_to_ac_line = std::sqrt(squaredDistanceXY(
    rx, ry, ac_foot_x, ac_foot_y));
  if (dist_to_ac_line > head_delete_robot_dist_m_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: robot %.2fm from A-C line > %.2fm, skip all deletes",
      dist_to_ac_line, head_delete_robot_dist_m_);
    // 远队头也记一轮 A-C，方便 RViz 看投影
    InsertInfo::ClipRound far_round;
    far_round.round_i = 1;
    far_round.ax = info.goala.pose.position.x;
    far_round.ay = info.goala.pose.position.y;
    far_round.cx = info.goalc.pose.position.x;
    far_round.cy = info.goalc.pose.position.y;
    far_round.fx = info.goald_x;
    far_round.fy = info.goald_y;
    far_round.t_d = lineParameterT(
      info.goald_x, info.goald_y,
      far_round.ax, far_round.ay, far_round.cx, far_round.cy);
    info.clip_rounds.push_back(far_round);
    return goals;
  }

  std::set<std::size_t> delete_idx;
  std::set<std::size_t> protected_corners;
  protected_corners.insert(c_idx);
  info.hit_mid_case = false;
  info.hit_forward_case = false;
  info.corners_kept_xy.clear();

  auto markBetween = [&](std::size_t left_idx, std::size_t right_c_idx, bool left_is_head) {
    const std::size_t begin = left_is_head ? left_idx : (left_idx + 1);
    for (std::size_t i = begin; i < right_c_idx; ++i) {
      if (protected_corners.count(i) == 0) {
        delete_idx.insert(i);
      }
    }
  };

  // 垂足在有序路径 [0, c_idx] 上的弧长
  auto footArcOnPathPrefix = [&](double px, double py, std::size_t path_end_idx) -> double {
    std::size_t seg = 0;
    double t = 0.0;
    double best = std::numeric_limits<double>::infinity();
    const std::size_t last_seg = std::min(path_end_idx, goals.size() - 1);
    for (std::size_t i = 0; i < last_seg; ++i) {
      double ti = 0.0;
      const double d2 = squaredDistancePointToSegment(
        px, py,
        goals[i].pose.position.x, goals[i].pose.position.y,
        goals[i + 1].pose.position.x, goals[i + 1].pose.position.y,
        &ti);
      if (d2 < best) {
        best = d2;
        seg = i;
        t = ti;
      }
    }
    return arc_s[seg] + t * (arc_s[seg + 1] - arc_s[seg]);
  };

  bool first_round = true;
  std::size_t a_idx = 0;
  double ax = info.goala.pose.position.x;
  double ay = info.goala.pose.position.y;
  double dx = info.goald_x;
  double dy = info.goald_y;
  int round_i = 0;

  constexpr double kEps = 1e-6;
  while (true) {
    double cx = goals[c_idx].pose.position.x;
    double cy = goals[c_idx].pose.position.y;
    double t_d = lineParameterT(dx, dy, ax, ay, cx, cy);

    // 非首轮且仍是前方延长线：先刷新过期角点；须同时保住当前 A，避免投影边被掐短后误判 reverse
    if (!first_round && t_d > 1.0 + kEps) {
      refreshCornersOnRemaining(
        goals, rx, ry, delete_idx, protected_corners, info.corners_kept_xy,
        &c_idx, &a_idx);
      if (delete_idx.count(a_idx) != 0) {
        std::size_t new_a = 0;
        for (std::size_t i = c_idx; i > 0; --i) {
          const std::size_t cand = i - 1;
          if (delete_idx.count(cand) == 0) {
            new_a = cand;
            break;
          }
        }
        a_idx = new_a;
        ax = goals[a_idx].pose.position.x;
        ay = goals[a_idx].pose.position.y;
        projectPointToInfiniteLine(
          gx, gy, ax, ay, cx, cy, dx, dy);
        t_d = lineParameterT(dx, dy, ax, ay, cx, cy);
        RCLCPP_DEBUG(
          node_->get_logger(),
          "InsertGarbagePose: A was stale, rebase A idx=%zu t=%.2f", a_idx, t_d);
      }
    }

    ++round_i;
    InsertInfo::ClipRound clip_round;
    clip_round.round_i = round_i;
    clip_round.ax = ax;
    clip_round.ay = ay;
    clip_round.cx = cx;
    clip_round.cy = cy;
    clip_round.fx = dx;
    clip_round.fy = dy;
    clip_round.t_d = t_d;
    info.clip_rounds.push_back(clip_round);

    // 反向延长线：不删
    if (t_d < -kEps) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: clip reverse t=%.3f, skip deletes A=(%.2f, %.2f) C=(%.2f, %.2f) F=(%.2f, %.2f)",
        t_d, ax, ay, cx, cy, dx, dy);
      break;
    }

    // 前方延长线：删 A–C 中间，保留角点，刷新后继续找下一角
    if (t_d > 1.0 + kEps) {
      info.hit_forward_case = true;
      if (first_round) {
        markBetween(0, c_idx, true);
      } else {
        if (protected_corners.count(a_idx) == 0 && delete_idx.count(a_idx) == 0) {
          delete_idx.insert(a_idx);
        }
        markBetween(a_idx, c_idx, false);
      }
      info.corners_kept_xy.emplace_back(cx, cy);
      protected_corners.insert(c_idx);

      refreshCornersOnRemaining(
        goals, rx, ry, delete_idx, protected_corners, info.corners_kept_xy, &c_idx);

      std::size_t next_c = 0;
      if (!findNextCornerAfter(goals, c_idx, rx, ry, next_c)) {
        // 无下一角：从当前 C 起沿路 goaltotal_range 内末点兜底再判一轮
        std::size_t range_end = c_idx;
        double accumulated = 0.0;
        for (std::size_t i = c_idx; i + 1 < goals.size(); ++i) {
          accumulated += std::sqrt(squaredDistanceXY(
            goals[i].pose.position.x, goals[i].pose.position.y,
            goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
          if (accumulated > goaltotal_range_m_) {
            break;
          }
          range_end = i + 1;
        }
        if (range_end <= c_idx) {
          refreshCornersOnRemaining(
            goals, rx, ry, delete_idx, protected_corners, info.corners_kept_xy, &c_idx);
          break;
        }
        next_c = range_end;
        RCLCPP_DEBUG(
          node_->get_logger(),
          "InsertGarbagePose: no next corner, fallback last-in-range idx=%zu (%.2f, %.2f)",
          next_c,
          goals[next_c].pose.position.x, goals[next_c].pose.position.y);
      }

      if (delete_idx.count(c_idx) == 0) {
        a_idx = c_idx;
      } else {
        a_idx = 0;
        for (std::size_t i = next_c; i > 0; --i) {
          const std::size_t cand = i - 1;
          if (delete_idx.count(cand) == 0) {
            a_idx = cand;
            break;
          }
        }
        RCLCPP_DEBUG(
          node_->get_logger(),
          "InsertGarbagePose: prev C deleted as stale, new A idx=%zu", a_idx);
      }

      c_idx = next_c;
      protected_corners.insert(c_idx);
      ax = goals[a_idx].pose.position.x;
      ay = goals[a_idx].pose.position.y;
      projectPointToInfiniteLine(
        gx, gy, ax, ay,
        goals[c_idx].pose.position.x, goals[c_idx].pose.position.y,
        dx, dy);
      first_round = false;
      continue;
    }

    // 垂足在段中：从队头往后删到垂足+clip_extend，或删到角点前
    info.hit_mid_case = true;
    const double s_d = footArcOnPathPrefix(dx, dy, c_idx);
    const double s_c = arc_s[c_idx];
    const double path_to_corner = s_c - s_d;

    double s_hi = s_c;
    bool include_hi = false;
    if (clip_extend_m_ > 0.0 && path_to_corner > clip_extend_m_) {
      s_hi = s_d + clip_extend_m_;
      include_hi = true;
    }

    const std::size_t begin = first_round ? 0 : a_idx;
    for (std::size_t j = begin; j < goals.size(); ++j) {
      if (j == begin && !first_round) {
        if (protected_corners.count(j) != 0) {
          continue;
        }
        delete_idx.insert(j);
        continue;
      }
      if (include_hi) {
        if (arc_s[j] > s_hi + 1e-9) {
          break;
        }
      } else if (arc_s[j] >= s_hi - 1e-9) {
        break;
      }
      if (protected_corners.count(j) != 0) {
        continue;
      }
      if (!isGoalNotCorner(goals, j, rx, ry)) {
        protected_corners.insert(j);
        continue;
      }
      delete_idx.insert(j);
    }
    // 段中删完也刷新过期角点
    // 往往已不是角点，但仍在 protected，不刷就会漏删
    refreshCornersOnRemaining(
      goals, rx, ry, delete_idx, protected_corners, info.corners_kept_xy, &c_idx);
    break;
  }

  // 保护已插入的垃圾点，不被删
  for (auto it = delete_idx.begin(); it != delete_idx.end(); ) {
    if (isProtectedGarbageXy(
        goals[*it].pose.position.x, goals[*it].pose.position.y)) {
      it = delete_idx.erase(it);
    } else {
      ++it;
    }
  }

  // 待删点写入 goaltotal，再从 goals 删除
  info.goaltotal.clear();
  info.goaltotal.reserve(delete_idx.size());
  for (const std::size_t idx : delete_idx) {
    info.goaltotal.push_back(goals[idx]);
  }

  Goals out;
  out.reserve(goals.size() - delete_idx.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (delete_idx.count(i) == 0) {
      out.push_back(goals[i]);
    }
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "InsertGarbagePose: clip batch-delete %zu goals, remain %zu / %zu",
    info.goaltotal.size(), out.size(), goals.size());

  return out;
}


// 插入真实垃圾、统一时间戳；
InsertGarbagePose::Goals InsertGarbagePose::insertGarbageIntoGoals(InsertInfo & info)
{
  Goals prefix;
  Goals path;
  int last_protected = -1;
  for (std::size_t i = 0; i < info.goals.size(); ++i) {
    if (isProtectedGarbageXy(
        info.goals[i].pose.position.x, info.goals[i].pose.position.y))
    {
      last_protected = static_cast<int>(i);
    }
  }
  if (last_protected >= 0) {
    prefix.assign(
      info.goals.begin(),
      info.goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1);
    path.assign(
      info.goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1,
      info.goals.end());
  } else {
    path = info.goals;
  }

  const double saved_yaw = info.path_yaw;
  const auto saved_garbage = info.garbage;
  Goals out;
  if (!prefix.empty() && path.size() >= 2) {
    InsertInfo path_info = gatherInsertInfo(
      path, info.robot_pose,
      saved_garbage.pose.pose.position.x,
      saved_garbage.pose.pose.position.y);
    if (path_info.valid) {
      path_info.path_yaw = saved_yaw;
      path_info.garbage = saved_garbage;
      out = clipGoalsNearGarbage(path_info);
      info.goala = path_info.goala;
      info.goalc = path_info.goalc;
      info.goalc_idx = path_info.goalc_idx;
      info.goald_x = path_info.goald_x;
      info.goald_y = path_info.goald_y;
      info.hit_mid_case = path_info.hit_mid_case;
      info.hit_forward_case = path_info.hit_forward_case;
      info.corners_kept_xy = std::move(path_info.corners_kept_xy);
      info.clip_rounds = std::move(path_info.clip_rounds);
      info.goaltotal = std::move(path_info.goaltotal);
    } else {
      out = std::move(path);
    }
  } else if (path.size() >= 2) {
    out = clipGoalsNearGarbage(info);
  } else {
    out = std::move(path);
  }
  info.path_yaw = saved_yaw;
  info.garbage = saved_garbage;

  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  double ac_foot_x = 0.0;
  double ac_foot_y = 0.0;
  projectPointToInfiniteLine(
    info.robot_pose.pose.position.x, info.robot_pose.pose.position.y,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    ac_foot_x, ac_foot_y);
  const double dist_to_ac_line = std::sqrt(squaredDistanceXY(
    info.robot_pose.pose.position.x, info.robot_pose.pose.position.y,
    ac_foot_x, ac_foot_y));
  const bool far_from_head = (dist_to_ac_line > head_delete_robot_dist_m_);


  std::size_t insert_anchor = 0;
  constexpr double kMatchTol = 0.08;
  constexpr double kMatchTol2 = kMatchTol * kMatchTol;
  bool have_insert_after = false;
  if (far_from_head) {
    insert_anchor = 0;
  } else if (info.hit_mid_case) {
    const double ax = info.goala.pose.position.x;
    const double ay = info.goala.pose.position.y;
    for (std::size_t j = 0; j < out.size(); ++j) {
      const double dx = out[j].pose.position.x - ax;
      const double dy = out[j].pose.position.y - ay;
      if (dx * dx + dy * dy < kMatchTol2) {
        insert_anchor = j + 1;
        have_insert_after = true;
        break;
      }
    }
  } else if (info.hit_forward_case && !info.corners_kept_xy.empty()) {
    for (auto it = info.corners_kept_xy.rbegin(); it != info.corners_kept_xy.rend(); ++it) {
      for (std::size_t j = 0; j < out.size(); ++j) {
        const double dx = out[j].pose.position.x - it->first;
        const double dy = out[j].pose.position.y - it->second;
        if (dx * dx + dy * dy < kMatchTol2) {
          insert_anchor = j + 1;
          have_insert_after = true;
          break;
        }
      }
      if (insert_anchor != 0) {
        break;
      }
    }
  }

  std::size_t resume_from = insert_anchor;
  // 保留最后角点作接回，只丢掉角点之前的残留
  if (info.hit_forward_case && have_insert_after && insert_anchor > 0) {
    resume_from = insert_anchor - 1;
  }

  if (resume_from > out.size()) {
    resume_from = out.size();
  }
  if (resume_from > 0) {
    info.goaltotal.insert(
      info.goaltotal.end(), out.begin(), out.begin() + static_cast<std::ptrdiff_t>(resume_from));
  }

  geometry_msgs::msg::PoseStamped garbage_pose = info.garbage.pose;
  if (garbage_pose.header.frame_id.empty() && !out.empty()) {
    garbage_pose.header.frame_id = out.front().header.frame_id;
  } else if (garbage_pose.header.frame_id.empty()) {
    garbage_pose.header.frame_id = global_frame_;
  }
  garbage_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
  // G/E 写入 goals 时用本节点约定的哨兵 z，供下游识别
  garbage_pose.pose.position.z = kGarbageSentinelPoseZ;

  // 沿 path_yaw 插 E：通则用假设到达点→G；不通则沿墙垂线、只留离该到达点更远的一侧
  const double extend_param = garbage_extend_m_;
  double extend_m = 0.0;
  bool add_extend = false;
  geometry_msgs::msg::PoseStamped extend_pose = garbage_pose;
  const double gx = garbage_pose.pose.position.x;
  const double gy = garbage_pose.pose.position.y;
  const double from_x = info.extend_from_x;
  const double from_y = info.extend_from_y;

  auto setExtendPose = [&](double yaw, double d) {
    extend_pose.pose.position.x = gx + d * std::cos(yaw);
    extend_pose.pose.position.y = gy + d * std::sin(yaw);
    extend_pose.pose.position.z = kGarbageSentinelPoseZ;
  };

  auto corridorClear = [&](std::string * reason) {
    return isStraightCorridorClear(
      gx, gy, extend_pose.pose.position.x, extend_pose.pose.position.y, reason);
  };

  auto applyExtendYaw = [&](double yaw) {
    info.path_yaw = yaw;
    garbage_pose.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    extend_pose.pose.orientation = garbage_pose.pose.orientation;
  };

  if (extend_param < -1e-9) {
    extend_m = extend_param;
    setExtendPose(info.path_yaw, extend_m);
    add_extend = true;
  } else if (extend_param > 1e-9) {
    extend_m = extend_param;
    setExtendPose(info.path_yaw, extend_m);
    const int pile_num = (info.dist_label > 0) ?
      info.dist_label :
      (viz_pile_count_ + 1);
    auto skipE = [&](const std::string & why) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip E%d (%.2f, %.2f), 通不过: %s，无法生成",
        pile_num,
        extend_pose.pose.position.x, extend_pose.pose.position.y,
        why.c_str());
    };
    std::string extend_reason;
    if (corridorClear(&extend_reason)) {
      add_extend = true;
    } else {
      double px = 0.0;
      double py = 0.0;
      if (!findNearestObstaclePixel(gx, gy, &px, &py)) {
        skipE("默认G->E走廊(" + extend_reason + ")，且无最近障碍P");
      } else {
        {
          bool already = false;
          for (auto & xy : viz_obstacle_pixels_) {
            if (squaredDistanceXY(xy.x, xy.y, px, py) < 0.04) {
              xy.pile_num = pile_num;
              already = true;
              break;
            }
          }
          if (!already) {
            viz_obstacle_pixels_.push_back(
              VizObstaclePixel{px, py, pile_num});
          }
        }
        const double nx = px - gx;
        const double ny = py - gy;
        const double nlen = std::hypot(nx, ny);
        if (nlen < 1e-6) {
          skipE("P与G重合");
        } else {
          const double tx = -ny / nlen;
          const double ty = nx / nlen;
          const double ex1 = gx + extend_m * tx;
          const double ey1 = gy + extend_m * ty;
          const double ex2 = gx - extend_m * tx;
          const double ey2 = gy - extend_m * ty;
          const double d1 = std::hypot(ex1 - from_x, ey1 - from_y);
          const double d2 = std::hypot(ex2 - from_x, ey2 - from_y);
          const double fwd_x = std::cos(info.path_yaw);
          const double fwd_y = std::sin(info.path_yaw);
          const double from_g = std::hypot(gx - from_x, gy - from_y);
          bool use_plus = d1 > d2;
          // from 贴着 G，或两侧几乎一样远：按扫向选前侧，避免 4cm 噪声翻面
          if (from_g < std::max(kMinExtendFromDistM, arrived_radius_) ||
            std::fabs(d1 - d2) < 0.3)
          {
            use_plus = (tx * fwd_x + ty * fwd_y) >= 0.0;
          }

          const double yaw = use_plus ? std::atan2(ty, tx) : std::atan2(-ty, -tx);
          setExtendPose(yaw, extend_m);
          std::string side_reason;
          if (corridorClear(&side_reason)) {
            applyExtendYaw(yaw);
            add_extend = true;
            RCLCPP_INFO(
              node_->get_logger(),
              "InsertGarbagePose: extend wall-tangent after G->E blocked (%s) "
              "P=(%.2f, %.2f) E=(%.2f, %.2f) from=(%.2f, %.2f)",
              extend_reason.c_str(), px, py,
              extend_pose.pose.position.x, extend_pose.pose.position.y,
              from_x, from_y);
          } else {
            // 切向 0° 不通：只绕这一侧 ±15/±30/±45° 小范围再找 E
            for (double step = kExtendYawSweepStepDeg;
              step <= kExtendYawSweepMaxDeg + 1e-6 && !add_extend;
              step += kExtendYawSweepStepDeg)
            {
              for (const double sign : {1.0, -1.0}) {
                const double yaw_try = yaw + sign * step * M_PI / 180.0;
                setExtendPose(yaw_try, extend_m);
                std::string sweep_reason;
                if (!corridorClear(&sweep_reason)) {
                  continue;
                }
                applyExtendYaw(yaw_try);
                add_extend = true;
                RCLCPP_INFO(
                  node_->get_logger(),
                  "InsertGarbagePose: extend wall-tangent sweep %+g deg after blocked (%s) "
                  "P=(%.2f, %.2f) E=(%.2f, %.2f) from=(%.2f, %.2f)",
                  sign * step, side_reason.c_str(), px, py,
                  extend_pose.pose.position.x, extend_pose.pose.position.y,
                  from_x, from_y);
                break;
              }
            }
            if (!add_extend) {
              skipE("远端墙切向走廊(" + side_reason + ")，±45deg 仍不通");
            }
          }
        }
      }
    }
  }

  info.extend_inserted = add_extend;
  info.extend_used_m = add_extend ? extend_m : 0.0;
  if (add_extend) {
    info.extend_x = extend_pose.pose.position.x;
    info.extend_y = extend_pose.pose.position.y;
    last_sweep_arrive_xy_ = {info.extend_x, info.extend_y};
    addProtectedGarbageXy(info.extend_x, info.extend_y);
  } else {
    last_sweep_arrive_xy_ = {gx, gy};
  }
  has_last_sweep_arrive_ = true;

  Goals suffix;
  suffix.assign(
    out.begin() + static_cast<std::ptrdiff_t>(resume_from), out.end());

  // 已生成 E：用 E 再剔接回段；先保护本堆 G
  if (add_extend && !far_from_head) {
    addProtectedGarbageXy(
      garbage_pose.pose.position.x, garbage_pose.pose.position.y);
    if (suffix.size() >= 2) {
      InsertInfo e_info = gatherInsertInfo(
        suffix, info.robot_pose,
        extend_pose.pose.position.x, extend_pose.pose.position.y);
      if (e_info.valid) {
        const std::size_t before = suffix.size();
        suffix = clipGoalsNearGarbage(e_info);
        info.goaltotal.insert(
          info.goaltotal.end(), e_info.goaltotal.begin(), e_info.goaltotal.end());
        info.clip_rounds.insert(
          info.clip_rounds.end(), e_info.clip_rounds.begin(), e_info.clip_rounds.end());
        double e_t = 0.0;
        if (!e_info.clip_rounds.empty()) {
          e_t = e_info.clip_rounds.front().t_d;
        }
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: E-clip extra delete %zu, suffix %zu -> %zu "
          "t=%.3f mid=%d forward=%d E=(%.2f, %.2f) F=(%.2f, %.2f)",
          before - suffix.size(), before, suffix.size(),
          e_t, e_info.hit_mid_case ? 1 : 0, e_info.hit_forward_case ? 1 : 0,
          extend_pose.pose.position.x, extend_pose.pose.position.y,
          e_info.goald_x, e_info.goald_y);
      }
    } else if (suffix.size() == 1) {
      const double px = suffix.front().pose.position.x;
      const double py = suffix.front().pose.position.y;
      if (!isProtectedGarbageXy(px, py)) {
        const double along =
          (px - extend_pose.pose.position.x) * std::cos(info.path_yaw) +
          (py - extend_pose.pose.position.y) * std::sin(info.path_yaw);
        if (along < clip_extend_m_ + 1e-9) {
          info.goaltotal.push_back(suffix.front());
          suffix.clear();
        }
      }
    }
  }

  // 往 rebuilt 写入 G 与可选 E：d>0 为 G→E，d<0 为 E→G
  auto push_garbage_and_extend = [&](Goals & rebuilt) {
    if (!add_extend) {
      rebuilt.push_back(garbage_pose);
    } else if (extend_m > 0.0) {
      rebuilt.push_back(garbage_pose);
      rebuilt.push_back(extend_pose);
    } else {
      rebuilt.push_back(extend_pose);
      rebuilt.push_back(garbage_pose);
    }
  };

  Goals rebuilt;
  const std::size_t extra = add_extend ? 2u : 1u;
  rebuilt.reserve(prefix.size() + extra + suffix.size());
  rebuilt.insert(rebuilt.end(), prefix.begin(), prefix.end());
  push_garbage_and_extend(rebuilt);
  rebuilt.insert(rebuilt.end(), suffix.begin(), suffix.end());
  out = std::move(rebuilt);

  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : out) {
    pose.header.stamp = stamp_now;
  }
  mission_stamp_record_ = stamp_now;
  has_mission_stamp_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: head-insert resume_from=%zu remain=%zu "
    "(mid=%d forward=%d far=%d)",
    resume_from, out.size(),
    info.hit_mid_case ? 1 : 0, info.hit_forward_case ? 1 : 0, far_from_head ? 1 : 0);

  return out;
}

// 新任务：清空本话题上全部 Marker
void InsertGarbagePose::clearMissionVisualization()
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = global_frame_;
  clear.header.stamp = node_->now();
  clear.ns = "";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  marker_pub_->publish(arr);
}

void InsertGarbagePose::publishFootprintStrippedMarkers()
{
  if (!marker_pub_ || footprint_stripped_viz_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();
  constexpr double kRingR = 0.16;

  for (std::size_t i = 0; i < footprint_stripped_viz_.size(); ++i) {
    const auto & pt = footprint_stripped_viz_[i];

    visualization_msgs::msg::Marker ring;
    ring.header.frame_id = global_frame_;
    ring.header.stamp = stamp;
    ring.ns = "footprint_stripped";
    ring.id = static_cast<int>(i * 2);
    ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring.action = visualization_msgs::msg::Marker::ADD;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = 0.025;
    ring.color.r = 0.05f;
    ring.color.g = 0.05f;
    ring.color.b = 0.05f;
    ring.color.a = 0.95f;
    ring.lifetime = rclcpp::Duration::from_seconds(0.0);
    constexpr int kSegments = 36;
    for (int seg = 0; seg <= kSegments; ++seg) {
      const double ang = 2.0 * M_PI * static_cast<double>(seg) / static_cast<double>(kSegments);
      geometry_msgs::msg::Point p;
      p.x = pt.x + kRingR * std::cos(ang);
      p.y = pt.y + kRingR * std::sin(ang);
      p.z = 0.05;
      ring.points.push_back(p);
    }
    arr.markers.push_back(ring);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = global_frame_;
    text.header.stamp = stamp;
    text.ns = "footprint_stripped";
    text.id = static_cast<int>(i * 2 + 1);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = pt.x;
    text.pose.position.y = pt.y;
    text.pose.position.z = 0.35;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.22;
    text.color.r = 0.05f;
    text.color.g = 0.05f;
    text.color.b = 0.05f;
    text.color.a = 0.95f;
    text.lifetime = rclcpp::Duration::from_seconds(0.0);
    text.text = "fp " + pt.label;
    arr.markers.push_back(text);
  }

  marker_pub_->publish(arr);
}

void InsertGarbagePose::publishWorkCircle()
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return;
  }
  publishRangeCircles(robot_pose.pose.position.x, robot_pose.pose.position.y);
}

void InsertGarbagePose::publishRangeCircles(double robot_x, double robot_y)
{
  if (!marker_pub_) {
    return;
  }
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("work_circle_radius_m", work_circle_radius_m_);

  auto make_circle = [this](
    const std::string & ns, double cx, double cy, double radius,
    float r, float g, float b, float a, double width)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = node_->now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    constexpr int n = 72;
    m.points.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      geometry_msgs::msg::Point p;
      p.x = cx + radius * std::cos(ang);
      p.y = cy + radius * std::sin(ang);
      p.z = 0.05;
      m.points.push_back(p);
    }
    return m;
  };

  visualization_msgs::msg::MarkerArray arr;
  if (max_garbage_robot_dist_m_ > 0.0) {
    arr.markers.push_back(
      make_circle(
        "detect_range", robot_x, robot_y, max_garbage_robot_dist_m_,
        0.55f, 0.95f, 0.50f, 0.90f, 0.05));
  }
  if (has_work_circle_ && work_circle_radius_m_ > 0.0) {
    arr.markers.push_back(
      make_circle(
        "work_circle", work_circle_x_, work_circle_y_, work_circle_radius_m_,
        0.02f, 0.40f, 0.10f, 0.95f, 0.08));
  }
  const double cell = std::max(0.12, viz_obstacle_cell_m_);
  constexpr int kObstacleTextIdBase = 1000;
  for (std::size_t i = 0; i < viz_obstacle_pixels_.size(); ++i) {
    const auto & obs = viz_obstacle_pixels_[i];
    visualization_msgs::msg::Marker box;
    box.header.frame_id = global_frame_;
    box.header.stamp = node_->now();
    box.ns = "nearest_obstacle";
    box.id = static_cast<int>(i);
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = obs.x;
    box.pose.position.y = obs.y;
    box.pose.position.z = 0.08;
    box.pose.orientation.w = 1.0;
    box.scale.x = cell;
    box.scale.y = cell;
    box.scale.z = 0.04;
    box.color.r = 0.95f;
    box.color.g = 0.12f;
    box.color.b = 0.10f;
    box.color.a = 0.95f;
    box.lifetime = rclcpp::Duration::from_seconds(0.0);
    arr.markers.push_back(box);

    visualization_msgs::msg::Marker text;
    text.header = box.header;
    text.ns = "nearest_obstacle";
    text.id = kObstacleTextIdBase + static_cast<int>(i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = obs.x;
    text.pose.position.y = obs.y;
    text.pose.position.z = 0.38;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.22;
    {
      std::ostringstream oss;
      const int n = (obs.pile_num > 0) ? obs.pile_num : static_cast<int>(i + 1);
      oss << "P" << n;
      text.text = oss.str();
    }
    text.color.r = 0.95f;
    text.color.g = 0.12f;
    text.color.b = 0.10f;
    text.color.a = 1.0f;
    text.lifetime = rclcpp::Duration::from_seconds(0.0);
    arr.markers.push_back(text);
  }
  for (std::size_t i = viz_obstacle_pixels_.size(); i < viz_obstacle_marker_count_; ++i) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = global_frame_;
    del.header.stamp = node_->now();
    del.ns = "nearest_obstacle";
    del.id = static_cast<int>(i);
    del.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(del);

    visualization_msgs::msg::Marker del_text = del;
    del_text.id = kObstacleTextIdBase + static_cast<int>(i);
    arr.markers.push_back(del_text);
  }
  viz_obstacle_marker_count_ = viz_obstacle_pixels_.size();
  if (!arr.markers.empty()) {
    marker_pub_->publish(arr);
  }
}

void InsertGarbagePose::clearWorkCircle()
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = node_->now();
  m.ns = "work_circle";
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETE;
  arr.markers.push_back(m);
  marker_pub_->publish(arr);
}

// 往 RViz 发本次插入的证据 Marker

void InsertGarbagePose::publishVisualization(
  const InsertInfo & info,
  bool enable,
  bool viz_accepted_garbage,
  bool viz_deleted_goals,
  bool viz_ac_points)
{
  if (!marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();

  const int pile_idx = viz_pile_count_;
  // 文字编号优先用离机器人远近：G1=最近；未设则回退插入序号
  const int pile_num = (info.dist_label > 0) ? info.dist_label : (pile_idx + 1);

  auto makeBase = [&](const std::string & ns, int id, int type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    return m;
  };

  if (!enable) {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = global_frame_;
    clear.header.stamp = stamp;
    clear.ns = "";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    marker_pub_->publish(arr);
    return;
  }

  auto setColor = [](visualization_msgs::msg::Marker & m,
      float r, float g, float b, float a = 1.0f) {
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
  };

  auto pushPoint = [](visualization_msgs::msg::Marker & m, double x, double y, double z = 0.05) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    m.points.push_back(p);
  };

  auto appendRing = [&](
      visualization_msgs::msg::Marker & m,
      double cx, double cy, double radius, int n = 36)
  {
    for (int i = 0; i <= n; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      pushPoint(m, cx + radius * std::cos(ang), cy + radius * std::sin(ang));
    }
  };

  constexpr int kDeletedIdBase = 6000;
  constexpr int kDeletedIdSpan = 64;

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;

  if (viz_accepted_garbage) {
    // 每堆预留 6 个 id：G 三个 + E 三个，避免后一堆盖掉前一堆
    const int base = pile_idx * 6;
    auto m = makeBase("accepted_garbage", base, visualization_msgs::msg::Marker::SPHERE);
    m.pose.position.x = gx;
    m.pose.position.y = gy;
    m.pose.position.z = 0.12;
    m.scale.x = 0.28;
    m.scale.y = 0.28;
    m.scale.z = 0.28;
    setColor(m, 0.85f, 0.12f, 0.12f, 0.95f);
    arr.markers.push_back(m);

    auto t = makeBase("accepted_garbage", base + 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    t.pose.position.x = gx;
    t.pose.position.y = gy;
    t.pose.position.z = 0.40;
    t.scale.z = 0.22;
    {
      std::ostringstream oss;
      oss << "G" << pile_num;
      t.text = oss.str();
    }
    setColor(t, 0.85f, 0.12f, 0.12f);
    arr.markers.push_back(t);

    auto arrow = makeBase("accepted_garbage", base + 2, visualization_msgs::msg::Marker::ARROW);
    arrow.pose.position.x = gx;
    arrow.pose.position.y = gy;
    arrow.pose.position.z = 0.12;
    arrow.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
    arrow.scale.x = 0.45;
    arrow.scale.y = 0.07;
    arrow.scale.z = 0.07;
    setColor(arrow, 1.00f, 0.35f, 0.05f, 0.95f);
    arr.markers.push_back(arrow);

    if (info.extend_inserted) {
      const double ex = info.extend_x;
      const double ey = info.extend_y;

      auto me = makeBase("accepted_garbage", base + 3, visualization_msgs::msg::Marker::SPHERE);
      me.pose.position.x = ex;
      me.pose.position.y = ey;
      me.pose.position.z = 0.12;
      me.scale.x = 0.28;
      me.scale.y = 0.28;
      me.scale.z = 0.28;
      setColor(me, 0.85f, 0.12f, 0.12f, 0.95f);
      arr.markers.push_back(me);

      auto te = makeBase("accepted_garbage", base + 4, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      te.pose.position.x = ex;
      te.pose.position.y = ey;
      te.pose.position.z = 0.40;
      te.scale.z = 0.22;
      {
        std::ostringstream oss;
        oss << "E" << pile_num;
        te.text = oss.str();
      }
      setColor(te, 0.85f, 0.12f, 0.12f);
      arr.markers.push_back(te);

      auto arrown = makeBase("accepted_garbage", base + 5, visualization_msgs::msg::Marker::ARROW);
      arrown.pose.position.x = ex;
      arrown.pose.position.y = ey;
      arrown.pose.position.z = 0.12;
      arrown.pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
      arrown.scale.x = 0.45;
      arrown.scale.y = 0.07;
      arrown.scale.z = 0.07;
      setColor(arrown, 1.00f, 0.35f, 0.05f, 0.95f);
      arr.markers.push_back(arrown);
    }
  }

  if (viz_deleted_goals) {
    const double ring_r = 0.16;
    int di = 0;
    for (const auto & g : info.goaltotal) {
      if (di >= kDeletedIdSpan) {
        break;
      }
      auto m = makeBase(
        "deleted_goals", kDeletedIdBase + pile_idx * kDeletedIdSpan + di,
        visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.025;
      setColor(m, 0.05f, 0.05f, 0.05f, 0.95f);
      appendRing(m, g.pose.position.x, g.pose.position.y, ring_r);
      arr.markers.push_back(m);
      ++di;
    }
  }

  constexpr int kAcPointsIdBase = 1000;
  constexpr int kRoundsPerPile = 8;
  constexpr int kIdsPerAcRound = 4;

  for (const auto & rnd : info.clip_rounds) {
    const bool first = (rnd.round_i == 1);
    const float lr = first ? 0.90f : 0.00f;
    const float lg = first ? 0.15f : 0.75f;
    const float lb = first ? 0.10f : 0.80f;

    const double vx = rnd.cx - rnd.ax;
    const double vy = rnd.cy - rnd.ay;
    const double L = std::sqrt(vx * vx + vy * vy) + 1e-9;
    const double ux = vx / L;
    const double uy = vy / L;
    constexpr double kLabelOff = 0.50;

    const int round_i0 = std::max(0, rnd.round_i - 1);

    if (viz_ac_points && round_i0 < kRoundsPerPile) {
      const int pbase =
        kAcPointsIdBase +
        pile_idx * (kRoundsPerPile * kIdsPerAcRound) +
        round_i0 * kIdsPerAcRound;

      auto ma = makeBase("ac_points", pbase, visualization_msgs::msg::Marker::CUBE);
      ma.pose.position.x = rnd.ax;
      ma.pose.position.y = rnd.ay;
      ma.pose.position.z = 0.08;
      ma.scale.x = 0.18;
      ma.scale.y = 0.18;
      ma.scale.z = 0.08;
      setColor(ma, lr, lg, lb, 0.95f);
      arr.markers.push_back(ma);

      auto ta = makeBase("ac_points", pbase + 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      ta.pose.position.x = rnd.ax - uy * kLabelOff;
      ta.pose.position.y = rnd.ay + ux * kLabelOff;
      ta.pose.position.z = 0.45;
      ta.scale.z = 0.28;
      {
        std::ostringstream oss;
        oss << "A" << pile_num;
        if (info.clip_rounds.size() > 1) {
          oss << "." << rnd.round_i;
        }
        ta.text = oss.str();
      }
      setColor(ta, lr, lg, lb);
      arr.markers.push_back(ta);

      auto mc = makeBase("ac_points", pbase + 2, visualization_msgs::msg::Marker::SPHERE);
      mc.pose.position.x = rnd.cx;
      mc.pose.position.y = rnd.cy;
      mc.pose.position.z = 0.10;
      mc.scale.x = 0.22;
      mc.scale.y = 0.22;
      mc.scale.z = 0.22;
      setColor(mc, lr, lg, lb, 0.95f);
      arr.markers.push_back(mc);

      auto tc = makeBase("ac_points", pbase + 3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      tc.pose.position.x = rnd.cx + uy * kLabelOff;
      tc.pose.position.y = rnd.cy - ux * kLabelOff;
      tc.pose.position.z = 0.45;
      tc.scale.z = 0.28;
      {
        std::ostringstream oss;
        oss << "C" << pile_num;
        if (info.clip_rounds.size() > 1) {
          oss << "." << rnd.round_i;
        }
        tc.text = oss.str();
      }
      setColor(tc, lr, lg, lb);
      arr.markers.push_back(tc);
    }
  }

  marker_pub_->publish(arr);
  ++viz_pile_count_;
}

// 行为树周期回调
BT::NodeStatus InsertGarbagePose::tick()
{
  // 连续 tick 只打一次；
  {
    static rclcpp::Time last_tick_time{0, 0, RCL_ROS_TIME};
    static bool has_tick_time = false;
    const rclcpp::Time now = node_->now();
    const bool resumed = !has_tick_time || (now - last_tick_time).seconds() > 1.0;
    if (resumed) {
      RCLCPP_INFO(node_->get_logger(), "InsertGarbagePose: tick");
    }
    last_tick_time = now;
    has_tick_time = true;
  }

  callback_group_executor_.spin_some();
  checkAndResetOnNewMission();

  const GarbageList before = garbage_list_;
  postProcessHistory();
  Goals goals_now = receiveGoals();
  // 仅在真正改动了 goals
  bool goals_dirty = false;

  if (goals_now.size() < 2) {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (nav2_util::getCurrentPose(
        robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
      publishRangeCircles(robot_pose.pose.position.x, robot_pose.pose.position.y);
    }
    publishProtectedGarbage();
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    publishProtectedGarbage();
    return BT::NodeStatus::SUCCESS;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  publishRangeCircles(rx, ry);

  std::vector<geometry_msgs::msg::Point> footprint_map;
  const bool have_fp = getRobotFootprintInMap(footprint_map);
  {
    std::string deleted_summary;
    const std::size_t stripped_n = stripReachedZNeg1Goals(
      goals_now, footprint_map, rx, ry, robot_yaw, &deleted_summary);
    if (stripped_n > 0) {
      const rclcpp::Time now_stamp = node_->now();
      for (auto & g : goals_now) {
        g.header.stamp = now_stamp;
      }
      mission_stamp_record_ = now_stamp;
      has_mission_stamp_ = true;
      goals_dirty = true;
    }
  }

  // 排查：active 堆是否还在 {goals}、footprint 是否已到、z=-1 还剩几个
  {
    std::size_t z_neg1_n = 0;
    std::ostringstream z_neg1_oss;
    for (const auto & g : goals_now) {
      if (isUnindexedSentinelPoseZ(g)) {
        if (z_neg1_n > 0) {
          z_neg1_oss << " ";
        }
        z_neg1_oss << "(" << g.pose.position.x << "," << g.pose.position.y << ")";
        ++z_neg1_n;
      }
    }
    if (!active_piles_.empty() || z_neg1_n > 0) {
      std::ostringstream active_oss;
      for (std::size_t i = 0; i < active_piles_.size(); ++i) {
        const double ax = active_piles_[i].pose.pose.position.x;
        const double ay = active_piles_[i].pose.pose.position.y;
        const double dist = std::sqrt(squaredDistanceXY(rx, ry, ax, ay));
        bool in_goals = false;
        for (const auto & g : goals_now) {
          if (!isUnindexedSentinelPoseZ(g)) {
            continue;
          }
          if (squaredDistanceXY(
              g.pose.position.x, g.pose.position.y, ax, ay) <
            kDedupDistanceM * kDedupDistanceM)
          {
            in_goals = true;
            break;
          }
        }
        bool fp_arrived = false;
        if (have_fp) {
          capella_ros_msg::msg::GarbageDetect tmp = active_piles_[i];
          fp_arrived = shouldStopInsertingGarbage(
            tmp, footprint_map, arrived_radius_, rx, ry, robot_yaw);
        }
        if (i > 0) {
          active_oss << " | ";
        }
        const int g_num = lookupStableGNum(ax, ay);
        active_oss << "G" << (g_num > 0 ? g_num : 0)
                   << "(" << ax << "," << ay << ") dist=" << dist
                   << " in_goals=" << (in_goals ? 1 : 0)
                   << " fp_arrived=" << (fp_arrived ? 1 : 0);
      }
      std::ostringstream prot_oss;
      for (std::size_t i = 0; i < reached_garbage_xy_.size(); ++i) {
        if (i > 0) {
          prot_oss << " ";
        }
        prot_oss << "(" << reached_garbage_xy_[i].first << ","
                 << reached_garbage_xy_[i].second << ")";
      }
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: diag status robot=(%.2f, %.2f) yaw=%.3f goals=%zu "
        "z=-1_n=%zu %s | active_n=%zu %s | protected_n=%zu %s | "
        "fp=%s arrived_r=%.2f",
        rx, ry, robot_yaw, goals_now.size(),
        z_neg1_n, z_neg1_oss.str().c_str(),
        active_piles_.size(), active_oss.str().c_str(),
        reached_garbage_xy_.size(), prot_oss.str().c_str(),
        have_fp ? "ok" : "none", arrived_radius_);
    }
  }

  auto logSweepOrder = [this, rx, ry, robot_yaw]() {
    if (garbage_list_.empty()) {
      return;
    }
    std::vector<std::size_t> by_dist(garbage_list_.size());
    for (std::size_t i = 0; i < by_dist.size(); ++i) {
      by_dist[i] = i;
    }
    std::sort(
      by_dist.begin(), by_dist.end(),
      [this, rx, ry](std::size_t a, std::size_t b) {
        return squaredDistanceXY(
          garbage_list_[a].pose.pose.position.x,
          garbage_list_[a].pose.pose.position.y, rx, ry) <
               squaredDistanceXY(
          garbage_list_[b].pose.pose.position.x,
          garbage_list_[b].pose.pose.position.y, rx, ry);
      });
    std::vector<int> dist_label(garbage_list_.size(), 0);
    for (std::size_t r = 0; r < by_dist.size(); ++r) {
      dist_label[by_dist[r]] = static_cast<int>(r + 1);
    }

    auto labelOfIdx = [&](std::size_t idx) -> int {
      if (idx >= garbage_list_.size()) {
        return 0;
      }
      return dist_label[idx];
    };
    auto formatOrder = [&](const std::vector<std::size_t> & idxs) {
      std::ostringstream oss;
      for (std::size_t i = 0; i < idxs.size(); ++i) {
        if (i > 0) {
          oss << "->";
        }
        oss << labelOfIdx(idxs[i]);
      }
      return oss.str();
    };

    std::vector<std::size_t> final_idxs(garbage_list_.size());
    for (std::size_t i = 0; i < final_idxs.size(); ++i) {
      final_idxs[i] = i;
    }
    const auto turn_idxs = findOrderMinTurn(garbage_list_, rx, ry, robot_yaw);
    const auto dist_idxs = findOrderMinDist(garbage_list_, rx, ry);

    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 一共 %zu 堆垃圾进入排序",
      garbage_list_.size());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 排序转角顺序 %s",
      formatOrder(turn_idxs).c_str());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 排序路程顺序 %s",
      formatOrder(dist_idxs).c_str());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 最终清扫顺序 %s",
      formatOrder(final_idxs).c_str());
  };

  // 判断插入的导航点被删除的
  {
    const double thresh2 = kDedupDistanceM * kDedupDistanceM;
    for (auto it = active_piles_.begin(); it != active_piles_.end(); ) {
      const double ax = it->pose.pose.position.x;
      const double ay = it->pose.pose.position.y;
      bool found = false;
      for (const auto & g : goals_now) {
        if (!isUnindexedSentinelPoseZ(g)) {
          continue;
        }
        if (squaredDistanceXY(
            g.pose.position.x, g.pose.position.y, ax, ay) < thresh2)
        {
          found = true;
          break;
        }
      }
      if (found) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
          "InsertGarbagePose: diag active keep (%.2f, %.2f): still in {goals} "
          "(will re-insert on next mid-mission pile)",
          ax, ay);
        ++it;
      } else {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: diag active drop (%.2f, %.2f): gone from {goals} "
          "(treated as swept for mid-mission)",
          ax, ay);
        it = active_piles_.erase(it);
      }
    }
  }

  if (has_work_circle_ && garbage_list_.empty() && active_piles_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "InsertGarbagePose: 工作圈取消");
    clearWorkCircle();
    has_work_circle_ = false;
  }

  bypass_pending_insert_ = false;
  std::size_t new_idx = 0;
  const bool have_new_pile = findNewGarbageIndex(before, rx, ry, new_idx);
  const bool mid_mission_new = have_new_pile && !active_piles_.empty();
  if (mid_mission_new) {
    // 把还没扫完的已插堆抽出来，和新堆一起按 4-1/4-2 重排后再放回
    std::ostringstream old_active_oss;
    for (std::size_t i = 0; i < active_piles_.size(); ++i) {
      if (i > 0) {
        old_active_oss << " ";
      }
      old_active_oss << "(" << active_piles_[i].pose.pose.position.x << ","
                     << active_piles_[i].pose.pose.position.y << ")";
    }
    std::ostringstream new_list_oss;
    for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
      if (i > 0) {
        new_list_oss << " ";
      }
      new_list_oss << "(" << garbage_list_[i].pose.pose.position.x << ","
                   << garbage_list_[i].pose.pose.position.y << ")";
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: diag mid-mission BEGIN goals=%zu active_n=%zu %s "
      "new_list_n=%zu %s protected_n=%zu (will peel protected from goals, "
      "clear protected, recombine active+new)",
      goals_now.size(), active_piles_.size(), old_active_oss.str().c_str(),
      garbage_list_.size(), new_list_oss.str().c_str(),
      reached_garbage_xy_.size());

    Goals kept_goals;
    kept_goals.reserve(goals_now.size());
    std::size_t peeled_n = 0;
    for (const auto & g : goals_now) {
      if (!isProtectedGarbageXy(g.pose.position.x, g.pose.position.y)) {
        kept_goals.push_back(g);
      } else {
        ++peeled_n;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: diag mid-mission peel from goals (%.2f, %.2f) z=%.1f",
          g.pose.position.x, g.pose.position.y, g.pose.position.z);
      }
    }
    goals_now = std::move(kept_goals);
    reached_garbage_xy_.clear();
    has_last_sweep_arrive_ = false;
    last_sweep_arrive_xy_ = {0.0, 0.0};

    last_sweep_xy_.clear();
    last_sweep_xy_.reserve(active_piles_.size());
    for (const auto & g : active_piles_) {
      last_sweep_xy_.emplace_back(
        g.pose.pose.position.x, g.pose.pose.position.y);
    }

    GarbageList combined = active_piles_;
    for (const auto & g : garbage_list_) {
      if (!isDuplicateOfKept(g, combined)) {
        combined.push_back(g);
      }
    }
    {
      std::ostringstream comb_oss;
      for (std::size_t i = 0; i < combined.size(); ++i) {
        if (i > 0) {
          comb_oss << " ";
        }
        comb_oss << "(" << combined[i].pose.pose.position.x << ","
                 << combined[i].pose.pose.position.y << ")";
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: diag mid-mission recombine peeled=%zu kept_goals=%zu "
        "combined_n=%zu %s (OLD piles in combined will be inserted again)",
        peeled_n, goals_now.size(), combined.size(), comb_oss.str().c_str());
    }
    garbage_list_ = std::move(combined);
    active_piles_.clear();

    GarbageList old_active;
    old_active.reserve(last_sweep_xy_.size());
    for (const auto & xy : last_sweep_xy_) {
      capella_ros_msg::msg::GarbageDetect dummy;
      dummy.pose.pose.position.x = xy.first;
      dummy.pose.pose.position.y = xy.second;
      old_active.push_back(dummy);
    }
    if (findNewGarbageIndex(old_active, rx, ry, new_idx)) {
      bypass_pending_insert_ = reorderGarbageListWithNewPile(
        rx, ry, robot_yaw, new_idx);
    } else if (garbage_list_.size() > 1) {
      reorderGarbageListBySweep(rx, ry, robot_yaw);
    }
    clearMissionVisualization();
    viz_pile_count_ = 0;
    has_last_viz_time_ = false;
    publishFootprintStrippedMarkers();
    logSweepOrder();
  } else if (garbage_list_.size() > 1 && last_sweep_xy_.empty()) {
    reorderGarbageListBySweep(rx, ry, robot_yaw);
    logSweepOrder();
  } else if (garbage_list_.size() == 1 && last_sweep_xy_.empty()) {
    syncLastSweepXyFromList();
  }

  if (garbage_list_.empty()) {
    if (goals_dirty) {
      emitOutputGoals(goals_now, "strip_z_neg1");
    }
    publishProtectedGarbage();
    return BT::NodeStatus::SUCCESS;
  }

  bool enable_viz = true;
  bool viz_garbage = true;
  bool viz_deleted = true;
  bool viz_ac_pts = true;
  getInput("enable_visualization", enable_viz);
  getInput("viz_accepted_garbage", viz_garbage);
  getInput("viz_deleted_goals", viz_deleted);
  getInput("viz_ac_points", viz_ac_pts);

  // 按当前顺序一次插入全部待插堆
  std::size_t inserted_count = 0;
  std::ostringstream inserted_xy;
  while (!garbage_list_.empty()) {
    const double gx = garbage_list_.front().pose.pose.position.x;
    const double gy = garbage_list_.front().pose.pose.position.y;
    if (isNearReachedGarbage(gx, gy)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: diag skip insert (%.2f, %.2f): near protected/reached",
        gx, gy);
      garbage_list_.erase(garbage_list_.begin());
      continue;
    }
    InsertInfo info = gatherInsertInfo(goals_now, robot_pose, gx, gy);
    if (!info.valid) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: diag skip insert (%.2f, %.2f): gather invalid (%s)",
        gx, gy, info.invalid_reason.c_str());
      garbage_list_.erase(garbage_list_.begin());
      continue;
    }
    info.dist_label = assignStableGNum(gx, gy);
    goals_now = insertGarbageIntoGoals(info);
    const rclcpp::Time viz_now = node_->now();
    if (has_last_viz_time_ &&
      (viz_now - last_viz_time_).seconds() > kVizTaskWindowSec)
    {
      clearMissionVisualization();
      viz_pile_count_ = 0;
      publishFootprintStrippedMarkers();
    }
    publishVisualization(info, enable_viz, viz_garbage, viz_deleted, viz_ac_pts);
    publishRangeCircles(rx, ry);
    last_viz_time_ = viz_now;
    has_last_viz_time_ = true;
    addProtectedGarbageXy(gx, gy);
    if (info.extend_inserted) {
      addProtectedGarbageXy(info.extend_x, info.extend_y);
    }
    active_piles_.push_back(garbage_list_.front());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: diag active+ G%d (%.2f, %.2f) extend=%d E=(%.2f, %.2f) "
      "active_n=%zu",
      info.dist_label, gx, gy, info.extend_inserted ? 1 : 0,
      info.extend_x, info.extend_y, active_piles_.size());
    garbage_list_.erase(garbage_list_.begin());
    ++inserted_count;
    inserted_xy << "(" << gx << ", " << gy << ") ";
  }

  last_sweep_xy_.clear();
  last_sweep_xy_.reserve(active_piles_.size());
  for (const auto & g : active_piles_) {
    last_sweep_xy_.emplace_back(
      g.pose.pose.position.x, g.pose.pose.position.y);
  }

  if (inserted_count > 0) {
    const rclcpp::Time now_stamp = node_->now();
    std::size_t z_neg1_n = 0;
    for (std::size_t i = 0; i < goals_now.size(); ++i) {
      // G/E 已是哨兵 z，重编号时不要改；其余途经点写序号
      if (!isUnindexedSentinelPoseZ(goals_now[i])) {
        goals_now[i].pose.position.z = static_cast<double>(i);
      } else {
        ++z_neg1_n;
      }
      goals_now[i].header.stamp = now_stamp;
    }
    mission_stamp_record_ = now_stamp;
    has_mission_stamp_ = true;
    goals_dirty = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: diag after-batch goals=%zu z=-1_n=%zu active_n=%zu "
      "protected_n=%zu",
      goals_now.size(), z_neg1_n, active_piles_.size(),
      reached_garbage_xy_.size());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: batch inserted %zu pile(s): %s, output goals %zu",
      inserted_count, inserted_xy.str().c_str(), goals_now.size());
  }

  if (goals_dirty) {
    const char * reason = (inserted_count > 0) ? "batch_insert" : "strip_z_neg1";
    emitOutputGoals(goals_now, reason);
  }
  publishProtectedGarbage();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
