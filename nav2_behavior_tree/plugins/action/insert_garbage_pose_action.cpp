#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
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
  global_costmap_topic_("global_costmap/costmap"),
  visualization_topic_("insert_garbage_pose/markers"),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  arrived_radius_(0.5),       // footprint 进入垃圾附近半径，停止再插入
  clip_extend_m_(2.5),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(30.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0),  // 离队头超过该距离就不删点
  max_garbage_robot_dist_m_(5.0),  // 垃圾离机器人超过该距离则忽略
  min_garbage_obstacle_clearance_m_(0.7),
  wall_edge_d_extend_m_(2.0),
  wall_edge_e_extend_m_(2.0),
  wall_edge_min_robot_dist_m_(3.0),
  wall_edge_sample_m_(0.5),
  wall_edge_normal_offset_m_(0.0),
  garbage_merge_radius_m_(1.0),    // 到种子小于该距离合为一堆
  garbage_extend_m_(2.0),          // 沿扫向相对垃圾再插一点，默认 2.0m；见 GARBAGE_EXTEND_M
  work_circle_radius_m_(10.0)
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("local_costmap_topic", local_costmap_topic_);
  getInput("global_costmap_topic", global_costmap_topic_);
  getInput("visualization_topic", visualization_topic_);
  getInput("arrived_radius", arrived_radius_);
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("min_garbage_obstacle_clearance_m", min_garbage_obstacle_clearance_m_);
  getInput("wall_edge_d_extend_m", wall_edge_d_extend_m_);
  getInput("wall_edge_e_extend_m", wall_edge_e_extend_m_);
  getInput("wall_edge_min_robot_dist_m", wall_edge_min_robot_dist_m_);
  getInput("wall_edge_sample_m", wall_edge_sample_m_);
  getInput("wall_edge_normal_offset_m", wall_edge_normal_offset_m_);
  getInput("garbage_merge_radius_m", garbage_merge_radius_m_);
  getInput("garbage_extend_m", garbage_extend_m_);
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

  rclcpp::QoS global_costmap_qos(rclcpp::KeepLast(1));
  global_costmap_qos.transient_local().reliable();
  global_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_,
    global_costmap_qos,
    std::bind(&InsertGarbagePose::globalCostmapCallback, this, std::placeholders::_1),
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
  if (!transformGarbageToMap(item)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: drop garbage, transform to map failed, wait for next frame");
    return;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!getRobotPoseXY(robot_x, robot_y)) {
    return;
  }

  const double gx = item.pose.pose.position.x;
  const double gy = item.pose.pose.position.y;
  getInput("confirm_match_dist_m", confirm_match_dist_m_);
  const double merge_r = std::max(0.0, garbage_merge_radius_m_);
  const double merge_r2 = merge_r * merge_r;
  // 两帧确认：第二帧落在 confirm_match_dist_m 内才收下
  const double confirm_r = std::max(0.0, confirm_match_dist_m_);
  const double confirm_r2 = confirm_r * confirm_r;

  auto near_xy = [&](double x, double y, double r2) {
    return squaredDistanceXY(gx, gy, x, y) < r2;
  };
    
  for (const auto & reached : reached_garbage_xy_) {
    if (near_xy(reached.first, reached.second, merge_r2)) {
      return;
    }
  }
  for (const auto & kept : garbage_list_) {
    if (near_xy(kept.pose.pose.position.x, kept.pose.pose.position.y, merge_r2)) {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(history_mutex_);
  for (const auto & existing : history_list_) {
    if (near_xy(
        existing.pose.pose.position.x, existing.pose.pose.position.y, merge_r2))
    {
      return;
    }
  }


  // 两帧确认：只出现一帧的先放待确认队列，超时未再匹配就丢掉
  const rclcpp::Time now = node_->now();
  for (auto it = confirm_wait_list_.begin(); it != confirm_wait_list_.end(); ) {
    const auto age = (now - rclcpp::Time(it->pose.header.stamp)).seconds();
    if (age > kConfirmHoldSec) {
      it = confirm_wait_list_.erase(it);
    } else {
      ++it;
    }
  }
  if (confirm_r > 1e-6) {
    std::size_t match_i = confirm_wait_list_.size();
    for (std::size_t i = 0; i < confirm_wait_list_.size(); ++i) {
      if (near_xy(
          confirm_wait_list_[i].pose.pose.position.x,
          confirm_wait_list_[i].pose.pose.position.y, confirm_r2))
      {
        match_i = i;
        break;
      }
    }
    if (match_i >= confirm_wait_list_.size()) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾待确认 坐标=(%.2f, %.2f)", gx, gy);
      item.pose.header.stamp = now;
      confirm_wait_list_.push_back(item);
      return;
    }
    const double px = confirm_wait_list_[match_i].pose.pose.position.x;
    const double py = confirm_wait_list_[match_i].pose.pose.position.y;
    item.pose.pose.position.x = 0.5 * (px + gx);
    item.pose.pose.position.y = 0.5 * (py + gy);
    confirm_wait_list_.erase(
      confirm_wait_list_.begin() + static_cast<std::ptrdiff_t>(match_i));
  }

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

// 加锁取最新 footprint
geometry_msgs::msg::PolygonStamped::SharedPtr
InsertGarbagePose::getFootprintSnapshot(std::string & source_frame) const
{
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    footprint_msg = latest_footprint_;
  }
  if (!footprint_msg || footprint_msg->polygon.points.empty()) {
    return nullptr;
  }
  source_frame = footprint_msg->header.frame_id;
  if (source_frame.empty()) {
    source_frame = robot_base_frame_;
  }
  return footprint_msg;
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

// 加锁取最新局部代价图
nav_msgs::msg::OccupancyGrid::SharedPtr
InsertGarbagePose::getLocalCostmapSnapshot() const
{
  std::lock_guard<std::mutex> lock(local_costmap_mutex_);
  return latest_local_costmap_;
}

void InsertGarbagePose::globalCostmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  latest_global_costmap_ = msg;
}

nav_msgs::msg::OccupancyGrid::SharedPtr
InsertGarbagePose::getGlobalCostmapSnapshot() const
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  return latest_global_costmap_;
}

bool InsertGarbagePose::costmapWorldToIndex(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & costmap,
  double x, double y,
  int * mx, int * my, std::size_t * idx,
  std::string * reason) const
{
  if (!costmap || costmap->info.width == 0 || costmap->info.height == 0 ||
    costmap->data.empty())
  {
    if (reason) {
      *reason = "no costmap received";
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
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  const int grid_x = static_cast<int>(std::floor(local_x / info.resolution));
  const int grid_y = static_cast<int>(std::floor(local_y / info.resolution));
  if (grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(info.width) || grid_y >= static_cast<int>(info.height))
  {
    if (reason) {
      *reason = "outside costmap";
    }
    return false;
  }

  const std::size_t index =
    static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(info.width) +
    static_cast<std::size_t>(grid_x);
  if (index >= costmap->data.size()) {
    if (reason) {
      *reason = "costmap index out of range";
    }
    return false;
  }

  if (mx) {
    *mx = grid_x;
  }
  if (my) {
    *my = grid_y;
  }
  if (idx) {
    *idx = index;
  }
  return true;
}

bool InsertGarbagePose::isMapPointPassableOnGlobalCostmap(
  double x, double y, std::string * reason) const
{
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getGlobalCostmapSnapshot();
  std::size_t idx = 0;
  if (!costmapWorldToIndex(costmap, x, y, nullptr, nullptr, &idx, reason)) {
    return false;
  }
  const int8_t cell = costmap->data[idx];
  if (cell < 0) {
    if (reason) {
      *reason = "costmap cell unknown";
    }
    return false;
  }
  if (cell >= 100) {
    if (reason) {
      *reason = "lethal on global costmap";
    }
    return false;
  }
  return true;
}

bool InsertGarbagePose::isStraightLineClearOnGlobalCostmap(
  double x0, double y0, double x1, double y1,
  double sample_m) const
{
  const double step = std::max(0.05, sample_m);
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);

  if (!isMapPointPassableOnGlobalCostmap(x0, y0)) {
    return false;
  }
  if (len < 1e-9) {
    return true;
  }

  const double ux = dx / len;
  const double uy = dy / len;
  for (double s = step; s < len - 1e-9; s += step) {
    if (!isMapPointPassableOnGlobalCostmap(x0 + ux * s, y0 + uy * s)) {
      return false;
    }
  }
  if (!isMapPointPassableOnGlobalCostmap(x1, y1)) {
    return false;
  }
  return true;
}

bool InsertGarbagePose::isExtendCandidateClear(
  double gx, double gy, double ex, double ey, double yaw,
  std::string * reason) const
{
  std::string local_reason;
  if (isObstacleInfoReadable(ex, ey, &local_reason)) {
    if (!isFootprintClearAtPose(ex, ey, yaw, reason)) {
      return false;
    }
  } else {
    std::string global_reason;
    if (!isMapPointPassableOnGlobalCostmap(ex, ey, &global_reason)) {
      if (reason) {
        *reason = "E outside local (" + local_reason + "), global: " + global_reason;
      }
      return false;
    }
  }
  if (!isStraightLineClearOnGlobalCostmap(gx, gy, ex, ey)) {
    if (reason) {
      *reason = "G->E blocked on global costmap";
    }
    return false;
  }
  return true;
}

// 有没有图    点在图内吗    格子可读吗，是否是栅格值 < 0
bool InsertGarbagePose::isObstacleInfoReadable(
  double x, double y, std::string * reason) const
{
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getLocalCostmapSnapshot();

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
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getLocalCostmapSnapshot();

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

  // 栅格下标越界 
  const int mx = static_cast<int>(std::floor(local_x / info.resolution));
  const int my = static_cast<int>(std::floor(local_y / info.resolution));
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
  {
    return true;
  }

  const std::size_t idx =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
    static_cast<std::size_t>(mx);
  if (idx >= costmap->data.size()) {
    return true;
  }

  // OccupancyGrid 发布后：-1 对应 255，>=100 对应 254；99 对应 253 可通过
  // unknown(-1/255，没有任何信息) 不算障碍：只有真致命格(>=100)才算不可通行
  const int8_t cell = costmap->data[idx];
  if (cell >= 100) {
    return false;
  }
  return true;
}

bool InsertGarbagePose::isStraightLineClearOnLocalCostmap(
  double x0, double y0, double x1, double y1,
  double sample_m) const
{
  // 从起点固定步长朝终点采点；起点必查，终点不强制落在采样上
  const double step = std::max(0.05, sample_m);
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);

  if (!isMapPointPassableOnLocalCostmap(x0, y0)) {
    return false;
  }
  if (len < 1e-9) {
    return true;
  }

  const double ux = dx / len;
  const double uy = dy / len;
  for (double s = step; s < len - 1e-9; s += step) {
    if (!isMapPointPassableOnLocalCostmap(x0 + ux * s, y0 + uy * s)) {
      return false;
    }
  }
  return true;
}

bool InsertGarbagePose::hasObstacleWithinRadius(
  double x, double y, double radius_m) const
{
  if (radius_m <= 0.0 || !tf_) {
    return false;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getLocalCostmapSnapshot();
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

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getLocalCostmapSnapshot();
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
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap = getLocalCostmapSnapshot();
  if (!costmap || costmap->data.empty()) {
    if (reason) {
      *reason = "no local costmap";
    }
    return false;
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

  // 失败时尽量给出真实原因：出窗(outside local costmap) / unknown 等；
  // 只有真致命格(>=100)才用调用方给的 "lethal ..." 兜底文本
  auto describeBlocked = [&](double x, double y, const char * fallback) {
    if (!reason) {
      return;
    }
    std::string detail;
    if (!isObstacleInfoReadable(x, y, &detail)) {
      *reason = detail.empty() ? std::string(fallback) : detail;
    } else {
      *reason = fallback;
    }
  };

  auto cornersClearAt = [&](double cx, double cy, const char * fail_reason) -> bool {
    for (const auto & off : local_xy) {
      const auto q = cornerWorld(cx, cy, off.first, off.second);
      if (!isMapPointPassableOnLocalCostmap(q.first, q.second)) {
        describeBlocked(q.first, q.second, fail_reason);
        return false;
      }
    }
    return true;
  };

  if (len < 1e-6) {
    if (!isMapPointPassableOnLocalCostmap(start_x, start_y)) {
      describeBlocked(start_x, start_y, "lethal at robot");
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
        describeBlocked(qx, qy, "lethal on corridor");
        return false;
      }
    }
    if (!cornersClearAt(cx, cy, "lethal at footprint corner")) {
      return false;
    }
  }

  if (!isMapPointPassableOnLocalCostmap(end_x, end_y)) {
    describeBlocked(end_x, end_y, "lethal at garbage");
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
    logGarbageListState("new garbage insert to list");
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
  logGarbageListState("replace farthest pile");
  return true;
}

// 种子点找离机器人最近；新点须与组内最远成员距离小于半径才并入
InsertGarbagePose::GarbageList InsertGarbagePose::mergeGarbagePiles(
  const GarbageList & candidates,
  double robot_x, double robot_y,
  double merge_radius_m,
  std::vector<std::vector<std::size_t>> * groups_out)
{
  GarbageList merged_garbage_list;
  if (groups_out != nullptr) {
    groups_out->clear();
  }
  if (candidates.empty()) {
    return merged_garbage_list;
  }

  const double radius = std::max(0.0, merge_radius_m);
  const double radius2 = radius * radius;

  auto xyOf = [&candidates](std::size_t idx) {
      return std::make_pair(
        candidates[idx].pose.pose.position.x,
        candidates[idx].pose.pose.position.y);
    };

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
      const auto g = xyOf(remaining[p]);
      const double d2 = squaredDistanceXY(g.first, g.second, robot_x, robot_y);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        seed_pos = p;
      }
    }
    const std::size_t seed_idx = remaining[seed_pos];
    const auto seed_xy = xyOf(seed_idx);

    // 组内成员
    std::vector<std::size_t> members{seed_idx};

    // 其余点按到种子距离升序，逐一判断能否并入
    std::vector<std::size_t> order;
    order.reserve(remaining.size());
    for (const std::size_t idx : remaining) {
      if (idx != seed_idx) {
        order.push_back(idx);
      }
    }
    std::sort(
      order.begin(), order.end(),
      [&](std::size_t a, std::size_t b) {
        const auto ga = xyOf(a);
        const auto gb = xyOf(b);
        return squaredDistanceXY(ga.first, ga.second, seed_xy.first, seed_xy.second) <
               squaredDistanceXY(gb.first, gb.second, seed_xy.first, seed_xy.second);
      });

    for (const std::size_t idx : order) {
      const auto q = xyOf(idx);
      // 与组内每个成员都要够得着
      double max_d2 = 0.0;
      for (const std::size_t m : members) {
        const auto gm = xyOf(m);
        max_d2 = std::max(
          max_d2, squaredDistanceXY(q.first, q.second, gm.first, gm.second));
      }
      if (max_d2 < radius2) {
        members.push_back(idx);
      }
    }
    merged_garbage_list.push_back(candidates[seed_idx]);
    if (groups_out != nullptr) {
      groups_out->push_back(members);
    }

    // 已消化的成员从 remaining 移除
    std::vector<std::size_t> next;
    next.reserve(remaining.size());
    for (const std::size_t idx : remaining) {
      if (std::find(members.begin(), members.end(), idx) == members.end()) {
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
  logGarbageListState("sweep reorder");
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: nearest-first then sweep, next (%.2f, %.2f), n=%zu",
    garbage_list_.front().pose.pose.position.x,
    garbage_list_.front().pose.pose.position.y,
    garbage_list_.size());
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
      reorderNearestFirstThenSweep(robot_x, robot_y, robot_yaw);
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
  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!getRobotPoseXY(robot_x, robot_y)) {
    return garbage_list_;
  }

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

  GarbageList candidates;
  std::vector<capella_ros_msg::msg::GarbageDetect> candidate_originals;
  candidates.reserve(snapshot.size());
  candidate_originals.reserve(snapshot.size());

  // 删掉一组已消化的原始检测
  auto erasePileMembers =
    [&](const std::vector<std::size_t> & member_idx) {
      for (const std::size_t i : member_idx) {
        if (i < candidate_originals.size()) {
          eraseFromHistory(candidate_originals[i]);
        }
      }
    };

  for (const auto & original : snapshot) {
    capella_ros_msg::msg::GarbageDetect garbage = original;

    // 转到 map，失败则留在 history
    if (!transformGarbageToMap(garbage)) {                  
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为转到map失败, 暂不处理",
        original.pose.pose.position.x, original.pose.pose.position.y);
      continue;
    }

    const double gx = garbage.pose.pose.position.x;
    const double gy = garbage.pose.pose.position.y;

    // 落在禁扫区则丢弃
    if (isPointInSpecialTerrain(gx, gy)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为在禁扫区, 丢弃", gx, gy);
      eraseFromHistory(original);
      continue;
    }

    // 离机器人太远：视为误识别，丢弃
    if (max_garbage_robot_dist_m_ > 0.0) {
      const double dist_robot = std::sqrt(squaredDistanceXY(gx, gy, robot_x, robot_y));
      if (dist_robot > max_garbage_robot_dist_m_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
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
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为局部代价图不可读(%s), 丢弃",
          gx, gy, costmap_reason.c_str());
        eraseFromHistory(original);
        continue;
      }
    }

    // 垃圾到机器人连线
    if (!isStraightLineClearOnLocalCostmap(gx, gy, robot_x, robot_y, 0.1)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为到机器人连线有障碍, 丢弃",
        gx, gy);
      eraseFromHistory(original);
      continue;
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
        clearMissionVisualization();
        viz_pile_count_ = 0;
        viz_footprint_fail_count_ = 0;
        publishRangeCircles(robot_x, robot_y);
      }
      if (has_work_circle_) {
        const double d_circle = std::sqrt(squaredDistanceXY(
            gx, gy, work_circle_x_, work_circle_y_));
        if (d_circle > work_circle_radius_m_) {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *(node_->get_clock()), 2000,
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

  // 种子=离车最近；新点须与组内最远成员距离仍 < 半径才并入；代表点仍是种子
  std::vector<std::vector<std::size_t>> groups;
  GarbageList merged_garbage_list = mergeGarbagePiles(
    candidates, robot_x, robot_y, garbage_merge_radius_m_, &groups);

  for (std::size_t gi = 0; gi < merged_garbage_list.size(); ++gi) {
    auto & seed = merged_garbage_list[gi];
    const std::vector<std::size_t> & members = groups[gi];
    const double sx = seed.pose.pose.position.x;
    const double sy = seed.pose.pose.position.y;

    if (members.size() > 1) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 合并 %zu 点, 首点=(%.2f, %.2f)",
        members.size(), sx, sy);
    }

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
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为已到达/已插入过, 丢弃", sx, sy);
      erasePileMembers(members);
      continue;
    }

    if (isDuplicateOfKept(seed, garbage_list_)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为与已规划堆重复, 丢弃", sx, sy);
      erasePileMembers(members);
      continue;
    }

    if (tryInsertPreferCloserToRobot(seed, robot_x, robot_y)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 新发现一堆垃圾, 坐标=(%.2f, %.2f), 进入清扫规划", sx, sy);
      erasePileMembers(members);
    } else {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
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
    return {};
  }
  return goals;
}

// 看当前是不是一次新的导航任务
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
    confirm_wait_list_.clear();
  }
  garbage_list_.clear();
  logGarbageListState("new mission clear");
  active_piles_.clear();
  reached_garbage_xy_.clear();
  viz_obstacle_pixels_.clear();
  viz_obstacle_marker_count_ = 0;
  has_pending_garbage_ = false;
  bypass_pending_insert_ = false;
  last_sweep_xy_.clear();
  has_last_sweep_arrive_ = false;
  last_sweep_arrive_xy_ = {0.0, 0.0};
  has_last_sweep_path_yaw_ = false;
  last_sweep_path_yaw_ = 0.0;
  viz_pile_count_ = 0;
  viz_footprint_fail_count_ = 0;
  g_num_xy_.clear();
  e_num_xy_.clear();
  next_g_num_ = 1;
  has_work_circle_ = false;
  mission_stamp_record_ = current_stamp;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 更新扫地任务");
}

// 取机器人当前位姿
bool InsertGarbagePose::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  if (!tf_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: 获取不到 TF，tf_buffer 为空，无法取机器人位姿");
    return false;
  }
  if (!nav2_util::getCurrentPose(
      pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: 获取不到 TF，%s -> %s 变换失败，无法取机器人位姿",
      robot_base_frame_.c_str(), global_frame_.c_str());
    return false;
  }
  return true;
}

// 取机器人当前 xy 与可选 yaw
bool InsertGarbagePose::getRobotPoseXY(double & x, double & y, double * yaw) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(pose)) {
    return false;
  }
  x = pose.pose.position.x;
  y = pose.pose.position.y;
  if (yaw != nullptr) {
    *yaw = tf2::getYaw(pose.pose.orientation);
  }
  return true;
}

// 获取机器人当前 footprint，并转到 map
bool InsertGarbagePose::getRobotFootprintInMap(
  std::vector<geometry_msgs::msg::Point> & footprint_map) const
{
  footprint_map.clear();
  if (!tf_) {
    return false;
  }

  std::string source_frame;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg =
    getFootprintSnapshot(source_frame);
  if (!footprint_msg) {
    return false;
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

  std::string source_frame;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg =
    getFootprintSnapshot(source_frame);
  if (!footprint_msg) {
    return false;
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

void InsertGarbagePose::registerStableENum(double x, double y, int g_num)
{
  if (g_num <= 0) {
    return;
  }
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (auto & item : e_num_xy_) {
    if (squaredDistanceXY(item.first.first, item.first.second, x, y) < thresh2) {
      item.second = g_num;
      return;
    }
  }
  e_num_xy_.push_back({{x, y}, g_num});
}

int InsertGarbagePose::lookupStableENum(double x, double y) const
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  for (const auto & item : e_num_xy_) {
    if (squaredDistanceXY(item.first.first, item.first.second, x, y) < thresh2) {
      return item.second;
    }
  }
  return 0;
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

// 按下标找这堆自己的槽：该格 z=-1，xy 只确认同一颗，不拿附近别的 -1 冒充
bool InsertGarbagePose::findUnindexedSentinelIndex(
  const Goals & goals, double x, double y, std::size_t * index_out) const
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      continue;
    }
    if (squaredDistanceXY(
        goals[i].pose.position.x, goals[i].pose.position.y, x, y) < thresh2)
    {
      if (index_out != nullptr) {
        *index_out = i;
      }
      return true;
    }
  }
  return false;
}

// 每 tick 扫全部已插堆：还占着自己 z=-1 槽的留下，找不到这格的从 active 去掉
std::size_t InsertGarbagePose::stripReachedZNeg1Goals(
  const Goals & goals,
  std::string * deleted_summary)
{
  if (active_piles_.empty()) {
    return 0;
  }

  GarbageList still;
  still.reserve(active_piles_.size());
  std::size_t n_gone = 0;
  std::ostringstream deleted_oss;

  for (const auto & pile : active_piles_) {
    const double ax = pile.pose.pose.position.x;
    const double ay = pile.pose.pose.position.y;
    const int g_num = lookupStableGNum(ax, ay);
    std::size_t idx = 0;
    if (findUnindexedSentinelIndex(goals, ax, ay, &idx)) {
      still.push_back(pile);
      continue;
    }

    addProtectedGarbageXy(ax, ay);
    if (g_num > 0) {
      for (const auto & item : e_num_xy_) {
        if (item.second == g_num) {
          addProtectedGarbageXy(item.first.first, item.first.second);
        }
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: strip pile G%d (%.2f, %.2f): gone from {goals} "
      "(no own z=-1 slot, xy confirm %.2fm), treated as swept",
      g_num, ax, ay, kSentinelIdentityMatchM);
    if (n_gone > 0) {
      deleted_oss << " ";
    }
    deleted_oss << "G" << g_num << " (" << ax << "," << ay << ")";
    ++n_gone;
  }

  active_piles_ = std::move(still);

  if (!active_piles_.empty()) {
    const double ax = active_piles_.front().pose.pose.position.x;
    const double ay = active_piles_.front().pose.pose.position.y;
    const int g_num = lookupStableGNum(ax, ay);
    std::size_t idx = 0;
    const bool found = findUnindexedSentinelIndex(goals, ax, ay, &idx);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: strip current pile G%d (%.2f, %.2f): still in {goals} "
      "at index %zu z=-1 (xy confirm %.2fm), not swept",
      g_num, ax, ay, found ? idx : static_cast<std::size_t>(-1),
      kSentinelIdentityMatchM);
  }

  if (deleted_summary != nullptr && n_gone > 0) {
    *deleted_summary = deleted_oss.str();
  }
  return n_gone;
}

std::string InsertGarbagePose::formatGoalsListCompact(const Goals & goals) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  for (const auto & g : goals) {
    const double x = g.pose.position.x;
    const double y = g.pose.position.y;
    oss << "(" << x << "," << y;
    if (isUnindexedSentinelPoseZ(g)) {
      oss << ",-1";
    }
    oss << ") ";
  }
  return oss.str();
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

void InsertGarbagePose::eraseProtectedGarbageXy(double x, double y)
{
  const double thresh2 = kDedupDistanceM * kDedupDistanceM;
  auto it = std::remove_if(
    reached_garbage_xy_.begin(), reached_garbage_xy_.end(),
    [x, y, thresh2](const std::pair<double, double> & xy) {
      return squaredDistanceXY(x, y, xy.first, xy.second) < thresh2;
    });
  if (it == reached_garbage_xy_.end()) {
    return;
  }
  reached_garbage_xy_.erase(it, reached_garbage_xy_.end());
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: diag protected- (%.2f, %.2f), protected_n=%zu",
    x, y, reached_garbage_xy_.size());
}

bool InsertGarbagePose::collectInProgressKeepXy(
  const Goals & goals,
  std::vector<std::pair<double, double>> * keep_xy,
  int * keep_g_num) const
{
  if (keep_xy == nullptr || keep_g_num == nullptr) {
    return false;
  }
  keep_xy->clear();
  *keep_g_num = 0;
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;

  auto already = [&](double x, double y) {
    for (const auto & p : *keep_xy) {
      if (squaredDistanceXY(p.first, p.second, x, y) < thresh2) {
        return true;
      }
    }
    return false;
  };
  auto add = [&](double x, double y) {
    if (!already(x, y)) {
      keep_xy->emplace_back(x, y);
    }
  };

  int g_num = 0;
  for (const auto & pile : active_piles_) {
    const double ax = pile.pose.pose.position.x;
    const double ay = pile.pose.pose.position.y;
    if (!findUnindexedSentinelIndex(goals, ax, ay, nullptr)) {
      continue;
    }
    g_num = lookupStableGNum(ax, ay);
    add(ax, ay);
    break;
  }
  if (keep_xy->empty()) {
    for (std::size_t i = 0; i < goals.size(); ++i) {
      if (!isUnindexedSentinelPoseZ(goals[i])) {
        continue;
      }
      const double x = goals[i].pose.position.x;
      const double y = goals[i].pose.position.y;
      g_num = lookupStableGNum(x, y);
      if (g_num <= 0) {
        g_num = lookupStableENum(x, y);
      }
      add(x, y);
      break;
    }
  }
  if (g_num > 0) {
    for (const auto & item : g_num_xy_) {
      if (item.second == g_num) {
        add(item.first.first, item.first.second);
      }
    }
    for (const auto & item : e_num_xy_) {
      if (item.second == g_num) {
        add(item.first.first, item.first.second);
      }
    }
  }
  *keep_g_num = g_num;
  if (keep_xy->empty()) {
    return false;
  }

  double rx = 0.0;
  double ry = 0.0;
  double robot_yaw = 0.0;
  if (!getRobotPoseXY(rx, ry, &robot_yaw)) {
    keep_xy->clear();
    *keep_g_num = 0;
    return false;
  }
  std::vector<geometry_msgs::msg::Point> footprint_map;
  const bool have_fp = getRobotFootprintInMap(footprint_map);
  (void)have_fp;
  const double gx = keep_xy->front().first;
  const double gy = keep_xy->front().second;
  if (!isPileSweepInProgress(gx, gy, goals, footprint_map, rx, ry, robot_yaw)) {
    keep_xy->clear();
    *keep_g_num = 0;
    return false;
  }
  return true;
}

bool InsertGarbagePose::isPileSweepInProgress(
  double gx, double gy,
  const Goals & goals,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double robot_x, double robot_y, double robot_yaw) const
{
  if (!findUnindexedSentinelIndex(goals, gx, gy, nullptr)) {
    return false;
  }
  capella_ros_msg::msg::GarbageDetect tmp;
  tmp.pose.pose.position.x = gx;
  tmp.pose.pose.position.y = gy;
  if (!footprint_map.empty() &&
    shouldStopInsertingGarbage(
      tmp, footprint_map, arrived_radius_, robot_x, robot_y, robot_yaw))
  {
    return true;
  }

  const int g_num = lookupStableGNum(gx, gy);
  if (g_num <= 0) {
    return false;
  }
  for (const auto & item : e_num_xy_) {
    if (item.second != g_num) {
      continue;
    }
    const double ex = item.first.first;
    const double ey = item.first.second;
    if (!findUnindexedSentinelIndex(goals, ex, ey, nullptr)) {
      continue;
    }
    const double dx = ex - gx;
    const double dy = ey - gy;
    if (dx * dx + dy * dy <= 1e-6) {
      continue;
    }
    // 已过 G、正在去 E：机器人在 G 的 E 一侧
    if ((robot_x - gx) * dx + (robot_y - gy) * dy > 0.0) {
      return true;
    }
  }
  return false;
}

bool InsertGarbagePose::isPendingGarbageInGoals(const Goals & goals) const
{
  if (!has_pending_garbage_) {
    return false;
  }
  return findUnindexedSentinelIndex(
    goals, pending_garbage_xy_.first, pending_garbage_xy_.second, nullptr);
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

// true=非角点，false=角点：当前点分别指向前后路径点，夹角偏离 180° 超过阈值则为角点
bool InsertGarbagePose::isGoalNotCorner(
  const Goals & goals,
  std::size_t idx,
  double robot_x, double robot_y) const
{
  (void)robot_x;
  (void)robot_y;
  if (goals.size() < 3 || idx >= goals.size()) {
    return true;
  }
  if (isUnindexedSentinelPoseZ(goals[idx])) {
    return true;
  }

  int prev_i = -1;
  for (int i = static_cast<int>(idx) - 1; i >= 0; --i) {
    if (!isUnindexedSentinelPoseZ(goals[static_cast<std::size_t>(i)])) {
      prev_i = i;
      break;
    }
  }
  std::size_t next_i = goals.size();
  for (std::size_t i = idx + 1; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      next_i = i;
      break;
    }
  }
  if (prev_i < 0 || next_i >= goals.size()) {
    return true;
  }

  const double cx = goals[idx].pose.position.x;
  const double cy = goals[idx].pose.position.y;
  const double to_prev_x = goals[static_cast<std::size_t>(prev_i)].pose.position.x - cx;
  const double to_prev_y = goals[static_cast<std::size_t>(prev_i)].pose.position.y - cy;
  const double to_next_x = goals[next_i].pose.position.x - cx;
  const double to_next_y = goals[next_i].pose.position.y - cy;

  const double len_prev_sq = to_prev_x * to_prev_x + to_prev_y * to_prev_y;
  const double len_next_sq = to_next_x * to_next_x + to_next_y * to_next_y;
  constexpr double kMinSegLenSq = 1e-6;
  if (len_prev_sq < kMinSegLenSq || len_next_sq < kMinSegLenSq) {
    return true;
  }

  // 直线时两方向相反，夹角约 180°；直角约 90°
  const double cos_theta = std::clamp(
    (to_prev_x * to_next_x + to_prev_y * to_next_y) /
    std::sqrt(len_prev_sq * len_next_sq), -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  const double corner_angle_rad = corner_angle_deg_ * M_PI / 180.0;
  return theta >= (M_PI - corner_angle_rad);
}

// 剩余队列第一个非 z=-1 点：工字当前长边队首，不用欧氏最近（对边 0.5~1m 会抢）
std::size_t InsertGarbagePose::ordinaryQueueHead(const Goals & goals) const
{
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      return i;
    }
  }
  return goals.size();
}

// 从当前长边队首往后找第一个角点
bool InsertGarbagePose::findFirstCornerFromRobot(
  const Goals & goals,
  double robot_x, double robot_y,
  std::size_t & corner_idx) const
{
  if (goals.size() < 2) {
    return false;
  }

  const std::size_t head = ordinaryQueueHead(goals);
  if (head >= goals.size()) {
    return false;
  }
  const std::size_t start_idx = head;
  if (start_idx >= goals.size()) {
    return false;
  }

  std::size_t range_end = start_idx;
  double accumulated = 0.0;
  for (std::size_t i = head; i + 1 < goals.size(); ++i) {
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
  (void)robot_x;
  (void)robot_y;

  // 从当前长边队首沿路径量 range，不用欧氏最近段
  std::size_t start_idx = ordinaryQueueHead(goals);
  if (start_idx >= goals.size()) {
    start_idx = 0;
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

double InsertGarbagePose::preferExtendYawAwayFromRobot(
  double gx, double gy, double yaw,
  double robot_x, double robot_y)
{
  const double dx = robot_x - gx;
  const double dy = robot_y - gy;
  if (std::hypot(dx, dy) < 1e-3) {
    return yaw;
  }
  const double fx = std::cos(yaw);
  const double fy = std::sin(yaw);
  // G→E 与 G→车同侧：E 会落在车旁/同向，翻转到对侧
  if (dx * fx + dy * fy > 0.0) {
    return std::atan2(-fy, -fx);
  }
  return yaw;
}

bool InsertGarbagePose::fillAcWhenOriginalPathGone(
  InsertInfo * info, double gx, double gy) const
{
  if (info == nullptr) {
    return false;
  }
  constexpr double kMinAc2 = 1e-12;
  const Goals & goals = info->goals;

  auto accept = [&](
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & c,
    std::size_t c_idx,
    const char * src)
  {
    const double ax = a.pose.position.x;
    const double ay = a.pose.position.y;
    const double cx = c.pose.position.x;
    const double cy = c.pose.position.y;
    if (squaredDistanceXY(ax, ay, cx, cy) < kMinAc2) {
      return false;
    }
    info->goala = a;
    info->goalc = c;
    info->goalc_idx = c_idx;
    info->ac_fallback = true;
    info->invalid_reason.clear();
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: A-C fallback %s A=(%.2f, %.2f) C=(%.2f, %.2f) G=(%.2f, %.2f)",
      src, ax, ay, cx, cy, gx, gy);
    return true;
  };

  // 已有 G-E：用最后一对哨兵当通道线（第一堆把原路径删光后的常态）
  int last_s = -1;
  int prev_s = -1;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (isUnindexedSentinelPoseZ(goals[i])) {
      prev_s = last_s;
      last_s = static_cast<int>(i);
    }
  }
  if (prev_s >= 0 && last_s >= 0 &&
    accept(
      goals[static_cast<std::size_t>(prev_s)],
      goals[static_cast<std::size_t>(last_s)],
      static_cast<std::size_t>(last_s), "G-E"))
  {
    return true;
  }

  // 还剩一个原路径点：车 → 该点
  int orig = -1;
  for (int i = static_cast<int>(goals.size()) - 1; i >= 0; --i) {
    const auto & g = goals[static_cast<std::size_t>(i)];
    if (isUnindexedSentinelPoseZ(g)) {
      continue;
    }
    if (isProtectedGarbageXy(g.pose.position.x, g.pose.position.y)) {
      continue;
    }
    orig = i;
    break;
  }
  if (orig >= 0 &&
    accept(
      info->robot_pose, goals[static_cast<std::size_t>(orig)],
      static_cast<std::size_t>(orig), "robot->path"))
  {
    return true;
  }

  // 原路径和 G-E 都不够：车 → G，只为出 yaw/E，后面不靠这条线删点
  geometry_msgs::msg::PoseStamped g_pose = info->robot_pose;
  g_pose.pose.position.x = gx;
  g_pose.pose.position.y = gy;
  if (accept(info->robot_pose, g_pose, 0, "robot->G")) {
    return true;
  }
  return false;
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
  const double gx = garbage_x;
  const double gy = garbage_y;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  info.radius_m = std::sqrt(squaredDistanceXY(rx, ry, gx, gy));
  if (info.radius_m < 1e-6) {
    info.invalid_reason = "radius ~ 0";
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(), gx, gy);
    return info;
  }

  auto fail_unless_ac_fallback = [&](const char * why) {
    if (fillAcWhenOriginalPathGone(&info, gx, gy)) {
      return false;
    }
    info.invalid_reason = why;
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(), gx, gy);
    return true;
  };

  bool ac_ok = false;
  if (info.goals.size() >= 2) {
    // 投影线起点取剩余队列头（当前长边队首），已插入的 G/E 不参与
    const std::size_t head_idx = ordinaryQueueHead(info.goals);
    if (head_idx < info.goals.size()) {
      info.goala = info.goals[head_idx];
      if (!findFirstCornerFromRobot(info.goals, rx, ry, info.goalc_idx)) {
        if (findLastGoalWithinPathRange(
            info.goals, rx, ry, goaltotal_range_m_, info.goalc_idx))
        {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "InsertGarbagePose: no corner, use last goal within %.2fm as goalc (idx %zu)",
            goaltotal_range_m_, info.goalc_idx);
        } else {
          info.goalc_idx = head_idx;
        }
      }
      if (info.goalc_idx < info.goals.size()) {
        info.goalc = info.goals[info.goalc_idx];
      }
      if (squaredDistanceXY(
          info.goala.pose.position.x, info.goala.pose.position.y,
          info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
      {
        std::size_t next_c = 0;
        if (findNextCornerAfter(info.goals, head_idx, rx, ry, next_c)) {
          info.goalc_idx = next_c;
        } else if (findLastGoalWithinPathRange(
            info.goals, rx, ry, goaltotal_range_m_, next_c) &&
          next_c > head_idx)
        {
          info.goalc_idx = next_c;
        } else if (head_idx + 1 < info.goals.size()) {
          info.goalc_idx = head_idx + 1;
        }
        if (info.goalc_idx < info.goals.size()) {
          info.goalc = info.goals[info.goalc_idx];
        }
      }
      if (squaredDistanceXY(
          info.goala.pose.position.x, info.goala.pose.position.y,
          info.goalc.pose.position.x, info.goalc.pose.position.y) >= 1e-12)
      {
        ac_ok = true;
      }
    }
  }
  if (!ac_ok) {
    const char * why = (info.goals.size() < 2) ?
      "goals size < 2" : "goala and goalc too close";
    if (fail_unless_ac_fallback(why)) {
      return info;
    }
  }

  // goald：垃圾投影到 goala-goalc 无限直线的垂足
  projectPointToInfiniteLine(
    gx, gy,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    info.goald_x, info.goald_y);

  // 插入朝向 / 默认伸 E：后一堆按规划来向，用上一离开点（仍在 goals 里的 G/E），
  // 不用发现时的车位，避免侧向堆的 E 和车头同向。
  double from_x = rx;
  double from_y = ry;
  const char * yaw_src = "robot->G";
  if (has_last_sweep_arrive_ &&
    findUnindexedSentinelIndex(
      info.goals,
      last_sweep_arrive_xy_.first, last_sweep_arrive_xy_.second, nullptr))
  {
    from_x = last_sweep_arrive_xy_.first;
    from_y = last_sweep_arrive_xy_.second;
    yaw_src = "arrive->G";
  } else if (has_last_sweep_arrive_) {
    yaw_src = "robot->G, prev arrive not in goals";
  }
  const double min_from_m = std::max(kMinExtendFromDistM, arrived_radius_);
  const double dx_from = gx - from_x;
  const double dy_from = gy - from_y;
  const double from_dist = std::hypot(dx_from, dy_from);
  const double back_m = std::max(std::fabs(garbage_extend_m_), min_from_m * 2.0);
  if (from_dist < min_from_m) {
    // 来向贴 G：优先延续上一堆扫向，避免退回车头导致后堆 E∥机器人
    double yaw_fwd = tf2::getYaw(robot_pose.pose.orientation);
    if (has_last_sweep_path_yaw_) {
      yaw_fwd = last_sweep_path_yaw_;
      yaw_src = "on-G, forward=last_sweep_yaw";
    } else {
      yaw_src = "on-G, forward=robot_yaw";
    }
    from_x = gx - back_m * std::cos(yaw_fwd);
    from_y = gy - back_m * std::sin(yaw_fwd);
    info.path_yaw = yaw_fwd;
  } else {
    info.path_yaw = std::atan2(dy_from, dx_from);
  }
  {
    const double before = info.path_yaw;
    // 用进近来向 from，不用发现时车位，避免后堆 E 按旧车位翻反
    info.path_yaw = preferExtendYawAwayFromRobot(
      gx, gy, info.path_yaw, from_x, from_y);
    if (std::cos(info.path_yaw - before) < 0.0) {
      // 翻转后保证 from 仍在 G 后方（-path_yaw 侧），墙切向选侧才一致
      from_x = gx - back_m * std::cos(info.path_yaw);
      from_y = gy - back_m * std::sin(info.path_yaw);
      yaw_src = "flipped away from approach";
    }
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
    if (isUnindexedSentinelPoseZ(goals[old_i])) {
      continue;
    }
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

// 按角点链从队头往后有序删点；碰到角点立即停，角点和后面都不删
InsertGarbagePose::Goals InsertGarbagePose::clipGoalsNearGarbage(
  InsertInfo & info, bool skip_reverse)
{
  const Goals & goals = info.goals;
  info.goaltotal.clear();
  info.clip_rounds.clear();
  if (goals.size() < 2) {
    return goals;
  }
  // 原路径不够成 A-C（已改用 G-E / 车→G）：只跳过删点，调用方仍插入 G-E
  if (info.ac_fallback ||
    squaredDistanceXY(
      info.goala.pose.position.x, info.goala.pose.position.y,
      info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: A-C fallback/degenerate, skip deletes, still insert "
      "G=(%.2f, %.2f)",
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
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

  info.corners_kept_xy.clear();
  const std::size_t H = ordinaryQueueHead(goals);
  std::size_t C = goals.size();
  const bool have_corner = findFirstCornerFromRobot(goals, rx, ry, C);
  if (!have_corner || C < H) {
    C = goals.size();
  }
  const std::size_t n_side = (C > H) ? (C - H) : 0;
  const std::size_t n_other = (C < goals.size()) ? (goals.size() - C) : 0;
  const std::size_t c_line = (C < goals.size()) ? C : (goals.size() - 1);
  info.goalc_idx = c_line;

  // 机器人到 A-C 直线的垂直距离超过阈值时不删点
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  double ac_foot_x = 0.0;
  double ac_foot_y = 0.0;
  projectPointToInfiniteLine(
    rx, ry,
    info.goala.pose.position.x, info.goala.pose.position.y,
    goals[c_line].pose.position.x, goals[c_line].pose.position.y,
    ac_foot_x, ac_foot_y);
  const double dist_to_ac_line = std::sqrt(squaredDistanceXY(
    rx, ry, ac_foot_x, ac_foot_y));
  if (dist_to_ac_line > head_delete_robot_dist_m_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: robot %.2fm from A-C line > %.2fm, skip all deletes",
      dist_to_ac_line, head_delete_robot_dist_m_);
    InsertInfo::ClipRound far_round;
    far_round.round_i = 1;
    far_round.ax = info.goala.pose.position.x;
    far_round.ay = info.goala.pose.position.y;
    far_round.cx = goals[c_line].pose.position.x;
    far_round.cy = goals[c_line].pose.position.y;
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
  if (C < goals.size()) {
    protected_corners.insert(C);
    info.corners_kept_xy.emplace_back(
      goals[C].pose.position.x, goals[C].pose.position.y);
  }
  info.hit_mid_case = false;
  info.hit_forward_case = false;

  auto footArcOnCurrentSide = [&](double px, double py) -> double {
    if (H + 1 >= goals.size() || n_side == 0) {
      return 0.0;
    }
    std::size_t seg = H;
    double t = 0.0;
    double best = std::numeric_limits<double>::infinity();
    const std::size_t last_seg = std::min(C > 0 ? C : H + 1, goals.size() - 1);
    for (std::size_t i = H; i < last_seg; ++i) {
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

  const double ax = info.goala.pose.position.x;
  const double ay = info.goala.pose.position.y;
  const double dx = info.goald_x;
  const double dy = info.goald_y;
  const double cx = goals[c_line].pose.position.x;
  const double cy = goals[c_line].pose.position.y;
  const double t_d = lineParameterT(dx, dy, ax, ay, cx, cy);

  InsertInfo::ClipRound clip_round;
  clip_round.round_i = 1;
  clip_round.ax = ax;
  clip_round.ay = ay;
  clip_round.cx = cx;
  clip_round.cy = cy;
  clip_round.fx = dx;
  clip_round.fy = dy;
  clip_round.t_d = t_d;
  info.clip_rounds.push_back(clip_round);

  constexpr double kEps = 1e-6;
  if (n_side == 0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: clip skip, current side empty H=%zu C=%zu n_other=%zu",
      H, C, n_other);
  } else if (t_d < -kEps && !skip_reverse) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: clip reverse t=%.3f, skip deletes A=(%.2f, %.2f) C=(%.2f, %.2f) F=(%.2f, %.2f)",
      t_d, ax, ay, cx, cy, dx, dy);
  } else if (t_d > 1.0 + kEps && !skip_reverse) {
    info.hit_forward_case = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: clip beyond current side t=%.3f, keep corner idx=%zu, skip side wipe",
      t_d, C < goals.size() ? C : c_line);
  } else {
    if (t_d < -kEps && skip_reverse) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: E-clip ignore reverse t=%.3f, clip along current side from foot",
        t_d);
    }
    info.hit_mid_case = true;
    const double s_robot = footArcOnCurrentSide(rx, ry);
    const double s_d = footArcOnCurrentSide(gx, gy);
    const double s_lo = s_robot;
    const double s_hi = std::max(s_lo, s_d + std::max(0.0, clip_extend_m_));
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: clip current-side H=%zu C=%zu n_side=%zu n_other=%zu "
      "s_lo=%.2f s_d=%.2f s_hi=%.2f target=(%.2f, %.2f)",
      H, C, n_side, n_other, s_lo, s_d, s_hi, gx, gy);

    for (std::size_t j = H; j < C && j < goals.size(); ++j) {
      if (arc_s[j] < s_lo - 1e-9) {
        continue;
      }
      if (arc_s[j] > s_hi + 1e-9) {
        break;
      }
      if (isUnindexedSentinelPoseZ(goals[j])) {
        continue;
      }
      if (protected_corners.count(j) != 0 ||
        !isGoalNotCorner(goals, j, rx, ry))
      {
        protected_corners.insert(j);
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: clip stop at corner idx=%zu (%.2f, %.2f)",
          j, goals[j].pose.position.x, goals[j].pose.position.y);
        break;
      }
      delete_idx.insert(j);
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: clip counts H=%zu C=%zu n_side=%zu n_other=%zu n_del=%zu N=%zu",
    H, C, n_side, n_other, delete_idx.size(), goals.size());

  // 保护已插入的垃圾点，以及 z=-1 哨兵 G/E，不被删
  for (auto it = delete_idx.begin(); it != delete_idx.end(); ) {
    if (isProtectedGarbageXy(
        goals[*it].pose.position.x, goals[*it].pose.position.y) ||
      isUnindexedSentinelPoseZ(goals[*it]))
    {
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
        info.goals[i].pose.position.x, info.goals[i].pose.position.y) ||
      isUnindexedSentinelPoseZ(info.goals[i]))
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


  // G-E 接到当前长边剩余队首，不再按欧氏最近 A 丢掉前缀（工字对边会被误删）
  const std::size_t resume_from = 0;

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

  // G 已单独做过 footprint；这里不跑 G→E 整段车宽走廊。
  // E 窗内用局部 footprint；窗外问全局。G→E 用全局细线。
  auto corridorClear = [&](std::string * reason) {
    const double ex = extend_pose.pose.position.x;
    const double ey = extend_pose.pose.position.y;
    const double yaw_e = std::atan2(ey - gy, ex - gx);
    return isExtendCandidateClear(gx, gy, ex, ey, yaw_e, reason);
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
    std::string neg_reason;
    const double yaw_e = std::atan2(
      extend_pose.pose.position.y - gy, extend_pose.pose.position.x - gx);
    if (!isExtendCandidateClear(
        gx, gy, extend_pose.pose.position.x, extend_pose.pose.position.y,
        yaw_e, &neg_reason))
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip E (%.2f, %.2f), 通不过: %s，无法生成",
        extend_pose.pose.position.x, extend_pose.pose.position.y,
        neg_reason.c_str());
      add_extend = false;
    }
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
    // E 点检查不通过时，把当时检查用的 footprint 框画在候选 E 位置上
    auto publishFailedEBox = [&]() {
      const double ex = extend_pose.pose.position.x;
      const double ey = extend_pose.pose.position.y;
      publishFootprintCheckBox(ex, ey, std::atan2(ey - gy, ex - gx));
    };
    std::string extend_reason;
    if (corridorClear(&extend_reason)) {
      add_extend = true;
    } else {
      publishFailedEBox();
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
          const double to_fx = from_x - gx;
          const double to_fy = from_y - gy;
          bool use_plus = d1 > d2;
          // from 贴着 G，或两侧几乎一样远：按扫向选前侧，避免 4cm 噪声翻面
          if (from_g < std::max(kMinExtendFromDistM, arrived_radius_) ||
            std::fabs(d1 - d2) < 0.3)
          {
            use_plus = (tx * fwd_x + ty * fwd_y) >= 0.0;
          }
          // 禁止 E 与进近来向同侧，参照 from 而非发现时车位
          {
            const double sx = use_plus ? tx : -tx;
            const double sy = use_plus ? ty : -ty;
            if (to_fx * sx + to_fy * sy > 0.0) {
              use_plus = !use_plus;
            }
          }

          double yaw = use_plus ? std::atan2(ty, tx) : std::atan2(-ty, -tx);
          yaw = preferExtendYawAwayFromRobot(gx, gy, yaw, from_x, from_y);
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
            // 切向 0° 不通：绕该侧按步长扫到 ±kExtendYawSweepMaxDeg 再找 E。
            // 注意：扫角内不要再 preferExtendYawAwayFromRobot——那会把朝开阔侧
            // （常与来车同侧）的候选翻成朝墙，导致 ±90° 一半扇区从未真正试过。
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
              skipE(
                "远端墙切向走廊(" + side_reason + ")，±" +
                std::to_string(static_cast<int>(kExtendYawSweepMaxDeg)) +
                "deg 仍不通");
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
  last_sweep_path_yaw_ = info.path_yaw;
  has_last_sweep_path_yaw_ = true;

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
        suffix = clipGoalsNearGarbage(e_info, true);
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
      if (!isProtectedGarbageXy(px, py) &&
        !isUnindexedSentinelPoseZ(suffix.front()))
      {
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

bool InsertGarbagePose::isFootprintClearAtPose(
  double x, double y, double yaw, std::string * reason) const
{
  const double ex = x + 0.05 * std::cos(yaw);
  const double ey = y + 0.05 * std::sin(yaw);
  return isStraightCorridorClear(x, y, ex, ey, reason);
}

InsertGarbagePose::WallEdgeExtendChain InsertGarbagePose::buildWallEdgeExtendChain(
  const InsertInfo & info)
{
  WallEdgeExtendChain chain;
  getInput("wall_edge_d_extend_m", wall_edge_d_extend_m_);
  getInput("wall_edge_e_extend_m", wall_edge_e_extend_m_);
  getInput("wall_edge_min_robot_dist_m", wall_edge_min_robot_dist_m_);
  getInput("wall_edge_sample_m", wall_edge_sample_m_);
  getInput("wall_edge_normal_offset_m", wall_edge_normal_offset_m_);

  const double gx0 = info.garbage.pose.pose.position.x;
  const double gy0 = info.garbage.pose.pose.position.y;
  const double from_x = info.extend_from_x;
  const double from_y = info.extend_from_y;

  double px = 0.0;
  double py = 0.0;
  if (!findNearestObstaclePixel(gx0, gy0, &px, &py)) {
    chain.invalid_reason = "no obstacle P";
    return chain;
  }

  double nx = gx0 - px;
  double ny = gy0 - py;
  const double nlen = std::hypot(nx, ny);
  if (nlen < 1e-6) {
    chain.invalid_reason = "P coincides G";
    return chain;
  }
  nx /= nlen;
  ny /= nlen;
  const double tx = -ny;
  const double ty = nx;

  const double extend_m = std::max(0.1, std::fabs(wall_edge_e_extend_m_));
  const double ex1 = gx0 + extend_m * tx;
  const double ey1 = gy0 + extend_m * ty;
  const double ex2 = gx0 - extend_m * tx;
  const double ey2 = gy0 - extend_m * ty;
  const double d1 = std::hypot(ex1 - from_x, ey1 - from_y);
  const double d2 = std::hypot(ex2 - from_x, ey2 - from_y);
  bool e_plus = d1 > d2;
  const double fwd_x = std::cos(info.path_yaw);
  const double fwd_y = std::sin(info.path_yaw);
  if (std::fabs(d1 - d2) < 0.3) {
    e_plus = (tx * fwd_x + ty * fwd_y) >= 0.0;
  }
  double e_tx = e_plus ? tx : -tx;
  double e_ty = e_plus ? ty : -ty;
  double e_yaw = std::atan2(e_ty, e_tx);
  // 贴边 E/D 也只相对进近 from，避免后堆按发现时车位翻反
  e_yaw = preferExtendYawAwayFromRobot(gx0, gy0, e_yaw, from_x, from_y);
  e_tx = std::cos(e_yaw);
  e_ty = std::sin(e_yaw);

  const double off = wall_edge_normal_offset_m_;
  auto footClear = [&](double x, double y, double yaw, std::string * reason) {
    return isFootprintClearAtPose(
      x + off * nx, y + off * ny, yaw, reason);
  };
  // D-E 连线只采点查占用，0.1m 一步，不做 footprint
  auto deLineClear = [&](double x0, double y0, double x1, double y1) {
    return isStraightLineClearOnLocalCostmap(
      x0 + off * nx, y0 + off * ny,
      x1 + off * nx, y1 + off * ny,
      0.1);
  };

  // E：先落点，footprint 不过则只绕当前 e_yaw 扫角，不重选切向左右
  double ex = gx0 + extend_m * e_tx;
  double ey = gy0 + extend_m * e_ty;
  bool e_ok = false;
  {
    std::string e_reason;
    if (footClear(ex, ey, e_yaw, &e_reason)) {
      e_ok = true;
    } else {
      for (double step_deg = kExtendYawSweepStepDeg;
        step_deg <= kExtendYawSweepMaxDeg + 1e-6 && !e_ok;
        step_deg += kExtendYawSweepStepDeg)
      {
        for (const double sign : {1.0, -1.0}) {
          const double yaw_try = e_yaw + sign * step_deg * M_PI / 180.0;
          const double ex_try = gx0 + extend_m * std::cos(yaw_try);
          const double ey_try = gy0 + extend_m * std::sin(yaw_try);
          std::string sweep_reason;
          if (!footClear(ex_try, ey_try, yaw_try, &sweep_reason)) {
            continue;
          }
          e_yaw = yaw_try;
          e_tx = std::cos(e_yaw);
          e_ty = std::sin(e_yaw);
          ex = ex_try;
          ey = ey_try;
          e_ok = true;
          RCLCPP_INFO(
            node_->get_logger(),
            "InsertGarbagePose: wall-edge E sweep %+g deg -> (%.2f, %.2f)",
            sign * step_deg, ex, ey);
          break;
        }
      }
      if (!e_ok) {
        chain.invalid_reason = "E footprint fail after sweep: " + e_reason;
        return chain;
      }
    }
  }

  // D：生成时与 E 对侧绑定；安全调整时 E 不动，只独立扫角/加长 D
  double d_yaw = std::atan2(-e_ty, -e_tx);
  const double step = std::max(0.1, wall_edge_d_extend_m_);
  const double gd = std::max(0.0, wall_edge_min_robot_dist_m_);
  auto placeD = [&](double yaw, double len, double * ox, double * oy) {
    *ox = gx0 + len * std::cos(yaw);
    *oy = gy0 + len * std::sin(yaw);
  };
  auto pushLenForGd = [&](double yaw) {
    double len = step;
    double ox = 0.0;
    double oy = 0.0;
    placeD(yaw, len, &ox, &oy);
    for (int i = 0; i < 20 && std::hypot(ox - from_x, oy - from_y) < gd; ++i) {
      len += step;
      placeD(yaw, len, &ox, &oy);
    }
    return len;
  };

  double d_len = pushLenForGd(d_yaw);
  double dx = 0.0;
  double dy = 0.0;
  placeD(d_yaw, d_len, &dx, &dy);
  bool d_ok = false;
  {
    std::string d_reason;
    const double yaw_travel = std::atan2(gy0 - dy, gx0 - dx);
    if (footClear(dx, dy, yaw_travel, &d_reason) && deLineClear(dx, dy, ex, ey)) {
      d_ok = true;
    } else {
      for (double step_deg = kExtendYawSweepStepDeg;
        step_deg <= kExtendYawSweepMaxDeg + 1e-6 && !d_ok;
        step_deg += kExtendYawSweepStepDeg)
      {
        for (const double sign : {1.0, -1.0}) {
          const double yaw_try = d_yaw + sign * step_deg * M_PI / 180.0;
          const double len_try = pushLenForGd(yaw_try);
          double dx_try = 0.0;
          double dy_try = 0.0;
          placeD(yaw_try, len_try, &dx_try, &dy_try);
          const double yaw_trav = std::atan2(gy0 - dy_try, gx0 - dx_try);
          std::string sweep_reason;
          if (!footClear(dx_try, dy_try, yaw_trav, &sweep_reason)) {
            continue;
          }
          if (!deLineClear(dx_try, dy_try, ex, ey)) {
            continue;
          }
          d_yaw = yaw_try;
          d_len = len_try;
          dx = dx_try;
          dy = dy_try;
          d_ok = true;
          RCLCPP_INFO(
            node_->get_logger(),
            "InsertGarbagePose: wall-edge D sweep %+g deg -> (%.2f, %.2f)",
            sign * step_deg, dx, dy);
          break;
        }
      }
      // 扫角仍不通：沿当前 d_yaw 再多推几步试安全点
      if (!d_ok) {
        for (int i = 0; i < 10 && !d_ok; ++i) {
          d_len += step;
          placeD(d_yaw, d_len, &dx, &dy);
          const double yaw_trav = std::atan2(gy0 - dy, gx0 - dx);
          std::string push_reason;
          if (footClear(dx, dy, yaw_trav, &push_reason) &&
            deLineClear(dx, dy, ex, ey))
          {
            d_ok = true;
            RCLCPP_INFO(
              node_->get_logger(),
              "InsertGarbagePose: wall-edge D push len=%.2f -> (%.2f, %.2f)",
              d_len, dx, dy);
          }
        }
      }
      // 不再联扫挪 E：贴墙平行时挪 E 易把一端顶进墙；E 定稿后只独立挪 D
      if (!d_ok) {
        chain.invalid_reason = "D footprint/D-E line fail after D-only sweep: " + d_reason;
        return chain;
      }
    }
  }

  const double sample = std::max(0.05, wall_edge_sample_m_);
  const double chain_len = std::hypot(ex - dx, ey - dy);
  std::vector<std::pair<double, double>> chain_xy;
  if (chain_len < 1e-6) {
    chain_xy.push_back({gx0, gy0});
  } else {
    const double ux = (ex - dx) / chain_len;
    const double uy = (ey - dy) / chain_len;
    const int n_seg = std::max(1, static_cast<int>(std::ceil(chain_len / sample)));
    chain_xy.reserve(static_cast<std::size_t>(n_seg) + 3u);
    for (int i = 0; i <= n_seg; ++i) {
      const double s = chain_len * static_cast<double>(i) / static_cast<double>(n_seg);
      chain_xy.emplace_back(dx + ux * s, dy + uy * s);
    }
    bool g_on_chain = false;
    for (const auto & p : chain_xy) {
      if (squaredDistanceXY(p.first, p.second, gx0, gy0) < 0.01) {
        g_on_chain = true;
        break;
      }
    }
    if (!g_on_chain) {
      const double sg = (gx0 - dx) * ux + (gy0 - dy) * uy;
      std::size_t insert_at = chain_xy.size();
      for (std::size_t i = 0; i < chain_xy.size(); ++i) {
        const double si =
          (chain_xy[i].first - dx) * ux + (chain_xy[i].second - dy) * uy;
        if (si > sg) {
          insert_at = i;
          break;
        }
      }
      chain_xy.insert(
        chain_xy.begin() + static_cast<std::ptrdiff_t>(insert_at), {gx0, gy0});
    }
  }

  for (auto & p : chain_xy) {
    p.first += off * nx;
    p.second += off * ny;
  }
  dx += off * nx;
  dy += off * ny;
  ex += off * nx;
  ey += off * ny;
  const double gx = gx0 + off * nx;
  const double gy = gy0 + off * ny;

  chain.valid = true;
  chain.xy = std::move(chain_xy);
  chain.dx = dx;
  chain.dy = dy;
  chain.gx = gx;
  chain.gy = gy;
  chain.ex = ex;
  chain.ey = ey;
  chain.path_yaw = e_yaw;
  chain.extend_used_m = extend_m;
  chain.px = px;
  chain.py = py;
  chain.normal_offset_m = off;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: wall-edge chain D=(%.2f, %.2f) G=(%.2f, %.2f) E=(%.2f, %.2f) "
    "n=%zu offset=%.2f P=(%.2f, %.2f)",
    dx, dy, gx, gy, ex, ey, chain.xy.size(), off, px, py);
  return chain;
}

InsertGarbagePose::Goals InsertGarbagePose::insertWallEdgeGarbageIntoGoals(
  InsertInfo & info)
{
  const WallEdgeExtendChain chain = buildWallEdgeExtendChain(info);
  if (!chain.valid) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: wall-edge chain fail (%s) at G=(%.2f, %.2f), 放弃该堆",
      chain.invalid_reason.c_str(),
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
    return info.goals;
  }

  const double gx = chain.gx;
  const double gy = chain.gy;
  const double ex = chain.ex;
  const double ey = chain.ey;
  const double e_yaw = chain.path_yaw;
  const auto & chain_xy = chain.xy;

  info.garbage.pose.pose.position.x = gx;
  info.garbage.pose.pose.position.y = gy;
  info.path_yaw = e_yaw;
  info.extend_x = ex;
  info.extend_y = ey;
  info.extend_inserted = true;
  info.extend_used_m = chain.extend_used_m;
  info.wall_edge_inserted = true;
  info.wall_edge_d_x = chain.dx;
  info.wall_edge_d_y = chain.dy;
  info.wall_edge_chain_xy = chain_xy;

  // 以下与普通插入相同的前缀裁剪与接回
  Goals prefix;
  Goals path;
  int last_protected = -1;
  for (std::size_t i = 0; i < info.goals.size(); ++i) {
    if (isProtectedGarbageXy(
        info.goals[i].pose.position.x, info.goals[i].pose.position.y) ||
      isUnindexedSentinelPoseZ(info.goals[i]))
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
      path, info.robot_pose, gx, gy);
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
      const double ddx = out[j].pose.position.x - ax;
      const double ddy = out[j].pose.position.y - ay;
      if (ddx * ddx + ddy * ddy < kMatchTol2) {
        insert_anchor = j + 1;
        have_insert_after = true;
        break;
      }
    }
  } else if (info.hit_forward_case && !info.corners_kept_xy.empty()) {
    for (auto it = info.corners_kept_xy.rbegin(); it != info.corners_kept_xy.rend(); ++it) {
      for (std::size_t j = 0; j < out.size(); ++j) {
        const double ddx = out[j].pose.position.x - it->first;
        const double ddy = out[j].pose.position.y - it->second;
        if (ddx * ddx + ddy * ddy < kMatchTol2) {
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

  Goals suffix;
  suffix.assign(
    out.begin() + static_cast<std::ptrdiff_t>(resume_from), out.end());

  // 不用 E 再全量 clip 进近段，只接回 suffix
  std::string frame_id = global_frame_;
  if (!out.empty() && !out.front().header.frame_id.empty()) {
    frame_id = out.front().header.frame_id;
  } else if (!info.goals.empty() && !info.goals.front().header.frame_id.empty()) {
    frame_id = info.goals.front().header.frame_id;
  }

  const auto orient = nav2_util::geometry_utils::orientationAroundZAxis(e_yaw);
  Goals chain_poses;
  chain_poses.reserve(chain_xy.size());
  for (std::size_t i = 0; i < chain_xy.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = chain_xy[i].first;
    pose.pose.position.y = chain_xy[i].second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = orient;
    const bool is_g = squaredDistanceXY(
        chain_xy[i].first, chain_xy[i].second, gx, gy) < 0.01;
    const bool is_e = squaredDistanceXY(
        chain_xy[i].first, chain_xy[i].second, ex, ey) < 0.01;
    if (is_g || is_e) {
      pose.pose.position.z = kGarbageSentinelPoseZ;
    }
    chain_poses.push_back(pose);
  }

  Goals rebuilt;
  rebuilt.reserve(prefix.size() + chain_poses.size() + suffix.size());
  rebuilt.insert(rebuilt.end(), prefix.begin(), prefix.end());
  rebuilt.insert(rebuilt.end(), chain_poses.begin(), chain_poses.end());
  rebuilt.insert(rebuilt.end(), suffix.begin(), suffix.end());
  out = std::move(rebuilt);

  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : out) {
    pose.header.stamp = stamp_now;
  }
  mission_stamp_record_ = stamp_now;
  has_mission_stamp_ = true;

  last_sweep_arrive_xy_ = {ex, ey};
  has_last_sweep_arrive_ = true;
  last_sweep_path_yaw_ = e_yaw;
  has_last_sweep_path_yaw_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: wall-edge insert chain=%zu resume_from=%zu remain=%zu",
    chain_poses.size(), resume_from, out.size());

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

// footprint 检查不通过时，把当时检查用的 footprint 框画在垃圾位置上（空心蓝框）
void InsertGarbagePose::publishFootprintCheckBox(double x, double y, double yaw)
{
  if (!marker_pub_) {
    return;
  }
  std::vector<std::pair<double, double>> local_xy;
  if (!getRobotFootprintInBase(local_xy) || local_xy.size() < 3) {
    return;
  }

  const double c_yaw = std::cos(yaw);
  const double s_yaw = std::sin(yaw);

  visualization_msgs::msg::Marker box;
  box.header.frame_id = global_frame_;
  box.header.stamp = node_->now();
  box.ns = "footprint_check_fail";
  box.id = static_cast<int>(viz_footprint_fail_count_);
  box.type = visualization_msgs::msg::Marker::LINE_STRIP;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.pose.orientation.w = 1.0;
  box.scale.x = 0.05;
  box.color.r = 0.15f;
  box.color.g = 0.40f;
  box.color.b = 0.95f;
  box.color.a = 1.0f;
  box.lifetime = rclcpp::Duration::from_seconds(0.0);
  box.points.reserve(local_xy.size() + 1);
  for (const auto & off : local_xy) {
    geometry_msgs::msg::Point p;
    p.x = x + off.first * c_yaw - off.second * s_yaw;
    p.y = y + off.first * s_yaw + off.second * c_yaw;
    p.z = 0.05;
    box.points.push_back(p);
  }
  if (!box.points.empty()) {
    box.points.push_back(box.points.front());   // 闭合矩形
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(box);
  marker_pub_->publish(arr);
  ++viz_footprint_fail_count_;
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
  // 可视化生命周期 = 工作圈生命周期：生成工作圈时开始，取消时被整体删除；
  // 没有工作圈时不画圈，避免"取消后又被下一帧画回来"
  if (has_work_circle_ && max_garbage_robot_dist_m_ > 0.0) {
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
  const double cell = std::max(0.12, viz_obstacle_cell_m_) * 1.4;
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

void InsertGarbagePose::logGarbageListState(const char * reason) const
{
  std::ostringstream xy_oss;
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    if (i > 0) {
      xy_oss << " ";
    }
    xy_oss << "(" << garbage_list_[i].pose.pose.position.x << ","
           << garbage_list_[i].pose.pose.position.y << ")";
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: garbage_list_ %s: %zu pile(s)%s%s",
    reason, garbage_list_.size(),
    garbage_list_.empty() ? "" : " ",
    xy_oss.str().c_str());
}

// 往 RViz 发本次插入的证据 Marker

void InsertGarbagePose::publishVisualization(
  const InsertInfo & info,
  bool enable,
  bool viz_accepted_garbage)
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

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;

  if (viz_accepted_garbage) {
    // 每堆 6 id：G 红点 + 标签 | G-E 蓝虚线 | E 蓝点 + 标签
    const int base = pile_idx * 6;
    constexpr double kGarbageDotZM = 0.06;
    constexpr double kGarbageDotSizeM = 0.10;
    constexpr double kDashLenM = 0.12;
    constexpr double kGapLenM = 0.08;
    constexpr double kDashLineWidthM = 0.030;

    auto makeSolidDot = [&](int id, double x, double y,
        float r, float g, float b)
    {
      auto dot = makeBase("accepted_garbage", id, visualization_msgs::msg::Marker::SPHERE);
      dot.pose.position.x = x;
      dot.pose.position.y = y;
      dot.pose.position.z = kGarbageDotZM;
      dot.scale.x = kGarbageDotSizeM;
      dot.scale.y = kGarbageDotSizeM;
      dot.scale.z = kGarbageDotSizeM;
      setColor(dot, r, g, b, 1.0f);
      return dot;
    };

    auto appendDashedLine = [&](visualization_msgs::msg::Marker & line,
        double x0, double y0, double x1, double y1, double z = 0.07)
    {
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double len = std::hypot(dx, dy);
      if (len < 1e-4) {
        return;
      }
      const double ux = dx / len;
      const double uy = dy / len;
      double s = 0.0;
      bool draw = true;
      while (s < len - 1e-6) {
        const double step = draw ? kDashLenM : kGapLenM;
        const double s_next = std::min(s + step, len);
        if (draw) {
          pushPoint(line, x0 + ux * s, y0 + uy * s, z);
          pushPoint(line, x0 + ux * s_next, y0 + uy * s_next, z);
        }
        s = s_next;
        draw = !draw;
      }
    };

    arr.markers.push_back(makeSolidDot(base, gx, gy, 0.92f, 0.10f, 0.10f));

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

    if (info.extend_inserted) {
      const double ex = info.extend_x;
      const double ey = info.extend_y;

      auto ge_line = makeBase("accepted_garbage", base + 2, visualization_msgs::msg::Marker::LINE_LIST);
      ge_line.scale.x = kDashLineWidthM;
      setColor(ge_line, 0.15f, 0.40f, 0.95f, 0.90f);
      appendDashedLine(ge_line, gx, gy, ex, ey);
      if (!ge_line.points.empty()) {
        arr.markers.push_back(ge_line);
      }

      arr.markers.push_back(makeSolidDot(base + 3, ex, ey, 0.15f, 0.40f, 0.95f));

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
      setColor(te, 0.15f, 0.40f, 0.95f);
      arr.markers.push_back(te);
    }

    // 贴边：D 蓝点+D标签（无虚线）；中间采样点纯蓝点（无文字、无虚线）
    if (info.wall_edge_inserted) {
      constexpr float kBlueR = 0.15f;
      constexpr float kBlueG = 0.40f;
      constexpr float kBlueB = 0.95f;
      constexpr double kMidDotSizeM = 0.07;
      constexpr int kWallEdgeIdBase = 8000;
      constexpr int kWallEdgeIdSpan = 128;
      const int wbase = kWallEdgeIdBase + pile_idx * kWallEdgeIdSpan;
      int wid = 0;

      {
        auto d_dot = makeBase("wall_edge_pts", wbase + wid++, visualization_msgs::msg::Marker::SPHERE);
        d_dot.pose.position.x = info.wall_edge_d_x;
        d_dot.pose.position.y = info.wall_edge_d_y;
        d_dot.pose.position.z = kGarbageDotZM;
        d_dot.scale.x = kGarbageDotSizeM;
        d_dot.scale.y = kGarbageDotSizeM;
        d_dot.scale.z = kGarbageDotSizeM;
        setColor(d_dot, kBlueR, kBlueG, kBlueB, 1.0f);
        arr.markers.push_back(d_dot);
      }
      auto td = makeBase(
        "wall_edge_pts", wbase + wid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      td.pose.position.x = info.wall_edge_d_x;
      td.pose.position.y = info.wall_edge_d_y;
      td.pose.position.z = 0.40;
      td.scale.z = 0.22;
      {
        std::ostringstream oss;
        oss << "D" << pile_num;
        td.text = oss.str();
      }
      setColor(td, kBlueR, kBlueG, kBlueB);
      arr.markers.push_back(td);

      for (const auto & p : info.wall_edge_chain_xy) {
        if (wid >= kWallEdgeIdSpan) {
          break;
        }
        if (squaredDistanceXY(p.first, p.second, info.wall_edge_d_x, info.wall_edge_d_y) < 0.01 ||
          squaredDistanceXY(p.first, p.second, gx, gy) < 0.01 ||
          (info.extend_inserted &&
          squaredDistanceXY(p.first, p.second, info.extend_x, info.extend_y) < 0.01))
        {
          continue;
        }
        auto mid = makeBase("wall_edge_pts", wbase + wid++, visualization_msgs::msg::Marker::SPHERE);
        mid.pose.position.x = p.first;
        mid.pose.position.y = p.second;
        mid.pose.position.z = kGarbageDotZM;
        mid.scale.x = kMidDotSizeM;
        mid.scale.y = kMidDotSizeM;
        mid.scale.z = kMidDotSizeM;
        setColor(mid, kBlueR, kBlueG, kBlueB, 1.0f);
        arr.markers.push_back(mid);
      }
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
    double robot_x = 0.0;
    double robot_y = 0.0;
    if (getRobotPoseXY(robot_x, robot_y)) {
      publishRangeCircles(robot_x, robot_y);
    }
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return BT::NodeStatus::SUCCESS;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  publishRangeCircles(rx, ry);

  std::vector<geometry_msgs::msg::Point> footprint_map;
  const bool have_fp = getRobotFootprintInMap(footprint_map);
  stripReachedZNeg1Goals(goals_now);

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
        std::size_t in_idx = 0;
        const bool in_goals = findUnindexedSentinelIndex(goals_now, ax, ay, &in_idx);
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
                   << " in_goals=" << (in_goals ? 1 : 0);
        if (in_goals) {
          active_oss << " idx=" << in_idx;
        }
        active_oss << " fp_arrived=" << (fp_arrived ? 1 : 0);
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

  if (has_work_circle_ && garbage_list_.empty() && active_piles_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "InsertGarbagePose: 工作圈取消");
    clearMissionVisualization();
    viz_obstacle_pixels_.clear();
    viz_obstacle_marker_count_ = 0;
    viz_pile_count_ = 0;
    viz_footprint_fail_count_ = 0;
    has_work_circle_ = false;
  }

  bypass_pending_insert_ = false;
  std::size_t new_idx = 0;
  const bool have_new_pile = findNewGarbageIndex(before, rx, ry, new_idx);
  const bool mid_mission_new = have_new_pile && !active_piles_.empty();
  if (mid_mission_new) {
    // 正在扫的 G/E 留在 {goals} 里继续扫完；只把尚未开始的已插堆 + 新堆重排后接在后面
    std::vector<std::pair<double, double>> keep_xy;
    int keep_g_num = 0;
    const bool have_keep = collectInProgressKeepXy(goals_now, &keep_xy, &keep_g_num);
    const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
    auto isKeepXy = [&](double x, double y) {
      for (const auto & p : keep_xy) {
        if (squaredDistanceXY(p.first, p.second, x, y) < thresh2) {
          return true;
        }
      }
      return false;
    };
    std::size_t keep_idx = 0;
    const bool keep_idx_found = have_keep && !keep_xy.empty() &&
      findUnindexedSentinelIndex(
        goals_now, keep_xy.front().first, keep_xy.front().second, &keep_idx);

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
    std::ostringstream keep_oss;
    for (std::size_t i = 0; i < keep_xy.size(); ++i) {
      if (i > 0) {
        keep_oss << " ";
      }
      keep_oss << "(" << keep_xy[i].first << "," << keep_xy[i].second << ")";
    }
    const std::string keep_idx_str = keep_idx_found ? std::to_string(keep_idx) : "-";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: diag mid-mission BEGIN goals=%zu active_n=%zu %s "
      "new_list_n=%zu %s keep_g=%d keep_idx=%s keep_xy=%s "
      "(keep in-progress G/E slot, reorder unstarted+new only)",
      goals_now.size(), active_piles_.size(), old_active_oss.str().c_str(),
      garbage_list_.size(), new_list_oss.str().c_str(),
      keep_g_num, keep_idx_str.c_str(), keep_oss.str().c_str());

    GarbageList keep_active;
    GarbageList rest_active;
    keep_active.reserve(active_piles_.size());
    rest_active.reserve(active_piles_.size());
    if (have_keep) {
      for (const auto & pile : active_piles_) {
        const double ax = pile.pose.pose.position.x;
        const double ay = pile.pose.pose.position.y;
        if (isKeepXy(ax, ay)) {
          keep_active.push_back(pile);
        } else {
          rest_active.push_back(pile);
        }
      }
      if (keep_active.empty() && !active_piles_.empty()) {
        keep_active.push_back(active_piles_.front());
        rest_active.clear();
        for (std::size_t i = 1; i < active_piles_.size(); ++i) {
          rest_active.push_back(active_piles_[i]);
        }
      }
    } else {
      // 还没真正扫到：已插堆全部可剥，和新堆一起重排
      rest_active = active_piles_;
    }

    Goals kept_goals;
    kept_goals.reserve(goals_now.size());
    std::size_t peeled_n = 0;
    for (std::size_t i = 0; i < goals_now.size(); ++i) {
      const auto & g = goals_now[i];
      const double px = g.pose.position.x;
      const double py = g.pose.position.y;
      const bool peel_this = isUnindexedSentinelPoseZ(g) &&
        (!have_keep || !isKeepXy(px, py));
      if (peel_this) {
        ++peeled_n;
        eraseProtectedGarbageXy(px, py);
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: diag mid-mission peel unstarted idx=%zu "
          "(%.2f, %.2f) z=%.1f",
          i, px, py, g.pose.position.z);
        continue;
      }
      kept_goals.push_back(g);
    }
    goals_now = std::move(kept_goals);

    for (const auto & pile : rest_active) {
      const double ax = pile.pose.pose.position.x;
      const double ay = pile.pose.pose.position.y;
      eraseProtectedGarbageXy(ax, ay);
      const int g_num = lookupStableGNum(ax, ay);
      if (g_num > 0) {
        for (const auto & item : e_num_xy_) {
          if (item.second == g_num) {
            eraseProtectedGarbageXy(item.first.first, item.first.second);
          }
        }
      }
    }

    double order_x = rx;
    double order_y = ry;
    double order_yaw = robot_yaw;
    if (have_keep && !keep_xy.empty()) {
      double gx = keep_xy.front().first;
      double gy = keep_xy.front().second;
      double ex = gx;
      double ey = gy;
      bool have_g = false;
      bool have_e = false;
      for (const auto & p : keep_xy) {
        if (lookupStableGNum(p.first, p.second) > 0) {
          gx = p.first;
          gy = p.second;
          have_g = true;
        }
        if (lookupStableENum(p.first, p.second) > 0) {
          ex = p.first;
          ey = p.second;
          have_e = true;
        }
      }
      if (have_e) {
        last_sweep_arrive_xy_ = {ex, ey};
      } else {
        last_sweep_arrive_xy_ = {gx, gy};
      }
      has_last_sweep_arrive_ = true;
      order_x = last_sweep_arrive_xy_.first;
      order_y = last_sweep_arrive_xy_.second;
      if (have_g && have_e) {
        const double dx = ex - gx;
        const double dy = ey - gy;
        if (dx * dx + dy * dy > 1e-6) {
          order_yaw = std::atan2(dy, dx);
        }
      }
      last_sweep_path_yaw_ = order_yaw;
      has_last_sweep_path_yaw_ = true;
    } else {
      has_last_sweep_arrive_ = false;
      has_last_sweep_path_yaw_ = false;
      last_sweep_xy_.clear();
    }

    GarbageList incoming = std::move(garbage_list_);
    garbage_list_.clear();
    garbage_list_.reserve(rest_active.size() + incoming.size());
    for (const auto & pile : rest_active) {
      garbage_list_.push_back(pile);
    }
    for (const auto & g : incoming) {
      if (!isDuplicateOfKept(g, garbage_list_) &&
        !isDuplicateOfKept(g, keep_active))
      {
        garbage_list_.push_back(g);
      }
    }
    active_piles_ = std::move(keep_active);

    {
      std::ostringstream rest_oss;
      for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
        if (i > 0) {
          rest_oss << " ";
        }
        rest_oss << "(" << garbage_list_[i].pose.pose.position.x << ","
                 << garbage_list_[i].pose.pose.position.y << ")";
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: diag mid-mission %s G%d, peeled=%zu "
        "kept_goals=%zu reorder_n=%zu %s",
        have_keep ? "keep in-progress" : "peel all unstarted",
        keep_g_num, peeled_n, goals_now.size(),
        garbage_list_.size(), rest_oss.str().c_str());
    }
    logGarbageListState("mid-mission reorder unstarted+new");

    (void)new_idx;
    if (garbage_list_.size() > 1) {
      reorderNearestFirstThenSweep(order_x, order_y, order_yaw);
    } else if (garbage_list_.size() == 1) {
      syncLastSweepXyFromList();
    }
    logSweepOrder();
  } else if (garbage_list_.size() > 1 && last_sweep_xy_.empty()) {
    reorderNearestFirstThenSweep(rx, ry, robot_yaw);
    logSweepOrder();
  } else if (garbage_list_.size() == 1 && last_sweep_xy_.empty()) {
    syncLastSweepXyFromList();
  }

  if (garbage_list_.empty()) {
    if (goals_dirty) {
      emitOutputGoals(goals_now, "strip_z_neg1");
    }
    return BT::NodeStatus::SUCCESS;
  }

  bool enable_viz = true;
  bool viz_garbage = true;
  getInput("enable_visualization", enable_viz);
  getInput("viz_accepted_garbage", viz_garbage);

  // 按当前顺序一次插入全部待插堆
  const std::size_t goals_before_batch = goals_now.size();
  std::size_t inserted_count = 0;
  std::size_t deleted_goals_total = 0;
  std::ostringstream inserted_xy;
  while (!garbage_list_.empty()) {
    const double gx = garbage_list_.front().pose.pose.position.x;
    const double gy = garbage_list_.front().pose.pose.position.y;
    if (isNearReachedGarbage(gx, gy)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: diag skip insert (%.2f, %.2f): near protected/reached",
        gx, gy);
      garbage_list_.erase(garbage_list_.begin());
      continue;
    }
    InsertInfo info = gatherInsertInfo(goals_now, robot_pose, gx, gy);
    if (!info.valid) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: diag skip insert (%.2f, %.2f): gather invalid (%s)",
        gx, gy, info.invalid_reason.c_str());
      garbage_list_.erase(garbage_list_.begin());
      continue;
    }
    info.dist_label = assignStableGNum(gx, gy);
    if (!isStraightLineClearOnGlobalCostmap(
        robot_pose.pose.position.x, robot_pose.pose.position.y, gx, gy))
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip garbage (%.2f, %.2f), 全局车到G连线有障碍，不生成",
        gx, gy);
      addProtectedGarbageXy(gx, gy);
      garbage_list_.erase(garbage_list_.begin());
      continue;
    }
    const std::size_t goals_before_pile = goals_now.size();
    std::string fp_reason;
    const bool footprint_ok = isFootprintClearAtPose(
      gx, gy, info.path_yaw, &fp_reason);
    if (footprint_ok) {
      // G 点 footprint 通过：正常 G-E 扫
      goals_now = insertGarbageIntoGoals(info);
    } else {
      // DEG（贴墙 D-G-E）先不尝试：走不通就跳过这堆，不生成。
      // 原贴边插入逻辑注释保留在下面，以后再开。
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip garbage (%.2f, %.2f), DEG 不通，不生成 (%s)",
        gx, gy, fp_reason.c_str());
      publishFootprintCheckBox(gx, gy, info.path_yaw);
      addProtectedGarbageXy(gx, gy);
      garbage_list_.erase(garbage_list_.begin());
      continue;
      /*
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: footprint fail at G (%.2f, %.2f): %s, wall-edge insert",
        gx, gy, fp_reason.c_str());
      goals_now = insertWallEdgeGarbageIntoGoals(info);
      if (!info.wall_edge_inserted) {
        // 贴边链没生成出来：这堆不扫，跳过并清掉（登记为已处理，免得检测反复塞回来）
        addProtectedGarbageXy(gx, gy);
        garbage_list_.erase(garbage_list_.begin());
        continue;
      }
      */
    }
    const std::size_t pile_deleted = info.goaltotal.size();
    deleted_goals_total += pile_deleted;
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: insert G%d (%.2f, %.2f) extend=%d, deleted %zu path goals, "
      "goals %zu -> %zu",
      info.dist_label,
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y,
      info.extend_inserted ? 1 : 0,
      pile_deleted, goals_before_pile, goals_now.size());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: goals list (%zu): %s",
      goals_now.size(), formatGoalsListCompact(goals_now).c_str());
    publishVisualization(info, enable_viz, viz_garbage);
    publishRangeCircles(rx, ry);
    addProtectedGarbageXy(gx, gy);
    addProtectedGarbageXy(
      info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
    if (info.dist_label > 0 &&
      squaredDistanceXY(
        gx, gy,
        info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y) > 1e-6)
    {
      g_num_xy_.push_back({
        {info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y},
        info.dist_label});
    }
    if (info.extend_inserted) {
      addProtectedGarbageXy(info.extend_x, info.extend_y);
      registerStableENum(info.extend_x, info.extend_y, info.dist_label);
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
      "InsertGarbagePose: batch insert done: inserted %zu pile(s), deleted %zu path goals, "
      "goals %zu -> %zu, xy %s",
      inserted_count, deleted_goals_total,
      goals_before_batch, goals_now.size(),
      inserted_xy.str().c_str());
    logGarbageListState("after batch insert");
  }

  if (goals_dirty) {
    const char * reason = (inserted_count > 0) ? "batch_insert" : "strip_z_neg1";
    emitOutputGoals(goals_now, reason);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
