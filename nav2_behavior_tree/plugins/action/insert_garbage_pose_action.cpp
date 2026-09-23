#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
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
  global_costmap_topic_("global_costmap/costmap_raw"),
  visualization_topic_("insert_garbage_pose/markers"),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  clip_extend_m_(2.5),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(30.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0),  // 车离路径最近点超过该距离就不删点
  max_garbage_robot_dist_m_(5.0),  // 垃圾离机器人超过该距离则忽略
  wall_edge_d_extend_m_(2.0),
  wall_edge_e_extend_m_(2.0),
  wall_edge_min_robot_dist_m_(3.0),
  wall_edge_sample_m_(0.5),
  wall_edge_normal_offset_m_(0.0),
  garbage_merge_radius_m_(1.0),    // 到种子小于该距离合为一堆
  garbage_extend_m_(2.0),          // 沿扫向相对垃圾再插一点，默认 2.0m
  work_circle_radius_m_(10.0)
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("global_costmap_topic", global_costmap_topic_);
  getInput("visualization_topic", visualization_topic_);
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("sweep_dist_weight", sweep_dist_weight_);
  getInput("sweep_turn_weight", sweep_turn_weight_);
  getInput("wall_edge_d_extend_m", wall_edge_d_extend_m_);
  getInput("wall_edge_e_extend_m", wall_edge_e_extend_m_);
  getInput("wall_edge_min_robot_dist_m", wall_edge_min_robot_dist_m_);
  getInput("wall_edge_sample_m", wall_edge_sample_m_);
  getInput("wall_edge_normal_offset_m", wall_edge_normal_offset_m_);
  getInput("garbage_merge_radius_m", garbage_merge_radius_m_);
  getInput("garbage_extend_m", garbage_extend_m_);
  getInput("extend_max_yaw_deg", extend_max_yaw_deg_);
  getInput("extend_step_yaw_deg", extend_step_yaw_deg_);
  getInput("work_circle_radius_m", work_circle_radius_m_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

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

  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node_, global_costmap_topic_);
  footprint_topic_sub_ = std::make_shared<FeedableFootprintSubscriber>(
    node_, footprint_topic_, *tf_, robot_base_frame_, transform_tolerance_);
  collision_checker_ = std::make_unique<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_topic_sub_, "insert_garbage_pose");

  // 挂到本节点 callback group：tick 里 spin_some 后立刻能查，不依赖默认组
  footprint_feed_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&InsertGarbagePose::footprintCallback, this, std::placeholders::_1),
    sub_option);

  rclcpp::QoS global_costmap_qos(rclcpp::KeepLast(1));
  global_costmap_qos.transient_local().reliable();
  costmap_feed_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    global_costmap_topic_,
    global_costmap_qos,
    std::bind(&InsertGarbagePose::globalCostmapCallback, this, std::placeholders::_1),
    sub_option);

  // RViz Marker，默认 VOLATILE，和 RViz 订阅对齐
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_, 10);
}

// 垃圾检测话题回调：转到 map；时间窗多帧确认后进 history
void InsertGarbagePose::garbageDetectCallback(
  const capella_ros_msg::msg::GarbageDetect::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    if (single_pile_block_intake_) {
      return;
    }
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
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: drop garbage, get the robot pose failed, wait for next frame");
    return;
  }

  const double gx = item.pose.pose.position.x;
  const double gy = item.pose.pose.position.y;
  getInput("confirm_match_dist_m", confirm_match_dist_m_);
  getInput("confirm_match_num", confirm_match_num_);
  const double merge_r2 =
    std::max(0.0, garbage_merge_radius_m_) * std::max(0.0, garbage_merge_radius_m_);
  const double confirm_r = std::max(0.0, confirm_match_dist_m_);
  const double confirm_r2 = confirm_r * confirm_r;
  const int confirm_match_num = std::max(1, confirm_match_num_);

  std::lock_guard<std::mutex> lock(history_mutex_);
  if (single_pile_block_intake_) {
    return;
  }

  for (const auto & kept : garbage_list_) {
    if (squaredDistanceXY(
        gx, gy, kept.pose.pose.position.x, kept.pose.pose.position.y) < merge_r2)
    {
      return;
    }
  }

  // 时间窗内累计检测：凑够 confirm_match_num 帧且距离在 confirm_match_dist_m 内，取平均往下传
  if (confirm_r > 1e-6 && confirm_match_num > 1) {
    const rclcpp::Time now = node_->now();
    for (auto it = tmp_list_.begin(); it != tmp_list_.end(); ) {
      const double age = (now - rclcpp::Time(it->pose.header.stamp)).seconds();
      if (age > TmpSecGarbageTime) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: 删除待确认垃圾 (%.2f, %.2f), 超过 %.1f 秒没再看到",
          it->pose.pose.position.x, it->pose.pose.position.y, TmpSecGarbageTime);
        it = tmp_list_.erase(it);
        continue;
      }
      ++it;
    }

    item.pose.header.stamp = now;
    if (tmp_list_.size() >= kMaxHistorySize) {
      tmp_list_.pop_front();
    }
    tmp_list_.push_back(item);

    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t match_n = 0;
    for (const auto & cand : tmp_list_) {
      if (squaredDistanceXY(
          gx, gy, cand.pose.pose.position.x, cand.pose.pose.position.y) < confirm_r2)
      {
        sum_x += cand.pose.pose.position.x;
        sum_y += cand.pose.pose.position.y;
        ++match_n;
      }
    }
    if (static_cast<int>(match_n) < confirm_match_num) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾待确认 坐标=(%.2f, %.2f) 同堆帧数=%zu/%d",
        gx, gy, match_n, confirm_match_num);
      return;
    }

    item.pose.pose.position.x = sum_x / static_cast<double>(match_n);
    item.pose.pose.position.y = sum_y / static_cast<double>(match_n);
    for (auto it = tmp_list_.begin(); it != tmp_list_.end(); ) {
      if (squaredDistanceXY(
          gx, gy, it->pose.pose.position.x, it->pose.pose.position.y) < confirm_r2)
      {
        it = tmp_list_.erase(it);
      } else {
        ++it;
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 垃圾确认通过 帧数=%zu 平均坐标=(%.2f, %.2f)",
      match_n, item.pose.pose.position.x, item.pose.pose.position.y);
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

// footprint 话题回调：喂给 Nav2 检查器，并按 2.11 缓存一次车长
void InsertGarbagePose::footprintCallback(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (footprint_topic_sub_) {
    footprint_topic_sub_->feed(msg);
  }
  // 清扫刷收放会改轮廓，收到新轮廓就标脏，下次用时重新取一遍车长
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  footprint_dirty_ = true;
}

void InsertGarbagePose::globalCostmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  if (!msg || !costmap_sub_) {
    return;
  }
  costmap_sub_->costmapCallback(msg);
}

bool InsertGarbagePose::ensureCachedFootprint(std::string * reason) const
{
  auto haveUsableCache = [this]() {
      std::lock_guard<std::mutex> lock(footprint_mutex_);
      return have_cached_footprint_ && cached_footprint_base_.size() >= 3;
    };
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    if (have_cached_footprint_ && !footprint_dirty_ && cached_footprint_base_.size() >= 3) {
      return true;
    }
  }
  if (!footprint_topic_sub_) {
    if (haveUsableCache()) {
      return true;
    }
    if (reason) {
      *reason = "no footprint subscriber";
    }
    return false;
  }

  std::vector<geometry_msgs::msg::Point> fp;
  std_msgs::msg::Header header;
  if (!footprint_topic_sub_->getFootprintInRobotFrame(fp, header) || fp.size() < 3) {
    // 取不到新的就继续用上一份，别因为一拍 TF 抖动让所有安全检查都判不通过
    if (haveUsableCache()) {
      return true;
    }
    if (reason) {
      *reason = "no footprint";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(footprint_mutex_);
  cached_footprint_base_.clear();
  cached_footprint_base_.reserve(fp.size());
  double min_x = fp.front().x;
  double max_x = fp.front().x;
  for (const auto & pt : fp) {
    cached_footprint_base_.emplace_back(pt.x, pt.y);
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
  }
  cached_robot_length_m_ = std::max(0.05, max_x - min_x);
  have_cached_footprint_ = true;
  footprint_dirty_ = false;
  return true;
}

// 2.11：车体轮廓摆到 (x,y,yaw)，判定交给 nav2 的 CostmapTopicCollisionChecker
bool InsertGarbagePose::isCollisionFreeAtPose(
  double x, double y, double yaw, std::string * reason,
  bool fetch_costmap_and_footprint) const
{
  if (!ensureCachedFootprint(reason)) {
    return false;
  }
  if (!collision_checker_) {
    if (reason) {
      *reason = "no collision checker";
    }
    return false;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = yaw;
  if (!collision_checker_->isCollisionFree(pose, fetch_costmap_and_footprint)) {
    if (reason) {
      *reason = "footprint collision";
    }
    return false;
  }
  return true;
}

// 2.11：车体轮廓沿 (x0,y0)->(x1,y1) 按车长步进摆放，扫出一条长条矩形走廊。
// 垃圾点检查、延长点检查、线段检查都用这一条，步长取 footprint 的车长。
bool InsertGarbagePose::isFootprintSweepClear(
  double x0, double y0, double x1, double y1, std::string * reason) const
{
  if (!ensureCachedFootprint(reason)) {
    return false;
  }

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double yaw = (len < 1e-9) ? 0.0 : std::atan2(dy, dx);
  double step = 0.5;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    step = std::max(0.05, cached_robot_length_m_);
  }

  if (!isCollisionFreeAtPose(x0, y0, yaw, reason, true)) {
    return false;
  }
  if (len < 1e-9) {
    return true;
  }

  const double ux = dx / len;
  const double uy = dy / len;
  for (double s = step; s < len - 1e-9; s += step) {
    if (!isCollisionFreeAtPose(x0 + ux * s, y0 + uy * s, yaw, reason, false)) {
      return false;
    }
  }
  if (!isCollisionFreeAtPose(x1, y1, yaw, reason, false)) {
    return false;
  }
  return true;
}

bool InsertGarbagePose::findNearestObstaclePixel(
  double x, double y, double * ox, double * oy)
{
  if (!ox || !oy || !costmap_sub_) {
    return false;
  }

  // 2.13.1：在全局代价图上找最近致命障碍格
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception &) {
    return false;
  }
  if (!costmap) {
    return false;
  }

  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!costmap->worldToMap(x, y, gx, gy)) {
    return false;
  }

  const double resolution = costmap->getResolution();
  if (resolution <= 0.0) {
    return false;
  }
  const double search_radius_m = std::max(1.0, std::fabs(garbage_extend_m_) + 0.5);
  const int r_cells = std::max(1, static_cast<int>(std::ceil(search_radius_m / resolution)));
  const double search_r2 = search_radius_m * search_radius_m;
  const int width = static_cast<int>(costmap->getSizeInCellsX());
  const int height = static_cast<int>(costmap->getSizeInCellsY());
  const int gx_i = static_cast<int>(gx);
  const int gy_i = static_cast<int>(gy);

  bool found = false;
  double best_d2 = std::numeric_limits<double>::infinity();
  double best_cx = 0.0;
  double best_cy = 0.0;

  const int mx0 = std::max(0, gx_i - r_cells);
  const int mx1 = std::min(width - 1, gx_i + r_cells);
  const int my0 = std::max(0, gy_i - r_cells);
  const int my1 = std::min(height - 1, gy_i + r_cells);
  for (int my = my0; my <= my1; ++my) {
    for (int mx = mx0; mx <= mx1; ++mx) {
      if (costmap->getCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my)) !=
        nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap->mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const double d2 = (wx - x) * (wx - x) + (wy - y) * (wy - y);
      if (d2 <= search_r2 && d2 < best_d2) {
        best_d2 = d2;
        best_cx = wx;
        best_cy = wy;
        found = true;
      }
    }
  }
  if (!found) {
    return false;
  }
  *ox = best_cx;
  *oy = best_cy;
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

// 2.12.6 新种子与 garbage_list_ 里已有种子的距离小于合堆半径就算重复
bool InsertGarbagePose::isDuplicateOfKept(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const GarbageList & kept) const
{
  const double x = garbage.pose.pose.position.x;
  const double y = garbage.pose.pose.position.y;
  const double r = std::max(0.0, garbage_merge_radius_m_);
  const double thresh2 = r * r;

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

// 从 from 指向垃圾再伸出 extend_m，作为这一堆的延长点
void extendBeyondGarbage(
  double from_x, double from_y, double gx, double gy, double extend_m,
  double & ex, double & ey, double & yaw)
{
  const double dx = gx - from_x;
  const double dy = gy - from_y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {
    ex = gx;
    ey = gy;
    yaw = 0.0;
    return;
  }
  yaw = std::atan2(dy, dx);
  const double step = std::max(0.0, extend_m);
  ex = gx + step * std::cos(yaw);
  ey = gy + step * std::sin(yaw);
}

// 一条中间顺序的得分：延长点之间的路程 + 折到 [-pi, pi] 的转角，不按最大值归一化
double routeExtendScore(
  const InsertGarbagePose::GarbageList & garbage_list,
  const std::vector<std::size_t> & order,
  double start_x, double start_y, double start_yaw,
  double extend_m, double w_dist, double w_turn)
{
  double px = start_x;
  double py = start_y;
  double heading = start_yaw;
  double total_dist = 0.0;
  double total_yaw = 0.0;
  for (const std::size_t idx : order) {
    if (idx >= garbage_list.size()) {
      continue;
    }
    const double gx = garbage_list[idx].pose.pose.position.x;
    const double gy = garbage_list[idx].pose.pose.position.y;
    double ex = gx;
    double ey = gy;
    double yaw = heading;
    extendBeyondGarbage(px, py, gx, gy, extend_m, ex, ey, yaw);
    total_dist += std::hypot(ex - px, ey - py);
    total_yaw += std::fabs(wrapAngleRad(yaw - heading));
    px = ex;
    py = ey;
    heading = yaw;
  }
  return w_dist * total_dist + w_turn * total_yaw;
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
  double robot_yaw,
  const capella_ros_msg::msg::GarbageDetect * locked_last)
{
  const std::size_t n = garbage_list.size();
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    order[i] = i;
  }
  if (n == 0) {
    return order;
  }

  const double extend_m = std::fabs(garbage_extend_m_);
  auto scoreOf = [&](const std::vector<std::size_t> & middle) {
    GarbageList scored;
    scored.reserve(middle.size() + (locked_last ? 1u : 0u));
    std::vector<std::size_t> scored_idx;
    scored_idx.reserve(scored.capacity());
    for (const std::size_t k : middle) {
      if (k < garbage_list.size()) {
        scored_idx.push_back(scored.size());
        scored.push_back(garbage_list[k]);
      }
    }
    if (locked_last) {
      scored_idx.push_back(scored.size());
      scored.push_back(*locked_last);
    }
    return routeExtendScore(
      scored, scored_idx, robot_x, robot_y, robot_yaw,
      extend_m, sweep_dist_weight_, sweep_turn_weight_);
  };

  if (n <= 1) {
    return order;
  }

  std::vector<std::size_t> best = order;
  if (n <= kSweepBruteMaxN) {
    std::vector<std::size_t> perm = order;
    double best_score = std::numeric_limits<double>::infinity();
    do {
      const double score = scoreOf(perm);
      if (score < best_score - 1e-9) {
        best_score = score;
        best = perm;
      }
    } while (std::next_permutation(perm.begin(), perm.end()));
  } else {
    std::vector<std::size_t> remaining = order;
    std::vector<std::size_t> greedy;
    greedy.reserve(n);
    double cur_x = robot_x;
    double cur_y = robot_y;
    double cur_yaw = robot_yaw;
    while (!remaining.empty()) {
      std::size_t best_pos = 0;
      double best_step = std::numeric_limits<double>::infinity();
      for (std::size_t p = 0; p < remaining.size(); ++p) {
        GarbageList step;
        std::vector<std::size_t> step_idx{0};
        step.push_back(garbage_list[remaining[p]]);
        if (locked_last) {
          step_idx.push_back(1);
          step.push_back(*locked_last);
        }
        const double score = routeExtendScore(
          step, step_idx, cur_x, cur_y, cur_yaw,
          extend_m, sweep_dist_weight_, sweep_turn_weight_);
        if (score < best_step - 1e-9) {
          best_step = score;
          best_pos = p;
        }
      }
      const std::size_t chosen = remaining[best_pos];
      greedy.push_back(chosen);
      remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_pos));
      const double gx = garbage_list[chosen].pose.pose.position.x;
      const double gy = garbage_list[chosen].pose.pose.position.y;
      extendBeyondGarbage(cur_x, cur_y, gx, gy, extend_m, cur_x, cur_y, cur_yaw);
    }
    best = std::move(greedy);
  }

  return best;
}

void InsertGarbagePose::reorderNearestFirstThenSweep(
  double robot_x, double robot_y, double robot_yaw)
{
  getInput("sweep_dist_weight", sweep_dist_weight_);
  getInput("sweep_turn_weight", sweep_turn_weight_);
  trimGarbageListToCap(robot_x, robot_y);
  const std::size_t n = garbage_list_.size();
  if (n <= 1) {
    if (n == 1) {
      syncLastSweepXyFromList();
    }
    return;
  }

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  auto dist2ToRobot = [&](std::size_t i) {
    return squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
  };
  // base_link 的 x：正值在车头前方
  auto baseX = [&](std::size_t i) {
    const double dx = garbage_list_[i].pose.pose.position.x - robot_x;
    const double dy = garbage_list_[i].pose.pose.position.y - robot_y;
    return dx * c + dy * s;
  };

  bool have_front = false;
  std::size_t first_idx = 0;
  double best_front_d2 = std::numeric_limits<double>::infinity();
  double best_any_d2 = std::numeric_limits<double>::infinity();
  std::size_t nearest_idx = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double d2 = dist2ToRobot(i);
    if (d2 < best_any_d2) {
      best_any_d2 = d2;
      nearest_idx = i;
    }
    if (baseX(i) > 0.0 && d2 < best_front_d2) {
      best_front_d2 = d2;
      first_idx = i;
      have_front = true;
    }
  }
  if (!have_front) {
    first_idx = nearest_idx;
  }

  // 2.7.1：每堆各自在原始 input_goals 副本上跑 2.5。有延长点则同一副本先 G 后 E。
  const Goals goals = receiveGoals();
  geometry_msgs::msg::PoseStamped robot_pose_for_clip;
  robot_pose_for_clip.header.frame_id = global_frame_;
  robot_pose_for_clip.pose.position.x = robot_x;
  robot_pose_for_clip.pose.position.y = robot_y;
  robot_pose_for_clip.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(robot_yaw);
  const double extend_m = std::fabs(garbage_extend_m_);
  auto clipReach = [&](double gx, double gy, std::size_t & reach) -> bool {
    std::vector<std::pair<double, double>> refs;
    refs.emplace_back(gx, gy);
    if (extend_m > 1e-9) {
      double ex = gx;
      double ey = gy;
      double yaw = robot_yaw;
      extendBeyondGarbage(robot_x, robot_y, gx, gy, extend_m, ex, ey, yaw);
      if (std::hypot(ex - gx, ey - gy) > 1e-6) {
        refs.emplace_back(ex, ey);
      }
    }
    InsertInfo clip_info;
    clip_info.valid = true;
    clip_info.goals = goals;
    clip_info.robot_pose = robot_pose_for_clip;
    std::set<std::size_t> del;
    clipReferencesInOrder(
      goals, robot_pose_for_clip, refs, del, clip_info, false);
    if (del.empty()) {
      return false;
    }
    reach = *del.rbegin();
    return true;
  };

  bool have_last = false;
  std::size_t last_idx = first_idx;
  std::size_t best_path_i = 0;
  double best_last_d2 = -1.0;
  bool have_reach = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (i == first_idx) {
      continue;
    }
    std::size_t reach = 0;
    const bool ok = clipReach(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      reach);
    const double d2 = dist2ToRobot(i);
    if (!ok) {
      continue;
    }
    if (!have_reach || reach > best_path_i ||
      (reach == best_path_i && d2 > best_last_d2))
    {
      have_reach = true;
      have_last = true;
      last_idx = i;
      best_path_i = reach;
      best_last_d2 = d2;
    }
  }
  if (!have_reach) {
    for (std::size_t i = 0; i < n; ++i) {
      if (i == first_idx) {
        continue;
      }
      const double d2 = dist2ToRobot(i);
      if (!have_last || d2 > best_last_d2) {
        have_last = true;
        last_idx = i;
        best_last_d2 = d2;
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: no clip reach, last pile falls back to farthest");
  }

  GarbageList middle;
  middle.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i == first_idx || (have_last && i == last_idx)) {
      continue;
    }
    middle.push_back(garbage_list_[i]);
  }

  const double fx = garbage_list_[first_idx].pose.pose.position.x;
  const double fy = garbage_list_[first_idx].pose.pose.position.y;
  double start_x = fx;
  double start_y = fy;
  double start_yaw = robot_yaw;
  extendBeyondGarbage(
    robot_x, robot_y, fx, fy, std::fabs(garbage_extend_m_),
    start_x, start_y, start_yaw);

  GarbageList reordered;
  reordered.reserve(n);
  reordered.push_back(garbage_list_[first_idx]);
  if (!middle.empty()) {
    const capella_ros_msg::msg::GarbageDetect * locked_last =
      have_last ? &garbage_list_[last_idx] : nullptr;
    const auto sub = computeSweepOrder(
      middle, start_x, start_y, start_yaw, locked_last);
    for (const std::size_t k : sub) {
      if (k < middle.size()) {
        reordered.push_back(middle[k]);
      }
    }
  }
  if (have_last) {
    reordered.push_back(garbage_list_[last_idx]);
  }

  garbage_list_ = std::move(reordered);
  syncLastSweepXyFromList();
  logGarbageListState("sweep reorder");
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: sweep order front=%d first=(%.2f, %.2f) last=(%.2f, %.2f) n=%zu",
    have_front ? 1 : 0,
    garbage_list_.front().pose.pose.position.x,
    garbage_list_.front().pose.pose.position.y,
    garbage_list_.back().pose.pose.position.x,
    garbage_list_.back().pose.pose.position.y,
    garbage_list_.size());
}

// 中途重排会直接拼表，绕过 tryInsertPreferCloserToRobot 的上限，这里统一收口
void InsertGarbagePose::trimGarbageListToCap(double robot_x, double robot_y)
{
  if (garbage_list_.size() <= kMaxGarbageSize) {
    return;
  }
  std::vector<std::size_t> by_dist(garbage_list_.size());
  for (std::size_t i = 0; i < by_dist.size(); ++i) {
    by_dist[i] = i;
  }
  std::stable_sort(
    by_dist.begin(), by_dist.end(),
    [this, robot_x, robot_y](std::size_t a, std::size_t b) {
      return squaredDistanceXY(
        garbage_list_[a].pose.pose.position.x,
        garbage_list_[a].pose.pose.position.y, robot_x, robot_y) <
      squaredDistanceXY(
        garbage_list_[b].pose.pose.position.x,
        garbage_list_[b].pose.pose.position.y, robot_x, robot_y);
    });
  // 留下离车最近的 kMaxGarbageSize 堆，保持它们原来的相对次序
  std::vector<std::size_t> keep(by_dist.begin(), by_dist.begin() + kMaxGarbageSize);
  std::sort(keep.begin(), keep.end());
  GarbageList trimmed;
  trimmed.reserve(kMaxGarbageSize);
  for (const std::size_t i : keep) {
    trimmed.push_back(garbage_list_[i]);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: garbage_list 超上限 %zu -> %zu, 丢掉离车最远的几堆",
    garbage_list_.size(), trimmed.size());
  garbage_list_ = std::move(trimmed);
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
  // 与 before 里某堆在合堆半径内就算同一堆，不是新堆
  const double merge_r = std::max(0.0, garbage_merge_radius_m_);
  const double thresh2 = merge_r * merge_r;
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

    // 2.12.4：车到垃圾做 footprint 长条，扫不过去的垃圾不进候选
    {
      std::string sweep_reason;
      if (!isFootprintSweepClear(robot_x, robot_y, gx, gy, &sweep_reason)) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为车到垃圾 footprint 长条不通过(%s), 丢弃",
          gx, gy, sweep_reason.c_str());
        eraseFromHistory(original);
        publishFailedSweepVisualization(robot_x, robot_y, gx, gy);
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

    // 2.12.7 已插入过的堆不再进候选，避免持续发布反复占队首
    if (isNearReachedGarbage(sx, sy)) {
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
    tmp_list_.clear();
  }
  garbage_list_.clear();
  logGarbageListState("new mission clear");
  active_piles_.clear();
  reached_garbage_xy_.clear();
  viz_obstacle_pixels_.clear();
  viz_obstacle_marker_count_ = 0;
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
  remembered_corner_xy_.clear();
  sweep_latch_g_num_ = 0;
  sweep_seen_g_ = false;
  sweep_seen_e_ = false;
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    single_pile_block_intake_ = false;
  }
  has_work_circle_ = false;
  mission_stamp_record_ = current_stamp;
  clearMissionVisualization();

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

bool InsertGarbagePose::getRobotFootprintInBase(
  std::vector<std::pair<double, double>> & local_xy) const
{
  local_xy.clear();
  if (!ensureCachedFootprint(nullptr)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  local_xy = cached_footprint_base_;
  return local_xy.size() >= 3;
}

int InsertGarbagePose::lookupStableGNum(double x, double y) const
{
  const double thresh2 = kPointMatchDistanceM * kPointMatchDistanceM;
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
  const double thresh2 = kPointMatchDistanceM * kPointMatchDistanceM;
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
  const double thresh2 = kPointMatchDistanceM * kPointMatchDistanceM;
  for (const auto & item : e_num_xy_) {
    if (squaredDistanceXY(item.first.first, item.first.second, x, y) < thresh2) {
      return item.second;
    }
  }
  return 0;
}

// 判断 goals 里某点是否为本节点写入的 G/E，而不是编号途经点
bool InsertGarbagePose::isUnindexedSentinelPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  return std::lround(pose_stamped_goal.pose.position.z) ==
         std::lround(kGarbageSentinelPoseZ);
}
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

bool InsertGarbagePose::isPointCoveredByRobotFootprint(
  double x, double y, const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  std::vector<std::pair<double, double>> local_xy;
  if (!getRobotFootprintInBase(local_xy)) {
    return false;
  }
  geometry_msgs::msg::Polygon poly;
  poly.points.reserve(local_xy.size());
  const double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  for (const auto & off : local_xy) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(rx + off.first * c - off.second * s);
    p.y = static_cast<float>(ry + off.first * s + off.second * c);
    p.z = 0.0f;
    poly.points.push_back(p);
  }
  return isPointInPolygon(x, y, poly);
}

bool InsertGarbagePose::isPastAlongDirection(
  double rx, double ry, double px, double py, double dir_x, double dir_y)
{
  const double len = std::hypot(dir_x, dir_y);
  if (len < 1e-6) {
    return false;
  }
  const double ux = dir_x / len;
  const double uy = dir_y / len;
  return (rx - px) * ux + (ry - py) * uy > 0.05;
}

bool InsertGarbagePose::eraseSweptSentinelsFromGoals(
  Goals & goals, const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::vector<std::pair<double, double>> keep_xy;
  int g_num = 0;
  if (!collectInProgressKeepXy(goals, &keep_xy, &g_num) || keep_xy.empty()) {
    sweep_latch_g_num_ = 0;
    sweep_seen_g_ = false;
    sweep_seen_e_ = false;
    return false;
  }

  if (g_num != sweep_latch_g_num_) {
    sweep_latch_g_num_ = g_num;
    sweep_seen_g_ = false;
    sweep_seen_e_ = false;
  }

  double gx = 0.0;
  double gy = 0.0;
  double ex = 0.0;
  double ey = 0.0;
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
  if (!have_g && !have_e) {
    gx = keep_xy.front().first;
    gy = keep_xy.front().second;
    have_g = true;
    if (keep_xy.size() > 1) {
      ex = keep_xy.back().first;
      ey = keep_xy.back().second;
      have_e = true;
    }
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  double dir_x = 0.0;
  double dir_y = 0.0;
  if (have_g && have_e) {
    dir_x = ex - gx;
    dir_y = ey - gy;
  } else {
    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    dir_x = std::cos(yaw);
    dir_y = std::sin(yaw);
  }

  if (!have_g && have_e) {
    sweep_seen_g_ = true;
  }

  const bool g_cover = have_g && isPointCoveredByRobotFootprint(gx, gy, robot_pose);
  const bool g_past = have_g && isPastAlongDirection(rx, ry, gx, gy, dir_x, dir_y);
  if (g_cover || g_past) {
    if (!sweep_seen_g_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: sweep reached G%d (%.2f, %.2f) cover=%d past=%d",
        g_num, gx, gy, g_cover ? 1 : 0, g_past ? 1 : 0);
    }
    sweep_seen_g_ = true;
  }

  if (have_e) {
    const bool e_cover = isPointCoveredByRobotFootprint(ex, ey, robot_pose);
    const bool e_past = isPastAlongDirection(rx, ry, ex, ey, dir_x, dir_y);
    if (e_cover || e_past) {
      if (!sweep_seen_e_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: sweep reached E%d (%.2f, %.2f) cover=%d past=%d",
          g_num, ex, ey, e_cover ? 1 : 0, e_past ? 1 : 0);
      }
      sweep_seen_e_ = true;
    }
  } else {
    sweep_seen_e_ = true;
  }

  if (!sweep_seen_g_ || !sweep_seen_e_) {
    return false;
  }

  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  auto isKeep = [&](double x, double y) {
    for (const auto & p : keep_xy) {
      if (squaredDistanceXY(p.first, p.second, x, y) < thresh2) {
        return true;
      }
    }
    return false;
  };

  Goals kept;
  kept.reserve(goals.size());
  std::size_t erased = 0;
  for (const auto & g : goals) {
    if (isUnindexedSentinelPoseZ(g) &&
      isKeep(g.pose.position.x, g.pose.position.y))
    {
      ++erased;
      continue;
    }
    kept.push_back(g);
  }
  if (erased == 0) {
    return false;
  }
  goals = std::move(kept);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: sweep done G%d, erase %zu sentinel(s) G/E from {goals}",
    g_num, erased);
  sweep_latch_g_num_ = 0;
  sweep_seen_g_ = false;
  sweep_seen_e_ = false;
  return true;
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

// 2.12.7 新垃圾落在已处理点的合堆半径内就不再作为候选
bool InsertGarbagePose::isNearReachedGarbage(double x, double y) const
{
  const double r = std::max(0.0, garbage_merge_radius_m_);
  const double thresh2 = r * r;
  for (const auto & reached : reached_garbage_xy_) {
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}

// 该 xy 是否就是某个已写入 goals 的 G/E 点：几何认点，不用合堆半径
bool InsertGarbagePose::isProtectedGarbageXy(double x, double y) const
{
  const double thresh2 = kPointMatchDistanceM * kPointMatchDistanceM;
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
  const double thresh2 = kPointMatchDistanceM * kPointMatchDistanceM;
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

  // 开扫了没：队首第一颗 z=-1 就是当前堆。扫完了没不在这里判，
  // 由 eraseSweptSentinelsFromGoals 按 footprint / 过 E 删哨兵。
  int g_num = 0;
  bool found_lead = false;
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
    found_lead = true;
    break;
  }
  if (!found_lead) {
    return false;
  }
  if (g_num > 0) {
    for (const auto & item : g_num_xy_) {
      if (item.second != g_num) {
        continue;
      }
      if (findUnindexedSentinelIndex(
          goals, item.first.first, item.first.second, nullptr))
      {
        add(item.first.first, item.first.second);
      }
    }
    for (const auto & item : e_num_xy_) {
      if (item.second != g_num) {
        continue;
      }
      if (findUnindexedSentinelIndex(
          goals, item.first.first, item.first.second, nullptr))
      {
        add(item.first.first, item.first.second);
      }
    }
  }
  *keep_g_num = g_num;
  return !keep_xy->empty();
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

// true=非角点，false=角点。z=-1 不能当角点；只在非 z=-1 点上算夹角，

bool InsertGarbagePose::isGoalNotCorner(
  const Goals & goals,
  std::size_t idx,
  double robot_x, double robot_y) const
{
  (void)robot_x;
  (void)robot_y;
  if (goals.empty() || idx >= goals.size()) {
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
  // 非 z=-1 里的最后一点：收住最后一段
  if (next_i >= goals.size()) {
    return false;
  }
  if (prev_i < 0 || goals.size() < 3) {
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

// 剩余队列第一个非 z=-1 点：工字当前长边队首，不用欧氏最近
std::size_t InsertGarbagePose::ordinaryQueueHead(const Goals & goals) const
{
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      return i;
    }
  }
  return goals.size();
}

bool InsertGarbagePose::isRememberedCorner(double x, double y) const
{
  constexpr double kTol2 = 0.08 * 0.08;
  for (const auto & c : remembered_corner_xy_) {
    if (squaredDistanceXY(c.first, c.second, x, y) < kTol2) {
      return true;
    }
  }
  return false;
}

void InsertGarbagePose::rememberCornerXy(double x, double y) const
{
  if (isRememberedCorner(x, y)) {
    return;
  }
  remembered_corner_xy_.emplace_back(x, y);
}

void InsertGarbagePose::forgetCornerXy(double x, double y) const
{
  constexpr double kTol2 = 0.08 * 0.08;
  remembered_corner_xy_.erase(
    std::remove_if(
      remembered_corner_xy_.begin(), remembered_corner_xy_.end(),
      [x, y](const std::pair<double, double> & c) {
        return squaredDistanceXY(c.first, c.second, x, y) < kTol2;
      }),
    remembered_corner_xy_.end());
}

bool InsertGarbagePose::robotEnteredNextSide(
  const Goals & goals, std::size_t head, double robot_x, double robot_y) const
{
  if (head + 1 >= goals.size()) {
    return false;
  }
  double t = 0.0;
  const double d2 = squaredDistancePointToSegment(
    robot_x, robot_y,
    goals[head].pose.position.x, goals[head].pose.position.y,
    goals[head + 1].pose.position.x, goals[head + 1].pose.position.y,
    &t);
  if (d2 > 1.5 * 1.5) {
    return false;
  }
  const double seg = std::sqrt(squaredDistanceXY(
    goals[head].pose.position.x, goals[head].pose.position.y,
    goals[head + 1].pose.position.x, goals[head + 1].pose.position.y));
  return t * seg > 0.25;
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
  const double hx = goals[head].pose.position.x;
  const double hy = goals[head].pose.position.y;
  if (isRememberedCorner(hx, hy)) {
    if (robotEnteredNextSide(goals, head, robot_x, robot_y)) {
      forgetCornerXy(hx, hy);
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: corner (%.2f, %.2f) passed, open next side",
        hx, hy);
    } else {
      corner_idx = head;
      return true;
    }
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
      const bool closed_side =
        info.goalc_idx == head_idx &&
        isRememberedCorner(
          info.goala.pose.position.x, info.goala.pose.position.y);
      if (!closed_side &&
        squaredDistanceXY(
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
      if (closed_side) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: hold corner as C (%.2f, %.2f), robot not on next side",
          info.goala.pose.position.x, info.goala.pose.position.y);
        ac_ok = true;
      } else if (squaredDistanceXY(
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
  const double min_from_m = kMinExtendFromDistM;
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

namespace
{

struct ClipNode
{
  std::size_t orig{0};
  geometry_msgs::msg::PoseStamped pose;
};

}  // namespace

// 多个参考点按顺序在同一份副本上跑 2.5。副本里会删点并重算角点，
// delete_idx 记的是调用方 goals 里的原始下标。
void InsertGarbagePose::clipReferencesInOrder(
  const Goals & goals,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<std::pair<double, double>> & refs,
  std::set<std::size_t> & delete_idx,
  InsertInfo & info,
  bool commit_memory)
{
  info.clip_rounds.clear();
  info.corners_kept_xy.clear();
  info.hit_mid_case = false;
  info.hit_forward_case = false;
  if (goals.size() < 2 || refs.empty()) {
    return;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("clip_extend_m", clip_extend_m_);

  double nearest_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
    nearest_d2 = std::min(
      nearest_d2,
      squaredDistancePointToSegment(
        rx, ry,
        goals[i].pose.position.x, goals[i].pose.position.y,
        goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
  }
  if (std::sqrt(nearest_d2) > head_delete_robot_dist_m_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: robot %.2fm from input_goals > %.2fm, skip all deletes",
      std::sqrt(nearest_d2), head_delete_robot_dist_m_);
    return;
  }

  std::vector<ClipNode> nodes;
  nodes.reserve(goals.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    nodes.push_back(ClipNode{i, goals[i]});
  }

  constexpr double kEps = 1e-6;
  const double clip_m = std::max(0.0, clip_extend_m_);
  int round_i = 0;

  auto canDelete = [this, &nodes](std::size_t node_i) {
    const auto & pose = nodes[node_i].pose;
    if (isUnindexedSentinelPoseZ(pose)) {
      return false;
    }
    if (isProtectedGarbageXy(pose.pose.position.x, pose.pose.position.y)) {
      return false;
    }
    return true;
  };

  auto dropNodes = [&](const std::vector<std::size_t> & origs) {
    if (origs.empty()) {
      return;
    }
    std::set<std::size_t> drop(origs.begin(), origs.end());
    std::vector<ClipNode> kept;
    kept.reserve(nodes.size());
    for (const auto & node : nodes) {
      if (drop.count(node.orig) == 0) {
        kept.push_back(node);
      } else {
        delete_idx.insert(node.orig);
      }
    }
    nodes.swap(kept);
  };

  for (const auto & ref : refs) {
    const double ref_x = ref.first;
    const double ref_y = ref.second;
    const std::size_t round_limit = nodes.size() + 1;
    for (std::size_t round = 0; round < round_limit; ++round) {
      Goals live;
      live.reserve(nodes.size());
      for (const auto & node : nodes) {
        live.push_back(node.pose);
      }
      std::vector<std::size_t> ordinary;
      ordinary.reserve(live.size());
      for (std::size_t i = 0; i < live.size(); ++i) {
        if (!isUnindexedSentinelPoseZ(live[i])) {
          ordinary.push_back(i);
        }
      }
      if (ordinary.size() < 2) {
        break;
      }

      // 当前边：第一个非 z=-1 → 其后第一个角点。队首本身不当这条边的终点。
      const std::size_t H = ordinary.front();
      std::size_t C = ordinary.back();
      for (std::size_t k = 1; k + 1 < ordinary.size(); ++k) {
        if (!isGoalNotCorner(live, ordinary[k], rx, ry)) {
          C = ordinary[k];
          break;
        }
      }

      const double hx = live[H].pose.position.x;
      const double hy = live[H].pose.position.y;
      const double cx = live[C].pose.position.x;
      const double cy = live[C].pose.position.y;
      const double seg_len = std::hypot(cx - hx, cy - hy);
      if (seg_len < 1e-9) {
        break;
      }
      double fx = 0.0;
      double fy = 0.0;
      projectPointToInfiniteLine(ref_x, ref_y, hx, hy, cx, cy, fx, fy);
      const double t = lineParameterT(fx, fy, hx, hy, cx, cy);

      InsertInfo::ClipRound clip_round;
      clip_round.round_i = ++round_i;
      clip_round.ax = hx;
      clip_round.ay = hy;
      clip_round.cx = cx;
      clip_round.cy = cy;
      clip_round.fx = fx;
      clip_round.fy = fy;
      clip_round.t_d = t;
      info.clip_rounds.push_back(clip_round);
      info.goala = live[H];
      info.goalc = live[C];
      info.goalc_idx = nodes[C].orig;

      if (t < -kEps) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: clip reverse t=%.3f, skip H=(%.2f, %.2f) C=(%.2f, %.2f)",
          t, hx, hy, cx, cy);
        break;
      }

      std::vector<std::size_t> drop_orig;
      if (t > 1.0 + kEps) {
        info.hit_forward_case = true;
        for (std::size_t j = H; j < C; ++j) {
          if (canDelete(j)) {
            drop_orig.push_back(nodes[j].orig);
          }
        }
        info.corners_kept_xy.emplace_back(cx, cy);
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: clip beyond t=%.3f, drop %zu before corner (%.2f, %.2f)",
          t, drop_orig.size(), cx, cy);
        if (drop_orig.empty()) {
          break;
        }
        dropNodes(drop_orig);
        continue;
      }

      info.hit_mid_case = true;
      // 垃圾垂足和路径点都投到 H->C 方向上量，避免弦长和折线弧长混着比
      const double ux = (cx - hx) / seg_len;
      const double uy = (cy - hy) / seg_len;
      const double cut = std::max(0.0, t) * seg_len + clip_m;
      for (std::size_t j = H; j < C; ++j) {
        const double sj =
          (live[j].pose.position.x - hx) * ux + (live[j].pose.position.y - hy) * uy;
        if (sj >= cut - 1e-9) {
          break;
        }
        if (canDelete(j)) {
          drop_orig.push_back(nodes[j].orig);
        }
      }
      info.corners_kept_xy.emplace_back(cx, cy);
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: clip on-seg t=%.3f H=%zu C=%zu cut=%.2f "
        "drop=%zu ref=(%.2f, %.2f), stop at corner",
        t, nodes[H].orig, nodes[C].orig, cut, drop_orig.size(), ref_x, ref_y);
      dropNodes(drop_orig);
      break;
    }
  }

  if (commit_memory) {
    for (const auto & xy : info.corners_kept_xy) {
      rememberCornerXy(xy.first, xy.second);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: clip refs=%zu rounds=%zu n_del=%zu N=%zu",
    refs.size(), info.clip_rounds.size(), delete_idx.size(), goals.size());
}

InsertGarbagePose::Goals InsertGarbagePose::clipGoalsNearGarbage(InsertInfo & info)
{
  const Goals & goals = info.goals;
  info.goaltotal.clear();
  info.clip_rounds.clear();
  info.corners_kept_xy.clear();
  info.hit_mid_case = false;
  info.hit_forward_case = false;
  if (goals.size() < 2) {
    return goals;
  }

  std::set<std::size_t> delete_idx;
  const std::vector<std::pair<double, double>> refs = {{
    info.garbage.pose.pose.position.x,
    info.garbage.pose.pose.position.y}};
  clipReferencesInOrder(
    goals, info.robot_pose, refs, delete_idx, info, true);

  info.goaltotal.clear();
  info.goaltotal.reserve(delete_idx.size());
  Goals out;
  out.reserve(goals.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (delete_idx.count(i) == 0) {
      out.push_back(goals[i]);
    } else {
      info.goaltotal.push_back(goals[i]);
    }
  }
  return out;
}


// 插入真实垃圾、统一时间戳；
InsertGarbagePose::Goals InsertGarbagePose::insertGarbageIntoGoals(InsertInfo & info)
{
  getInput("extend_max_yaw_deg", extend_max_yaw_deg_);
  getInput("extend_step_yaw_deg", extend_step_yaw_deg_);
  const double yaw_max_deg = std::max(0.0, extend_max_yaw_deg_);
  const double yaw_step_deg = std::max(1.0, extend_step_yaw_deg_);
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
  Goals work = path;
  InsertInfo frozen = info;
  bool have_clip_base = false;
  if (!prefix.empty() && path.size() >= 2) {
    InsertInfo path_info = gatherInsertInfo(
      path, info.robot_pose,
      saved_garbage.pose.pose.position.x,
      saved_garbage.pose.pose.position.y);
    if (path_info.valid) {
      path_info.path_yaw = saved_yaw;
      path_info.garbage = saved_garbage;
      path_info.goals = path;
      frozen = path_info;
      have_clip_base = true;
      info.goala = path_info.goala;
      info.goalc = path_info.goalc;
      info.goalc_idx = path_info.goalc_idx;
      info.goald_x = path_info.goald_x;
      info.goald_y = path_info.goald_y;
    }
  } else if (path.size() >= 2 && info.valid) {
    frozen.goals = path;
    frozen.path_yaw = saved_yaw;
    frozen.garbage = saved_garbage;
    have_clip_base = true;
  }
  Goals out = work;
  info.path_yaw = saved_yaw;
  info.garbage = saved_garbage;

  // G-E 接到已插入哨兵之后、删点剩下的原路径队首
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

  // 沿 path_yaw 插 E；默认方向不通则左右扇形扫，仍不通只留 G
  const double extend_param = garbage_extend_m_;
  double extend_m = 0.0;
  bool add_extend = false;
  geometry_msgs::msg::PoseStamped extend_pose = garbage_pose;
  const double gx = garbage_pose.pose.position.x;
  const double gy = garbage_pose.pose.position.y;

  auto setExtendPose = [&](double yaw, double d) {
    extend_pose.pose.position.x = gx + d * std::cos(yaw);
    extend_pose.pose.position.y = gy + d * std::sin(yaw);
    extend_pose.pose.position.z = kGarbageSentinelPoseZ;
  };

  // 2.11：G 到 E 做 footprint 长条
  auto corridorClear = [&](std::string * reason) {
    return isFootprintSweepClear(
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
    std::string neg_reason;
    if (!corridorClear(&neg_reason)) {
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
      const double yaw0 = info.path_yaw;
      for (double step = yaw_step_deg;
        step <= yaw_max_deg + 1e-6 && !add_extend;
        step += yaw_step_deg)
      {
        for (const double sign : {1.0, -1.0}) {
          const double yaw_try = yaw0 + sign * step * M_PI / 180.0;
          setExtendPose(yaw_try, extend_m);
          std::string sweep_reason;
          if (!corridorClear(&sweep_reason)) {
            continue;
          }
          applyExtendYaw(yaw_try);
          add_extend = true;
          RCLCPP_INFO(
            node_->get_logger(),
            "InsertGarbagePose: extend fan %+g deg after default blocked (%s) "
            "E=(%.2f, %.2f)",
            sign * step, extend_reason.c_str(),
            extend_pose.pose.position.x, extend_pose.pose.position.y);
          break;
        }
      }
      if (!add_extend) {
        skipE(
          "默认G->E走廊(" + extend_reason + ")，±" +
          std::to_string(static_cast<int>(yaw_max_deg)) +
          "deg 扇形仍不通");
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

  if (have_clip_base && work.size() >= 2) {
    // G、E 按行驶顺序在同一份副本上跑。后一个看得到前一个删完后的新角点。
    // 下标攒齐后对 work 删一次。
    std::vector<std::pair<double, double>> refs;
    if (add_extend && extend_m < 0.0) {
      refs.emplace_back(
        extend_pose.pose.position.x, extend_pose.pose.position.y);
    }
    refs.emplace_back(gx, gy);
    if (add_extend && extend_m >= 0.0) {
      refs.emplace_back(
        extend_pose.pose.position.x, extend_pose.pose.position.y);
    }
    InsertInfo cinfo = frozen;
    cinfo.goals = work;
    std::set<std::size_t> del;
    clipReferencesInOrder(work, info.robot_pose, refs, del, cinfo, true);
    info.clip_rounds = std::move(cinfo.clip_rounds);
    info.corners_kept_xy = std::move(cinfo.corners_kept_xy);
    info.hit_mid_case = cinfo.hit_mid_case;
    info.hit_forward_case = cinfo.hit_forward_case;
    if (cinfo.goalc_idx < work.size()) {
      info.goalc = work[cinfo.goalc_idx];
      info.goalc_idx = cinfo.goalc_idx;
    }
    info.goala = cinfo.goala;
    info.goaltotal.clear();
    info.goaltotal.reserve(del.size());
    Goals clipped;
    clipped.reserve(work.size());
    for (std::size_t i = 0; i < work.size(); ++i) {
      if (del.count(i) == 0) {
        clipped.push_back(work[i]);
      } else {
        info.goaltotal.push_back(work[i]);
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: union delete %zu refs=%zu, path %zu -> %zu",
      del.size(), refs.size(), work.size(), clipped.size());
    out = std::move(clipped);
  }

  Goals suffix;
  suffix.assign(
    out.begin() + static_cast<std::ptrdiff_t>(resume_from), out.end());

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
    "(mid=%d forward=%d)",
    resume_from, out.size(),
    info.hit_mid_case ? 1 : 0, info.hit_forward_case ? 1 : 0);

  return out;
}

bool InsertGarbagePose::isFootprintClearAtPose(
  double x, double y, double yaw, std::string * reason) const
{
  return isCollisionFreeAtPose(x, y, yaw, reason, true);
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
  getInput("extend_max_yaw_deg", extend_max_yaw_deg_);
  getInput("extend_step_yaw_deg", extend_step_yaw_deg_);
  const double yaw_max_deg = std::max(0.0, extend_max_yaw_deg_);
  const double yaw_step_deg = std::max(1.0, extend_step_yaw_deg_);

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
  // 2.13.1：entry 到 extend 做 2.11.1 footprint 长条
  auto deLineClear = [&](double x0, double y0, double x1, double y1) {
    return isFootprintSweepClear(
      x0 + off * nx, y0 + off * ny,
      x1 + off * nx, y1 + off * ny,
      nullptr);
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
      for (double step_deg = yaw_step_deg;
        step_deg <= yaw_max_deg + 1e-6 && !e_ok;
        step_deg += yaw_step_deg)
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
      for (double step_deg = yaw_step_deg;
        step_deg <= yaw_max_deg + 1e-6 && !d_ok;
        step_deg += yaw_step_deg)
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

  // 删点已在 clip 里做完，贴墙链接到剩余路径队首
  const std::size_t resume_from = 0;

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

void InsertGarbagePose::resetVisualizationState()
{
  viz_tracks_.clear();
  viz_pending_marker_count_ = 0;
  viz_have_head_ = false;
  viz_have_corner_ = false;
  viz_have_fail_strip_ = false;
  viz_pile_count_ = 0;
  viz_footprint_fail_count_ = 0;
}

// 新任务 / 工作圈取消：清空本话题上全部 Marker
void InsertGarbagePose::clearMissionVisualization()
{
  resetVisualizationState();
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

void InsertGarbagePose::appendDeleteMarker(
  visualization_msgs::msg::MarkerArray & arr, const std::string & ns, int id)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = node_->now();
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::msg::Marker::DELETE;
  arr.markers.push_back(m);
}

void InsertGarbagePose::appendFootprintStripMarkers(
  visualization_msgs::msg::MarkerArray & arr,
  const std::string & ns, int id_base,
  double x0, double y0, double x1, double y1,
  float r, float g, float b, float a_line, float a_fill)
{
  std::vector<std::pair<double, double>> fp;
  if (!getRobotFootprintInBase(fp) || fp.size() < 3) {
    return;
  }

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double yaw = (len < 1e-9) ? 0.0 : std::atan2(dy, dx);
  double step = 0.5;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    step = std::max(0.05, cached_robot_length_m_);
  }

  double half_w = 0.05;
  for (const auto & off : fp) {
    half_w = std::max(half_w, std::fabs(off.second));
  }

  const rclcpp::Time stamp = node_->now();
  auto makeBase = [&](int id, int type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    return m;
  };

  if (len >= 1e-9) {
    const double nx = -dy / len;
    const double ny = dx / len;
    auto corner = [&](double x, double y, double side) {
      geometry_msgs::msg::Point p;
      p.x = x + side * nx * half_w;
      p.y = y + side * ny * half_w;
      p.z = 0.03;
      return p;
    };
    const auto sl = corner(x0, y0, 1.0);
    const auto sr = corner(x0, y0, -1.0);
    const auto el = corner(x1, y1, 1.0);
    const auto er = corner(x1, y1, -1.0);
    auto fill = makeBase(id_base + 1, visualization_msgs::msg::Marker::TRIANGLE_LIST);
    fill.scale.x = 1.0;
    fill.scale.y = 1.0;
    fill.scale.z = 1.0;
    fill.color.a = a_fill;
    fill.points = {sl, sr, el, sr, er, el};
    arr.markers.push_back(fill);
  }

  auto outline = makeBase(id_base, visualization_msgs::msg::Marker::LINE_LIST);
  outline.scale.x = 0.025;
  outline.color.a = a_line;
  auto addPose = [&](double x, double y) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> pts;
    pts.reserve(fp.size());
    for (const auto & off : fp) {
      geometry_msgs::msg::Point p;
      p.x = x + off.first * c - off.second * s;
      p.y = y + off.first * s + off.second * c;
      p.z = 0.05;
      pts.push_back(p);
    }
    for (std::size_t i = 0; i < pts.size(); ++i) {
      outline.points.push_back(pts[i]);
      outline.points.push_back(pts[(i + 1) % pts.size()]);
    }
  };
  addPose(x0, y0);
  if (len >= 1e-9) {
    const double ux = dx / len;
    const double uy = dy / len;
    for (double s_pos = step; s_pos < len - 1e-9; s_pos += step) {
      addPose(x0 + ux * s_pos, y0 + uy * s_pos);
    }
    addPose(x1, y1);
  }
  if (!outline.points.empty()) {
    arr.markers.push_back(outline);
  }
}

void InsertGarbagePose::publishFailedSweepVisualization(
  double rx, double ry, double gx, double gy)
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  appendFootprintStripMarkers(
    arr, "footprint_strip_fail", 0, rx, ry, gx, gy,
    0.95f, 0.20f, 0.08f, 0.95f, 0.28f);

  visualization_msgs::msg::Marker ball;
  ball.header.frame_id = global_frame_;
  ball.header.stamp = node_->now();
  ball.ns = "footprint_strip_fail";
  ball.id = 2;
  ball.type = visualization_msgs::msg::Marker::SPHERE;
  ball.action = visualization_msgs::msg::Marker::ADD;
  ball.pose.position.x = gx;
  ball.pose.position.y = gy;
  ball.pose.position.z = 0.12;
  ball.pose.orientation.w = 1.0;
  ball.scale.x = ball.scale.y = ball.scale.z = 0.22;
  ball.color.r = 0.95f;
  ball.color.g = 0.10f;
  ball.color.b = 0.10f;
  ball.color.a = 1.0f;
  ball.lifetime = rclcpp::Duration::from_seconds(0.0);
  arr.markers.push_back(ball);

  viz_have_fail_strip_ = true;
  if (!arr.markers.empty()) {
    marker_pub_->publish(arr);
  }
}

void InsertGarbagePose::deletePileVisualization(int pile_num)
{
  if (!marker_pub_ || pile_num <= 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  const int base = pile_num * 10;
  appendDeleteMarker(arr, "garbage", base);
  appendDeleteMarker(arr, "garbage", base + 1);
  appendDeleteMarker(arr, "extend", base);
  appendDeleteMarker(arr, "extend", base + 1);
  appendDeleteMarker(arr, "extend", base + 2);
  appendDeleteMarker(arr, "footprint_strip", base);
  appendDeleteMarker(arr, "footprint_strip", base + 1);
  appendDeleteMarker(arr, "footprint_strip", base + 2);
  appendDeleteMarker(arr, "footprint_strip", base + 3);
  constexpr int kWallEdgeIdBase = 8000;
  constexpr int kWallEdgeIdSpan = 128;
  const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
  for (int i = 0; i < kWallEdgeIdSpan; ++i) {
    appendDeleteMarker(arr, "wall_edge_pts", wbase + i);
  }
  marker_pub_->publish(arr);
}

void InsertGarbagePose::pruneFinishedPileVisualization(const Goals & goals)
{
  if (viz_tracks_.empty()) {
    return;
  }
  std::vector<VizPileTrack> keep;
  keep.reserve(viz_tracks_.size());
  for (const auto & t : viz_tracks_) {
    const bool g_in = findUnindexedSentinelIndex(goals, t.gx, t.gy, nullptr);
    const bool e_in = t.has_e &&
      findUnindexedSentinelIndex(goals, t.ex, t.ey, nullptr);
    if (g_in || e_in) {
      keep.push_back(t);
      continue;
    }
    deletePileVisualization(t.pile_num);
  }
  viz_tracks_ = std::move(keep);
}

void InsertGarbagePose::publishPendingGarbageDots()
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    visualization_msgs::msg::Marker ball;
    ball.header.frame_id = global_frame_;
    ball.header.stamp = stamp;
    ball.ns = "garbage_pending";
    ball.id = static_cast<int>(i);
    ball.type = visualization_msgs::msg::Marker::SPHERE;
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.pose.position.x = garbage_list_[i].pose.pose.position.x;
    ball.pose.position.y = garbage_list_[i].pose.pose.position.y;
    ball.pose.position.z = 0.12;
    ball.pose.orientation.w = 1.0;
    ball.scale.x = ball.scale.y = ball.scale.z = 0.20;
    ball.color.r = 0.95f;
    ball.color.g = 0.10f;
    ball.color.b = 0.10f;
    ball.color.a = 1.0f;
    ball.lifetime = rclcpp::Duration::from_seconds(0.0);
    arr.markers.push_back(ball);
  }
  for (std::size_t i = garbage_list_.size(); i < viz_pending_marker_count_; ++i) {
    appendDeleteMarker(arr, "garbage_pending", static_cast<int>(i));
  }
  viz_pending_marker_count_ = garbage_list_.size();
  if (!arr.markers.empty()) {
    marker_pub_->publish(arr);
  }
}

void InsertGarbagePose::refreshClipAnchorVisualization(
  const Goals & goals, double robot_x, double robot_y)
{
  if (!marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();
  auto publishAnchor = [&](
    const char * ns, bool have, double x, double y, const char * label,
    float r, float g, float b)
  {
    if (!have) {
      appendDeleteMarker(arr, ns, 0);
      appendDeleteMarker(arr, ns, 1);
      return;
    }
    visualization_msgs::msg::Marker ball;
    ball.header.frame_id = global_frame_;
    ball.header.stamp = stamp;
    ball.ns = ns;
    ball.id = 0;
    ball.type = visualization_msgs::msg::Marker::SPHERE;
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.pose.position.x = x;
    ball.pose.position.y = y;
    ball.pose.position.z = 0.14;
    ball.pose.orientation.w = 1.0;
    ball.scale.x = ball.scale.y = ball.scale.z = 0.18;
    ball.color.r = r;
    ball.color.g = g;
    ball.color.b = b;
    ball.color.a = 1.0f;
    ball.lifetime = rclcpp::Duration::from_seconds(0.0);
    arr.markers.push_back(ball);

    visualization_msgs::msg::Marker text;
    text.header = ball.header;
    text.ns = ns;
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = x;
    text.pose.position.y = y;
    text.pose.position.z = 0.42;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.22;
    text.text = label;
    text.color = ball.color;
    text.lifetime = rclcpp::Duration::from_seconds(0.0);
    arr.markers.push_back(text);
  };

  const std::size_t head = ordinaryQueueHead(goals);
  bool have_h = head < goals.size();
  double hx = 0.0;
  double hy = 0.0;
  if (have_h) {
    hx = goals[head].pose.position.x;
    hy = goals[head].pose.position.y;
  }

  bool have_c = false;
  double cx = 0.0;
  double cy = 0.0;
  if (have_h) {
    if (isRememberedCorner(hx, hy) &&
      !robotEnteredNextSide(goals, head, robot_x, robot_y))
    {
      have_c = true;
      cx = hx;
      cy = hy;
    } else {
      std::size_t range_end = head;
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
      for (std::size_t i = head; i <= range_end && i < goals.size(); ++i) {
        if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
          continue;
        }
        if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
          have_c = true;
          cx = goals[i].pose.position.x;
          cy = goals[i].pose.position.y;
          break;
        }
      }
    }
  }

  publishAnchor("clip_head", have_h, hx, hy, "H", 0.55f, 0.95f, 0.50f);
  publishAnchor("clip_corner", have_c, cx, cy, "C", 0.02f, 0.40f, 0.10f);
  viz_have_head_ = have_h;
  viz_have_corner_ = have_c;
  if (!arr.markers.empty()) {
    marker_pub_->publish(arr);
  }
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

// 往 RViz 发本次插入：红 G、蓝 E、蓝虚线、footprint 长条
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
    resetVisualizationState();
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
  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;

  if (viz_have_fail_strip_) {
    appendDeleteMarker(arr, "footprint_strip_fail", 0);
    appendDeleteMarker(arr, "footprint_strip_fail", 1);
    appendDeleteMarker(arr, "footprint_strip_fail", 2);
    viz_have_fail_strip_ = false;
  }

  if (viz_accepted_garbage) {
    const int base = pile_num * 10;
    constexpr double kGarbageDotZM = 0.12;
    constexpr double kGarbageDotSizeM = 0.22;
    constexpr double kDashLenM = 0.12;
    constexpr double kGapLenM = 0.08;
    constexpr double kDashLineWidthM = 0.030;

    auto makeSolidDot = [&](const std::string & ns, int id, double x, double y,
        float r, float g, float b, double size)
    {
      auto dot = makeBase(ns, id, visualization_msgs::msg::Marker::SPHERE);
      dot.pose.position.x = x;
      dot.pose.position.y = y;
      dot.pose.position.z = kGarbageDotZM;
      dot.scale.x = size;
      dot.scale.y = size;
      dot.scale.z = size;
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

    arr.markers.push_back(
      makeSolidDot("garbage", base, gx, gy, 0.95f, 0.10f, 0.10f, kGarbageDotSizeM));

    auto t = makeBase("garbage", base + 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    t.pose.position.x = gx;
    t.pose.position.y = gy;
    t.pose.position.z = 0.42;
    t.scale.z = 0.22;
    {
      std::ostringstream oss;
      oss << "G" << pile_num;
      t.text = oss.str();
    }
    setColor(t, 0.95f, 0.10f, 0.10f);
    arr.markers.push_back(t);

    appendFootprintStripMarkers(
      arr, "footprint_strip", base, rx, ry, gx, gy,
      1.00f, 0.55f, 0.05f, 0.90f, 0.22f);

    if (info.extend_inserted) {
      const double ex = info.extend_x;
      const double ey = info.extend_y;

      auto ge_line = makeBase("extend", base + 2, visualization_msgs::msg::Marker::LINE_LIST);
      ge_line.scale.x = kDashLineWidthM;
      setColor(ge_line, 0.15f, 0.40f, 0.95f, 0.90f);
      appendDashedLine(ge_line, gx, gy, ex, ey);
      if (!ge_line.points.empty()) {
        arr.markers.push_back(ge_line);
      }

      arr.markers.push_back(
        makeSolidDot("extend", base, ex, ey, 0.15f, 0.40f, 0.95f, kGarbageDotSizeM));

      auto te = makeBase("extend", base + 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      te.pose.position.x = ex;
      te.pose.position.y = ey;
      te.pose.position.z = 0.42;
      te.scale.z = 0.22;
      {
        std::ostringstream oss;
        oss << "E" << pile_num;
        te.text = oss.str();
      }
      setColor(te, 0.15f, 0.40f, 0.95f);
      arr.markers.push_back(te);

      appendFootprintStripMarkers(
        arr, "footprint_strip", base + 2, gx, gy, ex, ey,
        0.15f, 0.45f, 0.95f, 0.85f, 0.18f);
    }

    if (info.wall_edge_inserted) {
      constexpr float kBlueR = 0.15f;
      constexpr float kBlueG = 0.40f;
      constexpr float kBlueB = 0.95f;
      constexpr double kMidDotSizeM = 0.07;
      constexpr int kWallEdgeIdBase = 8000;
      constexpr int kWallEdgeIdSpan = 128;
      const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
      int wid = 0;

      arr.markers.push_back(
        makeSolidDot(
          "wall_edge_pts", wbase + wid++, info.wall_edge_d_x, info.wall_edge_d_y,
          kBlueR, kBlueG, kBlueB, kGarbageDotSizeM));
      auto td = makeBase(
        "wall_edge_pts", wbase + wid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      td.pose.position.x = info.wall_edge_d_x;
      td.pose.position.y = info.wall_edge_d_y;
      td.pose.position.z = 0.42;
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
        arr.markers.push_back(
          makeSolidDot(
            "wall_edge_pts", wbase + wid++, p.first, p.second,
            kBlueR, kBlueG, kBlueB, kMidDotSizeM));
      }
    }

    VizPileTrack track;
    track.pile_num = pile_num;
    track.gx = gx;
    track.gy = gy;
    track.has_e = info.extend_inserted;
    track.ex = info.extend_x;
    track.ey = info.extend_y;
    viz_tracks_.push_back(track);
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
  // 改过 goals 就统一重编号 + 统一时间戳再写回，插入和剥离两条路都要走这里，
  // 否则剥掉哨兵点以后普通点的 z 序号会带着空洞发下去
  auto commitGoals = [&](const char * reason) {
      const rclcpp::Time now_stamp = node_->now();
      for (std::size_t i = 0; i < goals_now.size(); ++i) {
        if (!isUnindexedSentinelPoseZ(goals_now[i])) {
          goals_now[i].pose.position.z = static_cast<double>(i);
        }
        goals_now[i].header.stamp = now_stamp;
      }
      mission_stamp_record_ = now_stamp;
      has_mission_stamp_ = true;
      emitOutputGoals(goals_now, reason);
    };

  if (goals_now.size() < 2) {
    geometry_msgs::msg::PoseStamped robot_pose_short;
    if (getRobotPose(robot_pose_short)) {
      if (eraseSweptSentinelsFromGoals(goals_now, robot_pose_short)) {
        goals_dirty = true;
        stripReachedZNeg1Goals(goals_now);
      }
      const double robot_x = robot_pose_short.pose.position.x;
      const double robot_y = robot_pose_short.pose.position.y;
      publishRangeCircles(robot_x, robot_y);
      refreshClipAnchorVisualization(goals_now, robot_x, robot_y);
    }
    pruneFinishedPileVisualization(goals_now);
    publishPendingGarbageDots();
    if (goals_dirty) {
      commitGoals("sweep_done");
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

  stripReachedZNeg1Goals(goals_now);
  if (eraseSweptSentinelsFromGoals(goals_now, robot_pose)) {
    goals_dirty = true;
    stripReachedZNeg1Goals(goals_now);
  }
  pruneFinishedPileVisualization(goals_now);
  refreshClipAnchorVisualization(goals_now, rx, ry);
  publishPendingGarbageDots();

  // 排查：active 堆是否还在 {goals}、z=-1 还剩几个
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
        "z=-1_n=%zu %s | active_n=%zu %s | protected_n=%zu %s",
        rx, ry, robot_yaw, goals_now.size(),
        z_neg1_n, z_neg1_oss.str().c_str(),
        active_piles_.size(), active_oss.str().c_str(),
        reached_garbage_xy_.size(), prot_oss.str().c_str());
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

  bool single_pile_insert = true;
  getInput("single_pile_insert", single_pile_insert);
  bool ge_still_in_goals = false;
  if (single_pile_insert) {
    for (const auto & g : goals_now) {
      if (isUnindexedSentinelPoseZ(g)) {
        ge_still_in_goals = true;
        break;
      }
    }
  }
  if (ge_still_in_goals) {
    single_pile_block_intake_ = true;
  } else if (single_pile_insert) {
    single_pile_block_intake_ = false;
  }
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
    } else {
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
    if (peeled_n > 0) {
      // 剥掉的点必须写回黑板，否则这一轮白剥，下一 tick 还会看到它们
      goals_dirty = true;
    }

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
    // 这里是直接拼表，没走 tryInsertPreferCloserToRobot，得自己守住上限
    trimGarbageListToCap(order_x, order_y);
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
      commitGoals("sweep_or_peel");
    }
    return BT::NodeStatus::SUCCESS;
  }

  bool enable_viz = true;
  bool viz_garbage = true;
  getInput("enable_visualization", enable_viz);
  getInput("viz_accepted_garbage", viz_garbage);

  // 单堆占用中：当前 G/E 还在 goals 里，不再插下一堆
  if (single_pile_block_intake_) {
    if (goals_dirty) {
      commitGoals("sweep_or_peel");
    }
    return BT::NodeStatus::SUCCESS;
  }
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
    const std::size_t goals_before_pile = goals_now.size();
    std::string fp_reason;
    const bool footprint_ok = isFootprintSweepClear(
      robot_pose.pose.position.x, robot_pose.pose.position.y, gx, gy, &fp_reason);
    if (footprint_ok) {
      // 2.11.2 通过：正常 G-E 扫
      goals_now = insertGarbageIntoGoals(info);
    } else if (single_pile_insert) {
      // 单堆且 2.11.2 不过：贴墙 entry-G-E
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: footprint sweep fail robot->G (%.2f, %.2f): %s, wall-edge insert",
        gx, gy, fp_reason.c_str());
      publishFootprintCheckBox(gx, gy, info.path_yaw);
      goals_now = insertWallEdgeGarbageIntoGoals(info);
      if (!info.wall_edge_inserted) {
        addProtectedGarbageXy(gx, gy);
        garbage_list_.erase(garbage_list_.begin());
        continue;
      }
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip garbage (%.2f, %.2f), 多堆 footprint 不过，不生成 (%s)",
        gx, gy, fp_reason.c_str());
      publishFootprintCheckBox(gx, gy, info.path_yaw);
      addProtectedGarbageXy(gx, gy);
      garbage_list_.erase(garbage_list_.begin());
      continue;
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
    if (single_pile_insert) {
      single_pile_block_intake_ = true;
      break;
    }
  }

  last_sweep_xy_.clear();
  last_sweep_xy_.reserve(active_piles_.size());
  for (const auto & g : active_piles_) {
    last_sweep_xy_.emplace_back(
      g.pose.pose.position.x, g.pose.pose.position.y);
  }

  if (inserted_count > 0) {
    std::size_t z_neg1_n = 0;
    for (const auto & g : goals_now) {
      if (isUnindexedSentinelPoseZ(g)) {
        ++z_neg1_n;
      }
    }
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
    commitGoals((inserted_count > 0) ? "batch_insert" : "peel_unstarted");
  }
  pruneFinishedPileVisualization(goals_now);
  refreshClipAnchorVisualization(goals_now, rx, ry);
  publishPendingGarbageDots();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
