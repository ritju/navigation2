#include <algorithm>
#include <cmath>
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
  garbage_topic_("/garbage_cord"),
  special_terrain_topic_("/cleaning_tool_retraction_areas"),
  footprint_topic_("local_costmap/published_footprint"),
  visualization_topic_("insert_garbage_pose/markers"),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  arrived_radius_(0.5),       // footprint 进入垃圾附近半径，停止再插入
  clip_extend_m_(1.0),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(30.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0),  // 离队头超过该距离就不删点
  max_garbage_robot_dist_m_(9.0)   // 垃圾离机器人超过该距离则忽略
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("visualization_topic", visualization_topic_);
  getInput("arrived_radius", arrived_radius_);
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
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

  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&InsertGarbagePose::footprintCallback, this, std::placeholders::_1),
    sub_option);

  // RViz Marker，默认 VOLATILE，和 RViz 订阅对齐
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_, 10);
}

// 垃圾检测话题回调，把垃圾放进 history_list_
void InsertGarbagePose::garbageDetectCallback(
  const capella_ros_msg::msg::GarbageDetect::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(history_mutex_);
  history_list_.push_back(*msg);
  while (history_list_.size() > kMaxHistorySize) {
    history_list_.pop_front();
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

// 判断点是否在多边形内
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
  if (garbage_list_.size() < kMaxHistorySize) {
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

  for (const auto & original : snapshot) {
    capella_ros_msg::msg::GarbageDetect garbage = original;

    // 转到 map，失败则留在 history
    if (!transformGarbageToMap(garbage)) {
      continue;
    }

    // 落在禁扫区则丢弃
    if (isPointInSpecialTerrain(
        garbage.pose.pose.position.x, garbage.pose.pose.position.y))
    {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "InsertGarbagePose: drop garbage in special terrain at map (%.2f, %.2f)",
        garbage.pose.pose.position.x, garbage.pose.pose.position.y);
      eraseFromHistory(original);
      continue;
    }

    // 离机器人太远：视为误识别，丢弃
    if (max_garbage_robot_dist_m_ > 0.0) {
      const double dist_robot = std::sqrt(squaredDistanceXY(
          garbage.pose.pose.position.x, garbage.pose.pose.position.y,
          robot_x, robot_y));
      if (dist_robot > max_garbage_robot_dist_m_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: drop garbage %.2fm from robot > %.2fm at (%.2f, %.2f)",
          dist_robot, max_garbage_robot_dist_m_,
          garbage.pose.pose.position.x, garbage.pose.pose.position.y);
        eraseFromHistory(original);
        continue;
      }
    }

    // 已到达过的垃圾不再写入
    if (isNearReachedGarbage(
        garbage.pose.pose.position.x, garbage.pose.pose.position.y))
    {
      eraseFromHistory(original);
      continue;
    }

    // 与已有列表去重
    if (isDuplicateOfKept(garbage, garbage_list_)) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "InsertGarbagePose: drop duplicate garbage at map (%.2f, %.2f)",
        garbage.pose.pose.position.x, garbage.pose.pose.position.y);
      eraseFromHistory(original);
      continue;
    }

    // 写入 garbage_list_ 成功后再从 history 删除
    if (tryInsertPreferCloserToRobot(std::move(garbage), robot_x, robot_y)) {
      eraseFromHistory(original);
    } else {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "InsertGarbagePose: drop farther garbage, list full, keep in history");
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
  reached_garbage_xy_.clear();
  mission_stamp_record_ = current_stamp;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: new mission stamp detected, cleared history and garbage list");
}

// 获取机器人当前 footprint，并转到 map
bool InsertGarbagePose::getRobotFootprintInMap(
  std::vector<geometry_msgs::msg::Point> & footprint_map)
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

// 判断 footprint 是否已进入垃圾附近
bool InsertGarbagePose::shouldStopInsertingGarbage(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const std::vector<geometry_msgs::msg::Point> & footprint_map,
  double arrived_radius) const
{
  if (footprint_map.empty() || arrived_radius <= 0.0) {
    return false;
  }

  const double gx = garbage.pose.pose.position.x;
  const double gy = garbage.pose.pose.position.y;
  const double r2 = arrived_radius * arrived_radius;

  for (const auto & pt : footprint_map) {
    if (squaredDistanceXY(pt.x, pt.y, gx, gy) < r2) {
      return true;
    }
  }

  geometry_msgs::msg::Polygon footprint_poly;
  footprint_poly.points.reserve(footprint_map.size());
  for (const auto & pt : footprint_map) {
    geometry_msgs::msg::Point32 p32;
    p32.x = static_cast<float>(pt.x);
    p32.y = static_cast<float>(pt.y);
    p32.z = static_cast<float>(pt.z);
    footprint_poly.points.push_back(p32);
  }
  return isPointInPolygon(gx, gy, footprint_poly);
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

  std::size_t range_end = 0;
  std::size_t start_idx = 0;
  if (!findLastGoalWithinPathRange(
      goals, robot_x, robot_y, goaltotal_range_m_, range_end, nullptr, &start_idx))
  {
    return false;
  }

  for (std::size_t i = start_idx; i <= range_end && i < goals.size(); ++i) {
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
  double /*robot_x*/, double /*robot_y*/,
  double range_m,
  std::size_t & out_idx,
  std::size_t * nearest_seg_out,
  std::size_t * start_idx_out) const
{
  if (goals.size() < 2 || range_m <= 0.0) {
    return false;
  }

  const std::size_t start_idx = 0;
  double accumulated = 0.0;
  out_idx = 0;
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
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
    *nearest_seg_out = 0;
  }
  if (start_idx_out) {
    *start_idx_out = start_idx;
  }
  return true;
}

// 获取插入所需的全部信息并返回
InsertGarbagePose::InsertInfo InsertGarbagePose::gatherInsertInfo()
{
  InsertInfo info;

  // 后处理垃圾列表，读取完整 goals
  postProcessHistory();
  info.goals = receiveGoals();

  if (info.goals.size() < 2) {
    info.invalid_reason = "goals size < 2";
    return info;
  }
  if (garbage_list_.empty()) {
    info.invalid_reason = "no garbage";
    return info;
  }

  info.garbage = garbage_list_.front();

  // 机器人当前 map 位姿
  if (!nav2_util::getCurrentPose(
      info.robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    info.invalid_reason = "getCurrentPose failed";
    return info;
  }
  // 投影线起点 info.goala = info.robot_pose
  info.goala = info.goals.front();
 // 给垃圾点和机器人点各起一对短名字，写起来方便
  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;
  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;

  // 半径 R
  info.radius_m = std::sqrt(squaredDistanceXY(rx, ry, gx, gy));
  if (info.radius_m < 1e-6) {
    info.invalid_reason = "radius ~ 0";
    return info;
  }

  // 这段是在定投影线的终点 goalc   在队头起约 10m 路径里找第一个角点
  if (!findFirstCornerFromRobot(info.goals, rx, ry, info.goalc_idx)) {
    if (!findLastGoalWithinPathRange(
        info.goals, rx, ry, goaltotal_range_m_, info.goalc_idx))
    {
      info.invalid_reason = "no goalc fallback within range";
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
      return info;
    }
    info.goalc = info.goals[info.goalc_idx];
    if (squaredDistanceXY(
        info.goala.pose.position.x, info.goala.pose.position.y,
        info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
    {
      info.invalid_reason = "goala and goalc too close";
      return info;
    }
  }

  // goald：垃圾投影到 goala-goalc 无限直线的垂足
  projectPointToInfiniteLine(
    gx, gy,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    info.goald_x, info.goald_y);

  // 插入朝向取 goala → goalc
  info.path_yaw = std::atan2(
    info.goalc.pose.position.y - info.goala.pose.position.y,
    info.goalc.pose.position.x - info.goala.pose.position.x);

  info.valid = true;
  return info;
}

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
    old_to_new[old_i] = new_i;
  }

  auto is_kept = [&](std::size_t idx) {
    return (keep_idx != nullptr && idx == *keep_idx) ||
           (also_keep_idx != nullptr && idx == *also_keep_idx);
  };

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
    const std::size_t new_i = map_it->second;
    const bool lost_inbound = (old_i == 0) || (delete_idx.count(old_i - 1) != 0);
    const bool became_head = (new_i == 0);
    // 队首 + 原前驱已断 + 后方还有要保留的角点 → 过期肘点
    // （当前 A/C 已在 is_kept，不会误伤下一轮投影边）
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

  // 各 goal 折线弧长（从队头 goals[0] 起有序累加）
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

  // 距 goals[0] 太远：本轮不删任何点， 先去垃圾再回第一个导航点
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  const double dist_to_head = std::sqrt(squaredDistanceXY(
    rx, ry, goals[0].pose.position.x, goals[0].pose.position.y));
  if (dist_to_head > head_delete_robot_dist_m_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: robot %.2fm from goals[0] > %.2fm, skip all deletes",
      dist_to_head, head_delete_robot_dist_m_);
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
  Goals out = clipGoalsNearGarbage(info);

  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  const double dist_to_head = info.goals.empty()
    ? std::numeric_limits<double>::infinity()
    : std::sqrt(squaredDistanceXY(
        info.robot_pose.pose.position.x, info.robot_pose.pose.position.y,
        info.goals[0].pose.position.x, info.goals[0].pose.position.y));
  const bool far_from_head = (dist_to_head > head_delete_robot_dist_m_);


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

  Goals rebuilt;
  rebuilt.reserve(1 + (out.size() - resume_from));

  geometry_msgs::msg::PoseStamped garbage_pose = info.garbage.pose;
  if (garbage_pose.header.frame_id.empty() && !out.empty()) {
    garbage_pose.header.frame_id = out.front().header.frame_id;
  } else if (garbage_pose.header.frame_id.empty()) {
    garbage_pose.header.frame_id = global_frame_;
  }
  garbage_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
  rebuilt.push_back(garbage_pose);

  for (std::size_t i = resume_from; i < out.size(); ++i) {
    rebuilt.push_back(out[i]);
  }
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

// 往 RViz 发本次插入的证据 Marker
void InsertGarbagePose::publishVisualization(
  const InsertInfo & info,
  bool enable,
  bool viz_accepted_garbage,
  bool viz_deleted_goals,
  bool viz_ac_points,
  bool viz_ac_geometry)
{
  if (!marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();

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

  // 先清旧的，只留这一次插入
  {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = global_frame_;
    clear.header.stamp = stamp;
    clear.ns = "";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
  }

  if (!enable) {
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

  // 短线段拼虚线
  auto appendDashed = [&](
      visualization_msgs::msg::Marker & m,
      double x0, double y0, double x1, double y1,
      double dash = 0.18, double gap = 0.12)
  {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) {
      return;
    }
    const double ux = dx / len;
    const double uy = dy / len;
    double s = 0.0;
    bool draw = true;
    while (s < len - 1e-9) {
      const double seg = draw ? dash : gap;
      const double s1 = std::min(len, s + seg);
      if (draw) {
        pushPoint(m, x0 + ux * s, y0 + uy * s);
        pushPoint(m, x0 + ux * s1, y0 + uy * s1);
      }
      s = s1;
      draw = !draw;
    }
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

  int mid = 0;

  if (viz_accepted_garbage) {
    auto m = makeBase("accepted_garbage", mid++, visualization_msgs::msg::Marker::SPHERE);
    m.pose.position.x = info.garbage.pose.pose.position.x;
    m.pose.position.y = info.garbage.pose.pose.position.y;
    m.pose.position.z = 0.12;
    m.scale.x = 0.28;
    m.scale.y = 0.28;
    m.scale.z = 0.28;
    setColor(m, 0.85f, 0.12f, 0.12f, 0.95f);
    arr.markers.push_back(m);

    auto t = makeBase("accepted_garbage", mid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    t.pose.position.x = info.garbage.pose.pose.position.x;
    t.pose.position.y = info.garbage.pose.pose.position.y;
    t.pose.position.z = 0.40;
    t.scale.z = 0.22;
    t.text = "G";
    setColor(t, 0.85f, 0.12f, 0.12f);
    arr.markers.push_back(t);
  }

  if (viz_deleted_goals) {
    // 空心黑圈，比 goals 球略大一点即可
    const double ring_r = 0.16;
    for (const auto & g : info.goaltotal) {
      auto m = makeBase("deleted_goals", mid++, visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.025;
      setColor(m, 0.05f, 0.05f, 0.05f, 0.95f);
      appendRing(m, g.pose.position.x, g.pose.position.y, ring_r);
      arr.markers.push_back(m);
    }
  }

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;

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
    // 文字往侧面挪，和 F 一样，别压在点/圈上
    constexpr double kLabelOff = 0.50;

    if (viz_ac_points) {
      auto ma = makeBase("ac_points", mid++, visualization_msgs::msg::Marker::CUBE);
      ma.pose.position.x = rnd.ax;
      ma.pose.position.y = rnd.ay;
      ma.pose.position.z = 0.08;
      ma.scale.x = 0.18;
      ma.scale.y = 0.18;
      ma.scale.z = 0.08;
      setColor(ma, lr, lg, lb, 0.95f);
      arr.markers.push_back(ma);

      auto ta = makeBase("ac_points", mid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      ta.pose.position.x = rnd.ax - uy * kLabelOff;
      ta.pose.position.y = rnd.ay + ux * kLabelOff;
      ta.pose.position.z = 0.45;
      ta.scale.z = 0.28;
      {
        std::ostringstream oss;
        oss << "A" << rnd.round_i;
        ta.text = oss.str();
      }
      setColor(ta, lr, lg, lb);
      arr.markers.push_back(ta);

      auto mc = makeBase("ac_points", mid++, visualization_msgs::msg::Marker::SPHERE);
      mc.pose.position.x = rnd.cx;
      mc.pose.position.y = rnd.cy;
      mc.pose.position.z = 0.10;
      mc.scale.x = 0.22;
      mc.scale.y = 0.22;
      mc.scale.z = 0.22;
      setColor(mc, lr, lg, lb, 0.95f);
      arr.markers.push_back(mc);

      auto tc = makeBase("ac_points", mid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      tc.pose.position.x = rnd.cx + uy * kLabelOff;
      tc.pose.position.y = rnd.cy - ux * kLabelOff;
      tc.pose.position.z = 0.45;
      tc.scale.z = 0.28;
      {
        std::ostringstream oss;
        oss << "C" << rnd.round_i;
        tc.text = oss.str();
      }
      setColor(tc, lr, lg, lb);
      arr.markers.push_back(tc);
    }

    if (viz_ac_geometry) {
      // A→C 实线：首轮红粗，后续青
      auto solid = makeBase("ac_geometry", mid++, visualization_msgs::msg::Marker::LINE_STRIP);
      solid.scale.x = first ? 0.070 : 0.055;
      setColor(solid, lr, lg, lb, 0.95f);
      pushPoint(solid, rnd.ax, rnd.ay);
      pushPoint(solid, rnd.cx, rnd.cy);
      arr.markers.push_back(solid);

      const double ext = std::max(3.0, 0.6 * L);

      auto dashed = makeBase("ac_geometry", mid++, visualization_msgs::msg::Marker::LINE_LIST);
      dashed.scale.x = 0.035;
      setColor(dashed, lr, lg, lb, 0.75f);
      appendDashed(
        dashed,
        rnd.cx, rnd.cy,
        rnd.cx + ux * ext, rnd.cy + uy * ext);
      appendDashed(
        dashed,
        rnd.ax, rnd.ay,
        rnd.ax - ux * ext, rnd.ay - uy * ext);
      arr.markers.push_back(dashed);

      // 垃圾到垂足
      auto perp = makeBase("ac_geometry", mid++, visualization_msgs::msg::Marker::LINE_LIST);
      perp.scale.x = 0.040;
      setColor(perp, 0.83f, 0.00f, 0.98f, 0.90f);
      appendDashed(perp, gx, gy, rnd.fx, rnd.fy, 0.14, 0.10);
      arr.markers.push_back(perp);

      auto mf = makeBase("ac_geometry", mid++, visualization_msgs::msg::Marker::SPHERE);
      mf.pose.position.x = rnd.fx;
      mf.pose.position.y = rnd.fy;
      mf.pose.position.z = 0.08;
      mf.scale.x = 0.14;
      mf.scale.y = 0.14;
      mf.scale.z = 0.14;
      setColor(mf, 1.00f, 0.44f, 0.00f, 0.95f);
      arr.markers.push_back(mf);

      auto tf = makeBase("ac_geometry", mid++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      // F 往垂足侧向挪一点，别压在橙点上
      tf.pose.position.x = rnd.fx - uy * kLabelOff;
      tf.pose.position.y = rnd.fy + ux * kLabelOff;
      tf.pose.position.z = 0.36;
      tf.scale.z = 0.20;
      {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << "F" << rnd.round_i << " t=" << rnd.t_d;
        tf.text = oss.str();
      }
      setColor(tf, 1.00f, 0.44f, 0.00f);
      arr.markers.push_back(tf);
    }
  }

  marker_pub_->publish(arr);
}

// 行为树周期回调
BT::NodeStatus InsertGarbagePose::tick()
{
  callback_group_executor_.spin_some();
  checkAndResetOnNewMission();

  InsertInfo info = gatherInsertInfo();
  if (!info.valid) {
    setOutput("output_goals", info.goals.empty() ? receiveGoals() : info.goals);
    return BT::NodeStatus::SUCCESS;
  }

  const Goals updated_goals = insertGarbageIntoGoals(info);
  setOutput("output_goals", updated_goals);

  bool enable_viz = true;
  bool viz_garbage = true;
  bool viz_deleted = true;
  bool viz_ac_pts = true;
  bool viz_ac_geom = true;
  getInput("enable_visualization", enable_viz);
  getInput("viz_accepted_garbage", viz_garbage);
  getInput("viz_deleted_goals", viz_deleted);
  getInput("viz_ac_points", viz_ac_pts);
  getInput("viz_ac_geometry", viz_ac_geom);
  publishVisualization(info, enable_viz, viz_garbage, viz_deleted, viz_ac_pts, viz_ac_geom);

  // 插一次即完成：记入已处理，清空待插与 history，避免后续 tick 再插
  reached_garbage_xy_.emplace_back(
    info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    history_list_.clear();
  }
  garbage_list_.clear();
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: inserted garbage once at (%.2f, %.2f), will not re-insert",
    info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
