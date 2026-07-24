#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  global_frame_("map"),
  robot_base_frame_("base_link"),
  arrived_radius_(0.5),       // footprint 进入垃圾附近半径，停止再插入
  clip_extend_m_(1.0),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(60.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0)  // 离队头超过该距离就不删点
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("arrived_radius", arrived_radius_);
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
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

  // TRANSIENT_LOCAL：晚订阅也能收到最后一条区域消息
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

// true=非角点，false=角点：前一点→当前→下一点夹角 > corner_angle_deg 则为角点
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

// 机器人前方累计路径 range_m 内的最后一个 goal；从队头 goals[0] 沿有序路径往前量
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

  // goalc：第一角点；无角点则用前方 goaltotal_range_m 内最后一点当假角点
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

  // goala 与 goalc 重合时无法定投影    主要就是因为直线机器人离轨导致队首被判成角点
  // 改用下一角点；再没有则用 range 内末点；至少用 goals[1]
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

// 按角点链从队头往后有序删点；角点保留。不按机器人欧氏最近段跳删
InsertGarbagePose::Goals InsertGarbagePose::clipGoalsNearGarbage(InsertInfo & info)
{
  const Goals & goals = info.goals;
  info.goaltotal.clear();
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

  constexpr double kEps = 1e-6;
  while (true) {
    const double cx = goals[c_idx].pose.position.x;
    const double cy = goals[c_idx].pose.position.y;
    const double t_d = lineParameterT(dx, dy, ax, ay, cx, cy);

    // 反向延长线：不删
    if (t_d < -kEps) {
      break;
    }

    // 前方延长线：从队头删到角点前，保留角点，继续找下一角
    if (t_d > 1.0 + kEps) {
      info.hit_forward_case = true;
      if (first_round) {
        markBetween(0, c_idx, true);  // 从队头 0 往后删到角点前
      } else {
        markBetween(a_idx, c_idx, false);
      }
      info.corners_kept_xy.emplace_back(cx, cy);

      std::size_t next_c = 0;
      if (!findNextCornerAfter(goals, c_idx, rx, ry, next_c)) {
        break;
      }
      a_idx = c_idx;
      c_idx = next_c;
      protected_corners.insert(a_idx);
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


// 插入真实垃圾、统一时间戳
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

  // 默认队首；太远强制队首；段中插在 goala 后；延长线插在保留角点后
  std::size_t insert_idx = 0;
  constexpr double kMatchTol = 0.08;
  if (far_from_head) {
    insert_idx = 0;
  } else if (info.hit_mid_case) {
    const double ax = info.goala.pose.position.x;
    const double ay = info.goala.pose.position.y;
    for (std::size_t j = 0; j < out.size(); ++j) {
      const double dx = out[j].pose.position.x - ax;
      const double dy = out[j].pose.position.y - ay;
      if (dx * dx + dy * dy < kMatchTol * kMatchTol) {
        insert_idx = j + 1;
        break;
      }
    }
  } else if (info.hit_forward_case && !info.corners_kept_xy.empty()) {
    for (auto it = info.corners_kept_xy.rbegin(); it != info.corners_kept_xy.rend(); ++it) {
      for (std::size_t j = 0; j < out.size(); ++j) {
        const double dx = out[j].pose.position.x - it->first;
        const double dy = out[j].pose.position.y - it->second;
        if (dx * dx + dy * dy < kMatchTol * kMatchTol) {
          insert_idx = j + 1;
          break;
        }
      }
      if (insert_idx != 0) {
        break;
      }
    }
  }

  geometry_msgs::msg::PoseStamped garbage_pose = info.garbage.pose;
  if (garbage_pose.header.frame_id.empty() && !out.empty()) {
    garbage_pose.header.frame_id = out.front().header.frame_id;
  } else if (garbage_pose.header.frame_id.empty()) {
    garbage_pose.header.frame_id = global_frame_;
  }
  garbage_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);

  out.insert(out.begin() + static_cast<std::ptrdiff_t>(insert_idx), garbage_pose);

  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : out) {
    pose.header.stamp = stamp_now;
  }
  mission_stamp_record_ = stamp_now;
  has_mission_stamp_ = true;

  return out;
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
