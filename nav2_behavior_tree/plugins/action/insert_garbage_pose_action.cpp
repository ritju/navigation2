#include <algorithm>
#include <cmath>
#include <memory>
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

// 构造：创建垃圾话题和禁扫区域话题订阅
InsertGarbagePose::InsertGarbagePose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  garbage_topic_("/garbage_cord"),
  special_terrain_topic_("/cleaning_tool_retraction_areas"),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
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
// history 只拷贝不掏空：处理成功（或明确丢弃）后再从 history 删除，失败则仍留在 history
InsertGarbagePose::GarbageList InsertGarbagePose::postProcessHistory()
{
  std::deque<capella_ros_msg::msg::GarbageDetect> snapshot;
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    snapshot = history_list_;
  }

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

  auto sameDetect =
    [](const capella_ros_msg::msg::GarbageDetect & a,
      const capella_ros_msg::msg::GarbageDetect & b) {
      return a.class_id == b.class_id &&
             a.pose.header.stamp == b.pose.header.stamp &&
             std::fabs(a.pose.pose.position.x - b.pose.pose.position.x) < 1e-6 &&
             std::fabs(a.pose.pose.position.y - b.pose.pose.position.y) < 1e-6;
    };

  // 从 history 里删掉已处理完的那一条（按原始消息匹配）
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

    // 第一步：转到 map（失败则留在 history，下一轮再试）
    if (!transformGarbageToMap(garbage)) {
      continue;
    }

    // 第二步：禁扫区丢弃 → 已决策，从 history 去掉
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

    // 第三步：与已有列表去重 → 已决策，从 history 去掉
    if (isDuplicateOfKept(garbage, garbage_list_)) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "InsertGarbagePose: drop duplicate garbage at map (%.2f, %.2f)",
        garbage.pose.pose.position.x, garbage.pose.pose.position.y);
      eraseFromHistory(original);
      continue;
    }

    // 第四步：成功写入 garbage_list_ 后再从 history 删除；写不进去（更远）则仍留着
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
      "InsertGarbagePose: failed to get input_goals (e.g. {goals})");
    received_goals_.clear();
    return received_goals_;
  }

  received_goals_ = goals;
  return received_goals_;
}

// 点到无限直线 AB 的垂足（平面 xy）
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

// 点在无限直线 AB 上的参数 t（P ≈ A + t*(B-A)）
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

// 获取插入所需的全部信息并返回
InsertGarbagePose::InsertInfo InsertGarbagePose::gatherInsertInfo()
{
  InsertInfo info;

  // 1) 后处理垃圾列表 + 2) 完整 goals
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

  info.goal_a = info.goals[0];
  info.goal_b = info.goals[1];
  info.garbage = garbage_list_.front();

  // 3) 机器人当前 map 位姿
  if (!nav2_util::getCurrentPose(
      info.robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    info.invalid_reason = "getCurrentPose failed";
    return info;
  }

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;
  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;

  // 4) 半径 R
  info.radius_m = std::sqrt(squaredDistanceXY(rx, ry, gx, gy));
  if (info.radius_m < 1e-6) {
    info.invalid_reason = "radius ~ 0";
    return info;
  }

  // 5) 圆心：垃圾投影到 goals[0]-goals[1] 无限直线
  projectPointToInfiniteLine(
    gx, gy,
    info.goal_a.pose.position.x, info.goal_a.pose.position.y,
    info.goal_b.pose.position.x, info.goal_b.pose.position.y,
    info.circle_center_x, info.circle_center_y);

  // 6) 插入朝向：goal_a → goal_b
  info.path_yaw = std::atan2(
    info.goal_b.pose.position.y - info.goal_a.pose.position.y,
    info.goal_b.pose.position.x - info.goal_a.pose.position.x);

  info.valid = true;
  return info;
}

// 根据 InsertInfo：剔圆内点、插入真实垃圾、统一时间戳
InsertGarbagePose::Goals InsertGarbagePose::insertGarbageIntoGoals(const InsertInfo & info)
{
  Goals out = info.goals;

  const double cx = info.circle_center_x;
  const double cy = info.circle_center_y;
  const double r2 = info.radius_m * info.radius_m;

  // 1) 剔除落在圆心+半径 R 圆内的路径点（d < R）
  out.erase(
    std::remove_if(
      out.begin(), out.end(),
      [cx, cy, r2](const geometry_msgs::msg::PoseStamped & pose) {
        return squaredDistanceXY(
          pose.pose.position.x, pose.pose.position.y, cx, cy) < r2;
      }),
    out.end());

  // 2) 按 goal_a→goal_b 直线参数 t，插在圆心前一侧与后一侧之间
  const double ax = info.goal_a.pose.position.x;
  const double ay = info.goal_a.pose.position.y;
  const double bx = info.goal_b.pose.position.x;
  const double by = info.goal_b.pose.position.y;
  const double t_c = lineParameterT(cx, cy, ax, ay, bx, by);

  std::size_t insert_idx = out.size();
  for (std::size_t i = 0; i < out.size(); ++i) {
    const double t = lineParameterT(
      out[i].pose.position.x, out[i].pose.position.y, ax, ay, bx, by);
    if (t > t_c) {
      insert_idx = i;
      break;
    }
  }

  geometry_msgs::msg::PoseStamped garbage_pose = info.garbage.pose;
  if (garbage_pose.header.frame_id.empty() && !out.empty()) {
    garbage_pose.header.frame_id = out.front().header.frame_id;
  } else if (garbage_pose.header.frame_id.empty()) {
    garbage_pose.header.frame_id = global_frame_;
  }
  garbage_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
  // 不重排其它点的 z，新点 z 保持垃圾原样

  out.insert(out.begin() + static_cast<std::ptrdiff_t>(insert_idx), garbage_pose);

  // 3) 统一时间戳
  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : out) {
    pose.header.stamp = stamp_now;
  }

  return out;
}

// 行为树周期回调
BT::NodeStatus InsertGarbagePose::tick()
{
  callback_group_executor_.spin_some();

  const InsertInfo info = gatherInsertInfo();
  if (!info.valid) {
    setOutput("output_goals", info.goals.empty() ? receiveGoals() : info.goals);
    return BT::NodeStatus::SUCCESS;
  }

  const Goals updated_goals = insertGarbageIntoGoals(info);
  setOutput("output_goals", updated_goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
