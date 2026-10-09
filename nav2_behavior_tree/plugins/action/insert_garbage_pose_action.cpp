#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
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
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_behavior_tree/plugins/action/insert_garbage_pose_action.hpp"

namespace nav2_behavior_tree
{

InsertGarbagePose::InsertGarbagePose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  garbage_topic_("/garbage_cord1"),
  special_terrain_topic_("/cleaning_tool_retraction_areas"),
  footprint_topic_("local_costmap/published_footprint"),
  global_costmap_topic_("global_costmap/costmap_raw"),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  clip_extend_m_(2.5),        // 从垃圾垂足沿路径再删的距离
  corner_angle_deg_(30.0),    // 前后两段夹角超过此值视为角点
  goaltotal_range_m_(10.0),   // 无角点时，前方该距离内末点当作 goalc
  head_delete_robot_dist_m_(4.0),  // 车离路径最近点超过该距离就不删点
  max_garbage_robot_dist_m_(5.0),  // 垃圾离机器人超过该距离则忽略
  wall_edge_d_extend_m_(2.0),
  wall_edge_min_robot_dist_m_(3.0),
  wall_edge_sample_m_(0.5),
  wall_edge_normal_offset_m_(0.0),
  garbage_merge_radius_m_(1.0),    // 到种子小于该距离合为一堆
  garbage_extend_m_(2.0),          // 沿扫向相对垃圾再插一点，默认 2.0m
  ray_offset_deg_(30.0),
  extend_near_radius_m_(1.5),
  extend_extra_m_(1.0),
  work_circle_radius_m_(10.0)
{
  getInput("garbage_topic", garbage_topic_);
  getInput("special_terrain_topic", special_terrain_topic_);
  getInput("footprint_topic", footprint_topic_);
  getInput("global_costmap_topic", global_costmap_topic_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  refreshTunableInputPorts();

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  // 可靠性保持系统默认，只加深队列，避免两次 tick 之间的多条检测互相覆盖
  rclcpp::QoS garbage_qos = rclcpp::SystemDefaultsQoS().keep_last(10);
  garbage_sub_ = node_->create_subscription<capella_ros_msg::msg::GarbageDetect>(
    garbage_topic_,
    garbage_qos,
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

  // 挂到本节点 callback group：tick 里取完队列后立刻能查，不依赖默认组
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

  workspace_circle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "insert_workspace_circle", 10);
  anchor_point_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "insert_anchor_point", 10);
  footprint_check_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "insert_footprint_check", 10);
  garbage_pose_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "insert_garbage_pose", 10);
}

void InsertGarbagePose::refreshTunableInputPorts()
{
  getInput("clip_extend_m", clip_extend_m_);
  getInput("corner_angle_deg", corner_angle_deg_);
  getInput("goaltotal_range_m", goaltotal_range_m_);
  getInput("head_delete_robot_dist_m", head_delete_robot_dist_m_);
  getInput("max_garbage_robot_dist_m", max_garbage_robot_dist_m_);
  getInput("sweep_dist_weight", sweep_dist_weight_);
  getInput("sweep_turn_weight", sweep_turn_weight_);
  getInput("wall_edge_d_extend_m", wall_edge_d_extend_m_);
  getInput("wall_edge_min_robot_dist_m", wall_edge_min_robot_dist_m_);
  getInput("wall_edge_sample_m", wall_edge_sample_m_);
  getInput("wall_edge_normal_offset_m", wall_edge_normal_offset_m_);
  getInput("garbage_merge_radius_m", garbage_merge_radius_m_);
  getInput("garbage_extend_m", garbage_extend_m_);
  getInput("ray_offset_deg", ray_offset_deg_);
  getInput("extend_near_radius_m", extend_near_radius_m_);
  getInput("extend_extra_m", extend_extra_m_);
  getInput("extend_max_yaw_deg", extend_max_yaw_deg_);
  getInput("extend_step_yaw_deg", extend_step_yaw_deg_);
  getInput("work_circle_radius_m", work_circle_radius_m_);
  getInput("confirm_match_dist_m", confirm_match_dist_m_);
  getInput("confirm_match_num", confirm_match_num_);
  getInput("confirm_sec_garbage_time", confirm_sec_garbage_time_);
  getInput("single_pile_insert", single_pile_insert_);
  getInput("enable_wall_edge_insert", enable_wall_edge_insert_);
  getInput("enable_visualization", enable_visualization_);
}

void InsertGarbagePose::garbageDetectCallback(
  const capella_ros_msg::msg::GarbageDetect::SharedPtr msg)
{
  refreshTunableInputPorts();
  if (single_pile_insert_) {
    Goals goals;
    if (getInput("input_goals", goals) && hasUnindexedSentinel(goals)) {
      return;
    }
  }

  capella_ros_msg::msg::GarbageDetect item;
  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!msg || !transformGarbageToMap(item = *msg) || !getRobotPoseXY(robot_x, robot_y)) {
    if (msg) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: drop garbage, transform to map or robot pose failed, wait for next frame");
    }
    return;
  }

  const double gx = item.pose.pose.position.x;
  const double gy = item.pose.pose.position.y;
  const double confirm_r2 = confirm_match_dist_m_ * confirm_match_dist_m_;

  // 时间窗内累计检测：凑够 confirm_match_num 帧且距离在 confirm_match_dist_m 内，取平均往下传
  const rclcpp::Time now = node_->now();
  for (auto it = tmp_list_.begin(); it != tmp_list_.end(); ) {
    const double age = (now - rclcpp::Time(it->pose.header.stamp)).seconds();
    if (age > confirm_sec_garbage_time_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾 (%.2f, %.2f) 超过 %.1f 秒没有累计确认，删除",
        it->pose.pose.position.x, it->pose.pose.position.y, confirm_sec_garbage_time_);
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
  // 取算术平均坐标
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
  if (static_cast<int>(match_n) < confirm_match_num_) {
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
  if (isDuplicateGarbage(item, garbage_list_)) {
    return;
  }

  history_list_.push_back(std::move(item));  //塞进去，留着给后边做堆的
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
  if (footprint_topic_sub_) {
    footprint_topic_sub_->feed(msg);
  }
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

InsertGarbagePose::InsertLoopAction InsertGarbagePose::insertLockedChainAtFront(  //射线锁住的垃圾链
  InsertBatch * batch,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double rx, double ry)
{
  const std::size_t chain_n = lockedChainLengthAtFront();
  if (chain_n >= 2) {
    std::vector<std::size_t> kept;
    double fx = rx;
    double fy = ry;
    for (std::size_t i = 0; i < chain_n; ++i) {
      const double cx = garbage_list_[i].pose.pose.position.x;
      const double cy = garbage_list_[i].pose.pose.position.y;
      if (isNearReachedGarbage(cx, cy)) {
        continue;
      }
      std::string chain_fp;
      if (!isFootprintSweepClear(fx, fy, cx, cy, &chain_fp)) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: ray chain drop (%.2f, %.2f), footprint 不过 (%s)",
          cx, cy, chain_fp.c_str());
        publishFootprintCheckBox(cx, cy, std::atan2(cy - fy, cx - fx));
        addProtectedGarbageXy(cx, cy);
        continue;
      }
      kept.push_back(i);
      fx = cx;
      fy = cy;
    }
    GarbageList kept_piles;
    kept_piles.reserve(kept.size());
    for (const std::size_t i : kept) {
      kept_piles.push_back(garbage_list_[i]);
    }
    eraseGarbageFront(chain_n);
    if (kept_piles.size() >= 2) {
      std::vector<std::pair<double, double>> gxy;
      gxy.reserve(kept_piles.size());
      for (const auto & pile : kept_piles) {
        gxy.emplace_back(pile.pose.pose.position.x, pile.pose.pose.position.y);
      }
      InsertInfo info = gatherInsertInfo(
        batch->goals, robot_pose, gxy.front().first, gxy.front().second);
      if (!info.valid) {
        for (const auto & p : gxy) {
          addProtectedGarbageXy(p.first, p.second);
        }
        return InsertLoopAction::Continue;
      }
      info.dist_label = next_g_num_;
      next_g_num_ += static_cast<int>(kept_piles.size());
      info.ray_chain_xy = gxy;
      info.garbage = kept_piles.front();
      const std::size_t goals_before_pile = batch->goals.size();
      batch->goals = insertRayChainIntoGoals(info, gxy, garbage_extend_m_);
      const std::size_t pile_deleted = info.goaltotal.size();
      batch->deleted_goals_total += pile_deleted;
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: insert ray chain n=%zu G%d (%.2f, %.2f)->(%.2f, %.2f) "
        "extend=%d, deleted %zu path goals, goals %zu -> %zu",
        gxy.size(), info.dist_label,
        gxy.front().first, gxy.front().second, gxy.back().first, gxy.back().second,
        info.extend_inserted ? 1 : 0,
        pile_deleted, goals_before_pile, batch->goals.size());
      publishVisualization(info);
      publishRangeCircles(rx, ry);
      for (const auto & p : gxy) {
        addProtectedGarbageXy(p.first, p.second);
        batch->inserted_xy << "(" << p.first << ", " << p.second << ") ";
      }
      if (info.extend_inserted) {
        addProtectedGarbageXy(info.extend_x, info.extend_y);
      }
      batch->inserted_count += kept_piles.size();
      if (single_pile_insert_) {
        return InsertLoopAction::BreakLoop;
      }
      return InsertLoopAction::Continue;
    }
    if (kept_piles.size() == 1) {
      garbage_list_.insert(garbage_list_.begin(), kept_piles.front());
      pile_skip_extend_.insert(pile_skip_extend_.begin(), 0);
    } else {
      return InsertLoopAction::Continue;
    }
  }

  return InsertLoopAction::RunSinglePile;
}

InsertGarbagePose::InsertLoopAction InsertGarbagePose::insertFrontPile(
  InsertBatch * batch,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double rx, double ry)
{
  const double gx = garbage_list_.front().pose.pose.position.x;
  const double gy = garbage_list_.front().pose.pose.position.y;
  if (isNearReachedGarbage(gx, gy)) {
    eraseGarbageFront(1);
    return InsertLoopAction::Continue;
  }
  InsertInfo info = gatherInsertInfo(batch->goals, robot_pose, gx, gy);
  if (!info.valid) {
    eraseGarbageFront(1);
    return InsertLoopAction::Continue;
  }
  info.dist_label = next_g_num_++;
  const std::size_t goals_before_pile = batch->goals.size();
  std::string fp_reason;
  const bool footprint_ok = isFootprintSweepClear(
    robot_pose.pose.position.x, robot_pose.pose.position.y, gx, gy, &fp_reason);
  if (footprint_ok) {
    const InsertRollback saved = InsertRollback::snapshot(*this, batch->goals);
    batch->goals = insertGarbageIntoGoals(info);
    if (info.extend_inserted && garbage_list_.size() > 1) {
      double yaw_out = info.path_yaw;
      double ext_out = info.extend_used_m;
      const ExtendNearAction action = considerExtendNear(
        gx, gy, info.extend_x, info.extend_y, info.path_yaw, &yaw_out, &ext_out);
      if (action != ExtendNearAction::Keep) {
        saved.restore(*this, &batch->goals);
        if (action == ExtendNearAction::Rechain) {
          --next_g_num_;
          return InsertLoopAction::Continue;
        }
        InsertInfo info2 = gatherInsertInfo(batch->goals, robot_pose, gx, gy);
        if (!info2.valid) {
          eraseGarbageFront(1);
          return InsertLoopAction::Continue;
        }
        info2.path_yaw = yaw_out;
        info2.extend_length_override = true;
        info2.extend_length_m = ext_out;
        info2.dist_label = info.dist_label;
        batch->goals = insertGarbageIntoGoals(info2);
        info = std::move(info2);
      }
    }
  } else if (enable_wall_edge_insert_) {
    //  不过且启用贴墙：D-G-E
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: footprint sweep fail robot->G (%.2f, %.2f): %s, wall-edge insert",
      gx, gy, fp_reason.c_str());
    publishFootprintCheckBox(gx, gy, info.path_yaw);
    batch->goals = insertWallEdgeGarbageIntoGoals(info);
    if (!info.wall_edge_inserted) {
      addProtectedGarbageXy(gx, gy);
      eraseGarbageFront(1);
      return InsertLoopAction::Continue;
    }
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: skip garbage (%.2f, %.2f), footprint 不过且未启用贴墙 (%s)",
      gx, gy, fp_reason.c_str());
    publishFootprintCheckBox(gx, gy, info.path_yaw);
    addProtectedGarbageXy(gx, gy);
    eraseGarbageFront(1);
    return InsertLoopAction::Continue;
  }
  const std::size_t pile_deleted = info.goaltotal.size();
  batch->deleted_goals_total += pile_deleted;
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: insert G%d (%.2f, %.2f) extend=%d, deleted %zu path goals, "
    "goals %zu -> %zu",
    info.dist_label,
    info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y,
    info.extend_inserted ? 1 : 0,
    pile_deleted, goals_before_pile, batch->goals.size());
  publishVisualization(info);
  publishRangeCircles(rx, ry);
  addProtectedGarbageXy(gx, gy);
  addProtectedGarbageXy(
    info.garbage.pose.pose.position.x, info.garbage.pose.pose.position.y);
  if (info.extend_inserted) {
    addProtectedGarbageXy(info.extend_x, info.extend_y);
  }
  eraseGarbageFront(1);
  ++batch->inserted_count;
  batch->inserted_xy << "(" << gx << ", " << gy << ") ";
  if (single_pile_insert_) {
    return InsertLoopAction::BreakLoop;
  }
  return InsertLoopAction::Continue;
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

  // spin_some 对同一个订阅只取一条；这里要把本拍之前积压的垃圾都取完再合堆
  callback_group_executor_.spin_all(std::chrono::nanoseconds(0));
  refreshTunableInputPorts();
  checkAndResetOnNewMission();

  postProcessHistory();
  Goals goals_now = receiveGoals();

  if (goals_now.size() < 2) {
    geometry_msgs::msg::PoseStamped robot_pose_short;
    if (getRobotPose(robot_pose_short)) {
      const double robot_x = robot_pose_short.pose.position.x;
      const double robot_y = robot_pose_short.pose.position.y;
      publishRangeCircles(robot_x, robot_y);
      refreshClipAnchorVisualization(goals_now, robot_x, robot_y);
    }
    pruneFinishedPileVisualization(goals_now);
    publishPendingGarbageDots();
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
  pruneFinishedPileVisualization(goals_now);
  refreshClipAnchorVisualization(goals_now, rx, ry);
  publishPendingGarbageDots();

  auto logSweepOrder = [this, rx, ry]() {
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
    std::ostringstream order;
    for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
      if (i > 0) {
        order << "->";
      }
      order << dist_label[i];
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 一共 %zu 堆垃圾进入排序",
      garbage_list_.size());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 最终清扫顺序 %s",
      order.str().c_str());
  };

  if (has_work_circle_ && garbage_list_.empty() && !hasUnindexedSentinel(goals_now)) {
    RCLCPP_INFO(node_->get_logger(), "InsertGarbagePose: 工作圈取消");
    clearMissionVisualization();
    viz_obstacle_pixels_.clear();
    viz_obstacle_marker_count_ = 0;
    viz_pile_count_ = 0;
    viz_footprint_fail_count_ = 0;
    has_work_circle_ = false;
  }

  if (!garbage_list_.empty() && last_sweep_xy_.empty()) {
    planRayChainsBeforeSweep(rx, ry, robot_yaw);
    logSweepOrder();
  }

  if (garbage_list_.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  // 单堆：路径里还有 z=-1 的 G/E，不再插下一堆
  if (single_pile_insert_ && hasUnindexedSentinel(goals_now)) {
    return BT::NodeStatus::SUCCESS;
  }
  // 按当前顺序一次插入全部待插堆
  const std::size_t goals_before_batch = goals_now.size();
  InsertBatch batch;
  batch.goals = std::move(goals_now);
  while (!garbage_list_.empty()) {
    if (pile_skip_extend_.size() < garbage_list_.size()) {
      pile_skip_extend_.resize(garbage_list_.size(), 0);
    }
    const InsertLoopAction chain_action =
      insertLockedChainAtFront(&batch, robot_pose, rx, ry);
    if (chain_action == InsertLoopAction::BreakLoop) {
      break;
    }
    if (chain_action == InsertLoopAction::Continue) {
      continue;
    }
    if (insertFrontPile(&batch, robot_pose, rx, ry) == InsertLoopAction::BreakLoop) {
      break;
    }
  }
  goals_now = std::move(batch.goals);

  if (batch.inserted_count > 0) {   // 至少要插入一个点才调用
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: batch insert done: inserted %zu pile(s), deleted %zu path goals, "
      "goals %zu -> %zu, xy %s",
      batch.inserted_count, batch.deleted_goals_total,
      goals_before_batch, goals_now.size(),
      batch.inserted_xy.str().c_str());
    const rclcpp::Time now_stamp = node_->now();
    for (std::size_t i = 0; i < goals_now.size(); ++i) {
      if (!isUnindexedSentinelPoseZ(goals_now[i])) {
        goals_now[i].pose.position.z = static_cast<double>(i);
      }
      goals_now[i].header.stamp = now_stamp;
    }
    mission_stamp_record_ = now_stamp;
    has_mission_stamp_ = true;
    emitOutputGoals(goals_now, "batch_insert");
  }
  pruneFinishedPileVisualization(goals_now);
  refreshClipAnchorVisualization(goals_now, rx, ry);
  publishPendingGarbageDots();
  return BT::NodeStatus::SUCCESS;
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

  history_list_.clear();
  tmp_list_.clear();
  garbage_list_.clear();
  reached_garbage_xy_.clear();
  viz_obstacle_pixels_.clear();
  viz_obstacle_marker_count_ = 0;
  last_sweep_xy_.clear();
  pile_skip_extend_.clear();
  has_last_sweep_arrive_ = false;
  last_sweep_arrive_xy_ = {0.0, 0.0};
  has_last_sweep_path_yaw_ = false;
  last_sweep_path_yaw_ = 0.0;
  viz_pile_count_ = 0;
  viz_footprint_fail_count_ = 0;
  next_g_num_ = 1;
  remembered_corner_xy_.clear();
  has_work_circle_ = false;
  mission_stamp_record_ = current_stamp;
  clearMissionVisualization();

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 更新扫地任务");
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

// ========================================================================
// A. footprint / 碰撞检查
// ========================================================================

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

bool InsertGarbagePose::isFootprintClearAtPose(
  double x, double y, double yaw, std::string * reason) const
{
  return isCollisionFreeAtPose(x, y, yaw, reason, true);
}

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

bool InsertGarbagePose::isRobotGarbageSegmentClearOfLethal(
  double robot_x, double robot_y, double gx, double gy,
  std::string * reason) const
{
  if (!costmap_sub_) {
    if (reason) {
      *reason = "no costmap subscriber";
    }
    return false;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception &) {
    if (reason) {
      *reason = "costmap unavailable";
    }
    return false;
  }
  if (!costmap) {
    if (reason) {
      *reason = "no costmap";
    }
    return false;
  }

  unsigned int mx0 = 0;
  unsigned int my0 = 0;
  unsigned int mx1 = 0;
  unsigned int my1 = 0;
  if (!costmap->worldToMap(robot_x, robot_y, mx0, my0)) {
    if (reason) {
      *reason = "robot out of costmap";
    }
    return false;
  }
  if (!costmap->worldToMap(gx, gy, mx1, my1)) {
    if (reason) {
      *reason = "garbage out of costmap";
    }
    return false;
  }

  const int x0 = static_cast<int>(mx0);
  const int y0 = static_cast<int>(my0);
  const int x1 = static_cast<int>(mx1);
  const int y1 = static_cast<int>(my1);

  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    const int lx = line.getX();
    const int ly = line.getY();
    if (lx == x1 && ly == y1) {
      // 垃圾格常在墙边 lethal 上，贴墙进表不卡终点
      continue;
    }
    if (lx < 0 || ly < 0) {
      continue;
    }
    const unsigned int mx = static_cast<unsigned int>(lx);
    const unsigned int my = static_cast<unsigned int>(ly);
    if (mx >= costmap->getSizeInCellsX() || my >= costmap->getSizeInCellsY()) {
      if (reason) {
        *reason = "line leaves costmap";
      }
      return false;
    }
    if (costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      if (reason) {
        *reason = "lethal on robot-garbage line";
      }
      return false;
    }
  }
  return true;
}

bool InsertGarbagePose::findNearestObstaclePixel(
  double x, double y, double * ox, double * oy)
{
  if (!ox || !oy || !costmap_sub_) {
    return false;
  }

  // 在全局代价图上找最近致命障碍格
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
  const double search_radius_m = garbage_extend_m_ + 0.5;
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

// ========================================================================
// B. 垃圾后处理
// ========================================================================

// 接收到垃圾后的后处理函数，返回处理后的 garbage_list
InsertGarbagePose::GarbageList InsertGarbagePose::postProcessHistory()
{
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_copy = history_list_;

  if (history_list_copy.empty()) {
    return garbage_list_;
  }

  // base_link到map
  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!getRobotPoseXY(robot_x, robot_y)) {
    return garbage_list_;
  }

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
      for (auto it = history_list_.begin(); it != history_list_.end(); ++it) {
        if (sameDetect(*it, original)) {
          history_list_.erase(it);
          return;
        }
      }
    };

  GarbageList candidates;
  std::vector<capella_ros_msg::msg::GarbageDetect> candidate_originals;
  candidates.reserve(history_list_copy.size());
  candidate_originals.reserve(history_list_copy.size());

  // 删掉一组已消化的原始检测
  auto erasePileMembers =
    [&](const std::vector<std::size_t> & member_idx) {
      for (const std::size_t i : member_idx) {
        if (i < candidate_originals.size()) {
          eraseFromHistory(candidate_originals[i]);
        }
      }
    };

  for (const auto & original : history_list_copy) {
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
    {
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

    // GE 用 footprint 长条；enable_wall_edge_insert 时用 robot-G 连线无 lethal
    {
      std::string sweep_reason;
      const bool intake_clear = enable_wall_edge_insert_ ?
        isRobotGarbageSegmentClearOfLethal(robot_x, robot_y, gx, gy, &sweep_reason) :
        isFootprintSweepClear(robot_x, robot_y, gx, gy, &sweep_reason);
      if (!intake_clear) {
        if (enable_wall_edge_insert_) {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *(node_->get_clock()), 2000,
            "InsertGarbagePose: 垃圾=(%.2f, %.2f), 贴墙进表: 车到垃圾连线有障碍(%s), 丢弃",
            gx, gy, sweep_reason.c_str());
          publishBlockedSegmentVisualization(robot_x, robot_y, gx, gy);
        } else {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *(node_->get_clock()), 2000,
            "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为车到垃圾 footprint 长条不通过(%s), 丢弃",
            gx, gy, sweep_reason.c_str());
          publishFailedSweepVisualization(robot_x, robot_y, gx, gy);
        }
        eraseFromHistory(original);
        continue;
      }
    }

    if (!has_work_circle_ && garbage_list_.empty() &&
      !hasUnindexedSentinel(receiveGoals())) {
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

    candidates.push_back(std::move(garbage));
    candidate_originals.push_back(original);
  }

  if (candidates.empty()) {
    return garbage_list_;
  }

  // 离车最近开堆；代表点为成员质心；到质心小于合堆半径才并入
  std::vector<std::vector<std::size_t>> groups;
  GarbageList merged_garbage_list = mergeGarbagePiles(
    candidates, robot_x, robot_y, garbage_merge_radius_m_, &groups);

  for (std::size_t gi = 0; gi < merged_garbage_list.size(); ++gi) {
    auto & seed = merged_garbage_list[gi];
    const std::vector<std::size_t> & members = groups[gi];
    const double sx = seed.pose.pose.position.x;
    const double sy = seed.pose.pose.position.y;

    // 已插入过的堆不再进候选，避免持续发布反复占队首
    if (isNearReachedGarbage(sx, sy)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为已到达/已插入过, 丢弃", sx, sy);
      erasePileMembers(members);
      continue;
    }

    if (isDuplicateGarbage(seed, garbage_list_)) {
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
// 离车最近开堆；代表点为成员质心；离质心最近且小于合堆半径才并入，最近的并不上则结束本堆
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

  const double radius2 = merge_radius_m * merge_radius_m;

  auto xyOf = [&candidates](std::size_t idx) { // 对应的下标中取出它的xy
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
    std::vector<std::size_t> members{seed_idx};
    double sum_x = xyOf(seed_idx).first;
    double sum_y = xyOf(seed_idx).second;

    for (;;) {
      const double cx = sum_x / static_cast<double>(members.size());
      const double cy = sum_y / static_cast<double>(members.size());
      bool found = false;
      std::size_t best_idx = 0;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (const std::size_t idx : remaining) {
        if (std::find(members.begin(), members.end(), idx) != members.end()) {
          continue;
        }
        const auto q = xyOf(idx);
        const double d2 = squaredDistanceXY(q.first, q.second, cx, cy);
        if (d2 < best_d2) {
          best_d2 = d2;
          best_idx = idx;
          found = true;
        }
      }
      // 离当前质心最近的点都超半径，更远的不再试
      if (!found || best_d2 >= radius2) {
        break;
      }
      const auto q = xyOf(best_idx);
      members.push_back(best_idx);
      sum_x += q.first;
      sum_y += q.second;
    }

    auto pile = candidates[seed_idx];
    pile.pose.pose.position.x = sum_x / static_cast<double>(members.size());
    pile.pose.pose.position.y = sum_y / static_cast<double>(members.size());
    merged_garbage_list.push_back(std::move(pile));
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

bool InsertGarbagePose::isDuplicateGarbage(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const GarbageList & existing) const
{
  const double x = garbage.pose.pose.position.x;
  const double y = garbage.pose.pose.position.y;
  const double thresh2 = garbage_merge_radius_m_ * garbage_merge_radius_m_;

  for (const auto & pile : existing) {
    const double dx = x - pile.pose.pose.position.x;
    const double dy = y - pile.pose.pose.position.y;
    if (dx * dx + dy * dy < thresh2) {
      return true;
    }
  }
  return false;
}
// 新垃圾落在已处理点的合堆半径内就不再作为候选，防止重复插入
bool InsertGarbagePose::isNearReachedGarbage(double x, double y) const
{
  const double thresh2 = garbage_merge_radius_m_ * garbage_merge_radius_m_;
  for (const auto & reached : reached_garbage_xy_) {
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}

// ========================================================================
// C. goals 哨兵与保护点
// ========================================================================

// 判断 goals 里某点是否为本节点写入的 G/E，而不是input_goals里的点
bool InsertGarbagePose::isUnindexedSentinelPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  return std::lround(pose_stamped_goal.pose.position.z) ==
         std::lround(kGarbageSentinelPoseZ);
}

bool InsertGarbagePose::hasUnindexedSentinel(const Goals & goals)
{
  for (const auto & goal : goals) {
    if (isUnindexedSentinelPoseZ(goal)) {
      return true;
    }
  }
  return false;
}

bool InsertGarbagePose::findUnindexedSentinelIndex(  // 在整个goals里面找哨兵点
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
bool InsertGarbagePose::isProtectedGarbageXy(double x, double y) const   //判断某个已经写进 goals 的 G/E 点
{
  const double thresh2 =
    kSentinelIdentityMatchM * kSentinelIdentityMatchM;
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
}

// ========================================================================
// D. 新堆判定 + 排序
// ========================================================================

namespace
{

double wrapAngleRad(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));  // 规范化到 (-π, π]
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
  ex = gx + extend_m * std::cos(yaw);
  ey = gy + extend_m * std::sin(yaw);
}

// 一条中间顺序的得分：延长点之间的路程 + 折到 [-pi, pi] 的转角，不按最大值归一化
double routeExtendScore(
  const InsertGarbagePose::GarbageList & garbage_list,
  const std::vector<std::size_t> & garbage_order,
  double start_x, double start_y, double start_yaw,
  double extend_m, double w_dist, double w_turn)
{
  double px = start_x;
  double py = start_y;
  double heading = start_yaw;
  double total_dist = 0.0;
  double total_yaw = 0.0;
  for (const std::size_t idx : garbage_order) {
    if (idx >= garbage_list.size()) {
      continue;
    }
    const double gx = garbage_list[idx].pose.pose.position.x; //算延长点
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

}  // namespace

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

bool InsertGarbagePose::inForwardRaySector(
  double ox, double oy, double tx, double ty, double px, double py) const
{
  const double rx = tx - ox;
  const double ry = ty - oy;
  const double rlen = std::hypot(rx, ry);
  if (rlen < 1e-6) {
    return false;
  }
  const double vx = px - ox;
  const double vy = py - oy;
  const double vlen = std::hypot(vx, vy);
  if (vlen < 1e-6) {
    return false;
  }
  const double dot = vx * rx + vy * ry;
  if (dot <= 0.0) {
    return false;
  }
  const double cosang = std::clamp(dot / (vlen * rlen), -1.0, 1.0);
  const double ang = std::acos(cosang);
  return ang <= ray_offset_deg_ * M_PI / 180.0;
}

bool InsertGarbagePose::takeOneRayChain(
  GarbageList * remaining,
  double * pose_x, double * pose_y, double * pose_yaw,
  GarbageList * chain_out) const
{
  if (remaining == nullptr || chain_out == nullptr ||
    pose_x == nullptr || pose_y == nullptr || pose_yaw == nullptr ||
    remaining->size() < 2)
  {
    return false;
  }
  std::size_t nearest = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < remaining->size(); ++i) {
    const double d2 = squaredDistanceXY(
      (*remaining)[i].pose.pose.position.x,
      (*remaining)[i].pose.pose.position.y,
      *pose_x, *pose_y);
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest = i;
    }
  }
  const double g1x = (*remaining)[nearest].pose.pose.position.x;
  const double g1y = (*remaining)[nearest].pose.pose.position.y;
  // 射线方向是假设车位指向 G1，起点放在 G1，夹角在 G1 上量
  const double ray_tx = g1x + (g1x - *pose_x);
  const double ray_ty = g1y + (g1y - *pose_y);
  const std::vector<std::size_t> hit = collectSectorChain(
    g1x, g1y, ray_tx, ray_ty, remaining->size(), nearest,
    [&](std::size_t i) {
      return std::make_pair(
        (*remaining)[i].pose.pose.position.x,
        (*remaining)[i].pose.pose.position.y);
    });
  if (hit.empty()) {
    return false;
  }
  chain_out->clear();
  chain_out->reserve(hit.size());
  for (const std::size_t idx : hit) {
    chain_out->push_back((*remaining)[idx]);
  }
  std::vector<std::size_t> remove_idx = hit;
  std::sort(remove_idx.begin(), remove_idx.end(), [](std::size_t a, std::size_t b) {
    return a > b;
  });
  for (const std::size_t idx : remove_idx) {
    remaining->erase(remaining->begin() + static_cast<std::ptrdiff_t>(idx));
  }
  const auto & prev = (*chain_out)[chain_out->size() - 2];
  const auto & tail = chain_out->back();
  double ex = 0.0;
  double ey = 0.0;
  double yaw = *pose_yaw;
  extendBeyondGarbage(
    prev.pose.pose.position.x, prev.pose.pose.position.y,
    tail.pose.pose.position.x, tail.pose.pose.position.y,
    garbage_extend_m_, ex, ey, yaw);
  *pose_x = ex;
  *pose_y = ey;
  *pose_yaw = yaw;
  return true;
}

std::size_t InsertGarbagePose::planChainsThenSweepFrom(
  GarbageList remaining, double pose_x, double pose_y, double pose_yaw,
  GarbageList * ordered, std::vector<char> * skip, const char * chain_log)
{
  std::size_t chained_n = 0;
  for (;;) {
    GarbageList chain;
    if (!takeOneRayChain(&remaining, &pose_x, &pose_y, &pose_yaw, &chain)) {
      break;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: %s n=%zu (%.2f, %.2f)->(%.2f, %.2f), "
      "virtual E (%.2f, %.2f) yaw=%.3f",
      chain_log, chain.size(),
      chain.front().pose.pose.position.x, chain.front().pose.pose.position.y,
      chain.back().pose.pose.position.x, chain.back().pose.pose.position.y,
      pose_x, pose_y, pose_yaw);
    chained_n += chain.size();
    appendChainWithSkip(chain, ordered, skip);
  }
  garbage_list_ = std::move(remaining);
  reorderNearestFirstThenSweep(pose_x, pose_y, pose_yaw);
  remaining = std::move(garbage_list_);
  for (auto & pile : remaining) {
    ordered->push_back(std::move(pile));
    skip->push_back(0);
  }
  return chained_n;
}

void InsertGarbagePose::planRayChainsBeforeSweep(
  double robot_x, double robot_y, double robot_yaw)
{
  GarbageList ordered;
  std::vector<char> skip;
  const std::size_t chained_n = planChainsThenSweepFrom(
    garbage_list_, robot_x, robot_y, robot_yaw, &ordered, &skip, "lock ray chain");
  garbage_list_ = std::move(ordered);
  pile_skip_extend_ = std::move(skip);
  syncLastSweepXyFromList();
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: ray chains locked %zu, brute-force remainder %zu",
    chained_n, garbage_list_.size() - chained_n);
}

std::size_t InsertGarbagePose::nearestWaitingNear(
  double ex, double ey, std::size_t begin_idx) const
{
  const double limit2 = extend_near_radius_m_ * extend_near_radius_m_;
  std::size_t end = garbage_list_.size();
  const std::size_t flag_n = std::min(garbage_list_.size(), pile_skip_extend_.size());
  for (std::size_t i = begin_idx; i < flag_n; ++i) {
    if (pile_skip_extend_[i]) {
      end = i;
      break;
    }
  }
  std::size_t best = garbage_list_.size();
  double best_d2 = limit2;
  bool found = false;
  for (std::size_t i = begin_idx; i < end; ++i) {
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y, ex, ey);
    if (d2 <= limit2 && (!found || d2 < best_d2)) {
      found = true;
      best = i;
      best_d2 = d2;
    }
  }
  return best;
}

void InsertGarbagePose::nudgeExtendAwayFromGarbage(
  double gx, double gy, double yaw,
  double near_x, double near_y,
  double * yaw_out, double * extend_m_out) const
{
  const double ex = std::cos(yaw);
  const double ey = std::sin(yaw);
  const double vx = near_x - gx;
  const double vy = near_y - gy;
  const double cross = ex * vy - ey * vx;
  const double sign = (cross >= 0.0) ? -1.0 : 1.0;
  if (yaw_out != nullptr) {
    *yaw_out = yaw + sign * ray_offset_deg_ * M_PI / 180.0;
  }
  if (extend_m_out != nullptr) {
    *extend_m_out = garbage_extend_m_ + extend_extra_m_;
  }
}

InsertGarbagePose::ExtendNearAction InsertGarbagePose::considerExtendNear(
  double gx, double gy, double extend_x, double extend_y, double path_yaw,
  double * yaw_out, double * extend_m_out)
{
  const std::size_t near = nearestWaitingNear(extend_x, extend_y, 1);
  if (near >= garbage_list_.size()) {
    return ExtendNearAction::Keep;
  }
  const double near_x = garbage_list_[near].pose.pose.position.x;
  const double near_y = garbage_list_[near].pose.pose.position.y;
  double yaw_nudge = path_yaw;
  double ext_nudge = garbage_extend_m_ + extend_extra_m_;
  nudgeExtendAwayFromGarbage(gx, gy, path_yaw, near_x, near_y, &yaw_nudge, &ext_nudge);
  const double nex = gx + ext_nudge * std::cos(yaw_nudge);
  const double ney = gy + ext_nudge * std::sin(yaw_nudge);
  std::string reason;
  if (!isFootprintSweepClear(gx, gy, nex, ney, &reason)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: keep original E, nudged E (%.2f, %.2f) blocked (%s)",
      nex, ney, reason.c_str());
    publishFootprintCheckBox(nex, ney, yaw_nudge);
    return ExtendNearAction::Keep;
  }
  const std::size_t near2 = nearestWaitingNear(nex, ney, 1);
  const double tx = gx + std::cos(yaw_nudge);
  const double ty = gy + std::sin(yaw_nudge);
  std::size_t window_end = garbage_list_.size();
  const std::size_t flag_n = std::min(garbage_list_.size(), pile_skip_extend_.size());
  for (std::size_t i = 1; i < flag_n; ++i) {
    if (pile_skip_extend_[i]) {
      window_end = i;
      break;
    }
  }
  const std::vector<std::size_t> hits = collectSectorChain(
    gx, gy, tx, ty, window_end, 0,
    [&](std::size_t i) {
      return std::make_pair(
        garbage_list_[i].pose.pose.position.x,
        garbage_list_[i].pose.pose.position.y);
    });
  if (near2 >= garbage_list_.size() || hits.empty()) {
    if (yaw_out != nullptr) {
      *yaw_out = yaw_nudge;
    }
    if (extend_m_out != nullptr) {
      *extend_m_out = ext_nudge;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: nudge E away from (%.2f, %.2f) to (%.2f, %.2f) len=%.2f",
      near_x, near_y, nex, ney, ext_nudge);
    return ExtendNearAction::Rewrite;
  }

  GarbageList first_chain;
  first_chain.reserve(hits.size());
  std::vector<char> used(garbage_list_.size(), 0);
  for (const std::size_t idx : hits) {
    first_chain.push_back(garbage_list_[idx]);
    used[idx] = 1;
  }
  GarbageList remaining;
  for (std::size_t i = 1; i < window_end; ++i) {
    if (!used[i]) {
      remaining.push_back(garbage_list_[i]);
    }
  }
  GarbageList locked_suffix;
  std::vector<char> locked_suffix_skip;
  for (std::size_t i = window_end; i < garbage_list_.size(); ++i) {
    locked_suffix.push_back(garbage_list_[i]);
    locked_suffix_skip.push_back(
      i < pile_skip_extend_.size() ? pile_skip_extend_[i] : 0);
  }
  const double prev_x = first_chain[first_chain.size() - 2].pose.pose.position.x;
  const double prev_y = first_chain[first_chain.size() - 2].pose.pose.position.y;
  const double gn_x = first_chain.back().pose.pose.position.x;
  const double gn_y = first_chain.back().pose.pose.position.y;
  double pose_x = gn_x;
  double pose_y = gn_y;
  double pose_yaw = path_yaw;
  extendBeyondGarbage(prev_x, prev_y, gn_x, gn_y, garbage_extend_m_, pose_x, pose_y, pose_yaw);

  GarbageList ordered;
  std::vector<char> skip;
  appendChainWithSkip(first_chain, &ordered, &skip);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: E still near garbage, lock line n=%zu from G (%.2f, %.2f), "
    "virtual E (%.2f, %.2f)",
    first_chain.size(), gx, gy, pose_x, pose_y);
  planChainsThenSweepFrom(
    std::move(remaining), pose_x, pose_y, pose_yaw, &ordered, &skip,
    "lock follow-on ray chain");
  ordered.insert(ordered.end(), locked_suffix.begin(), locked_suffix.end());
  skip.insert(skip.end(), locked_suffix_skip.begin(), locked_suffix_skip.end());
  garbage_list_ = std::move(ordered);
  pile_skip_extend_ = std::move(skip);
  syncLastSweepXyFromList();
  return ExtendNearAction::Rechain;
}

std::size_t InsertGarbagePose::lockedChainLengthAtFront() const
{
  if (garbage_list_.size() < 2 || pile_skip_extend_.empty() || !pile_skip_extend_.front()) {
    return 0;
  }
  std::size_t n = 0;
  const std::size_t limit = std::min(garbage_list_.size(), pile_skip_extend_.size());
  while (n < limit && pile_skip_extend_[n]) {
    ++n;
  }
  if (n < garbage_list_.size() && n < pile_skip_extend_.size() && !pile_skip_extend_[n]) {
    return n + 1;
  }
  return n >= 2 ? n : 0;
}

void InsertGarbagePose::eraseGarbageFront(std::size_t count)
{
  if (count > garbage_list_.size()) {
    count = garbage_list_.size();
  }
  garbage_list_.erase(
    garbage_list_.begin(),
    garbage_list_.begin() + static_cast<std::ptrdiff_t>(count));
  if (count > pile_skip_extend_.size()) {
    pile_skip_extend_.clear();
  } else {
    pile_skip_extend_.erase(
      pile_skip_extend_.begin(),
      pile_skip_extend_.begin() + static_cast<std::ptrdiff_t>(count));
  }
}

void InsertGarbagePose::reorderNearestFirstThenSweep(
  double robot_x, double robot_y, double robot_yaw)
{
  trimGarbageListToCap(robot_x, robot_y);
  const std::size_t n = garbage_list_.size();
  // 没有第二堆可排：这一堆就是 first_garbage，直接返回
  if (n <= 1) {
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

  // 每堆各自在原始 input_goals 副本上跑 2.5。有延长点则同一副本先 G 后 E。
  const Goals goals = receiveGoals();
  geometry_msgs::msg::PoseStamped robot_pose_for_clip;
  robot_pose_for_clip.header.frame_id = global_frame_;
  robot_pose_for_clip.pose.position.x = robot_x;
  robot_pose_for_clip.pose.position.y = robot_y;
  robot_pose_for_clip.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(robot_yaw);
  const double extend_m = garbage_extend_m_;
  auto clipReach = [&](double gx, double gy, std::size_t & reach) -> bool {
    std::vector<std::pair<double, double>> refs;
    refs.emplace_back(gx, gy);
    {
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
    robot_x, robot_y, fx, fy, garbage_extend_m_,
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
// 返回清扫先后下标，[0] 对应下一堆；输入只读
std::vector<std::size_t> InsertGarbagePose::computeSweepOrder(
  const GarbageList & garbage_list,
  double robot_x, double robot_y,
  double robot_yaw,
  const capella_ros_msg::msg::GarbageDetect * locked_last)
{
  const std::size_t n = garbage_list.size();
  std::vector<std::size_t> garbage_order(n);
  for (std::size_t i = 0; i < n; ++i) {
    garbage_order[i] = i;
  }
  if (n == 0) {
    return garbage_order;
  }

  const double extend_m = garbage_extend_m_;
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
    return garbage_order;
  }

  std::vector<std::size_t> best = garbage_order;
  if (n <= kSweepBruteMaxN) {
    std::vector<std::size_t> perm = garbage_order;
    double best_score = std::numeric_limits<double>::infinity();
    do {
      const double score = scoreOf(perm);
      if (score < best_score - 1e-9) {
        best_score = score;
        best = perm;
      }
    } while (std::next_permutation(perm.begin(), perm.end()));
  } else {
    std::vector<std::size_t> remaining = garbage_order;
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

void InsertGarbagePose::syncLastSweepXyFromList() // 把排好的顺序保存下来
{
  last_sweep_xy_.clear();
  last_sweep_xy_.reserve(garbage_list_.size());
  for (const auto & g : garbage_list_) {
    last_sweep_xy_.emplace_back(
      g.pose.pose.position.x, g.pose.pose.position.y);
  }
}

// ========================================================================
// E. 通用几何工具
// ========================================================================

double InsertGarbagePose::squaredDistanceXY(
  double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
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

// ========================================================================
// F. 插入信息与裁剪
// ========================================================================

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
  const double back_m = std::max(garbage_extend_m_, min_from_m * 2.0);
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

namespace
{

struct ClipNode
{
  std::size_t orig{0};
  geometry_msgs::msg::PoseStamped pose;
};

}  // namespace

// 判断和删除分开。每个参考点都从同一份原始路径算自己的垂足，互不看到对方要删的点。
// 垂足在线段上时，从垂足沿后续普通点累加路程，超过 clip_extend_m 或碰到角点就停。
// 垂足在正向延长线上时，只在这个参考点自己的副本上重算下一条边。
// delete_idx 是各参考点原始下标的并集，由调用方一次删除。
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

  std::vector<ClipNode> base_nodes;
  base_nodes.reserve(goals.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    base_nodes.push_back(ClipNode{i, goals[i]});
  }

  constexpr double kEps = 1e-6;
  const double clip_m = clip_extend_m_;
  int round_i = 0;

  for (const auto & ref : refs) {
    std::vector<ClipNode> nodes = base_nodes;
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
        // 只挪这个参考点自己的副本，好让它去看下一条边。其它参考点仍从原始路径算。
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
      info.corners_kept_xy.emplace_back(cx, cy);
      std::vector<std::size_t> edge;
      edge.reserve(ordinary.size());
      for (const std::size_t idx : ordinary) {
        edge.push_back(idx);
        if (idx == C) {
          break;
        }
      }
      std::size_t next_k = edge.size();
      for (std::size_t k = 0; k < edge.size(); ++k) {
        const auto & p = live[edge[k]].pose.position;
        const double pt = lineParameterT(p.x, p.y, hx, hy, cx, cy);
        if (pt > t + kEps) {
          next_k = k;
          break;
        }
      }
      double prev_x = fx;
      double prev_y = fy;
      double accumulated = 0.0;
      for (std::size_t k = next_k; k < edge.size(); ++k) {
        const std::size_t j = edge[k];
        if (j == C) {
          break;
        }
        const double x = live[j].pose.position.x;
        const double y = live[j].pose.position.y;
        const double seg = std::hypot(x - prev_x, y - prev_y);
        if (accumulated + seg >= clip_m) {
          break;
        }
        accumulated += seg;
        if (canDelete(j)) {
          drop_orig.push_back(nodes[j].orig);
        }
        prev_x = x;
        prev_y = y;
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: clip on-seg t=%.3f H=%zu C=%zu walked=%.2f "
        "drop=%zu ref=(%.2f, %.2f)",
        t, nodes[H].orig, nodes[C].orig, accumulated, drop_orig.size(), ref_x, ref_y);
      for (const std::size_t orig : drop_orig) {
        delete_idx.insert(orig);
      }
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

// ========================================================================
// G. 角点 / 路径几何
// ========================================================================

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
  if (goals.size() < 2) {
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

// ========================================================================
// H. 普通插入
// ========================================================================

namespace
{

template<typename Pred>
bool tryYawFan(
  double yaw0, double max_deg, double step_deg, Pred pred,
  double * yaw_out, double * signed_step_deg)
{
  for (double step = step_deg; step <= max_deg + 1e-6; step += step_deg) {
    for (const double sign : {1.0, -1.0}) {
      const double yaw_try = yaw0 + sign * step * M_PI / 180.0;
      if (!pred(yaw_try)) {
        continue;
      }
      if (yaw_out != nullptr) {
        *yaw_out = yaw_try;
      }
      if (signed_step_deg != nullptr) {
        *signed_step_deg = sign * step;
      }
      return true;
    }
  }
  return false;
}

}  // namespace

void InsertGarbagePose::splitAtLastProtected(
  const Goals & goals, Goals * prefix, Goals * path) const
{
  int last_protected = -1;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (isProtectedGarbageXy(
        goals[i].pose.position.x, goals[i].pose.position.y) ||
      isUnindexedSentinelPoseZ(goals[i]))
    {
      last_protected = static_cast<int>(i);
    }
  }
  if (last_protected >= 0) {
    prefix->assign(
      goals.begin(),
      goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1);
    path->assign(
      goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1,
      goals.end());
  } else {
    prefix->clear();
    *path = goals;
  }
}

bool InsertGarbagePose::prepareClipBase(
  const Goals & prefix, const Goals & path,
  double gx, double gy, double saved_yaw,
  const capella_ros_msg::msg::GarbageDetect * saved_garbage,
  InsertInfo * info, InsertInfo * frozen)
{
  if (!prefix.empty() && path.size() >= 2) {
    InsertInfo path_info = gatherInsertInfo(path, info->robot_pose, gx, gy);
    if (!path_info.valid) {
      return false;
    }
    path_info.path_yaw = saved_yaw;
    path_info.goals = path;
    if (saved_garbage != nullptr) {
      path_info.garbage = *saved_garbage;
    }
    *frozen = path_info;
    info->goala = path_info.goala;
    info->goalc = path_info.goalc;
    info->goalc_idx = path_info.goalc_idx;
    info->goald_x = path_info.goald_x;
    info->goald_y = path_info.goald_y;
    return true;
  }
  if (path.size() >= 2 && info->valid) {
    frozen->goals = path;
    frozen->path_yaw = saved_yaw;
    if (saved_garbage != nullptr) {
      frozen->garbage = *saved_garbage;
    }
    return true;
  }
  return false;
}

InsertGarbagePose::Goals InsertGarbagePose::clipWithRefs(
  const Goals & work,
  const std::vector<std::pair<double, double>> & refs,
  const InsertInfo & frozen,
  InsertInfo * info,
  const char * log_tag)
{
  InsertInfo cinfo = frozen;
  cinfo.goals = work;
  std::set<std::size_t> del;
  clipReferencesInOrder(work, info->robot_pose, refs, del, cinfo, true);
  info->clip_rounds = std::move(cinfo.clip_rounds);
  info->corners_kept_xy = std::move(cinfo.corners_kept_xy);
  info->hit_mid_case = cinfo.hit_mid_case;
  info->hit_forward_case = cinfo.hit_forward_case;
  if (cinfo.goalc_idx < work.size()) {
    info->goalc = work[cinfo.goalc_idx];
    info->goalc_idx = cinfo.goalc_idx;
  }
  info->goala = cinfo.goala;
  info->goaltotal.clear();
  info->goaltotal.reserve(del.size());
  Goals clipped;
  clipped.reserve(work.size());
  for (std::size_t i = 0; i < work.size(); ++i) {
    if (del.count(i) == 0) {
      clipped.push_back(work[i]);
    } else {
      info->goaltotal.push_back(work[i]);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: %s %zu refs=%zu, path %zu -> %zu",
    log_tag, del.size(), refs.size(), work.size(), clipped.size());
  return clipped;
}

InsertGarbagePose::Goals InsertGarbagePose::assembleAndStamp(
  const Goals & prefix, const Goals & inserted, const Goals & rest)
{
  Goals rebuilt;
  rebuilt.reserve(prefix.size() + inserted.size() + rest.size());
  rebuilt.insert(rebuilt.end(), prefix.begin(), prefix.end());
  rebuilt.insert(rebuilt.end(), inserted.begin(), inserted.end());
  rebuilt.insert(rebuilt.end(), rest.begin(), rest.end());
  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : rebuilt) {
    pose.header.stamp = stamp_now;
  }
  mission_stamp_record_ = stamp_now;
  has_mission_stamp_ = true;
  return rebuilt;
}

void InsertGarbagePose::recordSweepArrive(double x, double y, double yaw)
{
  last_sweep_arrive_xy_ = {x, y};
  has_last_sweep_arrive_ = true;
  last_sweep_path_yaw_ = yaw;
  has_last_sweep_path_yaw_ = true;
}

InsertGarbagePose::InsertRollback InsertGarbagePose::InsertRollback::snapshot(
  const InsertGarbagePose & self, const Goals & goals)
{
  InsertRollback saved;
  saved.goals = goals;
  saved.reached = self.reached_garbage_xy_;
  saved.corners = self.remembered_corner_xy_;
  saved.arrive = self.last_sweep_arrive_xy_;
  saved.has_arrive = self.has_last_sweep_arrive_;
  saved.path_yaw = self.last_sweep_path_yaw_;
  saved.has_path_yaw = self.has_last_sweep_path_yaw_;
  return saved;
}

void InsertGarbagePose::InsertRollback::restore(
  InsertGarbagePose & self, Goals * goals) const
{
  *goals = this->goals;
  self.reached_garbage_xy_ = reached;
  self.remembered_corner_xy_ = corners;
  self.last_sweep_arrive_xy_ = arrive;
  self.has_last_sweep_arrive_ = has_arrive;
  self.last_sweep_path_yaw_ = path_yaw;
  self.has_last_sweep_path_yaw_ = has_path_yaw;
}

std::vector<std::size_t> InsertGarbagePose::collectSectorChain(
  double ox, double oy, double tx, double ty,
  std::size_t count, std::size_t seed,
  const std::function<std::pair<double, double>(std::size_t)> & at) const
{
  std::vector<std::size_t> hit;
  hit.push_back(seed);
  for (std::size_t i = 0; i < count; ++i) {
    if (i == seed) {
      continue;
    }
    const auto p = at(i);
    if (inForwardRaySector(ox, oy, tx, ty, p.first, p.second)) {
      hit.push_back(i);
    }
  }
  if (hit.size() < 2) {
    return {};
  }
  std::stable_sort(
    hit.begin(), hit.end(),
    [&](std::size_t a, std::size_t b) {
      const auto pa = at(a);
      const auto pb = at(b);
      return squaredDistanceXY(pa.first, pa.second, ox, oy) <
             squaredDistanceXY(pb.first, pb.second, ox, oy);
    });
  return hit;
}

void InsertGarbagePose::appendChainWithSkip(
  const GarbageList & chain, GarbageList * out, std::vector<char> * skip)
{
  for (std::size_t i = 0; i < chain.size(); ++i) {
    out->push_back(chain[i]);
    skip->push_back(i + 1 == chain.size() ? 0 : 1);
  }
}

// 插入真实垃圾、统一时间戳；
InsertGarbagePose::Goals InsertGarbagePose::insertGarbageIntoGoals(InsertInfo & info)
{
  const double yaw_max_deg = extend_max_yaw_deg_;
  const double yaw_step_deg = extend_step_yaw_deg_;
  Goals prefix;
  Goals path;
  splitAtLastProtected(info.goals, &prefix, &path);

  const double saved_yaw = info.path_yaw;
  const auto saved_garbage = info.garbage;
  Goals work = path;
  InsertInfo frozen = info;
  const bool have_clip_base = prepareClipBase(
    prefix, path,
    saved_garbage.pose.pose.position.x, saved_garbage.pose.pose.position.y,
    saved_yaw, &saved_garbage, &info, &frozen);
  Goals out = work;
  info.path_yaw = saved_yaw;
  info.garbage = saved_garbage;

  geometry_msgs::msg::PoseStamped garbage_pose = info.garbage.pose;
  if (garbage_pose.header.frame_id.empty() && !out.empty()) {
    garbage_pose.header.frame_id = out.front().header.frame_id;
  } else if (garbage_pose.header.frame_id.empty()) {
    garbage_pose.header.frame_id = global_frame_;
  }
  garbage_pose.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(info.path_yaw);
  garbage_pose.pose.position.z = kGarbageSentinelPoseZ;

  const double extend_m = info.extend_length_override ?
    info.extend_length_m : garbage_extend_m_;
  bool add_extend = false;
  geometry_msgs::msg::PoseStamped extend_pose = garbage_pose;
  const double gx = garbage_pose.pose.position.x;
  const double gy = garbage_pose.pose.position.y;

  auto setExtendPose = [&](double yaw, double d) {
    extend_pose.pose.position.x = gx + d * std::cos(yaw);
    extend_pose.pose.position.y = gy + d * std::sin(yaw);
    extend_pose.pose.position.z = kGarbageSentinelPoseZ;
  };
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

  setExtendPose(info.path_yaw, extend_m);
  const int pile_num = (info.dist_label > 0) ?
    info.dist_label :
    (viz_pile_count_ + 1);
  std::string extend_reason;
  if (corridorClear(&extend_reason)) {
    add_extend = true;
  } else {
    publishFootprintCheckBox(
      extend_pose.pose.position.x, extend_pose.pose.position.y,
      std::atan2(extend_pose.pose.position.y - gy, extend_pose.pose.position.x - gx));
    const double yaw0 = info.path_yaw;
    double yaw_found = yaw0;
    double signed_step = 0.0;
    if (tryYawFan(
        yaw0, yaw_max_deg, yaw_step_deg,
        [&](double yaw_try) {
          setExtendPose(yaw_try, extend_m);
          std::string sweep_reason;
          return corridorClear(&sweep_reason);
        },
        &yaw_found, &signed_step))
    {
      applyExtendYaw(yaw_found);
      add_extend = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: extend fan %+g deg after default blocked (%s) "
        "E=(%.2f, %.2f)",
        signed_step, extend_reason.c_str(),
        extend_pose.pose.position.x, extend_pose.pose.position.y);
    }
    if (!add_extend) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: skip E%d (%.2f, %.2f), 通不过: %s，无法生成",
        pile_num,
        extend_pose.pose.position.x, extend_pose.pose.position.y,
        ("默认G->E走廊(" + extend_reason + ")，±" +
        std::to_string(static_cast<int>(yaw_max_deg)) +
        "deg 扇形仍不通").c_str());
    }
  }

  info.extend_inserted = add_extend;
  info.extend_used_m = add_extend ? extend_m : 0.0;
  if (add_extend) {
    info.extend_x = extend_pose.pose.position.x;
    info.extend_y = extend_pose.pose.position.y;
    addProtectedGarbageXy(info.extend_x, info.extend_y);
    recordSweepArrive(info.extend_x, info.extend_y, info.path_yaw);
  } else {
    recordSweepArrive(gx, gy, info.path_yaw);
  }

  if (have_clip_base && work.size() >= 2) {
    std::vector<std::pair<double, double>> refs;
    refs.emplace_back(gx, gy);
    if (add_extend) {
      refs.emplace_back(
        extend_pose.pose.position.x, extend_pose.pose.position.y);
    }
    out = clipWithRefs(work, refs, frozen, &info, "union delete");
  }

  Goals inserted;
  inserted.push_back(garbage_pose);
  if (add_extend) {
    inserted.push_back(extend_pose);
  }
  out = assembleAndStamp(prefix, inserted, out);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: head-insert resume_from=%zu remain=%zu "
    "(mid=%d forward=%d)",
    static_cast<std::size_t>(0), out.size(),
    info.hit_mid_case ? 1 : 0, info.hit_forward_case ? 1 : 0);
  return out;
}

InsertGarbagePose::Goals InsertGarbagePose::insertRayChainIntoGoals(
  InsertInfo & info,
  const std::vector<std::pair<double, double>> & gxy,
  double extend_m)
{
  info.ray_chain_xy = gxy;
  if (gxy.size() < 2) {
    info.extend_inserted = false;
    return info.goals;
  }

  Goals prefix;
  Goals path;
  splitAtLastProtected(info.goals, &prefix, &path);

  const double saved_yaw = info.path_yaw;
  Goals work = path;
  InsertInfo frozen = info;
  const bool have_clip_base = prepareClipBase(
    prefix, path, gxy.front().first, gxy.front().second,
    saved_yaw, nullptr, &info, &frozen);
  info.path_yaw = saved_yaw;

  std::string frame = global_frame_;
  if (!info.goals.empty() && !info.goals.front().header.frame_id.empty()) {
    frame = info.goals.front().header.frame_id;
  }
  auto makeSentinel = [&](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = kGarbageSentinelPoseZ;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    return pose;
  };

  const double prev_x = gxy[gxy.size() - 2].first;
  const double prev_y = gxy[gxy.size() - 2].second;
  const double gn_x = gxy.back().first;
  const double gn_y = gxy.back().second;
  double ex = gn_x;
  double ey = gn_y;
  double yaw = saved_yaw;
  extendBeyondGarbage(prev_x, prev_y, gn_x, gn_y, extend_m, ex, ey, yaw);
  info.path_yaw = yaw;

  bool add_extend = false;
  std::string extend_reason;
  if (isFootprintSweepClear(gn_x, gn_y, ex, ey, &extend_reason)) {
    add_extend = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: ray chain skip E (%.2f, %.2f), 通不过: %s",
      ex, ey, extend_reason.c_str());
    publishFootprintCheckBox(ex, ey, yaw);
  }
  info.extend_inserted = add_extend;
  info.extend_used_m = add_extend ? extend_m : 0.0;
  if (add_extend) {
    info.extend_x = ex;
    info.extend_y = ey;
    addProtectedGarbageXy(ex, ey);
    recordSweepArrive(ex, ey, info.path_yaw);
  } else {
    info.extend_x = gn_x;
    info.extend_y = gn_y;
    recordSweepArrive(gn_x, gn_y, info.path_yaw);
  }

  std::vector<geometry_msgs::msg::PoseStamped> gposes;
  gposes.reserve(gxy.size());
  for (std::size_t i = 0; i < gxy.size(); ++i) {
    double seg_yaw = yaw;
    if (i + 1 < gxy.size()) {
      seg_yaw = std::atan2(
        gxy[i + 1].second - gxy[i].second, gxy[i + 1].first - gxy[i].first);
    }
    gposes.push_back(makeSentinel(gxy[i].first, gxy[i].second, seg_yaw));
  }
  const geometry_msgs::msg::PoseStamped extend_pose = makeSentinel(ex, ey, yaw);

  Goals out = work;
  if (have_clip_base && work.size() >= 2) {
    std::vector<std::pair<double, double>> refs = gxy;
    if (add_extend) {
      refs.emplace_back(ex, ey);
    }
    out = clipWithRefs(work, refs, frozen, &info, "ray chain union delete");
  } else {
    info.goaltotal.clear();
  }

  Goals inserted = gposes;
  if (add_extend) {
    inserted.push_back(extend_pose);
  }
  const Goals rebuilt = assembleAndStamp(prefix, inserted, out);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: ray chain insert n=%zu extend=%d remain=%zu",
    gxy.size(), add_extend ? 1 : 0, rebuilt.size());
  return rebuilt;
}

// ========================================================================
// I. 贴边插入
// ========================================================================

InsertGarbagePose::WallEdgeExtendChain InsertGarbagePose::buildWallEdgeExtendChain(
  const InsertInfo & info)
{
  WallEdgeExtendChain chain;
  const double yaw_max_deg = extend_max_yaw_deg_;
  const double yaw_step_deg = extend_step_yaw_deg_;

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

  const double extend_m = garbage_extend_m_;
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
      double yaw_found = e_yaw;
      double signed_step = 0.0;
      if (tryYawFan(
          e_yaw, yaw_max_deg, yaw_step_deg,
          [&](double yaw_try) {
            const double ex_try = gx0 + extend_m * std::cos(yaw_try);
            const double ey_try = gy0 + extend_m * std::sin(yaw_try);
            std::string sweep_reason;
            return footClear(ex_try, ey_try, yaw_try, &sweep_reason);
          },
          &yaw_found, &signed_step))
      {
        e_yaw = yaw_found;
        e_tx = std::cos(e_yaw);
        e_ty = std::sin(e_yaw);
        ex = gx0 + extend_m * e_tx;
        ey = gy0 + extend_m * e_ty;
        e_ok = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge E sweep %+g deg -> (%.2f, %.2f)",
          signed_step, ex, ey);
      }
      if (!e_ok) {
        chain.invalid_reason = "E footprint fail after sweep: " + e_reason;
        return chain;
      }
    }
  }

  // D：生成时与 E 对侧绑定；安全调整时 E 不动，只独立扫角/加长 D
  double d_yaw = std::atan2(-e_ty, -e_tx);
  const double step = wall_edge_d_extend_m_;
  const double gd = wall_edge_min_robot_dist_m_;
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
      double yaw_found = d_yaw;
      double signed_step = 0.0;
      if (tryYawFan(
          d_yaw, yaw_max_deg, yaw_step_deg,
          [&](double yaw_try) {
            const double len_try = pushLenForGd(yaw_try);
            double dx_try = 0.0;
            double dy_try = 0.0;
            placeD(yaw_try, len_try, &dx_try, &dy_try);
            const double yaw_trav = std::atan2(gy0 - dy_try, gx0 - dx_try);
            std::string sweep_reason;
            return footClear(dx_try, dy_try, yaw_trav, &sweep_reason) &&
                   deLineClear(dx_try, dy_try, ex, ey);
          },
          &yaw_found, &signed_step))
      {
        d_yaw = yaw_found;
        d_len = pushLenForGd(d_yaw);
        placeD(d_yaw, d_len, &dx, &dy);
        d_ok = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge D sweep %+g deg -> (%.2f, %.2f)",
          signed_step, dx, dy);
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

  // 先在切线上采样 D…G…E，再整链沿 n 往外挪
  const double chain_len = std::hypot(ex - dx, ey - dy);
  const double spacing = wall_edge_sample_m_;
  std::vector<std::pair<double, double>> chain_xy;
  if (chain_len < 1e-6) {
    chain_xy.push_back({gx0, gy0});
  } else {
    const double ux = (ex - dx) / chain_len;
    const double uy = (ey - dy) / chain_len;
    const int n_seg = std::max(
      1, static_cast<int>(std::ceil(chain_len / spacing)));
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

  if (std::fabs(off) > 1e-9) {
    std::string post_reason;
    const double yaw_trav_d = std::atan2(gy - dy, gx - dx);
    if (!isFootprintClearAtPose(dx, dy, yaw_trav_d, &post_reason) ||
      !isFootprintSweepClear(dx, dy, ex, ey, &post_reason))
    {
      chain.invalid_reason =
        "wall offset breaks footprint: " + post_reason;
      return chain;
    }
  }

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
    "n=%zu sample=%.2f offset=%.2f P=(%.2f, %.2f)",
    dx, dy, gx, gy, ex, ey, chain.xy.size(), spacing, off, px, py);
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

  Goals prefix;
  Goals path;
  splitAtLastProtected(info.goals, &prefix, &path);
  const double saved_yaw = info.path_yaw;
  const auto saved_garbage = info.garbage;
  Goals out = path;
  InsertInfo frozen = info;
  const bool have_clip_base = prepareClipBase(
    prefix, path, gx, gy, saved_yaw, &saved_garbage, &info, &frozen);
  if (have_clip_base && path.size() >= 2) {
    const std::vector<std::pair<double, double>> refs = {{gx, gy}};
    out = clipWithRefs(path, refs, frozen, &info, "union delete");
  }
  info.path_yaw = saved_yaw;
  info.garbage = saved_garbage;

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
    pose.pose.position.z = kGarbageSentinelPoseZ;
    pose.pose.orientation = orient;
    chain_poses.push_back(pose);
  }

  out = assembleAndStamp(prefix, chain_poses, out);
  recordSweepArrive(ex, ey, e_yaw);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: wall-edge insert chain=%zu resume_from=%zu remain=%zu",
    chain_poses.size(), static_cast<std::size_t>(0), out.size());
  return out;
}

// ========================================================================
// J. 可视化
// ========================================================================

namespace
{

visualization_msgs::msg::Marker makeMarker(
  const std::string & frame, const rclcpp::Time & stamp,
  const std::string & ns, int id, int type,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame;
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
  m.color.a = a;
  return m;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<geometry_msgs::msg::Point> footprintAt(
  const std::vector<std::pair<double, double>> & fp,
  double x, double y, double yaw, double z)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<geometry_msgs::msg::Point> pts;
  pts.reserve(fp.size());
  for (const auto & off : fp) {
    geometry_msgs::msg::Point p;
    p.x = x + off.first * c - off.second * s;
    p.y = y + off.first * s + off.second * c;
    p.z = z;
    pts.push_back(p);
  }
  return pts;
}

}  // namespace

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
void InsertGarbagePose::clearVisualizationTopics()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = global_frame_;
  clear.header.stamp = node_->now();
  clear.ns = "";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  auto send = [&](const auto & pub) {
    if (pub) {
      pub->publish(arr);
    }
  };
  send(workspace_circle_pub_);
  send(anchor_point_pub_);
  send(footprint_check_pub_);
  send(garbage_pose_pub_);
}

bool InsertGarbagePose::skipVisualization()
{
  if (enable_visualization_) {
    viz_switch_cleared_ = false;
    return false;
  }
  if (!viz_switch_cleared_) {
    clearVisualizationTopics();
    resetVisualizationState();
    viz_switch_cleared_ = true;
  }
  return true;
}

void InsertGarbagePose::publishMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const visualization_msgs::msg::MarkerArray & arr)
{
  if (!enable_visualization_ || !pub || arr.markers.empty()) {
    return;
  }
  pub->publish(arr);
}

// 新任务 / 工作圈取消：清空四个话题上的全部 Marker
void InsertGarbagePose::clearMissionVisualization()
{
  resetVisualizationState();
  clearVisualizationTopics();
  viz_switch_cleared_ = !enable_visualization_;
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
    auto fill = makeMarker(
      global_frame_, stamp, ns, id_base + 1,
      visualization_msgs::msg::Marker::TRIANGLE_LIST, r, g, b, a_fill);
    fill.scale.x = 1.0;
    fill.scale.y = 1.0;
    fill.scale.z = 1.0;
    fill.points = {sl, sr, el, sr, er, el};
    arr.markers.push_back(fill);
  }

  auto outline = makeMarker(
    global_frame_, stamp, ns, id_base,
    visualization_msgs::msg::Marker::LINE_LIST, r, g, b, a_line);
  outline.scale.x = 0.025;
  auto addPose = [&](double x, double y) {
    const std::vector<geometry_msgs::msg::Point> pts = footprintAt(fp, x, y, yaw, 0.05);
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
  if (skipVisualization()) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  appendFootprintStripMarkers(
    arr, "footprint_strip_fail", 0, rx, ry, gx, gy,
    0.95f, 0.20f, 0.08f, 0.95f, 0.28f);

  auto ball = makeMarker(
    global_frame_, node_->now(), "footprint_strip_fail", 2,
    visualization_msgs::msg::Marker::SPHERE, 0.95f, 0.10f, 0.10f, 1.0f);
  ball.pose.position.x = gx;
  ball.pose.position.y = gy;
  ball.pose.position.z = 0.12;
  ball.scale.x = ball.scale.y = ball.scale.z = 0.22;
  arr.markers.push_back(ball);

  viz_have_fail_strip_ = true;
  publishMarkers(footprint_check_pub_, arr);
}

void InsertGarbagePose::publishBlockedSegmentVisualization(
  double rx, double ry, double gx, double gy)
{
  if (skipVisualization()) {
    return;
  }
  auto line = makeMarker(
    global_frame_, node_->now(), "segment_check_fail", 0,
    visualization_msgs::msg::Marker::LINE_STRIP, 0.95f, 0.15f, 0.10f, 1.0f);
  line.scale.x = 0.06;
  line.points.push_back(makePoint(rx, ry, 0.08));
  line.points.push_back(makePoint(gx, gy, 0.08));

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(line);
  publishMarkers(footprint_check_pub_, arr);
}

void InsertGarbagePose::deletePileVisualization(int pile_num)
{
  if (!enable_visualization_ || pile_num <= 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray garbage_arr;
  visualization_msgs::msg::MarkerArray anchor_arr;
  visualization_msgs::msg::MarkerArray footprint_arr;
  const int base = pile_num * 10;
  appendDeleteMarker(garbage_arr, "garbage", base);
  appendDeleteMarker(garbage_arr, "garbage", base + 1);
  appendDeleteMarker(anchor_arr, "ray_chain", pile_num);
  for (int i = 0; i < 8; ++i) {
    appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i);
    appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i + 100);
  }
  appendDeleteMarker(anchor_arr, "extend", base);
  appendDeleteMarker(anchor_arr, "extend", base + 1);
  appendDeleteMarker(anchor_arr, "extend", base + 2);
  appendDeleteMarker(footprint_arr, "footprint_strip", base);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 1);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 2);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 3);
  constexpr int kWallEdgeIdBase = 8000;
  constexpr int kWallEdgeIdSpan = 128;
  const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
  for (int i = 0; i < kWallEdgeIdSpan; ++i) {
    appendDeleteMarker(anchor_arr, "wall_edge_pts", wbase + i);
  }
  publishMarkers(garbage_pose_pub_, garbage_arr);
  publishMarkers(anchor_point_pub_, anchor_arr);
  publishMarkers(footprint_check_pub_, footprint_arr);
}

void InsertGarbagePose::pruneFinishedPileVisualization(const Goals & goals)
{
  if (skipVisualization() || viz_tracks_.empty()) {
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
  if (skipVisualization()) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    auto ball = makeMarker(
      global_frame_, stamp, "garbage_pending", static_cast<int>(i),
      visualization_msgs::msg::Marker::SPHERE, 0.95f, 0.10f, 0.10f, 1.0f);
    ball.pose.position.x = garbage_list_[i].pose.pose.position.x;
    ball.pose.position.y = garbage_list_[i].pose.pose.position.y;
    ball.pose.position.z = 0.12;
    ball.scale.x = ball.scale.y = ball.scale.z = 0.20;
    arr.markers.push_back(ball);
  }
  for (std::size_t i = garbage_list_.size(); i < viz_pending_marker_count_; ++i) {
    appendDeleteMarker(arr, "garbage_pending", static_cast<int>(i));
  }
  viz_pending_marker_count_ = garbage_list_.size();
  publishMarkers(garbage_pose_pub_, arr);
}

void InsertGarbagePose::refreshClipAnchorVisualization(
  const Goals & goals, double robot_x, double robot_y)
{
  if (skipVisualization()) {
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
    auto ball = makeMarker(
      global_frame_, stamp, ns, 0,
      visualization_msgs::msg::Marker::SPHERE, r, g, b, 1.0f);
    ball.pose.position.x = x;
    ball.pose.position.y = y;
    ball.pose.position.z = 0.14;
    ball.scale.x = ball.scale.y = ball.scale.z = 0.18;
    arr.markers.push_back(ball);

    auto text = makeMarker(
      global_frame_, stamp, ns, 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, r, g, b, 1.0f);
    text.pose.position.x = x;
    text.pose.position.y = y;
    text.pose.position.z = 0.42;
    text.scale.z = 0.22;
    text.text = label;
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
  publishMarkers(anchor_point_pub_, arr);
}
// footprint 检查不通过时，把当时检查用的 footprint 框画在垃圾位置上（空心蓝框）
void InsertGarbagePose::publishFootprintCheckBox(double x, double y, double yaw)
{
  if (skipVisualization()) {
    return;
  }
  std::vector<std::pair<double, double>> local_xy;
  if (!getRobotFootprintInBase(local_xy) || local_xy.size() < 3) {
    return;
  }

  auto box = makeMarker(
    global_frame_, node_->now(), "footprint_check_fail",
    static_cast<int>(viz_footprint_fail_count_),
    visualization_msgs::msg::Marker::LINE_STRIP, 0.15f, 0.40f, 0.95f, 1.0f);
  box.scale.x = 0.05;
  box.points = footprintAt(local_xy, x, y, yaw, 0.05);
  if (!box.points.empty()) {
    box.points.push_back(box.points.front());   // 闭合矩形
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(box);
  publishMarkers(footprint_check_pub_, arr);
  ++viz_footprint_fail_count_;
}

void InsertGarbagePose::publishRangeCircles(double robot_x, double robot_y)
{
  if (skipVisualization()) {
    return;
  }

  auto make_circle = [this](
    const std::string & ns, double cx, double cy, double radius,
    float r, float g, float b, float a, double width)
  {
    auto m = makeMarker(
      global_frame_, node_->now(), ns, 0,
      visualization_msgs::msg::Marker::LINE_STRIP, r, g, b, a);
    m.scale.x = width;
    constexpr int n = 72;
    m.points.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      m.points.push_back(makePoint(
        cx + radius * std::cos(ang), cy + radius * std::sin(ang), 0.05));
    }
    return m;
  };

  visualization_msgs::msg::MarkerArray circles;
  visualization_msgs::msg::MarkerArray obstacles;
  // 可视化生命周期 = 工作圈生命周期：生成工作圈时开始，取消时被整体删除；
  // 没有工作圈时不画圈，避免"取消后又被下一帧画回来"
  if (has_work_circle_) {
    circles.markers.push_back(
      make_circle(
        "detect_range", robot_x, robot_y, max_garbage_robot_dist_m_,
        0.55f, 0.95f, 0.50f, 0.90f, 0.05));
    circles.markers.push_back(
      make_circle(
        "work_circle", work_circle_x_, work_circle_y_, work_circle_radius_m_,
        0.02f, 0.40f, 0.10f, 0.95f, 0.08));
  }
  const double cell = viz_obstacle_cell_m_ * 1.4;
  constexpr int kObstacleTextIdBase = 1000;
  for (std::size_t i = 0; i < viz_obstacle_pixels_.size(); ++i) {
    const auto & obs = viz_obstacle_pixels_[i];
    const rclcpp::Time stamp = node_->now();
    auto box = makeMarker(
      global_frame_, stamp, "nearest_obstacle", static_cast<int>(i),
      visualization_msgs::msg::Marker::CUBE, 0.95f, 0.12f, 0.10f, 0.95f);
    box.pose.position.x = obs.x;
    box.pose.position.y = obs.y;
    box.pose.position.z = 0.08;
    box.scale.x = cell;
    box.scale.y = cell;
    box.scale.z = 0.04;
    obstacles.markers.push_back(box);

    auto text = makeMarker(
      global_frame_, stamp, "nearest_obstacle",
      kObstacleTextIdBase + static_cast<int>(i),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.95f, 0.12f, 0.10f, 1.0f);
    text.pose.position.x = obs.x;
    text.pose.position.y = obs.y;
    text.pose.position.z = 0.38;
    text.scale.z = 0.22;
    {
      std::ostringstream oss;
      const int n = (obs.pile_num > 0) ? obs.pile_num : static_cast<int>(i + 1);
      oss << "P" << n;
      text.text = oss.str();
    }
    obstacles.markers.push_back(text);
  }
  for (std::size_t i = viz_obstacle_pixels_.size(); i < viz_obstacle_marker_count_; ++i) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = global_frame_;
    del.header.stamp = node_->now();
    del.ns = "nearest_obstacle";
    del.id = static_cast<int>(i);
    del.action = visualization_msgs::msg::Marker::DELETE;
    obstacles.markers.push_back(del);

    visualization_msgs::msg::Marker del_text = del;
    del_text.id = kObstacleTextIdBase + static_cast<int>(i);
    obstacles.markers.push_back(del_text);
  }
  viz_obstacle_marker_count_ = viz_obstacle_pixels_.size();
  publishMarkers(workspace_circle_pub_, circles);
  publishMarkers(anchor_point_pub_, obstacles);
}
// 往 RViz 发本次插入：G 在 insert_garbage_pose，锚点/虚线在 insert_anchor_point，长条在 insert_footprint_check
void InsertGarbagePose::publishVisualization(const InsertInfo & info)
{
  if (skipVisualization()) {
    return;
  }

  visualization_msgs::msg::MarkerArray garbage_arr;
  visualization_msgs::msg::MarkerArray anchor_arr;
  visualization_msgs::msg::MarkerArray footprint_arr;
  const rclcpp::Time stamp = node_->now();

  const int pile_idx = viz_pile_count_;
  const int pile_num = (info.dist_label > 0) ? info.dist_label : (pile_idx + 1);

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;
  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;

  if (viz_have_fail_strip_) {
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 0);
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 1);
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 2);
    viz_have_fail_strip_ = false;
  }
  appendDeleteMarker(footprint_arr, "segment_check_fail", 0);

  const int base = pile_num * 10;
    constexpr double kGarbageDotZM = 0.12;
    constexpr double kGarbageDotSizeM = 0.22;
    constexpr double kDashLenM = 0.12;
    constexpr double kGapLenM = 0.08;
    constexpr double kDashLineWidthM = 0.030;

    auto makeSolidDot = [&](const std::string & ns, int id, double x, double y,
        float r, float g, float b, double size)
    {
      auto dot = makeMarker(
        global_frame_, stamp, ns, id, visualization_msgs::msg::Marker::SPHERE,
        r, g, b, 1.0f);
      dot.pose.position.x = x;
      dot.pose.position.y = y;
      dot.pose.position.z = kGarbageDotZM;
      dot.scale.x = size;
      dot.scale.y = size;
      dot.scale.z = size;
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
          line.points.push_back(makePoint(x0 + ux * s, y0 + uy * s, z));
          line.points.push_back(makePoint(x0 + ux * s_next, y0 + uy * s_next, z));
        }
        s = s_next;
        draw = !draw;
      }
    };

    garbage_arr.markers.push_back(
      makeSolidDot("garbage", base, gx, gy, 0.95f, 0.10f, 0.10f, kGarbageDotSizeM));

    auto t = makeMarker(
      global_frame_, stamp, "garbage", base + 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.95f, 0.10f, 0.10f, 1.0f);
    t.pose.position.x = gx;
    t.pose.position.y = gy;
    t.pose.position.z = 0.42;
    t.scale.z = 0.22;
    {
      std::ostringstream oss;
      oss << "G" << pile_num;
      t.text = oss.str();
    }
    garbage_arr.markers.push_back(t);

    if (info.ray_chain_xy.size() >= 2) {
      auto chain_line = makeMarker(
        global_frame_, stamp, "ray_chain", pile_num,
        visualization_msgs::msg::Marker::LINE_STRIP, 0.95f, 0.10f, 0.10f, 0.95f);
      chain_line.scale.x = 0.04;
      for (const auto & p : info.ray_chain_xy) {
        chain_line.points.push_back(makePoint(p.first, p.second, 0.08));
      }
      anchor_arr.markers.push_back(chain_line);
      for (std::size_t i = 1; i < info.ray_chain_xy.size(); ++i) {
        const int cid = pile_num * 10 + static_cast<int>(i);
        garbage_arr.markers.push_back(
          makeSolidDot(
            "ray_chain_g", cid,
            info.ray_chain_xy[i].first, info.ray_chain_xy[i].second,
            0.95f, 0.10f, 0.10f, kGarbageDotSizeM));
        auto ct = makeMarker(
          global_frame_, stamp, "ray_chain_g", cid + 100,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.95f, 0.10f, 0.10f, 1.0f);
        ct.pose.position.x = info.ray_chain_xy[i].first;
        ct.pose.position.y = info.ray_chain_xy[i].second;
        ct.pose.position.z = 0.42;
        ct.scale.z = 0.22;
        {
          std::ostringstream oss;
          oss << "G" << (pile_num + static_cast<int>(i));
          ct.text = oss.str();
        }
        garbage_arr.markers.push_back(ct);
      }
    }

    appendFootprintStripMarkers(
      footprint_arr, "footprint_strip", base, rx, ry, gx, gy,
      1.00f, 0.55f, 0.05f, 0.90f, 0.22f);

    if (info.extend_inserted) {
      const double ex = info.extend_x;
      const double ey = info.extend_y;
      double line_x = gx;
      double line_y = gy;
      if (info.ray_chain_xy.size() >= 2) {
        line_x = info.ray_chain_xy.back().first;
        line_y = info.ray_chain_xy.back().second;
      }

      auto ge_line = makeMarker(
        global_frame_, stamp, "extend", base + 2,
        visualization_msgs::msg::Marker::LINE_LIST, 0.15f, 0.40f, 0.95f, 0.90f);
      ge_line.scale.x = kDashLineWidthM;
      appendDashedLine(ge_line, line_x, line_y, ex, ey);
      if (!ge_line.points.empty()) {
        anchor_arr.markers.push_back(ge_line);
      }

      anchor_arr.markers.push_back(
        makeSolidDot("extend", base, ex, ey, 0.15f, 0.40f, 0.95f, kGarbageDotSizeM));

      auto te = makeMarker(
        global_frame_, stamp, "extend", base + 1,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.15f, 0.40f, 0.95f, 1.0f);
      te.pose.position.x = ex;
      te.pose.position.y = ey;
      te.pose.position.z = 0.42;
      te.scale.z = 0.22;
      {
        std::ostringstream oss;
        oss << "E" << pile_num;
        te.text = oss.str();
      }
      anchor_arr.markers.push_back(te);

      appendFootprintStripMarkers(
        footprint_arr, "footprint_strip", base + 2, line_x, line_y, ex, ey,
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

      anchor_arr.markers.push_back(
        makeSolidDot(
          "wall_edge_pts", wbase + wid++, info.wall_edge_d_x, info.wall_edge_d_y,
          kBlueR, kBlueG, kBlueB, kGarbageDotSizeM));
      auto td = makeMarker(
        global_frame_, stamp, "wall_edge_pts", wbase + wid++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, kBlueR, kBlueG, kBlueB, 1.0f);
      td.pose.position.x = info.wall_edge_d_x;
      td.pose.position.y = info.wall_edge_d_y;
      td.pose.position.z = 0.42;
      td.scale.z = 0.22;
      {
        std::ostringstream oss;
        oss << "D" << pile_num;
        td.text = oss.str();
      }
      anchor_arr.markers.push_back(td);

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
        anchor_arr.markers.push_back(
          makeSolidDot(
            "wall_edge_pts", wbase + wid++, p.first, p.second,
            kBlueR, kBlueG, kBlueB, kMidDotSizeM));
      }
    }

    VizPileTrack track;
    track.pile_num = pile_num;
    track.gx = gx;
    track.gy = gy;
    if (!info.ray_chain_xy.empty()) {
      track.gx = info.ray_chain_xy.back().first;
      track.gy = info.ray_chain_xy.back().second;
    }
    track.has_e = info.extend_inserted;
    track.ex = info.extend_x;
    track.ey = info.extend_y;
    viz_tracks_.push_back(track);

  publishMarkers(garbage_pose_pub_, garbage_arr);
  publishMarkers(anchor_point_pub_, anchor_arr);
  publishMarkers(footprint_check_pub_, footprint_arr);
  ++viz_pile_count_;
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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InsertGarbagePose>("InsertGarbagePose");
}
