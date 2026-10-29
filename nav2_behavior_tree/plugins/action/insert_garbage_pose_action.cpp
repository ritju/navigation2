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

// 行为树参数、tick，以及写出 output_goals


InsertGarbagePose::InsertGarbagePose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  garbage_topic_("/garbage_rectangle_points"),
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
  getBehaviorTreeParameters();

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

void InsertGarbagePose::getBehaviorTreeParameters()
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
  getBehaviorTreeParameters();
  isNewMission();

  // 本拍只查一次 TF，后处理和插入共用这份位姿
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return BT::NodeStatus::SUCCESS;
  }
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  postProcessHistory(robot_pose);
  Goals goals_now = receiveGoals();

  // 少于 2 个点无法投影删点，只清理开放射线。排序和插入不跑
  if (goals_now.size() < 2) {
    pruneOpenGarbageRays(goals_now);
  } else {
    if (has_work_circle_ && garbage_list_.empty() && !hasGoalsZpoints(goals_now)) {
      RCLCPP_INFO(node_->get_logger(), "InsertGarbagePose: 工作圈取消");
      has_work_circle_ = false;
    }

    pruneOpenGarbageRays(goals_now);
    bool open_ray_goals_changed = false;
    std::vector<std::pair<double, double>> open_ray_clip_refs;
    if (!garbage_list_.empty()) {
      open_ray_goals_changed = attachNewGarbageToOpenRays(
        &goals_now, robot_pose, &open_ray_clip_refs);
    }

    if (!garbage_list_.empty() && planned_units_.empty()) {
      // 有上一离开点（上一堆的 E，没有 E 则是 G）就从那个点指向本堆再伸 E。
      // 没有上一离开点（新任务）才用当前车位。
      double plan_x = rx;
      double plan_y = ry;
      double plan_yaw = robot_yaw;
      const char * plan_src = "robot";
      if (has_last_sweep_arrive_) {
        plan_x = last_sweep_arrive_xy_.first;
        plan_y = last_sweep_arrive_xy_.second;
        if (has_last_sweep_path_yaw_) {
          plan_yaw = last_sweep_path_yaw_;
        }
        plan_src = "arrive";
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 规划来向 %s (%.2f, %.2f)",
        plan_src, plan_x, plan_y);
      garbageSortTotal(plan_x, plan_y, plan_yaw);
      logSortedInsertList(plan_x, plan_y, plan_yaw);
    }

    const bool wait_single_pile =
      single_pile_insert_ && hasGoalsZpoints(goals_now);
    if (garbage_list_.empty()) {
      if (open_ray_goals_changed) {
        Goals clip_prefix;
        Goals clip_tail;
        splitAtLastProtected(goals_now, &clip_prefix, &clip_tail);
        std::set<std::size_t> pending_delete;
        deferOpenRayClip(clip_tail, open_ray_clip_refs, robot_pose, &pending_delete);
        applyDeferredTailDelete(&goals_now, clip_tail, pending_delete);
        const rclcpp::Time now_stamp = node_->now();
        for (std::size_t i = 0; i < goals_now.size(); ++i) {
          if (!isUnindexedSentinelPoseZ(goals_now[i])) {
            goals_now[i].pose.position.z = static_cast<double>(i);
          }
          goals_now[i].header.stamp = now_stamp;
        }
        mission_stamp_record_ = now_stamp;
        has_mission_stamp_ = true;
        emitOutputGoals(goals_now, "open_ray_chain");
      }
    } else if (!wait_single_pile) {
      const std::size_t goals_before_batch = goals_now.size();
      InsertBatch batch;
      batch.goals = std::move(goals_now);
      {
        Goals clip_prefix;
        splitAtLastProtected(batch.goals, &clip_prefix, &batch.clip_tail);
      }
      deferOpenRayClip(
        batch.clip_tail, open_ray_clip_refs, robot_pose, &batch.pending_delete);
      while (!planned_units_.empty()) {
        const InsertLoopAction action = insertSweepUnit(&batch, robot_pose, rx, ry);
        if (action == InsertLoopAction::BreakLoop) {
          break;
        }
        if (single_pile_insert_) {
          break;
        }
      }
      batch.deleted_goals_total = applyDeferredTailDelete(
        &batch.goals, batch.clip_tail, batch.pending_delete);
      goals_now = std::move(batch.goals);

      if (batch.inserted_count > 0 || open_ray_goals_changed) {
        if (batch.inserted_count > 0) {
          RCLCPP_INFO(
            node_->get_logger(),
            "InsertGarbagePose: batch insert done: inserted %zu pile(s), deleted %zu path goals, "
            "goals %zu -> %zu, xy %s",
            batch.inserted_count, batch.deleted_goals_total,
            goals_before_batch, goals_now.size(),
            batch.inserted_xy.str().c_str());
        }
        const rclcpp::Time now_stamp = node_->now();
        for (std::size_t i = 0; i < goals_now.size(); ++i) {
          if (!isUnindexedSentinelPoseZ(goals_now[i])) {
            goals_now[i].pose.position.z = static_cast<double>(i);
          }
          goals_now[i].header.stamp = now_stamp;
        }
        mission_stamp_record_ = now_stamp;
        has_mission_stamp_ = true;
        emitOutputGoals(
          goals_now, batch.inserted_count > 0 ? "batch_insert" : "open_ray_chain");
      }
    }
  }

  visPublishRangeCircles(rx, ry);
  visPublishGarbageExtend();
  pruneFinishedPileVisualization(goals_now);
  visPublishClipAnchors(goals_now, rx, ry);
  return BT::NodeStatus::SUCCESS;
}
//看当前是不是一次新的导航任务
void InsertGarbagePose::isNewMission()
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
  open_garbage_rays_.clear();
  planned_units_.clear();
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
  viz_obstacle_pixels_.clear();
  viz_obstacle_marker_count_ = 0;

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
