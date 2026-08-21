// Copyright (c) 2021 Samsung Research America
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

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <mutex>
#include <vector>
#include <algorithm>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5),
  accumulate_distance_(8.0),
  previous_goal_queue_front_stamp_seconds_(0.0)
{
  getInput("radius", viapoint_achieved_radius_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("accumulate_distance", accumulate_distance_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);

  callback_group_exclusive_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_exclusive_.add_callback_group(
    callback_group_exclusive_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions subscription_options_exclusive;
  subscription_options_exclusive.callback_group = callback_group_exclusive_;

  rclcpp::QoS qos_debug_paths(rclcpp::KeepLast(5));
  publisher_removed_plan_debug_ = node->create_publisher<nav_msgs::msg::Path>("removed_plan", qos_debug_paths);
  publisher_passed_pose_indexes_ = node->create_publisher<capella_ros_msg::msg::PassedPosesIndex>(
    "passed_pose_indexes", qos_debug_paths);

  std::string topic_name_teb_global_plan;
  getInput("teb_topic", topic_name_teb_global_plan);
  rclcpp::QoS qos_teb_plan_keep_last_one(rclcpp::KeepLast(1));
  subscriber_teb_global_plan_ = node->create_subscription<nav_msgs::msg::Path>(
    topic_name_teb_global_plan, qos_teb_plan_keep_last_one,
    std::bind(&RemovePassedGoals::handleReceivedTebGlobalPlan, this, std::placeholders::_1),
    subscription_options_exclusive);

  std::string topic_name_odometry;
  getInput("odom_topic", topic_name_odometry);
  subscriber_odometry_ = node->create_subscription<nav_msgs::msg::Odometry>(
    topic_name_odometry, qos_debug_paths,
    std::bind(&RemovePassedGoals::handleReceivedOdometry, this, std::placeholders::_1),
    subscription_options_exclusive);

  rclcpp::QoS qos_enable_backward(rclcpp::KeepLast(1));
  qos_enable_backward.transient_local();
  qos_enable_backward.reliable();
  publisher_enable_backward_ = node->create_publisher<std_msgs::msg::Bool>(
    "/enable_backward", qos_enable_backward);

  clock_timestamp_last_valid_teb_plan_message_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void RemovePassedGoals::publishEnableBackwardIfChanged(bool desired_backward)
{
  if (!publisher_enable_backward_) {
    return;
  }
  if (has_published_enable_backward_ && desired_backward == last_published_enable_backward_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = desired_backward;
  publisher_enable_backward_->publish(msg);
  last_published_enable_backward_ = desired_backward;
  has_published_enable_backward_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "[RemovePassedGoals] /enable_backward published: %s",
    desired_backward ? "true (backward)" : "false (forward)");
}

bool RemovePassedGoals::tryMissionPoseGoalIndexFromPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal,
  uint32_t & discrete_goal_index_z)
{
  const long rounded_goal_index_z = std::lround(pose_stamped_goal.pose.position.z);
  if (rounded_goal_index_z < 0) {
    return false;
  }
  discrete_goal_index_z = static_cast<uint32_t>(rounded_goal_index_z);
  return true;
}

bool RemovePassedGoals::isUnindexedSentinelPoseZ(const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  return std::lround(pose_stamped_goal.pose.position.z) < 0;
}

bool RemovePassedGoals::tryIndexedGoalZAtOrAfter(
  const Goals & mission_goal_queue,
  size_t start_index,
  uint32_t & discrete_goal_index_z)
{
  for (size_t index_goal = start_index; index_goal < mission_goal_queue.size(); ++index_goal) {
    if (tryMissionPoseGoalIndexFromPoseZ(mission_goal_queue[index_goal], discrete_goal_index_z)) {
      return true;
    }
  }
  return false;
}

bool RemovePassedGoals::tryIndexedGoalZAtOrBefore(
  const Goals & mission_goal_queue,
  size_t end_index,
  uint32_t & discrete_goal_index_z)
{
  if (mission_goal_queue.empty()) {
    return false;
  }
  if (end_index >= mission_goal_queue.size()) {
    end_index = mission_goal_queue.size() - 1;
  }
  for (size_t index_goal = end_index + 1; index_goal-- > 0; ) {
    if (tryMissionPoseGoalIndexFromPoseZ(mission_goal_queue[index_goal], discrete_goal_index_z)) {
      return true;
    }
  }
  return false;
}

std::uint64_t RemovePassedGoals::fingerprintMixFromPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  uint32_t discrete_goal_index_z = 0;
  if (tryMissionPoseGoalIndexFromPoseZ(pose_stamped_goal, discrete_goal_index_z)) {
    return static_cast<std::uint64_t>(discrete_goal_index_z);
  }
  return 0xFFFFFFFFFFFFFFFFULL;
}

void RemovePassedGoals::handleReceivedTebGlobalPlan(const nav_msgs::msg::Path::SharedPtr plan_message)
{
  std::lock_guard<std::mutex> lock_guard_teb_plan(mutex_teb_plan_shared_);
  shared_ptr_latest_received_teb_plan_ = plan_message;
  if (plan_message) {
    shared_ptr_cached_copy_last_teb_plan_ = std::make_shared<nav_msgs::msg::Path>(*plan_message);
    const double polyline_len_m =
      tebGlobalPlanPolylineLengthMetersFromFirstPose(*plan_message);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *(node->get_clock()),
      5000,
      "[RemovePassedGoals] teb_global_plan rx: poses=%zu polyline_len_m=%.3f",
      plan_message->poses.size(), polyline_len_m);
  } else {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *(node->get_clock()), 5000, "[RemovePassedGoals] teb_global_plan rx: null message");
  }
  received_teb_plan_message_since_last_output_gpp_emit_ = true;
  if (node) {
    clock_timestamp_last_valid_teb_plan_message_ = node->get_clock()->now();
  }
}

void RemovePassedGoals::handleReceivedOdometry(const nav_msgs::msg::Odometry::SharedPtr msg_odometry)
{
  std::lock_guard<std::mutex> lock_guard_odometry(mutex_odometry_velocity_);
  latest_odometry_linear_velocity_x_mps_ = msg_odometry->twist.twist.linear.x;
  RCLCPP_INFO_THROTTLE(
    node->get_logger(),
    *(node->get_clock()),
    5000,
    "[RemovePassedGoals] odom rx: linear_x=%.4f m/s", latest_odometry_linear_velocity_x_mps_);
}

void RemovePassedGoals::resetStoredTebPlanBuffersKeepingRxTracking()
{
  RCLCPP_DEBUG(node->get_logger(), "[RemovePassedGoals] clear stored teb plan buffers (after gpp emit)");
  std::lock_guard<std::mutex> lock_guard_teb_plan(mutex_teb_plan_shared_);
  shared_ptr_latest_received_teb_plan_.reset();
  shared_ptr_cached_copy_last_teb_plan_.reset();
  clock_timestamp_last_valid_teb_plan_message_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void RemovePassedGoals::spinAllExclusiveSubscriptions()
{
  callback_executor_exclusive_.spin_some();
}

std::uint64_t RemovePassedGoals::fingerprintMissionQueueTailGoalStampOnly(const Goals & mission_goal_queue)
{
  if (mission_goal_queue.empty()) {
    return 0;
  }
  std::uint64_t fingerprint_mix = 0;
  const auto & stamp_terminal_goal_only = mission_goal_queue.back().header.stamp;
  fingerprint_mix ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(stamp_terminal_goal_only.sec)) *
    0x9e3779b97f4a7c15ULL;
  fingerprint_mix ^= static_cast<std::uint64_t>(stamp_terminal_goal_only.nanosec);
  return fingerprint_mix;
}

std::uint64_t RemovePassedGoals::fingerprintLastEmittedGppWindow(
  const Goals & filtered_gpp_goal_window,
  std::uint64_t mission_tail_goal_stamp_fingerprint) const
{
  std::uint64_t fingerprint_mix = mission_tail_goal_stamp_fingerprint;
  fingerprint_mix ^= (static_cast<std::uint64_t>(batch_span_goal_index_z_begin_) << 32) |
    static_cast<std::uint64_t>(batch_span_goal_index_z_end_);
  if (!filtered_gpp_goal_window.empty()) {
    fingerprint_mix ^= fingerprintMixFromPoseZ(filtered_gpp_goal_window.front());
    fingerprint_mix ^= fingerprintMixFromPoseZ(filtered_gpp_goal_window.back()) << 20;
    // Count and per-goal z mix in: endpoints alone are insufficient (middle removals keep same front/back z).
    fingerprint_mix ^= static_cast<std::uint64_t>(filtered_gpp_goal_window.size()) << 48;
    std::uint64_t xor_all_goal_index_z = 0;
    for (const auto & pose_stamped : filtered_gpp_goal_window) {
      xor_all_goal_index_z ^= fingerprintMixFromPoseZ(pose_stamped) * 0x9e3779b97f4a7c15ULL;
    }
    fingerprint_mix ^= xor_all_goal_index_z;
  }
  return fingerprint_mix;
}

void RemovePassedGoals::buildFirstWindowFromMission(const Goals & mission_goal_queue)
{
  buildGppWindowFromMissionIndex(mission_goal_queue, 0);
}

void RemovePassedGoals::buildGppWindowFromMissionIndex(
  const Goals & mission_goal_queue,
  size_t mission_segment_start_index)
{
  if (mission_goal_queue.empty()) {
    return;
  }
  if (mission_segment_start_index >= mission_goal_queue.size()) {
    mission_segment_start_index = mission_goal_queue.size() - 1;
  }

  auto assign_degenerate_span_at = [&](size_t index_goal) {
    uint32_t discrete_goal_index_z = 0;
    if (tryIndexedGoalZAtOrAfter(mission_goal_queue, index_goal, discrete_goal_index_z) ||
      tryIndexedGoalZAtOrBefore(mission_goal_queue, index_goal, discrete_goal_index_z))
    {
      batch_span_goal_index_z_begin_ = discrete_goal_index_z;
      batch_span_goal_index_z_end_ = discrete_goal_index_z;
    } else {
      batch_span_goal_index_z_begin_ = 0;
      batch_span_goal_index_z_end_ = 0;
    }
  };

  if (mission_goal_queue.size() == 1) {
    assign_degenerate_span_at(0);
    return;
  }
  if (mission_segment_start_index == mission_goal_queue.size() - 1) {
    assign_degenerate_span_at(mission_segment_start_index);
    return;
  }
  using namespace nav2_util::geometry_utils;  // NOLINT
  double window_max_polyline_length_meters = 0.0;
  getInput("max_gpp_segment_m", window_max_polyline_length_meters);

  uint32_t discrete_goal_index_z_begin = 0;
  if (!tryIndexedGoalZAtOrAfter(
      mission_goal_queue, mission_segment_start_index, discrete_goal_index_z_begin))
  {
    assign_degenerate_span_at(mission_segment_start_index);
    return;
  }
  batch_span_goal_index_z_begin_ = discrete_goal_index_z_begin;

  auto assign_span_end_from_queue_index = [&](size_t index_goal_end) {
    uint32_t discrete_goal_index_z_end = batch_span_goal_index_z_begin_;
    if (!tryIndexedGoalZAtOrBefore(mission_goal_queue, index_goal_end, discrete_goal_index_z_end) &&
      !tryIndexedGoalZAtOrAfter(mission_goal_queue, index_goal_end, discrete_goal_index_z_end))
    {
      discrete_goal_index_z_end = batch_span_goal_index_z_begin_;
    }
    if (discrete_goal_index_z_end < batch_span_goal_index_z_begin_) {
      discrete_goal_index_z_end = batch_span_goal_index_z_begin_;
    }
    batch_span_goal_index_z_end_ = discrete_goal_index_z_end;
  };

  double accumulated_segment_length_meters = 0.0;
  for (size_t idx_segment_end = mission_segment_start_index + 1; idx_segment_end < mission_goal_queue.size();
    ++idx_segment_end)
  {
    accumulated_segment_length_meters += euclidean_distance(
      mission_goal_queue[idx_segment_end - 1].pose,
      mission_goal_queue[idx_segment_end].pose);
    if (accumulated_segment_length_meters > window_max_polyline_length_meters) {
      assign_span_end_from_queue_index(idx_segment_end);
      return;
    }
  }
  assign_span_end_from_queue_index(mission_goal_queue.size() - 1);
}

void RemovePassedGoals::advanceGppWindowAfterShortTebPolyline(const Goals & mission_goal_queue)
{
  buildGppWindowFromMissionIndex(mission_goal_queue, 0);
}

RemovePassedGoals::Goals RemovePassedGoals::filterMissionGoalsByBatchZSpan(const Goals & mission_goal_queue) const
{
  Goals filtered_window_goal_poses;
  if (mission_goal_queue.empty()) {
    return filtered_window_goal_poses;
  }

  size_t index_first_indexed_in_span = mission_goal_queue.size();
  size_t index_last_indexed_in_span = 0;
  bool found_indexed_goal_in_span = false;
  bool found_any_indexed_goal = false;
  for (size_t index_goal = 0; index_goal < mission_goal_queue.size(); ++index_goal) {
    uint32_t discrete_goal_index_z = 0;
    if (!tryMissionPoseGoalIndexFromPoseZ(mission_goal_queue[index_goal], discrete_goal_index_z)) {
      continue;
    }
    found_any_indexed_goal = true;
    if (discrete_goal_index_z >= batch_span_goal_index_z_begin_ &&
      discrete_goal_index_z <= batch_span_goal_index_z_end_)
    {
      if (!found_indexed_goal_in_span) {
        index_first_indexed_in_span = index_goal;
        found_indexed_goal_in_span = true;
      }
      index_last_indexed_in_span = index_goal;
    }
  }

  if (!found_indexed_goal_in_span) {
    if (!found_any_indexed_goal) {
      return mission_goal_queue;
    }
    return filtered_window_goal_poses;
  }

  size_t index_window_start = index_first_indexed_in_span;
  while (index_window_start > 0 &&
    isUnindexedSentinelPoseZ(mission_goal_queue[index_window_start - 1]))
  {
    --index_window_start;
  }
  for (size_t index_goal = index_window_start; index_goal <= index_last_indexed_in_span; ++index_goal) {
    filtered_window_goal_poses.push_back(mission_goal_queue[index_goal]);
  }
  for (size_t index_goal = index_last_indexed_in_span + 1; index_goal < mission_goal_queue.size();
    ++index_goal)
  {
    if (!isUnindexedSentinelPoseZ(mission_goal_queue[index_goal])) {
      break;
    }
    filtered_window_goal_poses.push_back(mission_goal_queue[index_goal]);
  }
  return filtered_window_goal_poses;
}

double RemovePassedGoals::tebGlobalPlanPolylineLengthMetersFromFirstPose(const nav_msgs::msg::Path & teb_plan_path)
{
  if (teb_plan_path.poses.size() < 2) {
    return 0.0;
  }
  return nav2_util::geometry_utils::calculate_path_length(teb_plan_path, 0);
}

bool RemovePassedGoals::posesApproxEqualForGppMissionMatch(
  const geometry_msgs::msg::Pose & pose_a,
  const geometry_msgs::msg::Pose & pose_b,
  const double match_xy_m,
  const double match_yaw_rad)
{
  const double dx = pose_a.position.x - pose_b.position.x;
  const double dy = pose_a.position.y - pose_b.position.y;
  if (std::hypot(dx, dy) > match_xy_m) {
    return false;
  }
  if (match_yaw_rad < 0.0) {
    return true;
  }
  double yaw_a = tf2::getYaw(pose_a.orientation);
  double yaw_b = tf2::getYaw(pose_b.orientation);
  double dyaw = yaw_a - yaw_b;
  while (dyaw > M_PI) {
    dyaw -= 2.0 * M_PI;
  }
  while (dyaw < -M_PI) {
    dyaw += 2.0 * M_PI;
  }
  return std::fabs(dyaw) <= match_yaw_rad;
}

RemovePassedGoals::Goals RemovePassedGoals::pruneOutputGppGoalsByMissionPoseMatch(
  const Goals & output_gpp_candidates,
  const Goals & mission_goals,
  const double match_xy_m,
  const double match_yaw_rad) const
{
  Goals pruned;
  pruned.reserve(output_gpp_candidates.size());
  // Inner scan assumes mission_goals ordered by non-decreasing discrete index (pose.position.z).
  // Unindexed sentinel z < 0 is matched only against other unindexed poses (XY/yaw), never as uint32.
  for (const auto & pose_stamped_gpp : output_gpp_candidates) {
    uint32_t discrete_goal_index_z_gpp = 0;
    const bool gpp_goal_is_unindexed =
      !tryMissionPoseGoalIndexFromPoseZ(pose_stamped_gpp, discrete_goal_index_z_gpp);
    for (const auto & pose_stamped_mission : mission_goals) {
      uint32_t discrete_goal_index_z_mission = 0;
      const bool mission_goal_is_unindexed =
        !tryMissionPoseGoalIndexFromPoseZ(pose_stamped_mission, discrete_goal_index_z_mission);
      if (gpp_goal_is_unindexed) {
        if (!mission_goal_is_unindexed) {
          continue;
        }
        if (posesApproxEqualForGppMissionMatch(
            pose_stamped_gpp.pose, pose_stamped_mission.pose, match_xy_m, match_yaw_rad))
        {
          pruned.push_back(pose_stamped_mission);
          break;
        }
        continue;
      }
      if (mission_goal_is_unindexed) {
        continue;
      }
      if (discrete_goal_index_z_mission < discrete_goal_index_z_gpp) {
        continue;
      }
      if (discrete_goal_index_z_mission > discrete_goal_index_z_gpp) {
        break;
      }
      if (posesApproxEqualForGppMissionMatch(
          pose_stamped_gpp.pose, pose_stamped_mission.pose, match_xy_m, match_yaw_rad))
      {
        pruned.push_back(pose_stamped_mission);
      }
      break;
    }
  }
  return pruned;
}

void RemovePassedGoals::applyMonotonicGppGoalStampsToWindow(
  Goals & gpp_goals,
  const rclcpp::Time & clock_now,
  const bool assign_fresh_stamp_on_advance)
{
  if (gpp_goals.empty()) {
    return;
  }

  for (auto & pose_stamped_gpp : gpp_goals) {
    uint32_t goal_index_z = 0;
    const bool goal_is_unindexed = !tryMissionPoseGoalIndexFromPoseZ(pose_stamped_gpp, goal_index_z);
    rclcpp::Time stamp_candidate = assign_fresh_stamp_on_advance ?
      clock_now : rclcpp::Time(pose_stamped_gpp.header.stamp);

    if (!goal_is_unindexed) {
      const auto cached_stamp_it = emitted_gpp_goal_stamp_by_mission_index_z_.find(goal_index_z);
      if (cached_stamp_it != emitted_gpp_goal_stamp_by_mission_index_z_.end() &&
        stamp_candidate < cached_stamp_it->second)
      {
        stamp_candidate = cached_stamp_it->second;
      }
      emitted_gpp_goal_stamp_by_mission_index_z_[goal_index_z] = stamp_candidate;
    }

    pose_stamped_gpp.header.stamp = stamp_candidate;
  }

  if (last_emitted_gpp_front_stamp_.nanoseconds() > 0 &&
    rclcpp::Time(gpp_goals.front().header.stamp) < last_emitted_gpp_front_stamp_)
  {
    gpp_goals.front().header.stamp = last_emitted_gpp_front_stamp_;
    uint32_t front_goal_index_z = 0;
    if (tryMissionPoseGoalIndexFromPoseZ(gpp_goals.front(), front_goal_index_z)) {
      emitted_gpp_goal_stamp_by_mission_index_z_[front_goal_index_z] = last_emitted_gpp_front_stamp_;
    }
  }

  last_emitted_gpp_front_stamp_ = rclcpp::Time(gpp_goals.front().header.stamp);
}

BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  spinAllExclusiveSubscriptions();

  Goals incoming_goal_queue_from_blackboard;
  getInput("input_goals", incoming_goal_queue_from_blackboard);
  if (incoming_goal_queue_from_blackboard.empty()) {
    RCLCPP_INFO(node->get_logger(), "[RemovePassedGoals] tick: input_goals empty, reset & SUCCESS");
    mission_goal_queue_internal_.clear();
    fingerprint_accepted_blackboard_input_goals_tail_stamp_ = 0;
    fingerprint_internal_mission_queue_tail_stamp_ = 0;
    batch_window_initialized_ = false;
    has_emitted_output_gpp_goals_once_ = false;
    fingerprint_last_emitted_gpp_goal_window_ = 0;
    received_teb_plan_message_since_last_output_gpp_emit_ = false;
    clock_timestamp_last_emitted_output_gpp_goals_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    terminal_goal_dispatched_to_gpp_window_ = false;
    resetStoredTebPlanBuffersKeepingRxTracking();
    output_gpp_goals_snapshot_.clear();
    emitted_gpp_goal_stamp_by_mission_index_z_.clear();
    last_emitted_gpp_front_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    setOutput("output_gpp_goals", Goals{});
    setOutput("output_goals", Goals{});
    if (has_published_enable_backward_ && last_published_enable_backward_) {
      publishEnableBackwardIfChanged(false);
    }
    return BT::NodeStatus::SUCCESS;
  }

  const std::uint64_t fingerprint_incoming_blackboard_tail_stamp =
    fingerprintMissionQueueTailGoalStampOnly(incoming_goal_queue_from_blackboard);
  const bool board_mission_changed =
    (fingerprint_incoming_blackboard_tail_stamp != fingerprint_accepted_blackboard_input_goals_tail_stamp_);

  if (board_mission_changed) {
    RCLCPP_INFO(
      node->get_logger(),
      "[RemovePassedGoals] mission identity changed (tail_stamp_fp=%lx): reload queue size=%zu",
      static_cast<unsigned long>(fingerprint_incoming_blackboard_tail_stamp),
      incoming_goal_queue_from_blackboard.size());
    fingerprint_accepted_blackboard_input_goals_tail_stamp_ = fingerprint_incoming_blackboard_tail_stamp;
    mission_goal_queue_internal_ = incoming_goal_queue_from_blackboard;
    vector_passed_goal_indexes_recorded_.clear();
    batch_window_initialized_ = false;
    fingerprint_internal_mission_queue_tail_stamp_ = 0;
    previous_goal_queue_front_stamp_seconds_ = 0.0;
    has_emitted_output_gpp_goals_once_ = false;
    fingerprint_last_emitted_gpp_goal_window_ = 0;
    received_teb_plan_message_since_last_output_gpp_emit_ = false;
    clock_timestamp_last_emitted_output_gpp_goals_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    terminal_goal_dispatched_to_gpp_window_ = false;
    resetStoredTebPlanBuffersKeepingRxTracking();
    output_gpp_goals_snapshot_.clear();
    emitted_gpp_goal_stamp_by_mission_index_z_.clear();
    last_emitted_gpp_front_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool enable_backward_mode = false;
  getInput("enable_backward_mode", enable_backward_mode);
  publishEnableBackwardIfChanged(enable_backward_mode);

  Goals & mission_goal_queue_mutable_ref = mission_goal_queue_internal_;

  if (mission_goal_queue_mutable_ref.size() > 1) {
    const double stamp_seconds_goal_queue_front =
      mission_goal_queue_mutable_ref.front().header.stamp.sec +
      mission_goal_queue_mutable_ref.front().header.stamp.nanosec / 1e9;
    if (previous_goal_queue_front_stamp_seconds_ == 0.0) {
      previous_goal_queue_front_stamp_seconds_ = stamp_seconds_goal_queue_front;
    } else if (stamp_seconds_goal_queue_front - previous_goal_queue_front_stamp_seconds_ > 1e-1) {
      previous_goal_queue_front_stamp_seconds_ = stamp_seconds_goal_queue_front;
      vector_passed_goal_indexes_recorded_.clear();
    }
  }

  callback_executor_exclusive_.spin_some();

  using namespace nav2_util::geometry_utils;  // NOLINT
  geometry_msgs::msg::PoseStamped pose_robot_base_link_in_global_frame;
  if (!nav2_util::getCurrentPose(
      pose_robot_base_link_in_global_frame, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *(node->get_clock()),
      2000,
      "[RemovePassedGoals] getCurrentPose failed frame %s -> %s",
      global_frame_.c_str(), robot_base_frame_.c_str());
    setOutput("output_goals", mission_goal_queue_mutable_ref);
    return BT::NodeStatus::SUCCESS;
  }

  double minimum_odom_linear_x_abs_for_pass_checks_mps = -1.0;
  getInput("pass_check_min_odom_linear_x_mps", minimum_odom_linear_x_abs_for_pass_checks_mps);
  double linear_velocity_x_odometry_mps = 0.0;
  {
    std::lock_guard<std::mutex> lock_odometry_velocity(mutex_odometry_velocity_);
    linear_velocity_x_odometry_mps = latest_odometry_linear_velocity_x_mps_;
  }

  const bool odometry_velocity_gate_blocks_pass_checks =
    minimum_odom_linear_x_abs_for_pass_checks_mps >= 0.0 &&
    std::fabs(linear_velocity_x_odometry_mps) < minimum_odom_linear_x_abs_for_pass_checks_mps;

  if (odometry_velocity_gate_blocks_pass_checks) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[RemovePassedGoals] pass strip skipped: |odom_vx|=%.4f < min %.4f m/s",
      linear_velocity_x_odometry_mps, minimum_odom_linear_x_abs_for_pass_checks_mps);
  }

  if (has_emitted_output_gpp_goals_once_ && !odometry_velocity_gate_blocks_pass_checks) {
    const uint32_t count_passed_indexes_before_tick = vector_passed_goal_indexes_recorded_.size();
    // pass_check_max_search_dist_m (BT input) is unused: a prior whole-mission arc cap blocked valid head
    // stripping; full-dist mode relies on viapoint radius, behind-x, and optional yaw only.

    double pass_behind_max_x_m = 0.0;
    getInput("pass_behind_tol_m", pass_behind_max_x_m);

    bool pass_enable_yaw_check = true;
    getInput("pass_enable_yaw_check", pass_enable_yaw_check);

    double pass_yaw_threshold_rad = 1.57;
    getInput("pass_yaw_threshold_rad", pass_yaw_threshold_rad);
    if (pass_yaw_threshold_rad < 0.0) {
      pass_yaw_threshold_rad = 0.0;
    }

    const double robot_yaw_rad = tf2::getYaw(pose_robot_base_link_in_global_frame.pose.orientation);
    const tf2::Duration tf_tolerance_duration = tf2::durationFromSec(transform_tolerance_);

    double pass_tail_relax_remaining_path_max_m = 0.0;
    getInput("pass_tail_relax_remaining_path_max_m", pass_tail_relax_remaining_path_max_m);

    double pass_tail_relax_radius_m = 2.0;
    getInput("pass_tail_relax_radius_m", pass_tail_relax_radius_m);
    if (pass_tail_relax_radius_m <= 0.0) {
      pass_tail_relax_radius_m = viapoint_achieved_radius_;
    }

    double passed_goal_distance_threshold_m = -1.0;
    getInput("passed_goal_distance_threshold", passed_goal_distance_threshold_m);

    auto should_record_passed_goal_index_distance = [&](double distance_robot_to_goal_pose_meters) -> bool {
      return passed_goal_distance_threshold_m <= 0.0 ||
             distance_robot_to_goal_pose_meters < passed_goal_distance_threshold_m;
    };

    const size_t mission_goal_count = mission_goal_queue_mutable_ref.size();
    std::vector<double> cumulative_arc_distance_from_queue_head_meters(mission_goal_count, 0.0);
    for (size_t index_goal = 1; index_goal < mission_goal_count; ++index_goal) {
      cumulative_arc_distance_from_queue_head_meters[index_goal] =
        cumulative_arc_distance_from_queue_head_meters[index_goal - 1] +
        euclidean_distance(
        mission_goal_queue_mutable_ref[index_goal - 1].pose,
        mission_goal_queue_mutable_ref[index_goal].pose);
    }

    double remaining_polyline_length_head_to_terminal_goal_meters = 0.0;
    if (mission_goal_count >= 2) {
      remaining_polyline_length_head_to_terminal_goal_meters =
        cumulative_arc_distance_from_queue_head_meters.back();
    }

    const bool tail_segment_short_use_distance_only_checks =
      pass_tail_relax_remaining_path_max_m > 0.0 &&
      remaining_polyline_length_head_to_terminal_goal_meters <= pass_tail_relax_remaining_path_max_m;

    RCLCPP_DEBUG(
      node->get_logger(),
      "[RemovePassedGoals] pass strip: goals=%zu remaining_path_m=%.3f tail_relax_max=%.3f "
      "mode=%s radius_tail=%.3f viapoint_radius=%.3f",
      mission_goal_count,
      remaining_polyline_length_head_to_terminal_goal_meters,
      pass_tail_relax_remaining_path_max_m,
      tail_segment_short_use_distance_only_checks ? "distance_only_tail" : "full_dist_behind_yaw",
      pass_tail_relax_radius_m,
      viapoint_achieved_radius_);

    if (tail_segment_short_use_distance_only_checks) {
      for (size_t index_goal_candidate = 0; index_goal_candidate < mission_goal_count; ++index_goal_candidate) {
        const geometry_msgs::msg::PoseStamped & pose_goal_candidate =
          mission_goal_queue_mutable_ref[index_goal_candidate];
        const double distance_robot_to_goal_meters =
          euclidean_distance(pose_goal_candidate.pose, pose_robot_base_link_in_global_frame.pose);
        if (distance_robot_to_goal_meters > pass_tail_relax_radius_m) {
          RCLCPP_DEBUG(
            node->get_logger(),
            "[RemovePassedGoals] pass strip: goal %zu distance_m=%.3f > radius_m=%.3f, skip",
            index_goal_candidate,
            distance_robot_to_goal_meters,
            pass_tail_relax_radius_m);
          continue;
        }
        uint32_t discrete_goal_index_passed_z = 0;
        const bool has_mission_index_to_record =
          tryMissionPoseGoalIndexFromPoseZ(pose_goal_candidate, discrete_goal_index_passed_z);
        if (index_goal_candidate > 0) {
          mission_goal_queue_mutable_ref.erase(
            mission_goal_queue_mutable_ref.begin(),
            mission_goal_queue_mutable_ref.begin() + index_goal_candidate + 1);
        } else {
          mission_goal_queue_mutable_ref.erase(mission_goal_queue_mutable_ref.begin());
        }
        if (has_mission_index_to_record &&
          std::find(
            vector_passed_goal_indexes_recorded_.begin(),
            vector_passed_goal_indexes_recorded_.end(),
            discrete_goal_index_passed_z) == vector_passed_goal_indexes_recorded_.end() &&
          should_record_passed_goal_index_distance(distance_robot_to_goal_meters))
        {
          RCLCPP_INFO(
            node->get_logger(),
            "[RemovePassedGoals] In tail segment, add to passed_pose_indexes index: %u",
            static_cast<unsigned int>(discrete_goal_index_passed_z));
          vector_passed_goal_indexes_recorded_.push_back(discrete_goal_index_passed_z);
        } else if (!has_mission_index_to_record) {
          RCLCPP_DEBUG(
            node->get_logger(),
            "[RemovePassedGoals] In tail segment, strip unindexed z=-1 goal without recording passed index");
        }
        break;
      }
    } else {
      while (!mission_goal_queue_mutable_ref.empty()) {
        const geometry_msgs::msg::PoseStamped & pose_goal_front = mission_goal_queue_mutable_ref.front();
        const double distance_robot_to_front_goal_meters =
          euclidean_distance(pose_goal_front.pose, pose_robot_base_link_in_global_frame.pose);
        if (distance_robot_to_front_goal_meters > viapoint_achieved_radius_) {
          break;
        }

        geometry_msgs::msg::PoseStamped pose_goal_front_in_robot_base_frame;
        try {
          if (pose_goal_front.header.frame_id == robot_base_frame_) {
            pose_goal_front_in_robot_base_frame = pose_goal_front;
            pose_goal_front_in_robot_base_frame.header.frame_id = robot_base_frame_;
          } else {
            const geometry_msgs::msg::TransformStamped transform_global_to_robot_base =
              tf_->lookupTransform(
              robot_base_frame_, pose_goal_front.header.frame_id, tf2::TimePointZero,
              tf_tolerance_duration);
            tf2::doTransform(pose_goal_front, pose_goal_front_in_robot_base_frame, transform_global_to_robot_base);
          }
        } catch (const tf2::TransformException & exception_tf) {
          break;
        }

        if (pose_goal_front_in_robot_base_frame.pose.position.x > pass_behind_max_x_m) {
          break;
        }

        if (pass_enable_yaw_check) {
          double yaw_diff_goal_minus_robot_rad =
            tf2::getYaw(pose_goal_front.pose.orientation) - robot_yaw_rad;
          while (yaw_diff_goal_minus_robot_rad > M_PI) {
            yaw_diff_goal_minus_robot_rad -= 2.0 * M_PI;
          }
          while (yaw_diff_goal_minus_robot_rad < -M_PI) {
            yaw_diff_goal_minus_robot_rad += 2.0 * M_PI;
          }
          if (std::fabs(yaw_diff_goal_minus_robot_rad) > pass_yaw_threshold_rad) {
            // Yaw check enabled but alignment failed: do not strip front goal, do not record passed index,
            // and do not fall through to erase/push_back below (same iteration guarantees break here).
            RCLCPP_DEBUG(
              node->get_logger(),
              "[RemovePassedGoals] yaw over threshold |dyaw|=%.4f > %.4f rad: skip strip & passed_pose_indexes "
              "for front goal z=%.3f",
              std::fabs(yaw_diff_goal_minus_robot_rad),
              pass_yaw_threshold_rad,
              pose_goal_front.pose.position.z);
            break;
          }
        }

        uint32_t discrete_goal_index_passed_z = 0;
        const bool has_mission_index_to_record =
          tryMissionPoseGoalIndexFromPoseZ(pose_goal_front, discrete_goal_index_passed_z);
        mission_goal_queue_mutable_ref.erase(mission_goal_queue_mutable_ref.begin());
        if (has_mission_index_to_record &&
          std::find(
            vector_passed_goal_indexes_recorded_.begin(),
            vector_passed_goal_indexes_recorded_.end(),
            discrete_goal_index_passed_z) == vector_passed_goal_indexes_recorded_.end() &&
          should_record_passed_goal_index_distance(distance_robot_to_front_goal_meters))
        {
          RCLCPP_INFO(
            node->get_logger(),
            "[RemovePassedGoals] In full dist behind yaw, add to passed_pose_indexes index: %u", static_cast<unsigned int>(discrete_goal_index_passed_z));
          vector_passed_goal_indexes_recorded_.push_back(discrete_goal_index_passed_z);
        } else if (!has_mission_index_to_record) {
          RCLCPP_DEBUG(
            node->get_logger(),
            "[RemovePassedGoals] In full dist behind yaw, strip unindexed z=-1 goal without recording passed index");
        }
      }
    }

    if (vector_passed_goal_indexes_recorded_.size() > count_passed_indexes_before_tick) {
      capella_ros_msg::msg::PassedPosesIndex message_passed_indexes;
      message_passed_indexes.indexes = vector_passed_goal_indexes_recorded_;
      publisher_passed_pose_indexes_->publish(message_passed_indexes);
      RCLCPP_DEBUG(
        node->get_logger(),
        "[RemovePassedGoals] published passed_pose_indexes count=%zu",
        static_cast<size_t>(vector_passed_goal_indexes_recorded_.size()));
    }
  } else if (!has_emitted_output_gpp_goals_once_) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[RemovePassedGoals] pass strip skipped: waiting for first output_gpp_goals emit");
  }

  if (!mission_goal_queue_mutable_ref.empty()) {
    nav_msgs::msg::Path message_removed_plan_visualization;
    message_removed_plan_visualization.header.frame_id = "map";
    message_removed_plan_visualization.header.stamp =
      mission_goal_queue_mutable_ref.front().header.stamp;
    message_removed_plan_visualization.poses =
      std::vector<geometry_msgs::msg::PoseStamped>(
      mission_goal_queue_mutable_ref.begin(),
      mission_goal_queue_mutable_ref.end());
    for (auto & pose_stamped_goal : message_removed_plan_visualization.poses) {
      pose_stamped_goal.pose.position.z = 0.0;
    }
    publisher_removed_plan_debug_->publish(message_removed_plan_visualization);
  }

  const std::uint64_t queue_tail_stamp_fp =
    fingerprintMissionQueueTailGoalStampOnly(mission_goal_queue_mutable_ref);
  const bool queue_tail_fp_changed =
    (queue_tail_stamp_fp != fingerprint_internal_mission_queue_tail_stamp_);

  if (queue_tail_fp_changed) {
    fingerprint_internal_mission_queue_tail_stamp_ = queue_tail_stamp_fp;
    batch_window_initialized_ = false;
    RCLCPP_DEBUG(
      node->get_logger(),
      "[RemovePassedGoals] internal queue tail stamp fp changed -> batch reset fp=%lx",
      static_cast<unsigned long>(queue_tail_stamp_fp));
  }

  double replan_trigger_arc_length_meters = 5.0;
  getInput("replan_trigger_arc_m", replan_trigger_arc_length_meters);

  double teb_message_timeout_seconds = 10.0;
  getInput("teb_message_timeout_s", teb_message_timeout_seconds);
  if (teb_message_timeout_seconds < 0.05) {
    teb_message_timeout_seconds = 0.05;
  }

  const rclcpp::Time clock_now_robot_time = node->get_clock()->now();

  nav_msgs::msg::Path::SharedPtr shared_ptr_latest_teb_snapshot;
  rclcpp::Time clock_timestamp_last_teb_message_snapshot;
  nav_msgs::msg::Path::SharedPtr shared_ptr_cached_teb_snapshot;
  {
    std::lock_guard<std::mutex> lock_guard_teb_plan(mutex_teb_plan_shared_);
    shared_ptr_latest_teb_snapshot = shared_ptr_latest_received_teb_plan_;
    clock_timestamp_last_teb_message_snapshot = clock_timestamp_last_valid_teb_plan_message_;
    shared_ptr_cached_teb_snapshot = shared_ptr_cached_copy_last_teb_plan_;
  }

  const bool has_valid_timestamp_last_teb_plan_message =
    clock_timestamp_last_teb_message_snapshot.nanoseconds() > 0;

  const double seconds_since_last_teb_plan_message =
    has_valid_timestamp_last_teb_plan_message
    ? (clock_now_robot_time - clock_timestamp_last_teb_message_snapshot).seconds()
    : std::numeric_limits<double>::infinity();

  const bool teb_plan_message_within_timeout_window =
    has_valid_timestamp_last_teb_plan_message &&
    seconds_since_last_teb_plan_message <= teb_message_timeout_seconds;

  const nav_msgs::msg::Path * pointer_effective_teb_plan_for_batch_logic = nullptr;
  if (teb_plan_message_within_timeout_window && shared_ptr_latest_teb_snapshot &&
    shared_ptr_latest_teb_snapshot->poses.size() >= 2)
  {
    pointer_effective_teb_plan_for_batch_logic = shared_ptr_latest_teb_snapshot.get();
  } else if (shared_ptr_cached_teb_snapshot && shared_ptr_cached_teb_snapshot->poses.size() >= 2) {
    pointer_effective_teb_plan_for_batch_logic = shared_ptr_cached_teb_snapshot.get();
  }

  constexpr double epsilon_minimum_nonzero_polyline_length_meters = 1e-6;
  double total_polyline_length_effective_teb_meters = 0.0;
  if (pointer_effective_teb_plan_for_batch_logic) {
    total_polyline_length_effective_teb_meters =
      nav2_util::geometry_utils::calculate_path_length(*pointer_effective_teb_plan_for_batch_logic);
  }

  const nav_msgs::msg::Path * pointer_usable_non_degenerate_teb_plan = nullptr;
  if (pointer_effective_teb_plan_for_batch_logic &&
    total_polyline_length_effective_teb_meters > epsilon_minimum_nonzero_polyline_length_meters)
  {
    pointer_usable_non_degenerate_teb_plan = pointer_effective_teb_plan_for_batch_logic;
  }

  RCLCPP_DEBUG(
    node->get_logger(),
    "[RemovePassedGoals] teb timing: age_since_rx_sec=%.6f within_timeout=%s replan_arc_m=%.3f timeout_s=%.3f "
    "rx_since_last_gpp_emit=%s effective_teb_polyline_m=%.6f usable_non_degenerate=%s",
    seconds_since_last_teb_plan_message,
    teb_plan_message_within_timeout_window ? "yes" : "no",
    replan_trigger_arc_length_meters,
    teb_message_timeout_seconds,
    received_teb_plan_message_since_last_output_gpp_emit_ ? "yes" : "no",
    total_polyline_length_effective_teb_meters,
    pointer_usable_non_degenerate_teb_plan ? "yes" : "no");

  bool advanced_gpp_window_after_short_teb_this_tick = false;
  if (!batch_window_initialized_) {
    buildFirstWindowFromMission(mission_goal_queue_mutable_ref);
    batch_window_initialized_ = true;
    RCLCPP_INFO(
      node->get_logger(),
      "[RemovePassedGoals] init gpp batch z=[%u,%u] mission_goals=%zu",
      batch_span_goal_index_z_begin_,
      batch_span_goal_index_z_end_,
      mission_goal_queue_mutable_ref.size());
  } else if (
    pointer_usable_non_degenerate_teb_plan &&
    pointer_usable_non_degenerate_teb_plan->poses.size() >= 2)
  {
    const double teb_polyline_length_from_first_pose_meters =
      tebGlobalPlanPolylineLengthMetersFromFirstPose(*pointer_usable_non_degenerate_teb_plan);
    if (teb_polyline_length_from_first_pose_meters <= replan_trigger_arc_length_meters) {
      advanceGppWindowAfterShortTebPolyline(mission_goal_queue_mutable_ref);
      advanced_gpp_window_after_short_teb_this_tick = true;
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *(node->get_clock()),
        5000,
        "[RemovePassedGoals] advance gpp window: teb_polyline_m=%.3f <= %.3f -> z=[%u,%u]",
        teb_polyline_length_from_first_pose_meters,
        replan_trigger_arc_length_meters,
        batch_span_goal_index_z_begin_,
        batch_span_goal_index_z_end_);
    }
  }

  Goals filtered_gpp_goal_window_output =
    filterMissionGoalsByBatchZSpan(mission_goal_queue_mutable_ref);
  if (filtered_gpp_goal_window_output.empty() && !mission_goal_queue_mutable_ref.empty()) {
    buildFirstWindowFromMission(mission_goal_queue_mutable_ref);
    filtered_gpp_goal_window_output =
      filterMissionGoalsByBatchZSpan(mission_goal_queue_mutable_ref);
  }

  double gpp_goal_pose_match_xy_m = 0.2;
  getInput("gpp_goal_pose_match_xy_m", gpp_goal_pose_match_xy_m);
  if (gpp_goal_pose_match_xy_m < 1e-6) {
    gpp_goal_pose_match_xy_m = 1e-6;
  }
  double gpp_goal_pose_match_yaw_rad = -1.0;
  getInput("gpp_goal_pose_match_yaw_rad", gpp_goal_pose_match_yaw_rad);

  Goals filtered_gpp_goal_window_after_prune =
    pruneOutputGppGoalsByMissionPoseMatch(
    filtered_gpp_goal_window_output,
    mission_goal_queue_mutable_ref,
    gpp_goal_pose_match_xy_m,
    gpp_goal_pose_match_yaw_rad);

  const std::uint64_t gpp_window_fp_now =
    fingerprintLastEmittedGppWindow(filtered_gpp_goal_window_after_prune, queue_tail_stamp_fp);

  // Keep per-goal stamps monotonic: prune copies mission poses whose header.stamp can be older than
  // the last emitted value. On advance (before terminal freeze), assign a fresh stamp for replan.
  const bool assign_fresh_gpp_stamp_on_advance =
    advanced_gpp_window_after_short_teb_this_tick && !terminal_goal_dispatched_to_gpp_window_;
  applyMonotonicGppGoalStampsToWindow(
    filtered_gpp_goal_window_after_prune,
    clock_now_robot_time,
    assign_fresh_gpp_stamp_on_advance);
  if (assign_fresh_gpp_stamp_on_advance) {
    RCLCPP_INFO(
      node->get_logger(),
      "[RemovePassedGoals] refresh gpp goal stamp count=%zu",
      filtered_gpp_goal_window_after_prune.size());
  }

  if (!filtered_gpp_goal_window_after_prune.empty()) {
    // Detect whether the terminal mission goal has entered the GPP window.
    if (!terminal_goal_dispatched_to_gpp_window_ && !mission_goal_queue_mutable_ref.empty()) {
      const auto & pose_stamped_terminal = mission_goal_queue_mutable_ref.back();
      uint32_t z_terminal = 0;
      const bool terminal_is_unindexed =
        !tryMissionPoseGoalIndexFromPoseZ(pose_stamped_terminal, z_terminal);
      bool terminal_in_gpp_window = false;
      if (terminal_is_unindexed) {
        for (const auto & pose_stamped_gpp : filtered_gpp_goal_window_after_prune) {
          if (posesApproxEqualForGppMissionMatch(
              pose_stamped_gpp.pose, pose_stamped_terminal.pose,
              gpp_goal_pose_match_xy_m, gpp_goal_pose_match_yaw_rad))
          {
            terminal_in_gpp_window = true;
            break;
          }
        }
      } else {
        terminal_in_gpp_window = batch_span_goal_index_z_end_ >= z_terminal;
        if (!terminal_in_gpp_window) {
          for (const auto & pose_stamped_gpp : filtered_gpp_goal_window_after_prune) {
            uint32_t z_gpp = 0;
            if (tryMissionPoseGoalIndexFromPoseZ(pose_stamped_gpp, z_gpp) && z_gpp == z_terminal) {
              terminal_in_gpp_window = true;
              break;
            }
          }
        }
      }
      if (terminal_in_gpp_window) {
        terminal_goal_dispatched_to_gpp_window_ = true;
        if (terminal_is_unindexed) {
          RCLCPP_INFO(
            node->get_logger(),
            "[RemovePassedGoals] terminal unindexed goal (z=-1) entered gpp window: stamp freeze active");
        } else {
          RCLCPP_INFO(
            node->get_logger(),
            "[RemovePassedGoals] terminal goal z=%u entered gpp window: stamp freeze active",
            static_cast<unsigned int>(z_terminal));
        }
      }
    }



    setOutput("output_gpp_goals", filtered_gpp_goal_window_after_prune);
    output_gpp_goals_snapshot_ = filtered_gpp_goal_window_after_prune;
    fingerprint_last_emitted_gpp_goal_window_ = gpp_window_fp_now;
    has_emitted_output_gpp_goals_once_ = true;
    clock_timestamp_last_emitted_output_gpp_goals_ = clock_now_robot_time;

    if (advanced_gpp_window_after_short_teb_this_tick) {
      resetStoredTebPlanBuffersKeepingRxTracking();
      received_teb_plan_message_since_last_output_gpp_emit_ = false;
    }

    RCLCPP_DEBUG(
      node->get_logger(),
      "[RemovePassedGoals] emitted output_gpp_goals count=%zu window_fp=%lx advance=%s",
      filtered_gpp_goal_window_after_prune.size(),
      static_cast<unsigned long>(gpp_window_fp_now),
      advanced_gpp_window_after_short_teb_this_tick ? "yes" : "no");
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "[RemovePassedGoals] prune removed all goals (was %zu), keep previous gpp_goals snapshot count=%zu",
      filtered_gpp_goal_window_output.size(),
      output_gpp_goals_snapshot_.size());
  }

  if (!mission_goal_queue_mutable_ref.empty()) {
    setOutput("output_goals", mission_goal_queue_mutable_ref);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
