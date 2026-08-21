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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "capella_ros_msg/msg/passed_poses_index.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Maintains mission queue, optional passed-point stripping, GPP windows keyed by pose z-index,
 * and output_gpp_goals emission gated by mission identity (terminal stamp), TEB polyline length,
 * and a post-emit wait for teb_global_plan within teb_message_timeout_s.
 */
class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Mission goals from navigate action (read each tick)"),
      BT::OutputPort<Goals>("output_goals",
        "Goals after passed-point stripping; mirrors current mission queue for downstream nodes"),
      BT::OutputPort<Goals>("output_gpp_goals",
        "Windowed goals for GPP. Emission as before; between emits, goals still in the mission queue are "
        "kept (pose match vs current internal goals); points removed from mission are dropped from this output."),
      BT::InputPort<double>("radius", 0.5, "radius to goal for viapoint pass checks when tail relax inactive"),
      BT::InputPort<double>("pass_check_max_search_dist_m", 8.0,
        "Search along mission arc from queue head up to this distance for pass judgement"),
      BT::InputPort<double>("pass_behind_tol_m", 0.0,
        "In robot base_link: goals with x greater than this are treated as ahead (not passed); "
        "only x <= pass_behind_tol_m are candidates when tail relax inactive"),
      BT::InputPort<bool>("pass_enable_yaw_check", true, "Enable yaw alignment check when tail relax inactive"),
      BT::InputPort<double>("pass_yaw_threshold_rad", 1.57, "Max |goal_yaw - robot_yaw| when yaw check enabled"),
      BT::InputPort<double>("pass_tail_relax_remaining_path_max_m", 20.0,
        "When remaining mission polyline length from queue head is GREATER than this (m), use radius + "
        "behind + yaw checks for stripping; when LESS OR EQUAL, only distance vs pass_tail_relax_radius_m"),
      BT::InputPort<double>("pass_tail_relax_radius_m", 2.0,
        "Distance threshold (m) for relaxed tail-only checks; if <=0 uses radius port"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>("accumulate_distance", 8.0, "accumulate distance to search removed poses"),
      BT::InputPort<std::string>("teb_topic", std::string("teb_global_plan"), "TEB global plan topic"),
      BT::InputPort<double>("max_gpp_segment_m", 20.0, "Max mission polyline length in one GPP window"),
      BT::InputPort<double>("replan_trigger_arc_m", 5.0,
        "TEB polyline length from first pose compared for advance / secondary emit when TEB is fresh"),
      BT::InputPort<double>("teb_message_timeout_s", 10.0,
        "Used twice: (1) After emit: block_secondary until first valid teb_global_plan or until this many seconds "
        "since emit elapse; (2) Secondary emit_on_teb_timeout when post-emit TEB was received but last valid "
        "teb_global_plan is older than this many seconds (stale stream)."),
      BT::InputPort<std::string>("odom_topic", std::string("/odom"), "Odometry topic for linear-x gate on pass logic"),
      BT::InputPort<double>("pass_check_min_odom_linear_x_mps", -1.0,
        "If >= 0: skip passed-point stripping when |odom.twist.linear.x| is below this (m/s); if < 0 disable gate"),
      BT::InputPort<double>("gpp_goal_pose_match_xy_m", 0.2,
        "Match output_gpp_goals points to mission goals by pose: max XY distance (m)"),
      BT::InputPort<double>("gpp_goal_pose_match_yaw_rad", -1.0,
        "Match by pose: max |delta yaw| (rad); if < 0 skip yaw check (XY only)"),
      BT::InputPort<double>("passed_goal_distance_threshold", -1.0,
        "When >0: only append indexes to passed_pose_indexes if robot distance to that goal (m) is strictly "
        "less than this; when <=0, no extra distance gate on recording"),
      BT::InputPort<bool>("enable_backward_mode", false,
        "true: backward driving; false: forward. Publishes /enable_backward on state change only"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  static uint32_t missionPoseGoalIndexFromPoseZ(const geometry_msgs::msg::PoseStamped & pose_stamped_goal);
  static bool isUnindexedSentinelPoseZ(const geometry_msgs::msg::PoseStamped & pose_stamped_goal);
  static bool tryMissionPoseGoalIndexFromPoseZ(
    const geometry_msgs::msg::PoseStamped & pose_stamped_goal, uint32_t & discrete_goal_index_z);
  static bool tryIndexedGoalZAtOrAfter(
    const Goals & mission_goal_queue, size_t start_index, uint32_t & discrete_goal_index_z);
  static bool tryIndexedGoalZAtOrBefore(
    const Goals & mission_goal_queue, size_t start_index, uint32_t & discrete_goal_index_z);
  static std::uint64_t fingerprintMixFromPoseZ(
    const geometry_msgs::msg::PoseStamped & pose_stamped_goal);
  static std::uint64_t fingerprintMissionQueueTailGoalStampOnly(const Goals & mission_goal_queue);
  void buildFirstWindowFromMission(const Goals & mission_goal_queue);
  void buildGppWindowFromMissionIndex(const Goals & mission_goal_queue, size_t mission_segment_start_index);
  void advanceGppWindowAfterShortTebPolyline(const Goals & mission_goal_queue);
  Goals filterMissionGoalsByBatchZSpan(const Goals & mission_goal_queue) const;
  static double tebGlobalPlanPolylineLengthMetersFromFirstPose(const nav_msgs::msg::Path & teb_plan_path);

  void handleReceivedTebGlobalPlan(const nav_msgs::msg::Path::SharedPtr plan_message);
  void handleReceivedOdometry(const nav_msgs::msg::Odometry::SharedPtr msg_odometry);
  void resetStoredTebPlanBuffersKeepingRxTracking();
  void spinAllExclusiveSubscriptions();

  std::uint64_t fingerprintLastEmittedGppWindow(
    const Goals & filtered_gpp_goal_window,
    std::uint64_t mission_tail_goal_stamp_fingerprint) const;

  static bool posesApproxEqualForGppMissionMatch(
    const geometry_msgs::msg::Pose & pose_a,
    const geometry_msgs::msg::Pose & pose_b,
    double match_xy_m,
    double match_yaw_rad);
  Goals pruneOutputGppGoalsByMissionPoseMatch(
    const Goals & output_gpp_candidates,
    const Goals & mission_goals,
    double match_xy_m,
    double match_yaw_rad) const;
  void applyMonotonicGppGoalStampsToWindow(
    Goals & gpp_goals,
    const rclcpp::Time & clock_now,
    bool assign_fresh_stamp_on_advance);
  /** Publish /enable_backward only when desired state differs from last published. */
  void publishEnableBackwardIfChanged(bool desired_backward);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_exclusive_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_exclusive_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_removed_plan_debug_;
  rclcpp::Publisher<capella_ros_msg::msg::PassedPosesIndex>::SharedPtr publisher_passed_pose_indexes_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_enable_backward_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_teb_global_plan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odometry_;

  nav_msgs::msg::Path::SharedPtr shared_ptr_latest_received_teb_plan_;
  nav_msgs::msg::Path::SharedPtr shared_ptr_cached_copy_last_teb_plan_;
  rclcpp::Time clock_timestamp_last_valid_teb_plan_message_{0, 0, RCL_ROS_TIME};
  std::mutex mutex_teb_plan_shared_;

  double latest_odometry_linear_velocity_x_mps_{0.0};
  std::mutex mutex_odometry_velocity_;

  double viapoint_achieved_radius_{};
  double accumulate_distance_{};
  std::string robot_base_frame_;
  std::string global_frame_;
  double transform_tolerance_{};
  std::vector<uint32_t> vector_passed_goal_indexes_recorded_;

  Goals mission_goal_queue_internal_;
  std::uint64_t fingerprint_accepted_blackboard_input_goals_tail_stamp_{0};
  std::uint64_t fingerprint_internal_mission_queue_tail_stamp_{0};
  bool batch_window_initialized_{false};
  uint32_t batch_span_goal_index_z_begin_{0};
  uint32_t batch_span_goal_index_z_end_{0};

  bool has_emitted_output_gpp_goals_once_{false};
  std::uint64_t fingerprint_last_emitted_gpp_goal_window_{0};
  bool terminal_goal_dispatched_to_gpp_window_{false};

  bool received_teb_plan_message_since_last_output_gpp_emit_{false};
  rclcpp::Time clock_timestamp_last_emitted_output_gpp_goals_{0, 0, RCL_ROS_TIME};

  /** Last values written to output_gpp_goals (used to drop points no longer present in mission goals). */
  Goals output_gpp_goals_snapshot_;
  /** Per mission z-index: last emitted stamp so prune/mission poses cannot regress timestamps. */
  std::unordered_map<uint32_t, rclcpp::Time> emitted_gpp_goal_stamp_by_mission_index_z_;
  rclcpp::Time last_emitted_gpp_front_stamp_{0, 0, RCL_ROS_TIME};

  double previous_goal_queue_front_stamp_seconds_{0.0};

  bool last_published_enable_backward_{false};
  bool has_published_enable_backward_{false};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
