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
 * @brief Strip passed mission goals, then emit a z-index GPP window.
 *
 * Mission identity is the tail goal stamp. TEB polyline length drives window
 * advance. output_gpp_goals is gated by that window plus pose match vs the
 * current internal queue.
 */
class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief Construct the node, subscribe TEB/odom, and create debug publishers.
   */
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

  /** TEB plan callback: cache latest path and mark a receive since last GPP emit. */
  void onTebPlan(const nav_msgs::msg::Path::SharedPtr msg);
  /** Odometry callback: store linear.x for the pass-check speed gate. */
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  /** Drop cached TEB paths after a window advance (receive tracking is kept). */
  void clearTebBuffers();
  void spinSubs();

  /** Publish /enable_backward only when the desired value changes. */
  void publishEnableBackward(bool enable);

  /** Reset GPP window, stamps, and TEB buffers (queue / passed indexes unchanged). */
  void resetGppState();
  /** Empty input_goals: clear queue, outputs, and GPP state. */
  void resetOnEmptyInput();
  /** Reload internal queue when blackboard mission identity (tail stamp) changes. */
  bool reloadMissionIfChanged(const Goals & input_goals);

  /** Strip passed goals from the queue (tail-relax or radius/behind/yaw). */
  void stripPassedGoals(
    Goals & queue,
    const geometry_msgs::msg::PoseStamped & robot_pose);
  /** Record z-index into passed_pose_indexes when distance / uniqueness allow. */
  void maybeRecordPassed(
    uint32_t z,
    bool has_z,
    double dist,
    double dist_lim,
    const char * pass_kind);

  /** Publish remaining queue as removed_plan (z flattened for viz). */
  void publishDebugPlan(const Goals & queue);

  /** Fingerprint of last emitted GPP window (span z + contents). */
  std::uint64_t gppWindowFp(const Goals & window, std::uint64_t tail_fp) const;

  /** Build first GPP window from queue head. */
  void initWindow(const Goals & queue);
  /** Build GPP z-span starting at a queue index, capped by max_gpp_segment_m. */
  void buildWindowFrom(const Goals & queue, size_t start);
  /** Rebuild window from queue head after a short TEB polyline. */
  void advanceWindow(const Goals & queue);
  /** Keep goals whose z is in [z_begin_, z_end_], plus neighboring unindexed sentinels. */
  Goals filterByZSpan(const Goals & queue) const;
  /** Keep GPP candidates that still match a live mission pose (xy / optional yaw). */
  Goals pruneByMissionPose(
    const Goals & candidates,
    const Goals & mission,
    double match_xy,
    double match_yaw) const;
  /**
   * Keep per-goal stamps monotonic so prune cannot regress time.
   * On window advance (before terminal freeze), assign a fresh stamp for replan.
   */
  void applyMonotonicStamps(
    Goals & gpp_goals,
    const rclcpp::Time & now,
    bool fresh_on_advance);
  /** Latch terminal-in-window so later advances do not refresh stamps. */
  void markTerminalIfNeeded(
    const Goals & gpp_goals,
    const Goals & mission,
    double match_xy,
    double match_yaw);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor cb_executor_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_removed_plan_;
  rclcpp::Publisher<capella_ros_msg::msg::PassedPosesIndex>::SharedPtr pub_passed_indexes_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_backward_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_teb_plan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  nav_msgs::msg::Path::SharedPtr latest_teb_plan_;
  nav_msgs::msg::Path::SharedPtr cached_teb_plan_;
  rclcpp::Time last_teb_time_{0, 0, RCL_ROS_TIME};
  std::mutex teb_mutex_;

  double odom_vx_{0.0};
  std::mutex odom_mutex_;

  double radius_{};
  double accumulate_dist_{};
  std::string robot_base_frame_;
  std::string global_frame_;
  double transform_tolerance_{};
  std::vector<uint32_t> passed_indexes_;

  Goals mission_goals_;
  std::uint64_t board_tail_fp_{0};
  std::uint64_t queue_tail_fp_{0};
  bool window_inited_{false};
  uint32_t z_begin_{0};
  uint32_t z_end_{0};

  bool gpp_emitted_{false};
  std::uint64_t last_gpp_fp_{0};
  bool terminal_in_window_{false};

  bool teb_rx_since_emit_{false};
  rclcpp::Time last_gpp_emit_time_{0, 0, RCL_ROS_TIME};

  /** Last values written to output_gpp_goals (fallback if prune drops everything). */
  Goals last_gpp_goals_;
  /** Per mission z: last emitted stamp so prune/mission poses cannot go backwards in time. */
  std::unordered_map<uint32_t, rclcpp::Time> gpp_stamp_by_z_;
  rclcpp::Time last_gpp_front_stamp_{0, 0, RCL_ROS_TIME};

  double prev_front_stamp_s_{0.0};

  bool last_enable_backward_{false};
  bool published_enable_backward_{false};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
