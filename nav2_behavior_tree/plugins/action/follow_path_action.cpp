// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/mission_path_sync.hpp"

namespace nav2_behavior_tree
{

namespace
{

bool timeEqual(const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

}  // namespace

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

bool FollowPathAction::getCurrentInputGoalsStamp(builtin_interfaces::msg::Time & stamp_out) const
{
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  if (!getInput("input_goals", input_goals) || input_goals.empty()) {
    return false;
  }
  stamp_out = input_goals.front().header.stamp;
  return true;
}

bool FollowPathAction::tryApplyMissionPathUpdate()
{
  if (!isPathReadyForCurrentMission(config().blackboard)) {
    return false;
  }

  nav_msgs::msg::Path new_path;
  if (!getInput("path", new_path) || new_path.poses.empty()) {
    return false;
  }
  if (goal_.path == new_path) {
    return false;
  }
  goal_.path = new_path;
  return true;
}

void FollowPathAction::on_tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  if (getInput("input_goals", input_goals))
  {
    if (input_goals.empty())
    {
      throw BT::RuntimeError("FollowPathAction: input_goals is empty");
    }
  }
  else
  {
    RCLCPP_INFO(
      node_->get_logger(), "FollowPathAction: input_goals not provided, using path poses as goals");
  }

  getInput("path", goal_.path);
  if (goal_.path.poses.empty()) {
    throw BT::RuntimeError("FollowPathAction: path is empty");
  }
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);

  builtin_interfaces::msg::Time goals_stamp;
  if (getCurrentInputGoalsStamp(goals_stamp)) {
    mission_goals_stamp_at_send_ = goals_stamp;
    mission_goals_stamp_valid_ = true;
  }
}

bool FollowPathAction::on_tick_send_goal()
{
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  if (getInput("input_goals", input_goals) && input_goals.empty()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "FollowPathAction: waiting for non-empty input_goals after preempt");
    return false;
  }

  nav_msgs::msg::Path path_on_blackboard;
  getInput("path", path_on_blackboard);
  if (path_on_blackboard.poses.empty()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "FollowPathAction: waiting for replanned path on blackboard");
    return false;
  }

  if (!isPathReadyForCurrentMission(config().blackboard)) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "FollowPathAction: waiting for path synced to current mission generation");
    return false;
  }

  on_tick();
  return true;
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
{
  if (tryApplyMissionPathUpdate()) {
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }

  if (!mission_goals_stamp_valid_) {
    return;
  }

  builtin_interfaces::msg::Time current_goals_stamp;
  if (!getCurrentInputGoalsStamp(current_goals_stamp)) {
    return;
  }

  if (!timeEqual(current_goals_stamp, mission_goals_stamp_at_send_)) {
    if (tryApplyMissionPathUpdate()) {
      mission_goals_stamp_at_send_ = current_goals_stamp;
      goal_updated_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "FollowPathAction: mission preempted, updating path on controller without cancel");
    }
  }
}

BT::NodeStatus FollowPathAction::on_success()
{
  // Controller may have succeeded for the previous segment while a NavigateThroughPoses
  // preempt already bumped the mission generation and cleared the path. Do not bubble
  // SUCCESS up the tree until the planner has synced a path for the current mission.
  if (!isPathReadyForCurrentMission(config().blackboard)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPathAction: segment success but path not synced to current mission; waiting");
    requeueForNewGoalOnNextTick();
    return BT::NodeStatus::RUNNING;
  }

  nav_msgs::msg::Path path_on_blackboard;
  getInput("path", path_on_blackboard);
  if (path_on_blackboard.poses.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "FollowPathAction: controller succeeded but blackboard path is empty; failing");
    return BT::NodeStatus::FAILURE;
  }

  if (!mission_goals_stamp_valid_) {
    return BT::NodeStatus::SUCCESS;
  }

  builtin_interfaces::msg::Time current_goals_stamp;
  if (!getCurrentInputGoalsStamp(current_goals_stamp)) {
    return BT::NodeStatus::SUCCESS;
  }

  if (timeEqual(current_goals_stamp, mission_goals_stamp_at_send_)) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!tryApplyMissionPathUpdate()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPathAction: mission preempted after segment success, waiting for replanned path");
    requeueForNewGoalOnNextTick();
    return BT::NodeStatus::RUNNING;
  }

  mission_goals_stamp_at_send_ = current_goals_stamp;
  RCLCPP_INFO(
    node_->get_logger(),
    "FollowPathAction: mission preempted after segment success, continuing with new path");
  requeueForNewGoalOnNextTick();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPathAction::on_cancelled()
{
  if (!isPathReadyForCurrentMission(config().blackboard)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPathAction: cancelled but path not synced to current mission; waiting");
    requeueForNewGoalOnNextTick();
    return BT::NodeStatus::RUNNING;
  }

  if (mission_goals_stamp_valid_) {
    builtin_interfaces::msg::Time current_goals_stamp;
    if (getCurrentInputGoalsStamp(current_goals_stamp) &&
      !timeEqual(current_goals_stamp, mission_goals_stamp_at_send_))
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "FollowPathAction: cancelled after mission preempt; waiting for replanned path");
      requeueForNewGoalOnNextTick();
      return BT::NodeStatus::RUNNING;
    }
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "FollowPathAction: follow path cancelled; failing navigation");
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
