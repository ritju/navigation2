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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/path_check_action.hpp"

namespace nav2_behavior_tree
{

namespace
{

std::uint64_t fingerprintMissionQueueTailGoalStampOnly(
  const std::vector<geometry_msgs::msg::PoseStamped> & mission_goal_queue)
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

}  // namespace

  PathCheckAction::PathCheckAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<capella_ros_msg::action::PathCheck>(xml_tag_name, action_name, conf),
  check_distance_(8.0)
{

}

void PathCheckAction::on_tick()
{
  getInput("check_distance", check_distance_);
  getInput("input_goals", input_goals_);
  input_goals_mission_fingerprint_ = fingerprintMissionQueueTailGoalStampOnly(input_goals_);
  goal_.distance = check_distance_;
  goal_.input_goals = input_goals_;
}

BT::NodeStatus PathCheckAction::on_success()
{
  if (result_.result->output_goals.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PathCheckAction: output_goals is empty! Do not update output_goals!");
    return BT::NodeStatus::SUCCESS;
  }

  Goals current_input_goals;
  getInput("input_goals", current_input_goals);
  const std::uint64_t current_mission_fingerprint =
    fingerprintMissionQueueTailGoalStampOnly(current_input_goals);
  if (current_mission_fingerprint != input_goals_mission_fingerprint_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PathCheckAction: goals mission changed since action start (fp %lx -> %lx), "
      "skip output_goals writeback to avoid preempt overwrite",
      static_cast<unsigned long>(input_goals_mission_fingerprint_),
      static_cast<unsigned long>(current_mission_fingerprint));
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("output_goals", result_.result->output_goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::PathCheckAction>(name, "path_check", config);
    };

  factory.registerBuilder<nav2_behavior_tree::PathCheckAction>("PathCheck", builder);
}
