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

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"
#include "nav2_behavior_tree/mission_path_sync.hpp"

namespace nav2_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::on_tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  std::vector<geometry_msgs::msg::PoseStamped> gpp_goals;

  const bool has_goals = static_cast<bool>(getInput("goals", goals));
  const bool has_gpp_goals = static_cast<bool>(getInput("gpp_goals", gpp_goals));

  if (has_goals && has_gpp_goals) {
    goal_.goals = (goals.size() <= gpp_goals.size()) ? goals : gpp_goals;
  } else if (has_goals) {
    goal_.goals = goals;
  } else if (has_gpp_goals) {
    goal_.goals = gpp_goals;
  }

  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  if (result_.result->path.poses.empty()) {
    clearPathMissionSync(config().blackboard);
    RCLCPP_ERROR(
      node_->get_logger(),
      "ComputePathThroughPosesAction: planner succeeded with empty path");
    return BT::NodeStatus::FAILURE;
  }
  markPathSyncedToCurrentMission(config().blackboard);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  clearPathMissionSync(config().blackboard);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  clearPathMissionSync(config().blackboard);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
