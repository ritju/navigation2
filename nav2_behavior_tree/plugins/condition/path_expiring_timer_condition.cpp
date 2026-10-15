// Copyright (c) 2022 Joshua Wallace
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

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree
{

PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  // Empty path is invalid: treat as expired so Inverter(PathExpiringTimer)
  // fails and Fallback will ComputePathToPose again.
  if (path.poses.empty()) {
    prev_path_ = path;
    first_time_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  if (first_time_ || prev_path_ != path) {
    prev_path_ = path;
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  auto seconds = (node_->now() - start_).seconds();
  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = node_->now();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
