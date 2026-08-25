// Copyright (c) 2026 Capella
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

#include "nav2_behavior_tree/plugins/action/clear_path_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

ClearPath::ClearPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ClearPath::tick()
{
  nav_msgs::msg::Path path;
  getInput("input_path", path);
  const auto n_poses = path.poses.size();
  path.poses.clear();

  try {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    path.header.stamp = node->now();
    RCLCPP_INFO(
      node->get_logger(),
      "ClearPath: cleared %zu pose(s) from blackboard path", n_poses);
  } catch (...) {
  }

  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearPath>("ClearPath");
}
