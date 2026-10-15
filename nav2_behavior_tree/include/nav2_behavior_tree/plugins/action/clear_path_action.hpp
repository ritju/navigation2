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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_PATH_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Clear poses from a blackboard Path so PathExpiringTimer / FollowPath
 *        will not reuse a stale plan after recovery.
 */
class ClearPath : public BT::SyncActionNode
{
public:
  ClearPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to clear"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Empty path written back"),
    };
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_PATH_ACTION_HPP_
