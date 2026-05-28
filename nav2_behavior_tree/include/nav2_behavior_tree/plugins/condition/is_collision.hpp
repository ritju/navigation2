// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that calls collision_detection_service and
 * returns SUCCESS when no collision is detected and FAILURE otherwise
 */
class IsCollisionCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsCollisionCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsCollisionCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCollisionCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("expand_contour", false, "Expand robot contour for collision check"),
      BT::InputPort<std::string>("service_name", "/collision_detection_service", "Collision detection service name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_
