// Copyright (c) 2021 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
#define  NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace nav2_behavior_tree
{
/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard (or on the bound input ports) and FAILURE otherwise.
 * Ports bind to PoseStamped list / PoseStamped; if a port is not wired in XML,
 * reads from legacy blackboard entries "goals" / "goal".
 */
class GloballyUpdatedGoalCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GloballyUpdatedGoalCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GloballyUpdatedGoalCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GloballyUpdatedGoalCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Goals list bound to blackboard entry, e.g. {goals} or {gpp_goals}"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Single pose goal bound to blackboard entry, e.g. {goal}"),
    };
  }

private:
  /** Read goals/goal via inputs if set; otherwise from blackboard keys "goals"/"goal". */
  void getGoalsPortsOrDefaults(
    std::vector<geometry_msgs::msg::PoseStamped> & goals_out,
    geometry_msgs::msg::PoseStamped & goal_out);

  bool first_time;
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}  // namespace nav2_behavior_tree


#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
