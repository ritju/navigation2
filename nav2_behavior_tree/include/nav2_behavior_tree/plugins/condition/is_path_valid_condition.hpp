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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class IsPathValidCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsPathValidCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPathValidCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPathValidCondition() = delete;

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
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<nav_msgs::msg::Path>("prune_path", "Prune path based of robot pose"),
      BT::OutputPort<nav_msgs::msg::Path>("output_prune_path", "Output prune path based of robot pose"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "Goals"),
      BT::InputPort<double>("check_distance", "Total check distance !"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot);
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  nav_msgs::msg::Path received_path_;
  // The timeout value while waiting for a responce from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
  double check_distance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
