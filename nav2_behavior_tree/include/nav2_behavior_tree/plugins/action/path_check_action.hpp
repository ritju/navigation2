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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CHECK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CHECK_ACTION_HPP_

#include <cstdint>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "capella_ros_msg/action/path_check.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps capella_ros_msg::action::RelocalizationSpin
 */
class PathCheckAction : public BtActionNode<capella_ros_msg::action::PathCheck>
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;
  /**
   * @brief A constructor for nav2_behavior_tree::PathCheckAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  PathCheckAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<float>("check_distance", 8.0, "Total distance for path check !"),
        BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
        BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      });
  }
  double check_distance_;
  Goals input_goals_;
  std::uint64_t input_goals_mission_fingerprint_{0};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CHECK_ACTION_HPP_
