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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WELT_MODEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WELT_MODEL_NODE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "capella_ros_msg/action/empty.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps capella_ros_msg::action::RelocalizationSpin
 */
class WeltModelNode : public BtActionNode<capella_ros_msg::action::Empty>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  WeltModelNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
  
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
      });
  }

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__RELOCALIZATION_SPIN_ACTION_HPP_
