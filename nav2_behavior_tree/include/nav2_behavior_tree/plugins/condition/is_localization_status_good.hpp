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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_LOCALIZATION_STATUS_GOOD_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_LOCALIZATION_STATUS_GOOD_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to localizaiton_score topic and
 * returns SUCCESS when localizaiton_score is high and FAILURE otherwise
 */
class IsLocalizationGoodCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsLocalizationGoodCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsLocalizationGoodCondition() = delete;

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
      BT::InputPort<float>("min_localization_score", "Minimum localization score in percent"),
      BT::InputPort<std::string>(
        "localization_score_topic", std::string("/localization_score"), "localization_score_topic"),
    };
  }

private:
  /**
   * @brief Callback function for localization_score topic
   * @param msg Shared pointer to std_msgs::msg::Float32 message
   */
  void localizationscoreCallback(std_msgs::msg::Float32::SharedPtr msg);
  void chargePositionCallback(std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_score_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_position_sub_;
  std::string localization_score_topic_;
  float min_localization_score_;
  bool is_localization_score_good_;
  bool charge_position_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_LOCALIZATION_STATUS_GOOD_
