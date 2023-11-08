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

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_localization_status_good.hpp"

namespace nav2_behavior_tree
{

IsLocalizationGoodCondition::IsLocalizationGoodCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  localization_score_topic_("/localization_score"),
  min_localization_score_(0.6),
  is_localization_score_good_(true)
{
  getInput("min_localization_score", min_localization_score_);
  getInput("localization_score_topic", localization_score_topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  localization_score_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    localization_score_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsLocalizationGoodCondition::localizationscoreCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsLocalizationGoodCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_localization_score_good_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In localization status check !");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsLocalizationGoodCondition::localizationscoreCallback(std_msgs::msg::Float32::SharedPtr msg)
{
    is_localization_score_good_ = msg->data > min_localization_score_;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsLocalizationGoodCondition>("IsLocalizationGood");
}
