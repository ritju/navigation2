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

#include "nav2_behavior_tree/plugins/condition/is_collision.hpp"

namespace nav2_behavior_tree
{

IsCollisionCondition::IsCollisionCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_collision_topic_("/is_collision"),
  is_collsion_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  is_collision_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    is_collision_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&IsCollisionCondition::iscollisionCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsCollisionCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_collsion_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot collides!");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void IsCollisionCondition::iscollisionCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    is_collsion_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsCollisionCondition>("IsCollision");
}
