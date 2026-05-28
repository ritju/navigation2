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
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  std::string service_name = "/collision_detection_service";
  getInput("service_name", service_name);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  client_ = node_->create_client<std_srvs::srv::SetBool>(
    service_name,
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);
}

BT::NodeStatus IsCollisionCondition::tick()
{
  if (!client_->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Collision detection service is not ready");
    return BT::NodeStatus::SUCCESS;
  }

  bool expand_contour = false;
  getInput("expand_contour", expand_contour);

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = expand_contour;

  auto future = client_->async_send_request(request);
  if (callback_group_executor_.spin_until_future_complete(future, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Timed out waiting for collision detection service response");
    return BT::NodeStatus::SUCCESS;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_INFO(node_->get_logger(), "The robot collides!");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsCollisionCondition>("IsCollision");
}
