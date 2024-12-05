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

#include "nav2_behavior_tree/plugins/condition/is_pedestrian_in_front.hpp"

namespace nav2_behavior_tree
{

IsPedestrianFrontCondition::IsPedestrianFrontCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  person_detected_topic_("/person_detected"),
  pedestrain_distance_(2.0),
  is_pedestrain_in_front_(false)
{
  getInput("pedestrain_distance", pedestrain_distance_);
  getInput("person_detected_topic", person_detected_topic_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  person_detected_sub_ = node_->create_subscription<capella_ros_msg::msg::DetectResult>(
    person_detected_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsPedestrianFrontCondition::person_detected_callback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsPedestrianFrontCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_pedestrain_in_front_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is pedestrain in front !");
    is_pedestrain_in_front_ = false;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void IsPedestrianFrontCondition::person_detected_callback(capella_ros_msg::msg::DetectResult::SharedPtr msg)
{

    if (msg->result.size() > 0)
    {
        for (auto data : msg->result)
        {
                if (fabs(data.x) < 20 && fabs(data.y) < 20)
                {
                        geometry_msgs::msg::PoseStamped pedestrian_pose_in_local, pedestrian_pose_in_global;
                        pedestrian_pose_in_local.header.frame_id = robot_base_frame_;
                        pedestrian_pose_in_local.header.stamp = node_->get_clock()->now();
                        pedestrian_pose_in_local.pose.position.x = data.x;
                        pedestrian_pose_in_local.pose.position.y = data.y;
                        if (!nav2_util::transformPoseInTargetFrame(pedestrian_pose_in_local, 
                            pedestrian_pose_in_global, *tf_, global_frame_))
                        {
                          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can not transform pedestrian pose in local to global !");
                          break;
                        }
                        double last_to_latest_distance = sqrt(pow(last_pedestrian_pose_.pose.position.x - pedestrian_pose_in_global.pose.position.x, 2) + 
                                                              pow(last_pedestrian_pose_.pose.position.y - pedestrian_pose_in_global.pose.position.y, 2));

                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "last_to_latest_distance %f !", last_to_latest_distance);                         
                        if (last_to_latest_distance > 0.5)
                        {
                                is_pedestrain_in_front_ = (fabs(data.x) < pedestrain_distance_ && fabs(data.y) < pedestrain_distance_);
                                last_pedestrian_pose_ = pedestrian_pose_in_global;
                                return;
                        }                        
                }
        }
    }
    is_pedestrain_in_front_ = false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPedestrianFrontCondition>("IsPedestrianFront");
}
