// Copyright (c) 2021 Samsung Research America
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
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5),
  back_poses_(),
  command_subscribe_(nullptr)
{
  getInput("radius", viapoint_achieved_radius_);

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  std::string back_goals_topic;
  node->get_parameter_or<std::string>("back_goals_topic", back_goals_topic, "back_goals");

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  command_subscribe_ = node->create_subscription<nav_msgs::msg::Path>(
    back_goals_topic, rclcpp::QoS(1),
    std::bind(&RemovePassedGoals::command_callback, this, std::placeholders::_1),
    sub_option);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  getInput("input_goals", goal_poses);
  callback_group_executor_.spin_some();

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    goal_poses.erase(goal_poses.begin());
  }

  std::vector<geometry_msgs::msg::PoseStamped> temp; 
  geometry_msgs::msg::PoseStamped back_goal;
  for (const auto & back_pose : back_poses_.poses) {  
    for (auto it = goal_poses.begin(); it != goal_poses.end(); /* Increment in loop */) {  
      if (std::fabs(it->pose.position.x - back_pose.pose.position.x) < 0.01 &&  
          std::fabs(it->pose.position.y - back_pose.pose.position.y) < 0.01) {  
        temp.push_back(*it);
        goal_poses.erase(it);  
        break;
      } else {  
        ++it;  
      }  
    }  
  } 
  goal_poses.insert(goal_poses.end(), temp.begin(), temp.end());  
  

  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
