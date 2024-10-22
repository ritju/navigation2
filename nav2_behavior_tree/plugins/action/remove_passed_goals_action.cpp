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
  look_ahead_distance_(3.0),
  checked_path_received_(false),
  receive_new_goal_(true)
  // occupied_path_received_(false)
{
  getInput("radius", viapoint_achieved_radius_);
  getInput("look_ahead_distance", look_ahead_distance_);  
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("local_costmap_topic", local_costmap_topic_);
  getInput("look_ahead_distance", look_ahead_distance_);
  getInput("footprint_topic", footprint_topic_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  // qos.best_effort().durability_volatile();
  removed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("removed_plan", qos);
  passed_poses_index_pub_ = node->create_publisher<capella_ros_msg::msg::PassedPosesIndex>("passed_pose_indexes", qos);

  checked_path_sub_ = node->create_subscription<nav_msgs::msg::Path>("checked_path", qos, 
                      std::bind(&RemovePassedGoals::removed_path_callback, this, std::placeholders::_1), sub_option);
  
  // occupied_path_sub_ = node->create_subscription<nav_msgs::msg::Path>("occupied_path", qos, 
  //                     std::bind(&RemovePassedGoals::occupied_path_callback, this, std::placeholders::_1), sub_option);
  
  receive_new_sub_ = node->create_subscription<std_msgs::msg::Bool>("receive_new_goal", 1, 
                      std::bind(&RemovePassedGoals::receive_new_goal_callback, this, std::placeholders::_1), sub_option);

}

inline BT::NodeStatus RemovePassedGoals::tick()
{

  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  getInput("input_goals", goal_poses);
  
  RCLCPP_INFO(node->get_logger(), "First goal_poses.size: %ld , receive_new_goal_: %d!", goal_poses.size(), receive_new_goal_);
  // if (!receive_new_goal_ && checked_path_received_ && removed_path_.poses.size() > 0 && !occupied_path_received_)
  if (!receive_new_goal_ && checked_path_received_ && removed_path_.poses.size() > 0)
  {
    if (goal_poses.size() > 0)
    {
      std::vector<geometry_msgs::msg::PoseStamped> partial_goals;
      partial_goals = goal_poses.size() > 10 ? (std::vector(goal_poses.begin(), goal_poses.begin() + 10)) : 
                    (std::vector(goal_poses.begin(), goal_poses.end()));
      std::vector<geometry_msgs::msg::PoseStamped>::iterator it;
      for (it = removed_path_.poses.begin(); it != removed_path_.poses.begin() + 10 && it != removed_path_.poses.end();)
      {
        if (std::find_if(partial_goals.begin(), partial_goals.end(), 
          [=](const geometry_msgs::msg::PoseStamped& pose)
          {return ((pose.pose.position.x == it->pose.position.x) && (pose.pose.position.y == it->pose.position.y));}) == 
          partial_goals.end())
        {
          it = removed_path_.poses.erase(it);
        }
        else
        {
          ++it;
        }
      }
      if (removed_path_.poses.size() > 0)
      {
        goal_poses = removed_path_.poses;
      }
    }
    removed_path_.poses.clear();
  }
  // if (!receive_new_goal_ && occupied_path_.poses.size() > 0 && goal_poses.size() > 0)
  // {
  //   RCLCPP_INFO(node->get_logger(), "Insert occupied pose: %ld !", occupied_path_.poses.size());
  //   for (auto &pose : occupied_path_.poses)
  //   {
  //     pose.header.stamp = goal_poses.begin()->header.stamp;
  //   }
  //   occupied_path_.header.stamp = goal_poses.begin()->header.stamp;
  //   goal_poses.insert(goal_poses.end(), occupied_path_.poses.begin(), occupied_path_.poses.end());
  //   occupied_path_.poses.clear();
  //   RCLCPP_INFO(node->get_logger(), "Goal_poses after occupied, size(): %ld !", goal_poses.size());
  // }
  
  checked_path_received_ = false;
  receive_new_goal_ = false;
  callback_group_executor_.spin_some();
  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }
  
  // 对路径点pose重新赋值，适配lattice planner
  if (goal_poses.size() > 1)
  {
    for (size_t i = 0; i < goal_poses.size(); ++i)
    {
      if (i == 0)
      {
        double theta = std::atan2(goal_poses[1].pose.position.y - goal_poses[0].pose.position.y,
                                  goal_poses[1].pose.position.x - goal_poses[0].pose.position.x);
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, theta);
        goal_poses[0].pose.orientation.w = orientation.w();
        goal_poses[0].pose.orientation.z = orientation.z();
        goal_poses[0].pose.orientation.y = orientation.y();
        goal_poses[0].pose.orientation.x = orientation.x();
      }
      else
      {
        double theta = std::atan2(goal_poses[i].pose.position.y - goal_poses[i-1].pose.position.y,
                                  goal_poses[i].pose.position.x - goal_poses[i-1].pose.position.x);
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, theta);
        goal_poses[i].pose.orientation.w = orientation.w();
        goal_poses[i].pose.orientation.z = orientation.z();
        goal_poses[i].pose.orientation.y = orientation.y();
        goal_poses[i].pose.orientation.x = orientation.x();
      }
    }
  }
  
  // 对路径点pose重新赋值，适配lattice planner
  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  // goal_poses_ = goal_poses;


  double dist_to_goal;
  uint32_t indexes_size = passed_poses_indexes_.size();
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);
    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }
    passed_poses_indexes_.emplace_back(goal_poses[0].pose.position.z);
    goal_poses.erase(goal_poses.begin());
  }
  if (goal_poses.size() == 1)
  {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);
    if (dist_to_goal < 1.0 && passed_poses_indexes_.back() != goal_poses[0].pose.position.z)
    {
      passed_poses_indexes_.emplace_back(goal_poses[0].pose.position.z);
    }
  }
  if (passed_poses_indexes_.size() > indexes_size)
  {
    capella_ros_msg::msg::PassedPosesIndex msg;
    msg.indexes = passed_poses_indexes_;
    passed_poses_index_pub_->publish(msg);
  }
      
    

  RCLCPP_INFO(node->get_logger(), "Goal_poses end,size(): %ld !", goal_poses.size());
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = node->get_clock()->now();
  path_msg.poses = std::vector(goal_poses.begin(), goal_poses.end());
  removed_path_pub_->publish(path_msg);
  // occupied_path_received_ = false;

  // RCLCPP_INFO(node->get_logger(), "Set path, path lenth: %ld !", goal_poses.size());
  setOutput("output_goals", goal_poses);
  return BT::NodeStatus::SUCCESS;
}

void RemovePassedGoals::removed_path_callback(const nav_msgs::msg::Path &msg)
{
  checked_path_received_ = true;
  removed_path_ = msg;
}

// void RemovePassedGoals::occupied_path_callback(const nav_msgs::msg::Path &msg)
// {

//   occupied_path_ = msg;
// }

void RemovePassedGoals::receive_new_goal_callback(const std_msgs::msg::Bool &msg)
{
  receive_new_goal_ = msg.data;
  removed_path_.poses.clear();
  // occupied_path_received_ = true;
  passed_poses_indexes_.clear();
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
