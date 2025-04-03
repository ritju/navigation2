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
  receive_new_goal_(false),
  count(0)
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
}

inline BT::NodeStatus RemovePassedGoals::tick()
{

  setStatus(BT::NodeStatus::RUNNING);
  Goals goal_poses;
  getInput("input_goals", goal_poses);
  
  if (goal_poses.size() > 0)
  {
    if (last_initialize_time_ == 0)
    {
      last_initialize_time_ = goal_poses.front().header.stamp.sec + goal_poses.front().header.stamp.nanosec / 1e9;
    }
    else if (goal_poses.front().header.stamp.sec + goal_poses.front().header.stamp.nanosec / 1e9 - last_initialize_time_ > 1e-1)
    {  
      last_initialize_time_ = goal_poses.front().header.stamp.sec + goal_poses.front().header.stamp.nanosec / 1e9;
      receive_new_goal_ = true;
      passed_poses_indexes_.clear();
    }
  }
  
  if (!receive_new_goal_ && checked_path_received_ && (goal_poses.front().header.stamp == removed_path_.header.stamp))
  {
    if (goal_poses.size() > 0 && removed_path_.poses.size() > 0)
    {
      std::vector<geometry_msgs::msg::PoseStamped>::iterator it;
      for (it = removed_path_.poses.begin(); it != removed_path_.poses.begin() + 10 && it != removed_path_.poses.end();)
      {
        if (std::find_if(goal_poses.begin(), goal_poses.end(), 
          [=](const geometry_msgs::msg::PoseStamped& pose)
          {return ((pose.pose.position.z == it->pose.position.z));}) == 
          goal_poses.end())
        {
          it = removed_path_.poses.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }
    goal_poses = removed_path_.poses;
  }
  receive_new_goal_ = false;
  removed_path_.poses.clear();
  checked_path_received_ = false;
  
  callback_group_executor_.spin_some();
  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }
  
  // 对路径点pose重新赋值，适配lattice planner
  // if (goal_poses.size() > 1)
  // {
  //   for (size_t i = 0; i < goal_poses.size(); ++i)
  //   {
  //     if (i == 0)
  //     {
  //       double theta = std::atan2(goal_poses[1].pose.position.y - goal_poses[0].pose.position.y,
  //                                 goal_poses[1].pose.position.x - goal_poses[0].pose.position.x);
  //       tf2::Quaternion orientation;
  //       orientation.setRPY(0, 0, theta);
  //       goal_poses[0].pose.orientation.w = orientation.w();
  //       goal_poses[0].pose.orientation.z = orientation.z();
  //       goal_poses[0].pose.orientation.y = orientation.y();
  //       goal_poses[0].pose.orientation.x = orientation.x();
  //     }
  //     else
  //     {
  //       double theta = std::atan2(goal_poses[i].pose.position.y - goal_poses[i-1].pose.position.y,
  //                                 goal_poses[i].pose.position.x - goal_poses[i-1].pose.position.x);
  //       tf2::Quaternion orientation;
  //       orientation.setRPY(0, 0, theta);
  //       goal_poses[i].pose.orientation.w = orientation.w();
  //       goal_poses[i].pose.orientation.z = orientation.z();
  //       goal_poses[i].pose.orientation.y = orientation.y();
  //       goal_poses[i].pose.orientation.x = orientation.x();
  //     }
  //   }
  // }
  
  // 对路径点pose重新赋值，适配lattice planner
  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  double dist_to_goal;
  uint32_t indexes_size = passed_poses_indexes_.size();
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);
    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }
    // if (passed_poses_indexes_.size() > 0 && passed_poses_indexes_.back() != static_cast<uint32_t>(goal_poses[0].pose.position.z))
    if (passed_poses_indexes_.size() > 0 && 
        std::find_if(passed_poses_indexes_.begin(), passed_poses_indexes_.end(), 
          [=](const uint32_t& index)
          {return (index == static_cast<uint32_t>(goal_poses[0].pose.position.z));}) == 
          passed_poses_indexes_.end())
    {
      passed_poses_indexes_.emplace_back(static_cast<uint32_t>(goal_poses[0].pose.position.z));
    }
    else if (passed_poses_indexes_.size() == 0)
    {
      passed_poses_indexes_.emplace_back(static_cast<uint32_t>(goal_poses[0].pose.position.z));
    }
    goal_poses.erase(goal_poses.begin());
  }
  if (goal_poses.size() == 1)
  {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);
    if (dist_to_goal < 1.0 && 
        std::find_if(passed_poses_indexes_.begin(), passed_poses_indexes_.end(), 
          [=](const uint32_t& index)
          {return (index == static_cast<uint32_t>(goal_poses[0].pose.position.z));}) == 
          passed_poses_indexes_.end())
    {
      passed_poses_indexes_.emplace_back(static_cast<uint32_t>(goal_poses[0].pose.position.z));
    }
  }
  if (passed_poses_indexes_.size() > indexes_size)
  {
    capella_ros_msg::msg::PassedPosesIndex msg;
    msg.indexes = passed_poses_indexes_;
    passed_poses_index_pub_->publish(msg);
  }
      
  if (goal_poses.size() > 0)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    // path_msg.header.stamp = node->get_clock()->now();
    path_msg.header.stamp = goal_poses.front().header.stamp;
    path_msg.poses = std::vector(goal_poses.begin(), goal_poses.end());
    removed_path_pub_->publish(path_msg);
  }
  setOutput("output_goals", goal_poses);
  return BT::NodeStatus::SUCCESS;
}

void RemovePassedGoals::removed_path_callback(const nav_msgs::msg::Path &msg)
{
  checked_path_received_ = true;
  removed_path_ = msg;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
