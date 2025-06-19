// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  check_distance_(0.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus IsPathValidCondition::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goal_poses; 
  nav_msgs::msg::Path path, input_port_path, prune_path;
  getInput("path", input_port_path);
  getInput("goals", goal_poses);
  getInput("check_distance", check_distance_);
  getInput("prune_path", prune_path);
  callback_group_executor_.spin_some();

  if (input_port_path.poses.empty() && goal_poses.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (!goal_poses.empty())
  {
    path.header.stamp = node_->get_clock()->now();
    path.header.frame_id = "map";
    path.poses = goal_poses;
    goto check_path;
  }

  RCLCPP_INFO(node_->get_logger(), "Prune_path path.poses length: %ld !", prune_path.poses.size());
  
  if (!prune_path.poses.empty())
  {
    path.header.stamp = node_->get_clock()->now();
    path.header.frame_id = "map";
    path.poses = prune_path.poses;
    goto check_path;
  }

  if (!input_port_path.poses.empty())
  {
    path.header.stamp = node_->get_clock()->now();
    path.header.frame_id = "map";
    path.poses = input_port_path.poses;
    goto check_path;
  }

  check_path:
  if (path.poses.size() > 0)
  {
    using namespace nav2_util::geometry_utils;  // NOLINT
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_, "map", "base_link", 0.5))
    {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(node_->get_logger(), "Before prune path.poses length: %ld !", path.poses.size());

    if (!pruneGlobalPlan(current_pose, path.poses, 1))
    {
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("output_prune_path", path);
    RCLCPP_DEBUG(node_->get_logger(), "After prune path.poses length: %ld !", path.poses.size());
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Empty plan !");
  }
  
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  // 截断检查路径
  nav_msgs::msg::Path check_path;
  check_path.header = path.header;
  check_path.poses.push_back(path.poses.front());

  if (check_distance_ > 0.0 && path.poses.size() > 1)
  {
    double accumuldate_distance = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
      accumuldate_distance += nav2_util::geometry_utils::euclidean_distance(path.poses[i - 1].pose.position, path.poses[i].pose.position);
      if (accumuldate_distance < check_distance_)
      {
        check_path.poses.push_back(path.poses[i]);
      }
      else break;
    }
  }
  request->path = check_path;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  path.poses.clear();
  setOutput("output_prune_path", path);
  RCLCPP_WARN(node_->get_logger(), "Path invalid !");
  return BT::NodeStatus::FAILURE;
}

bool IsPathValidCondition::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return false;
  
  try
  {
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
    // int count_it = 0, count_high_precision = 0;
    while (it != global_plan.end())
    {
      double dx = global_pose.pose.position.x - it->pose.position.x;
      double dy = global_pose.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  catch (const std::runtime_error& ex)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
