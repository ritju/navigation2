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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp"


namespace nav2_behavior_tree
{

GoalUpdatedController::GoalUpdatedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus GoalUpdatedController::tick()
{
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "--------------------------- GoalUpdatedController ticked ---------------------------");
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "goal_updated_controller status(): %d ", (int)status());
  if (status() == BT::NodeStatus::IDLE) {
    // Reset since we're starting a new iteration of
    // the goal updated controller (moving from IDLE to RUNNING)

    // config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    // config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);

    goal_was_updated_ = false;
  }

  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "goal_   => x: %f, y: %f: ", goal_.pose.position.x, goal_.pose.position.y);
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "current => x: %f, y: %f: ", current_goal.pose.position.x, current_goal.pose.position.y);
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "goal_ != current_goal: %d", goal_ != current_goal );
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "goal_was_updated_: %d", goal_was_updated_ );

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    goal_was_updated_ = true;
    // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "<<<<<<<<<<<<<<< goal_was_updated_: %d <<<<<<<<<<<<<<< ", goal_was_updated_ );
  }
  // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "child_node status: %d", (int)child_node_->status() ); // 0=idle,running=1,success=2,failure=3

  // The child gets ticked the first time through and any time the goal has
  // changed or preempted. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) {
    goal_was_updated_ = false;
    // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "------------------------ clear costmap ticked ------------------------");
    const BT::NodeStatus child_state = child_node_->executeTick();
    // RCLCPP_INFO(rclcpp::get_logger("goal_updated_controller"), "goal_was_updated_: %d", goal_was_updated_ );


    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedController>("GoalUpdatedController");
}
