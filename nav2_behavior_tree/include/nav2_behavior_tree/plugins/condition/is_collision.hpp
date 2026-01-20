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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_

#include <string>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that checks for collision by predicting robot pose
 * based on current odometry and cmd_vel, and checking against costmap
 */
class IsCollisionCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsCollisionCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsCollisionCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCollisionCondition() = delete;

  ~IsCollisionCondition();

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  /**
   * @brief Callback function for odometry topic
   * @param msg Shared pointer to nav_msgs::msg::Odometry message
   */
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Callback function for cmd_vel topic
   * @param msg Shared pointer to geometry_msgs::msg::Twist message
   */
  void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Check if predicted pose is collision free (combines forward/backward and rotation collision checking)
   * @param cmd_vel Current commanded velocity
   * @param current_pose Current robot pose
   * @return true if collision free, false otherwise
   */
  bool isCollisionFree(
    const geometry_msgs::msg::Twist & cmd_vel,
    const geometry_msgs::msg::Pose2D & current_pose);

  /**
   * @brief Initialize collision checker components
   */
  void initialize();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Collision checker components
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  // Latest data (protected by mutex)
  std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  bool initialized_;
  bool has_odom_;
  bool has_cmd_vel_;

  // Configuration parameters
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  double simulate_ahead_time_;
  double cycle_frequency_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_COLLISION_
