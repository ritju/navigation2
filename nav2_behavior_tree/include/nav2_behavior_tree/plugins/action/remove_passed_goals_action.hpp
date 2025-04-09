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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>
#include <deque>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "capella_ros_msg/msg/passed_poses_index.hpp"

namespace nav2_behavior_tree
{

class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<std::string>("local_costmap_topic", std::string("local_costmap/costmap_raw"), "Local costmap topic"),
      BT::InputPort<double>("look_ahead_distance", 3.0, "distance that check if it is occupied"),
      BT::InputPort<std::string>("footprint_topic", std::string("local_costmap/published_footprint"), "Local costmap topic"),
    };
  }

private:
  
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  std::string robot_base_frame_, global_frame_, local_costmap_topic_, footprint_topic_;
  double transform_tolerance_, look_ahead_distance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr removed_path_pub_;
  rclcpp::Publisher<capella_ros_msg::msg::PassedPosesIndex>::SharedPtr passed_poses_index_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr checked_path_sub_;
  void removed_path_callback(const nav_msgs::msg::Path &msg);
  nav_msgs::msg::Path removed_path_;
  bool checked_path_received_, receive_new_goal_;
  std::vector<uint32_t> passed_poses_indexes_;
  double last_initialize_time_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
