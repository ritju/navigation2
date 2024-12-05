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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PEDESTRIAN_IN_FRONT_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PEDESTRIAN_IN_FRONT_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "capella_ros_msg/msg/detect_result.hpp"
#include "capella_ros_msg/msg/single_detector.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to person_detected topic and
 * returns FAILURE when pedestrian in front and SUCCESS otherwise
 */
class IsPedestrianFrontCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsPedestrianFrontCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPedestrianFrontCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPedestrianFrontCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("pedestrain_distance", "pedestrain_distance to base_link in meter"),
      BT::InputPort<std::string>(
        "person_detected_topic", std::string("/person_detected"), "person_detected_topic"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

private:
  /**
   * @brief Callback function for localization_score topic
   * @param msg Shared pointer to std_msgs::msg::Float32 message
   */
  void person_detected_callback(capella_ros_msg::msg::DetectResult::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::DetectResult>::SharedPtr person_detected_sub_;
  std::string person_detected_topic_;
  float pedestrain_distance_;
  bool is_pedestrain_in_front_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped last_pedestrian_pose_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_LOCALIZATION_STATUS_GOOD_
