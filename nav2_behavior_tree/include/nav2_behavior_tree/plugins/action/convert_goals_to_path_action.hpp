// Copyright (c) 2026 Capella
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONVERT_GOALS_TO_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONVERT_GOALS_TO_PATH_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief Convert input_goals to a nav_msgs::Path in map frame, starting from
 *        current base_link pose, with optional linear interpolation.
 */
class ConvertGoalsToPath : public BT::ActionNodeBase
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

  ConvertGoalsToPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Goals to convert into a path"),
      BT::InputPort<double>(
        "interval_length", 0.5,
        "Spacing (m) between inserted poses on each goals segment; <=0 disables insertion"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path converted from input_goals"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  bool ensurePoseInMap(geometry_msgs::msg::PoseStamped & pose);
  void appendInterpolatedSegment(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double interval_length);
  void logPathPoses(const nav_msgs::msg::Path & path) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::string robot_base_frame_{"base_footprint"};
  double transform_tolerance_{0.1};
  static constexpr const char * kMapFrame = "map";
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONVERT_GOALS_TO_PATH_ACTION_HPP_
