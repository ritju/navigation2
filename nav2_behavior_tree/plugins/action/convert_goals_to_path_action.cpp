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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/action/convert_goals_to_path_action.hpp"
#include "nav2_behavior_tree/mission_path_sync.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace nav2_behavior_tree
{

ConvertGoalsToPath::ConvertGoalsToPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  getInput("robot_base_frame", robot_base_frame_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("convert_goals_path", qos);
}

bool ConvertGoalsToPath::ensurePoseInMap(geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = kMapFrame;
    return true;
  }

  if (pose.header.frame_id == kMapFrame) {
    return true;
  }

  geometry_msgs::msg::PoseStamped pose_in_map;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, pose_in_map, *tf_, kMapFrame, transform_tolerance_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ConvertGoalsToPath: failed to transform pose from '%s' to '%s'",
      pose.header.frame_id.c_str(), kMapFrame);
    return false;
  }

  pose = pose_in_map;
  return true;
}

void ConvertGoalsToPath::appendInterpolatedSegment(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double interval_length)
{
  const double dx = end.pose.position.x - start.pose.position.x;
  const double dy = end.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (interval_length <= 0.0 || distance <= interval_length) {
    return;
  }

  const int num_intervals = static_cast<int>(distance / interval_length);
  const double yaw = std::atan2(dy, dx);
  const auto orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);

  for (int k = 1; k < num_intervals; ++k) {
    geometry_msgs::msg::PoseStamped intermediate = start;
    const double ratio = static_cast<double>(k) * interval_length / distance;
    intermediate.pose.position.x = start.pose.position.x + ratio * dx;
    intermediate.pose.position.y = start.pose.position.y + ratio * dy;
    intermediate.pose.position.z = 0.0;
    intermediate.pose.orientation = orientation;
    path.poses.push_back(intermediate);
  }
}

void ConvertGoalsToPath::logPathPoses(const nav_msgs::msg::Path & path) const
{
  RCLCPP_INFO(
    node_->get_logger(),
    "ConvertGoalsToPath: generated path with %zu poses (frame=%s)",
    path.poses.size(), path.header.frame_id.c_str());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose;
    RCLCPP_INFO(
      node_->get_logger(),
      "ConvertGoalsToPath: pose[%zu] x=%.3f y=%.3f z=%.3f yaw=%.3f",
      i, p.position.x, p.position.y, p.position.z, tf2::getYaw(p.orientation));
  }
}

BT::NodeStatus ConvertGoalsToPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Goals input_goals;
  getInput("input_goals", input_goals);

  double interval_length = 0.5;
  getInput("interval_length", interval_length);
  getInput("robot_base_frame", robot_base_frame_);

  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  path.header.stamp = node_->now();

  if (input_goals.empty()) {
    path.poses.clear();
    setOutput("path", path);
    path_pub_->publish(path);
    RCLCPP_WARN(
      node_->get_logger(),
      "ConvertGoalsToPath: input_goals is empty, returning empty path.");
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped start_pose;
  if (!nav2_util::getCurrentPose(
      start_pose, *tf_, kMapFrame, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ConvertGoalsToPath: failed to get current pose %s -> %s",
      robot_base_frame_.c_str(), kMapFrame);
    setOutput("path", path);
    return BT::NodeStatus::FAILURE;
  }
  start_pose.pose.position.z = 0.0;

  Goals goals_in_map = input_goals;
  for (auto & pose : goals_in_map) {
    if (!ensurePoseInMap(pose)) {
      setOutput("path", path);
      return BT::NodeStatus::FAILURE;
    }
    pose.pose.position.z = 0.0;
  }

  // base_footprint -> first goal, then through remaining goals
  path.poses.push_back(start_pose);
  appendInterpolatedSegment(path, start_pose, goals_in_map.front(), interval_length);
  path.poses.push_back(goals_in_map.front());

  for (size_t i = 0; i + 1 < goals_in_map.size(); ++i) {
    appendInterpolatedSegment(
      path, goals_in_map[i], goals_in_map[i + 1], interval_length);
    path.poses.push_back(goals_in_map[i + 1]);
  }

  setOutput("path", path);
  path_pub_->publish(path);
  logPathPoses(path);

  // Match ComputePathThroughPoses: FollowPath waits until path is synced to
  // the current NavigateThroughPoses mission generation.
  markPathSyncedToCurrentMission(config().blackboard);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ConvertGoalsToPath>("ConvertGoalsToPath");
}
