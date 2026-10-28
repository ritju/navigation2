// Copyright (c) 2026, Capella
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/curvature_model.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"

// Same map, obstacle and poses as CurvatureModel.straight_search_reaches_zero_steer_goal.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("curvature_debug_viz");
  auto lifecycle = std::make_shared<rclcpp_lifecycle::LifecycleNode>("curvature_debug_viz_lifecycle");

  constexpr double kWheelbase = 0.85;
  constexpr double kMaxAngle = 0.7853981633974483;
  constexpr double kRate = 0.5;
  constexpr double kAccel = 1.0;
  constexpr double kSpeed = 0.5;
  constexpr double kLength = 0.65;
  constexpr double kResolution = 0.1;

  nav2_smac_planner::SearchInfo info;
  info.use_curvature_state = true;
  info.heading_quantization = 72;
  info.change_penalty = 0.f;
  info.non_straight_penalty = 1.2f;
  info.reverse_penalty = 2.f;
  info.cost_penalty = 2.f;
  info.retrospective_penalty = 0.015f;
  info.wheelbase = static_cast<float>(kWheelbase);
  info.max_steering_angle = static_cast<float>(kMaxAngle);
  info.max_steering_rate = static_cast<float>(kRate);
  info.max_steering_accel = static_cast<float>(kAccel);
  info.reference_speed = static_cast<float>(kSpeed);
  info.primitive_length = static_cast<float>(kLength);
  info.costmap_resolution = static_cast<float>(kResolution);
  info.minimum_turning_radius =
    static_cast<float>((kWheelbase / std::tan(kMaxAngle)) / kResolution);
  info.analytic_expansion_ratio = 3.5f;
  info.analytic_expansion_max_length = 20.f;

  nav2_smac_planner::CurvatureLimits limits;
  limits.wheelbase = kWheelbase;
  limits.max_angle = kMaxAngle;
  limits.max_rate = kRate;
  limits.max_accel = kAccel;
  limits.reference_speed = kSpeed;
  limits.primitive_length = kLength;
  limits.resolution = kResolution;
  const unsigned int dim3 = 72u * nav2_smac_planner::curvatureBinCount(limits);

  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 20000;
  const int on_approach = 1000;
  a_star.initialize(false, max_iterations, on_approach, 10.0, 21.f, dim3);

  auto costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(80, 80, kResolution, 0.0, 0.0, 0);
  for (unsigned int x = 30; x <= 32; ++x) {
    for (unsigned int y = 19; y <= 21; ++y) {
      costmap->setCost(x, y, 254);
    }
  }
  auto checker = std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap.get(), 72, lifecycle);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star.setCollisionChecker(checker.get());

  const unsigned int straight =
    nav2_smac_planner::NodeHybrid::motion_table.curvature.bins.straight_index;
  a_star.setStart(10u, 20u, straight * 72u);
  a_star.setGoal(55u, 20u, straight * 72u);
  auto * start = a_star.getStart();
  start->cont_x = 10.f;
  start->cont_y = 20.f;
  start->cont_yaw = 0.f;
  start->cont_delta = 0.f;
  start->kappa_bin = static_cast<int>(straight);
  start->steer_settled = true;
  auto * goal = a_star.getGoal();
  goal->cont_x = 55.f;
  goal->cont_y = 20.f;
  goal->cont_yaw = 0.f;
  goal->cont_delta = 0.f;
  goal->kappa_bin = static_cast<int>(straight);
  goal->steer_settled = true;
  nav2_smac_planner::NodeHybrid::motion_table.goal_cont_x = 55.f;
  nav2_smac_planner::NodeHybrid::motion_table.goal_cont_y = 20.f;
  nav2_smac_planner::NodeHybrid::motion_table.goal_cont_yaw = 0.f;

  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  int iterations = 0;
  const float tolerance_cells = static_cast<float>(0.25 / kResolution);
  if (!a_star.createPath(path, iterations, tolerance_cells) || path.size() < 2u) {
    RCLCPP_ERROR(node->get_logger(), "curvature debug search failed");
    rclcpp::shutdown();
    return 1;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
  auto plan_pub = node->create_publisher<nav_msgs::msg::Path>("plan", qos);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("obstacle", qos);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.header.stamp = node->now();
  grid.info.resolution = static_cast<float>(kResolution);
  grid.info.width = costmap->getSizeInCellsX();
  grid.info.height = costmap->getSizeInCellsY();
  grid.info.origin.position.x = costmap->getOriginX();
  grid.info.origin.position.y = costmap->getOriginY();
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x < grid.info.width; ++x) {
      const unsigned char cost = costmap->getCost(x, y);
      int8_t occ = 0;
      if (cost >= 254) {
        occ = 100;
      }
      grid.data[x + y * grid.info.width] = occ;
    }
  }

  nav_msgs::msg::Path plan;
  plan.header = grid.header;
  plan.poses.reserve(path.size());
  for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose = nav2_smac_planner::getWorldCoords(path[i].x, path[i].y, costmap.get());
    pose.pose.orientation = nav2_smac_planner::getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  visualization_msgs::msg::Marker marker;
  marker.header = grid.header;
  marker.ns = "obstacle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = (31.0 + 0.5) * kResolution;
  marker.pose.position.y = (20.0 + 0.5) * kResolution;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 3.0 * kResolution;
  marker.scale.y = 3.0 * kResolution;
  marker.scale.z = 0.1;
  marker.color.r = 0.9f;
  marker.color.a = 0.9f;

  map_pub->publish(grid);
  plan_pub->publish(plan);
  marker_pub->publish(marker);
  RCLCPP_INFO(
    node->get_logger(),
    "published /map /plan (%zu poses, %d iterations) and /obstacle",
    plan.poses.size(), iterations);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
