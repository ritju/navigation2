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

#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "curvature_search_cases.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("curvature_random_viz");
  const auto cases = nav2_smac_planner::makeSearchCases();
  auto context = nav2_smac_planner::makeSearchContext(
    nav2_smac_planner::MotionModel::REEDS_SHEPP);

  nav_msgs::msg::OccupancyGrid grid;
  nav_msgs::msg::Path plan;
  visualization_msgs::msg::Marker obstacle;
  visualization_msgs::msg::MarkerArray paths;
  bool saved_blocked = false;

  for (const auto & item : cases) {
    if (item.kind == nav2_smac_planner::SearchKind::Infeasible) {
      continue;
    }
    const auto result = nav2_smac_planner::runSearchCase(context, item);
    if (!saved_blocked && item.kind == nav2_smac_planner::SearchKind::Blocked && result.success) {
      const auto * costmap = context.costmap.get();
      grid.header.frame_id = "map";
      grid.header.stamp = node->now();
      grid.info.resolution = static_cast<float>(costmap->getResolution());
      grid.info.width = costmap->getSizeInCellsX();
      grid.info.height = costmap->getSizeInCellsY();
      grid.info.origin.position.x = costmap->getOriginX();
      grid.info.origin.position.y = costmap->getOriginY();
      grid.info.origin.orientation.w = 1.0;
      grid.data.assign(grid.info.width * grid.info.height, 0);
      for (unsigned int y = 0; y < grid.info.height; ++y) {
        for (unsigned int x = 0; x < grid.info.width; ++x) {
          if (costmap->getCost(x, y) >= 254) {
            grid.data[x + y * grid.info.width] = 100;
          }
        }
      }
      plan.header = grid.header;
      for (const auto & sample : result.path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = plan.header;
        pose.pose.position.x = sample.x;
        pose.pose.position.y = sample.y;
        pose.pose.orientation.z = std::sin(sample.yaw * 0.5);
        pose.pose.orientation.w = std::cos(sample.yaw * 0.5);
        plan.poses.push_back(pose);
      }
      const auto & a = item.points[20];
      const auto & b = item.points[30];
      obstacle.header = grid.header;
      obstacle.ns = "obstacle";
      obstacle.id = 0;
      obstacle.type = visualization_msgs::msg::Marker::CUBE;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacle.pose.position.x = 0.5 * (a.x + b.x);
      obstacle.pose.position.y = 0.5 * (a.y + b.y);
      obstacle.pose.position.z = 0.05;
      obstacle.pose.orientation.w = 1.0;
      obstacle.scale.x = 0.3;
      obstacle.scale.y = 0.3;
      obstacle.scale.z = 0.1;
      obstacle.color.r = 0.9f;
      obstacle.color.a = 0.9f;
      saved_blocked = true;
    }
    if (!result.success) {
      continue;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node->now();
    marker.ns = "search_paths";
    marker.id = item.index;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    const float hue = static_cast<float>(item.index) / 100.f;
    marker.color.r = std::clamp(std::fabs(hue * 6.f - 3.f) - 1.f, 0.f, 1.f);
    marker.color.g = std::clamp(2.f - std::fabs(hue * 6.f - 2.f), 0.f, 1.f);
    marker.color.b = std::clamp(2.f - std::fabs(hue * 6.f - 4.f), 0.f, 1.f);
    marker.color.a = 0.85f;
    for (const auto & sample : result.path) {
      geometry_msgs::msg::Point position;
      position.x = sample.x;
      position.y = sample.y;
      marker.points.push_back(position);
    }
    paths.markers.push_back(marker);
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
  auto plan_pub = node->create_publisher<nav_msgs::msg::Path>("plan", qos);
  auto obstacle_pub = node->create_publisher<visualization_msgs::msg::Marker>("obstacle", qos);
  auto paths_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "random_paths", qos);
  map_pub->publish(grid);
  plan_pub->publish(plan);
  obstacle_pub->publish(obstacle);
  paths_pub->publish(paths);
  RCLCPP_INFO(
    node->get_logger(),
    "published %zu Reeds-Shepp Hybrid A* paths, blocked plan has %zu poses",
    paths.markers.size(), plan.poses.size());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
