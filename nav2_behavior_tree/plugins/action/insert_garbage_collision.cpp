#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_behavior_tree/plugins/action/insert_garbage_pose_action.hpp"

namespace nav2_behavior_tree
{

// footprint 与代价地图回调，以及车体碰撞检查

// footprint 话题回调
void InsertGarbagePose::footprintCallback(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (footprint_topic_sub_) {
    footprint_topic_sub_->feed(msg);
  }
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  footprint_dirty_ = true;
}

void InsertGarbagePose::globalCostmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  if (!msg || !costmap_sub_) {
    return;
  }
  costmap_sub_->costmapCallback(msg);
}

// ========================================================================
// A. footprint / 碰撞检查
// ========================================================================

bool InsertGarbagePose::ensureCachedFootprint(std::string * reason) const
{
  auto haveUsableCache = [this]() {
      std::lock_guard<std::mutex> lock(footprint_mutex_);
      return have_cached_footprint_ && cached_footprint_base_.size() >= 3;
    };
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    if (have_cached_footprint_ && !footprint_dirty_ && cached_footprint_base_.size() >= 3) {
      return true;
    }
  }
  if (!footprint_topic_sub_) {
    if (haveUsableCache()) {
      return true;
    }
    if (reason) {
      *reason = "no footprint subscriber";
    }
    return false;
  }

  std::vector<geometry_msgs::msg::Point> fp;
  std_msgs::msg::Header header;
  if (!footprint_topic_sub_->getFootprintInRobotFrame(fp, header) || fp.size() < 3) {
    // 取不到新的就继续用上一份，别因为一拍 TF 抖动让所有安全检查都判不通过
    if (haveUsableCache()) {
      return true;
    }
    if (reason) {
      *reason = "no footprint";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(footprint_mutex_);
  cached_footprint_base_.clear();
  cached_footprint_base_.reserve(fp.size());
  double min_x = fp.front().x;
  double max_x = fp.front().x;
  for (const auto & pt : fp) {
    cached_footprint_base_.emplace_back(pt.x, pt.y);
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
  }
  cached_robot_length_m_ = std::max(0.05, max_x - min_x);
  have_cached_footprint_ = true;
  footprint_dirty_ = false;
  return true;
}
// 2.11：车体轮廓摆到 (x,y,yaw)，判定交给 nav2 的 CostmapTopicCollisionChecker
bool InsertGarbagePose::isCollisionFreeAtPose(
  double x, double y, double yaw, std::string * reason,
  bool fetch_costmap_and_footprint) const
{
  if (!ensureCachedFootprint(reason)) {
    return false;
  }
  if (!collision_checker_) {
    if (reason) {
      *reason = "no collision checker";
    }
    return false;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = yaw;
  if (!collision_checker_->isCollisionFree(pose, fetch_costmap_and_footprint)) {
    if (reason) {
      *reason = "footprint collision";
    }
    return false;
  }
  return true;
}

bool InsertGarbagePose::isFootprintClearAtPose(
  double x, double y, double yaw, std::string * reason) const
{
  return isCollisionFreeAtPose(x, y, yaw, reason, true);
}

// 垃圾点检查、延长点检查、线段检查都用这一条，步长取 footprint 的车长。
bool InsertGarbagePose::isFootprintSweepClear(
  double x0, double y0, double x1, double y1, std::string * reason) const
{
  if (!ensureCachedFootprint(reason)) {
    return false;
  }

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double yaw = (len < 1e-9) ? 0.0 : std::atan2(dy, dx);
  double step = 0.5;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    step = std::max(0.05, cached_robot_length_m_);
  }

  if (!isCollisionFreeAtPose(x0, y0, yaw, reason, true)) {
    return false;
  }
  if (len < 1e-9) {
    return true;
  }

  const double ux = dx / len;
  const double uy = dy / len;
  for (double s = step; s < len - 1e-9; s += step) {
    if (!isCollisionFreeAtPose(x0 + ux * s, y0 + uy * s, yaw, reason, false)) {
      return false;
    }
  }
  if (!isCollisionFreeAtPose(x1, y1, yaw, reason, false)) {
    return false;
  }
  return true;
}

bool InsertGarbagePose::isRobotGarbageSegmentClearOfLethal(
  double robot_x, double robot_y, double gx, double gy,
  std::string * reason) const
{
  if (!costmap_sub_) {
    if (reason) {
      *reason = "no costmap subscriber";
    }
    return false;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception &) {
    if (reason) {
      *reason = "costmap unavailable";
    }
    return false;
  }
  if (!costmap) {
    if (reason) {
      *reason = "no costmap";
    }
    return false;
  }

  unsigned int mx0 = 0;
  unsigned int my0 = 0;
  unsigned int mx1 = 0;
  unsigned int my1 = 0;
  if (!costmap->worldToMap(robot_x, robot_y, mx0, my0)) {
    if (reason) {
      *reason = "robot out of costmap";
    }
    return false;
  }
  if (!costmap->worldToMap(gx, gy, mx1, my1)) {
    if (reason) {
      *reason = "garbage out of costmap";
    }
    return false;
  }

  const int x0 = static_cast<int>(mx0);
  const int y0 = static_cast<int>(my0);
  const int x1 = static_cast<int>(mx1);
  const int y1 = static_cast<int>(my1);

  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    const int lx = line.getX();
    const int ly = line.getY();
    if (lx == x1 && ly == y1) {
      // 垃圾格常在墙边 lethal 上，贴墙进表不卡终点
      continue;
    }
    if (lx < 0 || ly < 0) {
      continue;
    }
    const unsigned int mx = static_cast<unsigned int>(lx);
    const unsigned int my = static_cast<unsigned int>(ly);
    if (mx >= costmap->getSizeInCellsX() || my >= costmap->getSizeInCellsY()) {
      if (reason) {
        *reason = "line leaves costmap";
      }
      return false;
    }
    if (costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      if (reason) {
        *reason = "lethal on robot-garbage line";
      }
      return false;
    }
  }
  return true;
}

bool InsertGarbagePose::findNearestObstaclePixel(
  double x, double y, double search_radius_m, double * ox, double * oy)
{
  if (!ox || !oy || !costmap_sub_ || search_radius_m <= 0.0) {
    return false;
  }

  // 在全局代价图上找最近致命障碍格
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception &) {
    return false;
  }
  if (!costmap) {
    return false;
  }

  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!costmap->worldToMap(x, y, gx, gy)) {
    return false;
  }

  const double resolution = costmap->getResolution();
  if (resolution <= 0.0) {
    return false;
  }
  const int r_cells = std::max(1, static_cast<int>(std::ceil(search_radius_m / resolution)));
  const double search_r2 = search_radius_m * search_radius_m;
  const int width = static_cast<int>(costmap->getSizeInCellsX());
  const int height = static_cast<int>(costmap->getSizeInCellsY());
  const int gx_i = static_cast<int>(gx);
  const int gy_i = static_cast<int>(gy);

  bool found = false;
  double best_d2 = std::numeric_limits<double>::infinity();
  double best_cx = 0.0;
  double best_cy = 0.0;

  const int mx0 = std::max(0, gx_i - r_cells);
  const int mx1 = std::min(width - 1, gx_i + r_cells);
  const int my0 = std::max(0, gy_i - r_cells);
  const int my1 = std::min(height - 1, gy_i + r_cells);
  for (int my = my0; my <= my1; ++my) {
    for (int mx = mx0; mx <= mx1; ++mx) {
      if (costmap->getCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my)) !=
        nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap->mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const double d2 = (wx - x) * (wx - x) + (wy - y) * (wy - y);
      if (d2 <= search_r2 && d2 < best_d2) {
        best_d2 = d2;
        best_cx = wx;
        best_cy = wy;
        found = true;
      }
    }
  }
  if (!found) {
    return false;
  }
  *ox = best_cx;
  *oy = best_cy;
  return true;
}

}  // namespace nav2_behavior_tree
