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

#ifndef NAV2_SMAC_PLANNER__CURVATURE_SEARCH_CASES_HPP_
#define NAV2_SMAC_PLANNER__CURVATURE_SEARCH_CASES_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/curvature_model.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"

namespace nav2_smac_planner
{

inline constexpr int kSearchGroups = 100;
inline constexpr int kInputPoints = 50;
inline constexpr uint32_t kSearchSeed = 20261019u;
inline constexpr double kWheelbase = 0.85;
inline constexpr double kMaxAngle = 0.7853981633974483;
inline constexpr double kSteerRate = 0.5;
inline constexpr double kSteerAccel = 1.0;
inline constexpr double kReferenceSpeed = 0.5;
inline constexpr double kPrimitiveLength = 0.65;
inline constexpr double kResolution = 0.1;
inline constexpr double kPositionTolerance = 0.25;
inline constexpr unsigned int kHeadingBins = 72;

enum class SearchKind
{
  Straight,
  Turn,
  Blocked,
  Infeasible
};

struct InputPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SearchCase
{
  int index{0};
  SearchKind kind{SearchKind::Straight};
  std::vector<InputPoint> points;
};

struct GeneratedSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SearchResult
{
  bool success{false};
  bool curvature_ok{false};
  int searches{0};
  int iterations{0};
  int generated_points{0};
  int violation_index{-1};
  std::string error;
  std::string curvature_reason;
  double max_abs_delta{0.0};
  double max_abs_kappa{0.0};
  double max_rate{0.0};
  double max_accel{0.0};
  std::vector<GeneratedSample> path;
};

inline const char * searchKindName(SearchKind kind)
{
  switch (kind) {
    case SearchKind::Straight:
      return "straight";
    case SearchKind::Turn:
      return "turn";
    case SearchKind::Blocked:
      return "blocked";
    case SearchKind::Infeasible:
      return "infeasible";
  }
  return "unknown";
}

inline double kappaMax()
{
  return std::tan(kMaxAngle) / kWheelbase;
}

inline double wrapAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline unsigned int headingBin(double yaw)
{
  double wrapped = std::fmod(yaw, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  int heading = static_cast<int>(std::llround(wrapped / (2.0 * M_PI / kHeadingBins)));
  heading %= static_cast<int>(kHeadingBins);
  if (heading < 0) {
    heading += static_cast<int>(kHeadingBins);
  }
  return static_cast<unsigned int>(heading);
}

inline float cellOf(double world)
{
  return static_cast<float>(world / kResolution - 0.5);
}

inline unsigned int cellIndex(double world, unsigned int size)
{
  const int rounded = static_cast<int>(std::llround(cellOf(world)));
  if (rounded < 0) {
    return 0u;
  }
  if (rounded >= static_cast<int>(size)) {
    return size - 1u;
  }
  return static_cast<unsigned int>(rounded);
}

inline std::vector<SearchCase> makeSearchCases()
{
  std::mt19937 rng(kSearchSeed);
  std::uniform_real_distribution<double> jitter(-0.03, 0.03);
  std::uniform_real_distribution<double> turn_step(0.035, 0.05);
  std::vector<SearchCase> cases;
  cases.reserve(kSearchGroups);
  for (int group = 0; group < kSearchGroups; ++group) {
    SearchCase item;
    item.index = group;
    const int band = group % 4;
    if (band == 0) {
      item.kind = SearchKind::Straight;
    } else if (band == 1) {
      item.kind = SearchKind::Turn;
    } else if (band == 2) {
      item.kind = SearchKind::Blocked;
    } else {
      item.kind = SearchKind::Infeasible;
    }
    item.points.resize(kInputPoints);
    if (item.kind == SearchKind::Turn) {
      double x = 6.0;
      double y = 5.0;
      double yaw = 0.15;
      for (int i = 0; i < kInputPoints; ++i) {
        item.points[i] = InputPoint{x, y, yaw};
        const double step = 0.45;
        x += step * std::cos(yaw);
        y += step * std::sin(yaw);
        yaw = wrapAngle(yaw + turn_step(rng));
      }
    } else {
      const double y0 = item.kind == SearchKind::Blocked ? 6.0 : 8.0;
      for (int i = 0; i < kInputPoints; ++i) {
        item.points[i] = InputPoint{2.0 + i * 0.45, y0 + jitter(rng), 0.0};
      }
    }
    cases.push_back(item);
  }
  return cases;
}

inline void paintBlockedWall(nav2_costmap_2d::Costmap2D * costmap, const SearchCase & item)
{
  const auto & a = item.points[20];
  const auto & b = item.points[30];
  const int cx = static_cast<int>(cellIndex(0.5 * (a.x + b.x), costmap->getSizeInCellsX()));
  const int cy = static_cast<int>(cellIndex(0.5 * (a.y + b.y), costmap->getSizeInCellsY()));
  const int x_size = static_cast<int>(costmap->getSizeInCellsX());
  const int y_size = static_cast<int>(costmap->getSizeInCellsY());
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const int x = cx + dx;
      const int y = cy + dy;
      if (x < 0 || y < 0 || x >= x_size || y >= y_size) {
        continue;
      }
      costmap->setCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y), 254);
    }
  }
}

struct SearchContext
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap;
  std::unique_ptr<GridCollisionChecker> checker;
  std::unique_ptr<AStarAlgorithm<NodeHybrid>> astar;
  unsigned int straight_bin{0};
};

inline SearchContext makeSearchContext(MotionModel model = MotionModel::DUBIN)
{
  SearchContext context;
  context.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("curvature_search_cases");
  context.costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    320, 240, kResolution, 0.0, 0.0, 0);
  context.checker = std::make_unique<GridCollisionChecker>(
    context.costmap.get(), static_cast<unsigned int>(kHeadingBins), context.node);
  context.checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  SearchInfo info;
  info.use_curvature_state = true;
  info.heading_quantization = kHeadingBins;
  info.change_penalty = 0.f;
  info.non_straight_penalty = 1.2f;
  info.reverse_penalty = 2.f;
  info.cost_penalty = 2.f;
  info.retrospective_penalty = 0.015f;
  info.wheelbase = static_cast<float>(kWheelbase);
  info.max_steering_angle = static_cast<float>(kMaxAngle);
  info.max_steering_rate = static_cast<float>(kSteerRate);
  info.max_steering_accel = static_cast<float>(kSteerAccel);
  info.reference_speed = static_cast<float>(kReferenceSpeed);
  info.primitive_length = static_cast<float>(kPrimitiveLength);
  info.costmap_resolution = static_cast<float>(kResolution);
  info.minimum_turning_radius = static_cast<float>(
    (kWheelbase / std::tan(kMaxAngle)) / kResolution);
  info.analytic_expansion_ratio = 3.5f;
  info.analytic_expansion_max_length = 20.f;
  CurvatureLimits limits;
  limits.wheelbase = kWheelbase;
  limits.max_angle = kMaxAngle;
  limits.max_rate = kSteerRate;
  limits.max_accel = kSteerAccel;
  limits.reference_speed = kReferenceSpeed;
  limits.primitive_length = kPrimitiveLength;
  limits.resolution = kResolution;
  const unsigned int dim3 = kHeadingBins * curvatureBinCount(limits);
  context.astar = std::make_unique<AStarAlgorithm<NodeHybrid>>(model, info);
  int max_iterations = 30000;
  context.astar->initialize(false, max_iterations, 1000, 5.0, 51.f, dim3);
  context.astar->setCollisionChecker(context.checker.get());
  context.astar->setGoalHeadingTolerance(-1.0);
  context.straight_bin = NodeHybrid::motion_table.curvature.bins.straight_index;
  return context;
}

inline bool searchBetween(
  SearchContext & context,
  const InputPoint & from,
  const InputPoint & to,
  NodeHybrid::CoordinateVector & path,
  int & iterations,
  std::string & error)
{
  const unsigned int straight = context.straight_bin;
  const unsigned int h0 = headingBin(from.yaw);
  const unsigned int h1 = headingBin(to.yaw);
  const unsigned int d0 = h0 + kHeadingBins * straight;
  const unsigned int d1 = h1 + kHeadingBins * straight;
  const unsigned int mx0 = cellIndex(from.x, context.costmap->getSizeInCellsX());
  const unsigned int my0 = cellIndex(from.y, context.costmap->getSizeInCellsY());
  const unsigned int mx1 = cellIndex(to.x, context.costmap->getSizeInCellsX());
  const unsigned int my1 = cellIndex(to.y, context.costmap->getSizeInCellsY());
  context.astar->setCollisionChecker(context.checker.get());
  try {
    context.astar->setStart(mx0, my0, d0);
    context.astar->setGoal(mx1, my1, d1);
    auto * start = context.astar->getStart();
    start->cont_x = cellOf(from.x);
    start->cont_y = cellOf(from.y);
    start->cont_yaw = static_cast<float>(from.yaw);
    start->cont_delta = 0.f;
    start->kappa_bin = static_cast<int>(straight);
    start->steer_settled = true;
    start->pose.theta = static_cast<float>(h0);
    auto * goal = context.astar->getGoal();
    goal->cont_x = cellOf(to.x);
    goal->cont_y = cellOf(to.y);
    goal->cont_yaw = static_cast<float>(to.yaw);
    goal->cont_delta = 0.f;
    goal->kappa_bin = static_cast<int>(straight);
    goal->steer_settled = true;
    goal->pose.theta = static_cast<float>(h1);
    NodeHybrid::motion_table.goal_cont_x = goal->cont_x;
    NodeHybrid::motion_table.goal_cont_y = goal->cont_y;
    NodeHybrid::motion_table.goal_cont_yaw = goal->cont_yaw;
    path.clear();
    iterations = 0;
    const float tolerance_cells = static_cast<float>(kPositionTolerance / kResolution);
    if (!context.astar->createPath(path, iterations, tolerance_cells) || path.empty()) {
      error = "createPath returned false";
      return false;
    }
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
  return true;
}

inline void appendGenerated(
  std::vector<GeneratedSample> & path,
  const NodeHybrid::CoordinateVector & cell_path,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  for (int i = static_cast<int>(cell_path.size()) - 1; i >= 0; --i) {
    const auto world = getWorldCoords(cell_path[i].x, cell_path[i].y, costmap);
    GeneratedSample sample;
    sample.x = world.position.x;
    sample.y = world.position.y;
    sample.yaw = cell_path[i].theta;
    if (!path.empty()) {
      const double ds = std::hypot(sample.x - path.back().x, sample.y - path.back().y);
      const double dyaw = std::abs(wrapAngle(sample.yaw - path.back().yaw));
      if (ds < 1e-4 && dyaw < 1e-4) {
        continue;
      }
    }
    path.push_back(sample);
  }
}

inline void checkGeneratedCurvature(SearchResult & result)
{
  result.curvature_ok = true;
  result.max_abs_delta = 0.0;
  result.max_abs_kappa = 0.0;
  result.max_rate = 0.0;
  result.max_accel = 0.0;
  const auto & path = result.path;
  if (path.size() < 2u) {
    result.curvature_ok = false;
    result.curvature_reason = "generated path has fewer than 2 points";
    return;
  }
  std::vector<double> delta(path.size(), 0.0);
  std::vector<double> t_mid(path.size(), 0.0);
  double traveled = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const double dyaw = wrapAngle(path[i].yaw - path[i - 1].yaw);
    if (ds < 1e-4) {
      if (std::abs(dyaw) > 1e-3) {
        result.curvature_ok = false;
        result.violation_index = static_cast<int>(i);
        result.curvature_reason = "in-place steering";
      }
      t_mid[i] = traveled;
      delta[i] = i > 1 ? delta[i - 1] : 0.0;
      continue;
    }
    const double travel = std::atan2(
      path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
    const double along = std::cos(wrapAngle(travel - path[i - 1].yaw));
    const double kappa = (along < 0.0 ? -dyaw : dyaw) / ds;
    const double steer = std::atan(kappa * kWheelbase);
    traveled += ds / kReferenceSpeed;
    delta[i] = steer;
    t_mid[i] = traveled - 0.5 * ds / kReferenceSpeed;
    result.max_abs_kappa = std::max(result.max_abs_kappa, std::abs(kappa));
    result.max_abs_delta = std::max(result.max_abs_delta, std::abs(steer));
    if (std::abs(kappa) > kappaMax() + 1e-2) {
      result.curvature_ok = false;
      result.violation_index = static_cast<int>(i);
      result.curvature_reason = "geometric curvature exceeds tan(max_angle) / wheelbase";
    }
    if (std::abs(steer) > kMaxAngle + 1e-2) {
      result.curvature_ok = false;
      result.violation_index = static_cast<int>(i);
      result.curvature_reason = "steering angle exceeds max_angle";
    }
  }
  std::vector<double> rate;
  std::vector<double> rate_time;
  rate.reserve(path.size());
  rate_time.reserve(path.size());
  for (std::size_t i = 2; i < path.size(); ++i) {
    const double span = t_mid[i] - t_mid[i - 1];
    if (span <= 1e-6) {
      continue;
    }
    rate.push_back((delta[i] - delta[i - 1]) / span);
    rate_time.push_back(0.5 * (t_mid[i] + t_mid[i - 1]));
    result.max_rate = std::max(result.max_rate, std::abs(rate.back()));
    if (std::abs(rate.back()) > kSteerRate + 0.05) {
      result.curvature_ok = false;
      result.violation_index = static_cast<int>(i);
      result.curvature_reason = "steering rate exceeds max_rate at reference_speed";
    }
  }
  for (std::size_t i = 1; i < rate.size(); ++i) {
    const double span = rate_time[i] - rate_time[i - 1];
    if (span <= 1e-6) {
      continue;
    }
    const double accel = std::abs(rate[i] - rate[i - 1]) / span;
    result.max_accel = std::max(result.max_accel, accel);
    if (accel > kSteerAccel + 0.05) {
      result.curvature_ok = false;
      result.violation_index = static_cast<int>(i);
      result.curvature_reason = "steering acceleration exceeds max_accel at reference_speed";
    }
  }
}

inline SearchResult runSearchCase(SearchContext & context, const SearchCase & item)
{
  SearchResult result;
  context.costmap->resetMap(
    0, 0, context.costmap->getSizeInCellsX(), context.costmap->getSizeInCellsY());
  if (item.kind == SearchKind::Blocked) {
    paintBlockedWall(context.costmap.get(), item);
  }
  if (item.kind == SearchKind::Infeasible) {
    const unsigned int mx = cellIndex(item.points.front().x, context.costmap->getSizeInCellsX());
    const unsigned int my = cellIndex(item.points.front().y, context.costmap->getSizeInCellsY());
    context.costmap->setCost(mx, my, 254);
  }

  std::vector<int> goal_index;
  if (item.kind == SearchKind::Infeasible) {
    goal_index.push_back(kInputPoints - 1);
  } else if (item.kind == SearchKind::Blocked) {
    goal_index = {10, 20, 30, 40, kInputPoints - 1};
  } else {
    for (int i = 1; i < kInputPoints; ++i) {
      goal_index.push_back(i);
    }
  }
  InputPoint cursor = item.points.front();
  for (const int goal : goal_index) {
    const InputPoint & to = item.points[goal];
    NodeHybrid::CoordinateVector piece;
    int iterations = 0;
    std::string error;
    ++result.searches;
    const bool ok = searchBetween(context, cursor, to, piece, iterations, error);
    result.iterations += iterations;
    if (!ok) {
      result.success = false;
      result.error = error;
      result.generated_points = static_cast<int>(result.path.size());
      return result;
    }
    appendGenerated(result.path, piece, context.costmap.get());
    if (!result.path.empty()) {
      cursor.x = result.path.back().x;
      cursor.y = result.path.back().y;
      cursor.yaw = result.path.back().yaw;
    }
  }
  result.success = true;
  result.generated_points = static_cast<int>(result.path.size());
  checkGeneratedCurvature(result);
  return result;
}

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__CURVATURE_SEARCH_CASES_HPP_
