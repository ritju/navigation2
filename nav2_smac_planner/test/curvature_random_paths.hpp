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

#ifndef NAV2_SMAC_PLANNER__CURVATURE_RANDOM_PATHS_HPP_
#define NAV2_SMAC_PLANNER__CURVATURE_RANDOM_PATHS_HPP_

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "nav2_smac_planner/curvature_model.hpp"

namespace nav2_smac_planner
{

struct CurvaturePathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double delta{0.0};
  double kappa{0.0};
  double time{0.0};
};

struct CurvaturePathGroup
{
  int index{0};
  std::string kind;
  std::vector<CurvaturePathPoint> points;
};

struct CurvatureViolation
{
  bool ok{true};
  int index{-1};
  std::string reason;
  double max_abs_delta{0.0};
  double max_abs_kappa{0.0};
  double max_abs_kappa_geom{0.0};
  double max_rate{0.0};
  double max_accel{0.0};
};

inline CurvatureLimits defaultCurvatureLimits()
{
  CurvatureLimits limits;
  limits.wheelbase = 0.85;
  limits.max_angle = 0.7853981633974483;
  limits.max_rate = 0.5;
  limits.max_accel = 1.0;
  limits.reference_speed = 0.5;
  limits.primitive_length = 0.65;
  limits.resolution = 0.1;
  return limits;
}

inline double wrapPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline int primitiveSampleCount(double length_m, const CurvatureLimits & limits)
{
  const double duration = length_m / limits.reference_speed;
  return std::max(
    1, static_cast<int>(std::ceil((limits.reference_speed * duration) / limits.resolution)));
}

inline void appendPrimitiveSamples(
  std::vector<CurvaturePathPoint> & points,
  double & x, double & y, double & yaw, double & time_s,
  double delta0, double delta1, double length_m, bool reverse,
  const CurvatureLimits & limits, int point_limit)
{
  std::vector<BodySample> body;
  integrateSteeringPrimitive(
    delta0, delta1, length_m, true, reverse, limits, body);
  if (body.empty()) {
    return;
  }
  const double duration = length_m / limits.reference_speed;
  const double dt = duration / static_cast<double>(body.size());
  const double c0 = std::cos(yaw);
  const double s0 = std::sin(yaw);
  const double x0 = x;
  const double y0 = y;
  const double yaw0 = yaw;
  for (std::size_t i = 0; i < body.size() &&
    static_cast<int>(points.size()) < point_limit; ++i)
  {
    CurvaturePathPoint point;
    point.x = x0 + c0 * body[i].x - s0 * body[i].y;
    point.y = y0 + s0 * body[i].x + c0 * body[i].y;
    point.yaw = yaw0 + static_cast<double>(body[i].yaw);
    point.delta = body[i].delta;
    point.kappa = body[i].kappa;
    point.time = time_s + (static_cast<double>(i) + 1.0) * dt;
    points.push_back(point);
  }
  x = points.back().x;
  y = points.back().y;
  yaw = points.back().yaw;
  time_s = points.back().time;
}

inline std::vector<CurvaturePathGroup> makeRandomCurvaturePaths(
  int group_count, int point_count, uint32_t seed)
{
  const CurvatureLimits limits = defaultCurvatureLimits();
  const CurvatureBinTable bins = buildCurvatureBins(limits);
  const int last_bin = static_cast<int>(bins.angles.size()) - 1;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> step_dist(0, 2);
  std::uniform_int_distribution<int> reverse_dist(0, 4);
  std::vector<CurvaturePathGroup> groups;
  groups.reserve(static_cast<std::size_t>(group_count));

  for (int group = 0; group < group_count; ++group) {
    CurvaturePathGroup path;
    path.index = group;
    if (group % 10 == 0) {
      path.kind = "straight";
    } else if (group % 10 == 1) {
      path.kind = "turn";
    } else if (group % 10 == 2) {
      path.kind = "reverse";
    } else {
      path.kind = "random";
    }
    path.points.push_back(CurvaturePathPoint{});
    int bin = static_cast<int>(bins.straight_index);
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double time_s = 0.0;
    const int turn_sign = (group % 20 == 1) ? 1 : -1;
    while (static_cast<int>(path.points.size()) < point_count) {
      int step = step_dist(rng) - 1;
      bool reverse = reverse_dist(rng) == 0;
      if (path.kind == "straight") {
        step = 0;
        reverse = false;
      } else if (path.kind == "turn") {
        step = turn_sign;
        reverse = false;
      } else if (path.kind == "reverse") {
        reverse = true;
      }
      int next = std::clamp(bin + step, 0, last_bin);
      if (next == bin) {
        step = 0;
      }
      const bool turn = step != 0;
      const double length = turn ? limits.primitive_length : limits.resolution;
      const int need = primitiveSampleCount(length, limits);
      const int room = point_count - static_cast<int>(path.points.size());
      if (turn && need <= room) {
        appendPrimitiveSamples(
          path.points, x, y, yaw, time_s,
          bins.angles[bin], bins.angles[next], length, reverse, limits, point_count);
        bin = next;
      } else {
        appendPrimitiveSamples(
          path.points, x, y, yaw, time_s,
          bins.angles[bin], bins.angles[bin], limits.resolution, reverse,
          limits, point_count);
      }
    }
    groups.push_back(path);
  }
  return groups;
}

inline CurvatureViolation checkCurvaturePath(
  const std::vector<CurvaturePathPoint> & points, const CurvatureLimits & limits)
{
  CurvatureViolation result;
  const double kappa_max = std::tan(limits.max_angle) / limits.wheelbase;
  auto fail = [&result](int index, const std::string & reason) {
      if (result.ok) {
        result.ok = false;
        result.index = index;
        result.reason = reason;
      }
    };
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    const double abs_delta = std::abs(points[i].delta);
    const double abs_kappa = std::abs(points[i].kappa);
    result.max_abs_delta = std::max(result.max_abs_delta, abs_delta);
    result.max_abs_kappa = std::max(result.max_abs_kappa, abs_kappa);
    if (abs_delta > limits.max_angle + 1e-4) {
      fail(i, "steering angle exceeds max_angle");
    }
    if (abs_kappa > kappa_max + 1e-3) {
      fail(i, "curvature exceeds tan(max_angle) / wheelbase");
    }
    const double kappa_from_delta = std::tan(points[i].delta) / limits.wheelbase;
    if (std::abs(points[i].kappa - kappa_from_delta) > 1e-3) {
      fail(i, "stored curvature disagrees with tan(delta) / wheelbase");
    }
  }
  for (int i = 1; i < static_cast<int>(points.size()); ++i) {
    const double dt = points[i].time - points[i - 1].time;
    if (dt <= 1e-9) {
      fail(i, "sample time does not increase");
      continue;
    }
    const double rate = std::abs(points[i].delta - points[i - 1].delta) / dt;
    result.max_rate = std::max(result.max_rate, rate);
    if (rate > limits.max_rate + 1e-3) {
      fail(i, "steering rate exceeds max_rate at reference_speed");
    }
    const double ds = std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    if (ds > 1e-6) {
      const double dyaw = wrapPi(points[i].yaw - points[i - 1].yaw);
      const double kappa_geom = std::abs(dyaw / ds);
      result.max_abs_kappa_geom = std::max(result.max_abs_kappa_geom, kappa_geom);
      if (kappa_geom > kappa_max + 1e-3) {
        fail(i, "geometric curvature exceeds tan(max_angle) / wheelbase");
      }
    }
  }
  for (int i = 2; i < static_cast<int>(points.size()); ++i) {
    const double dt0 = points[i - 1].time - points[i - 2].time;
    const double dt1 = points[i].time - points[i - 1].time;
    if (dt0 <= 1e-9 || dt1 <= 1e-9) {
      continue;
    }
    const double rate0 = (points[i - 1].delta - points[i - 2].delta) / dt0;
    const double rate1 = (points[i].delta - points[i - 1].delta) / dt1;
    const double accel = std::abs(rate1 - rate0) / (0.5 * (dt0 + dt1));
    result.max_accel = std::max(result.max_accel, accel);
    if (accel > limits.max_accel + 1e-2) {
      fail(i, "steering acceleration exceeds max_accel at reference_speed");
    }
  }
  return result;
}

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__CURVATURE_RANDOM_PATHS_HPP_
