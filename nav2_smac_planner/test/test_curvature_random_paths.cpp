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
#include <iostream>
#include <string>

#include "gtest/gtest.h"
#include "curvature_random_paths.hpp"

TEST(CurvatureRandomPaths, hundred_groups_of_fifty_points_meet_limits)
{
  constexpr int kGroups = 100;
  constexpr int kPoints = 50;
  constexpr uint32_t kSeed = 20261019u;
  const auto limits = nav2_smac_planner::defaultCurvatureLimits();
  const double kappa_max = std::tan(limits.max_angle) / limits.wheelbase;
  const auto paths = nav2_smac_planner::makeRandomCurvaturePaths(kGroups, kPoints, kSeed);
  ASSERT_EQ(paths.size(), 100u);

  int violations = 0;
  double max_delta = 0.0;
  double max_kappa = 0.0;
  double max_kappa_geom = 0.0;
  double max_rate = 0.0;
  double max_accel = 0.0;
  for (const auto & path : paths) {
    ASSERT_EQ(path.points.size(), 50u);
    const auto check = nav2_smac_planner::checkCurvaturePath(path.points, limits);
    max_delta = std::max(max_delta, check.max_abs_delta);
    max_kappa = std::max(max_kappa, check.max_abs_kappa);
    max_kappa_geom = std::max(max_kappa_geom, check.max_abs_kappa_geom);
    max_rate = std::max(max_rate, check.max_rate);
    max_accel = std::max(max_accel, check.max_accel);
    if (!check.ok) {
      ++violations;
      ADD_FAILURE() << "group " << path.index << " kind " << path.kind
                    << " point " << check.index << " " << check.reason;
    }
  }
  std::cout << "CURVATURE_SUMMARY groups=" << kGroups
            << " points=" << kPoints
            << " violations=" << violations
            << " max_delta=" << max_delta
            << " max_kappa=" << max_kappa
            << " max_kappa_geom=" << max_kappa_geom
            << " max_rate=" << max_rate
            << " max_accel=" << max_accel
            << " limit_delta=" << limits.max_angle
            << " limit_kappa=" << kappa_max
            << " limit_rate=" << limits.max_rate
            << " limit_accel=" << limits.max_accel
            << std::endl;
  EXPECT_EQ(violations, 0);
}

TEST(CurvatureRandomPaths, steer_jump_on_fifty_points_is_rejected)
{
  const auto limits = nav2_smac_planner::defaultCurvatureLimits();
  std::vector<nav2_smac_planner::CurvaturePathPoint> points(50);
  for (int i = 0; i < 50; ++i) {
    points[i].x = i * limits.resolution;
    points[i].time = i * limits.resolution / limits.reference_speed;
  }
  points[25].delta = 0.7;
  points[25].kappa = std::tan(points[25].delta) / limits.wheelbase;
  const auto check = nav2_smac_planner::checkCurvaturePath(points, limits);
  EXPECT_FALSE(check.ok);
  EXPECT_GT(check.max_rate, limits.max_rate);
}
