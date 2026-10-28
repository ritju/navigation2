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

#include <iostream>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "curvature_search_cases.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

void expectFeasiblePaths(nav2_smac_planner::MotionModel model, const char * model_name)
{
  const auto cases = nav2_smac_planner::makeSearchCases();
  ASSERT_EQ(cases.size(), 100u);
  auto context = nav2_smac_planner::makeSearchContext(model);

  int groups = 0;
  int searches = 0;
  int iterations = 0;
  int violations = 0;
  int failures = 0;
  double max_delta = 0.0;
  double max_kappa = 0.0;
  double max_rate = 0.0;
  double max_accel = 0.0;
  for (const auto & item : cases) {
    ASSERT_EQ(item.points.size(), 50u);
    if (item.kind == nav2_smac_planner::SearchKind::Infeasible) {
      continue;
    }
    ++groups;
    const auto result = nav2_smac_planner::runSearchCase(context, item);
    searches += result.searches;
    iterations += result.iterations;
    max_delta = std::max(max_delta, result.max_abs_delta);
    max_kappa = std::max(max_kappa, result.max_abs_kappa);
    max_rate = std::max(max_rate, result.max_rate);
    max_accel = std::max(max_accel, result.max_accel);
    if (!result.success) {
      ++failures;
      ADD_FAILURE() << "group " << item.index << " kind "
                    << nav2_smac_planner::searchKindName(item.kind)
                    << " search failed: " << result.error
                    << " iterations " << result.iterations;
      continue;
    }
    if (!result.curvature_ok) {
      ++violations;
      ADD_FAILURE() << "group " << item.index << " kind "
                    << nav2_smac_planner::searchKindName(item.kind)
                    << " point " << result.violation_index << " "
                    << result.curvature_reason
                    << " max_kappa " << result.max_abs_kappa
                    << " max_rate " << result.max_rate
                    << " max_accel " << result.max_accel;
    }
    if (item.kind == nav2_smac_planner::SearchKind::Blocked) {
      EXPECT_GT(result.iterations, 20) << "group " << item.index;
    }
  }
  std::cout << "CURVATURE_SEARCH_SUMMARY model=" << model_name
            << " groups=" << groups
            << " input_points=50 searches=" << searches
            << " failures=" << failures
            << " iterations=" << iterations
            << " curvature_violations=" << violations
            << " max_delta=" << max_delta
            << " max_kappa=" << max_kappa
            << " max_rate=" << max_rate
            << " max_accel=" << max_accel
            << " limit_delta=" << nav2_smac_planner::kMaxAngle
            << " limit_kappa=" << nav2_smac_planner::kappaMax()
            << " limit_rate=" << nav2_smac_planner::kSteerRate
            << " limit_accel=" << nav2_smac_planner::kSteerAccel
            << std::endl;
  EXPECT_EQ(groups, 75);
  EXPECT_GT(iterations, 100);
  EXPECT_EQ(failures, 0);
  EXPECT_EQ(violations, 0);
}

TEST(CurvatureSearchRandom, feasible_groups_search_and_meet_curvature_limits)
{
  expectFeasiblePaths(nav2_smac_planner::MotionModel::DUBIN, "DUBIN");
}

TEST(CurvatureSearchRandom, reeds_shepp_feasible_groups_meet_curvature_limits)
{
  expectFeasiblePaths(nav2_smac_planner::MotionModel::REEDS_SHEPP, "REEDS_SHEPP");
}

void expectLethalStartRejected(nav2_smac_planner::MotionModel model)
{
  const auto cases = nav2_smac_planner::makeSearchCases();
  auto context = nav2_smac_planner::makeSearchContext(model);
  int groups = 0;
  for (const auto & item : cases) {
    if (item.kind != nav2_smac_planner::SearchKind::Infeasible) {
      continue;
    }
    ++groups;
    ASSERT_EQ(item.points.size(), 50u);
    const auto result = nav2_smac_planner::runSearchCase(context, item);
    EXPECT_FALSE(result.success) << "group " << item.index;
    EXPECT_NE(result.error.find("lethal"), std::string::npos) << result.error;
  }
  EXPECT_EQ(groups, 25);
}

TEST(CurvatureSearchRandom, infeasible_groups_reject_lethal_start)
{
  expectLethalStartRejected(nav2_smac_planner::MotionModel::DUBIN);
}

TEST(CurvatureSearchRandom, reeds_shepp_infeasible_groups_reject_lethal_start)
{
  expectLethalStartRejected(nav2_smac_planner::MotionModel::REEDS_SHEPP);
}
