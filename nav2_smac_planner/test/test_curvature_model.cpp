// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0

#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/curvature_model.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"

namespace
{
constexpr double kWheelbase = 0.85;
constexpr double kMaxAngle = 0.7853981633974483;
constexpr double kRate = 0.5;
constexpr double kAccel = 1.0;
constexpr double kSpeed = 0.5;
constexpr double kLength = 0.65;
constexpr double kResolution = 0.1;
}

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(CurvatureModel, default_bins_match_rate_and_acceleration)
{
  nav2_smac_planner::CurvatureLimits limits;
  limits.wheelbase = kWheelbase;
  limits.max_angle = kMaxAngle;
  limits.max_rate = kRate;
  limits.max_accel = kAccel;
  limits.reference_speed = kSpeed;
  limits.primitive_length = kLength;
  limits.resolution = kResolution;

  const double delta_max = nav2_smac_planner::maxSteerChange(kLength, limits);
  EXPECT_NEAR(delta_max, 0.4, 1e-6);
  const auto bins = nav2_smac_planner::buildCurvatureBins(limits);
  EXPECT_EQ(bins.angles.size(), 5u);
  EXPECT_EQ(bins.straight_index, 2u);
  EXPECT_NEAR(bins.angles.front(), -kMaxAngle, 1e-6);
  EXPECT_NEAR(bins.angles[2], 0.0, 1e-6);
  EXPECT_NEAR(bins.angles.back(), kMaxAngle, 1e-6);
  EXPECT_LE(bins.angles[1] - bins.angles[0], delta_max + 1e-6);
}

TEST(CurvatureModel, rest_to_rest_step_ends_at_the_next_bin)
{
  nav2_smac_planner::CurvatureLimits limits;
  limits.wheelbase = kWheelbase;
  limits.max_angle = kMaxAngle;
  limits.max_rate = kRate;
  limits.max_accel = kAccel;
  limits.reference_speed = kSpeed;
  limits.primitive_length = kLength;
  limits.resolution = kResolution;
  const auto bins = nav2_smac_planner::buildCurvatureBins(limits);
  std::vector<nav2_smac_planner::BodySample> samples;
  nav2_smac_planner::integrateSteeringPrimitive(
    bins.angles[2], bins.angles[3], kLength, true, false, limits, samples);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.back().delta, bins.angles[3], 1e-3);
  EXPECT_LE(std::abs(samples.back().x), kLength + 0.05);
  EXPECT_GT(samples.back().x, 0.5);
  const double dt = (kLength / kSpeed) / static_cast<double>(samples.size());
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double rate = std::abs(samples[i].delta - samples[i - 1].delta) / dt;
    EXPECT_LE(rate, kRate + 0.05);
  }
}

TEST(CurvatureModel, negative_heading_tolerance_skips_yaw_and_nonzero_steer_is_not_goal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("curvature_goal_test");
  nav2_smac_planner::SearchInfo info;
  info.use_curvature_state = true;
  info.heading_quantization = 72;
  info.wheelbase = static_cast<float>(kWheelbase);
  info.max_steering_angle = static_cast<float>(kMaxAngle);
  info.max_steering_rate = static_cast<float>(kRate);
  info.max_steering_accel = static_cast<float>(kAccel);
  info.reference_speed = static_cast<float>(kSpeed);
  info.primitive_length = static_cast<float>(kLength);
  info.costmap_resolution = static_cast<float>(kResolution);
  info.minimum_turning_radius =
    static_cast<float>((kWheelbase / std::tan(kMaxAngle)) / kResolution);
  info.retrospective_penalty = 0.0f;
  unsigned int size_x = 20;
  unsigned int size_y = 20;
  unsigned int heading = 72;
  nav2_smac_planner::NodeHybrid::initMotionModel(
    nav2_smac_planner::MotionModel::DUBIN, size_x, size_y, heading, info);

  nav2_smac_planner::NodeHybrid start(0);
  nav2_smac_planner::NodeHybrid goal(1);
  start.cont_x = 5.f;
  start.cont_y = 5.f;
  start.cont_yaw = 0.2f;
  start.cont_delta = 0.2f;
  start.kappa_bin = 2;
  goal.cont_x = 5.f;
  goal.cont_y = 5.f;
  goal.cont_yaw = 0.f;
  EXPECT_FALSE(start.reachedCurvatureGoal(&goal, 1.f, -1.0));

  start.cont_delta = 0.f;
  start.cont_yaw = 1.2f;
  EXPECT_TRUE(start.reachedCurvatureGoal(&goal, 1.f, -1.0));
  EXPECT_FALSE(start.reachedCurvatureGoal(&goal, 1.f, 0.1));
  (void)node;
}

TEST(CurvatureModel, straight_search_reaches_zero_steer_goal)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("curvature_search_test");
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

  auto * costmap = new nav2_costmap_2d::Costmap2D(80, 80, kResolution, 0.0, 0.0, 0);
  for (unsigned int x = 30; x <= 32; ++x) {
    for (unsigned int y = 19; y <= 21; ++y) {
      costmap->setCost(x, y, 254);
    }
  }
  auto checker = std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap, 72, node);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star.setCollisionChecker(checker.get());

  const unsigned int straight = nav2_smac_planner::NodeHybrid::motion_table.curvature.bins.straight_index;
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
  EXPECT_TRUE(a_star.createPath(path, iterations, tolerance_cells));
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x, 55.f, tolerance_cells + 1.f);
  EXPECT_NEAR(path.back().x, 10.f, 1.f);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(
      std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 1.05);
  }
  delete costmap;
}
