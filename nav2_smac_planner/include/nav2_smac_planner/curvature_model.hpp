// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_SMAC_PLANNER__CURVATURE_MODEL_HPP_
#define NAV2_SMAC_PLANNER__CURVATURE_MODEL_HPP_

#include <functional>
#include <string>
#include <vector>

namespace nav2_smac_planner
{

struct CurvatureLimits
{
  double wheelbase{0.85};
  double max_angle{0.7853981633974483};
  double max_rate{0.5};
  double max_accel{1.0};
  double reference_speed{0.5};
  double primitive_length{0.65};
  double resolution{0.05};
};

struct CurvatureBinTable
{
  std::vector<double> angles;
  double delta_max{0.0};
  unsigned int straight_index{0};
};

struct BodySample
{
  float x{0.f};
  float y{0.f};
  float yaw{0.f};
  float delta{0.f};
  float kappa{0.f};
};

struct BodyPrimitive
{
  int from_bin{0};
  int to_bin{0};
  int step_sign{0};
  float length_m{0.f};
  bool reverse{false};
  bool padded{true};
  std::vector<BodySample> samples;
};

struct CurvatureModel
{
  bool enabled{false};
  CurvatureLimits limits{};
  CurvatureBinTable bins{};
  std::vector<std::vector<BodyPrimitive>> forward;
  std::vector<std::vector<BodyPrimitive>> reverse_prims;
  std::vector<std::vector<BodyPrimitive>> short_forward;
  std::vector<std::vector<BodyPrimitive>> short_reverse;
  double cached_resolution{-1.0};
  double cached_length{-1.0};
  double cached_speed{-1.0};
  double cached_rate{-1.0};
  double cached_accel{-1.0};
  double cached_wheelbase{-1.0};
  double cached_max_angle{-1.0};
  bool cached_reverse{false};
};

struct CurvatureDebugPoint
{
  float x{0.f};
  float y{0.f};
  float yaw{0.f};
};

using CurvatureDebugCallback =
  std::function<void (const std::string &, const std::vector<CurvatureDebugPoint> &)>;

double maxSteerChange(double arc_length, const CurvatureLimits & limits);

double minSteerTime(double abs_delta, const CurvatureLimits & limits);

CurvatureBinTable buildCurvatureBins(const CurvatureLimits & limits);

unsigned int curvatureBinCount(const CurvatureLimits & limits);

int nearestCurvatureBin(double steering_angle, const CurvatureBinTable & bins);

bool steeringOnBin(double steering_angle, int bin, const CurvatureBinTable & bins);

void integrateSteeringPrimitive(
  double delta0,
  double delta1,
  double length_m,
  bool pad_to_length,
  bool reverse,
  const CurvatureLimits & limits,
  std::vector<BodySample> & samples);

void buildCurvatureModel(const CurvatureLimits & limits, bool allow_reverse, CurvatureModel & model);

bool curvatureCacheMatches(
  const CurvatureModel & model, const CurvatureLimits & limits, bool allow_reverse);

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__CURVATURE_MODEL_HPP_
