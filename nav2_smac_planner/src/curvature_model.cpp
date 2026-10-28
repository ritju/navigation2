// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_smac_planner/curvature_model.hpp"

namespace nav2_smac_planner
{
namespace
{

constexpr double kSteerEps = 1e-6;

double steerAt(
  double delta0, double delta1, double t, double t_min,
  double rate, double accel)
{
  if (t_min <= kSteerEps || t >= t_min) {
    return delta1;
  }
  if (t <= 0.0) {
    return delta0;
  }
  const double delta = delta1 - delta0;
  const double sign = delta >= 0.0 ? 1.0 : -1.0;
  const double abs_delta = std::abs(delta);
  const double tri_max = rate * rate / accel;
  if (abs_delta <= tri_max + kSteerEps) {
    const double half = t_min * 0.5;
    if (t <= half) {
      return delta0 + sign * 0.5 * accel * t * t;
    }
    const double tau = t - half;
    const double peak = 0.5 * accel * half * half;
    return delta0 + sign * (peak + accel * half * tau - 0.5 * accel * tau * tau);
  }

  const double t1 = rate / accel;
  const double cruise = t_min - 2.0 * t1;
  if (t <= t1) {
    return delta0 + sign * 0.5 * accel * t * t;
  }
  if (t <= t1 + cruise) {
    const double tau = t - t1;
    const double acc_angle = 0.5 * accel * t1 * t1;
    return delta0 + sign * (acc_angle + rate * tau);
  }
  const double tau = t - t1 - cruise;
  const double acc_angle = 0.5 * accel * t1 * t1;
  const double cruise_angle = rate * cruise;
  return delta0 + sign * (acc_angle + cruise_angle + rate * tau - 0.5 * accel * tau * tau);
}

BodyPrimitive makePrimitive(
  int from_bin, int to_bin, double length_m, bool pad, bool reverse,
  const CurvatureBinTable & bins, const CurvatureLimits & limits)
{
  BodyPrimitive prim;
  prim.from_bin = from_bin;
  prim.to_bin = to_bin;
  const int raw = to_bin - from_bin;
  prim.step_sign = raw > 0 ? 1 : (raw < 0 ? -1 : 0);
  prim.padded = pad;
  prim.reverse = reverse;
  integrateSteeringPrimitive(
    bins.angles[from_bin], bins.angles[to_bin], length_m, pad, reverse, limits, prim.samples);
  prim.length_m = prim.samples.empty() ? 0.f :
    static_cast<float>(limits.reference_speed * (
      pad ? length_m / limits.reference_speed :
      minSteerTime(std::abs(bins.angles[to_bin] - bins.angles[from_bin]), limits)));
  if (!pad) {
    prim.length_m = static_cast<float>(
      limits.reference_speed *
      minSteerTime(std::abs(bins.angles[to_bin] - bins.angles[from_bin]), limits));
  } else {
    prim.length_m = static_cast<float>(length_m);
  }
  return prim;
}

}  // namespace

double maxSteerChange(double arc_length, const CurvatureLimits & limits)
{
  if (limits.reference_speed <= 0.0 || limits.max_accel <= 0.0 || limits.max_rate <= 0.0) {
    throw std::invalid_argument("curvature limits need positive speed, rate and acceleration");
  }
  const double duration = arc_length / limits.reference_speed;
  const double switch_time = 2.0 * limits.max_rate / limits.max_accel;
  if (duration <= switch_time) {
    return limits.max_accel * duration * duration / 4.0;
  }
  return limits.max_rate * duration -
         (limits.max_rate * limits.max_rate) / limits.max_accel;
}

double minSteerTime(double abs_delta, const CurvatureLimits & limits)
{
  if (abs_delta <= kSteerEps) {
    return 0.0;
  }
  const double tri_max = limits.max_rate * limits.max_rate / limits.max_accel;
  if (abs_delta <= tri_max) {
    return 2.0 * std::sqrt(abs_delta / limits.max_accel);
  }
  return limits.max_rate / limits.max_accel + abs_delta / limits.max_rate;
}

CurvatureBinTable buildCurvatureBins(const CurvatureLimits & limits)
{
  if (limits.primitive_length <= 0.0 || limits.max_angle <= 0.0) {
    throw std::invalid_argument("primitive length and max steering angle must be positive");
  }
  const double delta_max = maxSteerChange(limits.primitive_length, limits);
  if (delta_max <= kSteerEps) {
    throw std::invalid_argument("primitive length is too short to change steering angle");
  }
  const int steps = std::max(
    1, static_cast<int>(std::ceil((limits.max_angle / delta_max) - 1e-9)));
  const double spacing = limits.max_angle / static_cast<double>(steps);
  CurvatureBinTable table;
  table.delta_max = delta_max;
  table.straight_index = static_cast<unsigned int>(steps);
  table.angles.reserve(static_cast<std::size_t>(2 * steps + 1));
  for (int i = -steps; i <= steps; ++i) {
    table.angles.push_back(spacing * static_cast<double>(i));
  }
  return table;
}

unsigned int curvatureBinCount(const CurvatureLimits & limits)
{
  return static_cast<unsigned int>(buildCurvatureBins(limits).angles.size());
}

int nearestCurvatureBin(double steering_angle, const CurvatureBinTable & bins)
{
  int best = 0;
  double best_abs = std::abs(steering_angle - bins.angles[0]);
  for (int i = 1; i < static_cast<int>(bins.angles.size()); ++i) {
    const double err = std::abs(steering_angle - bins.angles[i]);
    if (err < best_abs) {
      best_abs = err;
      best = i;
    }
  }
  return best;
}

bool steeringOnBin(double steering_angle, int bin, const CurvatureBinTable & bins)
{
  if (bin < 0 || bin >= static_cast<int>(bins.angles.size())) {
    return false;
  }
  return std::abs(steering_angle - bins.angles[bin]) <= 1e-4;
}

void integrateSteeringPrimitive(
  double delta0,
  double delta1,
  double length_m,
  bool pad_to_length,
  bool reverse,
  const CurvatureLimits & limits,
  std::vector<BodySample> & samples)
{
  samples.clear();
  const double abs_delta = std::abs(delta1 - delta0);
  const double t_min = minSteerTime(abs_delta, limits);
  const double t_total = pad_to_length ? length_m / limits.reference_speed : t_min;
  if (t_total + 1e-5 < t_min) {
    throw std::invalid_argument("steering change does not fit in the primitive length");
  }
  if (t_total <= kSteerEps) {
    return;
  }
  const double speed = limits.reference_speed;
  const double v = reverse ? -speed : speed;
  const int steps = std::max(
    1, static_cast<int>(std::ceil((speed * t_total) / limits.resolution)));
  const double dt = t_total / static_cast<double>(steps);
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  samples.reserve(static_cast<std::size_t>(steps));
  for (int i = 1; i <= steps; ++i) {
    const double t_mid = (static_cast<double>(i) - 0.5) * dt;
    const double delta_mid = steerAt(delta0, delta1, t_mid, t_min, limits.max_rate, limits.max_accel);
    const double kappa = std::tan(delta_mid) / limits.wheelbase;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw += v * kappa * dt;
    const double delta_end = steerAt(
      delta0, delta1, static_cast<double>(i) * dt, t_min, limits.max_rate, limits.max_accel);
    BodySample sample;
    sample.x = static_cast<float>(x);
    sample.y = static_cast<float>(y);
    sample.yaw = static_cast<float>(yaw);
    sample.delta = static_cast<float>(delta_end);
    sample.kappa = static_cast<float>(std::tan(delta_end) / limits.wheelbase);
    samples.push_back(sample);
  }
}

bool curvatureCacheMatches(
  const CurvatureModel & model, const CurvatureLimits & limits, bool allow_reverse)
{
  return model.enabled &&
         model.cached_reverse == allow_reverse &&
         std::abs(model.cached_resolution - limits.resolution) < 1e-9 &&
         std::abs(model.cached_length - limits.primitive_length) < 1e-9 &&
         std::abs(model.cached_speed - limits.reference_speed) < 1e-9 &&
         std::abs(model.cached_rate - limits.max_rate) < 1e-9 &&
         std::abs(model.cached_accel - limits.max_accel) < 1e-9 &&
         std::abs(model.cached_wheelbase - limits.wheelbase) < 1e-9 &&
         std::abs(model.cached_max_angle - limits.max_angle) < 1e-9;
}

void buildCurvatureModel(
  const CurvatureLimits & limits, bool allow_reverse, CurvatureModel & model)
{
  if (curvatureCacheMatches(model, limits, allow_reverse)) {
    return;
  }
  model.bins = buildCurvatureBins(limits);
  const int n = static_cast<int>(model.bins.angles.size());
  model.forward.assign(n, {});
  model.reverse_prims.assign(n, {});
  model.short_forward.assign(n, {});
  model.short_reverse.assign(n, {});
  const int short_count = std::max(
    1, static_cast<int>(std::floor(limits.primitive_length / limits.resolution)));
  for (int bin = 0; bin < n; ++bin) {
    for (int step = -1; step <= 1; ++step) {
      const int next = bin + step;
      if (next < 0 || next >= n) {
        continue;
      }
      const double ddelta = std::abs(model.bins.angles[next] - model.bins.angles[bin]);
      if (ddelta > model.bins.delta_max + kSteerEps) {
        continue;
      }
      model.forward[bin].push_back(
        makePrimitive(bin, next, limits.primitive_length, true, false, model.bins, limits));
      if (allow_reverse) {
        model.reverse_prims[bin].push_back(
          makePrimitive(bin, next, limits.primitive_length, true, true, model.bins, limits));
      }
    }
    for (int k = 1; k <= short_count; ++k) {
      const double length = limits.resolution * static_cast<double>(k);
      if (length >= limits.primitive_length - 1e-6) {
        break;
      }
      model.short_forward[bin].push_back(
        makePrimitive(bin, bin, length, true, false, model.bins, limits));
      if (allow_reverse) {
        model.short_reverse[bin].push_back(
          makePrimitive(bin, bin, length, true, true, model.bins, limits));
      }
    }
  }
  model.enabled = true;
  model.limits = limits;
  model.cached_resolution = limits.resolution;
  model.cached_length = limits.primitive_length;
  model.cached_speed = limits.reference_speed;
  model.cached_rate = limits.max_rate;
  model.cached_accel = limits.max_accel;
  model.cached_wheelbase = limits.wheelbase;
  model.cached_max_angle = limits.max_angle;
  model.cached_reverse = allow_reverse;
}

}  // namespace nav2_smac_planner
