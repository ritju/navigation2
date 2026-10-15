// Copyright (c) 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "nav2_planner/fast_path_planner.hpp"
#include "nav2_planner/planning_debug_viz.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_planner
{

namespace
{

const char * reasonToString(const FastPlanReason reason)
{
  switch (reason) {
    case FastPlanReason::StraightOk:
      return "StraightOk";
    case FastPlanReason::NeedAstar:
      return "NeedAstar";
    case FastPlanReason::GoalUnreachable:
      return "GoalUnreachable";
  }
  return "Unknown";
}

double normalizeAngle(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/** 射线法判断点是否在多边形内。 */
bool pointInPolygon(const geometry_msgs::msg::Polygon & polygon, double x, double y)
{
  if (polygon.points.size() < 3) {
    return false;
  }
  bool inside = false;
  size_t j = polygon.points.size() - 1;
  for (size_t i = 0; i < polygon.points.size(); ++i) {
    const auto & pi = polygon.points[i];
    const auto & pj = polygon.points[j];
    const bool intersect =
      ((pi.y > y) != (pj.y > y)) &&
      (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-9) + pi.x);
    if (intersect) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

double getDoubleParam(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & name,
  double fallback)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).as_double();
  }
  return fallback;
}

bool getBoolParam(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & name,
  bool fallback)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).as_bool();
  }
  return fallback;
}

int getIntParam(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & name,
  int fallback)
{
  if (node->has_parameter(name)) {
    return static_cast<int>(node->get_parameter(name).as_int());
  }
  return fallback;
}

}  // namespace

FastPathPlanner::FastPathPlanner()
: logger_(rclcpp::get_logger("FastPathPlanner"))
{
}

void FastPathPlanner::configure(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
  const std::shared_ptr<FootprintChecker> & footprint_checker,
  const std::vector<std::string> & planner_ids)
{
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  footprint_checker_ = footprint_checker;
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  _goal_occupied_tolerance = getDoubleParam(node, "goal_occupied_tolerance", 0.5);
  _goal_search_resolution = getDoubleParam(node, "goal_search_resolution", 0.1);
  _straight_check_length_ratio = getDoubleParam(node, "straight_check_length_ratio", 0.5);
  _straight_path_resolution = getDoubleParam(node, "straight_path_resolution", 0.1);
  enable_line_stretch_ = getBoolParam(node, "enable_line_stretch", true);
  line_stretch_max_ = getDoubleParam(node, "line_stretch_max", 0.4);
  line_stretch_goal_window_ = getDoubleParam(node, "line_stretch_goal_window", 0.8);
  line_stretch_allow_extend_ = getBoolParam(node, "line_stretch_allow_extend", false);
  enable_line_rotate_ = getBoolParam(node, "enable_line_rotate", true);
  line_rotate_max_iters_ = getIntParam(node, "line_rotate_max_iters", 5);
  line_rotate_goal_shift_tol_ = getDoubleParam(node, "line_rotate_goal_shift_tol", 0.5);
  corridor_intrusion_tol_ = std::max(
    0.0, getDoubleParam(node, "corridor_intrusion_tol", 0.08));
  corner_sweep_scale_ = std::max(0.05, getDoubleParam(node, "corner_sweep_scale", 1.2));
  corner_snap_tolerance_ = std::max(0.0, getDoubleParam(node, "corner_snap_tolerance", 1.5));
  corner_snap_enable_ = getBoolParam(node, "corner_snap_enable", true);

  using std::placeholders::_1;
  narrow_passages_sub_ = node->create_subscription<garage_utils_msgs::msg::Polygons>(
    "/narrow_passages",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&FastPathPlanner::narrowPassagesCallback, this, _1));
  enable_backward_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/enable_backward",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&FastPathPlanner::enableBackwardCallback, this, _1));

  RCLCPP_INFO(
    logger_,
    "[FastPath] configured: goal_occupied_tolerance=%.2f "
    "goal_search_resolution=%.2f "
    "straight_check_length_ratio=%.2f straight_path_resolution=%.2f "
    "stretch=%s rotate=%s corridor_intrusion_tol=%.3f "
    "corner_sweep_scale=%.2f corner_snap_tol=%.2f corner_snap=%s planner_ids=%zu",
    _goal_occupied_tolerance, _goal_search_resolution,
    _straight_check_length_ratio, _straight_path_resolution,
    enable_line_stretch_ ? "true" : "false",
    enable_line_rotate_ ? "true" : "false",
    corridor_intrusion_tol_,
    corner_sweep_scale_, corner_snap_tolerance_,
    corner_snap_enable_ ? "true" : "false",
    planner_ids.size());
}

void FastPathPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "[FastPath] cleanup");
  narrow_passages_sub_.reset();
  enable_backward_sub_.reset();
  debug_viz_.reset();
  footprint_checker_.reset();
  costmap_ros_.reset();
  costmap_ = nullptr;
}

FastPlanResult FastPathPlanner::compute(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const FastPlanOptions & options)
{
  FastPlanResult result;
  result.snapped_goal = goal;
  result.reason = FastPlanReason::NeedAstar;
  result.path.header.stamp = clock_->now();
  result.path.header.frame_id = costmap_ros_->getGlobalFrameID();

  const double approach_yaw = std::atan2(
    goal.pose.position.y - start.pose.position.y,
    goal.pose.position.x - start.pose.position.x);
  if (options.rewrite_goal_yaw_to_approach) {
    result.snapped_goal.pose.orientation = yawToQuaternion(approach_yaw);
  }

  RCLCPP_INFO(
    logger_,
    "[FastPath] compute start=(%.3f, %.3f, yaw=%.3f) goal=(%.3f, %.3f, yaw=%.3f) "
    "approach=%.3f rewrite_yaw=%s straight=%s reverse=%s stretch=%s rotate=%s "
    "footprint_corridor=%s sweep=%s exempt=%s",
    start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation),
    goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation),
    approach_yaw,
    options.rewrite_goal_yaw_to_approach ? "true" : "false",
    options.allow_straight ? "true" : "false",
    options.allow_reverse ? "true" : "false",
    options.allow_stretch ? "true" : "false",
    options.allow_rotate ? "true" : "false",
    options.strict_goal_footprint ? "true" : "false",
    options.use_corner_sweep ? "true" : "false",
    options.allow_intrusion_exempt ? "true" : "false");

  if (debug_viz_) {
    debug_viz_->publishStart(start);
    debug_viz_->publishOriginalGoal(goal);
  }

  if (!costmap_) {
    RCLCPP_ERROR(logger_, "[FastPath] not configured, fallback NeedAstar");
    return result;
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  updateFootprintExtents();

  if (!snapOccupiedGoal(start, result.snapped_goal, options)) {
    if (debug_viz_ && options.use_corner_sweep) {
      double xf = 0.0, xr = 0.0, yl = 0.0, yr = 0.0;
      computeCornerSweepBounds(xf, xr, yl, yr);
      debug_viz_->publishCornerSweep(
        result.snapped_goal, xr, xf, yl, yr, corner_sweep_scale_, options.out_yaw);
    }
    result.reason = FastPlanReason::GoalUnreachable;
    const double snap_r = options.snap_tolerance >= 0.0 ?
      options.snap_tolerance : _goal_occupied_tolerance;
    RCLCPP_WARN(
      logger_,
      "[FastPath] GoalUnreachable: original goal (%.3f, %.3f) occupied (254), "
      "no free pose within tolerance=%.2f res=%.2f",
      goal.pose.position.x, goal.pose.position.y,
      snap_r, _goal_search_resolution);
    return result;
  }

  if (debug_viz_ && options.use_corner_sweep) {
    double xf = 0.0, xr = 0.0, yl = 0.0, yr = 0.0;
    computeCornerSweepBounds(xf, xr, yl, yr);
    debug_viz_->publishCornerSweep(
      result.snapped_goal, xr, xf, yl, yr, corner_sweep_scale_, options.out_yaw);
  }

  if (std::hypot(
      result.snapped_goal.pose.position.x - goal.pose.position.x,
      result.snapped_goal.pose.position.y - goal.pose.position.y) > 1e-4)
  {
    RCLCPP_INFO(
      logger_,
      "[FastPath] using snapped_goal=(%.3f, %.3f) original=(%.3f, %.3f)",
      result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);
  }

  if (!options.allow_straight) {
    result.reason = FastPlanReason::NeedAstar;
    RCLCPP_INFO(
      logger_,
      "[FastPath] skip corridor, NeedAstar snapped_goal=(%.3f, %.3f)",
      result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y);
    return result;
  }

  const geometry_msgs::msg::PoseStamped original_goal = result.snapped_goal;
  auto finish_straight = [&](const char * kind) {
    const double line_yaw = std::atan2(
      result.snapped_goal.pose.position.y - start.pose.position.y,
      result.snapped_goal.pose.position.x - start.pose.position.x);
    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double rev_yaw = normalizeAngle(line_yaw + M_PI);
    const bool prefer_reverse = options.allow_reverse &&
      std::fabs(normalizeAngle(start_yaw - rev_yaw)) <
      std::fabs(normalizeAngle(start_yaw - line_yaw));
    const double heading = prefer_reverse ? rev_yaw : line_yaw;
    result.path = buildStraightPath(start, result.snapped_goal, heading);
    result.snapped_goal.pose.orientation = yawToQuaternion(line_yaw);
    result.reason = FastPlanReason::StraightOk;
    RCLCPP_INFO(
      logger_,
      "[FastPath] StraightOk kind=%s reverse=%s poses=%zu goal=(%.3f, %.3f)",
      kind, prefer_reverse ? "true" : "false", result.path.poses.size(),
      result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y);
    if (debug_viz_) {
      debug_viz_->publishStraightPath(result.path);
      if (std::string(kind) == "rotate") {
        debug_viz_->publishRotatedPath(result.path);
      }
    }
  };

  const bool use_fp = options.strict_goal_footprint;
  const bool allow_exempt = options.allow_intrusion_exempt;
  CorridorHit hit = checkCorridor(start, result.snapped_goal, use_fp, allow_exempt);
  if (debug_viz_) {
    debug_viz_->publishCorridor(
      start, result.snapped_goal, halfWidth(), hit.blocked, hit.pose);
    debug_viz_->publishStraightCandidate(
      start, result.snapped_goal, 0, use_fp ? "footprint_corridor" : "corridor");
  }

  if (!hit.blocked) {
    finish_straight(use_fp ? "footprint_corridor" : "corridor");
    return result;
  }

  RCLCPP_INFO(
    logger_,
    "[FastPath] %s blocked s=%.3f / %.3f dy=%.3f both_sides=%s "
    "intrusion=%.3f clearance_ok=%s free=%.3f",
    use_fp ? "footprint_corridor" : "halfwidth_corridor",
    hit.s, hit.L, hit.dy, hit.both_sides ? "true" : "false",
    hit.intrusion, hit.clearance_ok ? "true" : "false", hit.clearance_len);
  if (debug_viz_) {
    debug_viz_->publishCollision(hit.pose, 0.0, hit.dy, hit.s, hit.L);
  }

  const bool can_stretch =
    options.allow_stretch && enable_line_stretch_ && !options.use_corner_sweep;
  if (can_stretch && tryStretchGoal(start, original_goal, result.snapped_goal, hit)) {
    hit = checkCorridor(start, result.snapped_goal, use_fp, allow_exempt);
    if (debug_viz_) {
      debug_viz_->publishCorridor(
        start, result.snapped_goal, halfWidth(), hit.blocked, hit.pose);
      debug_viz_->publishStraightCandidate(start, result.snapped_goal, 1, "stretch");
      debug_viz_->publishAdjustedGoal(result.snapped_goal, "stretch");
    }
    if (!hit.blocked) {
      finish_straight("stretch");
      return result;
    }
    if (debug_viz_) {
      debug_viz_->publishCollision(hit.pose, 0.0, hit.dy, hit.s, hit.L);
    }
  }

  const bool can_rotate = options.allow_rotate && enable_line_rotate_;
  if (can_rotate && tryRotateGoal(
      start, original_goal, result.snapped_goal, hit, use_fp, allow_exempt))
  {
    finish_straight("rotate");
    return result;
  }

  result.reason = FastPlanReason::NeedAstar;
  result.snapped_goal = original_goal;
  if (options.rewrite_goal_yaw_to_approach) {
    result.snapped_goal.pose.orientation = yawToQuaternion(
      std::atan2(
        original_goal.pose.position.y - start.pose.position.y,
        original_goal.pose.position.x - start.pose.position.x));
  }
  RCLCPP_INFO(
    logger_,
    "[FastPath] corridor failed, NeedAstar reason=%s snapped_goal=(%.3f, %.3f) "
    "(snap only, stretch/rotate discarded) tried_stretch=%s tried_rotate=%s",
    reasonToString(result.reason),
    result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y,
    can_stretch ? "true" : "false",
    can_rotate ? "true" : "false");
  return result;
}

void FastPathPlanner::enableBackwardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(enable_backward_mutex_);
  enable_backward_cmd_ = msg->data;
  enable_backward_cmd_received_ = true;
  RCLCPP_INFO(
    logger_,
    "[FastPath] /enable_backward=%s（允许倒车=%s，前进直线失败后可试后退直线）",
    msg->data ? "true" : "false",
    msg->data ? "true" : "false");
}

void FastPathPlanner::narrowPassagesCallback(
  const garage_utils_msgs::msg::Polygons::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  narrow_polygons_received_ = true;
  narrow_polygons_ = msg->polygons;
  if (narrow_polygons_.empty()) {
    latched_narrow_passage_ = false;
    RCLCPP_WARN(logger_, "[FastPath] 狭窄通道: 收到空的 /narrow_passages，Latch 清除");
  } else {
    RCLCPP_INFO(
      logger_,
      "[FastPath] 狭窄通道: 更新 /narrow_passages，多边形数量=%zu",
      narrow_polygons_.size());
  }
}

bool FastPathPlanner::isBackwardActive() const
{
  std::lock_guard<std::mutex> lock(enable_backward_mutex_);
  return enable_backward_cmd_received_ && enable_backward_cmd_;
}

bool FastPathPlanner::narrowPolygonsAvailable() const
{
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  return narrow_polygons_received_ && !narrow_polygons_.empty();
}

bool FastPathPlanner::isPointInNarrowPassage(const double x, const double y) const
{
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  for (const auto & polygon : narrow_polygons_) {
    if (pointInPolygon(polygon, x, y)) {
      return true;
    }
  }
  return false;
}

bool FastPathPlanner::isFootprintFullyOutsideNarrowPassages(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  std::lock_guard<std::mutex> lock(narrow_polygons_mutex_);
  if (!narrow_polygons_received_ || narrow_polygons_.empty()) {
    return true;
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
  for (const auto & pt : footprint) {
    const double wx = pose.pose.position.x + pt.x * cos_y - pt.y * sin_y;
    const double wy = pose.pose.position.y + pt.x * sin_y + pt.y * cos_y;
    for (const auto & polygon : narrow_polygons_) {
      if (pointInPolygon(polygon, wx, wy)) {
        return false;
      }
    }
  }
  return true;
}

void FastPathPlanner::updateNarrowPassageLatch(const geometry_msgs::msg::PoseStamped & start)
{
  const bool prev_latched = latched_narrow_passage_;
  if (!narrowPolygonsAvailable()) {
    latched_narrow_passage_ = false;
    if (prev_latched) {
      RCLCPP_INFO(logger_, "[FastPath] 狭窄通道: 无有效多边形，Latch 由 true 置 false");
    }
    return;
  }
  if (isPointInNarrowPassage(start.pose.position.x, start.pose.position.y)) {
    latched_narrow_passage_ = true;
    if (!prev_latched) {
      RCLCPP_INFO(
        logger_,
        "[FastPath] 狭窄通道: base_link (%.2f, %.2f) 进入窄通道，Latch 置 true",
        start.pose.position.x, start.pose.position.y);
    }
    return;
  }
  if (isFootprintFullyOutsideNarrowPassages(start)) {
    latched_narrow_passage_ = false;
    if (prev_latched) {
      RCLCPP_INFO(logger_, "[FastPath] 狭窄通道: footprint 完全离开窄通道，Latch 由 true 置 false");
    }
  }
}

bool FastPathPlanner::isNarrowActive(
  const geometry_msgs::msg::PoseStamped & /*start*/,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!narrowPolygonsAvailable()) {
    return false;
  }
  const bool goal_in = isPointInNarrowPassage(goal.pose.position.x, goal.pose.position.y);
  const bool active = latched_narrow_passage_ || goal_in;
  RCLCPP_DEBUG(
    logger_,
    "[FastPath] narrow_active=%s latched=%s goal_in=%s goal=(%.2f, %.2f)",
    active ? "true" : "false",
    latched_narrow_passage_ ? "true" : "false",
    goal_in ? "true" : "false",
    goal.pose.position.x, goal.pose.position.y);
  return active;
}

void FastPathPlanner::updateFootprintExtents()
{
  footprint_back_x_ = 0.0;
  footprint_front_x_ = 0.0;
  footprint_y_min_ = 0.0;
  footprint_y_max_ = 0.0;
  const nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
  for (const auto & pt : footprint) {
    footprint_back_x_ = std::min(footprint_back_x_, static_cast<double>(pt.x));
    footprint_front_x_ = std::max(footprint_front_x_, static_cast<double>(pt.x));
    footprint_y_min_ = std::min(footprint_y_min_, static_cast<double>(pt.y));
    footprint_y_max_ = std::max(footprint_y_max_, static_cast<double>(pt.y));
  }
}

void FastPathPlanner::computeCornerSweepBounds(
  double & x_front, double & x_rear, double & y_left, double & y_right) const
{
  const nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
  bool have_fl = false;
  bool have_fr = false;
  bool have_rl = false;
  bool have_rr = false;
  double xf_l = 0.0, yf_l = 0.0;
  double xf_r = 0.0, yf_r = 0.0;
  double xb_l = 0.0, yb_l = 0.0;
  double xb_r = 0.0, yb_r = 0.0;
  for (const auto & pt : footprint) {
    const double px = static_cast<double>(pt.x);
    const double py = static_cast<double>(pt.y);
    if (py >= 0.0) {
      if (!have_fl || px > xf_l) {
        xf_l = px;
        yf_l = py;
        have_fl = true;
      }
      if (!have_rl || px < xb_l) {
        xb_l = px;
        yb_l = py;
        have_rl = true;
      }
    } else {
      if (!have_fr || px > xf_r) {
        xf_r = px;
        yf_r = py;
        have_fr = true;
      }
      if (!have_rr || px < xb_r) {
        xb_r = px;
        yb_r = py;
        have_rr = true;
      }
    }
  }
  const double r_fl = have_fl ? std::hypot(xf_l, yf_l) : 0.0;
  const double r_fr = have_fr ? std::hypot(xf_r, yf_r) : 0.0;
  const double r_rl = have_rl ? std::hypot(xb_l, yb_l) : 0.0;
  const double r_rr = have_rr ? std::hypot(xb_r, yb_r) : 0.0;
  const double k = corner_sweep_scale_;
  x_front = k * std::max(r_fl, r_fr);
  if (x_front < footprint_front_x_) {
    x_front = footprint_front_x_;
  }
  x_rear = footprint_back_x_;
  y_left = k * (have_rl ? r_rl : std::max(0.0, footprint_y_max_));
  y_right = k * (have_rr ? r_rr : std::max(0.0, -footprint_y_min_));
}

double FastPathPlanner::straightCheckStep() const
{
  const double resolution = costmap_->getResolution();
  const double robot_length = std::max(0.0, footprint_front_x_ - footprint_back_x_);
  const double ratio = std::clamp(_straight_check_length_ratio, 0.05, 1.0);
  if (robot_length < resolution) {
    return resolution;
  }
  return std::max(resolution, robot_length * ratio);
}

double FastPathPlanner::straightPathStep() const
{
  return std::max(_straight_path_resolution, 1e-3);
}

double FastPathPlanner::halfWidth() const
{
  const double hw = std::max(footprint_y_max_, -footprint_y_min_);
  const double res = costmap_ ? costmap_->getResolution() : 0.05;
  return std::max(hw, res);
}

double FastPathPlanner::robotWidth() const
{
  const double res = costmap_ ? costmap_->getResolution() : 0.05;
  return std::max(footprint_y_max_ - footprint_y_min_, res);
}

double FastPathPlanner::inscribedRadius() const
{
  if (!costmap_ros_) {
    return 0.0;
  }
  nav2_costmap_2d::LayeredCostmap * layered = costmap_ros_->getLayeredCostmap();
  if (!layered) {
    return 0.0;
  }
  return layered->getInscribedRadius();
}

bool FastPathPlanner::centerlinePrecheckReliable() const
{
  return inscribedRadius() + 1e-6 >= halfWidth();
}

bool FastPathPlanner::isLethalWorld(double wx, double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;
  }
  return costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE;
}

double FastPathPlanner::measureOppositeFreeLength(
  double px, double py, double nx, double ny, double dy) const
{
  const double res = costmap_->getResolution();
  const double width = robotWidth();
  const double search = width + 2.0 * corridor_intrusion_tol_;
  const double sign = (dy >= 0.0) ? -1.0 : 1.0;
  const double ox = px + dy * nx;
  const double oy = py + dy * ny;
  const double dx = sign * nx;
  const double dyy = sign * ny;

  bool left_lethal = true;
  int n_free = 0;
  for (double t = 0.0; t <= search + 1e-9; t += res) {
    const bool lethal = isLethalWorld(ox + t * dx, oy + t * dyy);
    if (left_lethal) {
      if (lethal) {
        continue;
      }
      left_lethal = false;
    }
    if (lethal) {
      break;
    }
    ++n_free;
  }
  return static_cast<double>(n_free) * res;
}

bool FastPathPlanner::isCrossSectionFree(double x, double y, double yaw) const
{
  const double hw = halfWidth();
  const double res = costmap_->getResolution();
  const double nx = -std::sin(yaw);
  const double ny = std::cos(yaw);
  for (double t = -hw; ; t += res) {
    const double sample_t = std::min(t, hw);
    if (isLethalWorld(x + sample_t * nx, y + sample_t * ny)) {
      return false;
    }
    if (sample_t >= hw - 1e-9) {
      break;
    }
  }
  return true;
}

bool FastPathPlanner::orientedRectHitsLethal(
  double x, double y, double yaw,
  double x0, double x1, double y0, double y1) const
{
  if (x1 < x0) {
    std::swap(x0, x1);
  }
  if (y1 < y0) {
    std::swap(y0, y1);
  }
  const double res = costmap_->getResolution();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (double fx = x0; ; fx += res) {
    const double sample_fx = std::min(fx, x1);
    for (double fy = y0; ; fy += res) {
      const double sample_fy = std::min(fy, y1);
      const double wx = x + sample_fx * c - sample_fy * s;
      const double wy = y + sample_fx * s + sample_fy * c;
      if (isLethalWorld(wx, wy)) {
        return true;
      }
      if (sample_fy >= y1 - 1e-9) {
        break;
      }
    }
    if (sample_fx >= x1 - 1e-9) {
      break;
    }
  }
  return false;
}

bool FastPathPlanner::cornerSweepHitsLethal(double x, double y, double yaw) const
{
  double xf = 0.0, xr = 0.0, yl = 0.0, yr = 0.0;
  computeCornerSweepBounds(xf, xr, yl, yr);
  return orientedRectHitsLethal(x, y, yaw, xr, xf, -yr, yl);
}

bool FastPathPlanner::poseFreeForSnap(
  const FastPlanOptions & options,
  double x, double y, double in_yaw) const
{
  if (options.use_corner_sweep) {
    if (cornerSweepHitsLethal(x, y, in_yaw)) {
      return false;
    }
    if (std::isfinite(options.out_yaw) &&
      cornerSweepHitsLethal(x, y, options.out_yaw))
    {
      return false;
    }
    return true;
  }
  if (options.strict_goal_footprint) {
    double fx = 0.0, fy = 0.0, wx = 0.0, wy = 0.0;
    return !footprintHitsLethal(x, y, in_yaw, 0.0, false, fx, fy, wx, wy);
  }
  return isCrossSectionFree(x, y, in_yaw);
}

bool FastPathPlanner::tailWindowHitsLethal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const FastPlanOptions & options) const
{
  const double ax = start.pose.position.x;
  const double ay = start.pose.position.y;
  const double bx = goal.pose.position.x;
  const double by = goal.pose.position.y;
  const double L = std::hypot(bx - ax, by - ay);
  if (L < 1e-3) {
    return false;
  }
  const double ux = (bx - ax) / L;
  const double uy = (by - ay) / L;
  const double yaw = std::atan2(by - ay, bx - ax);
  const double robot_len = std::max(0.0, footprint_front_x_ - footprint_back_x_);
  const double window = std::max(robot_len, 1e-3);
  const double res = costmap_->getResolution();
  const double s0 = std::max(res, L - window);
  for (double s = s0; ; s += res) {
    const double sample_s = std::min(s, L);
    if (poseFreeForSnap(options, ax + ux * sample_s, ay + uy * sample_s, yaw) == false) {
      return true;
    }
    if (sample_s >= L - 1e-9) {
      break;
    }
  }
  return false;
}

bool FastPathPlanner::footprintHitsLethal(
  double x, double y, double yaw,
  double path_s,
  bool skip_behind_start,
  double & hit_fx,
  double & hit_fy,
  double & hit_wx,
  double & hit_wy) const
{
  const double x0 = footprint_back_x_;
  const double x1 = footprint_front_x_;
  const double y0 = footprint_y_min_;
  const double y1 = footprint_y_max_;
  const double res = costmap_->getResolution();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (double fx = x0; ; fx += res) {
    const double sample_fx = std::min(fx, x1);
    if (skip_behind_start && path_s + sample_fx < -1e-4) {
      if (sample_fx >= x1 - 1e-9) {
        break;
      }
      continue;
    }
    for (double fy = y0; ; fy += res) {
      const double sample_fy = std::min(fy, y1);
      const double wx = x + sample_fx * c - sample_fy * s;
      const double wy = y + sample_fx * s + sample_fy * c;
      if (isLethalWorld(wx, wy)) {
        hit_fx = sample_fx;
        hit_fy = sample_fy;
        hit_wx = wx;
        hit_wy = wy;
        return true;
      }
      if (sample_fy >= y1 - 1e-9) {
        break;
      }
    }
    if (sample_fx >= x1 - 1e-9) {
      break;
    }
  }
  return false;
}

bool FastPathPlanner::findCenterlineTrigger(
  double ax, double ay, double ux, double uy,
  double s_begin, double L,
  double & s_out, unsigned char & cost_out) const
{
  if (s_begin > L + 1e-9) {
    return false;
  }
  const double res = costmap_->getResolution();
  for (double s = s_begin; ; s += res) {
    const double sample_s = std::min(s, L);
    const double wx = ax + ux * sample_s;
    const double wy = ay + uy * sample_s;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      s_out = sample_s;
      cost_out = nav2_costmap_2d::LETHAL_OBSTACLE;
      return true;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      s_out = sample_s;
      cost_out = cost;
      return true;
    }
    if (sample_s >= L - 1e-9) {
      break;
    }
  }
  return false;
}

FastPathPlanner::CorridorHit FastPathPlanner::scanHalfWidthBand(
  const geometry_msgs::msg::PoseStamped & start,
  double ax, double ay, double ux, double uy, double yaw, double L,
  double s_begin, double s_end,
  bool allow_exempt,
  bool log_accept) const
{
  CorridorHit hit;
  hit.L = L;
  if (s_end < s_begin - 1e-9) {
    return hit;
  }
  const double hw = halfWidth();
  const double width = robotWidth();
  const double res = costmap_->getResolution();
  const double nx = -std::sin(yaw);
  const double ny = std::cos(yaw);
  // s=0 是当前位姿截面：边线贴 254 不否决（机器人已在 S），否则转线永远清不掉起点。
  const double s0 = (s_begin <= 1e-9) ? res : s_begin;
  if (s_end < s0 - 1e-9) {
    return hit;
  }

  CorridorHit rotate_hit;
  bool have_rotate = false;
  int n_exempt = 0;

  auto fill_pose = [&](CorridorHit & out, double sample_s, double px, double py) {
    out.L = L;
    out.s = sample_s;
    out.pose.header = start.header;
    out.pose.pose.position.x = px;
    out.pose.pose.position.y = py;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yawToQuaternion(yaw);
  };

  for (double s = s0; ; s += res) {
    const double sample_s = std::min(s, s_end);
    const double px = ax + ux * sample_s;
    const double py = ay + uy * sample_s;
    bool slice_hit = false;
    bool left = false;
    bool right = false;
    double best_abs_dy = std::numeric_limits<double>::max();
    double best_dy = 0.0;

    for (double t = -hw; ; t += res) {
      const double sample_t = std::min(t, hw);
      const double wx = px + sample_t * nx;
      const double wy = py + sample_t * ny;
      if (isLethalWorld(wx, wy)) {
        slice_hit = true;
        if (sample_t > 0.5 * res) {
          left = true;
        } else if (sample_t < -0.5 * res) {
          right = true;
        }
        const double abs_dy = std::fabs(sample_t);
        if (abs_dy < best_abs_dy) {
          best_abs_dy = abs_dy;
          best_dy = sample_t;
        }
      }
      if (sample_t >= hw - 1e-9) {
        break;
      }
    }

    if (slice_hit) {
      const bool centerline = std::fabs(best_dy) <= 0.5 * res;
      const bool both = left && right;
      CorridorHit cur;
      cur.blocked = true;
      cur.both_sides = both;
      cur.dy = best_dy;
      cur.intrusion = std::max(0.0, hw - std::fabs(best_dy));
      fill_pose(cur, sample_s, px, py);

      if (both || centerline) {
        cur.n_exempt = n_exempt;
        RCLCPP_INFO(
          logger_,
          "[FastPath] corridor hard block s=%.3f dy=%.3f both=%s centerline=%s",
          sample_s, best_dy, both ? "true" : "false",
          centerline ? "true" : "false");
        return cur;
      }

      if (!allow_exempt) {
        cur.n_exempt = n_exempt;
        RCLCPP_INFO(
          logger_,
          "[FastPath] corridor 254 (no exempt) s=%.3f dy=%.3f",
          sample_s, best_dy);
        return cur;
      }

      cur.clearance_len = measureOppositeFreeLength(px, py, nx, ny, best_dy);
      cur.clearance_ok = cur.clearance_len + 1e-9 >= width;
      if (!cur.clearance_ok) {
        cur.n_exempt = n_exempt;
        RCLCPP_INFO(
          logger_,
          "[FastPath] corridor clearance fail s=%.3f dy=%.3f intrusion=%.3f "
          "free=%.3f width=%.3f (no rotate)",
          sample_s, best_dy, cur.intrusion, cur.clearance_len, width);
        return cur;
      }

      if (cur.intrusion <= corridor_intrusion_tol_ + 1e-9) {
        ++n_exempt;
        RCLCPP_DEBUG(
          logger_,
          "[FastPath] corridor exempt s=%.3f dy=%.3f intrusion=%.3f<=%.3f "
          "free=%.3f",
          sample_s, best_dy, cur.intrusion, corridor_intrusion_tol_,
          cur.clearance_len);
      } else if (!have_rotate || cur.intrusion > rotate_hit.intrusion) {
        rotate_hit = cur;
        have_rotate = true;
      }
    }
    if (sample_s >= s_end - 1e-9) {
      break;
    }
  }

  if (have_rotate) {
    rotate_hit.n_exempt = n_exempt;
    RCLCPP_INFO(
      logger_,
      "[FastPath] corridor rotate candidate s=%.3f dy=%.3f intrusion=%.3f "
      "free=%.3f exempted=%d",
      rotate_hit.s, rotate_hit.dy, rotate_hit.intrusion,
      rotate_hit.clearance_len, n_exempt);
    return rotate_hit;
  }

  hit.n_exempt = n_exempt;
  if (log_accept && n_exempt > 0) {
    RCLCPP_INFO(
      logger_,
      "[FastPath] corridor accepted with %d exempted same-side 254 slice(s) "
      "intrusion_tol=%.3f",
      n_exempt, corridor_intrusion_tol_);
  }
  return hit;
}

FastPathPlanner::CorridorHit FastPathPlanner::checkBandCorridor(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double front_overhang,
  bool allow_exempt) const
{
  CorridorHit hit;
  const double ax = start.pose.position.x;
  const double ay = start.pose.position.y;
  const double bx = goal.pose.position.x;
  const double by = goal.pose.position.y;
  hit.L = std::hypot(bx - ax, by - ay);
  const double yaw = std::atan2(by - ay, bx - ax);
  const double ux = (hit.L > 1e-6) ? (bx - ax) / hit.L : 1.0;
  const double uy = (hit.L > 1e-6) ? (by - ay) / hit.L : 0.0;
  const double front = std::max(0.0, front_overhang);
  const double s_limit = hit.L + front;
  const double res = costmap_->getResolution();

  if (centerlinePrecheckReliable()) {
    const double W = std::max(
      inscribedRadius() + res,
      straightCheckStep());
    double s_cursor = 0.0;
    double scanned_until = -1.0;
    int n_exempt_all = 0;
    double s_trig = 0.0;
    unsigned char trig_cost = 0;

    while (findCenterlineTrigger(
        ax, ay, ux, uy, s_cursor, hit.L, s_trig, trig_cost))
    {
      if (trig_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        hit.blocked = true;
        hit.s = s_trig;
        hit.dy = 0.0;
        hit.intrusion = halfWidth();
        hit.n_exempt = n_exempt_all;
        hit.pose.header = start.header;
        hit.pose.pose.position.x = ax + ux * s_trig;
        hit.pose.pose.position.y = ay + uy * s_trig;
        hit.pose.pose.position.z = 0.0;
        hit.pose.pose.orientation = yawToQuaternion(yaw);
        RCLCPP_INFO(
          logger_,
          "[FastPath] corridor hard block centerline 254 s=%.3f / %.3f",
          s_trig, hit.L);
        return hit;
      }

      // 253：已在先前窗口内扫过半宽则不再重复开窗，但中线 254 上面已先判过。
      if (s_trig <= scanned_until + 1e-9) {
        s_cursor = s_trig + res;
        continue;
      }

      const double s0 = std::max(0.0, s_trig - W);
      double s1 = std::min(s_limit, s_trig + W);
      if (front > 1e-6 && s_trig + W >= hit.L) {
        s1 = s_limit;
      }
      RCLCPP_DEBUG(
        logger_,
        "[FastPath] centerline 253 s=%.3f / %.3f window=[%.3f, %.3f] front=%.3f",
        s_trig, hit.L, s0, s1, front);
      CorridorHit deep = scanHalfWidthBand(
        start, ax, ay, ux, uy, yaw, hit.L, s0, s1, allow_exempt, false);
      if (deep.blocked) {
        deep.n_exempt += n_exempt_all;
        return deep;
      }
      n_exempt_all += deep.n_exempt;
      scanned_until = s1;
      s_cursor = s_trig + res;
    }

    if (front > 1e-6 && scanned_until + 1e-9 < s_limit) {
      const double nose0 = std::max(hit.L, scanned_until);
      CorridorHit nose = scanHalfWidthBand(
        start, ax, ay, ux, uy, yaw, hit.L, nose0, s_limit, allow_exempt, false);
      if (nose.blocked) {
        nose.n_exempt += n_exempt_all;
        return nose;
      }
      n_exempt_all += nose.n_exempt;
    }

    if (n_exempt_all > 0) {
      RCLCPP_INFO(
        logger_,
        "[FastPath] corridor accepted with %d exempted same-side 254 slice(s) "
        "intrusion_tol=%.3f",
        n_exempt_all, corridor_intrusion_tol_);
    }
    hit.n_exempt = n_exempt_all;
    return hit;
  }

  RCLCPP_DEBUG(
    logger_,
    "[FastPath] inscribed=%.3f < halfWidth=%.3f, full band [0, %.3f]",
    inscribedRadius(), halfWidth(), s_limit);
  return scanHalfWidthBand(
    start, ax, ay, ux, uy, yaw, hit.L, 0.0, s_limit, allow_exempt);
}

FastPathPlanner::CorridorHit FastPathPlanner::checkCorridor(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool use_footprint_corridor,
  bool allow_exempt) const
{
  if (use_footprint_corridor) {
    return checkBandCorridor(start, goal, footprint_front_x_, allow_exempt);
  }
  return checkBandCorridor(start, goal, 0.0, allow_exempt);
}

FastPathPlanner::CorridorHit FastPathPlanner::checkFootprintCorridor(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return checkBandCorridor(start, goal, footprint_front_x_, true);
}

FastPathPlanner::CorridorHit FastPathPlanner::checkHalfWidthCorridor(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return checkBandCorridor(start, goal, 0.0, true);
}

nav_msgs::msg::Path FastPathPlanner::buildStraightPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double heading) const
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();

  geometry_msgs::msg::Pose2D start_pose2d;
  start_pose2d.x = start.pose.position.x;
  start_pose2d.y = start.pose.position.y;
  geometry_msgs::msg::Pose2D goal_pose2d;
  goal_pose2d.x = goal.pose.position.x;
  goal_pose2d.y = goal.pose.position.y;
  const double distance = nav2_util::geometry_utils::euclidean_distance(start_pose2d, goal_pose2d);
  const double path_step = straightPathStep();

  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = yawToQuaternion(heading);
  for (double d = path_step; d < distance - 1e-6; d += path_step) {
    geometry_msgs::msg::Pose2D path_pose;
    findPose(start_pose2d, goal_pose2d, d, path_pose);
    pose.pose.position.x = path_pose.x;
    pose.pose.position.y = path_pose.y;
    plan.poses.emplace_back(pose);
  }
  pose.pose.position = goal.pose.position;
  plan.poses.emplace_back(pose);
  return plan;
}

bool FastPathPlanner::tryStretchGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & original_goal,
  geometry_msgs::msg::PoseStamped & goal,
  const CorridorHit & hit)
{
  if (!hit.blocked || hit.L < 1e-3) {
    return false;
  }
  double s_for_stretch = hit.s;
  if (hit.s > hit.L + 1e-6) {
    s_for_stretch = hit.s - std::max(0.0, footprint_front_x_);
  }
  if (s_for_stretch <= hit.L - line_stretch_goal_window_) {
    RCLCPP_INFO(
      logger_,
      "[LineStretch] skip: hit in mid-segment s=%.3f L=%.3f window=%.3f",
      hit.s, hit.L, line_stretch_goal_window_);
    return false;
  }

  const double ux = (goal.pose.position.x - start.pose.position.x) / hit.L;
  const double uy = (goal.pose.position.y - start.pose.position.y) / hit.L;
  const double res = costmap_->getResolution();
  const double new_L = std::max(2.0 * res, s_for_stretch - res);
  geometry_msgs::msg::PoseStamped stretched = goal;
  stretched.pose.position.x = start.pose.position.x + ux * new_L;
  stretched.pose.position.y = start.pose.position.y + uy * new_L;
  stretched.pose.orientation = yawToQuaternion(std::atan2(uy, ux));

  const double shift = std::hypot(
    stretched.pose.position.x - original_goal.pose.position.x,
    stretched.pose.position.y - original_goal.pose.position.y);
  if (shift > line_stretch_max_) {
    RCLCPP_INFO(
      logger_,
      "[LineStretch] skip: |G'-G|=%.3f > max=%.3f",
      shift, line_stretch_max_);
    return false;
  }
  if (new_L >= hit.L - 1e-6) {
    if (!line_stretch_allow_extend_) {
      return false;
    }
  }

  RCLCPP_INFO(
    logger_,
    "[LineStretch] G (%.3f, %.3f) -> (%.3f, %.3f) shift=%.3f new_L=%.3f",
    goal.pose.position.x, goal.pose.position.y,
    stretched.pose.position.x, stretched.pose.position.y, shift, new_L);
  goal = stretched;
  return true;
}

bool FastPathPlanner::tryRotateGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & original_goal,
  geometry_msgs::msg::PoseStamped & goal,
  CorridorHit & hit,
  bool use_footprint_corridor,
  bool allow_exempt)
{
  if (!hit.blocked || hit.both_sides || !hit.clearance_ok) {
    RCLCPP_INFO(
      logger_,
      "[LineRotate] skip: blocked=%s both_sides=%s clearance_ok=%s "
      "(rotate only when same-side and clearance >= width)",
      hit.blocked ? "true" : "false",
      hit.both_sides ? "true" : "false",
      hit.clearance_ok ? "true" : "false");
    return false;
  }

  const geometry_msgs::msg::PoseStamped goal_on_entry = goal;
  const double res = costmap_->getResolution();
  if (std::fabs(hit.dy) <= 0.5 * res) {
    RCLCPP_INFO(
      logger_,
      "[LineRotate] skip: centerline 254 dy=%.3f, rotate cannot clear",
      hit.dy);
    return false;
  }

  const double hw = halfWidth();
  const double side = (hit.dy >= 0.0) ? -1.0 : 1.0;
  double accumulated = 0.0;
  const double L0 = std::hypot(
    original_goal.pose.position.x - start.pose.position.x,
    original_goal.pose.position.y - start.pose.position.y);

  for (int iter = 1; iter <= line_rotate_max_iters_; ++iter) {
    const double s_min = std::max(straightCheckStep(), 4.0 * res);
    const double s_arm = std::max(hit.s, s_min);
    const double need = std::max(hw + res - std::fabs(hit.dy), res);
    const double d_alpha = need / s_arm;
    accumulated += side * d_alpha;
    const double base_yaw = std::atan2(
      original_goal.pose.position.y - start.pose.position.y,
      original_goal.pose.position.x - start.pose.position.x);
    const double yaw = base_yaw + accumulated;
    const double ux = std::cos(yaw);
    const double uy = std::sin(yaw);
    const double vx = original_goal.pose.position.x - start.pose.position.x;
    const double vy = original_goal.pose.position.y - start.pose.position.y;
    const double proj = vx * ux + vy * uy;
    if (proj < 2.0 * res) {
      RCLCPP_INFO(logger_, "[LineRotate] iter=%d proj behind start, abort", iter);
      goal = goal_on_entry;
      return false;
    }
    geometry_msgs::msg::PoseStamped rotated = original_goal;
    rotated.pose.position.x = start.pose.position.x + ux * proj;
    rotated.pose.position.y = start.pose.position.y + uy * proj;
    rotated.pose.orientation = yawToQuaternion(yaw);
    const double shift = std::hypot(
      rotated.pose.position.x - original_goal.pose.position.x,
      rotated.pose.position.y - original_goal.pose.position.y);
    if (shift > line_rotate_goal_shift_tol_) {
      RCLCPP_INFO(
        logger_,
        "[LineRotate] iter=%d |G'-G|=%.3f > tol=%.3f abort",
        iter, shift, line_rotate_goal_shift_tol_);
      goal = goal_on_entry;
      return false;
    }

    if (debug_viz_) {
      debug_viz_->publishStraightCandidate(start, rotated, iter, "rotate");
    }

    CorridorHit again = checkCorridor(
      start, rotated, use_footprint_corridor, allow_exempt);
    RCLCPP_INFO(
      logger_,
      "[LineRotate] iter=%d d_alpha=%.4f s_arm=%.3f need=%.3f alpha=%.3f "
      "shift=%.3f blocked=%s dy=%.3f",
      iter, d_alpha, s_arm, need, accumulated, shift,
      again.blocked ? "true" : "false", again.dy);
    if (!again.blocked) {
      if (debug_viz_) {
        debug_viz_->publishCorridor(start, rotated, halfWidth(), false, again.pose);
        debug_viz_->publishAdjustedGoal(rotated, "rotate");
      }
      goal = rotated;
      hit = again;
      (void)L0;
      return true;
    }
    if (again.both_sides || !again.clearance_ok || (again.dy * side > 0.0)) {
      RCLCPP_INFO(
        logger_,
        "[LineRotate] opposite/both/no-clearance at iter=%d, abort "
        "both=%s clearance_ok=%s dy=%.3f",
        iter, again.both_sides ? "true" : "false",
        again.clearance_ok ? "true" : "false", again.dy);
      goal = goal_on_entry;
      return false;
    }
    hit = again;
    if (debug_viz_) {
      debug_viz_->publishCorridor(start, rotated, halfWidth(), true, again.pose);
      debug_viz_->publishCollision(again.pose, 0.0, again.dy, again.s, again.L);
    }
  }
  goal = goal_on_entry;
  return false;
}

bool FastPathPlanner::snapOccupiedGoal(
  const geometry_msgs::msg::PoseStamped & start,
  geometry_msgs::msg::PoseStamped & goal,
  const FastPlanOptions & options)
{
  const double approach_yaw = std::atan2(
    goal.pose.position.y - start.pose.position.y,
    goal.pose.position.x - start.pose.position.x);
  const double plan_yaw = options.rewrite_goal_yaw_to_approach ?
    approach_yaw : tf2::getYaw(goal.pose.orientation);
  if (options.rewrite_goal_yaw_to_approach) {
    goal.pose.orientation = yawToQuaternion(approach_yaw);
  }

  const char * snap_mode = options.use_corner_sweep ? "sweep" :
    (options.strict_goal_footprint ? "footprint" : "halfwidth");
  const double snap_r = options.snap_tolerance >= 0.0 ?
    options.snap_tolerance : _goal_occupied_tolerance;

  bool origin_free = poseFreeForSnap(
    options, goal.pose.position.x, goal.pose.position.y, plan_yaw);
  if (origin_free && (options.use_corner_sweep || options.strict_goal_footprint) &&
    tailWindowHitsLethal(start, goal, options))
  {
    origin_free = false;
    RCLCPP_INFO(
      logger_,
      "[FastPath] goal (%.3f, %.3f) origin free but tail window 254, force snap mode=%s",
      goal.pose.position.x, goal.pose.position.y, snap_mode);
  }
  if (origin_free) {
    RCLCPP_DEBUG(
      logger_,
      "[FastPath] goal (%.3f, %.3f) 254-clear snap_mode=%s, no snap",
      goal.pose.position.x, goal.pose.position.y, snap_mode);
    return true;
  }

  const double res = std::max(_goal_search_resolution, 1e-3);
  const int max_ring = std::max(1, static_cast<int>(std::ceil(snap_r / res)));
  const double ux = std::cos(approach_yaw);
  const double uy = std::sin(approach_yaw);
  double bx = 0.0;
  double by = 0.0;
  if (std::isfinite(options.out_yaw)) {
    bx = std::cos(approach_yaw) + std::cos(options.out_yaw);
    by = std::sin(approach_yaw) + std::sin(options.out_yaw);
    const double bn = std::hypot(bx, by);
    if (bn > 1e-6) {
      bx /= bn;
      by /= bn;
    } else {
      bx = 0.0;
      by = 0.0;
    }
  }
  RCLCPP_INFO(
    logger_,
    "[FastPath] goal (%.3f, %.3f) 254 occupied snap_mode=%s, ring search "
    "tolerance=%.2f res=%.2f k=%.2f",
    goal.pose.position.x, goal.pose.position.y, snap_mode, snap_r, res,
    options.use_corner_sweep ? corner_sweep_scale_ : 0.0);

  const auto original = goal;
  for (int ring = 1; ring <= max_ring; ++ring) {
    bool found_in_ring = false;
    double best_score = std::numeric_limits<double>::max();
    geometry_msgs::msg::PoseStamped best_goal = original;

    auto consider = [&](int ix, int iy) {
      const double dx = static_cast<double>(ix) * res;
      const double dy = static_cast<double>(iy) * res;
      const double dist = std::hypot(dx, dy);
      if (dist > snap_r + 1e-6) {
        return;
      }
      auto search_goal = original;
      search_goal.pose.position.x += dx;
      search_goal.pose.position.y += dy;
      const double along = dx * ux + dy * uy;
      const double cross = ux * dy - uy * dx;
      if (options.rewrite_goal_yaw_to_approach) {
        search_goal.pose.orientation = yawToQuaternion(
          std::atan2(
            search_goal.pose.position.y - start.pose.position.y,
            search_goal.pose.position.x - start.pose.position.x));
      } else {
        search_goal.pose.orientation = yawToQuaternion(plan_yaw);
      }
      const double check_yaw = tf2::getYaw(search_goal.pose.orientation);
      if (!poseFreeForSnap(
          options, search_goal.pose.position.x, search_goal.pose.position.y, check_yaw))
      {
        return;
      }
      found_in_ring = true;
      double score = dist + 0.25 * std::fabs(cross) + 0.05 * std::fabs(along);
      if (options.use_corner_sweep && (bx * bx + by * by) > 1e-12) {
        const double inward = dx * bx + dy * by;
        score += 0.20 * std::max(0.0, -inward);
      }
      if (score < best_score) {
        best_score = score;
        best_goal = search_goal;
      }
    };

    for (int ix = -ring; ix <= ring; ++ix) {
      consider(ix, ring);
      consider(ix, -ring);
    }
    for (int iy = -ring + 1; iy <= ring - 1; ++iy) {
      consider(ring, iy);
      consider(-ring, iy);
    }

    if (found_in_ring) {
      goal = best_goal;
      RCLCPP_INFO(
        logger_,
        "[FastPath] snapped goal (%.3f, %.3f) -> (%.3f, %.3f) score=%.3f ring=%d mode=%s",
        original.pose.position.x, original.pose.position.y,
        goal.pose.position.x, goal.pose.position.y, best_score, ring, snap_mode);
      if (debug_viz_) {
        debug_viz_->publishSnappedGoal(original, goal);
      }
      return true;
    }
  }

  return false;
}

bool FastPathPlanner::findPose(
  const geometry_msgs::msg::Pose2D & original_pose,
  const geometry_msgs::msg::Pose2D & edge_pose,
  double d,
  geometry_msgs::msg::Pose2D & output_pose) const
{
  double vx = edge_pose.x - original_pose.x;
  double vy = edge_pose.y - original_pose.y;
  double param_x = 0.0;
  double param_y = 0.0;
  calculateLineParam(param_x, param_y, vx, vy);
  output_pose.x = original_pose.x + param_x * d;
  output_pose.y = original_pose.y + param_y * d;
  return true;
}

void FastPathPlanner::calculateLineParam(double & x, double & y, double vx, double vy) const
{
  if (std::fabs(vx) < 1e-5 && std::fabs(vy) < 1e-5) {
    x = 0.0;
    y = 0.0;
    return;
  }
  const double magnitude = std::sqrt(vx * vx + vy * vy);
  x = vx / magnitude;
  y = vy / magnitude;
}

geometry_msgs::msg::Quaternion FastPathPlanner::yawToQuaternion(double yaw) const
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quat);
}

}  // namespace nav2_planner
