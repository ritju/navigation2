// Copyright (c) 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "nav2_planner/fast_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
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
  _footprint_extend_back_x = getDoubleParam(node, "footprint_extend_back_x", 0.0);
  _footprint_extend_front_x = getDoubleParam(node, "footprint_extend_front_x", 0.0);
  _footprint_extend_y = getDoubleParam(node, "footprint_extend_y", 0.0);
  _straight_check_length_ratio = getDoubleParam(node, "straight_check_length_ratio", 0.5);
  _straight_path_resolution = getDoubleParam(node, "straight_path_resolution", 0.1);

  // 兼容旧 yaml：插件命名空间下的同名参数覆盖 server 根参数。
  for (const auto & id : planner_ids) {
    _goal_occupied_tolerance = getDoubleParam(
      node, id + ".goal_occupied_tolerance", _goal_occupied_tolerance);
    _goal_search_resolution = getDoubleParam(
      node, id + ".goal_search_resolution", _goal_search_resolution);
    _footprint_extend_back_x = getDoubleParam(
      node, id + ".footprint_extend_back_x", _footprint_extend_back_x);
    _footprint_extend_front_x = getDoubleParam(
      node, id + ".footprint_extend_front_x", _footprint_extend_front_x);
    _footprint_extend_y = getDoubleParam(
      node, id + ".footprint_extend_y", _footprint_extend_y);
    _straight_check_length_ratio = getDoubleParam(
      node, id + ".straight_check_length_ratio", _straight_check_length_ratio);
    _straight_path_resolution = getDoubleParam(
      node, id + ".straight_path_resolution", _straight_path_resolution);
  }

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
    "goal_search_resolution=%.2f footprint_extend=(back=%.2f, front=%.2f, y=%.2f) "
    "straight_check_length_ratio=%.2f straight_path_resolution=%.2f planner_ids=%zu",
    _goal_occupied_tolerance, _goal_search_resolution,
    _footprint_extend_back_x, _footprint_extend_front_x, _footprint_extend_y,
    _straight_check_length_ratio, _straight_path_resolution, planner_ids.size());
}

void FastPathPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "[FastPath] cleanup");
  narrow_passages_sub_.reset();
  enable_backward_sub_.reset();
  footprint_checker_.reset();
  costmap_ros_.reset();
  costmap_ = nullptr;
}

FastPlanResult FastPathPlanner::compute(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool allow_straight,
  bool allow_reverse)
{
  FastPlanResult result;
  result.snapped_goal = goal;
  result.reason = FastPlanReason::NeedAstar;
  result.path.header.stamp = clock_->now();
  result.path.header.frame_id = costmap_ros_->getGlobalFrameID();

  RCLCPP_INFO(
    logger_,
    "[FastPath] compute start=(%.3f, %.3f, yaw=%.3f) goal=(%.3f, %.3f, yaw=%.3f) "
    "allow_straight=%s allow_reverse=%s",
    start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation),
    goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation),
    allow_straight ? "true" : "false",
    allow_reverse ? "true" : "false");

  if (!costmap_ || !footprint_checker_) {
    RCLCPP_ERROR(logger_, "[FastPath] not configured, fallback NeedAstar");
    return result;
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  updateFootprintExtents();

  // 1) 目标占用则由近及远 snap
  if (!snapOccupiedGoal(result.snapped_goal)) {
    result.reason = FastPlanReason::GoalUnreachable;
    RCLCPP_WARN(
      logger_,
      "[FastPath] GoalUnreachable: original goal (%.3f, %.3f) occupied, "
      "no free pose within tolerance=%.2f res=%.2f",
      goal.pose.position.x, goal.pose.position.y,
      _goal_occupied_tolerance, _goal_search_resolution);
    return result;
  }

  if (!allow_straight) {
    result.reason = FastPlanReason::NeedAstar;
    RCLCPP_INFO(
      logger_,
      "[FastPath] skip straight-line, reason=NeedAstar snapped_goal=(%.3f, %.3f)",
      result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y);
    return result;
  }

  // 2) 先试前进直线
  result.path = tryStraightPath(start, result.snapped_goal, false);
  if (!result.path.poses.empty()) {
    result.reason = FastPlanReason::StraightOk;
    RCLCPP_INFO(
      logger_,
      "[FastPath] StraightOk forward poses=%zu snapped_goal=(%.3f, %.3f)",
      result.path.poses.size(),
      result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y);
    return result;
  }

  // 3) 允许倒车时再试后退直线（同一套转向+整车沿线）
  if (allow_reverse) {
    result.path = tryStraightPath(start, result.snapped_goal, true);
    if (!result.path.poses.empty()) {
      result.reason = FastPlanReason::StraightOk;
      RCLCPP_INFO(
        logger_,
        "[FastPath] StraightOk reverse poses=%zu snapped_goal=(%.3f, %.3f)",
        result.path.poses.size(),
        result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y);
      return result;
    }
  }

  result.reason = FastPlanReason::NeedAstar;
  RCLCPP_INFO(
    logger_,
    "[FastPath] straight-line failed, reason=%s snapped_goal=(%.3f, %.3f) "
    "tried_reverse=%s",
    reasonToString(result.reason),
    result.snapped_goal.pose.position.x, result.snapped_goal.pose.position.y,
    allow_reverse ? "true" : "false");
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
  const nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
  for (const auto & pt : footprint) {
    footprint_back_x_ = std::min(footprint_back_x_, static_cast<double>(pt.x));
    footprint_front_x_ = std::max(footprint_front_x_, static_cast<double>(pt.x));
  }
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

bool FastPathPlanner::isFree(
  const geometry_msgs::msg::PoseStamped & pose,
  double footprint_extend_back_x,
  double footprint_extend_front_x,
  double footprint_extend_y) const
{
  try {
    const double theta = tf2::getYaw(pose.pose.orientation);
    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);
    const double resolution = costmap_->getResolution();
    std::vector<double> footprint_extend{0.0};
    if (footprint_extend_y != 0.0) {
      footprint_extend.emplace_back(-footprint_extend_y);
      footprint_extend.emplace_back(footprint_extend_y);
    }
    const double x_start = footprint_back_x_ + footprint_extend_back_x;
    const double x_end = footprint_front_x_ + footprint_extend_front_x;
    auto sample_occupied = [&](double x, double y) {
      unsigned int map_x = 0;
      unsigned int map_y = 0;
      const double g_x = pose.pose.position.x + x * cos_th - y * sin_th;
      const double g_y = pose.pose.position.y + x * sin_th + y * cos_th;
      if (!costmap_->worldToMap(g_x, g_y, map_x, map_y)) {
        return true;
      }
      const unsigned char footprint_cost = costmap_->getCost(map_x, map_y);
      return footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
             footprint_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    };

    for (auto y : footprint_extend) {
      for (double x = x_start; ; x += resolution) {
        const double sample_x = std::min(x, x_end);
        if (sample_occupied(sample_x, y)) {
          return false;
        }
        if (sample_x >= x_end - 1e-9) {
          break;
        }
      }
    }
  } catch (const nav2_costmap_2d::IllegalPoseException & e) {
    RCLCPP_ERROR(logger_, "[FastPath] isFree IllegalPose: %s", e.what());
    return false;
  } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
    RCLCPP_ERROR(logger_, "[FastPath] isFree CollisionChecker: %s", e.what());
    return false;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger_, "[FastPath] isFree runtime_error: %s", e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(logger_, "[FastPath] isFree failed to check pose score");
    return false;
  }
  return true;
}

bool FastPathPlanner::snapOccupiedGoal(geometry_msgs::msg::PoseStamped & goal)
{
  if (isFree(goal, _footprint_extend_back_x, _footprint_extend_front_x, _footprint_extend_y)) {
    RCLCPP_DEBUG(
      logger_,
      "[FastPath] goal (%.3f, %.3f) already free, no snap",
      goal.pose.position.x, goal.pose.position.y);
    return true;
  }

  const double res = std::max(_goal_search_resolution, 1e-3);
  const int max_ring = static_cast<int>(std::ceil(_goal_occupied_tolerance / res));
  RCLCPP_INFO(
    logger_,
    "[FastPath] goal (%.3f, %.3f) occupied, ring search tolerance=%.2f res=%.2f max_ring=%d",
    goal.pose.position.x, goal.pose.position.y,
    _goal_occupied_tolerance, res, max_ring);

  const auto original = goal;
  // 按切比雪夫圈由近及远；某一圈找到自由点即停止，圈内取欧氏距离最近。
  for (int ring = 1; ring <= max_ring; ++ring) {
    bool found_in_ring = false;
    double best_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::PoseStamped best_goal = original;

    auto consider = [&](int ix, int iy) {
      const double dx = static_cast<double>(ix) * res;
      const double dy = static_cast<double>(iy) * res;
      const double dist = std::hypot(dx, dy);
      if (dist > _goal_occupied_tolerance + 1e-6) {
        return;
      }
      auto search_goal = original;
      search_goal.pose.position.x += dx;
      search_goal.pose.position.y += dy;
      if (!isFree(
          search_goal, _footprint_extend_back_x, _footprint_extend_front_x,
          _footprint_extend_y))
      {
        return;
      }
      found_in_ring = true;
      if (dist < best_dist) {
        best_dist = dist;
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
        "[FastPath] snapped goal (%.3f, %.3f) -> (%.3f, %.3f) offset=%.3f m ring=%d",
        original.pose.position.x, original.pose.position.y,
        goal.pose.position.x, goal.pose.position.y, best_dist, ring);
      return true;
    }
  }

  return false;
}

nav_msgs::msg::Path FastPathPlanner::tryStraightPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool reverse)
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();

  geometry_msgs::msg::Pose2D start_pose2d;
  start_pose2d.x = start.pose.position.x;
  start_pose2d.y = start.pose.position.y;
  start_pose2d.theta = tf2::getYaw(start.pose.orientation);

  geometry_msgs::msg::Pose2D goal_pose2d;
  goal_pose2d.x = goal.pose.position.x;
  goal_pose2d.y = goal.pose.position.y;
  goal_pose2d.theta = tf2::getYaw(goal.pose.orientation);

  const double line_yaw =
    std::atan2(goal_pose2d.y - start_pose2d.y, goal_pose2d.x - start_pose2d.x);
  const double heading = reverse ? normalizeAngle(line_yaw + M_PI) : line_yaw;
  const double distance_start_to_goal =
    nav2_util::geometry_utils::euclidean_distance(start_pose2d, goal_pose2d);
  const double check_step = straightCheckStep();
  const double path_step = straightPathStep();
  const nav2_costmap_2d::Footprint check_footprint = costmap_ros_->getRobotFootprint();
  const double extend_back = reverse ? _footprint_extend_front_x : _footprint_extend_back_x;
  const double extend_front = reverse ? _footprint_extend_back_x : _footprint_extend_front_x;

  RCLCPP_INFO(
    logger_,
    "[FastPath] try %s straight-line dist=%.3f m line_yaw=%.3f heading=%.3f "
    "start_yaw=%.3f check_step=%.3f path_step=%.3f ratio=%.2f",
    reverse ? "reverse" : "forward",
    distance_start_to_goal, line_yaw, heading, start_pose2d.theta,
    check_step, path_step, _straight_check_length_ratio);

  // 原地转到连线对应的车头朝向；任一步 LETHAL 则放弃本次直线。
  const double start_theta = start_pose2d.theta;
  double diff_theta = normalizeAngle(heading - start_theta);
  const double yaw_step = diff_theta < 0.0 ? -0.087 : 0.087;
  const size_t n_rot = static_cast<size_t>(std::floor(std::fabs(diff_theta / yaw_step)));
  for (size_t i = 1; i < n_rot; ++i) {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = start.pose.position.x;
    pose2d.y = start.pose.position.y;
    pose2d.theta = normalizeAngle(start_theta + yaw_step * static_cast<double>(i));
    const double footprint_cost = footprint_checker_->footprintCostAtPose(
      pose2d.x, pose2d.y, pose2d.theta, check_footprint);
    if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(
        logger_,
        "[FastPath] %s rotation occupied at yaw=%.3f (step %zu), abort this straight",
        reverse ? "reverse" : "forward", pose2d.theta, i);
      return plan;
    }
  }

  const double goal_footprint_cost = footprint_checker_->footprintCostAtPose(
    goal_pose2d.x, goal_pose2d.y, heading, check_footprint);
  if (goal_footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    RCLCPP_INFO(
      logger_,
      "[FastPath] %s goal footprint LETHAL at heading=%.3f, abort this straight",
      reverse ? "reverse" : "forward", heading);
    return plan;
  }

  try {
    bool is_path_free = true;

    // 碰撞：按车长比例步长做整车检测；起点也要查，避免短路径漏检。
    for (double d = 0.0; ; d += check_step) {
      const double sample_d = std::min(d, distance_start_to_goal);
      geometry_msgs::msg::Pose2D path_pose;
      path_pose.theta = heading;
      findPose(start_pose2d, goal_pose2d, sample_d, path_pose);
      geometry_msgs::msg::PoseStamped path_posestamped;
      path_posestamped.header = plan.header;
      path_posestamped.pose.position.x = path_pose.x;
      path_posestamped.pose.position.y = path_pose.y;
      path_posestamped.pose.orientation = yawToQuaternion(heading);
      is_path_free = isFree(
        path_posestamped, extend_back, extend_front, _footprint_extend_y);
      if (!is_path_free) {
        RCLCPP_INFO(
          logger_,
          "[FastPath] %s straight-line blocked at d=%.3f / %.3f pose=(%.3f, %.3f)",
          reverse ? "reverse" : "forward",
          sample_d, distance_start_to_goal, path_pose.x, path_pose.y);
        break;
      }
      if (sample_d >= distance_start_to_goal - 1e-9) {
        break;
      }
    }

    if (is_path_free) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(heading);
      for (double d = path_step; d < distance_start_to_goal - 1e-6; d += path_step) {
        geometry_msgs::msg::Pose2D path_pose;
        path_pose.theta = heading;
        findPose(start_pose2d, goal_pose2d, d, path_pose);
        pose.pose.position.x = path_pose.x;
        pose.pose.position.y = path_pose.y;
        plan.poses.emplace_back(pose);
      }
      pose.pose.position = goal.pose.position;
      pose.pose.orientation = yawToQuaternion(heading);
      plan.poses.emplace_back(pose);
      RCLCPP_INFO(
        logger_,
        "[FastPath] %s straight-line clear, poses=%zu dist=%.3f m "
        "check_step=%.3f path_step=%.3f",
        reverse ? "reverse" : "forward",
        plan.poses.size(), distance_start_to_goal, check_step, path_step);
    }
  } catch (const nav2_costmap_2d::IllegalPoseException & e) {
    RCLCPP_ERROR(logger_, "[FastPath] tryStraightPath IllegalPose: %s", e.what());
    plan.poses.clear();
  } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
    RCLCPP_ERROR(logger_, "[FastPath] tryStraightPath CollisionChecker: %s", e.what());
    plan.poses.clear();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger_, "[FastPath] tryStraightPath runtime_error: %s", e.what());
    plan.poses.clear();
  } catch (...) {
    RCLCPP_ERROR(logger_, "[FastPath] tryStraightPath failed to check pose score");
    plan.poses.clear();
  }

  return plan;
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
