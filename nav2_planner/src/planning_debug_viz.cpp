// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "nav2_planner/planning_debug_viz.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_planner
{

namespace
{
constexpr double kArrowLen = 0.45;
constexpr double kArrowWidth = 0.08;
constexpr double kTextHeight = 0.22;
constexpr double kZ = 0.08;
}

PlanningDebugViz::PlanningDebugViz()
: logger_(rclcpp::get_logger("PlanningDebugViz"))
{
}

void PlanningDebugViz::configure(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  costmap_ros_ = costmap_ros;
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  if (!node->has_parameter("publish_planning_debug")) {
    node->declare_parameter("publish_planning_debug", true);
  }
  node->get_parameter("publish_planning_debug", enabled_);

  if (!node->has_parameter("planning_debug_footprint_stride")) {
    node->declare_parameter("planning_debug_footprint_stride", 0);
  }
  footprint_stride_ = node->get_parameter("planning_debug_footprint_stride").as_int();

  if (!node->has_parameter("planning_debug_keep_mode")) {
    node->declare_parameter("planning_debug_keep_mode", std::string("session"));
  }
  setKeepMode(node->get_parameter("planning_debug_keep_mode").as_string());

  // Path 话题只保留最后一条；Marker 在 session 模式由 flush() 整包发出，depth=1 即可 latch。
  auto qos = rclcpp::QoS(1).transient_local().reliable();
  auto marker_qos = rclcpp::QoS(accumulate_ ? 1 : 32).transient_local().reliable();
  straight_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "planning_debug/straight_path", qos);
  straight_iter_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "planning_debug/straight_iter_path", qos);
  rotated_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "planning_debug/rotated_path", qos);
  hybrid_raw_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "planning_debug/hybrid_raw_path", qos);
  hybrid_clipped_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "planning_debug/hybrid_clipped_path", qos);
  marker_pub_ = node->create_publisher<MarkerArray>(
    "planning_debug/markers", marker_qos);

  RCLCPP_INFO(
    logger_,
    "[PlanDbg] configured publish_planning_debug=%s keep_mode=%s footprint_stride=%d",
    enabled_ ? "true" : "false",
    accumulate_ ? "session" : "current",
    footprint_stride_);
}

void PlanningDebugViz::activate()
{
  if (straight_path_pub_) {straight_path_pub_->on_activate();}
  if (straight_iter_pub_) {straight_iter_pub_->on_activate();}
  if (rotated_path_pub_) {rotated_path_pub_->on_activate();}
  if (hybrid_raw_pub_) {hybrid_raw_pub_->on_activate();}
  if (hybrid_clipped_pub_) {hybrid_clipped_pub_->on_activate();}
  if (marker_pub_) {marker_pub_->on_activate();}
  activated_ = true;
}

void PlanningDebugViz::deactivate()
{
  activated_ = false;
  if (straight_path_pub_) {straight_path_pub_->on_deactivate();}
  if (straight_iter_pub_) {straight_iter_pub_->on_deactivate();}
  if (rotated_path_pub_) {rotated_path_pub_->on_deactivate();}
  if (hybrid_raw_pub_) {hybrid_raw_pub_->on_deactivate();}
  if (hybrid_clipped_pub_) {hybrid_clipped_pub_->on_deactivate();}
  if (marker_pub_) {marker_pub_->on_deactivate();}
}

void PlanningDebugViz::cleanup()
{
  straight_path_pub_.reset();
  straight_iter_pub_.reset();
  rotated_path_pub_.reset();
  hybrid_raw_pub_.reset();
  hybrid_clipped_pub_.reset();
  marker_pub_.reset();
  costmap_ros_.reset();
  clock_.reset();
  session_markers_.clear();
  activated_ = false;
}

bool PlanningDebugViz::canPublish() const
{
  return enabled_ && activated_ && marker_pub_ && marker_pub_->is_activated();
}

void PlanningDebugViz::setKeepMode(const std::string & mode)
{
  accumulate_ = (mode != "current");
}

int PlanningDebugViz::markerId(int local) const
{
  if (!accumulate_) {
    return local;
  }
  return static_cast<int>(via_index_) * 32 + local;
}

int PlanningDebugViz::markerId(int local, int extra) const
{
  if (!accumulate_) {
    return local + extra;
  }
  return static_cast<int>(via_index_) * 256 + local + extra;
}

std::string PlanningDebugViz::frame() const
{
  return costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string("map");
}

builtin_interfaces::msg::Time PlanningDebugViz::now() const
{
  if (!clock_) {
    return builtin_interfaces::msg::Time();
  }
  return clock_->now();
}

void PlanningDebugViz::publishPath(
  const PathPub::SharedPtr & pub, const nav_msgs::msg::Path & path)
{
  if (!canPublish() || !pub || !pub->is_activated()) {
    return;
  }
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (msg->header.frame_id.empty()) {
    msg->header.frame_id = frame();
  }
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    msg->header.stamp = now();
  }
  pub->publish(std::move(msg));
}

void PlanningDebugViz::publishMarkers(const MarkerArray & arr)
{
  if (!canPublish()) {
    return;
  }
  if (accumulate_) {
    ingestMarkers(arr);
    return;
  }
  if (arr.markers.empty()) {
    return;
  }
  marker_pub_->publish(arr);
}

void PlanningDebugViz::ingestMarkers(const MarkerArray & arr)
{
  for (const auto & m : arr.markers) {
    if (m.action == Marker::DELETEALL) {
      session_markers_.clear();
      continue;
    }
    session_markers_[std::make_pair(m.ns, m.id)] = m;
  }
}

void PlanningDebugViz::publishSessionSnapshot()
{
  if (!canPublish() || !accumulate_) {
    return;
  }
  MarkerArray out;
  out.markers.push_back(deleteAll(""));
  out.markers.reserve(session_markers_.size() + 1);
  for (const auto & kv : session_markers_) {
    out.markers.push_back(kv.second);
  }
  marker_pub_->publish(out);
}

visualization_msgs::msg::Marker PlanningDebugViz::deleteAll(const std::string & ns) const
{
  Marker m;
  m.header.frame_id = frame();
  m.header.stamp = now();
  m.ns = ns;
  m.id = 0;
  m.action = Marker::DELETEALL;
  return m;
}

std_msgs::msg::ColorRGBA PlanningDebugViz::rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

std_msgs::msg::ColorRGBA PlanningDebugViz::segmentColor(unsigned int index, float alpha)
{
  // 黄金分割步长，相邻 via 色相拉开；s/v 固定保证在 RViz 里够亮。
  const float hue = std::fmod(static_cast<float>(index) * 0.6180339887f, 1.0f);
  const float sat = 0.85f;
  const float val = 0.95f;
  const float chroma = val * sat;
  const float x = chroma * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
  const float m = val - chroma;
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  const int sector = static_cast<int>(std::floor(hue * 6.0f)) % 6;
  switch (sector) {
    case 0: r = chroma; g = x; b = 0.0f; break;
    case 1: r = x; g = chroma; b = 0.0f; break;
    case 2: r = 0.0f; g = chroma; b = x; break;
    case 3: r = 0.0f; g = x; b = chroma; break;
    case 4: r = x; g = 0.0f; b = chroma; break;
    default: r = chroma; g = 0.0f; b = x; break;
  }
  return rgba(r + m, g + m, b + m, alpha);
}

visualization_msgs::msg::Marker PlanningDebugViz::poseArrow(
  const std::string & ns, int id,
  const geometry_msgs::msg::PoseStamped & pose,
  const std_msgs::msg::ColorRGBA & color) const
{
  Marker m;
  m.header.frame_id = pose.header.frame_id.empty() ? frame() : pose.header.frame_id;
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = Marker::ARROW;
  m.action = Marker::ADD;
  m.pose = pose.pose;
  m.pose.position.z = kZ;
  m.scale.x = kArrowLen;
  m.scale.y = kArrowWidth;
  m.scale.z = kArrowWidth;
  m.color = color;
  m.lifetime = rclcpp::Duration(0, 0);
  return m;
}

visualization_msgs::msg::Marker PlanningDebugViz::poseSphere(
  const std::string & ns, int id,
  const geometry_msgs::msg::Point & point,
  const std_msgs::msg::ColorRGBA & color,
  double scale) const
{
  Marker m;
  m.header.frame_id = frame();
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.pose.position = point;
  m.pose.position.z = kZ;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color = color;
  m.lifetime = rclcpp::Duration(0, 0);
  return m;
}

visualization_msgs::msg::Marker PlanningDebugViz::poseText(
  const std::string & ns, int id,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & color) const
{
  Marker m;
  m.header.frame_id = pose.header.frame_id.empty() ? frame() : pose.header.frame_id;
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = Marker::TEXT_VIEW_FACING;
  m.action = Marker::ADD;
  m.pose = pose.pose;
  m.pose.position.z = kZ + 0.25;
  m.scale.z = kTextHeight;
  m.color = color;
  m.text = text;
  m.lifetime = rclcpp::Duration(0, 0);
  return m;
}

visualization_msgs::msg::Marker PlanningDebugViz::footprintStrip(
  const std::string & ns, int id,
  const geometry_msgs::msg::PoseStamped & pose,
  const std_msgs::msg::ColorRGBA & color) const
{
  Marker m;
  m.header.frame_id = pose.header.frame_id.empty() ? frame() : pose.header.frame_id;
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = Marker::LINE_STRIP;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.03;
  m.color = color;
  m.lifetime = rclcpp::Duration(0, 0);

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const std::vector<geometry_msgs::msg::Point> fp = costmap_ros_ ?
    costmap_ros_->getRobotFootprint() : std::vector<geometry_msgs::msg::Point>{};
  for (const auto & pt : fp) {
    geometry_msgs::msg::Point p;
    p.x = pose.pose.position.x + pt.x * c - pt.y * s;
    p.y = pose.pose.position.y + pt.x * s + pt.y * c;
    p.z = kZ;
    m.points.push_back(p);
  }
  if (!m.points.empty()) {
    m.points.push_back(m.points.front());
  }
  return m;
}

visualization_msgs::msg::Marker PlanningDebugViz::pathStrip(
  const std::string & ns, int id,
  const nav_msgs::msg::Path & path,
  const std_msgs::msg::ColorRGBA & color) const
{
  Marker m;
  m.header.frame_id = frame();
  m.header.stamp = now();
  m.ns = ns;
  m.id = id;
  m.type = Marker::LINE_STRIP;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.055;
  m.color = color;
  m.lifetime = rclcpp::Duration(0, 0);
  for (const auto & ps : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = kZ;
    m.points.push_back(p);
  }
  return m;
}

nav_msgs::msg::Path PlanningDebugViz::twoPointPath(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame();
  path.header.stamp = now();
  auto pa = a;
  auto pb = b;
  pa.header = path.header;
  pb.header = path.header;
  path.poses.push_back(pa);
  path.poses.push_back(pb);
  return path;
}

void PlanningDebugViz::beginSession()
{
  via_index_ = 0;
  via_count_ = 0;
  session_markers_.clear();
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(deleteAll(""));
  marker_pub_->publish(arr);
  clearPaths();
  RCLCPP_INFO(
    logger_,
    "[PlanDbg] beginSession: cleared markers (keep_mode=%s)",
    accumulate_ ? "session" : "current");
}

void PlanningDebugViz::endSession()
{
  flush();
  RCLCPP_INFO(
    logger_,
    "[PlanDbg] endSession: published %zu markers",
    session_markers_.size());
}

void PlanningDebugViz::flush()
{
  if (!accumulate_) {
    return;
  }
  publishSessionSnapshot();
}

void PlanningDebugViz::clearPaths()
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = frame();
  empty.header.stamp = now();
  publishPath(straight_path_pub_, empty);
  publishPath(straight_iter_pub_, empty);
  publishPath(rotated_path_pub_, empty);
  publishPath(hybrid_raw_pub_, empty);
  publishPath(hybrid_clipped_pub_, empty);
}

void PlanningDebugViz::setSegmentContext(
  unsigned int via_index,
  std::size_t via_count,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal_original)
{
  via_index_ = via_index;
  via_count_ = via_count;
  if (!accumulate_) {
    MarkerArray clear;
    clear.markers.push_back(deleteAll(""));
    publishMarkers(clear);
    clearPaths();
  }
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseArrow("segment", markerId(0), start, rgba(0.1f, 0.4f, 1.0f)));
  std::ostringstream ss;
  ss << "via " << via_index << "/" << via_count;
  arr.markers.push_back(poseText("segment", markerId(1), start, ss.str(), rgba(1.0f, 1.0f, 1.0f)));
  arr.markers.push_back(poseArrow("segment", markerId(2), goal_original, rgba(0.0f, 0.8f, 1.0f)));
  arr.markers.push_back(poseText("segment", markerId(3), goal_original, "G_orig", rgba(0.0f, 0.8f, 1.0f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishStart(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseArrow("start", markerId(0), pose, rgba(0.1f, 0.4f, 1.0f)));
  arr.markers.push_back(poseText("start", markerId(1), pose, "S", rgba(0.1f, 0.4f, 1.0f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishOriginalGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseArrow("goal_original", markerId(0), pose, rgba(0.0f, 0.8f, 1.0f)));
  arr.markers.push_back(poseText("goal_original", markerId(1), pose, "G_orig", rgba(0.0f, 0.8f, 1.0f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishSnappedGoal(
  const geometry_msgs::msg::PoseStamped & original,
  const geometry_msgs::msg::PoseStamped & snapped)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseArrow("goal_original", markerId(0), original, rgba(0.0f, 0.8f, 1.0f)));
  arr.markers.push_back(poseText("goal_original", markerId(1), original, "G_orig", rgba(0.0f, 0.8f, 1.0f)));
  arr.markers.push_back(poseArrow("goal_snapped", markerId(0), snapped, rgba(0.4f, 0.2f, 1.0f)));
  arr.markers.push_back(poseText("goal_snapped", markerId(1), snapped, "G_snap", rgba(0.4f, 0.2f, 1.0f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishAdjustedGoal(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & reason)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseArrow("goal_adjusted", markerId(0), pose, rgba(0.1f, 0.9f, 0.2f)));
  arr.markers.push_back(poseText(
      "goal_adjusted", markerId(1), pose, "G' " + reason, rgba(0.1f, 0.9f, 0.2f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishPushedGoal(
  const geometry_msgs::msg::PoseStamped & original,
  const geometry_msgs::msg::PoseStamped & pushed)
{
  publishOriginalGoal(original);
  publishAdjustedGoal(pushed, "push");
}

void PlanningDebugViz::publishDroppedVia(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & reason)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(poseSphere(
      "via_dropped", markerId(0), pose.pose.position, rgba(0.5f, 0.5f, 0.5f), 0.22));
  arr.markers.push_back(poseText("via_dropped", markerId(1), pose, "drop:" + reason, rgba(0.8f, 0.8f, 0.8f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishCollision(
  const geometry_msgs::msg::PoseStamped & pose,
  double dx,
  double dy,
  double sample_d,
  double segment_length)
{
  if (!canPublish()) {
    return;
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  geometry_msgs::msg::Point lethal;
  lethal.x = pose.pose.position.x + dx * std::cos(yaw) - dy * std::sin(yaw);
  lethal.y = pose.pose.position.y + dx * std::sin(yaw) + dy * std::cos(yaw);
  lethal.z = 0.0;

  MarkerArray arr;
  arr.markers.push_back(poseArrow("collision", markerId(0), pose, rgba(1.0f, 0.15f, 0.15f)));
  arr.markers.push_back(footprintStrip(
      "collision", markerId(1), pose, rgba(1.0f, 0.25f, 0.0f, 0.9f)));
  arr.markers.push_back(poseSphere(
      "collision", markerId(2), lethal, rgba(1.0f, 0.1f, 0.1f), 0.14));
  std::ostringstream ss;
  ss << "v" << via_index_ << " base s=" << std::fixed << std::setprecision(2) << sample_d
     << "/" << segment_length << " dx=" << dx << " dy=" << dy;
  arr.markers.push_back(poseText("collision", markerId(3), pose, ss.str(), rgba(1.0f, 0.4f, 0.4f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishCorridor(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double half_width,
  bool blocked,
  const geometry_msgs::msg::PoseStamped & hit_pose)
{
  if (!canPublish()) {
    return;
  }
  const double ax = start.pose.position.x;
  const double ay = start.pose.position.y;
  const double bx = goal.pose.position.x;
  const double by = goal.pose.position.y;
  const double L = std::hypot(bx - ax, by - ay);
  const double ux = (L > 1e-6) ? (bx - ax) / L : 1.0;
  const double uy = (L > 1e-6) ? (by - ay) / L : 0.0;
  const double nx = -uy;
  const double ny = ux;

  Marker box;
  box.header.frame_id = frame();
  box.header.stamp = now();
  box.ns = "corridor";
  box.id = markerId(0);
  box.type = Marker::LINE_STRIP;
  box.action = Marker::ADD;
  box.pose.orientation.w = 1.0;
  box.scale.x = 0.04;
  box.color = blocked ? rgba(1.0f, 0.15f, 0.15f, 0.95f) : rgba(0.15f, 0.9f, 0.25f, 0.95f);
  box.lifetime = rclcpp::Duration(0, 0);
  auto corner = [&](double px, double py, double side) {
    geometry_msgs::msg::Point p;
    p.x = px + side * half_width * nx;
    p.y = py + side * half_width * ny;
    p.z = kZ;
    return p;
  };
  box.points.push_back(corner(ax, ay, 1.0));
  box.points.push_back(corner(bx, by, 1.0));
  box.points.push_back(corner(bx, by, -1.0));
  box.points.push_back(corner(ax, ay, -1.0));
  box.points.push_back(box.points.front());

  MarkerArray arr;
  arr.markers.push_back(box);
  std::ostringstream ss;
  ss << "v" << via_index_ << (blocked ? " corridor 254" : " corridor ok")
     << " w=" << std::fixed << std::setprecision(2) << (2.0 * half_width);
  arr.markers.push_back(poseText(
      "corridor", markerId(1), blocked ? hit_pose : start, ss.str(),
      blocked ? rgba(1.0f, 0.4f, 0.4f) : rgba(0.2f, 0.9f, 0.3f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishStraightCandidate(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  int iter,
  const std::string & kind)
{
  auto path = twoPointPath(start, goal);
  publishPath(straight_iter_pub_, path);
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  std::ostringstream ss;
  ss << "v" << via_index_ << " " << kind << " iter=" << iter;
  arr.markers.push_back(poseText("straight_iter", markerId(0), goal, ss.str(), rgba(1.0f, 0.85f, 0.1f)));
  arr.markers.push_back(footprintStrip("straight_iter", markerId(1), start, rgba(1.0f, 0.7f, 0.1f, 0.7f)));
  arr.markers.push_back(footprintStrip("straight_iter", markerId(2), goal, rgba(1.0f, 0.7f, 0.1f, 0.7f)));
  arr.markers.push_back(pathStrip("path_straight_iter", markerId(0), path, rgba(1.0f, 0.85f, 0.1f, 0.9f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishStraightPath(const nav_msgs::msg::Path & path)
{
  publishPath(straight_path_pub_, path);
  if (path.poses.empty() || !canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(pathStrip("path_straight", markerId(0), path, rgba(0.2f, 0.95f, 0.3f, 0.95f)));
  publishMarkers(arr);
  if (!accumulate_ || footprint_stride_ > 0) {
    publishFootprintsAlongPath(path, "footprint_straight", footprint_stride_);
  }
}

void PlanningDebugViz::publishRotatedPath(const nav_msgs::msg::Path & path)
{
  publishPath(rotated_path_pub_, path);
  if (path.poses.empty() || !canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(pathStrip("path_rotated", markerId(0), path, rgba(1.0f, 0.55f, 0.05f, 0.95f)));
  publishMarkers(arr);
  if (!accumulate_ || footprint_stride_ > 0) {
    publishFootprintsAlongPath(
      path, "footprint_rotated", footprint_stride_ > 0 ? footprint_stride_ : 1);
  }
}

void PlanningDebugViz::publishFootprintsAlongPath(
  const nav_msgs::msg::Path & path,
  const std::string & ns,
  int stride)
{
  if (!canPublish() || path.poses.empty()) {
    return;
  }
  MarkerArray arr;
  const int n = static_cast<int>(path.poses.size());
  const int step = stride > 0 ? stride : std::max(1, n - 1);
  const int base = markerId(0);
  int id = 0;
  for (int i = 0; i < n; i += step) {
    arr.markers.push_back(footprintStrip(
        ns, base + id++, path.poses[static_cast<std::size_t>(i)],
        rgba(1.0f, 0.55f, 0.05f, 0.55f)));
  }
  if ((n - 1) % step != 0) {
    arr.markers.push_back(footprintStrip(
        ns, base + id++, path.poses.back(),
        rgba(1.0f, 0.55f, 0.05f, 0.55f)));
  }
  publishMarkers(arr);
}

void PlanningDebugViz::publishHybridRaw(const nav_msgs::msg::Path & path)
{
  publishPath(hybrid_raw_pub_, path);
  if (path.poses.empty() || !canPublish()) {
    return;
  }
  const auto color = segmentColor(via_index_);
  Marker strip = pathStrip("path_hybrid_raw", markerId(0), path, color);
  strip.scale.x = 0.08;
  MarkerArray arr;
  arr.markers.push_back(strip);
  std::ostringstream ss;
  ss << "H via=" << via_index_;
  if (via_count_ > 0) {
    ss << "/" << via_count_;
  }
  arr.markers.push_back(poseText(
      "path_hybrid_raw", markerId(1), path.poses.front(), ss.str(), color));
  publishMarkers(arr);
}

void PlanningDebugViz::publishHybridClipped(const nav_msgs::msg::Path & path)
{
  publishPath(hybrid_clipped_pub_, path);
  if (path.poses.empty() || !canPublish()) {
    return;
  }
  MarkerArray arr;
  arr.markers.push_back(pathStrip("path_hybrid_clipped", markerId(0), path, rgba(0.3f, 0.6f, 1.0f, 0.95f)));
  publishMarkers(arr);
}

void PlanningDebugViz::publishIntersections(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  if (!canPublish()) {
    return;
  }
  MarkerArray arr;
  for (std::size_t i = 0; i < points.size(); ++i) {
    arr.markers.push_back(poseSphere(
        "intersect", markerId(0, static_cast<int>(i)), points[i], rgba(1.0f, 0.0f, 1.0f), 0.16));
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame();
    ps.pose.position = points[i];
    ps.pose.orientation.w = 1.0;
    arr.markers.push_back(poseText(
        "intersect", markerId(100, static_cast<int>(i)), ps, "X", rgba(1.0f, 0.4f, 1.0f)));
  }
  publishMarkers(arr);
}

}  // namespace nav2_planner
