// Copyright (c) 2021 Samsung Research America
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
#include <cstdint>
#include <cstddef>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <mutex>
#include <vector>
#include <algorithm>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

namespace
{

using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

/** Wrap yaw difference into [-pi, pi]. */
double wrapYaw(double dyaw)
{
  while (dyaw > M_PI) {
    dyaw -= 2.0 * M_PI;
  }
  while (dyaw < -M_PI) {
    dyaw += 2.0 * M_PI;
  }
  return dyaw;
}

/**
 * Read mission index from pose.z.
 * Negative z (typically -1) is an unindexed sentinel and must not be cast to uint32.
 */
bool tryGoalZ(const geometry_msgs::msg::PoseStamped & pose, uint32_t & z_out)
{
  const long z_round = std::lround(pose.pose.position.z);
  if (z_round < 0) {
    return false;
  }
  z_out = static_cast<uint32_t>(z_round);
  return true;
}

bool isUnindexedZ(const geometry_msgs::msg::PoseStamped & pose)
{
  return std::lround(pose.pose.position.z) < 0;
}

/** First indexed z at or after start. */
bool findZAtOrAfter(const Goals & queue, size_t start, uint32_t & z_out)
{
  for (size_t i = start; i < queue.size(); ++i) {
    if (tryGoalZ(queue[i], z_out)) {
      return true;
    }
  }
  return false;
}

/** Last indexed z at or before end. */
bool findZAtOrBefore(const Goals & queue, size_t end, uint32_t & z_out)
{
  if (queue.empty()) {
    return false;
  }
  if (end >= queue.size()) {
    end = queue.size() - 1;
  }
  for (size_t i = end + 1; i-- > 0; ) {
    if (tryGoalZ(queue[i], z_out)) {
      return true;
    }
  }
  return false;
}

/** Mix z into a fingerprint; unindexed poses use an all-ones sentinel. */
std::uint64_t mixFromZ(const geometry_msgs::msg::PoseStamped & pose)
{
  uint32_t z = 0;
  if (tryGoalZ(pose, z)) {
    return static_cast<std::uint64_t>(z);
  }
  return 0xFFFFFFFFFFFFFFFFULL;
}

/** Mission identity: mix of the last goal's header stamp. */
std::uint64_t tailStampFp(const Goals & queue)
{
  if (queue.empty()) {
    return 0;
  }
  std::uint64_t fp = 0;
  const auto & stamp = queue.back().header.stamp;
  fp ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(stamp.sec)) *
    0x9e3779b97f4a7c15ULL;
  fp ^= static_cast<std::uint64_t>(stamp.nanosec);
  return fp;
}

/** TEB polyline length from the first pose (0 if fewer than 2 poses). */
double tebLenFromFirst(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }
  return nav2_util::geometry_utils::calculate_path_length(path, 0);
}

/**
 * XY (and optional yaw) match between a GPP candidate and a mission pose.
 * match_yaw < 0 skips the yaw check.
 */
bool posesMatch(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b,
  const double match_xy,
  const double match_yaw)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  if (std::hypot(dx, dy) > match_xy) {
    return false;
  }
  if (match_yaw < 0.0) {
    return true;
  }
  const double dyaw = wrapYaw(tf2::getYaw(a.orientation) - tf2::getYaw(b.orientation));
  return std::fabs(dyaw) <= match_yaw;
}

}  // namespace

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  radius_(0.5),
  accumulate_dist_(8.0),
  prev_front_stamp_s_(0.0)
{
  getInput("radius", radius_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("accumulate_distance", accumulate_dist_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  cb_executor_.add_callback_group(
    cb_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  rclcpp::QoS qos_debug(rclcpp::KeepLast(5));
  pub_removed_plan_ = node_->create_publisher<nav_msgs::msg::Path>("removed_plan", qos_debug);
  pub_passed_indexes_ = node_->create_publisher<capella_ros_msg::msg::PassedPosesIndex>(
    "passed_pose_indexes", qos_debug);

  std::string teb_topic;
  getInput("teb_topic", teb_topic);
  rclcpp::QoS qos_teb(rclcpp::KeepLast(1));
  sub_teb_plan_ = node_->create_subscription<nav_msgs::msg::Path>(
    teb_topic, qos_teb,
    std::bind(&RemovePassedGoals::onTebPlan, this, std::placeholders::_1),
    sub_opts);

  std::string odom_topic;
  getInput("odom_topic", odom_topic);
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, qos_debug,
    std::bind(&RemovePassedGoals::onOdom, this, std::placeholders::_1),
    sub_opts);

  rclcpp::QoS qos_backward(rclcpp::KeepLast(1));
  qos_backward.transient_local();
  qos_backward.reliable();
  pub_enable_backward_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/enable_backward", qos_backward);

  last_teb_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void RemovePassedGoals::publishEnableBackward(bool enable)
{
  if (!pub_enable_backward_) {
    return;
  }
  if (published_enable_backward_ && enable == last_enable_backward_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = enable;
  pub_enable_backward_->publish(msg);
  last_enable_backward_ = enable;
  published_enable_backward_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "RemovePassedGoals: enable_backward=%s",
    enable ? "true (backward)" : "false (forward)");
}

void RemovePassedGoals::onTebPlan(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(teb_mutex_);
  latest_teb_plan_ = msg;
  if (msg) {
    cached_teb_plan_ = std::make_shared<nav_msgs::msg::Path>(*msg);
    const double len_m = tebLenFromFirst(*msg);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()),
      5000,
      "RemovePassedGoals: teb plan received, poses=%zu length=%.3f m",
      msg->poses.size(), len_m);
  } else {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 5000,
      "RemovePassedGoals: teb plan received null");
  }
  teb_rx_since_emit_ = true;
  if (node_) {
    last_teb_time_ = node_->get_clock()->now();
  }
}

void RemovePassedGoals::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_vx_ = msg->twist.twist.linear.x;
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *(node_->get_clock()),
    5000,
    "RemovePassedGoals: odom vx=%.4f m/s", odom_vx_);
}

void RemovePassedGoals::clearTebBuffers()
{
  RCLCPP_DEBUG(node_->get_logger(), "RemovePassedGoals: cleared teb plan buffers");
  std::lock_guard<std::mutex> lock(teb_mutex_);
  latest_teb_plan_.reset();
  cached_teb_plan_.reset();
  last_teb_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void RemovePassedGoals::spinSubs()
{
  cb_executor_.spin_some();
}

std::uint64_t RemovePassedGoals::gppWindowFp(
  const Goals & window,
  std::uint64_t tail_fp) const
{
  std::uint64_t fp = tail_fp;
  fp ^= (static_cast<std::uint64_t>(z_begin_) << 32) |
    static_cast<std::uint64_t>(z_end_);
  if (!window.empty()) {
    fp ^= mixFromZ(window.front());
    fp ^= mixFromZ(window.back()) << 20;
    // Endpoints alone miss middle removals that keep the same front/back z.
    fp ^= static_cast<std::uint64_t>(window.size()) << 48;
    std::uint64_t xor_z = 0;
    for (const auto & pose : window) {
      xor_z ^= mixFromZ(pose) * 0x9e3779b97f4a7c15ULL;
    }
    fp ^= xor_z;
  }
  return fp;
}

void RemovePassedGoals::initWindow(const Goals & queue)
{
  buildWindowFrom(queue, 0);
}

void RemovePassedGoals::buildWindowFrom(const Goals & queue, size_t start)
{
  if (queue.empty()) {
    return;
  }
  if (start >= queue.size()) {
    start = queue.size() - 1;
  }

  auto set_span_at = [&](size_t i) {
    uint32_t z = 0;
    if (findZAtOrAfter(queue, i, z) || findZAtOrBefore(queue, i, z)) {
      z_begin_ = z;
      z_end_ = z;
    } else {
      z_begin_ = 0;
      z_end_ = 0;
    }
  };

  if (queue.size() == 1) {
    set_span_at(0);
    return;
  }
  if (start == queue.size() - 1) {
    set_span_at(start);
    return;
  }
  using namespace nav2_util::geometry_utils;  // NOLINT
  double max_len = 0.0;
  getInput("max_gpp_segment_m", max_len);

  uint32_t z0 = 0;
  if (!findZAtOrAfter(queue, start, z0)) {
    set_span_at(start);
    return;
  }
  z_begin_ = z0;

  auto set_span_end = [&](size_t end_i) {
    uint32_t z1 = z_begin_;
    if (!findZAtOrBefore(queue, end_i, z1) && !findZAtOrAfter(queue, end_i, z1)) {
      z1 = z_begin_;
    }
    if (z1 < z_begin_) {
      z1 = z_begin_;
    }
    z_end_ = z1;
  };

  double acc_len = 0.0;
  for (size_t i = start + 1; i < queue.size(); ++i) {
    acc_len += euclidean_distance(queue[i - 1].pose, queue[i].pose);
    if (acc_len > max_len) {
      set_span_end(i);
      return;
    }
  }
  set_span_end(queue.size() - 1);
}

void RemovePassedGoals::advanceWindow(const Goals & queue)
{
  buildWindowFrom(queue, 0);
}

RemovePassedGoals::Goals RemovePassedGoals::filterByZSpan(const Goals & queue) const
{
  Goals window;
  if (queue.empty()) {
    return window;
  }

  size_t first_i = queue.size();
  size_t last_i = 0;
  bool found_in_span = false;
  bool found_any = false;
  for (size_t i = 0; i < queue.size(); ++i) {
    uint32_t z = 0;
    if (!tryGoalZ(queue[i], z)) {
      continue;
    }
    found_any = true;
    if (z >= z_begin_ && z <= z_end_) {
      if (!found_in_span) {
        first_i = i;
        found_in_span = true;
      }
      last_i = i;
    }
  }

  if (!found_in_span) {
    // No indexed z in the current span: pass the whole queue if nothing is indexed.
    if (!found_any) {
      return queue;
    }
    return window;
  }

  size_t start = first_i;
  while (start > 0 && isUnindexedZ(queue[start - 1])) {
    --start;
  }
  for (size_t i = start; i <= last_i; ++i) {
    window.push_back(queue[i]);
  }
  // Trailing unindexed sentinels after the last in-span indexed goal.
  for (size_t i = last_i + 1; i < queue.size(); ++i) {
    if (!isUnindexedZ(queue[i])) {
      break;
    }
    window.push_back(queue[i]);
  }
  return window;
}

RemovePassedGoals::Goals RemovePassedGoals::pruneByMissionPose(
  const Goals & candidates,
  const Goals & mission,
  const double match_xy,
  const double match_yaw) const
{
  Goals pruned;
  pruned.reserve(candidates.size());
  // Inner scan assumes mission is ordered by non-decreasing indexed z.
  // Unindexed sentinel z < 0 matches only other unindexed poses (XY/yaw), never as uint32.
  for (const auto & gpp : candidates) {
    uint32_t z_gpp = 0;
    const bool gpp_unindexed = !tryGoalZ(gpp, z_gpp);
    for (const auto & m : mission) {
      uint32_t z_m = 0;
      const bool m_unindexed = !tryGoalZ(m, z_m);
      if (gpp_unindexed) {
        if (!m_unindexed) {
          continue;
        }
        if (posesMatch(gpp.pose, m.pose, match_xy, match_yaw)) {
          pruned.push_back(m);
          break;
        }
        continue;
      }
      if (m_unindexed) {
        continue;
      }
      if (z_m < z_gpp) {
        continue;
      }
      if (z_m > z_gpp) {
        break;
      }
      if (posesMatch(gpp.pose, m.pose, match_xy, match_yaw)) {
        pruned.push_back(m);
      }
      break;
    }
  }
  return pruned;
}

void RemovePassedGoals::applyMonotonicStamps(
  Goals & gpp_goals,
  const rclcpp::Time & now,
  const bool fresh_on_advance)
{
  if (gpp_goals.empty()) {
    return;
  }

  for (auto & pose : gpp_goals) {
    uint32_t z = 0;
    const bool unindexed = !tryGoalZ(pose, z);
    rclcpp::Time stamp = fresh_on_advance ? now : rclcpp::Time(pose.header.stamp);

    if (!unindexed) {
      const auto it = gpp_stamp_by_z_.find(z);
      if (it != gpp_stamp_by_z_.end() && stamp < it->second) {
        stamp = it->second;
      }
      gpp_stamp_by_z_[z] = stamp;
    }

    pose.header.stamp = stamp;
  }

  if (last_gpp_front_stamp_.nanoseconds() > 0 &&
    rclcpp::Time(gpp_goals.front().header.stamp) < last_gpp_front_stamp_)
  {
    gpp_goals.front().header.stamp = last_gpp_front_stamp_;
    uint32_t z_front = 0;
    if (tryGoalZ(gpp_goals.front(), z_front)) {
      gpp_stamp_by_z_[z_front] = last_gpp_front_stamp_;
    }
  }

  last_gpp_front_stamp_ = rclcpp::Time(gpp_goals.front().header.stamp);
}

void RemovePassedGoals::resetGppState()
{
  queue_tail_fp_ = 0;
  window_inited_ = false;
  gpp_emitted_ = false;
  last_gpp_fp_ = 0;
  teb_rx_since_emit_ = false;
  last_gpp_emit_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  terminal_in_window_ = false;
  clearTebBuffers();
  last_gpp_goals_.clear();
  gpp_stamp_by_z_.clear();
  last_gpp_front_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void RemovePassedGoals::resetOnEmptyInput()
{
  RCLCPP_INFO(node_->get_logger(), "RemovePassedGoals: input_goals empty, reset and return SUCCESS");
  mission_goals_.clear();
  board_tail_fp_ = 0;
  resetGppState();
  setOutput("output_gpp_goals", Goals{});
  setOutput("output_goals", Goals{});
  if (published_enable_backward_ && last_enable_backward_) {
    publishEnableBackward(false);
  }
}

bool RemovePassedGoals::reloadMissionIfChanged(const Goals & input_goals)
{
  const std::uint64_t incoming_fp = tailStampFp(input_goals);
  if (incoming_fp == board_tail_fp_) {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "RemovePassedGoals: mission changed (tail_fp=%lx), reload queue size=%zu",
    static_cast<unsigned long>(incoming_fp),
    input_goals.size());
  board_tail_fp_ = incoming_fp;
  mission_goals_ = input_goals;
  passed_indexes_.clear();
  prev_front_stamp_s_ = 0.0;
  resetGppState();
  return true;
}

void RemovePassedGoals::maybeRecordPassed(
  uint32_t z,
  bool has_z,
  double dist,
  double dist_lim,
  const char * pass_kind)
{
  const bool dist_ok = dist_lim <= 0.0 || dist < dist_lim;
  if (has_z &&
    std::find(passed_indexes_.begin(), passed_indexes_.end(), z) == passed_indexes_.end() &&
    dist_ok)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoals: %s pass, record index=%u",
      pass_kind,
      static_cast<unsigned int>(z));
    passed_indexes_.push_back(z);
  } else if (!has_z) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "RemovePassedGoals: %s pass, strip unindexed goal (z=-1) without recording index",
      pass_kind);
  }
}

void RemovePassedGoals::stripPassedGoals(
  Goals & queue,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // pass_check_max_search_dist_m is unused: a prior whole-mission arc cap blocked valid
  // head stripping. Full-dist mode relies on viapoint radius, behind-x, and optional yaw.

  double behind_x = 0.0;
  getInput("pass_behind_tol_m", behind_x);

  bool check_yaw = true;
  getInput("pass_enable_yaw_check", check_yaw);

  double yaw_lim = 1.57;
  getInput("pass_yaw_threshold_rad", yaw_lim);
  if (yaw_lim < 0.0) {
    yaw_lim = 0.0;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const tf2::Duration tf_tol = tf2::durationFromSec(transform_tolerance_);

  double tail_max = 0.0;
  getInput("pass_tail_relax_remaining_path_max_m", tail_max);

  double tail_radius = 2.0;
  getInput("pass_tail_relax_radius_m", tail_radius);
  if (tail_radius <= 0.0) {
    tail_radius = radius_;
  }

  double dist_lim = -1.0;
  getInput("passed_goal_distance_threshold", dist_lim);

  using namespace nav2_util::geometry_utils;  // NOLINT
  const size_t n = queue.size();
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    arc[i] = arc[i - 1] + euclidean_distance(queue[i - 1].pose, queue[i].pose);
  }

  double remain_m = 0.0;
  if (n >= 2) {
    remain_m = arc.back();
  }

  const bool tail_relax = tail_max > 0.0 && remain_m <= tail_max;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "RemovePassedGoals: pass check, goals=%zu remain=%.3f m tail_max=%.3f "
    "mode=%s tail_radius=%.3f viapoint_radius=%.3f",
    n,
    remain_m,
    tail_max,
    tail_relax ? "tail_distance_only" : "radius_behind_yaw",
    tail_radius,
    radius_);

  if (tail_relax) {
    // Short remaining path: strip by distance only (first goal within tail_radius).
    for (size_t i = 0; i < n; ++i) {
      const geometry_msgs::msg::PoseStamped & cand = queue[i];
      const double dist = euclidean_distance(cand.pose, robot_pose.pose);
      if (dist > tail_radius) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "RemovePassedGoals: tail pass, goal %zu dist=%.3f m > radius=%.3f m, skip",
          i, dist, tail_radius);
        continue;
      }
      uint32_t z = 0;
      const bool has_z = tryGoalZ(cand, z);
      if (i > 0) {
        queue.erase(queue.begin(), queue.begin() + i + 1);
      } else {
        queue.erase(queue.begin());
      }
      maybeRecordPassed(z, has_z, dist, dist_lim, "tail");
      break;
    }
  } else {
    // Full check: front goal must be in viapoint radius, behind robot, and yaw-aligned.
    while (!queue.empty()) {
      const geometry_msgs::msg::PoseStamped & front = queue.front();
      const double dist = euclidean_distance(front.pose, robot_pose.pose);
      if (dist > radius_) {
        break;
      }

      geometry_msgs::msg::PoseStamped front_in_base;
      try {
        if (front.header.frame_id == robot_base_frame_) {
          front_in_base = front;
          front_in_base.header.frame_id = robot_base_frame_;
        } else {
          const geometry_msgs::msg::TransformStamped tf_to_base = tf_->lookupTransform(
            robot_base_frame_, front.header.frame_id, tf2::TimePointZero, tf_tol);
          tf2::doTransform(front, front_in_base, tf_to_base);
        }
      } catch (const tf2::TransformException &) {
        break;
      }

      if (front_in_base.pose.position.x > behind_x) {
        break;
      }

      if (check_yaw) {
        const double dyaw = wrapYaw(tf2::getYaw(front.pose.orientation) - robot_yaw);
        if (std::fabs(dyaw) > yaw_lim) {
          // Yaw enabled but alignment failed: do not strip or record this front goal.
          RCLCPP_DEBUG(
            node_->get_logger(),
            "RemovePassedGoals: yaw |dyaw|=%.4f > %.4f rad, skip strip & index for front z=%.3f",
            std::fabs(dyaw),
            yaw_lim,
            front.pose.position.z);
          break;
        }
      }

      uint32_t z = 0;
      const bool has_z = tryGoalZ(front, z);
      queue.erase(queue.begin());
      maybeRecordPassed(z, has_z, dist, dist_lim, "radius/behind/yaw");
    }
  }
}

void RemovePassedGoals::publishDebugPlan(const Goals & queue)
{
  if (queue.empty()) {
    return;
  }
  nav_msgs::msg::Path viz;
  viz.header.frame_id = "map";
  viz.header.stamp = queue.front().header.stamp;
  viz.poses = std::vector<geometry_msgs::msg::PoseStamped>(queue.begin(), queue.end());
  for (auto & pose : viz.poses) {
    pose.pose.position.z = 0.0;
  }
  pub_removed_plan_->publish(viz);
}

void RemovePassedGoals::markTerminalIfNeeded(
  const Goals & gpp_goals,
  const Goals & mission,
  double match_xy,
  double match_yaw)
{
  if (terminal_in_window_ || mission.empty()) {
    return;
  }

  const auto & terminal = mission.back();
  uint32_t z_term = 0;
  const bool term_unindexed = !tryGoalZ(terminal, z_term);
  bool in_window = false;
  if (term_unindexed) {
    for (const auto & gpp : gpp_goals) {
      if (posesMatch(gpp.pose, terminal.pose, match_xy, match_yaw)) {
        in_window = true;
        break;
      }
    }
  } else {
    in_window = z_end_ >= z_term;
    if (!in_window) {
      for (const auto & gpp : gpp_goals) {
        uint32_t z_gpp = 0;
        if (tryGoalZ(gpp, z_gpp) && z_gpp == z_term) {
          in_window = true;
          break;
        }
      }
    }
  }
  if (!in_window) {
    return;
  }

  terminal_in_window_ = true;
  if (term_unindexed) {
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoals: terminal unindexed goal (z=-1) entered gpp window, freeze stamps");
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoals: terminal goal z=%u entered gpp window, freeze stamps",
      static_cast<unsigned int>(z_term));
  }
}

BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  spinSubs();

  Goals input_goals;
  getInput("input_goals", input_goals);
  if (input_goals.empty()) {
    resetOnEmptyInput();
    return BT::NodeStatus::SUCCESS;
  }

  reloadMissionIfChanged(input_goals);

  bool enable_backward = false;
  getInput("enable_backward_mode", enable_backward);
  publishEnableBackward(enable_backward);

  Goals & queue = mission_goals_;

  // New front stamp (preempt / refreshed head) invalidates recorded passed indexes.
  if (queue.size() > 1) {
    const double front_stamp_s =
      queue.front().header.stamp.sec +
      queue.front().header.stamp.nanosec / 1e9;
    if (prev_front_stamp_s_ == 0.0) {
      prev_front_stamp_s_ = front_stamp_s;
    } else if (front_stamp_s - prev_front_stamp_s_ > 1e-1) {
      prev_front_stamp_s_ = front_stamp_s;
      passed_indexes_.clear();
    }
  }

  cb_executor_.spin_some();

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *(node_->get_clock()),
      2000,
      "RemovePassedGoals: getCurrentPose failed %s -> %s",
      global_frame_.c_str(), robot_base_frame_.c_str());
    setOutput("output_goals", queue);
    return BT::NodeStatus::SUCCESS;
  }

  double min_vx = -1.0;
  getInput("pass_check_min_odom_linear_x_mps", min_vx);
  double vx = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    vx = odom_vx_;
  }
  const bool odom_blocks = min_vx >= 0.0 && std::fabs(vx) < min_vx;
  if (odom_blocks) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "RemovePassedGoals: skip pass check, |odom_vx|=%.4f < min %.4f m/s",
      vx, min_vx);
  }

  if (gpp_emitted_ && !odom_blocks) {
    const uint32_t n_passed_before = passed_indexes_.size();
    stripPassedGoals(queue, robot_pose);
    if (passed_indexes_.size() > n_passed_before) {
      capella_ros_msg::msg::PassedPosesIndex msg;
      msg.indexes = passed_indexes_;
      pub_passed_indexes_->publish(msg);
      RCLCPP_DEBUG(
        node_->get_logger(),
        "RemovePassedGoals: published passed_pose_indexes count=%zu",
        static_cast<size_t>(passed_indexes_.size()));
    }
  } else if (!gpp_emitted_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "RemovePassedGoals: skip pass check, waiting for first gpp emit");
  }

  publishDebugPlan(queue);

  const std::uint64_t tail_fp = tailStampFp(queue);
  if (tail_fp != queue_tail_fp_) {
    queue_tail_fp_ = tail_fp;
    window_inited_ = false;
    RCLCPP_DEBUG(
      node_->get_logger(),
      "RemovePassedGoals: queue tail stamp changed, reset window fp=%lx",
      static_cast<unsigned long>(tail_fp));
  }

  double replan_arc = 5.0;
  getInput("replan_trigger_arc_m", replan_arc);

  double teb_timeout = 10.0;
  getInput("teb_message_timeout_s", teb_timeout);
  if (teb_timeout < 0.05) {
    teb_timeout = 0.05;
  }

  const rclcpp::Time now = node_->get_clock()->now();

  nav_msgs::msg::Path::SharedPtr teb_latest;
  rclcpp::Time teb_time;
  nav_msgs::msg::Path::SharedPtr teb_cached;
  {
    std::lock_guard<std::mutex> lock(teb_mutex_);
    teb_latest = latest_teb_plan_;
    teb_time = last_teb_time_;
    teb_cached = cached_teb_plan_;
  }

  const bool have_teb_time = teb_time.nanoseconds() > 0;
  const double teb_age = have_teb_time
    ? (now - teb_time).seconds()
    : std::numeric_limits<double>::infinity();
  const bool teb_fresh = have_teb_time && teb_age <= teb_timeout;

  const nav_msgs::msg::Path * teb_eff = nullptr;
  if (teb_fresh && teb_latest && teb_latest->poses.size() >= 2) {
    teb_eff = teb_latest.get();
  } else if (teb_cached && teb_cached->poses.size() >= 2) {
    teb_eff = teb_cached.get();
  }

  constexpr double k_min_len = 1e-6;
  double teb_len = 0.0;
  if (teb_eff) {
    teb_len = nav2_util::geometry_utils::calculate_path_length(*teb_eff);
  }
  const nav_msgs::msg::Path * teb_ok =
    (teb_eff && teb_len > k_min_len) ? teb_eff : nullptr;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "RemovePassedGoals: teb age=%.6f s fresh=%s replan_arc=%.3f m timeout=%.3f s "
    "rx_since_emit=%s teb_len=%.6f m usable=%s",
    teb_age,
    teb_fresh ? "yes" : "no",
    replan_arc,
    teb_timeout,
    teb_rx_since_emit_ ? "yes" : "no",
    teb_len,
    teb_ok ? "yes" : "no");

  bool window_advanced = false;
  if (!window_inited_) {
    initWindow(queue);
    window_inited_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoals: init gpp window z=[%u,%u] mission_goals=%zu",
      z_begin_, z_end_, queue.size());
  } else if (teb_ok && teb_ok->poses.size() >= 2) {
    const double teb_from_first = tebLenFromFirst(*teb_ok);
    if (teb_from_first <= replan_arc) {
      advanceWindow(queue);
      window_advanced = true;
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()),
        5000,
        "RemovePassedGoals: advance gpp window, teb_len=%.3f m <= %.3f m -> z=[%u,%u]",
        teb_from_first, replan_arc, z_begin_, z_end_);
    }
  }

  Goals gpp_window = filterByZSpan(queue);
  if (gpp_window.empty() && !queue.empty()) {
    initWindow(queue);
    gpp_window = filterByZSpan(queue);
  }

  double match_xy = 0.2;
  getInput("gpp_goal_pose_match_xy_m", match_xy);
  if (match_xy < 1e-6) {
    match_xy = 1e-6;
  }
  double match_yaw = -1.0;
  getInput("gpp_goal_pose_match_yaw_rad", match_yaw);

  Goals gpp_pruned = pruneByMissionPose(gpp_window, queue, match_xy, match_yaw);
  const std::uint64_t gpp_fp = gppWindowFp(gpp_pruned, tail_fp);

  // Keep stamps monotonic: prune copies mission poses that may be older than last emit.
  // On advance (before terminal freeze), assign a fresh stamp so the controller replans.
  const bool fresh_stamp = window_advanced && !terminal_in_window_;
  applyMonotonicStamps(gpp_pruned, now, fresh_stamp);
  if (fresh_stamp) {
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoals: refresh gpp goal stamp count=%zu",
      gpp_pruned.size());
  }

  if (!gpp_pruned.empty()) {
    markTerminalIfNeeded(gpp_pruned, queue, match_xy, match_yaw);

    setOutput("output_gpp_goals", gpp_pruned);
    last_gpp_goals_ = gpp_pruned;
    last_gpp_fp_ = gpp_fp;
    gpp_emitted_ = true;
    last_gpp_emit_time_ = now;

    if (window_advanced) {
      clearTebBuffers();
      teb_rx_since_emit_ = false;
    }

    RCLCPP_DEBUG(
      node_->get_logger(),
      "RemovePassedGoals: emitted output_gpp_goals count=%zu window_fp=%lx advance=%s",
      gpp_pruned.size(),
      static_cast<unsigned long>(gpp_fp),
      window_advanced ? "yes" : "no");
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "RemovePassedGoals: prune removed all goals (was %zu), keep previous gpp_goals count=%zu",
      gpp_window.size(),
      last_gpp_goals_.size());
  }

  if (!queue.empty()) {
    setOutput("output_goals", queue);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
