// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_behavior_tree/plugins/condition/is_moving_condition.hpp"

#include <cmath>

namespace nav2_behavior_tree
{

IsMovingCondition::IsMovingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  last_stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  rearm_start_ = last_stamp_;
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread_ = std::thread(
    [this]() {callback_group_executor_.spin();});

  refreshParams();
  std::string odom_topic = "/odom";
  getInput("odom_topic", odom_topic);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsMovingCondition::onOdomReceived, this, std::placeholders::_1),
    sub_option);
}

IsMovingCondition::~IsMovingCondition()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

bool IsMovingCondition::finiteTwist(const nav_msgs::msg::Odometry & msg)
{
  const auto & v = msg.twist.twist;
  return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.linear.z) &&
         std::isfinite(v.angular.x) && std::isfinite(v.angular.y) && std::isfinite(v.angular.z);
}

void IsMovingCondition::refreshParams()
{
  Params next = params_;
  getInput("duration", next.duration);
  getInput("min_linear_vel", next.min_linear_vel);
  getInput("min_angular_vel", next.min_angular_vel);
  getInput("min_moving_ratio", next.min_moving_ratio);
  getInput("min_samples", next.min_samples);
  getInput("odom_timeout", next.odom_timeout);
  getInput("max_dt_jump", next.max_dt_jump);

  if (next.duration <= 0.0) {
    next.duration = 5.0;
  }
  if (next.min_linear_vel < 0.0) {
    next.min_linear_vel = 0.0;
  }
  if (next.min_angular_vel < 0.0) {
    next.min_angular_vel = 0.0;
  }
  if (next.min_moving_ratio < 0.0) {
    next.min_moving_ratio = 0.0;
  }
  if (next.min_moving_ratio > 1.0) {
    next.min_moving_ratio = 1.0;
  }
  if (next.min_samples < 1) {
    next.min_samples = 1;
  }
  if (next.odom_timeout <= 0.0) {
    next.odom_timeout = 0.5;
  }
  if (next.max_dt_jump <= 0.0) {
    next.max_dt_jump = 1.0;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  params_ = next;
}

void IsMovingCondition::pruneLocked(const rclcpp::Time & now_stamp)
{
  const auto cutoff = now_stamp - rclcpp::Duration::from_seconds(params_.duration);
  while (!samples_.empty() && samples_.front().stamp < cutoff) {
    samples_.pop_front();
  }
}

IsMovingCondition::RatioResult IsMovingCondition::ratioLocked() const
{
  RatioResult result;
  result.n = static_cast<int>(samples_.size());
  for (const auto & sample : samples_) {
    if (sample.moving) {
      ++result.moving_n;
    }
  }
  if (result.n > 0) {
    result.ratio = static_cast<double>(result.moving_n) / static_cast<double>(result.n);
  }
  result.enough = result.n >= params_.min_samples;
  result.is_moving = result.enough && result.ratio >= params_.min_moving_ratio;
  return result;
}

void IsMovingCondition::beginRearmLocked(
  const rclcpp::Time & now, const char * reason, const RatioResult & ratio)
{
  samples_.clear();
  rearm_start_ = now;
  phase_ = Phase::kRearm;
  RCLCPP_INFO(
    node_->get_logger(),
    "IsMoving: not moving (%s), ratio=%.2f (%d/%d), start rearm for %.2f s",
    reason, ratio.ratio, ratio.moving_n, ratio.n, params_.duration);
}

void IsMovingCondition::onOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg || !finiteTwist(*msg)) {
    return;
  }

  rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    stamp = node_->now();
  }

  Params params;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params = params_;
  }

  const double vx = std::fabs(msg->twist.twist.linear.x);
  const double wz = std::fabs(msg->twist.twist.angular.z);
  const bool moving = (vx > params.min_linear_vel) || (wz > params.min_angular_vel);

  std::lock_guard<std::mutex> lock(mutex_);
  if (have_stamp_) {
    if (stamp < last_stamp_) {
      return;
    }
    const double dt = (stamp - last_stamp_).seconds();
    if (dt < 1e-6) {
      return;
    }
    if (dt > params_.max_dt_jump) {
      samples_.clear();
      if (phase_ == Phase::kRearm) {
        rearm_start_ = stamp;
        RCLCPP_WARN(
          node_->get_logger(),
          "IsMoving: odom stamp jumped %.3f s during rearm, restart rearm window", dt);
      }
    }
  }

  last_stamp_ = stamp;
  have_stamp_ = true;
  samples_.push_back(Sample{stamp, moving});
  if (phase_ == Phase::kTracking) {
    pruneLocked(stamp);
  }
}

BT::NodeStatus IsMovingCondition::tick()
{
  refreshParams();

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();

  if (!have_stamp_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsMoving: no valid odom yet, report not moving");
    return BT::NodeStatus::FAILURE;
  }

  const double age = (now - last_stamp_).seconds();
  if (age > params_.odom_timeout) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsMoving: odom stale (%.3f s > timeout %.3f s), report not moving",
      age, params_.odom_timeout);
    return BT::NodeStatus::FAILURE;
  }

  if (phase_ == Phase::kTracking) {
    pruneLocked(last_stamp_);
    const RatioResult ratio = ratioLocked();
    if (!ratio.enough) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "IsMoving: only %d/%d samples, report not moving",
        ratio.n, params_.min_samples);
      return BT::NodeStatus::FAILURE;
    }
    if (ratio.is_moving) {
      return BT::NodeStatus::SUCCESS;
    }
    beginRearmLocked(now, "rolling ratio below threshold", ratio);
    return BT::NodeStatus::FAILURE;
  }

  const RatioResult ratio = ratioLocked();
  if (ratio.is_moving) {
    phase_ = Phase::kTracking;
    RCLCPP_INFO(
      node_->get_logger(),
      "IsMoving: motion resumed during rearm, ratio=%.2f (%d/%d), back to rolling check",
      ratio.ratio, ratio.moving_n, ratio.n);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = (now - rearm_start_).seconds();
  if (elapsed < params_.duration) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "IsMoving: rearm %.2f/%.2f s, ratio=%.2f (%d/%d), hold SUCCESS until window completes",
      elapsed, params_.duration, ratio.ratio, ratio.moving_n, ratio.n);
    return BT::NodeStatus::SUCCESS;
  }

  beginRearmLocked(now, "rearm window still not moving", ratio);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsMovingCondition>("IsMoving");
}
