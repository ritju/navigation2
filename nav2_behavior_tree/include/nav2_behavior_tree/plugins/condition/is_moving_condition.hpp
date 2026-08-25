// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MOVING_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MOVING_CONDITION_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Moving uses a rolling window. After a not-moving FAILURE the window is
 * cleared and a new duration must elapse before another FAILURE; ticks during
 * that rearm interval return SUCCESS unless motion is detected again.
 * A frame is moving iff |vx| > min_linear_vel OR |wz| > min_angular_vel.
 */
class IsMovingCondition : public BT::ConditionNode
{
public:
  IsMovingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsMovingCondition() = delete;
  ~IsMovingCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("odom_topic", "/odom", "Odometry topic"),
      BT::InputPort<double>("duration", 5.0, "Window / rearm length in seconds"),
      BT::InputPort<double>("min_linear_vel", 0.1, "Abs linear.x threshold (m/s)"),
      BT::InputPort<double>("min_angular_vel", 0.1, "Abs angular.z threshold (rad/s)"),
      BT::InputPort<double>(
        "min_moving_ratio", 0.1,
        "Min fraction of valid window samples that exceed velocity thresholds"),
      BT::InputPort<int>("min_samples", 5, "Min valid odom samples required in the window"),
      BT::InputPort<double>("odom_timeout", 0.5, "Max age of last valid odom (s) before FAILURE"),
      BT::InputPort<double>(
        "max_dt_jump", 1.0, "Clear the window if consecutive odom stamps jump by more than this (s)"),
    };
  }

private:
  enum class Phase
  {
    kTracking,
    kRearm,
  };

  struct Sample
  {
    rclcpp::Time stamp;
    bool moving;
  };

  struct Params
  {
    double duration{5.0};
    double min_linear_vel{0.1};
    double min_angular_vel{0.1};
    double min_moving_ratio{0.3};
    int min_samples{5};
    double odom_timeout{0.5};
    double max_dt_jump{1.0};
  };

  struct RatioResult
  {
    int n{0};
    int moving_n{0};
    double ratio{0.0};
    bool enough{false};
    bool is_moving{false};
  };

  void onOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg);
  void refreshParams();
  void pruneLocked(const rclcpp::Time & now_stamp);
  void beginRearmLocked(const rclcpp::Time & now, const char * reason, const RatioResult & ratio);
  RatioResult ratioLocked() const;
  static bool finiteTwist(const nav_msgs::msg::Odometry & msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex mutex_;
  Params params_;
  std::deque<Sample> samples_;
  rclcpp::Time last_stamp_;
  rclcpp::Time rearm_start_;
  bool have_stamp_{false};
  Phase phase_{Phase::kTracking};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MOVING_CONDITION_HPP_
