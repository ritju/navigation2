// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_moving_condition.hpp"

using namespace std::chrono_literals;

class IsMovingTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::IsMovingCondition>("is_moving", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::IsMovingCondition> bt_node_;
};

std::shared_ptr<nav2_behavior_tree::IsMovingCondition>
IsMovingTestFixture::bt_node_ = nullptr;

static void publishOdom(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
  const rclcpp::Node::SharedPtr & node,
  double vx,
  double wz,
  double t_offset_s)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node->now() + rclcpp::Duration::from_seconds(t_offset_s);
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.angular.z = wz;
  pub->publish(msg);
}

TEST_F(IsMovingTestFixture, no_odom_is_not_moving)
{
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(IsMovingTestFixture, stopped_is_failure)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.0, 0.0, i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(IsMovingTestFixture, rearm_holds_success_until_duration)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.0, 0.0, i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsMovingTestFixture, linear_or_angular_is_success)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.3, 0.0, i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  bt_node_.reset();
  bt_node_ = std::make_shared<nav2_behavior_tree::IsMovingCondition>("is_moving", *config_);
  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.0, 0.3, i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsMovingTestFixture, motion_during_rearm_returns_to_tracking)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.0, 0.0, i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  for (int i = 0; i < 5; ++i) {
    publishOdom(odom_pub, node_, 0.3, 0.0, 0.3 + i * 0.05);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsMovingTestFixture, nan_is_ignored)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node_->now();
  msg.twist.twist.linear.x = std::nan("");
  odom_pub->publish(msg);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
