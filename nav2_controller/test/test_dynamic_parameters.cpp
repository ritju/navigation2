// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions())
  {
  }

  void initializeVelocityPublisher()
  {
    controller_ids_ = {"FollowPath"};
    current_controller_ = "FollowPath";
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/twist_switch_test", 10);
    vel_publisher_->on_activate();
  }
  void initializeFrontSteerStopPublisher()
  {
    auto pub = create_publisher<capella_ros_msg::msg::FrontSteerDriveCmd>(
      "/front_steer_stop_test", rclcpp::SystemDefaultsQoS());
    pub->on_activate();
    front_steer_stop_publishers_["FollowPath"] = pub;
  }
  bool hasStopSubscriber() const
  {return front_steer_stop_publishers_.at("FollowPath")->get_subscription_count() > 0;}
  bool hasVelocitySubscriber() const {return vel_publisher_->get_subscription_count() > 0;}
  void selectController(const std::string & id) {current_controller_ = id;}
  void sendVelocity(double speed)
  {
    geometry_msgs::msg::TwistStamped command;
    command.twist.linear.x = speed;
    publishVelocity(command);
  }
  void sendZero() {publishZeroVelocity();}

  // Since we cannot call configure/activate due to costmaps
  // requiring TF
  void setDynamicCallback()
  {
    auto node = shared_from_this();
    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&ControllerShim::dynamicParamsShim, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParamsShim(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    dynamicParametersCallback(parameters);
    return result;
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    controller->get_node_base_interface(), controller->get_node_topics_interface(),
    controller->get_node_graph_interface(),
    controller->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 100.0),
      rclcpp::Parameter("min_x_velocity_threshold", 100.0),
      rclcpp::Parameter("min_y_velocity_threshold", 100.0),
      rclcpp::Parameter("min_theta_velocity_threshold", 100.0),
      rclcpp::Parameter("failure_tolerance", 5.0)});

  rclcpp::spin_until_future_complete(
    controller->get_node_base_interface(),
    results);

  EXPECT_EQ(controller->get_parameter("controller_frequency").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_x_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_y_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_theta_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("failure_tolerance").as_double(), 5.0);
}

TEST(WPTest, front_steer_twist_publication_switch)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->initializeVelocityPublisher();
  auto observer = std::make_shared<rclcpp::Node>("twist_switch_observer");
  int received = 0;
  double last_speed = 0.0;
  auto sub = observer->create_subscription<geometry_msgs::msg::Twist>(
    "/twist_switch_test", 10,
    [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) {++received; last_speed = msg->linear.x;});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!controller->hasVelocitySubscriber() &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(controller->hasVelocitySubscriber());
  const auto drain = [&]() {
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
      while (std::chrono::steady_clock::now() < until) {
        rclcpp::spin_some(observer);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    };
  // Missing settings preserve the default Controller Server behavior.
  controller->sendVelocity(0.2); drain();
  ASSERT_EQ(received, 1);
  controller->declare_parameter("FollowPath.front_steer_drive_interface.mode", "front_steer_cmd");
  controller->declare_parameter("FollowPath.front_steer_drive_interface.publish_twist", false);
  controller->sendVelocity(0.3); controller->sendZero(); drain();
  EXPECT_EQ(received, 1);
  // A single configured direct controller also suppresses startup/lifecycle zeros.
  controller->selectController("");
  controller->sendZero(); drain();
  EXPECT_EQ(received, 1);
  controller->selectController("FollowPath");
  controller->set_parameter(rclcpp::Parameter("FollowPath.front_steer_drive_interface.publish_twist", true));
  controller->sendVelocity(0.4); drain();
  EXPECT_EQ(received, 2); EXPECT_DOUBLE_EQ(last_speed, 0.4);
  controller->sendZero(); drain();
  EXPECT_EQ(received, 3); EXPECT_DOUBLE_EQ(last_speed, 0.0);
  // The flag does not suppress legacy cmd_vel mode or another selected plugin.
  controller->set_parameter(rclcpp::Parameter("FollowPath.front_steer_drive_interface.publish_twist", false));
  controller->set_parameter(rclcpp::Parameter("FollowPath.front_steer_drive_interface.mode", "cmd_vel"));
  controller->sendVelocity(0.5); drain();
  EXPECT_EQ(received, 4);
  controller->set_parameter(rclcpp::Parameter("FollowPath.front_steer_drive_interface.mode", "front_steer_cmd"));
  controller->selectController("OtherController");
  controller->sendVelocity(0.1); drain();
  EXPECT_EQ(received, 5);
}

TEST(WPTest, front_steer_goal_cancel_failure_stop_ignores_twist_switch)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->initializeVelocityPublisher();
  controller->initializeFrontSteerStopPublisher();
  controller->declare_parameter("FollowPath.front_steer_drive_interface.mode", "front_steer_cmd");
  controller->declare_parameter("FollowPath.front_steer_drive_interface.publish_twist", false);
  auto observer = std::make_shared<rclcpp::Node>("front_steer_stop_observer");
  int count = 0;
  auto sub = observer->create_subscription<capella_ros_msg::msg::FrontSteerDriveCmd>(
    "/front_steer_stop_test", rclcpp::SystemDefaultsQoS(),
    [&](capella_ros_msg::msg::FrontSteerDriveCmd::ConstSharedPtr msg) {
      ++count;
      EXPECT_EQ(msg->linear_x, 0.0f); EXPECT_EQ(msg->front_steer_angle, 0.0f);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!controller->hasStopSubscriber() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(controller->hasStopSubscriber());
  // The central stop path is used by goal success, cancel and exception paths.
  controller->sendZero();
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (count == 0 && std::chrono::steady_clock::now() < until) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(count, 1);
  // Ordinary velocity output must never emit a native chassis stop.
  controller->sendVelocity(0.3);
  controller->selectController("OtherController");
  controller->sendZero();
  const auto quiet = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (std::chrono::steady_clock::now() < quiet) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(count, 1);
}
