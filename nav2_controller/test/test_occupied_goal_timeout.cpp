#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <nav2_controller/controller_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using FollowPath = nav2_msgs::action::FollowPath;
using namespace std::chrono_literals;

// Inject the typed wait timeout inside the real computeControl action callback.
class TimeoutController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path &) override
  {throw nav2_core::OccupiedGoalTimeout("occupied goal wait expired");}
  void setSpeedLimit(const double &, const bool &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override
  {throw nav2_core::OccupiedGoalTimeout("occupied goal wait expired");}
};

class TimeoutServer : public nav2_controller::ControllerServer
{
public:
  void startTestAction()
  {
    controllers_["timeout"] = std::make_shared<TimeoutController>();
    goal_checkers_["goal"] = nullptr;  // setPlan throws before any goal-checker use.
    failure_tolerance_ = 5.0;
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("timeout_zero_test", 10);
    vel_publisher_->on_activate();
    action_server_ = std::make_unique<nav2_util::SimpleActionServer<FollowPath>>(
      shared_from_this(), "occupied_goal_timeout_test", [this]() {computeControl();},
      nullptr, 500ms, true);
    action_server_->activate();
  }
  bool hasZeroSubscriber() const {return vel_publisher_->get_subscription_count() > 0;}
};

TEST(OccupiedGoalTimeout, completesFollowPathSuccessfullyAndPublishesZero)
{
  rclcpp::init(0, nullptr);
  {
    auto server = std::make_shared<TimeoutServer>();
    server->startTestAction();
    auto client_node = std::make_shared<rclcpp::Node>("occupied_goal_timeout_client");
    bool zero_received = false;
    auto zero_sub = client_node->create_subscription<geometry_msgs::msg::Twist>(
      "timeout_zero_test", 10, [&](geometry_msgs::msg::Twist::SharedPtr msg) {
        zero_received = msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->angular.z == 0.0;
      });
    auto client = rclcpp_action::create_client<FollowPath>(client_node, "occupied_goal_timeout_test");
    ASSERT_TRUE(client->wait_for_action_server(2s));
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!server->hasZeroSubscriber() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(server->hasZeroSubscriber());
    FollowPath::Goal goal;
    goal.controller_id = "timeout";
    goal.goal_checker_id = "goal";
    goal.path.header.frame_id = "map";
    goal.path.poses.resize(1);
    goal.path.poses.front().pose.orientation.w = 1.0;
    auto accepted = client->async_send_goal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, accepted, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_NE(accepted.get(), nullptr);
    auto result = client->async_get_result(accepted.get());
    ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, result, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    rclcpp::spin_some(client_node);
    EXPECT_TRUE(zero_received);
  }
  rclcpp::shutdown();
}
