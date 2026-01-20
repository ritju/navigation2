// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <chrono>
#include <thread>

#include "nav2_behavior_tree/plugins/condition/is_collision.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

using namespace std::chrono_literals;

namespace nav2_behavior_tree
{

namespace
{

// 原先用于详细调试的辅助函数（格式化位姿/footprint/cost 名称）在当前精简版本中不再使用，
// 如果后续需要重新打开详细调试，可以从 git/history 或系统安装的 nav2 源码中恢复类似实现。

}  // namespace

IsCollisionCondition::IsCollisionCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  has_odom_(false),
  has_cmd_vel_(false),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  transform_tolerance_(0.1),
  simulate_ahead_time_(2.0),
  cycle_frequency_(10.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread_ = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  // Subscribe to odom topic
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsCollisionCondition::odomCallback, this, std::placeholders::_1),
    sub_option);

  // Subscribe to cmd_vel topic
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsCollisionCondition::cmdVelCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Constructor: Initialized IsCollisionCondition BT node");
  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Subscribed to topics: odom, cmd_vel");
  RCLCPP_INFO_ONCE(node_->get_logger(), "[IsCollision] Waiting on odometry and cmd_vel");

  bool use_sim_time = false;
  node_->get_parameter_or("use_sim_time", use_sim_time, false);
  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Time source: use_sim_time=%d now=%.3f",
    use_sim_time, node_->now().seconds());
}

IsCollisionCondition::~IsCollisionCondition()
{
  RCLCPP_INFO(node_->get_logger(), "Shutting down IsCollisionCondition BT node");
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

void IsCollisionCondition::initialize()
{
  if (initialized_) {
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Already initialized, skipping");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Starting initialization...");

  // Get tf_buffer from blackboard
  try {
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Got tf_buffer from blackboard");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "[IsCollision] Failed to get tf_buffer: %s", e.what());
    return;
  }

  // Get parameters
  nav2_util::declare_parameter_if_not_declared(
    node_, "global_frame", rclcpp::ParameterValue(global_frame_));
  nav2_util::declare_parameter_if_not_declared(
    node_, "robot_base_frame", rclcpp::ParameterValue(robot_base_frame_));
  nav2_util::declare_parameter_if_not_declared(
    node_, "transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node_, "simulate_ahead_time", rclcpp::ParameterValue(simulate_ahead_time_));
  nav2_util::declare_parameter_if_not_declared(
    node_, "cycle_frequency", rclcpp::ParameterValue(cycle_frequency_));

  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  node_->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  node_->get_parameter("cycle_frequency", cycle_frequency_);

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Parameters: global_frame=%s, robot_base_frame=%s, "
    "simulate_ahead_time=%.2f, cycle_frequency=%.2f",
    global_frame_.c_str(), robot_base_frame_.c_str(), simulate_ahead_time_, cycle_frequency_);

  // Initialize collision checker components
  std::string costmap_topic, footprint_topic;
  nav2_util::declare_parameter_if_not_declared(
    node_, "local_costmap/costmap_topic", rclcpp::ParameterValue("/local_costmap/costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node_, "local_costmap/published_footprint_topic",
    rclcpp::ParameterValue("local_costmap/published_footprint"));
  node_->get_parameter_or("local_costmap/costmap_topic", costmap_topic, std::string("/local_costmap/costmap_raw"));
  node_->get_parameter_or("local_costmap/published_footprint_topic", footprint_topic,
    std::string("/local_costmap/published_footprint"));

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Costmap topic: %s, Footprint topic: %s",
    costmap_topic.c_str(), footprint_topic.c_str());

  // Check if costmap topic exists
  auto topic_list = node_->get_topic_names_and_types();
  bool costmap_topic_exists = false;
  for (const auto & topic_info : topic_list) {
    if (topic_info.first == costmap_topic) {
      costmap_topic_exists = true;
      RCLCPP_INFO(node_->get_logger(), "[IsCollision] Found costmap topic: %s (types: %s)",
        topic_info.first.c_str(),
        [&topic_info]() {
          std::string types_str;
          for (const auto & type : topic_info.second) {
            if (!types_str.empty()) types_str += ", ";
            types_str += type;
          }
          return types_str;
        }().c_str());
      break;
    }
  }
  
  if (!costmap_topic_exists) {
    RCLCPP_WARN(node_->get_logger(), "[IsCollision] Costmap topic '%s' not found! Available topics:", costmap_topic.c_str());
    // List all available topics that contain "costmap"
    for (const auto & topic_info : topic_list) {
      if (topic_info.first.find("costmap") != std::string::npos) {
        RCLCPP_WARN(node_->get_logger(), "[IsCollision]   - %s", topic_info.first.c_str());
      }
    }
    RCLCPP_WARN(node_->get_logger(), "[IsCollision] Will continue anyway, but costmap may not be available.");
  }

  try {
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node_, costmap_topic);
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] CostmapSubscriber created for topic: %s", costmap_topic.c_str());
    
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_, footprint_topic, *tf_, robot_base_frame_, transform_tolerance_);
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] FootprintSubscriber created for topic: %s", footprint_topic.c_str());
    
    collision_checker_ = std::make_unique<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_, "is_collision_condition");
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Collision checker created successfully");
    
    // Wait for costmap message to arrive (transient_local QoS requires waiting)
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Waiting for costmap message to arrive...");
    const int max_wait_attempts = 50;  // 5 seconds total (50 * 100ms)
    const int wait_interval_ms = 100;
    bool costmap_received = false;
    
    for (int i = 0; i < max_wait_attempts; ++i) {
      try {
        auto test_costmap = costmap_sub_->getCostmap();
        if (test_costmap) {
          RCLCPP_INFO(node_->get_logger(), "[IsCollision] Costmap received! Size: %dx%d, Resolution: %.3f",
            test_costmap->getSizeInCellsX(), test_costmap->getSizeInCellsY(), test_costmap->getResolution());
          costmap_received = true;
          break;
        }
      } catch (const std::exception & e) {
        // Costmap not available yet, continue waiting
        if (i % 10 == 0) {  // Log every second
          RCLCPP_INFO(node_->get_logger(), "[IsCollision] Still waiting for costmap... (attempt %d/%d, error: %s)", 
            i + 1, max_wait_attempts, e.what());
        }
      }
      
      // Spin to process callbacks - this is critical for receiving messages
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_interval_ms));
    }
    
    if (!costmap_received) {
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] Costmap not received after %d attempts (%.1f seconds). "
        "It may not be published yet. Will continue and check again when needed.",
        max_wait_attempts, max_wait_attempts * wait_interval_ms / 1000.0);
      
      // Additional diagnostics
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] === Costmap Reception Diagnostics ===");
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] Subscribed topic: %s", costmap_topic.c_str());
      
      // Check if topic exists (which means publisher exists)
      auto topic_list = node_->get_topic_names_and_types();
      bool topic_exists = false;
      for (const auto & topic_info : topic_list) {
        if (topic_info.first == costmap_topic) {
          topic_exists = true;
          std::string types_str;
          for (const auto & type : topic_info.second) {
            if (!types_str.empty()) types_str += ", ";
            types_str += type;
          }
          RCLCPP_WARN(node_->get_logger(), "[IsCollision] Topic exists: YES [%s]", types_str.c_str());
          RCLCPP_WARN(node_->get_logger(), "[IsCollision] Note: Topic exists but message not received. "
            "This may mean the publisher hasn't sent a message yet, or there's a QoS mismatch.");
          break;
        }
      }
      if (!topic_exists) {
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Topic does not exist: NO");
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Make sure local_costmap is running and activated.");
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Available costmap-related topics:");
        bool found_any = false;
        for (const auto & topic_info : topic_list) {
          if (topic_info.first.find("costmap") != std::string::npos) {
            found_any = true;
            std::string types_str;
            for (const auto & type : topic_info.second) {
              if (!types_str.empty()) types_str += ", ";
              types_str += type;
            }
            RCLCPP_WARN(node_->get_logger(), "[IsCollision]   - %s [%s]", topic_info.first.c_str(), types_str.c_str());
          }
        }
        if (!found_any) {
          RCLCPP_WARN(node_->get_logger(), "[IsCollision]   (none found)");
        }
      }
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] =====================================");
    }
    
    // Wait for footprint message to arrive
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Waiting for footprint message to arrive...");
    bool footprint_received = false;
    
    for (int i = 0; i < max_wait_attempts; ++i) {
      std::vector<geometry_msgs::msg::Point> test_footprint;
      std_msgs::msg::Header test_header;
      test_header.stamp = node_->get_clock()->now();
      if (footprint_sub_->getFootprintInRobotFrame(test_footprint, test_header)) {
        RCLCPP_INFO(node_->get_logger(), "[IsCollision] Footprint received! Points: %zu, frame: %s",
          test_footprint.size(), test_header.frame_id.c_str());
        footprint_received = true;
        break;
      }
      
      if (i % 10 == 0) {  // Log every second
        RCLCPP_INFO(node_->get_logger(), "[IsCollision] Still waiting for footprint... (attempt %d/%d)", 
          i + 1, max_wait_attempts);
      }
      
      // Spin to process callbacks
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_interval_ms));
    }
    
    if (!footprint_received) {
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] Footprint not received after %d attempts (%.1f seconds). "
        "It may not be published yet. Will continue and check again when needed.",
        max_wait_attempts, max_wait_attempts * wait_interval_ms / 1000.0);
      
      // Additional diagnostics
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] === Footprint Reception Diagnostics ===");
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] Subscribed topic: %s", footprint_topic.c_str());
      
      // Check if topic exists
      auto topic_list = node_->get_topic_names_and_types();
      bool topic_exists = false;
      for (const auto & topic_info : topic_list) {
        if (topic_info.first == footprint_topic || 
            topic_info.first == "/" + footprint_topic) {
          topic_exists = true;
          std::string types_str;
          for (const auto & type : topic_info.second) {
            if (!types_str.empty()) types_str += ", ";
            types_str += type;
          }
          RCLCPP_WARN(node_->get_logger(), "[IsCollision] Topic exists: YES [%s]", types_str.c_str());
          RCLCPP_WARN(node_->get_logger(), "[IsCollision] Note: Topic exists but message not received. "
            "This may mean the publisher hasn't sent a message yet.");
          break;
        }
      }
      if (!topic_exists) {
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Topic does not exist: NO");
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Make sure local_costmap is running and activated.");
        RCLCPP_WARN(node_->get_logger(), "[IsCollision] Available footprint-related topics:");
        bool found_any = false;
        for (const auto & topic_info : topic_list) {
          if (topic_info.first.find("footprint") != std::string::npos) {
            found_any = true;
            std::string types_str;
            for (const auto & type : topic_info.second) {
              if (!types_str.empty()) types_str += ", ";
              types_str += type;
            }
            RCLCPP_WARN(node_->get_logger(), "[IsCollision]   - %s [%s]", topic_info.first.c_str(), types_str.c_str());
          }
        }
        if (!found_any) {
          RCLCPP_WARN(node_->get_logger(), "[IsCollision]   (none found)");
        }
      }
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] ======================================");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "[IsCollision] Failed to create collision checker: %s", e.what());
    return;
  }

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Initialization completed successfully");
}

void IsCollisionCondition::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "[IsCollision] Got first odometry message");
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_odom_ = msg;
  has_odom_ = true;
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[IsCollision] Odom: vx=%.3f, vy=%.3f, vz=%.3f, wx=%.3f, wy=%.3f, wz=%.3f",
  //   msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
  //   msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

void IsCollisionCondition::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "[IsCollision] Got first cmd_vel message");
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_cmd_vel_ = msg;
  has_cmd_vel_ = true;
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[IsCollision] CmdVel: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f",
  //   msg->linear.x, msg->linear.y, msg->angular.z);
}

BT::NodeStatus IsCollisionCondition::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[IsCollision] tick() called");

  if (!initialized_) {
    RCLCPP_INFO(node_->get_logger(), "[IsCollision] Not initialized, calling initialize()");
    initialize();
  }

  if (!initialized_) {
    RCLCPP_WARN(node_->get_logger(), "[IsCollision] Initialization failed, returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // Get current pose from TF
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "[IsCollision] Failed to get current pose from TF (frame: %s -> %s)",
      robot_base_frame_.c_str(), global_frame_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Current pose: x=%.3f, y=%.3f, theta=%.3f",
    current_pose.pose.position.x, current_pose.pose.position.y,
    tf2::getYaw(current_pose.pose.orientation));

  // Get latest velocity data
  geometry_msgs::msg::Twist cmd_vel;
  bool using_cmd_vel = false;
  (void)using_cmd_vel;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Combine odom velocity and cmd_vel for prediction
    if (has_cmd_vel_ && latest_cmd_vel_) {
      // Use cmd_vel if available (command velocity)
      cmd_vel = *latest_cmd_vel_;
      using_cmd_vel = true;
      RCLCPP_INFO(node_->get_logger(), "[IsCollision] Using cmd_vel: linear.x=%.3f, angular.z=%.3f",
        cmd_vel.linear.x, cmd_vel.angular.z);
    } else if (has_odom_ && latest_odom_) {
      // Fall back to odom velocity if cmd_vel is not available
      cmd_vel = latest_odom_->twist.twist;
      using_cmd_vel = false;
      RCLCPP_INFO(node_->get_logger(), "[IsCollision] Using odom velocity: linear.x=%.3f, angular.z=%.3f",
        cmd_vel.linear.x, cmd_vel.angular.z);
    } else {
      // No velocity data available yet
      RCLCPP_WARN(node_->get_logger(), "[IsCollision] No velocity data available (has_odom=%d, has_cmd_vel=%d)",
        has_odom_, has_cmd_vel_);
      return BT::NodeStatus::FAILURE;
    }
  }

  // Convert current pose to Pose2D
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] Checking collision with pose2d: x=%.3f, y=%.3f, theta=%.3f",
    pose2d.x, pose2d.y, pose2d.theta);

  // Check for collision using predicted pose
  bool collision_free = isCollisionFree(cmd_vel, pose2d);
  if (!collision_free) {
    RCLCPP_WARN(node_->get_logger(), "[IsCollision] COLLISION DETECTED! Robot will collide. "
      "Pose: (%.3f, %.3f, %.3f), Vel: linear.x=%.3f, angular.z=%.3f",
      pose2d.x, pose2d.y, pose2d.theta, cmd_vel.linear.x, cmd_vel.angular.z);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[IsCollision] No collision detected, returning SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

bool IsCollisionCondition::isCollisionFree(
  const geometry_msgs::msg::Twist & cmd_vel,
  const geometry_msgs::msg::Pose2D & current_pose)
{
  if (!collision_checker_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsCollision] Collision checker not initialized, treating pose as collision-free");
    return true;
  }

  // Process pending callbacks to ensure CostmapSubscriber receives latest updates
  // Note: CostmapSubscriber's callback is in the default callback group (not callback_group_),
  // so we need to spin the node to process its callbacks. This ensures we get the latest
  // costmap with updated origin when using rolling window.
  rclcpp::spin_some(node_);

  // Print costmap origin from CostmapSubscriber (for debugging/verification)
  // This ensures we're using the same costmap source as the collision checker
  try {
    if (costmap_sub_) {
      auto costmap = costmap_sub_->getCostmap();
      if (costmap) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[IsCollision] Costmap origin: (%.3f, %.3f) size=(%u,%u) res=%.3f",
          costmap->getOriginX(), costmap->getOriginY(),
          costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
          costmap->getResolution());
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[IsCollision] CostmapSubscriber returned null costmap");
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsCollision] Failed to get costmap for origin logging: %s", ex.what());
  }

  // Cache costmap once per call (avoid calling getCostmap() inside the loop)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cached_costmap;
  try {
    if (costmap_sub_) {
      cached_costmap = costmap_sub_->getCostmap();
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsCollision] Failed to get cached costmap: %s", ex.what());
  }

  // 使用简单的非完整小车模型，根据当前 cmd_vel 从 current_pose 向前模拟一段时间，
  // 在每个采样点调用 CostmapTopicCollisionChecker 检查碰撞。
  const double dt = 1.0 / cycle_frequency_;
  const int steps = std::max(1, static_cast<int>(simulate_ahead_time_ * cycle_frequency_));

  geometry_msgs::msg::Pose2D pose = current_pose;

  for (int i = 0; i < steps; ++i) {
    const double v = cmd_vel.linear.x;
    const double w = cmd_vel.angular.z;

    // 基于当前朝向积分位姿
    pose.x += v * std::cos(pose.theta) * dt;
    pose.y += v * std::sin(pose.theta) * dt;
    pose.theta += w * dt;

    // 在当前预测位姿处打印 footprint 的代价值
    try {
      if (costmap_sub_ && footprint_sub_) {
        if (cached_costmap) {
          // 获取当前机器人在 base_link 下的 footprint（未旋转，以机器人坐标系为原点）
          std::vector<geometry_msgs::msg::Point> footprint;
          std_msgs::msg::Header footprint_header;
          if (footprint_sub_->getFootprintInRobotFrame(footprint, footprint_header)) {
            nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
              fp_checker(cached_costmap);
            const double fp_cost = fp_checker.footprintCostAtPose(
              pose.x, pose.y, pose.theta, footprint);

            RCLCPP_INFO(
              node_->get_logger(),
              "[IsCollision] Step %d/%d pose=(%.3f, %.3f, %.3f) footprint_cost=%.1f",
              i + 1, steps,
              pose.x, pose.y, pose.theta,
              fp_cost);
          } else {
            RCLCPP_WARN(
              node_->get_logger(),
              "[IsCollision] Step %d/%d: footprint not available in robot frame, "
              "cannot compute footprint cost",
              i + 1, steps);
          }
        } else {
          RCLCPP_WARN(
            node_->get_logger(),
            "[IsCollision] Step %d/%d: cached costmap is null when "
            "querying footprint cost",
            i + 1, steps);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[IsCollision] Exception while querying footprint cost at step %d/%d: %s",
        i + 1, steps, ex.what());
    }

    const bool fetch_costmap_and_footprint = (i == 0);
    const bool free = collision_checker_->isCollisionFree(pose, fetch_costmap_and_footprint);

    if (!free) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[IsCollision] Predicted collision at step %d/%d: pose=(%.3f, %.3f, %.3f), "
        "cmd_vel: linear.x=%.3f angular.z=%.3f",
        i + 1, steps,
        pose.x, pose.y, pose.theta,
        cmd_vel.linear.x, cmd_vel.angular.z);
      return false;
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[IsCollision] Trajectory of %.2fs ahead is collision-free (steps=%d, dt=%.3f)",
    simulate_ahead_time_, steps, dt);

  return true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsCollisionCondition>("IsCollision");
}
