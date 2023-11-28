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

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "capella_ros_msg/msg/single_detector.hpp"
#include "capella_ros_msg/msg/detect_result.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;

  /**
   * @brief Constructor for nav2_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
   */
  ~ControllerServer();
  bool isGoalOccupied(){
    geometry_msgs::msg::Point obstacle;
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    // RCLCPP_INFO(rclcpp::get_logger("trans"), "goal pose: %f, %f", goal_pose.pose.x,goal_pose.pose.y);
    for(unsigned int j=0;j<size_y;j++){
      for(unsigned int k=0;k<size_x;k++){
        if(costmap_->getCost(k,j) >= 253){
          costmap_->mapToWorld(k,j,obstacle.x,obstacle.y);
          double distance = sqrt((obstacle.x - goal_x)*(obstacle.x - goal_x)+(obstacle.y - goal_y)*(obstacle.y - goal_y));
          if(distance < 0.1){
            return true;
          }
        }
      }
    }
    return false;
  }
  bool isobstacleultraforward()
  {
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    // RCLCPP_INFO(rclcpp::get_logger("TEST"), "ultra_back_time  %.9f", ultra_back_time);
    if(ultra_count >= 2 && ultra_back_time <= 2){
      if (!update_time)
      {
        start_time_= steady_clock_.now();
      }
      update_time = true;
      ultra_back_time = steady_clock_.now().seconds() - start_time_.seconds();
      return true;
    }
    else{
      ultra_back_time = 0;
      update_time = false;
      return false;
    }

    // back_ultra = 0;
    // std::vector<tf2::Vector3> footprint_pose;
    // tf2::Vector3 c1(local_width_,local_height_,0);
    // unsigned int s[15][2];
    // // unsigned int m[441];
    // for (double x = 0.6; x <= 1.0; x += 0.1) {
    //   for (double y = -0.1; y <= 0.1; y += 0.1) {
    //     footprint_pose.push_back(tf2::Vector3(x, y, 0));
    //   }
    // }
    // std::vector<tf2::Vector3> odom_pose;
    // bool tferr = true;
    // while(tferr){
    //   try {
    //     tferr = false;
    //     geometry_msgs::msg::TransformStamped transform_stamped;
    //     transform_stamped = costmap_ros_->getTfBuffer()->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
    //     tf2::Matrix3x3 rotation_matrix(
    //     tf2::Quaternion(
    //     transform_stamped.transform.rotation.x,
    //     transform_stamped.transform.rotation.y,
    //     transform_stamped.transform.rotation.z,
    //     transform_stamped.transform.rotation.w));
    //     for(int i=0;i<15;i++){
    //       odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[i] + c1);
    //     }       
    //     for(int i=0;i<15;i++){ 
    //       s[i][0] = static_cast<unsigned int>(odom_pose[i][0] / 0.05);
    //       s[i][1] = static_cast<unsigned int>(odom_pose[i][1] / 0.05);
    //       if(costmap_->getCost(s[i][0],s[i][1]) >= 253){
    //         back_ultra += 1;
    //       }
    //       else{
    //         back_ultra += 0;
    //       }
    //     }
    //   }
    //   catch (tf2::TransformException& e) {
    //     RCLCPP_WARN(get_logger(), "Failed to transform base_footprint to odom: %s", e.what());
    //     tferr = true;
    //     continue;
    //   }
    // }
    // back_messages.push(back_ultra);
    // if(back_messages.size()>5){
    //   back_messages.pop();
    // }
    // if(back_ultra == 0){
    //   back_messages1 = back_messages;
    //   for(int i=0;i<5;i++){
    //     if(back_messages1.empty()){
    //       return false;
    //     }
    //     if(back_messages1.front()>0){
    //       return true;
    //     }
    //     back_messages1.pop();
    //   }
    //   return false;
    // }
    // else{
    //   return true;
    // }
  }
  bool isobstacleultra()
  {
    std::vector<tf2::Vector3> footprint_pose;
    tf2::Vector3 c1(local_width_,local_height_,0);
    unsigned int s[2][2];
    footprint_pose.push_back(tf2::Vector3(0.6, -0.1, 0));
    footprint_pose.push_back(tf2::Vector3(0.6, 0.1, 0));
    std::vector<tf2::Vector3> odom_pose;
    bool tferr = true;
    while(tferr){
      try {
        tferr = false;
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = costmap_ros_->getTfBuffer()->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
        tf2::Matrix3x3 rotation_matrix(
        tf2::Quaternion(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w));
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[0] + c1);  
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[1] + c1);  
        for(int i=0;i<2;i++){ 
          s[i][0] = static_cast<unsigned int>(odom_pose[i][0] / 0.05);
          s[i][1] = static_cast<unsigned int>(odom_pose[i][1] / 0.05);
          if(costmap_->getCost(s[i][0],s[i][1]) >= 253 && fabs(cvt) < 0.2){
            ultra_count++;
            return true;
          }
        }
      }
      catch (tf2::TransformException& e) {
        RCLCPP_WARN(get_logger(), "Failed to transform base_footprint to odom: %s", e.what());
        tferr = true;
        continue;
      }
    }
    ultra_count = 0;
    return false;
  }
  bool isobstacleback()
  {
    std::vector<tf2::Vector3> footprint_pose;
    tf2::Vector3 c1(local_width_,local_height_,0);
    unsigned int s[2][2];
    footprint_pose.push_back(tf2::Vector3(-0.65, -0.1, 0));
    footprint_pose.push_back(tf2::Vector3(-0.65, 0.1, 0));
    std::vector<tf2::Vector3> odom_pose;
    bool tferr = true;
    while(tferr){
      try {
        tferr = false;
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = costmap_ros_->getTfBuffer()->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
        tf2::Matrix3x3 rotation_matrix(
        tf2::Quaternion(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w));
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[0] + c1);  
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[1] + c1);  
        for(int i=0;i<2;i++){ 
          s[i][0] = static_cast<unsigned int>(odom_pose[i][0] / 0.05);
          s[i][1] = static_cast<unsigned int>(odom_pose[i][1] / 0.05);
          if(costmap_->getCost(s[i][0],s[i][1]) >= 253 && fabs(cvt) < 0.2){
            return true;
          }
        }
      }
      catch (tf2::TransformException& e) {
        RCLCPP_WARN(get_logger(), "Failed to transform base_footprint to odom: %s", e.what());
        tferr = true;
        continue;
      }
    }
    return false;
  }
  

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    cvt=twist.theta;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;
  double local_width_;
  double local_height_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;
  nav2_costmap_2d::Costmap2D * costmap_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
  double ultra_back_time = 0;
  rclcpp::Time start_time_;
  int icp = 0;
  // int icf;
  int stop_1;
  double cvt;
  bool stop = false;
  // int stop_ = 0;
  int lcz = 0;
  bool above_threshold = false;
  int stop_2;
  std::queue<int> recent_messages;
  std::queue<int> recent_messages1;
  // std::queue<int> back_messages;
  // std::queue<int> back_messages1;
  std::queue<int> ultra_messages;
  std::queue<int> ultra_messages1;
  // int back_ultra;
  int drop_s = 0;
  double goal_x,goal_y;
  int ultra_count = 0;
  bool back_threshold = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dropsignal_subscribe_;
  void dropsignalsubscribecallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data){
      drop_s = 1;
    }
    else
      drop_s = 0;
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_subscribe_;
  void localizationsubscribecallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if(msg->data < 0.35){
      lcz = 0;
      above_threshold = true;
    }
    else if(msg->data >= 0.35 && msg->data < 0.75 && above_threshold){
      lcz = 0;
    }
    else if(msg->data >= 0.75 && above_threshold){
      lcz = 2;
      above_threshold = false;
    }
    else{
      lcz = 1;
    }
  }
  rclcpp::Subscription<capella_ros_msg::msg::DetectResult>::SharedPtr person_subscribe_;
  void personsubscribecallback(const capella_ros_msg::msg::DetectResult::SharedPtr msg)
  {
    icp=0;
    // icf=0;
    stop_1 = 0;
    //RCLCPP_INFO(rclcpp::get_logger("critics"),"theta: %f",cvt);
    for(size_t i=0;i<msg->result.size();i++){
      if(fabs(cvt) >= 0.2 && msg->result[i].x < 0.8 && fabs(msg->result[i].y) < 0.6 && msg->result[i].part){
        icp += 1;
        // icf += 0;
      }
      else if(fabs(cvt) < 0.2 && msg->result[i].x < 1.6 && fabs(msg->result[i].y) < 0.6 && msg->result[i].part){
        icp += 1;
        // icf += 0;
      }
      else if(msg->result[i].x < 1.5 && fabs(msg->result[i].y) < 0.4 && !msg->result[i].part){
        // icf += 1;
        icp += 0;
        // if(msg->result[i].x < 1.5){
        stop_1 += 1;
        // }
        // else
          // stop_1 += 0; 
      }
      else{
        icp += 0;
        // icf += 0;
        stop_1 += 0;
      }       
    }
    if(stop_1 == 0){
      recent_messages1 = recent_messages;
      for (int i = 0; i < 5; i++) {  
        if (recent_messages1.front() > 0) {  
          stop_2 += 1;
          break;  
        }  
        recent_messages1.pop();  
      }
      stop_2 = 0;
    }
    else{
      stop_2 += 1;
    }
    recent_messages.push(stop_1);
    if(recent_messages.size()>5){
      recent_messages.pop();
    }
    // RCLCPP_INFO(rclcpp::get_logger("back"),"recent stop1: %d",recent_messages.back());
  }
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
