/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <iomanip>
#include "angles/angles.h"

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <Eigen/Dense>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace dwb_core
{

DWBLocalPlanner::DWBLocalPlanner()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void DWBLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  dwb_plugin_name_ = name;
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".critics",
    rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".default_critic_namespaces",
    rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".prune_distance",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".forward_prune_distance",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".debug_trajectory_details",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".trajectory_generator_name",
    rclcpp::ParameterValue(std::string("dwb_plugins::StandardTrajectoryGenerator")));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".shorten_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    rclcpp::ParameterValue(true));

  std::string traj_generator_name;

  double transform_tolerance;
  node->get_parameter(dwb_plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(logger_, "Setting transform_tolerance to %f", transform_tolerance);

  node->get_parameter(dwb_plugin_name_ + ".prune_plan", prune_plan_);
  node->get_parameter(dwb_plugin_name_ + ".prune_distance", prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".forward_prune_distance", forward_prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".debug_trajectory_details", debug_trajectory_details_);
  node->get_parameter(dwb_plugin_name_ + ".trajectory_generator_name", traj_generator_name);
  node->get_parameter(
    dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    short_circuit_trajectory_evaluation_);
  node->get_parameter(dwb_plugin_name_ + ".shorten_transformed_plan", shorten_transformed_plan_);

  pub_ = std::make_unique<DWBPublisher>(node, dwb_plugin_name_);
  pub_->on_configure();

  traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);

  traj_generator_->initialize(node, dwb_plugin_name_);
  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
  footprint_w_front = footprint[0].y;
  footprint_w_back = footprint[0].y;
  footprint_h_front = footprint[0].x;
  footprint_h_back = footprint[0].x;
  for (unsigned int i = 1; i < footprint.size(); ++i) {
    if(footprint_h_front < footprint[i].x){
      footprint_h_front = footprint[i].x;
      footprint_w_front = footprint[i].y;
    }
    if(footprint_h_back > footprint[i].x){
      footprint_h_back = footprint[i].x;
      footprint_w_back = footprint[i].y;
    }
  }
  footprint_h_back = fabs(footprint_h_back);
  footprint_w_front = fabs(footprint_w_front);
  footprint_w_back = fabs(footprint_w_back);
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());

  try {
    loadCritics();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Couldn't load critics! Caught exception: %s", e.what());
    throw;
  }
}

void
DWBLocalPlanner::activate()
{
  pub_->on_activate();
}

void
DWBLocalPlanner::deactivate()
{
  pub_->on_deactivate();
}

void
DWBLocalPlanner::cleanup()
{
  pub_->on_cleanup();

  traj_generator_.reset();
}

std::string
DWBLocalPlanner::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void
DWBLocalPlanner::loadCritics()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(dwb_plugin_name_ + ".default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!node->get_parameter(dwb_plugin_name_ + ".critics", critic_names)) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name_);
  }

  node->get_parameter(dwb_plugin_name_ + ".critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + critic_plugin_name + ".class",
      rclcpp::ParameterValue(critic_plugin_name));
    node->get_parameter(dwb_plugin_name_ + "." + critic_plugin_name + ".class", plugin_class);

    plugin_class = resolveCriticClassName(plugin_class);

    TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    RCLCPP_INFO(
      logger_,
      "Using critic \"%s\" (%s)", critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(node, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize critic plugin!");
      throw;
    }
    RCLCPP_INFO(logger_, "Critic plugin initialized");
  }
}

void
DWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  auto path2d = nav_2d_utils::pathToPath2D(path);
  for (TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }

  traj_generator_->reset();

  pub_->publishGlobalPlan(path2d);
  
  global_plan_ = path2d;
}

bool DWBLocalPlanner::is_obstacle_ultra()
{
  std::string globalfameid = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          globalfameid, "base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
      }
  double robot_x = t.transform.translation.x;
  double robot_y = t.transform.translation.y;
  geometry_msgs::msg::Quaternion q = t.transform.rotation;
  Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
  double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                  1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z()));
  double cos_th = cos(yaw);
  double sin_th = sin(yaw);
  for (double x = footprint_h_front; x <= footprint_h_front + 0.15; x += 0.05) {
    for (double y = -footprint_w_front; y < footprint_w_front; y += 0.1) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        robot_pose_in_obstacle.x = robot_x;
        robot_pose_in_obstacle.y = robot_y;
        if(yaw > 0){
          robot_pose_in_obstacle.theta = yaw - M_PI;
        }
        else{
          robot_pose_in_obstacle.theta = M_PI + yaw;
        }
        return true;
      }
    }
  }
  for (double x = footprint_h_front + 0.15; x <= footprint_h_front + 0.3; x += 0.1) {
    for (double y = -footprint_w_front / 2; y < footprint_w_front / 2; y += 0.05) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        robot_pose_in_obstacle.x = robot_x;
        robot_pose_in_obstacle.y = robot_y;
        if(yaw > 0){
          robot_pose_in_obstacle.theta = yaw - M_PI;
        }
        else{
          robot_pose_in_obstacle.theta = M_PI + yaw;
        }
        return true;
      }
    }
  }
  for (double x = footprint_h_front + 0.3; x <= footprint_h_front + 0.8; x += 0.05) {
    for (double y = -0.05; y < 0.05; y += 0.05) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        robot_pose_in_obstacle.x = robot_x;
        robot_pose_in_obstacle.y = robot_y;
        if(yaw > 0){
          robot_pose_in_obstacle.theta = yaw - M_PI;
        }
        else{
          robot_pose_in_obstacle.theta = M_PI + yaw;
        }
        return true;
      }
    }
  }
  // for (double x = footprint_h_front + 0.4; x <= footprint_h_front + 0.8; x += 0.05) {
  //   unsigned int map_x,map_y;
  //   double g_x = robot_x + x * cos_th;
  //   double g_y = robot_y + x * sin_th;
  //   if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
  //     robot_pose_in_obstacle.x = robot_x;
  //     robot_pose_in_obstacle.y = robot_y;
  //     if(yaw > 0){
  //       robot_pose_in_obstacle.theta = yaw - M_PI;
  //     }
  //     else{
  //       robot_pose_in_obstacle.theta = M_PI + yaw;
  //     }
  //     return true;
  //   }
  // }
  robot_pose_in_obstacle.x = robot_x;
  robot_pose_in_obstacle.y = robot_y;
  if(yaw > 0){
    robot_pose_in_obstacle.theta = yaw - M_PI;
  }
  else{
    robot_pose_in_obstacle.theta = M_PI + yaw;
  }
  return false;
}

bool DWBLocalPlanner::is_obstacle_back()
{
  std::string globalfameid = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          globalfameid, "base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
      }
  double robot_x = t.transform.translation.x;
  double robot_y = t.transform.translation.y;
  geometry_msgs::msg::Quaternion q = t.transform.rotation;
  Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
  double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                  1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z()));
  double cos_th = cos(yaw);
  double sin_th = sin(yaw);
  for (double x = -footprint_h_back - 0.35; x <= -footprint_h_back; x += 0.05) {
    for (double y = -footprint_w_back; y < footprint_w_back; y += 0.1) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        return true;
      }
    }
  }
  // for (double x = -footprint_h_back - 0.35; x <= -footprint_h_back; x += 0.05) {
  //   unsigned int map_x,map_y;
  //   double g_x = robot_x + x * cos_th;
  //   double g_y = robot_y + x * sin_th;
  //   if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
  //     return true;
  //   }
  // }
  return false;
}

bool DWBLocalPlanner::is_rotation_to_included_angle(){
  std::string globalfameid = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          globalfameid, "base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
      }
  geometry_msgs::msg::Quaternion q = t.transform.rotation;
  Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
  double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                  1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z())); 
  double pose_z = robot_pose_in_obstacle.theta;
  double difference = pose_z - yaw;
  if (difference < -M_PI) {
    difference = 2 * M_PI + difference;
  } 
  if (difference > M_PI) {  
    difference = -2 * M_PI + difference;  
  } 
  if(fabs(difference) < 0.2){
    return false;
  }
  return true;
}

bool DWBLocalPlanner::rotation_reverse(){
  std::string globalfameid = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          globalfameid, "base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
      }
  geometry_msgs::msg::Quaternion q = t.transform.rotation;
  Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
  double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                  1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z())); 
  double pose_z = robot_pose_in_obstacle.theta;
  double difference = pose_z - yaw;
  if (difference < -M_PI) {
    difference = 2 * M_PI + difference;
  } 
  if (difference > M_PI) {  
    difference = -2 * M_PI + difference;  
  } 
  if(fabs(difference) < 0.2){
    return false;
  }
  rotation_sign = -1.0;
  geometry_msgs::msg::Twist cmd_vel_2d;
  cmd_vel_2d.linear.x = 0;
  cmd_vel_2d.linear.y = 0;
  cmd_vel_2d.angular.z = -0.9;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = t.transform.translation.x;
  pose.pose.position.y = t.transform.translation.y;
  pose.pose.orientation = t.transform.rotation;

  stop_move = false;
  if(isCollisionPre(cmd_vel_2d, pose)){
    cmd_vel_2d.angular.z = 0.9;
    if(isCollisionPre(cmd_vel_2d, pose)){
      stop_move = true;
      return false;
    }
    else{
      rotation_sign = 1.0;
      return true;
    }
  }
  return true;
}

bool DWBLocalPlanner::isCollisionPre(
  const geometry_msgs::msg::Twist & cmd_vel,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw = 0.0;
  double footprint_cost = 0.0;

  while (simulated_time < 2) {
    simulated_time += 0.1;
    yaw = initial_yaw + cmd_vel.angular.z * simulated_time;
    using namespace nav2_costmap_2d;  // NOLINT
    footprint_cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      yaw, costmap_ros_->getRobotFootprint());

    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      return true;
    }

    if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::TwistStamped
DWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  nav_2d_msgs::msg::Twist2D cur_vel = nav_2d_utils::twist3Dto2D(velocity);
  

  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr;
  if (pub_->shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }

  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(
      nav_2d_utils::poseStampedToPose2D(pose),
      nav_2d_utils::twist3Dto2D(velocity), results);
    pub_->publishEvaluation(results);
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    if(is_obstacle_ultra()  && cur_vel.x > 0.1 && fabs(cmd_vel.twist.linear.x) > 0.1 && fabs(cur_vel.theta) < 0.1 && global_plan_.poses.size() > 40){
      is_update_obstacle_front = true;
    }
    bool switch_state = false;
   
    while(is_update_obstacle_front){
      if(rotation_reverse()){
        // RCLCPP_INFO(rclcpp::get_logger("planner"), "444444444444");
        is_update_reverse = true;
      }
      while(is_update_reverse){
        is_update_obstacle_front = true;
        geometry_msgs::msg::Twist cmd_vel_2d;
        cmd_vel_2d.linear.x = 0;
        cmd_vel_2d.linear.y = 0;
        cmd_vel_2d.angular.z = 0.2 * rotation_sign;
        pub_->publishCmdvel(cmd_vel_2d);
        if(!is_rotation_to_included_angle()){
          // RCLCPP_INFO(rclcpp::get_logger("planner"), "5555555555");
          is_update_reverse = false;
        }
        continue;
      }
      if(stop_move){
        is_update_obstacle_front = false;
        geometry_msgs::msg::Twist cmd_vel_2d;
        cmd_vel_2d.linear.x = 0;
        cmd_vel_2d.linear.y = 0;
        cmd_vel_2d.angular.z = 0;
        pub_->publishCmdvel(cmd_vel_2d);
        // RCLCPP_INFO(rclcpp::get_logger("planner"), "666666666666");
        continue;
      }
      // is_update_reverse = true;
      rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
      start_time_= steady_clock_.now();
      while(!is_obstacle_back() && !update){
        // RCLCPP_INFO(rclcpp::get_logger("planner"), "77777777777");
        is_update_obstacle_front = true;
        if(steady_clock_.now().seconds() - start_time_.seconds() > 10.0){
          update = true;
        }
        geometry_msgs::msg::Twist cmd_vel_2d;
        cmd_vel_2d.linear.x = -0.2;
        cmd_vel_2d.linear.y = 0;
        cmd_vel_2d.angular.z = 0;
        pub_->publishCmdvel(cmd_vel_2d);
        switch_state = true;
        continue;
      }
      update = false;
      is_update_obstacle_front = false;
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z = 0;
    }
    if(switch_state){
      geometry_msgs::msg::Twist cmd_vel_2d;
      cmd_vel_2d.linear.x = 0;
      cmd_vel_2d.linear.y = 0;
      cmd_vel_2d.angular.z = 0;
      pub_->publishCmdvel(cmd_vel_2d);
      sleep(1);
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z = 0;
      switch_state = false;
    }
    // RCLCPP_INFO(rclcpp::get_logger("planner"), "8888888888");
    return cmd_vel;
  } catch (const nav2_core::PlannerException & e) {
    pub_->publishEvaluation(results);
    throw;
  } 
}

void
DWBLocalPlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan);
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(
    tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
    goal_pose, transform_tolerance_);
}

nav_2d_msgs::msg::Twist2DStamped
DWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = clock_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;
  nav_2d_msgs::msg::Pose2DStamped front_pose;
  // front_pose.pose.x = pose.pose.x + 0.8 * cos(pose.pose.theta);
  // front_pose.pose.y = pose.pose.y + 0.8 * sin(pose.pose.theta);
  // front_pose.pose.theta = pose.pose.theta;
  // RCLCPP_INFO(rclcpp::get_logger("dwb"), "pose: %f,%f", pose.pose.x,pose.pose.y);
  // RCLCPP_INFO(rclcpp::get_logger("dwb"), "front pose: %f,%f", front_pose.pose.x,front_pose.pose.y);

  prepareGlobalPlan(pose, transformed_plan, goal_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (TrajectoryCritic::Ptr & critic : critics_) {
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw;
  }
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::msg::Twist2D twist;
  dwb_msgs::msg::Trajectory2D traj;
  // dwb_msgs::msg::Trajectory2D front_traj;
  dwb_msgs::msg::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    twist = traj_generator_->nextTwist();
    // nav_2d_msgs::msg::Twist2D front_twist,front_velocity;
    // front_twist.theta = twist.theta;
    // front_velocity.theta = velocity.theta;
    // if(front_twist.theta != 0){
    //   front_twist.x = sqrt(0.09*front_twist.theta*front_twist.theta+twist.x*twist.x);
    // }
    // else{
    //   front_twist.x = twist.x;
    // }
    // if(front_velocity.theta != 0){
    //   front_velocity.x = sqrt(0.09*front_velocity.theta*front_velocity.theta+velocity.x*velocity.x);
    // }
    // else{
    //   front_velocity.x = velocity.x;
    // }
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);
    // front_traj.velocity = traj.velocity;
    // geometry_msgs::msg::Pose2D front_pose;
    // for(unsigned int i=0;i<traj.poses.size();i++){
    //     front_pose.x = traj.poses[i].x + 0.8 * cos(traj.poses[i].theta);
    //     front_pose.y = traj.poses[i].y + 0.8 * sin(traj.poses[i].theta);
    //     front_pose.theta = traj.poses[i].theta;
    //     front_traj.poses.push_back(front_pose);
    // }
    // front_traj.time_offsets = traj.time_offsets;

    try {
      dwb_msgs::msg::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & e) {
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::msg::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {
    if (debug_trajectory_details_) {
      RCLCPP_ERROR(rclcpp::get_logger("DWBLocalPlanner"), "%s", tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "DWBLocalPlanner"), "%.2f: %10s/%s", x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj,
  double best_score)
{
  dwb_msgs::msg::TrajectoryScore score;
  score.traj = traj;

  for (TrajectoryCritic::Ptr & critic : critics_) {
    dwb_msgs::msg::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}

nav_2d_msgs::msg::Path2D
DWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw dwb_core::
          PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace dwb_core

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  dwb_core::DWBLocalPlanner,
  nav2_core::Controller)
