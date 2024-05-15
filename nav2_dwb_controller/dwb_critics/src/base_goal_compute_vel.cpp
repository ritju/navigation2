#include "dwb_critics/base_goal_compute_vel.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseGoalComputeVelCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
  void BaseGoalComputeVelCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  command_publisher_ = node->create_publisher<geometry_msgs::msg::Pose>("distance", 10);
  slide_subscription_ = node->create_subscription<geometry_msgs::msg::Pose>("person_goal_pose", 10, std::bind(&BaseGoalComputeVelCritic::slidesubscribecallback, this, std::placeholders::_1));
  subscription_ = node->create_subscription<std_msgs::msg::Bool>("following_person", 10, std::bind(&BaseGoalComputeVelCritic::followingpersonsubscribecallback, this, std::placeholders::_1));
}
bool BaseGoalComputeVelCritic::prepare(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, 
    const nav_2d_msgs::msg::Path2D &)
{
    geometry_msgs::msg::TransformStamped t;
    try {
          t = tf_buffer_->lookupTransform(
            "map", "base_link",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
        }
    robot_x = t.transform.translation.x;
    robot_y = t.transform.translation.y;
    geometry_msgs::msg::Pose message;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    if(next_time >= 1.88){
      dx = (sqrt(pow(pose_x - previous_goal_x, 2) + pow(pose_y - previous_goal_y, 2)))/2;
      // RCLCPP_INFO(rclcpp::get_logger("goal"), "current pose : x: %f, y: %f", pose.x, pose.y);
      update_time = false;
    }  
    if (!update_time){
      start_time_= steady_clock_.now();
      previous_goal_x = pose_x;
      previous_goal_y = pose_y;
      update_time = true;
    }
    next_time = steady_clock_.now().seconds() - start_time_.seconds();
    dxy = sqrt(pow(robot_x - pose_x, 2) + pow(robot_y - pose_y, 2));
    queue_dxy.push_back(dxy);
    if(queue_dxy.size() > 2){
        queue_dxy.pop_front();
    }
    message.position.x = dx;
    message.position.y = dxy;
    command_publisher_->publish(message);
    return true;
}

double BaseGoalComputeVelCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj){
    double s;
    if(follow_person_){  
      if(dxy <= 0.7 && traj.velocity.x < 0.01){
        s = 0;
      }
      else if(dxy <= 0.7 && traj.velocity.x >= 0.01){
        s = 80;
      }
      else if(queue_dxy[0] <= 0.8 && dxy > 0.8 && traj.velocity.x >= 0.3 && traj.velocity.x < 0.32){
        s = 0;
      }   
      else if(queue_dxy[0] <= 0.8 && dxy > 0.8 && (traj.velocity.x < 0.3 || traj.velocity.x >= 0.32)){
        s = 80;
      } 
      else if(dxy <= 5.0 && dx <= 0.3 && traj.velocity.x >= 0.3 && traj.velocity.x < 0.32){
        s = 0;
      }
      else if(dxy <= 5.0 && dx <= 0.3 && (traj.velocity.x < 0.3 || traj.velocity.x >= 0.32)){
        s = 80;
      }
      else if(dx <= 0.6 && fabs(dx - traj.velocity.x) < 0.05){
        s = 0;
      }
      else if(dx <= 0.6 && fabs(dx - traj.velocity.x) >= 0.05){
        s = 80;
      }
      else{
        s = 0;
      }
    }
    else{
      s = 0;
    }

    return s;
}

} 

