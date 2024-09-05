#include "dwb_critics/base_goal_compute_vel.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include <Eigen/Dense>

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseGoalComputeVelCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
  void BaseGoalComputeVelCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".max_distance",
    rclcpp::ParameterValue(1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_distance", max_distance_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".min_x",
    rclcpp::ParameterValue(0.4));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".min_x", min_x_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".nav_x",
    rclcpp::ParameterValue(0.4));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".nav_x", nav_x_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".max_xt",
    rclcpp::ParameterValue(0.6));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_xt", max_xt_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".min_distance",
    rclcpp::ParameterValue(0.4));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".min_distance", min_distance_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".max_x",
    rclcpp::ParameterValue(0.8));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_x", max_x_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  command_publisher_ = node->create_publisher<geometry_msgs::msg::Pose>("distance", 10);
  slide_subscription_ = node->create_subscription<geometry_msgs::msg::Pose>("person_goal_pose", 10, std::bind(&BaseGoalComputeVelCritic::slidesubscribecallback, this, std::placeholders::_1));
  subscription_ = node->create_subscription<std_msgs::msg::Bool>("following_person", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&BaseGoalComputeVelCritic::followingpersonsubscribecallback, this, std::placeholders::_1));
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
    // RCLCPP_INFO(rclcpp::get_logger("tf"), "tf: %f, %f", t.transform.translation.x,t.transform.translation.y);
    robot_x = t.transform.translation.x;
    robot_y = t.transform.translation.y;
    geometry_msgs::msg::Quaternion q = t.transform.rotation;
    Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
    double yaw = atan2(2.0 * (eigen_q.w() * eigen_q.z() + eigen_q.x() * eigen_q.y()),  
                    1.0 - 2.0 * (eigen_q.y() * eigen_q.y() + eigen_q.z() * eigen_q.z())); 
    difference = pose_z - yaw;
    if (difference < -M_PI) {
      difference = 2 * M_PI + difference;
    } 
    if (difference > M_PI) {  
      difference = -2 * M_PI + difference;  
    } 
    geometry_msgs::msg::Pose message;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    if(next_time >= 1.88){
      // RCLCPP_INFO(rclcpp::get_logger("time"), "next time: %f", next_time);
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
    message.position.z = difference;
    command_publisher_->publish(message);
    // RCLCPP_INFO(rclcpp::get_logger("distance"), "distance in goal threshold: %f, dxy: %f", dx, dxy);
    return true;
}

double BaseGoalComputeVelCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj){
    double s = 80;
    // 跟随模式下，机器人速度根据行人速度与行人距离进行调节
    if(follow_person_){
      if(dxy < min_distance_ - 0.2){
        if(traj.velocity.x < 0.01){
          s = 0;
        }
      }
      else if(dxy < min_distance_){
        if(fabs(traj.velocity.x - min_x_ - 0.1) < 0.03){
          s = 0;
        }
      }
      // 机器人速度匹配行人速度，同时限制机器人最小速度为0.3（看需求，可以不限制）
      else if(dxy < max_distance_){
        if ((dx <= min_x_ && dxy > (max_distance_ + min_distance_)/2 && fabs(traj.velocity.x - min_x_ + 0.2) < 0.03) ||
            (dx <= min_x_ && dxy <= (max_distance_ + min_distance_)/2 && fabs(traj.velocity.x - min_x_) < 0.03) ||  
            (dx > min_x_ && dxy > max_distance_ - 0.2 && fabs(traj.velocity.x - dx - 0.1) < 0.03) ||
            (dx > min_x_ && dxy < min_distance_ + 0.2 && fabs(traj.velocity.x - dx + 0.1) < 0.03) ||
            (dx > min_x_ && dxy >= min_distance_ + 0.2 && dxy <= max_distance_ - 0.2 && fabs(traj.velocity.x - dx) < 0.03)){  
            s = 0;  
        }
      }
      else{
        if(traj.velocity.x > 0.1){
          s = 0;
        }
      }
      // 限制线速度与角速度同时过高，否则容易漂移 
      if(traj.velocity.x > max_xt_ && fabs(traj.velocity.theta) > max_xt_){
        s += 80;
      }
    }
    // 带人模式下
    else{
      s = (traj.velocity.x <= nav_x_) ? 0 : 80; 
    }

    return s;
}

}  // namespace dwb_critics

