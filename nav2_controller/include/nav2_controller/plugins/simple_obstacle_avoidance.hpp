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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_OBSTACLE_AVOIDANCE_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_OBSTACLE_AVOIDANCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/obstacle_avoidance.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "capella_ros_msg/msg/single_detector.hpp"
#include "capella_ros_msg/msg/detect_result.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"

namespace nav2_controller
{
class PersonSubscriber
{
public:
  explicit PersonSubscriber(
    nav2_util::LifecycleNode::SharedPtr nh,
    std::string default_topic = "person_detected")
  {
    nav2_util::declare_parameter_if_not_declared(
      nh, "person_topic", rclcpp::ParameterValue(default_topic));

    std::string person_topic;
    nh->get_parameter_or("person_topic", person_topic, default_topic);
    person_sub_ = nh->create_subscription<capella_ros_msg::msg::DetectResult>(person_topic, 10, std::bind(&PersonSubscriber::personsubscribecallback, this, std::placeholders::_1));
  }
  inline void getcvt(const nav_2d_msgs::msg::Twist2D & twist){cvt = twist.theta;}
  inline int geticp() {
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    if(icp > 0 && check_icp_time <= 300){
      if (!icp_update_time)
      {
        icp_start_time_= steady_clock_.now();
      }
      icp_update_time = true;
      check_icp_time = steady_clock_.now().seconds() - icp_start_time_.seconds();
      return icp;
    }
    else if(icp > 0 && check_icp_time > 300 && check_icp_time <= 320){
      if (!icp_update_time)
      {
        icp_start_time_= steady_clock_.now();
      }
      icp_update_time = true;
      check_icp_time = steady_clock_.now().seconds() - icp_start_time_.seconds();
      return 0;
    }
    else{
      check_icp_time = 0;
      icp_update_time = false;
      return 0;
    }
  }
  inline int getstop() {return stop_2;}
protected:
  void personsubscribecallback(const capella_ros_msg::msg::DetectResult::SharedPtr msg){
    // RCLCPP_INFO(rclcpp::get_logger("TEST"), "cvt: %f", cvt);
    icp = 0;
    stop_1 = 0;
    for(size_t i=0;i<msg->result.size();i++){
      if(fabs(cvt) >= 0.2 && msg->result[i].x < 0.8 && fabs(msg->result[i].y) < 0.6 && msg->result[i].part){
        icp += 1;
      }
      else if(fabs(cvt) < 0.2 && msg->result[i].x < 1.6 && fabs(msg->result[i].y) < 0.6 && msg->result[i].part){
        icp += 1;
      }
      else if(fabs(cvt) >= 0.2 && msg->result[i].x < 0.8 && fabs(msg->result[i].y) < 0.4 && !msg->result[i].part){
        icp += 0;
        stop_1 += 1;
      }
      else if(fabs(cvt) < 0.2 && msg->result[i].x < 1.5 && fabs(msg->result[i].y) < 0.4 && !msg->result[i].part){
        icp += 0;
        stop_1 += 1;
      }
      else{
        icp += 0;
        stop_1 += 0;
      }       
    }
    if(icp == 0){
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
    }
    else{
      stop_2 = 0;
    }
  }

  rclcpp::Subscription<capella_ros_msg::msg::DetectResult>::SharedPtr person_sub_;
  int icp = 0;
  double check_icp_time = 0;
  rclcpp::Time icp_start_time_;
  bool icp_update_time = false;
  double cvt;
  int stop_1 = 0;
  int stop_2 = 0;
  std::queue<int> recent_messages;
  std::queue<int> recent_messages1;
};

class DropSubscriber
{
public:
  explicit DropSubscriber(
    nav2_util::LifecycleNode::SharedPtr nh,
    std::string default_topic = "drop_signal")
  {
    nav2_util::declare_parameter_if_not_declared(
      nh, "drop_topic", rclcpp::ParameterValue(default_topic));

    std::string drop_topic;
    nh->get_parameter_or("drop_topic", drop_topic, default_topic);
    drop_sub_ = nh->create_subscription<std_msgs::msg::Bool>(drop_topic, 10, std::bind(&DropSubscriber::dropsignalsubscribecallback, this, std::placeholders::_1));
  }
  inline int getdrop() {return drop_s;}
protected:
  void dropsignalsubscribecallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data){
      drop_s = 1;
    }
    else
      drop_s = 0;
  }
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_sub_;
  int drop_s = 0;
};

class SimpleObstacleAvoidance : public nav2_core::ObstacleAvoidance
{
public:
  SimpleObstacleAvoidance();
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  bool isGoalOccupied(double goal_x, double goal_y) override;
  bool isobstacleback() override;
  bool isobstacleultraforward() override;
  bool isobstacleultra() override;
  bool is_collision_front() override;
  bool is_collision_back() override;

protected:
  double local_width_;
  double local_height_;
  int ultra_count = 0;
  double ultra_back_time = 0;
  rclcpp::Time start_time_;
  bool update_time;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;
  double footprint_h_front,footprint_h_back, footprint_w_back, footprint_w_front;
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};


}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
