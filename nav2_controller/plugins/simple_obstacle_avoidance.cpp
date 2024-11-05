

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_controller/plugins/simple_obstacle_avoidance.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include <Eigen/Dense>

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

SimpleObstacleAvoidance::SimpleObstacleAvoidance()
: local_width_(1.5),
  local_height_(1.5)
{
}

void SimpleObstacleAvoidance::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".local_width", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".local_height", rclcpp::ParameterValue(1.5));

  node->get_parameter(plugin_name + ".local_width", local_width_);
  node->get_parameter(plugin_name + ".local_height", local_height_);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
  footprint_w = fabs(footprint[0].y);
  footprint_h_front = fabs(footprint[0].x);
  footprint_h_back = fabs(footprint[0].x);
  for (unsigned int i = 1; i < footprint.size(); ++i) {
    if(footprint_h_front < fabs(footprint[i].x)){
      footprint_h_front = fabs(footprint[i].x);
    }
    if(footprint_h_back > fabs(footprint[i].x)){
      footprint_h_back = fabs(footprint[i].x);
    }
  }
  // RCLCPP_INFO(rclcpp::get_logger("simple_obstacle"), "footprint: %f, %f", footprint_h_front, footprint_h_back);
  
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleObstacleAvoidance::dynamicParametersCallback, this, _1));
}


bool SimpleObstacleAvoidance::isGoalOccupied(double goal_x, double goal_y){
  
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
bool SimpleObstacleAvoidance::isobstacleback()
{
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          "odom", "base_link",
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
  for (double x = -footprint_h_back - 0.3; x <= -footprint_h_back; x += 0.05) {
    for (double y = -footprint_w; y < footprint_w; y += 0.1) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        return true;
      }
    }
  }
  
  // for (double x = -footprint_h_back - 0.3; x <= -footprint_h_back; x += 0.05) {
  //   unsigned int map_x,map_y;
  //   double g_x = robot_x + x * cos_th;
  //   double g_y = robot_y + x * sin_th;
  //   if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
  //     return true;
  //   }
  // }
  

  // for (double x = -footprint_h_back + 0.1; x < -footprint_h_back + 0.31; x += 0.05) {
  //   for (double y = -0.15; y < 0.15; y += 0.05) {
  //     unsigned int map_x,map_y;
  //     double g_x = robot_x + x * cos_th - y * sin_th;
  //     double g_y = robot_y + x * sin_th + y * cos_th;
  //     if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
  //       return true;
  //     }
  //   }
  // }
  return false;
}
bool SimpleObstacleAvoidance::isobstacleultraforward()
{
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  if(ultra_count >= 4 && ultra_back_time <= 2){
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
}
bool SimpleObstacleAvoidance::isobstacleultra()
{
  
  geometry_msgs::msg::TransformStamped t;
  try {
        t = costmap_ros_->getTfBuffer()->lookupTransform(
          "odom", "base_link",
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
    for (double y = -footprint_w; y < footprint_w; y += 0.1) {
      unsigned int map_x,map_y;
      double g_x = robot_x + x * cos_th - y * sin_th;
      double g_y = robot_y + x * sin_th + y * cos_th;
      if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
        ultra_count++;
        return true;
      }
    }
  }
  for (double x = footprint_h_front; x <= footprint_h_front + 0.3; x += 0.05) {
    unsigned int map_x,map_y;
    double g_x = robot_x + x * cos_th;
    double g_y = robot_y + x * sin_th;
    if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 254){
      ultra_count++;
      return true;
    }
  }
  

  // for (double x = footprint_h_front - 0.2; x < footprint_h_front + 0.01; x += 0.05) {
  //   for (double y = -0.15; y < 0.15; y += 0.05) {
  //     unsigned int map_x,map_y;
  //     double g_x = robot_x + x * cos_th - y * sin_th;
  //     double g_y = robot_y + x * sin_th + y * cos_th;
  //     if (costmap_ros_->getCostmap()->worldToMap(g_x, g_y, map_x, map_y) && costmap_ros_->getCostmap()->getCost(map_x, map_y) >= 253){
  //       ultra_count++;
  //       return true;
  //     }
  //   }
  // }
  ultra_count = 0;
  return false;
}

rcl_interfaces::msg::SetParametersResult
SimpleObstacleAvoidance::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".local_width") {
        local_width_ = parameter.as_double();
      } else if (name == plugin_name_ + ".local_height") {
        local_height_ = parameter.as_double();
      }
    } 
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleObstacleAvoidance, nav2_core::ObstacleAvoidance)
