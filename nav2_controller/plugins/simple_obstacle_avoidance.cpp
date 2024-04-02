

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

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;



namespace nav2_controller
{
double isobstacleback_min_distance = -0.65, isobstacleback_max_distance = -0.54, isobstacleultra_min_distance = 0.5, isobstacleultra_max_distance = 0.61;


SimpleObstacleAvoidance::SimpleObstacleAvoidance()
: local_width_(1.5),
  local_height_(1.5)
{
  try
  {
    isobstacleback_min_distance = std::stod(getenv("ISOBSTACLEBACK_MIN_DISTANCE"));
    isobstacleback_max_distance = std::stod(getenv("ISOBSTACLEBACK_MAX_DISTANCE"));
    isobstacleultra_min_distance = std::stod(getenv("ISOBSTACLEULTRA_MIN_DISTANCE"));
    isobstacleultra_max_distance = std::stod(getenv("ISOBSTACLEULTRA_MAX_DISTANCE"));
  }
  catch(...)
  {
    auto now = rclcpp::Clock();
    RCLCPP_WARN(rclcpp::get_logger("simple_avoidance"),  "ENV in controller {ISOBSTACLEBACK_MIN_DISTANCE} or {ISOBSTACLEBACK_MAX_DISTANCE} not set! Use default values !");
  }
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
  // RCLCPP_INFO(rclcpp::get_logger("TEST"), "width: %f, height: %f", local_width_,local_height_);
  std::vector<tf2::Vector3> footprint_pose;
  tf2::Vector3 c1(local_width_,local_height_,0);
  unsigned int s[15][2];
  for (double x = isobstacleback_min_distance; x < isobstacleback_max_distance ; x += 0.05) {
    for (double y = -0.1; y < 0.11; y += 0.05) {
      footprint_pose.push_back(tf2::Vector3(x, y, 0));
    }
  }
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
      for(int i=0;i<15;i++){
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[i] + c1);
      }  
      for(int i=0;i<15;i++){ 
        s[i][0] = static_cast<unsigned int>(odom_pose[i][0] / 0.05);
        s[i][1] = static_cast<unsigned int>(odom_pose[i][1] / 0.05);
        if(costmap_->getCost(s[i][0],s[i][1]) >= 253){
          return true;
        }
      }
    }
    catch (tf2::TransformException& e) {
      tferr = true;
      continue;
    }
  }
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
  std::vector<tf2::Vector3> footprint_pose;
  tf2::Vector3 c1(local_width_,local_height_,0);
  unsigned int s[15][2];
  for (double x = isobstacleultra_min_distance; x < isobstacleultra_max_distance; x += 0.05) {
    for (double y = -0.1; y < 0.11; y += 0.05) {
      footprint_pose.push_back(tf2::Vector3(x, y, 0));
    }
  }
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
      for(int i=0;i<15;i++){
        odom_pose.push_back(rotation_matrix.inverse() * footprint_pose[i] + c1);
      }  
      for(int i=0;i<15;i++){ 
        s[i][0] = static_cast<unsigned int>(odom_pose[i][0] / 0.05);
        s[i][1] = static_cast<unsigned int>(odom_pose[i][1] / 0.05);
        if(costmap_->getCost(s[i][0],s[i][1]) >= 253){
          ultra_count++;
          return true;
        }
      }
    }
    catch (tf2::TransformException& e) {
      tferr = true;
      continue;
    }
  }
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
