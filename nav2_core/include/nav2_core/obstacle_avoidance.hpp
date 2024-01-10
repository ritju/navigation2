#ifndef NAV2_CORE__OBSTACLE_AVOIDANCE_HPP_
#define NAV2_CORE__OBSTACLE_AVOIDANCE_HPP_


#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "capella_ros_msg/msg/single_detector.hpp"
#include "capella_ros_msg/msg/detect_result.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
namespace nav2_core
{
class ObstacleAvoidance
{
public:
  typedef std::shared_ptr<nav2_core::ObstacleAvoidance> Ptr;
  virtual ~ObstacleAvoidance() {}
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  
  virtual bool isGoalOccupied(double goal_x, double goal_y) = 0;
  virtual bool isobstacleback() = 0;
  virtual bool isobstacleultraforward() = 0;
  virtual bool isobstacleultra() = 0;
  
};
}  

#endif