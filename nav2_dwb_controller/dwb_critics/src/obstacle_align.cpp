#include "dwb_critics/obstacle_align.hpp"
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "dwb_core/trajectory_critic.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleAlignCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
void ObstacleAlignCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
}

double ObstacleAlignCritic::getobstaclemindist(double x, double y)
{
  geometry_msgs::msg::PoseStamped nearest_obstacle;
  std::vector<geometry_msgs::msg::Point> obstacle_vec;
  geometry_msgs::msg::Point obstacle;
  //查找所有障碍物位置
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  //RCLCPP_INFO(rclcpp::get_logger("critic"),"size: %d %d",size_x,size_y);
  for(unsigned int j=0;j<size_y;j++) 
  {
    for(unsigned int k=0;k<size_x;k++)
    {
      if(costmap_->getCost(k,j) != nav2_costmap_2d::FREE_SPACE)
      {
        costmap_->mapToWorld(k,j,obstacle.x,obstacle.y);
        obstacle_vec.push_back(obstacle);
      }
    }
  }
  double distance=0,min_distance_obst = 1000.0;
  double diff_x,diff_y;
  for(size_t l=0;l<obstacle_vec.size();l++) 
  {
    diff_x = obstacle_vec[l].x - x;
    diff_y = obstacle_vec[l].y - y;
    distance = sqrt(diff_x*diff_x+diff_y*diff_y);
    if(min_distance_obst > distance)
    {
      min_distance_obst = distance;
    }
  }
  return min_distance_obst;
}

double ObstacleAlignCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double s;
  if(getobstaclemindist(traj.poses[0].x, traj.poses[0].y) <= 0.3)
  {
    if(getobstaclemindist(traj.poses[0].x, traj.poses[0].y) <= getobstaclemindist(traj.poses[1].x, traj.poses[1].y) 
    && getobstaclemindist(traj.poses[1].x, traj.poses[1].y) <= getobstaclemindist(traj.poses[2].x, traj.poses[2].y)
    && getobstaclemindist(traj.poses[2].x, traj.poses[2].y) <= getobstaclemindist(traj.poses[3].x, traj.poses[3].y))
    {
      s=0.0;
    }
    else
      s=7.5;
  }
  else 
    s=0.0;
  return s;
}
}
