#include "dwb_critics/goal_threshold.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalThresholdCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

bool GoalThresholdCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  dxy = (pose.x - goal.x) * (pose.x - goal.x)+ (pose.y - goal.y) * (pose.y - goal.y);
  return true;
}

double GoalThresholdCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double s;
  // RCLCPP_INFO(rclcpp::get_logger("distance"), "distance in goal threshold: %.2f", dxy);
  if(dxy < 0.36 && dxy >= 0.16)
  {
    if(traj.velocity.x >= 0.2 && traj.velocity.x <= 0.21){
        s = 0.0;
    }
    else
      s=100.0;
  }
  else if(dxy <= 0.16)
  {
    if(traj.velocity.x >= 0.2 && fabs(traj.velocity.theta) <= 0.02){
        s = 0.0;
    }
    else
      s=100.0;
  }
  else 
    s=0.0;
  return s;
}

}  // namespace dwb_critics

