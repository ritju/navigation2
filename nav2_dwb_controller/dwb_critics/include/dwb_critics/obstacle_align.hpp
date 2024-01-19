#ifndef DWB_CRITICS__OBSTACLE_ALIGN_HPP_
#define DWB_CRITICS__OBSTACLE_ALIGN_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

class ObstacleAlignCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit();
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj);
  double getobstaclemindist(double x, double y);
protected:
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace dwb_critics
#endif  
