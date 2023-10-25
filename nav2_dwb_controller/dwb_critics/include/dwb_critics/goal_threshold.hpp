#ifndef DWB_CRITICS__GOAL_THRESHOLD_HPP_
#define DWB_CRITICS__GOAL_THRESHOLD_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{
class GoalThresholdCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double dxy;
};

}  // namespace dwb_critics
#endif  
