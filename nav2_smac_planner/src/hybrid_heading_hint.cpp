// Copyright 2026 Capella
//
// Small non-plugin library so planner_server can share the heading hint without
// linking libnav2_smac_planner.so (which breaks pluginlib in composition).

#include "nav2_smac_planner/hybrid_heading_hint.hpp"

#include <atomic>

namespace nav2_smac_planner
{

namespace
{
std::atomic<double> g_pending_goal_heading_tolerance{-1.0};
}  // namespace

void setHybridPendingGoalHeadingTolerance(const double heading_tolerance_rad)
{
  g_pending_goal_heading_tolerance.store(heading_tolerance_rad);
}

double getHybridPendingGoalHeadingTolerance()
{
  return g_pending_goal_heading_tolerance.load();
}

}  // namespace nav2_smac_planner
