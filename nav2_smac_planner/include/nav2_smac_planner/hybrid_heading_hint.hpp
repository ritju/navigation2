// Copyright 2026 Capella
//
// New header (not present in distro Nav2). planner_server includes this instead of
// smac_planner_hybrid.hpp so /opt/ros/humble/include does not shadow the class.

#ifndef NAV2_SMAC_PLANNER__HYBRID_HEADING_HINT_HPP_
#define NAV2_SMAC_PLANNER__HYBRID_HEADING_HINT_HPP_

namespace nav2_smac_planner
{

/** Per-call A* heading gate (rad). Negative disables XY-only early return. */
void setHybridPendingGoalHeadingTolerance(double heading_tolerance_rad);

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__HYBRID_HEADING_HINT_HPP_
