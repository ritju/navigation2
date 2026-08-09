// Copyright (c) 2026 Capella
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__MISSION_PATH_SYNC_HPP_
#define NAV2_BEHAVIOR_TREE__MISSION_PATH_SYNC_HPP_

#include <cstdint>

#include "behaviortree_cpp_v3/blackboard.h"

namespace nav2_behavior_tree
{

/** Incremented on each new NavigateThroughPoses goal / preempt. */
inline constexpr char kNavMissionGenerationIdKey[] = "nav_mission_generation_id";
/** Set equal to nav_mission_generation_id when planner path is ready for that mission. */
inline constexpr char kPathMissionGenerationIdKey[] = "path_mission_generation_id";

inline uint64_t bumpNavMissionGenerationId(const BT::Blackboard::Ptr & blackboard)
{
  uint64_t mission_id = 0;
  try {
    blackboard->get(kNavMissionGenerationIdKey, mission_id);
    ++mission_id;
  } catch (...) {
    mission_id = 1;
  }
  blackboard->set(kNavMissionGenerationIdKey, mission_id);
  blackboard->set(kPathMissionGenerationIdKey, static_cast<uint64_t>(0));
  return mission_id;
}

inline void markPathSyncedToCurrentMission(const BT::Blackboard::Ptr & blackboard)
{
  uint64_t mission_id = 0;
  blackboard->get(kNavMissionGenerationIdKey, mission_id);
  blackboard->set(kPathMissionGenerationIdKey, mission_id);
}

inline bool isPathReadyForCurrentMission(const BT::Blackboard::Ptr & blackboard)
{
  uint64_t mission_id = 0;
  try {
    blackboard->get(kNavMissionGenerationIdKey, mission_id);
  } catch (...) {
    // Legacy trees that do not use mission/path generation sync.
    return true;
  }

  if (mission_id == 0) {
    return true;
  }

  uint64_t path_id = 0;
  try {
    blackboard->get(kPathMissionGenerationIdKey, path_id);
  } catch (...) {
    return false;
  }

  return path_id == mission_id;
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__MISSION_PATH_SYNC_HPP_
