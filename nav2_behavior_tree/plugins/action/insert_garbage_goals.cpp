#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_behavior_tree/plugins/action/insert_garbage_pose_action.hpp"

namespace nav2_behavior_tree
{

// 哨兵点、保护点，以及 goals 的插入和一次删除

// ========================================================================
// C. goals 哨兵与保护点
// ========================================================================

// 判断 goals 里某点是否为本节点写入的 G/E，而不是input_goals里的点
bool InsertGarbagePose::isUnindexedSentinelPoseZ(
  const geometry_msgs::msg::PoseStamped & pose_stamped_goal)
{
  return std::lround(pose_stamped_goal.pose.position.z) ==
         std::lround(kGarbageSentinelPoseZ);
}

bool InsertGarbagePose::hasUnindexedSentinel(const Goals & goals)
{
  for (const auto & goal : goals) {
    if (isUnindexedSentinelPoseZ(goal)) {
      return true;
    }
  }
  return false;
}

bool InsertGarbagePose::findUnindexedSentinelIndex(  // 在整个goals里面找哨兵点
  const Goals & goals, double x, double y, std::size_t * index_out) const
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      continue;
    }
    if (squaredDistanceXY(
        goals[i].pose.position.x, goals[i].pose.position.y, x, y) < thresh2)
    {
      if (index_out != nullptr) {
        *index_out = i;
      }
      return true;
    }
  }
  return false;
}
bool InsertGarbagePose::isProtectedGarbageXy(double x, double y) const   //判断某个已经写进 goals 的 G/E 点
{
  const double thresh2 =
    kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (const auto & reached : reached_garbage_xy_) {
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}
// 记录已插入的垃圾点，防重复添加
void InsertGarbagePose::addProtectedGarbageXy(double x, double y)
{
  if (isProtectedGarbageXy(x, y)) {
    return;
  }
  reached_garbage_xy_.emplace_back(x, y);
}

void InsertGarbagePose::splitAtLastProtected(
  const Goals & goals, Goals * prefix, Goals * path) const
{
  int last_protected = -1;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (isProtectedGarbageXy(
        goals[i].pose.position.x, goals[i].pose.position.y) ||
      isUnindexedSentinelPoseZ(goals[i]))
    {
      last_protected = static_cast<int>(i);
    }
  }
  if (last_protected >= 0) {
    prefix->assign(
      goals.begin(),
      goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1);
    path->assign(
      goals.begin() + static_cast<std::ptrdiff_t>(last_protected) + 1,
      goals.end());
  } else {
    prefix->clear();
    *path = goals;
  }
}

void InsertGarbagePose::splitFrozenTail(
  const Goals & goals, const Goals * frozen_tail,
  Goals * prefix, Goals * path) const
{
  if (frozen_tail != nullptr && !frozen_tail->empty() &&
    goals.size() >= frozen_tail->size())
  {
    const std::size_t off = goals.size() - frozen_tail->size();
    bool suffix = true;
    for (std::size_t i = 0; i < frozen_tail->size(); ++i) {
      const auto & got = goals[off + i].pose.position;
      const auto & expect = (*frozen_tail)[i].pose.position;
      if (std::hypot(got.x - expect.x, got.y - expect.y) > 1e-3) {
        suffix = false;
        break;
      }
    }
    if (suffix) {
      prefix->assign(
        goals.begin(), goals.begin() + static_cast<std::ptrdiff_t>(off));
      *path = *frozen_tail;
      return;
    }
  }
  splitAtLastProtected(goals, prefix, path);
}

bool InsertGarbagePose::prepareClipBase(
  const Goals & prefix, const Goals & path,
  double gx, double gy, double saved_yaw,
  const capella_ros_msg::msg::GarbageDetect * saved_garbage,
  InsertInfo * info, InsertInfo * frozen)
{
  if (!prefix.empty() && path.size() >= 2) {
    InsertInfo path_info = gatherInsertInfo(path, info->robot_pose, gx, gy);
    if (!path_info.valid) {
      return false;
    }
    path_info.path_yaw = saved_yaw;
    path_info.goals = path;
    if (saved_garbage != nullptr) {
      path_info.garbage = *saved_garbage;
    }
    *frozen = path_info;
    info->goala = path_info.goala;
    info->goalc = path_info.goalc;
    info->goalc_idx = path_info.goalc_idx;
    info->goald_x = path_info.goald_x;
    info->goald_y = path_info.goald_y;
    return true;
  }
  if (path.size() >= 2 && info->valid) {
    frozen->goals = path;
    frozen->path_yaw = saved_yaw;
    if (saved_garbage != nullptr) {
      frozen->garbage = *saved_garbage;
    }
    return true;
  }
  return false;
}

std::size_t InsertGarbagePose::applyDeferredTailDelete(
  Goals * goals,
  const Goals & clip_tail,
  const std::set<std::size_t> & pending_delete)
{
  if (goals == nullptr || pending_delete.empty() || clip_tail.empty() ||
    goals->size() < clip_tail.size())
  {
    return 0;
  }
  const std::size_t tail_n = clip_tail.size();
  const std::size_t off = goals->size() - tail_n;
  for (std::size_t i = 0; i < tail_n; ++i) {
    const auto & got = (*goals)[off + i].pose.position;
    const auto & expect = clip_tail[i].pose.position;
    if (std::hypot(got.x - expect.x, got.y - expect.y) > 1e-3) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: deferred clip suffix mismatch, pending=%zu tail=%zu",
        pending_delete.size(), tail_n);
      return 0;
    }
  }
  Goals kept_tail;
  kept_tail.reserve(tail_n);
  std::size_t removed = 0;
  for (std::size_t i = 0; i < tail_n; ++i) {
    if (pending_delete.count(i) != 0) {
      ++removed;
      continue;
    }
    kept_tail.push_back((*goals)[off + i]);
  }
  Goals merged;
  merged.reserve(off + kept_tail.size());
  merged.insert(
    merged.end(), goals->begin(),
    goals->begin() + static_cast<std::ptrdiff_t>(off));
  merged.insert(merged.end(), kept_tail.begin(), kept_tail.end());
  *goals = std::move(merged);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: deferred clip removed %zu path goals from tail %zu",
    removed, tail_n);
  return removed;
}

InsertGarbagePose::Goals InsertGarbagePose::assembleAndStamp(
  const Goals & prefix, const Goals & inserted, const Goals & rest)
{
  Goals rebuilt;
  rebuilt.reserve(prefix.size() + inserted.size() + rest.size());
  rebuilt.insert(rebuilt.end(), prefix.begin(), prefix.end());
  rebuilt.insert(rebuilt.end(), inserted.begin(), inserted.end());
  rebuilt.insert(rebuilt.end(), rest.begin(), rest.end());
  const rclcpp::Time stamp_now = node_->now();
  for (auto & pose : rebuilt) {
    pose.header.stamp = stamp_now;
  }
  mission_stamp_record_ = stamp_now;
  has_mission_stamp_ = true;
  return rebuilt;
}

void InsertGarbagePose::recordSweepArrive(double x, double y, double yaw)
{
  last_sweep_arrive_xy_ = {x, y};
  has_last_sweep_arrive_ = true;
  last_sweep_path_yaw_ = yaw;
  has_last_sweep_path_yaw_ = true;
}

InsertGarbagePose::InsertLoopAction InsertGarbagePose::insertSweepUnit(
  InsertBatch * batch,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double rx, double ry)
{
  if (batch == nullptr || planned_units_.empty()) {
    return InsertLoopAction::BreakLoop;
  }
  const SweepUnit unit = planned_units_.front();
  planned_units_.erase(planned_units_.begin());
  const std::size_t n_piles = unit.piles.size();
  auto drop = [&]() {
    for (const auto & pile : unit.piles) {
      addProtectedGarbageXy(pile.pose.pose.position.x, pile.pose.pose.position.y);
    }
    eraseGarbageFront(n_piles);
  };
  if (unit.gxy.empty()) {
    drop();
    return InsertLoopAction::Continue;
  }

  const double gx = unit.gxy.front().first;
  const double gy = unit.gxy.front().second;
  InsertInfo info = gatherInsertInfo(batch->goals, robot_pose, gx, gy);
  if (!info.valid) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 单元 G%d (%.2f, %.2f) 无法插入 (%s), 跳过",
      unit.head_label, gx, gy, info.invalid_reason.c_str());
    drop();
    return InsertLoopAction::Continue;
  }
  info.dist_label = unit.head_label;
  info.path_yaw = unit.yaw;
  info.extend_from_x = unit.from_x;
  info.extend_from_y = unit.from_y;
  info.ray_chain_xy = unit.gxy;
  info.extend_inserted = unit.has_e;
  info.extend_x = unit.ex;
  info.extend_y = unit.ey;
  info.extend_used_m = unit.has_e ?
    std::hypot(unit.ex - unit.gxy.back().first, unit.ey - unit.gxy.back().second) : 0.0;
  info.wall_edge_has_d = unit.has_d;
  info.wall_edge_inserted = unit.has_d;
  info.wall_edge_d_x = unit.dx;
  info.wall_edge_d_y = unit.dy;
  info.wall_edge_has_p = unit.has_p;
  info.wall_edge_px = unit.px;
  info.wall_edge_py = unit.py;
  if (!unit.piles.empty()) {
    info.garbage = unit.piles.front();
  }
  info.garbage.pose.pose.position.x = gx;
  info.garbage.pose.pose.position.y = gy;

  Goals prefix;
  Goals path;
  splitFrozenTail(info.goals, &batch->clip_tail, &prefix, &path);
  const double saved_yaw = unit.yaw;
  Goals work = path;
  InsertInfo frozen = info;
  const bool have_clip_base = prepareClipBase(
    prefix, path, gx, gy, saved_yaw, &info.garbage, &info, &frozen);
  info.path_yaw = saved_yaw;
  info.dist_label = unit.head_label;
  info.ray_chain_xy = unit.gxy;
  info.extend_inserted = unit.has_e;
  info.extend_x = unit.ex;
  info.extend_y = unit.ey;
  info.wall_edge_has_d = unit.has_d;
  info.wall_edge_inserted = unit.has_d;
  info.wall_edge_d_x = unit.dx;
  info.wall_edge_d_y = unit.dy;
  info.wall_edge_has_p = unit.has_p;
  info.wall_edge_px = unit.px;
  info.wall_edge_py = unit.py;
  info.garbage.pose.pose.position.x = gx;
  info.garbage.pose.pose.position.y = gy;

  std::string frame = global_frame_;
  if (!info.goals.empty() && !info.goals.front().header.frame_id.empty()) {
    frame = info.goals.front().header.frame_id;
  }
  auto makeSentinel = [&](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = kGarbageSentinelPoseZ;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    return pose;
  };

  Goals inserted;
  if (unit.has_d) {
    const double d_yaw = std::atan2(gy - unit.dy, gx - unit.dx);
    inserted.push_back(makeSentinel(unit.dx, unit.dy, d_yaw));
  }
  for (std::size_t i = 0; i < unit.gxy.size(); ++i) {
    double seg_yaw = unit.yaw;
    if (i + 1 < unit.gxy.size()) {
      seg_yaw = std::atan2(
        unit.gxy[i + 1].second - unit.gxy[i].second,
        unit.gxy[i + 1].first - unit.gxy[i].first);
    }
    inserted.push_back(makeSentinel(unit.gxy[i].first, unit.gxy[i].second, seg_yaw));
  }
  if (unit.has_e) {
    inserted.push_back(makeSentinel(unit.ex, unit.ey, unit.yaw));
  }

  Goals out = work;
  if (have_clip_base && work.size() >= 2) {
    std::vector<std::pair<double, double>> refs = unit.gxy;
    if (unit.has_d) {
      refs.insert(refs.begin(), {unit.dx, unit.dy});
    }
    if (unit.has_e) {
      refs.emplace_back(unit.ex, unit.ey);
    }
    const Goals & judge =
      batch->clip_tail.size() >= 2 ? batch->clip_tail : work;
    out = clipWithRefs(
      judge, refs, frozen, &info, "sweep unit delete", &batch->pending_delete);
  }

  const std::size_t goals_before = batch->goals.size();
  batch->goals = assembleAndStamp(prefix, inserted, out);
  if (unit.has_e) {
    recordSweepArrive(unit.ex, unit.ey, unit.yaw);
  } else {
    recordSweepArrive(unit.gxy.back().first, unit.gxy.back().second, unit.yaw);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: 插入单元 G%d n=%zu D=%d E=%d goals %zu -> %zu",
    unit.head_label, unit.gxy.size(), unit.has_d ? 1 : 0, unit.has_e ? 1 : 0,
    goals_before, batch->goals.size());

  publishVisualization(info);
  publishRangeCircles(rx, ry);
  for (const auto & pile : unit.piles) {
    addProtectedGarbageXy(pile.pose.pose.position.x, pile.pose.pose.position.y);
    batch->inserted_xy << "(" << pile.pose.pose.position.x << ", "
                       << pile.pose.pose.position.y << ") ";
  }
  for (const auto & p : unit.gxy) {
    addProtectedGarbageXy(p.first, p.second);
  }
  if (unit.has_d) {
    addProtectedGarbageXy(unit.dx, unit.dy);
  }
  if (unit.has_e) {
    addProtectedGarbageXy(unit.ex, unit.ey);
  }
  for (std::size_t i = 0; i < unit.gxy.size(); ++i) {
    const bool tail = i + 1 == unit.gxy.size();
    rememberOpenGarbageRay(
      unit.gxy[i].first, unit.gxy[i].second, unit.yaw,
      tail && unit.has_e, unit.ex, unit.ey, unit.from_x, unit.from_y);
  }
  eraseGarbageFront(n_piles);
  batch->inserted_count += n_piles;
  return InsertLoopAction::Continue;
}

}  // namespace nav2_behavior_tree
