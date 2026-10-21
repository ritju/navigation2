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

// 链条、暴力穷举和贴边，产出 planned_units_

namespace
{

double wrapAngleRad(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));  // 规范化到 (-π, π]
}

// 从 from 指向垃圾再伸出 extend_m，作为这一堆的延长点
void extendBeyondGarbage(
  double from_x, double from_y, double gx, double gy, double extend_m,
  double & ex, double & ey, double & yaw)
{
  const double dx = gx - from_x;
  const double dy = gy - from_y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {
    ex = gx;
    ey = gy;
    yaw = 0.0;
    return;
  }
  yaw = std::atan2(dy, dx);
  ex = gx + extend_m * std::cos(yaw);
  ey = gy + extend_m * std::sin(yaw);
}

// 一条中间顺序的得分：假设清扫段终点之间的路程 + 折到 [-pi, pi] 的转角
double forceListScore(
  const InsertGarbagePose::GarbageList & garbage_list,
  const std::vector<std::size_t> & garbage_order,
  double start_x, double start_y, double start_yaw,
  double w_dist, double w_turn,
  const std::function<void(
    double px, double py, double heading,
    double gx, double gy,
    double & ex, double & ey, double & yaw)> & leave)
{
  double px = start_x;
  double py = start_y;
  double heading = start_yaw;
  double total_dist = 0.0;
  double total_yaw = 0.0;
  for (const std::size_t idx : garbage_order) {
    if (idx >= garbage_list.size()) {
      continue;
    }
    const double gx = garbage_list[idx].pose.pose.position.x;
    const double gy = garbage_list[idx].pose.pose.position.y;
    double ex = gx;
    double ey = gy;
    double yaw = heading;
    leave(px, py, heading, gx, gy, ex, ey, yaw);
    total_dist += std::hypot(ex - px, ey - py);
    total_yaw += std::fabs(wrapAngleRad(yaw - heading));
    px = ex;
    py = ey;
    heading = yaw;
  }
  return w_dist * total_dist + w_turn * total_yaw;
}

}  // namespace

bool InsertGarbagePose::inForwardRaySector(
  double ox, double oy, double tx, double ty, double px, double py) const
{
  const double rx = tx - ox;
  const double ry = ty - oy;
  const double rlen = std::hypot(rx, ry);
  if (rlen < 1e-6) {
    return false;
  }
  const double vx = px - ox;
  const double vy = py - oy;
  const double vlen = std::hypot(vx, vy);
  if (vlen < 1e-6) {
    return false;
  }
  const double dot = vx * rx + vy * ry;
  if (dot <= 0.0) {
    return false;
  }
  const double cosang = std::clamp(dot / (vlen * rlen), -1.0, 1.0);
  const double ang = std::acos(cosang);
  return ang <= ray_offset_deg_ * M_PI / 180.0;
}

void InsertGarbagePose::logSortedInsertList(
  double pose_x, double pose_y, double pose_yaw)
{
  (void)pose_x;
  (void)pose_y;
  (void)pose_yaw;
  for (const SweepUnit & unit : planned_units_) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    if (unit.has_d) {
      oss << "D" << unit.head_label << "(" << unit.dx << ", " << unit.dy << ") ";
    }
    for (std::size_t i = 0; i < unit.gxy.size(); ++i) {
      if (i > 0) {
        oss << "->";
      }
      oss << "G" << (unit.head_label + static_cast<int>(i))
          << "(" << unit.gxy[i].first << ", " << unit.gxy[i].second << ")";
    }
    if (unit.has_e && !unit.gxy.empty()) {
      const int e_num = unit.head_label + static_cast<int>(unit.gxy.size()) - 1;
      oss << " E" << e_num << "(" << unit.ex << ", " << unit.ey << ")";
    }
    oss << " from=(" << unit.from_x << ", " << unit.from_y << ")";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 规划单元 %s", oss.str().c_str());
  }
}

void InsertGarbagePose::garbageSortTotal(double pose_x, double pose_y, double pose_yaw)
{
  planSweepUnits(pose_x, pose_y, pose_yaw);
}

void InsertGarbagePose::nudgeExtendAwayFromGarbage(
  double gx, double gy, double yaw,
  double near_x, double near_y,
  double * yaw_out, double * extend_m_out) const
{
  const double ex = std::cos(yaw);
  const double ey = std::sin(yaw);
  const double vx = near_x - gx;
  const double vy = near_y - gy;
  const double cross = ex * vy - ey * vx;
  const double sign = (cross >= 0.0) ? -1.0 : 1.0;
  if (yaw_out != nullptr) {
    *yaw_out = yaw + sign * ray_offset_deg_ * M_PI / 180.0;
  }
  if (extend_m_out != nullptr) {
    *extend_m_out = garbage_extend_m_ + extend_extra_m_;
  }
}

void InsertGarbagePose::eraseGarbageFront(std::size_t count)
{
  if (count > garbage_list_.size()) {
    count = garbage_list_.size();
  }
  garbage_list_.erase(
    garbage_list_.begin(),
    garbage_list_.begin() + static_cast<std::ptrdiff_t>(count));
}

InsertGarbagePose::GarbageList InsertGarbagePose::forcegarbagesort(
  GarbageList piles, double pose_x, double pose_y, double pose_yaw)
{
  double w_dist = sweep_dist_weight_;
  double w_turn = sweep_turn_weight_;
  getInput("garbage_extend_m", garbage_extend_m_);
  getInput("sweep_dist_weight", w_dist);
  getInput("sweep_turn_weight", w_turn);

  if (piles.size() > kMaxGarbageSize) {
    std::vector<std::size_t> by_dist(piles.size());
    for (std::size_t i = 0; i < by_dist.size(); ++i) {
      by_dist[i] = i;
    }
    std::stable_sort(
      by_dist.begin(), by_dist.end(),
      [&piles, pose_x, pose_y](std::size_t a, std::size_t b) {
        return squaredDistanceXY(
          piles[a].pose.pose.position.x, piles[a].pose.pose.position.y,
          pose_x, pose_y) <
        squaredDistanceXY(
          piles[b].pose.pose.position.x, piles[b].pose.pose.position.y,
          pose_x, pose_y);
      });
    std::vector<std::size_t> keep(by_dist.begin(), by_dist.begin() + kMaxGarbageSize);
    std::sort(keep.begin(), keep.end());
    GarbageList trimmed;
    trimmed.reserve(kMaxGarbageSize);
    for (const std::size_t i : keep) {
      trimmed.push_back(piles[i]);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: garbage_list 超上限 %zu -> %zu, 丢掉离车最远的几堆",
      piles.size(), trimmed.size());
    piles = std::move(trimmed);
  }

  const std::size_t n = piles.size();
  if (n <= 1) {
    return piles;
  }

  const double heading_c = std::cos(pose_yaw);
  const double heading_s = std::sin(pose_yaw);
  auto dist2ToPose = [&](std::size_t i) {
    return squaredDistanceXY(
      piles[i].pose.pose.position.x, piles[i].pose.pose.position.y,
      pose_x, pose_y);
  };
  auto forwardX = [&](std::size_t i) {
    const double dx = piles[i].pose.pose.position.x - pose_x;
    const double dy = piles[i].pose.pose.position.y - pose_y;
    return dx * heading_c + dy * heading_s;
  };

  bool have_front = false;
  std::size_t first_idx = 0;
  double best_front_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const double d2 = dist2ToPose(i);
    if (forwardX(i) > 0.0 && d2 < best_front_d2) {
      best_front_d2 = d2;
      first_idx = i;
      have_front = true;
    }
  }

  const Goals goals = receiveGoals();
  geometry_msgs::msg::PoseStamped pose_for_clip;
  pose_for_clip.header.frame_id = global_frame_;
  pose_for_clip.pose.position.x = pose_x;
  pose_for_clip.pose.position.y = pose_y;
  pose_for_clip.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(pose_yaw);
  auto leavePoint = [&](
    double px, double py, double heading, double gx, double gy,
    double & ex, double & ey, double & yaw)
  {
    const AssumedSweep seg = assumeSweepSegment(px, py, heading, gx, gy);
    double sx = 0.0;
    double sy = 0.0;
    seg.endpoints(sx, sy, ex, ey);
    if (std::hypot(ex - sx, ey - sy) > 1e-6) {
      yaw = std::atan2(ey - sy, ex - sx);
    } else {
      yaw = heading;
    }
  };
  auto clipReach = [&](double gx, double gy, std::size_t & reach) -> bool {
    const AssumedSweep seg = assumeSweepSegment(pose_x, pose_y, pose_yaw, gx, gy);
    double sx = 0.0;
    double sy = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    seg.endpoints(sx, sy, tx, ty);
    (void)sx;
    (void)sy;
    std::vector<std::pair<double, double>> refs;
    refs.emplace_back(seg.gx, seg.gy);
    if (std::hypot(tx - seg.gx, ty - seg.gy) > 1e-6) {
      refs.emplace_back(tx, ty);
    }
    InsertInfo clip_info;
    clip_info.valid = true;
    clip_info.goals = goals;
    clip_info.robot_pose = pose_for_clip;
    std::set<std::size_t> del;
    clipReferencesInOrder(goals, pose_for_clip, refs, del, clip_info, false);
    if (del.empty()) {
      return false;
    }
    reach = *del.rbegin();
    return true;
  };

  bool have_last = false;
  std::size_t last_idx = 0;
  std::size_t best_path_i = 0;
  double best_last_d2 = -1.0;
  bool have_reach = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (have_front && i == first_idx) {
      continue;
    }
    std::size_t reach = 0;
    const bool ok = clipReach(
      piles[i].pose.pose.position.x, piles[i].pose.pose.position.y, reach);
    const double d2 = dist2ToPose(i);
    if (!ok) {
      continue;
    }
    if (!have_reach || reach > best_path_i ||
      (reach == best_path_i && d2 > best_last_d2))
    {
      have_reach = true;
      have_last = true;
      last_idx = i;
      best_path_i = reach;
      best_last_d2 = d2;
    }
  }
  if (!have_reach) {
    for (std::size_t i = 0; i < n; ++i) {
      if (have_front && i == first_idx) {
        continue;
      }
      const double d2 = dist2ToPose(i);
      if (!have_last || d2 > best_last_d2) {
        have_last = true;
        last_idx = i;
        best_last_d2 = d2;
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: no clip reach, last pile falls back to farthest");
  }

  GarbageList middle;
  middle.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if ((have_front && i == first_idx) || (have_last && i == last_idx)) {
      continue;
    }
    middle.push_back(piles[i]);
  }

  double start_x = pose_x;
  double start_y = pose_y;
  double start_yaw = pose_yaw;
  if (have_front) {
    leavePoint(
      pose_x, pose_y, pose_yaw,
      piles[first_idx].pose.pose.position.x,
      piles[first_idx].pose.pose.position.y,
      start_x, start_y, start_yaw);
  }

  const capella_ros_msg::msg::GarbageDetect * locked_last =
    have_last ? &piles[last_idx] : nullptr;
  std::vector<std::size_t> middle_order(middle.size());
  for (std::size_t i = 0; i < middle.size(); ++i) {
    middle_order[i] = i;
  }
  if (middle.size() > 1) {
    auto scoreOf = [&](const std::vector<std::size_t> & perm) {
      GarbageList scored;
      std::vector<std::size_t> scored_idx;
      scored.reserve(perm.size() + (locked_last != nullptr ? 1u : 0u));
      scored_idx.reserve(scored.capacity());
      for (const std::size_t k : perm) {
        if (k < middle.size()) {
          scored_idx.push_back(scored.size());
          scored.push_back(middle[k]);
        }
      }
      if (locked_last != nullptr) {
        scored_idx.push_back(scored.size());
        scored.push_back(*locked_last);
      }
      return forceListScore(
        scored, scored_idx, start_x, start_y, start_yaw,
        w_dist, w_turn, leavePoint);
    };
    if (middle.size() <= kSweepBruteMaxN) {
      std::vector<std::size_t> perm = middle_order;
      double best_score = std::numeric_limits<double>::infinity();
      do {
        const double score = scoreOf(perm);
        if (score < best_score - 1e-9) {
          best_score = score;
          middle_order = perm;
        }
      } while (std::next_permutation(perm.begin(), perm.end()));
    } else {
      std::vector<std::size_t> pool = middle_order;
      std::vector<std::size_t> greedy;
      greedy.reserve(middle.size());
      double cur_x = start_x;
      double cur_y = start_y;
      double cur_yaw = start_yaw;
      while (!pool.empty()) {
        std::size_t best_pos = 0;
        double best_step = std::numeric_limits<double>::infinity();
        for (std::size_t p = 0; p < pool.size(); ++p) {
          GarbageList step;
          std::vector<std::size_t> step_idx{0};
          step.push_back(middle[pool[p]]);
          if (locked_last != nullptr) {
            step_idx.push_back(1);
            step.push_back(*locked_last);
          }
          const double score = forceListScore(
            step, step_idx, cur_x, cur_y, cur_yaw,
            w_dist, w_turn, leavePoint);
          if (score < best_step - 1e-9) {
            best_step = score;
            best_pos = p;
          }
        }
        const std::size_t chosen = pool[best_pos];
        greedy.push_back(chosen);
        pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(best_pos));
        leavePoint(
          cur_x, cur_y, cur_yaw,
          middle[chosen].pose.pose.position.x,
          middle[chosen].pose.pose.position.y,
          cur_x, cur_y, cur_yaw);
      }
      middle_order = std::move(greedy);
    }
  }

  GarbageList ordered;
  ordered.reserve(n);
  if (have_front) {
    ordered.push_back(piles[first_idx]);
  }
  for (const std::size_t k : middle_order) {
    if (k < middle.size()) {
      ordered.push_back(middle[k]);
    }
  }
  if (have_last) {
    ordered.push_back(piles[last_idx]);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: force garbage sort front=%d first=(%.2f, %.2f) "
    "last=(%.2f, %.2f) n=%zu",
    have_front ? 1 : 0,
    ordered.front().pose.pose.position.x,
    ordered.front().pose.pose.position.y,
    ordered.back().pose.pose.position.x,
    ordered.back().pose.pose.position.y,
    ordered.size());
  return ordered;
}

double InsertGarbagePose::squaredDistanceXY(
  double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}
// ========================================================================
// H. 普通插入
// ========================================================================

namespace
{

template<typename Pred>
bool tryYawFan(
  double yaw0, double max_deg, double step_deg, Pred pred,
  double * yaw_out, double * signed_step_deg)
{
  for (double step = step_deg; step <= max_deg + 1e-6; step += step_deg) {
    for (const double sign : {1.0, -1.0}) {
      const double yaw_try = yaw0 + sign * step * M_PI / 180.0;
      if (!pred(yaw_try)) {
        continue;
      }
      if (yaw_out != nullptr) {
        *yaw_out = yaw_try;
      }
      if (signed_step_deg != nullptr) {
        *signed_step_deg = sign * step;
      }
      return true;
    }
  }
  return false;
}

}  // namespace

namespace
{

bool sameGarbageXy(double ax, double ay, double bx, double by)
{
  return (ax - bx) * (ax - bx) + (ay - by) * (ay - by) < 1e-8;
}

}  // namespace

std::size_t InsertGarbagePose::pickSeedIndex(
  const GarbageList & piles, double pose_x, double pose_y, double pose_yaw) const
{
  const double c = std::cos(pose_yaw);
  const double s = std::sin(pose_yaw);
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  bool have_front = false;
  for (std::size_t i = 0; i < piles.size(); ++i) {
    const double dx = piles[i].pose.pose.position.x - pose_x;
    const double dy = piles[i].pose.pose.position.y - pose_y;
    if (dx * c + dy * s <= 0.0) {
      continue;
    }
    const double d2 = dx * dx + dy * dy;
    if (!have_front || d2 < best_d2) {
      have_front = true;
      best = i;
      best_d2 = d2;
    }
  }
  if (have_front) {
    return best;
  }
  best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < piles.size(); ++i) {
    const double d2 = squaredDistanceXY(
      piles[i].pose.pose.position.x, piles[i].pose.pose.position.y, pose_x, pose_y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

void InsertGarbagePose::rayFromAssumed(
  const AssumedSweep & sweep, double from_x, double from_y,
  double * ox, double * oy, double * tx, double * ty)
{
  if (sweep.has_e) {
    *ox = sweep.gx;
    *oy = sweep.gy;
    *tx = sweep.ex;
    *ty = sweep.ey;
    return;
  }
  if (sweep.has_d) {
    *ox = sweep.dx;
    *oy = sweep.dy;
    *tx = sweep.gx;
    *ty = sweep.gy;
    return;
  }
  *ox = sweep.gx;
  *oy = sweep.gy;
  const double dx = sweep.gx - from_x;
  const double dy = sweep.gy - from_y;
  if (std::hypot(dx, dy) < 1e-6) {
    *tx = sweep.gx + std::cos(sweep.yaw);
    *ty = sweep.gy + std::sin(sweep.yaw);
    return;
  }
  *tx = sweep.gx + dx;
  *ty = sweep.gy + dy;
}

bool InsertGarbagePose::makeChainTailExtend(
  double prev_x, double prev_y, double gx, double gy,
  const GarbageList & others,
  double * ex, double * ey, double * yaw) const
{
  if (ex == nullptr || ey == nullptr || yaw == nullptr) {
    return false;
  }
  extendBeyondGarbage(prev_x, prev_y, gx, gy, garbage_extend_m_, *ex, *ey, *yaw);
  auto clear = [&](double x, double y) {
    return std::hypot(x - gx, y - gy) <= 1e-3 ||
           isFootprintSweepClear(gx, gy, x, y, nullptr);
  };
  bool ok = clear(*ex, *ey);
  if (!ok) {
    double yaw_found = *yaw;
    double signed_step = 0.0;
    if (tryYawFan(
        *yaw, extend_max_yaw_deg_, extend_step_yaw_deg_,
        [&](double yaw_try) {
          return clear(
            gx + garbage_extend_m_ * std::cos(yaw_try),
            gy + garbage_extend_m_ * std::sin(yaw_try));
        },
        &yaw_found, &signed_step))
    {
      *yaw = yaw_found;
      *ex = gx + garbage_extend_m_ * std::cos(yaw_found);
      *ey = gy + garbage_extend_m_ * std::sin(yaw_found);
      ok = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 链尾 E 偏头 %+g deg -> (%.2f, %.2f)",
        signed_step, *ex, *ey);
    }
  }
  if (!ok) {
    return false;
  }
  const double limit2 = extend_near_radius_m_ * extend_near_radius_m_;
  bool near = false;
  double near_x = 0.0;
  double near_y = 0.0;
  double best_d2 = limit2;
  for (const auto & pile : others) {
    const double px = pile.pose.pose.position.x;
    const double py = pile.pose.pose.position.y;
    if (sameGarbageXy(px, py, gx, gy)) {
      continue;
    }
    const double d2 = squaredDistanceXY(px, py, *ex, *ey);
    if (d2 <= best_d2) {
      near = true;
      best_d2 = d2;
      near_x = px;
      near_y = py;
    }
  }
  if (!near) {
    return true;
  }
  double yaw_n = *yaw;
  double ext_n = garbage_extend_m_;
  nudgeExtendAwayFromGarbage(gx, gy, *yaw, near_x, near_y, &yaw_n, &ext_n);
  const double nx = gx + ext_n * std::cos(yaw_n);
  const double ny = gy + ext_n * std::sin(yaw_n);
  if (clear(nx, ny)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: E (%.2f, %.2f) 离垃圾太近, 偏开到 (%.2f, %.2f)",
      *ex, *ey, nx, ny);
    *yaw = yaw_n;
    *ex = nx;
    *ey = ny;
  }
  return true;
}

bool InsertGarbagePose::buildChainFromSeed(
  GarbageList * remaining, std::size_t seed_idx,
  double pose_x, double pose_y, double pose_yaw, SweepUnit * unit)
{
  if (remaining == nullptr || unit == nullptr || seed_idx >= remaining->size() ||
    remaining->size() < 2)
  {
    return false;
  }
  const auto seed_pile = (*remaining)[seed_idx];
  const double sgx = seed_pile.pose.pose.position.x;
  const double sgy = seed_pile.pose.pose.position.y;
  const AssumedSweep seed = assumeSweepSegment(pose_x, pose_y, pose_yaw, sgx, sgy);
  double ox = 0.0;
  double oy = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  rayFromAssumed(seed, pose_x, pose_y, &ox, &oy, &tx, &ty);
  const std::vector<std::size_t> hit = collectSectorChain(
    ox, oy, tx, ty, remaining->size(), seed_idx,
    [&](std::size_t i) {
      return std::make_pair(
        (*remaining)[i].pose.pose.position.x,
        (*remaining)[i].pose.pose.position.y);
    });
  if (hit.size() < 2) {
    return false;
  }
  std::vector<std::size_t> ordered;
  ordered.reserve(hit.size());
  ordered.push_back(seed_idx);
  for (const std::size_t idx : hit) {
    if (idx != seed_idx) {
      ordered.push_back(idx);
    }
  }

  GarbageList chain;
  std::vector<std::pair<double, double>> gxy;
  chain.reserve(ordered.size());
  for (std::size_t k = 0; k < ordered.size(); ++k) {
    chain.push_back((*remaining)[ordered[k]]);
    if (k == 0) {
      gxy.emplace_back(seed.gx, seed.gy);
    } else {
      gxy.emplace_back(
        chain.back().pose.pose.position.x, chain.back().pose.pose.position.y);
    }
  }

  for (std::size_t i = 1; i < gxy.size(); ++i) {
    std::string reason;
    if (isFootprintSweepClear(
        gxy[i - 1].first, gxy[i - 1].second, gxy[i].first, gxy[i].second, &reason))
    {
      continue;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 链截断 (%.2f, %.2f)->(%.2f, %.2f) 车体不过 (%s)",
      gxy[i - 1].first, gxy[i - 1].second, gxy[i].first, gxy[i].second,
      reason.c_str());
    chain.resize(i);
    gxy.resize(i);
    break;
  }
  if (chain.size() < 2) {
    return false;
  }

  bool has_e = false;
  double ex = 0.0;
  double ey = 0.0;
  double yaw = pose_yaw;
  while (chain.size() >= 2) {
    GarbageList others;
    others.reserve(remaining->size());
    for (const auto & pile : *remaining) {
      bool in_chain = false;
      for (const auto & kept : chain) {
        if (sameGarbageXy(
            pile.pose.pose.position.x, pile.pose.pose.position.y,
            kept.pose.pose.position.x, kept.pose.pose.position.y))
        {
          in_chain = true;
          break;
        }
      }
      if (!in_chain) {
        others.push_back(pile);
      }
    }
    const double prev_x = gxy[gxy.size() - 2].first;
    const double prev_y = gxy[gxy.size() - 2].second;
    const double gx = gxy.back().first;
    const double gy = gxy.back().second;
    if (makeChainTailExtend(prev_x, prev_y, gx, gy, others, &ex, &ey, &yaw)) {
      has_e = true;
      break;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: 链尾 E 不通过, 拆出 G (%.2f, %.2f)",
      chain.back().pose.pose.position.x, chain.back().pose.pose.position.y);
    chain.pop_back();
    gxy.pop_back();
  }
  if (chain.size() < 2) {
    return false;
  }

  std::vector<char> used(remaining->size(), 0);
  for (const auto & kept : chain) {
    for (std::size_t i = 0; i < remaining->size(); ++i) {
      if (used[i]) {
        continue;
      }
      if (sameGarbageXy(
          (*remaining)[i].pose.pose.position.x, (*remaining)[i].pose.pose.position.y,
          kept.pose.pose.position.x, kept.pose.pose.position.y))
      {
        used[i] = 1;
        break;
      }
    }
  }
  GarbageList rest;
  rest.reserve(remaining->size());
  for (std::size_t i = 0; i < remaining->size(); ++i) {
    if (!used[i]) {
      rest.push_back((*remaining)[i]);
    }
  }
  *remaining = std::move(rest);

  unit->piles = std::move(chain);
  unit->gxy = std::move(gxy);
  unit->has_d = seed.has_d;
  unit->dx = seed.dx;
  unit->dy = seed.dy;
  unit->has_e = has_e;
  unit->ex = ex;
  unit->ey = ey;
  unit->from_x = pose_x;
  unit->from_y = pose_y;
  unit->yaw = yaw;
  unit->has_p = seed.has_p;
  unit->px = seed.px;
  unit->py = seed.py;
  unit->head_label = next_g_num_;
  next_g_num_ += static_cast<int>(unit->piles.size());
  return true;
}

InsertGarbagePose::SweepUnit InsertGarbagePose::planSingleUnit(
  const capella_ros_msg::msg::GarbageDetect & pile,
  double pose_x, double pose_y, double pose_yaw,
  const GarbageList & others)
{
  const double gx = pile.pose.pose.position.x;
  const double gy = pile.pose.pose.position.y;
  AssumedSweep sweep = assumeSweepSegment(pose_x, pose_y, pose_yaw, gx, gy);
  if (sweep.has_e) {
    const double limit2 = extend_near_radius_m_ * extend_near_radius_m_;
    bool near = false;
    double near_x = 0.0;
    double near_y = 0.0;
    double best_d2 = limit2;
    for (const auto & other : others) {
      const double d2 = squaredDistanceXY(
        other.pose.pose.position.x, other.pose.pose.position.y, sweep.ex, sweep.ey);
      if (d2 <= best_d2) {
        near = true;
        best_d2 = d2;
        near_x = other.pose.pose.position.x;
        near_y = other.pose.pose.position.y;
      }
    }
    if (near) {
      double yaw_n = sweep.yaw;
      double ext_n = garbage_extend_m_;
      nudgeExtendAwayFromGarbage(
        sweep.gx, sweep.gy, sweep.yaw, near_x, near_y, &yaw_n, &ext_n);
      const double nx = sweep.gx + ext_n * std::cos(yaw_n);
      const double ny = sweep.gy + ext_n * std::sin(yaw_n);
      if (isFootprintSweepClear(sweep.gx, sweep.gy, nx, ny, nullptr)) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: E (%.2f, %.2f) 离垃圾太近, 偏开到 (%.2f, %.2f)",
          sweep.ex, sweep.ey, nx, ny);
        sweep.ex = nx;
        sweep.ey = ny;
        sweep.yaw = yaw_n;
      }
    }
  }
  SweepUnit unit;
  unit.piles.push_back(pile);
  unit.gxy.emplace_back(sweep.gx, sweep.gy);
  unit.has_d = sweep.has_d;
  unit.dx = sweep.dx;
  unit.dy = sweep.dy;
  unit.has_e = sweep.has_e;
  unit.ex = sweep.ex;
  unit.ey = sweep.ey;
  unit.has_p = sweep.has_p;
  unit.px = sweep.px;
  unit.py = sweep.py;
  unit.from_x = pose_x;
  unit.from_y = pose_y;
  unit.yaw = sweep.yaw;
  unit.head_label = next_g_num_++;
  return unit;
}

void InsertGarbagePose::advancePoseFromUnit(
  const SweepUnit & unit, double * pose_x, double * pose_y, double * pose_yaw)
{
  if (pose_x == nullptr || pose_y == nullptr || pose_yaw == nullptr || unit.gxy.empty()) {
    return;
  }
  if (unit.has_e) {
    *pose_x = unit.ex;
    *pose_y = unit.ey;
  } else {
    *pose_x = unit.gxy.back().first;
    *pose_y = unit.gxy.back().second;
  }
  *pose_yaw = unit.yaw;
}

void InsertGarbagePose::planSweepUnits(double pose_x, double pose_y, double pose_yaw)
{
  planned_units_.clear();
  GarbageList remaining = std::move(garbage_list_);
  auto takeChains = [&]() {
    while (remaining.size() >= 2) {
      const std::size_t seed = pickSeedIndex(remaining, pose_x, pose_y, pose_yaw);
      SweepUnit unit;
      if (!buildChainFromSeed(&remaining, seed, pose_x, pose_y, pose_yaw, &unit)) {
        break;
      }
      advancePoseFromUnit(unit, &pose_x, &pose_y, &pose_yaw);
      planned_units_.push_back(std::move(unit));
    }
  };
  takeChains();
  if (!remaining.empty()) {
    remaining = forcegarbagesort(std::move(remaining), pose_x, pose_y, pose_yaw);
    while (!remaining.empty()) {
      SweepUnit unit;
      if (remaining.size() >= 2 &&
        buildChainFromSeed(&remaining, 0, pose_x, pose_y, pose_yaw, &unit))
      {
        advancePoseFromUnit(unit, &pose_x, &pose_y, &pose_yaw);
        planned_units_.push_back(std::move(unit));
        continue;
      }
      GarbageList others;
      if (remaining.size() > 1) {
        others.assign(remaining.begin() + 1, remaining.end());
      }
      unit = planSingleUnit(remaining.front(), pose_x, pose_y, pose_yaw, others);
      remaining.erase(remaining.begin());
      advancePoseFromUnit(unit, &pose_x, &pose_y, &pose_yaw);
      planned_units_.push_back(std::move(unit));
    }
  }
  // 贴边 G 若和上一个单独的 G 重合，那个点会先被开到，再折去 D。
  // 把它并进贴边单元，清扫从 D 开始，再过 G 到 E。
  for (std::size_t i = 1; i < planned_units_.size(); ) {
    const SweepUnit & prev = planned_units_[i - 1];
    SweepUnit & wall = planned_units_[i];
    if (!wall.has_d || prev.has_d || prev.has_e || prev.gxy.size() != 1 ||
      wall.gxy.empty())
    {
      ++i;
      continue;
    }
    const double px = prev.gxy.front().first;
    const double py = prev.gxy.front().second;
    bool overlap = false;
    for (const auto & g : wall.gxy) {
      if (std::hypot(px - g.first, py - g.second) < kSentinelIdentityMatchM) {
        overlap = true;
        break;
      }
    }
    if (!overlap) {
      ++i;
      continue;
    }
    const int g_num = wall.head_label;
    const int e_num = g_num + static_cast<int>(wall.gxy.size()) - 1;
    if (wall.has_e) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: G%d (%.2f, %.2f) 落在贴边 G%d 上, 并入该贴边, 清扫顺序 D%d->G%d->E%d",
        prev.head_label, px, py, g_num, g_num, g_num, e_num);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: G%d (%.2f, %.2f) 落在贴边 G%d 上, 并入该贴边, 清扫顺序 D%d->G%d",
        prev.head_label, px, py, g_num, g_num, g_num);
    }
    wall.piles.insert(wall.piles.begin(), prev.piles.begin(), prev.piles.end());
    wall.from_x = prev.from_x;
    wall.from_y = prev.from_y;
    planned_units_.erase(planned_units_.begin() + static_cast<std::ptrdiff_t>(i - 1));
  }
  garbage_list_.clear();
  for (const SweepUnit & unit : planned_units_) {
    garbage_list_.insert(garbage_list_.end(), unit.piles.begin(), unit.piles.end());
  }
}

InsertGarbagePose::AssumedSweep InsertGarbagePose::assumeSweepSegment(
  double from_x, double from_y, double from_yaw, double gx, double gy)
{
  AssumedSweep out;
  out.gx = gx;
  out.gy = gy;
  out.yaw = from_yaw;

  double ex = gx;
  double ey = gy;
  double yaw = from_yaw;
  extendBeyondGarbage(from_x, from_y, gx, gy, garbage_extend_m_, ex, ey, yaw);
  out.yaw = yaw;

  const bool approach_ok =
    std::hypot(gx - from_x, gy - from_y) <= 1e-3 ||
    isFootprintSweepClear(from_x, from_y, gx, gy, nullptr);
  auto extendClear = [&](double e_x, double e_y) {
    return std::hypot(e_x - gx, e_y - gy) <= 1e-3 ||
      isFootprintSweepClear(gx, gy, e_x, e_y, nullptr);
  };

  if (approach_ok && std::hypot(ex - gx, ey - gy) > 1e-6 && extendClear(ex, ey)) {
    out.has_e = true;
    out.ex = ex;
    out.ey = ey;
    return out;
  }

  if (approach_ok) {
    double yaw_found = yaw;
    double signed_step = 0.0;
    if (tryYawFan(
        yaw, extend_max_yaw_deg_, extend_step_yaw_deg_,
        [&](double yaw_try) {
          const double ex_try = gx + garbage_extend_m_ * std::cos(yaw_try);
          const double ey_try = gy + garbage_extend_m_ * std::sin(yaw_try);
          return extendClear(ex_try, ey_try);
        },
        &yaw_found, &signed_step))
    {
      (void)signed_step;
      out.has_e = true;
      out.ex = gx + garbage_extend_m_ * std::cos(yaw_found);
      out.ey = gy + garbage_extend_m_ * std::sin(yaw_found);
      out.yaw = yaw_found;
      return out;
    }
  }

  if (!enable_wall_edge_insert_) {
    return out;
  }

  InsertInfo info;
  info.valid = true;
  info.garbage.pose.pose.position.x = gx;
  info.garbage.pose.pose.position.y = gy;
  info.extend_from_x = from_x;
  info.extend_from_y = from_y;
  info.path_yaw = yaw;
  const WallEdgeExtendChain chain = buildWallEdgeExtendChain(info);
  if (chain.has_d) {
    out.has_d = true;
    out.dx = chain.dx;
    out.dy = chain.dy;
  }
  if (chain.has_e) {
    out.has_e = true;
    out.ex = chain.ex;
    out.ey = chain.ey;
    out.yaw = chain.path_yaw;
  }
  if (chain.has_d || chain.has_e) {
    out.gx = chain.gx;
    out.gy = chain.gy;
    out.has_p = true;
    out.px = chain.px;
    out.py = chain.py;
  }
  return out;
}

std::vector<std::size_t> InsertGarbagePose::collectSectorChain(
  double ox, double oy, double tx, double ty,
  std::size_t count, std::size_t seed,
  const std::function<std::pair<double, double>(std::size_t)> & at) const
{
  std::vector<std::size_t> hit;
  hit.push_back(seed);
  for (std::size_t i = 0; i < count; ++i) {
    if (i == seed) {
      continue;
    }
    const auto p = at(i);
    if (inForwardRaySector(ox, oy, tx, ty, p.first, p.second)) {
      hit.push_back(i);
    }
  }
  if (hit.size() < 2) {
    return {};
  }
  std::stable_sort(
    hit.begin(), hit.end(),
    [&](std::size_t a, std::size_t b) {
      const auto pa = at(a);
      const auto pb = at(b);
      return squaredDistanceXY(pa.first, pa.second, ox, oy) <
             squaredDistanceXY(pb.first, pb.second, ox, oy);
    });
  return hit;
}

// ========================================================================
// I. 贴边插入
// ========================================================================

InsertGarbagePose::WallEdgeExtendChain InsertGarbagePose::buildWallEdgeExtendChain(
  const InsertInfo & info)
{
  WallEdgeExtendChain chain;
  const double yaw_max_deg = extend_max_yaw_deg_;
  const double yaw_step_deg = extend_step_yaw_deg_;

  const double gx0 = info.garbage.pose.pose.position.x;
  const double gy0 = info.garbage.pose.pose.position.y;
  const double from_x = info.extend_from_x;
  const double from_y = info.extend_from_y;

  // 是否走贴边仍由直线 E 过不过决定。P 只从垃圾本身找最近致命格，
  // 半径覆盖延长距离再多 1.5 m，避免用延长点当圆心把墙找偏。
  double px = 0.0;
  double py = 0.0;
  const double wall_search_r = garbage_extend_m_ + 1.5;
  if (!findNearestObstaclePixel(gx0, gy0, wall_search_r, &px, &py)) {
    chain.invalid_reason = "no obstacle P";
    return chain;
  }

  double nx = gx0 - px;
  double ny = gy0 - py;
  const double nlen = std::hypot(nx, ny);
  if (nlen < 1e-6) {
    chain.invalid_reason = "P coincides G";
    return chain;
  }
  nx /= nlen;
  ny /= nlen;
  const double tx = -ny;
  const double ty = nx;

  const double extend_m = garbage_extend_m_;
  const double ex1 = gx0 + extend_m * tx;
  const double ey1 = gy0 + extend_m * ty;
  const double ex2 = gx0 - extend_m * tx;
  const double ey2 = gy0 - extend_m * ty;
  const double d1 = std::hypot(ex1 - from_x, ey1 - from_y);
  const double d2 = std::hypot(ex2 - from_x, ey2 - from_y);
  bool e_plus = d1 > d2;
  const double fwd_x = std::cos(info.path_yaw);
  const double fwd_y = std::sin(info.path_yaw);
  if (std::fabs(d1 - d2) < 0.3) {
    e_plus = (tx * fwd_x + ty * fwd_y) >= 0.0;
  }
  double e_tx = e_plus ? tx : -tx;
  double e_ty = e_plus ? ty : -ty;
  double e_yaw = std::atan2(e_ty, e_tx);
  // 贴边 E/D 也只相对进近 from，避免后堆按发现时车位翻反
  e_yaw = preferExtendYawAwayFromRobot(gx0, gy0, e_yaw, from_x, from_y);
  e_tx = std::cos(e_yaw);
  e_ty = std::sin(e_yaw);

  const double off = wall_edge_normal_offset_m_;
  auto footClear = [&](double x, double y, double yaw, std::string * reason) {
    return isFootprintClearAtPose(
      x + off * nx, y + off * ny, yaw, reason);
  };
  // 2.13.1：entry 到 extend 做 2.11.1 footprint 长条
  auto deLineClear = [&](double x0, double y0, double x1, double y1) {
    return isFootprintSweepClear(
      x0 + off * nx, y0 + off * ny,
      x1 + off * nx, y1 + off * ny,
      nullptr);
  };

  // E：先落点，footprint 不过则只绕当前 e_yaw 扫角，不重选切向左右
  double ex = gx0 + extend_m * e_tx;
  double ey = gy0 + extend_m * e_ty;
  bool e_ok = false;
  {
    std::string e_reason;
    if (footClear(ex, ey, e_yaw, &e_reason)) {
      e_ok = true;
    } else {
      double yaw_found = e_yaw;
      double signed_step = 0.0;
      if (tryYawFan(
          e_yaw, yaw_max_deg, yaw_step_deg,
          [&](double yaw_try) {
            const double ex_try = gx0 + extend_m * std::cos(yaw_try);
            const double ey_try = gy0 + extend_m * std::sin(yaw_try);
            std::string sweep_reason;
            return footClear(ex_try, ey_try, yaw_try, &sweep_reason);
          },
          &yaw_found, &signed_step))
      {
        e_yaw = yaw_found;
        e_tx = std::cos(e_yaw);
        e_ty = std::sin(e_yaw);
        ex = gx0 + extend_m * e_tx;
        ey = gy0 + extend_m * e_ty;
        e_ok = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge E sweep %+g deg -> (%.2f, %.2f)",
          signed_step, ex, ey);
      }
      if (!e_ok) {
        chain.invalid_reason = "E footprint fail after sweep: " + e_reason;
      }
    }
  }

  // D：生成时与 E 对侧绑定；安全调整时 E 不动，只独立扫角/加长 D
  double d_yaw = std::atan2(-e_ty, -e_tx);
  const double step = wall_edge_d_extend_m_;
  const double gd = wall_edge_min_robot_dist_m_;
  auto placeD = [&](double yaw, double len, double * ox, double * oy) {
    *ox = gx0 + len * std::cos(yaw);
    *oy = gy0 + len * std::sin(yaw);
  };
  auto pushLenForGd = [&](double yaw) {
    double len = step;
    double ox = 0.0;
    double oy = 0.0;
    placeD(yaw, len, &ox, &oy);
    for (int i = 0; i < 20 && std::hypot(ox - from_x, oy - from_y) < gd; ++i) {
      len += step;
      placeD(yaw, len, &ox, &oy);
    }
    return len;
  };

  double d_len = pushLenForGd(d_yaw);
  double dx = 0.0;
  double dy = 0.0;
  placeD(d_yaw, d_len, &dx, &dy);
  bool d_ok = false;
  auto lineToE = [&](double x0, double y0, double x1, double y1) {
    return !e_ok || deLineClear(x0, y0, x1, y1);
  };
  {
    std::string d_reason;
    const double yaw_travel = std::atan2(gy0 - dy, gx0 - dx);
    if (footClear(dx, dy, yaw_travel, &d_reason) && lineToE(dx, dy, ex, ey)) {
      d_ok = true;
    } else {
      double yaw_found = d_yaw;
      double signed_step = 0.0;
      if (tryYawFan(
          d_yaw, yaw_max_deg, yaw_step_deg,
          [&](double yaw_try) {
            const double len_try = pushLenForGd(yaw_try);
            double dx_try = 0.0;
            double dy_try = 0.0;
            placeD(yaw_try, len_try, &dx_try, &dy_try);
            const double yaw_trav = std::atan2(gy0 - dy_try, gx0 - dx_try);
            std::string sweep_reason;
            return footClear(dx_try, dy_try, yaw_trav, &sweep_reason) &&
                   lineToE(dx_try, dy_try, ex, ey);
          },
          &yaw_found, &signed_step))
      {
        d_yaw = yaw_found;
        d_len = pushLenForGd(d_yaw);
        placeD(d_yaw, d_len, &dx, &dy);
        d_ok = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge D sweep %+g deg -> (%.2f, %.2f)",
          signed_step, dx, dy);
      }
      // 扫角仍不通：沿当前 d_yaw 再多推几步试安全点
      if (!d_ok) {
        for (int i = 0; i < 10 && !d_ok; ++i) {
          d_len += step;
          placeD(d_yaw, d_len, &dx, &dy);
          const double yaw_trav = std::atan2(gy0 - dy, gx0 - dx);
          std::string push_reason;
          if (footClear(dx, dy, yaw_trav, &push_reason) &&
            lineToE(dx, dy, ex, ey))
          {
            d_ok = true;
            RCLCPP_INFO(
              node_->get_logger(),
              "InsertGarbagePose: wall-edge D push len=%.2f -> (%.2f, %.2f)",
              d_len, dx, dy);
          }
        }
      }
      // 不再联扫挪 E：贴墙平行时挪 E 易把一端顶进墙；E 定稿后只独立挪 D
      if (!d_ok) {
        chain.invalid_reason = "D footprint/D-E line fail after D-only sweep: " + d_reason;
      }
    }
  }

  // 偏移后的 D-G、G-E 不能穿过致命障碍，避免贴边线段穿墙
  auto segmentClearOfObstacle = [&](double x0, double y0, double x1, double y1) {
    return std::hypot(x1 - x0, y1 - y0) <= 1e-3 ||
           isRobotGarbageSegmentClearOfLethal(x0, y0, x1, y1, nullptr);
  };

  if (!e_ok || !d_ok) {
    if (e_ok) {
      const double gx = gx0 + off * nx;
      const double gy = gy0 + off * ny;
      const double ex_o = ex + off * nx;
      const double ey_o = ey + off * ny;
      if (!segmentClearOfObstacle(gx, gy, ex_o, ey_o)) {
        chain.invalid_reason = "G-E line crosses obstacle";
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge G-E 连线穿过障碍, 不生成 "
          "G=(%.2f, %.2f) E=(%.2f, %.2f) P=(%.2f, %.2f)",
          gx, gy, ex_o, ey_o, px, py);
        return chain;
      }
      chain.has_e = true;
      chain.ex = ex_o;
      chain.ey = ey_o;
      chain.gx = gx;
      chain.gy = gy;
      chain.path_yaw = e_yaw;
      chain.extend_used_m = extend_m;
      chain.px = px;
      chain.py = py;
      chain.normal_offset_m = off;
      if (chain.invalid_reason.empty()) {
        chain.invalid_reason = "D missing";
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: wall-edge only E G=(%.2f, %.2f) E=(%.2f, %.2f) P=(%.2f, %.2f)",
        chain.gx, chain.gy, chain.ex, chain.ey, px, py);
    } else if (d_ok) {
      const double gx = gx0 + off * nx;
      const double gy = gy0 + off * ny;
      const double dx_o = dx + off * nx;
      const double dy_o = dy + off * ny;
      if (!segmentClearOfObstacle(dx_o, dy_o, gx, gy)) {
        chain.invalid_reason = "D-G line crosses obstacle";
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: wall-edge D-G 连线穿过障碍, 不生成 "
          "D=(%.2f, %.2f) G=(%.2f, %.2f) P=(%.2f, %.2f)",
          dx_o, dy_o, gx, gy, px, py);
        return chain;
      }
      chain.has_d = true;
      chain.dx = dx_o;
      chain.dy = dy_o;
      chain.gx = gx;
      chain.gy = gy;
      chain.path_yaw = std::atan2(chain.gy - chain.dy, chain.gx - chain.dx);
      chain.extend_used_m = d_len;
      chain.px = px;
      chain.py = py;
      chain.normal_offset_m = off;
      if (chain.invalid_reason.empty()) {
        chain.invalid_reason = "E missing";
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: wall-edge only D D=(%.2f, %.2f) G=(%.2f, %.2f) P=(%.2f, %.2f)",
        chain.dx, chain.dy, chain.gx, chain.gy, px, py);
    }
    return chain;
  }

  // 先在切线上采样 D…G…E，再整链沿 n 往外挪
  const double chain_len = std::hypot(ex - dx, ey - dy);
  const double spacing = wall_edge_sample_m_;
  std::vector<std::pair<double, double>> chain_xy;
  if (chain_len < 1e-6) {
    chain_xy.push_back({gx0, gy0});
  } else {
    const double ux = (ex - dx) / chain_len;
    const double uy = (ey - dy) / chain_len;
    const int n_seg = std::max(
      1, static_cast<int>(std::ceil(chain_len / spacing)));
    chain_xy.reserve(static_cast<std::size_t>(n_seg) + 3u);
    for (int i = 0; i <= n_seg; ++i) {
      const double s = chain_len * static_cast<double>(i) / static_cast<double>(n_seg);
      chain_xy.emplace_back(dx + ux * s, dy + uy * s);
    }
    bool g_on_chain = false;
    for (const auto & p : chain_xy) {
      if (squaredDistanceXY(p.first, p.second, gx0, gy0) < 0.01) {
        g_on_chain = true;
        break;
      }
    }
    if (!g_on_chain) {
      const double sg = (gx0 - dx) * ux + (gy0 - dy) * uy;
      std::size_t insert_at = chain_xy.size();
      for (std::size_t i = 0; i < chain_xy.size(); ++i) {
        const double si =
          (chain_xy[i].first - dx) * ux + (chain_xy[i].second - dy) * uy;
        if (si > sg) {
          insert_at = i;
          break;
        }
      }
      chain_xy.insert(
        chain_xy.begin() + static_cast<std::ptrdiff_t>(insert_at), {gx0, gy0});
    }
  }

  for (auto & p : chain_xy) {
    p.first += off * nx;
    p.second += off * ny;
  }
  dx += off * nx;
  dy += off * ny;
  ex += off * nx;
  ey += off * ny;
  const double gx = gx0 + off * nx;
  const double gy = gy0 + off * ny;

  if (std::fabs(off) > 1e-9) {
    std::string post_reason;
    const double yaw_trav_d = std::atan2(gy - dy, gx - dx);
    if (!isFootprintClearAtPose(dx, dy, yaw_trav_d, &post_reason) ||
      !isFootprintSweepClear(dx, dy, ex, ey, &post_reason))
    {
      chain.invalid_reason =
        "wall offset breaks footprint: " + post_reason;
      return chain;
    }
  }
  if (!segmentClearOfObstacle(dx, dy, gx, gy) ||
    !segmentClearOfObstacle(gx, gy, ex, ey))
  {
    chain.invalid_reason = "D-G-E line crosses obstacle";
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: wall-edge D-G-E 连线穿过障碍, 不生成 "
      "D=(%.2f, %.2f) G=(%.2f, %.2f) E=(%.2f, %.2f) P=(%.2f, %.2f)",
      dx, dy, gx, gy, ex, ey, px, py);
    return chain;
  }

  chain.valid = true;
  chain.has_d = true;
  chain.has_e = true;
  chain.xy = std::move(chain_xy);
  chain.dx = dx;
  chain.dy = dy;
  chain.gx = gx;
  chain.gy = gy;
  chain.ex = ex;
  chain.ey = ey;
  chain.path_yaw = e_yaw;
  chain.extend_used_m = extend_m;
  chain.px = px;
  chain.py = py;
  chain.normal_offset_m = off;

  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: wall-edge chain D=(%.2f, %.2f) G=(%.2f, %.2f) E=(%.2f, %.2f) "
    "n=%zu sample=%.2f offset=%.2f P=(%.2f, %.2f)",
    dx, dy, gx, gy, ex, ey, chain.xy.size(), spacing, off, px, py);
  return chain;
}

}  // namespace nav2_behavior_tree
