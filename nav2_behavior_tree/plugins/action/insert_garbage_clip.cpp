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

// 垃圾几何：判断要删除的 goals 下标，以及拐角


namespace
{

// 点到有限线段 AB 的距离平方；t_out 为夹在 [0,1] 的投影参数
double squaredDistancePointToSegment(
  double px, double py,
  double ax, double ay,
  double bx, double by,
  double * t_out = nullptr)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab_len2 = abx * abx + aby * aby;
  double t = 0.0;
  if (ab_len2 > 1e-12) {
    t = (apx * abx + apy * aby) / ab_len2;
    t = std::clamp(t, 0.0, 1.0);
  }
  if (t_out) {
    *t_out = t;
  }
  const double qx = ax + t * abx;
  const double qy = ay + t * aby;
  const double dx = px - qx;
  const double dy = py - qy;
  return dx * dx + dy * dy;
}

}  // namespace

// 点到无限直线 AB 的垂足
void InsertGarbagePose::projectPointToInfiniteLine(
  double px, double py,
  double ax, double ay,
  double bx, double by,
  double & out_x, double & out_y)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab_len2 = abx * abx + aby * aby;
  if (ab_len2 < 1e-12) {
    out_x = ax;
    out_y = ay;
    return;
  }
  const double apx = px - ax;
  const double apy = py - ay;
  const double t = (apx * abx + apy * aby) / ab_len2;
  out_x = ax + t * abx;
  out_y = ay + t * aby;
}
// 点在无限直线 AB 上的参数 t
double InsertGarbagePose::lineParameterT(
  double px, double py,
  double ax, double ay,
  double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab_len2 = abx * abx + aby * aby;
  if (ab_len2 < 1e-12) {
    return 0.0;
  }
  return ((px - ax) * abx + (py - ay) * aby) / ab_len2;
}

// ========================================================================
// F. 插入信息与裁剪
// ========================================================================

double InsertGarbagePose::preferExtendYawAwayFromRobot(
  double gx, double gy, double yaw,
  double robot_x, double robot_y)
{
  const double dx = robot_x - gx;
  const double dy = robot_y - gy;
  if (std::hypot(dx, dy) < 1e-3) {
    return yaw;
  }
  const double fx = std::cos(yaw);
  const double fy = std::sin(yaw);
  // G→E 与 G→车同侧：E 会落在车旁/同向，翻转到对侧
  if (dx * fx + dy * fy > 0.0) {
    return std::atan2(-fy, -fx);
  }
  return yaw;
}
// 获取插入所需的全部信息并返回
InsertGarbagePose::InsertInfo InsertGarbagePose::gatherInsertInfo(
  const Goals & goals,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double garbage_x, double garbage_y)
{
  InsertInfo info;
  info.goals = goals;
  info.robot_pose = robot_pose;
  info.garbage.pose.pose.position.x = garbage_x;
  info.garbage.pose.pose.position.y = garbage_y;
  const double gx = garbage_x;
  const double gy = garbage_y;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  info.radius_m = std::sqrt(squaredDistanceXY(rx, ry, gx, gy));
  if (info.radius_m < 1e-6) {
    info.invalid_reason = "radius ~ 0";
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(), gx, gy);
    return info;
  }

  auto fail_unless_ac_fallback = [&](const char * why) {
    if (fillAcWhenOriginalPathGone(&info, gx, gy)) {
      return false;
    }
    info.invalid_reason = why;
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "InsertGarbagePose: gather invalid (%s) for garbage at (%.2f, %.2f)",
      info.invalid_reason.c_str(), gx, gy);
    return true;
  };

  bool ac_ok = false;
  if (info.goals.size() >= 2) {
    // 投影线起点取剩余队列头（当前长边队首），已插入的 G/E 不参与
    const std::size_t head_idx = ordinaryQueueHead(info.goals);
    if (head_idx < info.goals.size()) {
      info.goala = info.goals[head_idx];
      if (!findFirstCornerFromRobot(info.goals, rx, ry, info.goalc_idx)) {
        if (findLastGoalWithinPathRange(
            info.goals, rx, ry, goaltotal_range_m_, info.goalc_idx))
        {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "InsertGarbagePose: no corner, use last goal within %.2fm as goalc (idx %zu)",
            goaltotal_range_m_, info.goalc_idx);
        } else {
          info.goalc_idx = head_idx;
        }
      }
      if (info.goalc_idx < info.goals.size()) {
        info.goalc = info.goals[info.goalc_idx];
      }
      const bool closed_side =
        info.goalc_idx == head_idx &&
        isRememberedCorner(
          info.goala.pose.position.x, info.goala.pose.position.y);
      if (!closed_side &&
        squaredDistanceXY(
          info.goala.pose.position.x, info.goala.pose.position.y,
          info.goalc.pose.position.x, info.goalc.pose.position.y) < 1e-12)
      {
        std::size_t next_c = 0;
        if (findNextCornerAfter(info.goals, head_idx, rx, ry, next_c)) {
          info.goalc_idx = next_c;
        } else if (findLastGoalWithinPathRange(
            info.goals, rx, ry, goaltotal_range_m_, next_c) &&
          next_c > head_idx)
        {
          info.goalc_idx = next_c;
        } else if (head_idx + 1 < info.goals.size()) {
          info.goalc_idx = head_idx + 1;
        }
        if (info.goalc_idx < info.goals.size()) {
          info.goalc = info.goals[info.goalc_idx];
        }
      }
      if (closed_side) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: hold corner as C (%.2f, %.2f), robot not on next side",
          info.goala.pose.position.x, info.goala.pose.position.y);
        ac_ok = true;
      } else if (squaredDistanceXY(
          info.goala.pose.position.x, info.goala.pose.position.y,
          info.goalc.pose.position.x, info.goalc.pose.position.y) >= 1e-12)
      {
        ac_ok = true;
      }
    }
  }
  if (!ac_ok) {
    const char * why = (info.goals.size() < 2) ?
      "goals size < 2" : "goala and goalc too close";
    if (fail_unless_ac_fallback(why)) {
      return info;
    }
  }

  // goald：垃圾投影到 goala-goalc 无限直线的垂足
  projectPointToInfiniteLine(
    gx, gy,
    info.goala.pose.position.x, info.goala.pose.position.y,
    info.goalc.pose.position.x, info.goalc.pose.position.y,
    info.goald_x, info.goald_y);

  // 插入朝向 / 默认伸 E：后一堆按规划来向，用上一离开点（仍在 goals 里的 G/E），
  // 不用发现时的车位，避免侧向堆的 E 和车头同向。
  double from_x = rx;
  double from_y = ry;
  const char * yaw_src = "robot->G";
  if (has_last_sweep_arrive_ &&
    findUnindexedSentinelIndex(
      info.goals,
      last_sweep_arrive_xy_.first, last_sweep_arrive_xy_.second, nullptr))
  {
    from_x = last_sweep_arrive_xy_.first;
    from_y = last_sweep_arrive_xy_.second;
    yaw_src = "arrive->G";
  } else if (has_last_sweep_arrive_) {
    yaw_src = "robot->G, prev arrive not in goals";
  }
  const double min_from_m = kMinExtendFromDistM;
  const double dx_from = gx - from_x;
  const double dy_from = gy - from_y;
  const double from_dist = std::hypot(dx_from, dy_from);
  const double back_m = std::max(garbage_extend_m_, min_from_m * 2.0);
  if (from_dist < min_from_m) {
    // 来向贴 G：优先延续上一堆扫向，避免退回车头导致后堆 E∥机器人
    double yaw_fwd = tf2::getYaw(robot_pose.pose.orientation);
    if (has_last_sweep_path_yaw_) {
      yaw_fwd = last_sweep_path_yaw_;
      yaw_src = "on-G, forward=last_sweep_yaw";
    } else {
      yaw_src = "on-G, forward=robot_yaw";
    }
    from_x = gx - back_m * std::cos(yaw_fwd);
    from_y = gy - back_m * std::sin(yaw_fwd);
    info.path_yaw = yaw_fwd;
  } else {
    info.path_yaw = std::atan2(dy_from, dx_from);
  }
  {
    const double before = info.path_yaw;
    // 用进近来向 from，不用发现时车位，避免后堆 E 按旧车位翻反
    info.path_yaw = preferExtendYawAwayFromRobot(
      gx, gy, info.path_yaw, from_x, from_y);
    if (std::cos(info.path_yaw - before) < 0.0) {
      // 翻转后保证 from 仍在 G 后方（-path_yaw 侧），墙切向选侧才一致
      from_x = gx - back_m * std::cos(info.path_yaw);
      from_y = gy - back_m * std::sin(info.path_yaw);
      yaw_src = "flipped away from approach";
    }
  }
  info.extend_from_x = from_x;
  info.extend_from_y = from_y;
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: path_yaw=%.3f rad (%s) from=(%.2f, %.2f) garbage=(%.2f, %.2f)",
    info.path_yaw, yaw_src, from_x, from_y, gx, gy);

  info.valid = true;
  return info;
}

bool InsertGarbagePose::fillAcWhenOriginalPathGone(
  InsertInfo * info, double gx, double gy) const
{
  if (info == nullptr) {
    return false;
  }
  constexpr double kMinAc2 = 1e-12;
  const Goals & goals = info->goals;

  auto accept = [&](
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & c,
    std::size_t c_idx,
    const char * src)
  {
    const double ax = a.pose.position.x;
    const double ay = a.pose.position.y;
    const double cx = c.pose.position.x;
    const double cy = c.pose.position.y;
    if (squaredDistanceXY(ax, ay, cx, cy) < kMinAc2) {
      return false;
    }
    info->goala = a;
    info->goalc = c;
    info->goalc_idx = c_idx;
    info->invalid_reason.clear();
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: A-C fallback %s A=(%.2f, %.2f) C=(%.2f, %.2f) G=(%.2f, %.2f)",
      src, ax, ay, cx, cy, gx, gy);
    return true;
  };

  // 已有 G-E：用最后一对哨兵当通道线（第一堆把原路径删光后的常态）
  int last_s = -1;
  int prev_s = -1;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (isUnindexedSentinelPoseZ(goals[i])) {
      prev_s = last_s;
      last_s = static_cast<int>(i);
    }
  }
  if (prev_s >= 0 && last_s >= 0 &&
    accept(
      goals[static_cast<std::size_t>(prev_s)],
      goals[static_cast<std::size_t>(last_s)],
      static_cast<std::size_t>(last_s), "G-E"))
  {
    return true;
  }

  // 还剩一个原路径点：车 → 该点
  int orig = -1;
  for (int i = static_cast<int>(goals.size()) - 1; i >= 0; --i) {
    const auto & g = goals[static_cast<std::size_t>(i)];
    if (isUnindexedSentinelPoseZ(g)) {
      continue;
    }
    if (isProtectedGarbageXy(g.pose.position.x, g.pose.position.y)) {
      continue;
    }
    orig = i;
    break;
  }
  if (orig >= 0 &&
    accept(
      info->robot_pose, goals[static_cast<std::size_t>(orig)],
      static_cast<std::size_t>(orig), "robot->path"))
  {
    return true;
  }

  // 原路径和 G-E 都不够：车 → G，只为出 yaw/E，后面不靠这条线删点
  geometry_msgs::msg::PoseStamped g_pose = info->robot_pose;
  g_pose.pose.position.x = gx;
  g_pose.pose.position.y = gy;
  if (accept(info->robot_pose, g_pose, 0, "robot->G")) {
    return true;
  }
  return false;
}

namespace
{

struct ClipNode
{
  std::size_t orig{0};
  geometry_msgs::msg::PoseStamped pose;
};

}  // namespace

// 判断和删除分开。每个参考点都从同一份原始路径算自己的垂足，互不看到对方要删的点。
// 垂足在线段上时，先收当前边首点到垂足，再从垂足沿后续普通点累加路程，
// 超过 clip_extend_m 或碰到角点就停。两截只写入 delete_idx，不改这份路径。
// 垂足在正向延长线上时，只在这个参考点自己的副本上重算下一条边。
// delete_idx 是各参考点原始下标的并集，由调用方一次删除。
void InsertGarbagePose::clipReferencesInOrder(
  const Goals & goals,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<std::pair<double, double>> & refs,
  std::set<std::size_t> & delete_idx,
  InsertInfo & info,
  bool commit_memory)
{
  info.clip_rounds.clear();
  info.corners_kept_xy.clear();
  info.hit_mid_case = false;
  info.hit_forward_case = false;
  if (goals.size() < 2 || refs.empty()) {
    return;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  double nearest_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
    nearest_d2 = std::min(
      nearest_d2,
      squaredDistancePointToSegment(
        rx, ry,
        goals[i].pose.position.x, goals[i].pose.position.y,
        goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
  }
  if (std::sqrt(nearest_d2) > head_delete_robot_dist_m_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: robot %.2fm from input_goals > %.2fm, skip all deletes",
      std::sqrt(nearest_d2), head_delete_robot_dist_m_);
    return;
  }

  std::vector<ClipNode> base_nodes;
  base_nodes.reserve(goals.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    base_nodes.push_back(ClipNode{i, goals[i]});
  }

  constexpr double kEps = 1e-6;
  const double clip_m = clip_extend_m_;
  int round_i = 0;

  for (const auto & ref : refs) {
    std::vector<ClipNode> nodes = base_nodes;
    auto canDelete = [this, &nodes](std::size_t node_i) {
      const auto & pose = nodes[node_i].pose;
      if (isUnindexedSentinelPoseZ(pose)) {
        return false;
      }
      if (isProtectedGarbageXy(pose.pose.position.x, pose.pose.position.y)) {
        return false;
      }
      return true;
    };
    auto dropNodes = [&](const std::vector<std::size_t> & origs) {
      if (origs.empty()) {
        return;
      }
      std::set<std::size_t> drop(origs.begin(), origs.end());
      std::vector<ClipNode> kept;
      kept.reserve(nodes.size());
      for (const auto & node : nodes) {
        if (drop.count(node.orig) == 0) {
          kept.push_back(node);
        } else {
          delete_idx.insert(node.orig);
        }
      }
      nodes.swap(kept);
    };

    const double ref_x = ref.first;
    const double ref_y = ref.second;
    const std::size_t round_limit = nodes.size() + 1;
    for (std::size_t round = 0; round < round_limit; ++round) {
      Goals live;
      live.reserve(nodes.size());
      for (const auto & node : nodes) {
        live.push_back(node.pose);
      }
      std::vector<std::size_t> ordinary;
      ordinary.reserve(live.size());
      for (std::size_t i = 0; i < live.size(); ++i) {
        if (!isUnindexedSentinelPoseZ(live[i])) {
          ordinary.push_back(i);
        }
      }
      if (ordinary.size() < 2) {
        break;
      }

      // 当前边：第一个非 z=-1 → 其后第一个角点。队首本身不当这条边的终点。
      const std::size_t H = ordinary.front();
      std::size_t C = ordinary.back();
      for (std::size_t k = 1; k + 1 < ordinary.size(); ++k) {
        if (!isGoalNotCorner(live, ordinary[k], rx, ry)) {
          C = ordinary[k];
          break;
        }
      }

      const double hx = live[H].pose.position.x;
      const double hy = live[H].pose.position.y;
      const double cx = live[C].pose.position.x;
      const double cy = live[C].pose.position.y;
      const double seg_len = std::hypot(cx - hx, cy - hy);
      if (seg_len < 1e-9) {
        break;
      }
      double fx = 0.0;
      double fy = 0.0;
      projectPointToInfiniteLine(ref_x, ref_y, hx, hy, cx, cy, fx, fy);
      const double t = lineParameterT(fx, fy, hx, hy, cx, cy);

      InsertInfo::ClipRound clip_round;
      clip_round.round_i = ++round_i;
      clip_round.ax = hx;
      clip_round.ay = hy;
      clip_round.cx = cx;
      clip_round.cy = cy;
      clip_round.fx = fx;
      clip_round.fy = fy;
      clip_round.t_d = t;
      info.clip_rounds.push_back(clip_round);
      info.goala = live[H];
      info.goalc = live[C];
      info.goalc_idx = nodes[C].orig;

      if (t < -kEps) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: clip reverse t=%.3f, skip H=(%.2f, %.2f) C=(%.2f, %.2f)",
          t, hx, hy, cx, cy);
        break;
      }

      std::vector<std::size_t> drop_orig;
      if (t > 1.0 + kEps) {
        // 只挪这个参考点自己的副本，好让它去看下一条边。其它参考点仍从原始路径算。
        info.hit_forward_case = true;
        for (std::size_t j = H; j < C; ++j) {
          if (canDelete(j)) {
            drop_orig.push_back(nodes[j].orig);
          }
        }
        info.corners_kept_xy.emplace_back(cx, cy);
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: clip beyond t=%.3f, drop %zu before corner (%.2f, %.2f)",
          t, drop_orig.size(), cx, cy);
        if (drop_orig.empty()) {
          break;
        }
        dropNodes(drop_orig);
        continue;
      }

      info.hit_mid_case = true;
      info.corners_kept_xy.emplace_back(cx, cy);
      std::vector<std::size_t> edge;
      edge.reserve(ordinary.size());
      for (const std::size_t idx : ordinary) {
        edge.push_back(idx);
        if (idx == C) {
          break;
        }
      }
      std::size_t next_k = edge.size();
      for (std::size_t k = 0; k < edge.size(); ++k) {
        const auto & p = live[edge[k]].pose.position;
        const double pt = lineParameterT(p.x, p.y, hx, hy, cx, cy);
        if (pt > t + kEps) {
          next_k = k;
          break;
        }
      }
      std::size_t head_drop = 0;
      for (std::size_t k = 0; k < next_k; ++k) {
        const std::size_t j = edge[k];
        if (j == C) {
          break;
        }
        if (canDelete(j)) {
          drop_orig.push_back(nodes[j].orig);
          ++head_drop;
        }
      }
      double prev_x = fx;
      double prev_y = fy;
      double accumulated = 0.0;
      for (std::size_t k = next_k; k < edge.size(); ++k) {
        const std::size_t j = edge[k];
        if (j == C) {
          break;
        }
        const double x = live[j].pose.position.x;
        const double y = live[j].pose.position.y;
        const double seg = std::hypot(x - prev_x, y - prev_y);
        if (accumulated + seg >= clip_m) {
          break;
        }
        accumulated += seg;
        if (canDelete(j)) {
          drop_orig.push_back(nodes[j].orig);
        }
        prev_x = x;
        prev_y = y;
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: clip on-seg t=%.3f H=%zu C=%zu walked=%.2f "
        "head_drop=%zu drop=%zu ref=(%.2f, %.2f)",
        t, nodes[H].orig, nodes[C].orig, accumulated, head_drop, drop_orig.size(),
        ref_x, ref_y);
      for (const std::size_t orig : drop_orig) {
        delete_idx.insert(orig);
      }
      break;
    }
  }

  if (commit_memory) {
    for (const auto & xy : info.corners_kept_xy) {
      rememberCornerXy(xy.first, xy.second);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: clip refs=%zu rounds=%zu n_del=%zu N=%zu",
    refs.size(), info.clip_rounds.size(), delete_idx.size(), goals.size());
}

// ========================================================================
// G. 角点 / 路径几何
// ========================================================================

// true=非角点，false=角点。z=-1 不能当角点；只在非 z=-1 点上算夹角，

bool InsertGarbagePose::isGoalNotCorner(
  const Goals & goals,
  std::size_t idx,
  double robot_x, double robot_y) const
{
  (void)robot_x;
  (void)robot_y;
  if (goals.empty() || idx >= goals.size()) {
    return true;
  }
  if (isUnindexedSentinelPoseZ(goals[idx])) {
    return true;
  }

  int prev_i = -1;
  for (int i = static_cast<int>(idx) - 1; i >= 0; --i) {
    if (!isUnindexedSentinelPoseZ(goals[static_cast<std::size_t>(i)])) {
      prev_i = i;
      break;
    }
  }
  std::size_t next_i = goals.size();
  for (std::size_t i = idx + 1; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      next_i = i;
      break;
    }
  }
  // 非 z=-1 里的最后一点：收住最后一段
  if (next_i >= goals.size()) {
    return false;
  }
  if (prev_i < 0 || goals.size() < 3) {
    return true;
  }

  const double cx = goals[idx].pose.position.x;
  const double cy = goals[idx].pose.position.y;
  const double to_prev_x = goals[static_cast<std::size_t>(prev_i)].pose.position.x - cx;
  const double to_prev_y = goals[static_cast<std::size_t>(prev_i)].pose.position.y - cy;
  const double to_next_x = goals[next_i].pose.position.x - cx;
  const double to_next_y = goals[next_i].pose.position.y - cy;

  const double len_prev_sq = to_prev_x * to_prev_x + to_prev_y * to_prev_y;
  const double len_next_sq = to_next_x * to_next_x + to_next_y * to_next_y;
  constexpr double kMinSegLenSq = 1e-6;
  if (len_prev_sq < kMinSegLenSq || len_next_sq < kMinSegLenSq) {
    return true;
  }

  // 直线时两方向相反，夹角约 180°；直角约 90°
  const double cos_theta = std::clamp(
    (to_prev_x * to_next_x + to_prev_y * to_next_y) /
    std::sqrt(len_prev_sq * len_next_sq), -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  const double corner_angle_rad = corner_angle_deg_ * M_PI / 180.0;
  return theta >= (M_PI - corner_angle_rad);
}
// 剩余队列第一个非 z=-1 点：工字当前长边队首，不用欧氏最近
std::size_t InsertGarbagePose::ordinaryQueueHead(const Goals & goals) const
{
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!isUnindexedSentinelPoseZ(goals[i])) {
      return i;
    }
  }
  return goals.size();
}

bool InsertGarbagePose::isRememberedCorner(double x, double y) const
{
  constexpr double kTol2 = 0.08 * 0.08;
  for (const auto & c : remembered_corner_xy_) {
    if (squaredDistanceXY(c.first, c.second, x, y) < kTol2) {
      return true;
    }
  }
  return false;
}

void InsertGarbagePose::rememberCornerXy(double x, double y) const
{
  if (isRememberedCorner(x, y)) {
    return;
  }
  remembered_corner_xy_.emplace_back(x, y);
}

void InsertGarbagePose::forgetCornerXy(double x, double y) const
{
  constexpr double kTol2 = 0.08 * 0.08;
  remembered_corner_xy_.erase(
    std::remove_if(
      remembered_corner_xy_.begin(), remembered_corner_xy_.end(),
      [x, y](const std::pair<double, double> & c) {
        return squaredDistanceXY(c.first, c.second, x, y) < kTol2;
      }),
    remembered_corner_xy_.end());
}

bool InsertGarbagePose::robotEnteredNextSide(
  const Goals & goals, std::size_t head, double robot_x, double robot_y) const
{
  if (head + 1 >= goals.size()) {
    return false;
  }
  double t = 0.0;
  const double d2 = squaredDistancePointToSegment(
    robot_x, robot_y,
    goals[head].pose.position.x, goals[head].pose.position.y,
    goals[head + 1].pose.position.x, goals[head + 1].pose.position.y,
    &t);
  if (d2 > 1.5 * 1.5) {
    return false;
  }
  const double seg = std::sqrt(squaredDistanceXY(
    goals[head].pose.position.x, goals[head].pose.position.y,
    goals[head + 1].pose.position.x, goals[head + 1].pose.position.y));
  return t * seg > 0.25;
}
// 从当前长边队首往后找第一个角点
bool InsertGarbagePose::findFirstCornerFromRobot(
  const Goals & goals,
  double robot_x, double robot_y,
  std::size_t & corner_idx) const
{
  if (goals.size() < 2) {
    return false;
  }

  const std::size_t head = ordinaryQueueHead(goals);
  if (head >= goals.size()) {
    return false;
  }
  const double hx = goals[head].pose.position.x;
  const double hy = goals[head].pose.position.y;
  if (isRememberedCorner(hx, hy)) {
    if (robotEnteredNextSide(goals, head, robot_x, robot_y)) {
      forgetCornerXy(hx, hy);
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: corner (%.2f, %.2f) passed, open next side",
        hx, hy);
    } else {
      corner_idx = head;
      return true;
    }
  }
  const std::size_t start_idx = head;
  if (start_idx >= goals.size()) {
    return false;
  }

  std::size_t range_end = start_idx;
  double accumulated = 0.0;
  for (std::size_t i = head; i + 1 < goals.size(); ++i) {
    accumulated += std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    if (accumulated > goaltotal_range_m_) {
      break;
    }
    range_end = i + 1;
  }

  for (std::size_t i = start_idx; i <= range_end && i < goals.size(); ++i) {
    if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
      continue;
    }
    if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
      corner_idx = i;
      return true;
    }
  }
  return false;
}
// 从 after_idx 之后找下一个角点
bool InsertGarbagePose::findNextCornerAfter(
  const Goals & goals,
  std::size_t after_idx,
  double robot_x, double robot_y,
  std::size_t & corner_idx) const
{
  if (goals.size() < 2 || after_idx + 1 >= goals.size()) {
    return false;
  }

  // 从 after_idx 起沿路径累加 range，得到搜索上界
  std::size_t range_end = after_idx;
  double accumulated = 0.0;
  for (std::size_t i = after_idx; i + 1 < goals.size(); ++i) {
    accumulated += std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    if (accumulated > goaltotal_range_m_) {
      break;
    }
    range_end = i + 1;
  }
  if (range_end <= after_idx) {
    return false;
  }

  for (std::size_t i = after_idx + 1; i <= range_end; ++i) {
    if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
      continue;
    }
    if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
      corner_idx = i;
      return true;
    }
  }
  return false;
}
// 根据累计路径 range_m ，去找最后一个点的下标
bool InsertGarbagePose::findLastGoalWithinPathRange(
  const Goals & goals,
  double robot_x, double robot_y,
  double range_m,
  std::size_t & out_idx,
  std::size_t * nearest_seg_out,
  std::size_t * start_idx_out) const
{
  if (goals.size() < 2) {
    return false;
  }
  (void)robot_x;
  (void)robot_y;

  // 从当前长边队首沿路径量 range，不用欧氏最近段
  std::size_t start_idx = ordinaryQueueHead(goals);
  if (start_idx >= goals.size()) {
    start_idx = 0;
  }

  double accumulated = 0.0;
  out_idx = start_idx;
  for (std::size_t i = start_idx; i + 1 < goals.size(); ++i) {
    const double seg_len = std::sqrt(squaredDistanceXY(
      goals[i].pose.position.x, goals[i].pose.position.y,
      goals[i + 1].pose.position.x, goals[i + 1].pose.position.y));
    accumulated += seg_len;
    if (accumulated > range_m) {
      break;
    }
    out_idx = i + 1;
  }

  if (nearest_seg_out) {
    *nearest_seg_out = start_idx;
  }
  if (start_idx_out) {
    *start_idx_out = start_idx;
  }
  return true;
}


InsertGarbagePose::Goals InsertGarbagePose::clipWithRefs(
  const Goals & work,
  const std::vector<std::pair<double, double>> & refs,
  const InsertInfo & frozen,
  InsertInfo * info,
  const char * log_tag,
  std::set<std::size_t> * defer_delete)
{
  InsertInfo cinfo = frozen;
  cinfo.goals = work;
  std::set<std::size_t> del;
  clipReferencesInOrder(work, info->robot_pose, refs, del, cinfo, true);
  info->clip_rounds = std::move(cinfo.clip_rounds);
  info->corners_kept_xy = std::move(cinfo.corners_kept_xy);
  info->hit_mid_case = cinfo.hit_mid_case;
  info->hit_forward_case = cinfo.hit_forward_case;
  if (cinfo.goalc_idx < work.size()) {
    info->goalc = work[cinfo.goalc_idx];
    info->goalc_idx = cinfo.goalc_idx;
  }
  info->goala = cinfo.goala;
  info->goaltotal.clear();
  info->goaltotal.reserve(del.size());
  for (const std::size_t idx : del) {
    if (idx < work.size()) {
      info->goaltotal.push_back(work[idx]);
    }
  }
  if (defer_delete != nullptr) {
    defer_delete->insert(del.begin(), del.end());
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: %s defer %zu refs=%zu, path %zu unchanged",
      log_tag, del.size(), refs.size(), work.size());
    return work;
  }
  Goals clipped;
  clipped.reserve(work.size());
  for (std::size_t i = 0; i < work.size(); ++i) {
    if (del.count(i) == 0) {
      clipped.push_back(work[i]);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: %s %zu refs=%zu, path %zu -> %zu",
    log_tag, del.size(), refs.size(), work.size(), clipped.size());
  return clipped;
}

void InsertGarbagePose::deferOpenRayClip(
  const Goals & clip_tail,
  const std::vector<std::pair<double, double>> & refs,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  std::set<std::size_t> * pending_delete)
{
  if (pending_delete == nullptr || refs.empty() || clip_tail.size() < 2) {
    return;
  }
  InsertInfo info;
  info.valid = true;
  info.robot_pose = robot_pose;
  info.goals = clip_tail;
  const InsertInfo frozen = info;
  clipWithRefs(
    clip_tail, refs, frozen, &info, "open ray union delete", pending_delete);
}

}  // namespace nav2_behavior_tree
