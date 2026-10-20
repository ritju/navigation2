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

// 开放扫向链：判断新垃圾接到哪条链上，并写回 goals

void InsertGarbagePose::forgetProtectedGarbageXy(double x, double y)
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  reached_garbage_xy_.erase(
    std::remove_if(
      reached_garbage_xy_.begin(), reached_garbage_xy_.end(),
      [&](const std::pair<double, double> & reached) {
        return squaredDistanceXY(x, y, reached.first, reached.second) < thresh2;
      }),
    reached_garbage_xy_.end());
}

bool InsertGarbagePose::isOpenRayExtendPoint(double x, double y) const
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (const auto & ray : open_garbage_rays_) {
    if (!ray.has_e) {
      continue;
    }
    if (squaredDistanceXY(x, y, ray.ex, ray.ey) >= thresh2) {
      continue;
    }
    bool also_g = false;
    for (const auto & other : open_garbage_rays_) {
      if (squaredDistanceXY(x, y, other.gx, other.gy) < thresh2) {
        also_g = true;
        break;
      }
    }
    if (!also_g) {
      return true;
    }
  }
  return false;
}

void InsertGarbagePose::rememberOpenGarbageRay(
  double gx, double gy, double yaw, bool has_e, double ex, double ey,
  double fx, double fy)
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (auto & ray : open_garbage_rays_) {
    if (squaredDistanceXY(ray.gx, ray.gy, gx, gy) >= thresh2) {
      continue;
    }
    ray.yaw = yaw;
    ray.has_e = has_e;
    ray.ex = ex;
    ray.ey = ey;
    ray.fx = fx;
    ray.fy = fy;
    return;
  }
  OpenGarbageRay ray;
  ray.gx = gx;
  ray.gy = gy;
  ray.yaw = yaw;
  ray.has_e = has_e;
  ray.ex = ex;
  ray.ey = ey;
  ray.fx = fx;
  ray.fy = fy;
  open_garbage_rays_.push_back(ray);
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: open ray on G=(%.2f, %.2f) yaw=%.3f from=(%.2f, %.2f)",
    gx, gy, yaw, fx, fy);
}

void InsertGarbagePose::updateOpenRayFrom(double gx, double gy, double fx, double fy)
{
  const double thresh2 = kSentinelIdentityMatchM * kSentinelIdentityMatchM;
  for (auto & ray : open_garbage_rays_) {
    if (squaredDistanceXY(ray.gx, ray.gy, gx, gy) >= thresh2) {
      continue;
    }
    ray.fx = fx;
    ray.fy = fy;
  }
}

bool InsertGarbagePose::onReverseApproachSegment(
  double gx, double gy, double yaw, double fx, double fy,
  double px, double py, double robot_x, double robot_y, double * proj_out) const
{
  double proj = 0.0;
  if (!onOpenRayAxis(gx, gy, yaw, px, py, &proj) || proj > 1e-3) {
    return false;
  }
  const double from_proj = (fx - gx) * std::cos(yaw) + (fy - gy) * std::sin(yaw);
  // from 落在 G 前方时这段不存在，反向的点不并进这条链
  if (from_proj >= -1e-3) {
    return false;
  }
  double start_proj = from_proj - garbage_merge_radius_m_;
  double robot_proj = 0.0;
  const bool robot_on_ray = onOpenRayAxis(gx, gy, yaw, robot_x, robot_y, &robot_proj);
  // 车已经开上 from 到 G 这一段：起点收到车的投影，车后面的不再折返
  if (robot_on_ray && robot_proj > from_proj && robot_proj < 1e-3) {
    start_proj = robot_proj - garbage_merge_radius_m_;
  }
  if (proj < start_proj) {
    return false;
  }
  if (proj_out != nullptr) {
    *proj_out = proj;
  }
  return true;
}

void InsertGarbagePose::pruneOpenGarbageRays(const Goals & goals)
{
  std::vector<OpenGarbageRay> kept;
  kept.reserve(open_garbage_rays_.size());
  for (const OpenGarbageRay & ray : open_garbage_rays_) {
    if (findUnindexedSentinelIndex(goals, ray.gx, ray.gy, nullptr)) {
      kept.push_back(ray);
      continue;
    }
    // G 已被车经过删掉，E 还在：射线继续开着，后面的点还能接到这条线上
    if (ray.has_e && findUnindexedSentinelIndex(goals, ray.ex, ray.ey, nullptr)) {
      kept.push_back(ray);
      continue;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "InsertGarbagePose: open ray drop G=(%.2f, %.2f), gone from goals",
      ray.gx, ray.gy);
  }
  open_garbage_rays_.swap(kept);
}

bool InsertGarbagePose::onOpenRayAxis(
  double ox, double oy, double yaw, double px, double py, double * proj_out) const
{
  const double vx = px - ox;
  const double vy = py - oy;
  const double vlen = std::hypot(vx, vy);
  if (vlen < 1e-6) {
    return false;
  }
  const double proj = vx * std::cos(yaw) + vy * std::sin(yaw);
  const double cosang = std::clamp(std::fabs(proj) / vlen, 0.0, 1.0);
  if (std::acos(cosang) > ray_offset_deg_ * M_PI / 180.0) {
    return false;
  }
  if (proj_out != nullptr) {
    *proj_out = proj;
  }
  return true;
}

bool InsertGarbagePose::attachNewGarbageToOpenRays(
  Goals * goals, const geometry_msgs::msg::PoseStamped & robot_pose,
  std::vector<std::pair<double, double>> * clip_refs)
{
  if (goals == nullptr || open_garbage_rays_.empty() || garbage_list_.empty()) {
    return false;
  }

  struct Hit
  {
    std::size_t list_idx{0};
    std::size_t ray_idx{0};
    double proj{0.0};
  };
  std::vector<Hit> hits;
  std::vector<std::size_t> passed_robot;
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    const double px = garbage_list_[i].pose.pose.position.x;
    const double py = garbage_list_[i].pose.pose.position.y;
    std::size_t best = open_garbage_rays_.size();
    double best_proj = 0.0;
    double best_abs = std::numeric_limits<double>::infinity();
    bool passed = false;
    for (std::size_t r = 0; r < open_garbage_rays_.size(); ++r) {
      const OpenGarbageRay & ray = open_garbage_rays_[r];
      double proj = 0.0;
      if (!onOpenRayAxis(ray.gx, ray.gy, ray.yaw, px, py, &proj)) {
        continue;
      }
      // G 前方沿用原来的扇形。反方向只认 from 到链头这一段，from 后面的当新堆
      if (proj <= 1e-3 &&
        !onReverseApproachSegment(
          ray.gx, ray.gy, ray.yaw, ray.fx, ray.fy,
          px, py, robot_x, robot_y, &proj))
      {
        const double from_proj =
          (ray.fx - ray.gx) * std::cos(ray.yaw) +
          (ray.fy - ray.gy) * std::sin(ray.yaw);
        double robot_proj = 0.0;
        const bool robot_on_ray = onOpenRayAxis(
          ray.gx, ray.gy, ray.yaw, robot_x, robot_y, &robot_proj);
        if (robot_on_ray && robot_proj > from_proj && robot_proj < 1e-3 &&
          proj <= robot_proj + 1e-3 && proj >= from_proj - garbage_merge_radius_m_)
        {
          passed = true;
        }
        continue;
      }
      const double abs_proj = std::fabs(proj);
      if (abs_proj < best_abs) {
        best_abs = abs_proj;
        best_proj = proj;
        best = r;
      }
    }
    if (best < open_garbage_rays_.size()) {
      hits.push_back(Hit{i, best, best_proj});
    } else if (passed) {
      passed_robot.push_back(i);
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: open ray 后方已过 (%.2f, %.2f)，不折返",
        px, py);
    }
  }
  std::vector<std::size_t> consumed = passed_robot;
  for (const std::size_t idx : passed_robot) {
    if (idx >= garbage_list_.size()) {
      continue;
    }
    addProtectedGarbageXy(
      garbage_list_[idx].pose.pose.position.x,
      garbage_list_[idx].pose.pose.position.y);
  }
  if (hits.empty()) {
    if (!consumed.empty()) {
      std::sort(consumed.begin(), consumed.end(), [](std::size_t a, std::size_t b) {
        return a > b;
      });
      for (const std::size_t idx : consumed) {
        if (idx >= garbage_list_.size()) {
          continue;
        }
        garbage_list_.erase(garbage_list_.begin() + static_cast<std::ptrdiff_t>(idx));
      }
    }
    return false;
  }

  std::vector<std::size_t> ray_ids;
  for (const Hit & hit : hits) {
    if (std::find(ray_ids.begin(), ray_ids.end(), hit.ray_idx) == ray_ids.end()) {
      ray_ids.push_back(hit.ray_idx);
    }
  }
  std::sort(
    ray_ids.begin(), ray_ids.end(),
    [&](std::size_t a, std::size_t b) {
      std::size_t ia = goals->size();
      std::size_t ib = goals->size();
      findUnindexedSentinelIndex(
        *goals, open_garbage_rays_[a].gx, open_garbage_rays_[a].gy, &ia);
      findUnindexedSentinelIndex(
        *goals, open_garbage_rays_[b].gx, open_garbage_rays_[b].gy, &ib);
      return ia > ib;
    });

  bool changed = false;
  for (const std::size_t ray_idx : ray_ids) {
    if (ray_idx >= open_garbage_rays_.size()) {
      continue;
    }
    const double anchor_x = open_garbage_rays_[ray_idx].gx;
    const double anchor_y = open_garbage_rays_[ray_idx].gy;
    const double yaw = open_garbage_rays_[ray_idx].yaw;
    const bool had_e = open_garbage_rays_[ray_idx].has_e;
    const double old_ex = open_garbage_rays_[ray_idx].ex;
    const double old_ey = open_garbage_rays_[ray_idx].ey;

    std::size_t g_idx = 0;
    const bool have_g = findUnindexedSentinelIndex(*goals, anchor_x, anchor_y, &g_idx);
    std::vector<Hit> mine;
    for (const Hit & hit : hits) {
      if (hit.ray_idx == ray_idx &&
        std::find(consumed.begin(), consumed.end(), hit.list_idx) == consumed.end())
      {
        mine.push_back(hit);
      }
    }
    std::sort(mine.begin(), mine.end(), [](const Hit & a, const Hit & b) {
      return a.proj < b.proj;
    });

    std::size_t e_idx = goals->size();
    bool have_e_idx = false;
    if (had_e) {
      std::size_t found = 0;
      if (findUnindexedSentinelIndex(*goals, old_ex, old_ey, &found) &&
        (!have_g || found > g_idx))
      {
        e_idx = found;
        have_e_idx = true;
      }
    }
    if (!have_g && !have_e_idx) {
      continue;
    }
    std::size_t run_begin = have_g ? g_idx : e_idx;
    while (run_begin > 0) {
      const auto & prev_goal = (*goals)[run_begin - 1];
      double prev_proj = 0.0;
      if (!isUnindexedSentinelPoseZ(prev_goal) ||
        !onOpenRayAxis(
          anchor_x, anchor_y, yaw,
          prev_goal.pose.position.x, prev_goal.pose.position.y, &prev_proj))
      {
        break;
      }
      --run_begin;
    }
    const std::size_t run_end = have_e_idx ? (e_idx + 1) : (g_idx + 1);
    bool foreign = false;
    struct ChainPt
    {
      double proj{0.0};
      double x{0.0};
      double y{0.0};
      bool is_new{false};
      std::size_t list_idx{0};
      geometry_msgs::msg::PoseStamped pose;
    };
    std::vector<ChainPt> chain;
    for (std::size_t i = run_begin; i < run_end && i < goals->size(); ++i) {
      if (have_e_idx && i == e_idx) {
        continue;
      }
      const auto & goal = (*goals)[i];
      double goal_proj = 0.0;
      if (!isUnindexedSentinelPoseZ(goal) ||
        !onOpenRayAxis(
          anchor_x, anchor_y, yaw,
          goal.pose.position.x, goal.pose.position.y, &goal_proj))
      {
        foreign = true;
        break;
      }
      ChainPt pt;
      pt.proj = goal_proj;
      pt.x = goal.pose.position.x;
      pt.y = goal.pose.position.y;
      pt.pose = goal;
      chain.push_back(pt);
    }
    if (foreign) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: open ray skip G=(%.2f, %.2f), 链中间有不属于这条线的点",
        anchor_x, anchor_y);
      continue;
    }
    for (const Hit & hit : mine) {
      ChainPt pt;
      pt.proj = hit.proj;
      pt.x = garbage_list_[hit.list_idx].pose.pose.position.x;
      pt.y = garbage_list_[hit.list_idx].pose.pose.position.y;
      pt.is_new = true;
      pt.list_idx = hit.list_idx;
      chain.push_back(pt);
    }
    std::sort(chain.begin(), chain.end(), [](const ChainPt & a, const ChainPt & b) {
      return a.proj < b.proj;
    });

    std::vector<ChainPt> kept;
    for (const ChainPt & pt : chain) {
      if (kept.empty()) {
        kept.push_back(pt);
        continue;
      }
      std::string reason;
      if (isFootprintSweepClear(kept.back().x, kept.back().y, pt.x, pt.y, &reason)) {
        kept.push_back(pt);
        continue;
      }
      if (pt.is_new) {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: open ray skip (%.2f, %.2f), footprint 不过 (%s)",
          pt.x, pt.y, reason.c_str());
        continue;
      }
      while (!kept.empty() && kept.back().is_new &&
        !isFootprintSweepClear(kept.back().x, kept.back().y, pt.x, pt.y, &reason))
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "InsertGarbagePose: open ray skip (%.2f, %.2f), footprint 不过 (%s)",
          kept.back().x, kept.back().y, reason.c_str());
        kept.pop_back();
      }
      kept.push_back(pt);
    }
    bool any_new = false;
    for (const ChainPt & pt : kept) {
      if (pt.is_new) {
        any_new = true;
        break;
      }
    }
    if (!any_new) {
      continue;
    }

    const ChainPt & far = kept.back();
    const double far_x = far.x;
    const double far_y = far.y;
    const double ex = far_x + garbage_extend_m_ * std::cos(yaw);
    const double ey = far_y + garbage_extend_m_ * std::sin(yaw);
    std::string extend_reason;
    const bool add_e = had_e &&
      isFootprintSweepClear(far_x, far_y, ex, ey, &extend_reason);
    if (had_e && !add_e) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: open ray skip E (%.2f, %.2f), 通不过: %s",
        ex, ey, extend_reason.c_str());
    }

    std::string frame_id = global_frame_;
    const std::size_t frame_src = have_g ? g_idx : e_idx;
    if (frame_src < goals->size() && !(*goals)[frame_src].header.frame_id.empty()) {
      frame_id = (*goals)[frame_src].header.frame_id;
    }
    const auto orient = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    Goals rebuilt;
    rebuilt.reserve(goals->size() + kept.size() + 1);
    for (std::size_t i = 0; i < run_begin && i < goals->size(); ++i) {
      rebuilt.push_back((*goals)[i]);
    }
    for (const ChainPt & pt : kept) {
      geometry_msgs::msg::PoseStamped pose;
      if (pt.is_new) {
        pose.header.frame_id = frame_id;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = kGarbageSentinelPoseZ;
      } else {
        pose = pt.pose;
      }
      pose.pose.orientation = orient;
      rebuilt.push_back(pose);
    }
    for (std::size_t i = run_end; i < goals->size(); ++i) {
      rebuilt.push_back((*goals)[i]);
    }
    const bool place_e = had_e && add_e;
    if (place_e) {
      geometry_msgs::msg::PoseStamped extend_pose;
      extend_pose.header.frame_id = frame_id;
      extend_pose.pose.position.x = ex;
      extend_pose.pose.position.y = ey;
      extend_pose.pose.position.z = kGarbageSentinelPoseZ;
      extend_pose.pose.orientation = orient;
      const std::size_t insert_at = run_begin + kept.size();
      if (insert_at <= rebuilt.size()) {
        rebuilt.insert(rebuilt.begin() + static_cast<std::ptrdiff_t>(insert_at), extend_pose);
      } else {
        rebuilt.push_back(extend_pose);
      }
    }
    *goals = std::move(rebuilt);

    if (had_e) {
      forgetProtectedGarbageXy(old_ex, old_ey);
      for (auto & track : viz_tracks_) {
        if (!track.has_e) {
          continue;
        }
        if (squaredDistanceXY(track.ex, track.ey, old_ex, old_ey) >=
          kSentinelIdentityMatchM * kSentinelIdentityMatchM)
        {
          continue;
        }
        visualization_msgs::msg::MarkerArray clear_e;
        const int base = track.pile_num * 10;
        appendDeleteMarker(clear_e, "extend", base);
        appendDeleteMarker(clear_e, "extend", base + 1);
        appendDeleteMarker(clear_e, "extend", base + 2);
        publishMarkers(anchor_point_pub_, clear_e);
        track.has_e = false;
      }
    }

    open_garbage_rays_[ray_idx].yaw = yaw;
    open_garbage_rays_[ray_idx].has_e = place_e;
    open_garbage_rays_[ray_idx].ex = ex;
    open_garbage_rays_[ray_idx].ey = ey;
    // 新链头插到最前面：这条链的 from 改成新链头前面那个点
    double chain_fx = open_garbage_rays_[ray_idx].fx;
    double chain_fy = open_garbage_rays_[ray_idx].fy;
    if (!kept.empty() && kept.front().is_new) {
      const double head_x = kept.front().x;
      const double head_y = kept.front().y;
      double prev_x = robot_x;
      double prev_y = robot_y;
      if (run_begin > 0) {
        const auto & prev_goal = (*goals)[run_begin - 1];
        prev_x = prev_goal.pose.position.x;
        prev_y = prev_goal.pose.position.y;
      }
      const double back_m = std::max(garbage_extend_m_, kMinExtendFromDistM * 2.0);
      if (std::hypot(head_x - prev_x, head_y - prev_y) < kMinExtendFromDistM) {
        chain_fx = head_x - back_m * std::cos(yaw);
        chain_fy = head_y - back_m * std::sin(yaw);
      } else {
        chain_fx = prev_x;
        chain_fy = prev_y;
      }
      for (const ChainPt & old_pt : kept) {
        if (old_pt.is_new) {
          continue;
        }
        updateOpenRayFrom(old_pt.x, old_pt.y, chain_fx, chain_fy);
      }
      open_garbage_rays_[ray_idx].fx = chain_fx;
      open_garbage_rays_[ray_idx].fy = chain_fy;
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: open ray from 更新为 (%.2f, %.2f), 新链头=(%.2f, %.2f)",
        chain_fx, chain_fy, head_x, head_y);
    }
    for (const ChainPt & pt : kept) {
      if (!pt.is_new) {
        continue;
      }
      const bool tail = place_e &&
        squaredDistanceXY(pt.x, pt.y, far_x, far_y) <
        kSentinelIdentityMatchM * kSentinelIdentityMatchM;
      rememberOpenGarbageRay(pt.x, pt.y, yaw, tail, ex, ey, chain_fx, chain_fy);
      addProtectedGarbageXy(pt.x, pt.y);
      if (clip_refs != nullptr) {
        clip_refs->emplace_back(pt.x, pt.y);
      }
      InsertInfo info;
      info.garbage = garbage_list_[pt.list_idx];
      info.garbage.pose.header.frame_id = frame_id;
      info.robot_pose = robot_pose;
      info.path_yaw = yaw;
      info.dist_label = next_g_num_++;
      info.extend_inserted = tail;
      info.extend_x = ex;
      info.extend_y = ey;
      publishVisualization(info);
      consumed.push_back(pt.list_idx);
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: open ray chain (%.2f, %.2f) proj=%.2f onto G=(%.2f, %.2f) "
        "yaw=%.3f E=%d (%.2f, %.2f)",
        pt.x, pt.y, pt.proj, anchor_x, anchor_y, yaw, tail ? 1 : 0, ex, ey);
    }
    if (place_e) {
      addProtectedGarbageXy(ex, ey);
      if (clip_refs != nullptr) {
        clip_refs->emplace_back(ex, ey);
      }
      recordSweepArrive(ex, ey, yaw);
    } else if (had_e) {
      recordSweepArrive(far_x, far_y, yaw);
    }
    changed = true;
  }

  if (!consumed.empty()) {
    std::sort(consumed.begin(), consumed.end(), [](std::size_t a, std::size_t b) {
      return a > b;
    });
    for (const std::size_t idx : consumed) {
      if (idx >= garbage_list_.size()) {
        continue;
      }
      garbage_list_.erase(garbage_list_.begin() + static_cast<std::ptrdiff_t>(idx));
    }
  }
  return changed;
}

}  // namespace nav2_behavior_tree
