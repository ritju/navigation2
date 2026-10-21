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

// 垃圾回调、禁扫区回调和后处理，产出 garbage_list_

void InsertGarbagePose::garbageDetectCallback(
  const capella_ros_msg::msg::GarbageDetect::SharedPtr msg)
{
  getBehaviorTreeParameters();
  if (single_pile_insert_) {
    Goals goals;
    if (getInput("input_goals", goals) && hasGoalsZpoints(goals)) {
      return;
    }
  }

  capella_ros_msg::msg::GarbageDetect item;
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!msg || !transformGarbageToMap(item = *msg) || !getRobotPose(robot_pose)) {
    if (msg) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: drop garbage, transform to map or robot pose failed, wait for next frame");
    }
    return;
  }
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  const double gx = item.pose.pose.position.x;
  const double gy = item.pose.pose.position.y;
  const double confirm_r2 = confirm_match_dist_m_ * confirm_match_dist_m_;

  // 时间窗内累计检测：凑够 confirm_match_num 帧且距离在 confirm_match_dist_m 内，取平均往下传
  const rclcpp::Time now = node_->now();
  for (auto it = tmp_list_.begin(); it != tmp_list_.end(); ) {
    const double age = (now - rclcpp::Time(it->pose.header.stamp)).seconds();
    if (age > confirm_sec_garbage_time_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 垃圾 (%.2f, %.2f) 超过 %.1f 秒没有累计确认，删除",
        it->pose.pose.position.x, it->pose.pose.position.y, confirm_sec_garbage_time_);
      it = tmp_list_.erase(it);
      continue;
    }
    ++it;
  }

  item.pose.header.stamp = now;
  if (tmp_list_.size() >= kMaxHistorySize) {
    tmp_list_.pop_front();
  }
  tmp_list_.push_back(item);
  // 取算术平均坐标
  double sum_x = 0.0;
  double sum_y = 0.0;
  std::size_t match_n = 0;
  for (const auto & cand : tmp_list_) {
    if (squaredDistanceXY(
        gx, gy, cand.pose.pose.position.x, cand.pose.pose.position.y) < confirm_r2)
    {
      sum_x += cand.pose.pose.position.x;
      sum_y += cand.pose.pose.position.y;
      ++match_n;
    }
  }
  if (static_cast<int>(match_n) < confirm_match_num_) {
    return;
  }

  item.pose.pose.position.x = sum_x / static_cast<double>(match_n);
  item.pose.pose.position.y = sum_y / static_cast<double>(match_n);
  for (auto it = tmp_list_.begin(); it != tmp_list_.end(); ) {
    if (squaredDistanceXY(
        gx, gy, it->pose.pose.position.x, it->pose.pose.position.y) < confirm_r2)
    {
      it = tmp_list_.erase(it);
    } else {
      ++it;
    }
  }
  if (isDuplicateGarbage(item, garbage_list_)) {
    return;
  }

  history_list_.push_back(std::move(item));  //塞进去，留着给后边做堆的
  while (history_list_.size() > kMaxHistorySize) {
    auto farthest_it = history_list_.begin();
    double farthest_d2 = -1.0;
    for (auto it = history_list_.begin(); it != history_list_.end(); ++it) {
      const double d2 = squaredDistanceXY(
        it->pose.pose.position.x, it->pose.pose.position.y, robot_x, robot_y);
      if (d2 > farthest_d2) {
        farthest_d2 = d2;
        farthest_it = it;
      }
    }
    history_list_.erase(farthest_it);
  }
}
// 特殊清扫/禁扫区域话题回调
void InsertGarbagePose::special_terrain_callback(
  const garage_utils_msgs::msg::Polygons::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(special_terrain_mutex_);
  special_terrain_polygons_ = msg->polygons;
  RCLCPP_INFO(
    node_->get_logger(),
    "InsertGarbagePose: received %zu special terrain polygon(s)",
    special_terrain_polygons_.size());
}

// 接收到垃圾后的后处理函数，返回处理后的 garbage_list
InsertGarbagePose::GarbageList InsertGarbagePose::postProcessHistory(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_copy = history_list_;

  if (history_list_copy.empty()) {
    return garbage_list_;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  auto sameDetect =     // id，stamp，xy 都相同则认为是同一条垃圾
    [](const capella_ros_msg::msg::GarbageDetect & a,
      const capella_ros_msg::msg::GarbageDetect & b) {
      return a.class_id == b.class_id &&
             a.pose.header.stamp == b.pose.header.stamp &&
             std::fabs(a.pose.pose.position.x - b.pose.pose.position.x) < 1e-6 &&
             std::fabs(a.pose.pose.position.y - b.pose.pose.position.y) < 1e-6;
    };

  // 从 history 里删掉已处理完的那一条
  auto eraseFromHistory =
    [this, &sameDetect](const capella_ros_msg::msg::GarbageDetect & original) {
      for (auto it = history_list_.begin(); it != history_list_.end(); ++it) {
        if (sameDetect(*it, original)) {
          history_list_.erase(it);
          return;
        }
      }
    };

  GarbageList candidates;
  std::vector<capella_ros_msg::msg::GarbageDetect> candidate_originals;
  candidates.reserve(history_list_copy.size());
  candidate_originals.reserve(history_list_copy.size());

  // 删掉一组已消化的原始检测
  auto erasePileMembers =
    [&](const std::vector<std::size_t> & member_idx) {
      for (const std::size_t i : member_idx) {
        if (i < candidate_originals.size()) {
          eraseFromHistory(candidate_originals[i]);
        }
      }
    };

  for (const auto & original : history_list_copy) {
    capella_ros_msg::msg::GarbageDetect garbage = original;

    // 转到 map，失败则留在 history
    if (!transformGarbageToMap(garbage)) {                  
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为转到map失败, 暂不处理",
        original.pose.pose.position.x, original.pose.pose.position.y);
      continue;
    }

    const double gx = garbage.pose.pose.position.x;
    const double gy = garbage.pose.pose.position.y;

    // 落在禁扫区则丢弃
    if (isPointInSpecialTerrain(gx, gy)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为在禁扫区, 丢弃", gx, gy);
      eraseFromHistory(original);
      continue;
    }

    // 离机器人太远：视为误识别，丢弃
    {
      const double dist_robot = std::sqrt(squaredDistanceXY(gx, gy, robot_x, robot_y));
      if (dist_robot > max_garbage_robot_dist_m_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为离机器人过远(%.2fm>%.2fm), 丢弃",
          gx, gy, dist_robot, max_garbage_robot_dist_m_);
        eraseFromHistory(original);
        continue;
      }
    }

    // GE 用 footprint 长条；enable_wall_edge_insert 时用 robot-G 连线无 lethal
    {
      std::string sweep_reason;
      const bool intake_clear = enable_wall_edge_insert_ ?
        isRobotGarbageSegmentClearOfLethal(robot_x, robot_y, gx, gy, &sweep_reason) :
        isFootprintSweepClear(robot_x, robot_y, gx, gy, &sweep_reason);
      if (!intake_clear) {
        if (enable_wall_edge_insert_) {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *(node_->get_clock()), 2000,
            "InsertGarbagePose: 垃圾=(%.2f, %.2f), 贴墙进表: 车到垃圾连线有障碍(%s), 丢弃",
            gx, gy, sweep_reason.c_str());
        } else {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *(node_->get_clock()), 2000,
            "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为车到垃圾 footprint 长条不通过(%s), 丢弃",
            gx, gy, sweep_reason.c_str());
          publishFailedSweepVisualization(robot_x, robot_y, gx, gy);
        }
        eraseFromHistory(original);
        continue;
      }
    }

    if (!has_work_circle_ && garbage_list_.empty() &&
      !hasGoalsZpoints(receiveGoals())) {
      has_work_circle_ = true;
      work_circle_x_ = robot_x;
      work_circle_y_ = robot_y;
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 生成工作圈 圆心=(%.2f, %.2f) 半径=%.2f m",
        work_circle_x_, work_circle_y_, work_circle_radius_m_);
    }
    if (has_work_circle_) {
      const double d_circle = std::sqrt(squaredDistanceXY(
          gx, gy, work_circle_x_, work_circle_y_));
      if (d_circle > work_circle_radius_m_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 2000,
          "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为在工作圈外(%.2fm>%.2fm), 丢弃",
          gx, gy, d_circle, work_circle_radius_m_);
        eraseFromHistory(original);
        continue;
      }
    }

    candidates.push_back(std::move(garbage));
    candidate_originals.push_back(original);
  }

  if (candidates.empty()) {
    return garbage_list_;
  }

  // 离车最近开堆；代表点为成员质心；到质心小于合堆半径才并入
  std::vector<std::vector<std::size_t>> groups;
  GarbageList merged_garbage_list = mergeGarbagePiles(
    candidates, robot_x, robot_y, garbage_merge_radius_m_, &groups);

  for (std::size_t gi = 0; gi < merged_garbage_list.size(); ++gi) {
    auto & seed = merged_garbage_list[gi];
    const std::vector<std::size_t> & members = groups[gi];
    const double sx = seed.pose.pose.position.x;
    const double sy = seed.pose.pose.position.y;

    // 已插入过的堆不再进候选，避免持续发布反复占队首
    if (isNearReachedGarbage(sx, sy)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为已到达/已插入过, 丢弃", sx, sy);
      erasePileMembers(members);
      continue;
    }

    if (isDuplicateGarbage(seed, garbage_list_)) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为与已规划堆重复, 丢弃", sx, sy);
      erasePileMembers(members);
      continue;
    }

    if (tryInsertPreferCloserToRobot(seed, robot_x, robot_y)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "InsertGarbagePose: 新发现一堆垃圾, 坐标=(%.2f, %.2f), 进入清扫规划", sx, sy);
      erasePileMembers(members);
    } else {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 2000,
        "InsertGarbagePose: 垃圾=(%.2f, %.2f), 因为队列已满且更远, 丢弃",
        sx, sy);
    }
  }

  return garbage_list_;
}
// 把单个垃圾从 base_link 转到 map
bool InsertGarbagePose::transformGarbageToMap(
  capella_ros_msg::msg::GarbageDetect & garbage) const
{
  if (!tf_) {
    return false;
  }
  geometry_msgs::msg::PoseStamped pose_in = garbage.pose;
  if (pose_in.header.frame_id.empty()) {
    pose_in.header.frame_id = robot_base_frame_;
  }
  // 已经在 map 下，不用再转
  if (pose_in.header.frame_id == global_frame_) {
    return true;
  }
  geometry_msgs::msg::PoseStamped pose_out;
  if (!nav2_util::transformPoseInTargetFrame(
      pose_in, pose_out, *tf_, global_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "InsertGarbagePose: failed to transform garbage pose from '%s' to '%s'",
      pose_in.header.frame_id.c_str(), global_frame_.c_str());
    return false;
  }

  // bbox 角点一并转到 map
  std::vector<geometry_msgs::msg::Point> corners_map;
  corners_map.reserve(garbage.bbox_corner_points.size());
  for (const auto & corner : garbage.bbox_corner_points) {
    geometry_msgs::msg::PointStamped pin;
    pin.header = pose_in.header;
    pin.point = corner;
    try {
      geometry_msgs::msg::PointStamped pout = tf_->transform(
        pin, global_frame_, tf2::durationFromSec(transform_tolerance_));
      corners_map.push_back(pout.point);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "InsertGarbagePose: failed to transform bbox corner: %s", ex.what());
      return false;
    }
  }

  garbage.pose = pose_out;
  garbage.bbox_corner_points = std::move(corners_map);
  return true;
}
// 判断点是否在禁扫区域内
bool InsertGarbagePose::isPointInSpecialTerrain(double x, double y) const
{
  std::lock_guard<std::mutex> lock(special_terrain_mutex_);
  if (special_terrain_polygons_.empty()) {
    return false;
  }

  for (const auto & polygon : special_terrain_polygons_) {
    if (isPointInPolygon(x, y, polygon)) {
      return true;
    }
  }
  return false;
}
// 判断 看垃圾点是否在禁行区里面的函数
bool InsertGarbagePose::isPointInPolygon(
  double x, double y, const geometry_msgs::msg::Polygon & polygon)
{
  const auto & pts = polygon.points;
  if (pts.size() < 3) {
    return false;
  }

  // 射线法
  bool inside = false;
  for (std::size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
    const double xi = static_cast<double>(pts[i].x);
    const double yi = static_cast<double>(pts[i].y);
    const double xj = static_cast<double>(pts[j].x);
    const double yj = static_cast<double>(pts[j].y);
    const bool intersect =
      ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}
// 离车最近开堆；代表点为成员质心；离质心最近且小于合堆半径才并入，最近的并不上则结束本堆
InsertGarbagePose::GarbageList InsertGarbagePose::mergeGarbagePiles(
  const GarbageList & candidates,
  double robot_x, double robot_y,
  double merge_radius_m,
  std::vector<std::vector<std::size_t>> * groups_out)
{
  GarbageList merged_garbage_list;
  if (groups_out != nullptr) {
    groups_out->clear();
  }
  if (candidates.empty()) {
    return merged_garbage_list;
  }

  const double radius2 = merge_radius_m * merge_radius_m;

  auto xyOf = [&candidates](std::size_t idx) { // 对应的下标中取出它的xy
      return std::make_pair(
        candidates[idx].pose.pose.position.x,
        candidates[idx].pose.pose.position.y);
    };

  // remaining：还没分堆的候选下标
  std::vector<std::size_t> remaining(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    remaining[i] = i;
  }

  while (!remaining.empty()) {
    // 在剩余点里找离机器人最近的，作为本堆种子
    std::size_t seed_pos = 0;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t p = 0; p < remaining.size(); ++p) {
      const auto g = xyOf(remaining[p]);
      const double d2 = squaredDistanceXY(g.first, g.second, robot_x, robot_y);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        seed_pos = p;
      }
    }
    const std::size_t seed_idx = remaining[seed_pos];
    std::vector<std::size_t> members{seed_idx};
    double sum_x = xyOf(seed_idx).first;
    double sum_y = xyOf(seed_idx).second;

    for (;;) {
      const double cx = sum_x / static_cast<double>(members.size());
      const double cy = sum_y / static_cast<double>(members.size());
      bool found = false;
      std::size_t best_idx = 0;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (const std::size_t idx : remaining) {
        if (std::find(members.begin(), members.end(), idx) != members.end()) {
          continue;
        }
        const auto q = xyOf(idx);
        const double d2 = squaredDistanceXY(q.first, q.second, cx, cy);
        if (d2 < best_d2) {
          best_d2 = d2;
          best_idx = idx;
          found = true;
        }
      }
      // 离当前质心最近的点都超半径，更远的不再试
      if (!found || best_d2 >= radius2) {
        break;
      }
      const auto q = xyOf(best_idx);
      members.push_back(best_idx);
      sum_x += q.first;
      sum_y += q.second;
    }

    auto pile = candidates[seed_idx];
    pile.pose.pose.position.x = sum_x / static_cast<double>(members.size());
    pile.pose.pose.position.y = sum_y / static_cast<double>(members.size());
    merged_garbage_list.push_back(std::move(pile));
    if (groups_out != nullptr) {
      groups_out->push_back(members);
    }

    // 已消化的成员从 remaining 移除
    std::vector<std::size_t> next;
    next.reserve(remaining.size());
    for (const std::size_t idx : remaining) {
      if (std::find(members.begin(), members.end(), idx) == members.end()) {
        next.push_back(idx);
      }
    }
    remaining = std::move(next);
  }

  return merged_garbage_list;
}
// 队列未满直接加；满了则用更近的新垃圾替换离机器人最远的
bool InsertGarbagePose::tryInsertPreferCloserToRobot(
  capella_ros_msg::msg::GarbageDetect garbage,
  double robot_x, double robot_y)
{
  if (garbage_list_.size() < kMaxGarbageSize) {
    garbage_list_.push_back(std::move(garbage));
    return true;
  }

  const double new_dist2 = squaredDistanceXY(
    garbage.pose.pose.position.x, garbage.pose.pose.position.y,
    robot_x, robot_y);

  // 找队列中离机器人最远的
  std::size_t farthest_idx = 0;
  double farthest_dist2 = -1.0;
  for (std::size_t i = 0; i < garbage_list_.size(); ++i) {
    const double d2 = squaredDistanceXY(
      garbage_list_[i].pose.pose.position.x,
      garbage_list_[i].pose.pose.position.y,
      robot_x, robot_y);
    if (d2 > farthest_dist2) {
      farthest_dist2 = d2;
      farthest_idx = i;
    }
  }
  // 新垃圾不比最远的更近 → 不加
  if (new_dist2 >= farthest_dist2) {
    return false;
  }
  garbage_list_.erase(garbage_list_.begin() + static_cast<std::ptrdiff_t>(farthest_idx));
  garbage_list_.push_back(std::move(garbage));
  return true;
}

bool InsertGarbagePose::isDuplicateGarbage(
  const capella_ros_msg::msg::GarbageDetect & garbage,
  const GarbageList & existing) const
{
  const double x = garbage.pose.pose.position.x;
  const double y = garbage.pose.pose.position.y;
  const double thresh2 = garbage_merge_radius_m_ * garbage_merge_radius_m_;

  for (const auto & pile : existing) {
    const double dx = x - pile.pose.pose.position.x;
    const double dy = y - pile.pose.pose.position.y;
    if (dx * dx + dy * dy < thresh2) {
      return true;
    }
  }
  return false;
}
// 新垃圾落在已处理点的合堆半径内就不再作为候选，防止重复插入
bool InsertGarbagePose::isNearReachedGarbage(double x, double y) const
{
  const double thresh2 = garbage_merge_radius_m_ * garbage_merge_radius_m_;
  for (const auto & reached : reached_garbage_xy_) {
    if (isOpenRayExtendPoint(reached.first, reached.second)) {
      continue;
    }
    if (squaredDistanceXY(x, y, reached.first, reached.second) < thresh2) {
      return true;
    }
  }
  return false;
}

}  // namespace nav2_behavior_tree
