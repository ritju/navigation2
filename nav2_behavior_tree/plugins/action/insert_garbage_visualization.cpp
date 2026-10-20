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

// RViz 可视化

// ========================================================================
// J. 可视化
// ========================================================================

namespace
{

visualization_msgs::msg::Marker makeMarker(
  const std::string & frame, const rclcpp::Time & stamp,
  const std::string & ns, int id, int type,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = rclcpp::Duration::from_seconds(0.0);
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  return m;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<geometry_msgs::msg::Point> footprintAt(
  const std::vector<std::pair<double, double>> & fp,
  double x, double y, double yaw, double z)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<geometry_msgs::msg::Point> pts;
  pts.reserve(fp.size());
  for (const auto & off : fp) {
    geometry_msgs::msg::Point p;
    p.x = x + off.first * c - off.second * s;
    p.y = y + off.first * s + off.second * c;
    p.z = z;
    pts.push_back(p);
  }
  return pts;
}

}  // namespace

void InsertGarbagePose::resetVisualizationState()
{
  viz_tracks_.clear();
  viz_have_fail_strip_ = false;
  viz_pile_count_ = 0;
  viz_footprint_fail_count_ = 0;
}
void InsertGarbagePose::clearVisualizationTopics()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = global_frame_;
  clear.header.stamp = node_->now();
  clear.ns = "";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  auto send = [&](const auto & pub) {
    if (pub) {
      pub->publish(arr);
    }
  };
  send(workspace_circle_pub_);
  send(anchor_point_pub_);
  send(footprint_check_pub_);
  send(garbage_pose_pub_);
}

bool InsertGarbagePose::skipVisualization()
{
  if (enable_visualization_) {
    viz_switch_cleared_ = false;
    return false;
  }
  if (!viz_switch_cleared_) {
    visDeleteRangeCircles();
    visDeleteGarbageExtend();
    visDeleteClipAnchors();
    clearVisualizationTopics();
    resetVisualizationState();
    viz_switch_cleared_ = true;
  }
  return true;
}

void InsertGarbagePose::publishMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const visualization_msgs::msg::MarkerArray & arr)
{
  if (!enable_visualization_ || !pub || arr.markers.empty()) {
    return;
  }
  pub->publish(arr);
}

// 新任务 / 工作圈取消：清空四个话题上的全部 Marker
void InsertGarbagePose::clearMissionVisualization()
{
  visDeleteRangeCircles();
  visDeleteGarbageExtend();
  visDeleteClipAnchors();
  resetVisualizationState();
  clearVisualizationTopics();
  viz_switch_cleared_ = !enable_visualization_;
}

void InsertGarbagePose::appendDeleteMarker(
  visualization_msgs::msg::MarkerArray & arr, const std::string & ns, int id)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = node_->now();
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::msg::Marker::DELETE;
  arr.markers.push_back(m);
}

void InsertGarbagePose::appendFootprintStripMarkers(
  visualization_msgs::msg::MarkerArray & arr,
  const std::string & ns, int id_base,
  double x0, double y0, double x1, double y1,
  float r, float g, float b, float a_line, float a_fill)
{
  std::vector<std::pair<double, double>> fp;
  if (!getRobotFootprintInBase(fp) || fp.size() < 3) {
    return;
  }

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double yaw = (len < 1e-9) ? 0.0 : std::atan2(dy, dx);
  double step = 0.5;
  {
    std::lock_guard<std::mutex> lock(footprint_mutex_);
    step = std::max(0.05, cached_robot_length_m_);
  }

  double half_w = 0.05;
  for (const auto & off : fp) {
    half_w = std::max(half_w, std::fabs(off.second));
  }

  const rclcpp::Time stamp = node_->now();

  if (len >= 1e-9) {
    const double nx = -dy / len;
    const double ny = dx / len;
    auto corner = [&](double x, double y, double side) {
      geometry_msgs::msg::Point p;
      p.x = x + side * nx * half_w;
      p.y = y + side * ny * half_w;
      p.z = 0.03;
      return p;
    };
    const auto sl = corner(x0, y0, 1.0);
    const auto sr = corner(x0, y0, -1.0);
    const auto el = corner(x1, y1, 1.0);
    const auto er = corner(x1, y1, -1.0);
    auto fill = makeMarker(
      global_frame_, stamp, ns, id_base + 1,
      visualization_msgs::msg::Marker::TRIANGLE_LIST, r, g, b, a_fill);
    fill.scale.x = 1.0;
    fill.scale.y = 1.0;
    fill.scale.z = 1.0;
    fill.points = {sl, sr, el, sr, er, el};
    arr.markers.push_back(fill);
  }

  auto outline = makeMarker(
    global_frame_, stamp, ns, id_base,
    visualization_msgs::msg::Marker::LINE_LIST, r, g, b, a_line);
  outline.scale.x = 0.025;
  auto addPose = [&](double x, double y) {
    const std::vector<geometry_msgs::msg::Point> pts = footprintAt(fp, x, y, yaw, 0.05);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      outline.points.push_back(pts[i]);
      outline.points.push_back(pts[(i + 1) % pts.size()]);
    }
  };
  addPose(x0, y0);
  if (len >= 1e-9) {
    const double ux = dx / len;
    const double uy = dy / len;
    for (double s_pos = step; s_pos < len - 1e-9; s_pos += step) {
      addPose(x0 + ux * s_pos, y0 + uy * s_pos);
    }
    addPose(x1, y1);
  }
  if (!outline.points.empty()) {
    arr.markers.push_back(outline);
  }
}

void InsertGarbagePose::publishFailedSweepVisualization(
  double rx, double ry, double gx, double gy)
{
  if (skipVisualization()) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  appendFootprintStripMarkers(
    arr, "footprint_strip_fail", 0, rx, ry, gx, gy,
    0.95f, 0.20f, 0.08f, 0.95f, 0.28f);

  auto ball = makeMarker(
    global_frame_, node_->now(), "footprint_strip_fail", 2,
    visualization_msgs::msg::Marker::SPHERE, 0.95f, 0.10f, 0.10f, 1.0f);
  ball.pose.position.x = gx;
  ball.pose.position.y = gy;
  ball.pose.position.z = 0.12;
  ball.scale.x = ball.scale.y = ball.scale.z = 0.22;
  arr.markers.push_back(ball);

  viz_have_fail_strip_ = true;
  publishMarkers(footprint_check_pub_, arr);
}

void InsertGarbagePose::deletePileVisualization(int pile_num)
{
  if (!enable_visualization_ || pile_num <= 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray garbage_arr;
  visualization_msgs::msg::MarkerArray anchor_arr;
  visualization_msgs::msg::MarkerArray footprint_arr;
  const int base = pile_num * 10;
  appendDeleteMarker(garbage_arr, "garbage", base);
  appendDeleteMarker(garbage_arr, "garbage", base + 1);
  appendDeleteMarker(garbage_arr, "obstacle_p", base);
  appendDeleteMarker(garbage_arr, "obstacle_p", base + 1);
  appendDeleteMarker(anchor_arr, "ray_chain", pile_num);
  for (int i = 0; i < 8; ++i) {
    appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i);
    appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i + 100);
  }
  appendDeleteMarker(anchor_arr, "extend", base);
  appendDeleteMarker(anchor_arr, "extend", base + 1);
  appendDeleteMarker(anchor_arr, "extend", base + 2);
  appendDeleteMarker(footprint_arr, "footprint_strip", base);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 1);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 2);
  appendDeleteMarker(footprint_arr, "footprint_strip", base + 3);
  constexpr int kWallEdgeIdBase = 8000;
  constexpr int kWallEdgeIdSpan = 128;
  const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
  for (int i = 0; i < kWallEdgeIdSpan; ++i) {
    appendDeleteMarker(anchor_arr, "wall_edge_pts", wbase + i);
  }
  publishMarkers(garbage_pose_pub_, garbage_arr);
  publishMarkers(anchor_point_pub_, anchor_arr);
  publishMarkers(footprint_check_pub_, footprint_arr);
}

void InsertGarbagePose::pruneFinishedPileVisualization(const Goals & goals)
{
  if (skipVisualization() || viz_tracks_.empty()) {
    return;
  }
  std::vector<VizPileTrack> keep;
  keep.reserve(viz_tracks_.size());
  for (const auto & t : viz_tracks_) {
    const bool g_in = findUnindexedSentinelIndex(goals, t.gx, t.gy, nullptr);
    const bool e_in = t.has_e &&
      findUnindexedSentinelIndex(goals, t.ex, t.ey, nullptr);
    if (g_in || e_in) {
      keep.push_back(t);
      continue;
    }
    deletePileVisualization(t.pile_num);
  }
  viz_tracks_ = std::move(keep);
}

void InsertGarbagePose::visPublishClipAnchors(
  const Goals & goals, double robot_x, double robot_y)
{
  if (skipVisualization()) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();
  auto publishAnchor = [&](
    const char * ns, bool have, double x, double y, const char * label,
    float r, float g, float b)
  {
    if (!have) {
      appendDeleteMarker(arr, ns, 0);
      appendDeleteMarker(arr, ns, 1);
      return;
    }
    auto ball = makeMarker(
      global_frame_, stamp, ns, 0,
      visualization_msgs::msg::Marker::SPHERE, r, g, b, 1.0f);
    ball.pose.position.x = x;
    ball.pose.position.y = y;
    ball.pose.position.z = 0.14;
    ball.scale.x = ball.scale.y = ball.scale.z = 0.18;
    arr.markers.push_back(ball);

    auto text = makeMarker(
      global_frame_, stamp, ns, 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, r, g, b, 1.0f);
    text.pose.position.x = x;
    text.pose.position.y = y;
    text.pose.position.z = 0.42;
    text.scale.z = 0.22;
    text.text = label;
    arr.markers.push_back(text);
  };

  const std::size_t head = ordinaryQueueHead(goals);
  bool have_h = head < goals.size();
  double hx = 0.0;
  double hy = 0.0;
  if (have_h) {
    hx = goals[head].pose.position.x;
    hy = goals[head].pose.position.y;
  }

  bool have_c = false;
  double cx = 0.0;
  double cy = 0.0;
  if (have_h) {
    if (isRememberedCorner(hx, hy) &&
      !robotEnteredNextSide(goals, head, robot_x, robot_y))
    {
      have_c = true;
      cx = hx;
      cy = hy;
    } else {
      std::size_t range_end = head;
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
      for (std::size_t i = head; i <= range_end && i < goals.size(); ++i) {
        if (isProtectedGarbageXy(goals[i].pose.position.x, goals[i].pose.position.y)) {
          continue;
        }
        if (!isGoalNotCorner(goals, i, robot_x, robot_y)) {
          have_c = true;
          cx = goals[i].pose.position.x;
          cy = goals[i].pose.position.y;
          break;
        }
      }
    }
  }

  publishAnchor("clip_head", have_h, hx, hy, "H", 0.55f, 0.95f, 0.50f);
  publishAnchor("clip_corner", have_c, cx, cy, "C", 0.02f, 0.40f, 0.10f);
  publishMarkers(anchor_point_pub_, arr);
}

void InsertGarbagePose::visDeleteClipAnchors()
{
  if (!anchor_point_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  appendDeleteMarker(arr, "clip_head", 0);
  appendDeleteMarker(arr, "clip_head", 1);
  appendDeleteMarker(arr, "clip_corner", 0);
  appendDeleteMarker(arr, "clip_corner", 1);
  anchor_point_pub_->publish(arr);
}
// footprint 检查不通过时，把当时检查用的 footprint 框画在检查位姿上（空心红框）
void InsertGarbagePose::publishFootprintCheckBox(double x, double y, double yaw)
{
  if (skipVisualization()) {
    return;
  }
  std::vector<std::pair<double, double>> local_xy;
  if (!getRobotFootprintInBase(local_xy) || local_xy.size() < 3) {
    return;
  }

  auto box = makeMarker(
    global_frame_, node_->now(), "footprint_check_fail",
    static_cast<int>(viz_footprint_fail_count_),
    visualization_msgs::msg::Marker::LINE_STRIP, 0.95f, 0.10f, 0.10f, 1.0f);
  box.scale.x = 0.05;
  box.points = footprintAt(local_xy, x, y, yaw, 0.05);
  if (!box.points.empty()) {
    box.points.push_back(box.points.front());   // 闭合矩形
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(box);
  publishMarkers(footprint_check_pub_, arr);
  ++viz_footprint_fail_count_;
}

void InsertGarbagePose::visPublishRangeCircles(double robot_x, double robot_y)
{
  if (skipVisualization()) {
    return;
  }

  auto make_circle = [this](
    const std::string & ns, double cx, double cy, double radius,
    float r, float g, float b, float a, double width)
  {
    auto m = makeMarker(
      global_frame_, node_->now(), ns, 0,
      visualization_msgs::msg::Marker::LINE_STRIP, r, g, b, a);
    m.scale.x = width;
    constexpr int n = 72;
    m.points.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      m.points.push_back(makePoint(
        cx + radius * std::cos(ang), cy + radius * std::sin(ang), 0.05));
    }
    return m;
  };

  visualization_msgs::msg::MarkerArray circles;
  circles.markers.push_back(
    make_circle(
      "detect_range", robot_x, robot_y, max_garbage_robot_dist_m_,
      0.55f, 0.95f, 0.50f, 0.90f, 0.05));
  if (hasGoalsZpoints(receiveGoals())) {
    circles.markers.push_back(
      make_circle(
        "work_circle", work_circle_x_, work_circle_y_, work_circle_radius_m_,
        0.02f, 0.40f, 0.10f, 0.95f, 0.08));
  } else {
    appendDeleteMarker(circles, "work_circle", 0);
  }
  publishMarkers(workspace_circle_pub_, circles);
}

void InsertGarbagePose::visDeleteRangeCircles()
{
  if (!workspace_circle_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray circles;
  appendDeleteMarker(circles, "detect_range", 0);
  appendDeleteMarker(circles, "work_circle", 0);
  workspace_circle_pub_->publish(circles);
}

// 贴墙最近障碍：红块 + P{n}，发到 insert_anchor_point。每拍刷新。
void InsertGarbagePose::visPublishGarbageExtend()
{
  if (skipVisualization()) {
    return;
  }

  visualization_msgs::msg::MarkerArray obstacles;
  const double cell = viz_obstacle_cell_m_ * 1.4;
  constexpr int kObstacleTextIdBase = 1000;
  for (std::size_t i = 0; i < viz_obstacle_pixels_.size(); ++i) {
    const auto & obs = viz_obstacle_pixels_[i];
    const rclcpp::Time stamp = node_->now();
    auto box = makeMarker(
      global_frame_, stamp, "nearest_obstacle", static_cast<int>(i),
      visualization_msgs::msg::Marker::CUBE, 0.95f, 0.12f, 0.10f, 0.95f);
    box.pose.position.x = obs.x;
    box.pose.position.y = obs.y;
    box.pose.position.z = 0.08;
    box.scale.x = cell;
    box.scale.y = cell;
    box.scale.z = 0.04;
    obstacles.markers.push_back(box);

    auto text = makeMarker(
      global_frame_, stamp, "nearest_obstacle",
      kObstacleTextIdBase + static_cast<int>(i),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.95f, 0.12f, 0.10f, 1.0f);
    text.pose.position.x = obs.x;
    text.pose.position.y = obs.y;
    text.pose.position.z = 0.38;
    text.scale.z = 0.22;
    {
      std::ostringstream oss;
      const int n = (obs.pile_num > 0) ? obs.pile_num : static_cast<int>(i + 1);
      oss << "P" << n;
      text.text = oss.str();
    }
    obstacles.markers.push_back(text);
  }
  for (std::size_t i = viz_obstacle_pixels_.size(); i < viz_obstacle_marker_count_; ++i) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = global_frame_;
    del.header.stamp = node_->now();
    del.ns = "nearest_obstacle";
    del.id = static_cast<int>(i);
    del.action = visualization_msgs::msg::Marker::DELETE;
    obstacles.markers.push_back(del);

    visualization_msgs::msg::Marker del_text = del;
    del_text.id = kObstacleTextIdBase + static_cast<int>(i);
    obstacles.markers.push_back(del_text);
  }
  viz_obstacle_marker_count_ = viz_obstacle_pixels_.size();
  publishMarkers(anchor_point_pub_, obstacles);
}
// 已经进入链式、穷举或删点的结果：G、E、虚线、垃圾之间的红线、障碍 P、贴墙 D。
void InsertGarbagePose::visPublishGarbageExtend(const InsertInfo & info)
{
  if (skipVisualization()) {
    return;
  }

  visualization_msgs::msg::MarkerArray garbage_arr;
  visualization_msgs::msg::MarkerArray anchor_arr;
  const rclcpp::Time stamp = node_->now();

  const int pile_idx = viz_pile_count_;
  const int pile_num = (info.dist_label > 0) ? info.dist_label : (pile_idx + 1);

  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;
  const int base = pile_num * 10;
    constexpr double kGarbageDotZM = 0.12;
    constexpr double kGarbageDotSizeM = 0.22;
    constexpr double kDashLenM = 0.12;
    constexpr double kGapLenM = 0.08;
    constexpr double kDashLineWidthM = 0.030;
    constexpr double kLabelScaleM = 0.48;
    constexpr double kLabelGapM = 0.10;
    constexpr float kGarbageR = 0.95f;
    constexpr float kGarbageG = 0.10f;
    constexpr float kGarbageB = 0.10f;
    constexpr float kExtendR = 0.15f;
    constexpr float kExtendG = 0.40f;
    constexpr float kExtendB = 0.95f;
    // 字在球的地图 -Y 侧，俯视时在球正下方；相对方向固定，不跟清扫朝向转
    auto appendCaptionBelow = [&](
      visualization_msgs::msg::MarkerArray & arr,
      const std::string & ns, int id,
      double x, double y,
      float r, float g, float b,
      const std::string & text)
    {
      auto label = makeMarker(
        global_frame_, stamp, ns, id,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, r, g, b, 1.0f);
      label.pose.position.x = x;
      label.pose.position.y =
        y - (kGarbageDotSizeM * 0.5 + kLabelGapM + kLabelScaleM * 0.5);
      label.pose.position.z = kGarbageDotZM;
      label.pose.orientation.w = 1.0;
      label.scale.z = kLabelScaleM;
      label.text = text;
      arr.markers.push_back(label);
    };

    auto makeSolidDot = [&](const std::string & ns, int id, double x, double y,
        float r, float g, float b, double size)
    {
      auto dot = makeMarker(
        global_frame_, stamp, ns, id, visualization_msgs::msg::Marker::SPHERE,
        r, g, b, 1.0f);
      dot.pose.position.x = x;
      dot.pose.position.y = y;
      dot.pose.position.z = kGarbageDotZM;
      dot.scale.x = size;
      dot.scale.y = size;
      dot.scale.z = size;
      return dot;
    };

    auto appendDashedLine = [&](visualization_msgs::msg::Marker & line,
        double x0, double y0, double x1, double y1, double z = 0.07)
    {
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double len = std::hypot(dx, dy);
      if (len < 1e-4) {
        return;
      }
      const double ux = dx / len;
      const double uy = dy / len;
      double s = 0.0;
      bool draw = true;
      while (s < len - 1e-6) {
        const double step = draw ? kDashLenM : kGapLenM;
        const double s_next = std::min(s + step, len);
        if (draw) {
          line.points.push_back(makePoint(x0 + ux * s, y0 + uy * s, z));
          line.points.push_back(makePoint(x0 + ux * s_next, y0 + uy * s_next, z));
        }
        s = s_next;
        draw = !draw;
      }
    };

    garbage_arr.markers.push_back(
      makeSolidDot("garbage", base, gx, gy, kGarbageR, kGarbageG, kGarbageB, kGarbageDotSizeM));
    appendCaptionBelow(
      garbage_arr, "garbage", base + 1, gx, gy,
      kGarbageR, kGarbageG, kGarbageB, "G" + std::to_string(pile_num));

    if (info.wall_edge_has_p) {
      constexpr float kPR = 0.95f;
      constexpr float kPG = 0.55f;
      constexpr float kPB = 0.05f;
      garbage_arr.markers.push_back(
        makeSolidDot(
          "obstacle_p", base, info.wall_edge_px, info.wall_edge_py,
          kPR, kPG, kPB, kGarbageDotSizeM));
      appendCaptionBelow(
        garbage_arr, "obstacle_p", base + 1,
        info.wall_edge_px, info.wall_edge_py,
        kPR, kPG, kPB, "P" + std::to_string(pile_num));
    }

    if (info.ray_chain_xy.size() >= 2) {
      auto chain_line = makeMarker(
        global_frame_, stamp, "ray_chain", pile_num,
        visualization_msgs::msg::Marker::LINE_STRIP, 0.95f, 0.10f, 0.10f, 0.95f);
      chain_line.scale.x = 0.04;
      for (const auto & p : info.ray_chain_xy) {
        chain_line.points.push_back(makePoint(p.first, p.second, 0.08));
      }
      anchor_arr.markers.push_back(chain_line);
      for (std::size_t i = 1; i < info.ray_chain_xy.size(); ++i) {
        const int cid = pile_num * 10 + static_cast<int>(i);
        garbage_arr.markers.push_back(
          makeSolidDot(
            "ray_chain_g", cid,
            info.ray_chain_xy[i].first, info.ray_chain_xy[i].second,
            kGarbageR, kGarbageG, kGarbageB, kGarbageDotSizeM));
        appendCaptionBelow(
          garbage_arr, "ray_chain_g", cid + 100,
          info.ray_chain_xy[i].first, info.ray_chain_xy[i].second,
          kGarbageR, kGarbageG, kGarbageB,
          "G" + std::to_string(pile_num + static_cast<int>(i)));
      }
    }

    if (info.extend_inserted) {
      const double ex = info.extend_x;
      const double ey = info.extend_y;
      double line_x = gx;
      double line_y = gy;
      if (info.ray_chain_xy.size() >= 2) {
        line_x = info.ray_chain_xy.back().first;
        line_y = info.ray_chain_xy.back().second;
      }

      auto ge_line = makeMarker(
        global_frame_, stamp, "extend", base + 2,
        visualization_msgs::msg::Marker::LINE_LIST, 0.15f, 0.40f, 0.95f, 0.90f);
      ge_line.scale.x = kDashLineWidthM;
      appendDashedLine(ge_line, line_x, line_y, ex, ey);
      if (!ge_line.points.empty()) {
        anchor_arr.markers.push_back(ge_line);
      }

      anchor_arr.markers.push_back(
        makeSolidDot("extend", base, ex, ey, kExtendR, kExtendG, kExtendB, kGarbageDotSizeM));
      appendCaptionBelow(
        anchor_arr, "extend", base + 1, ex, ey,
        kExtendR, kExtendG, kExtendB,
        "E" + std::to_string(
          pile_num + (info.ray_chain_xy.size() >= 2 ?
          static_cast<int>(info.ray_chain_xy.size()) - 1 : 0)));
    }

    if (info.wall_edge_inserted) {
      constexpr double kMidDotSizeM = 0.07;
      constexpr int kWallEdgeIdBase = 8000;
      constexpr int kWallEdgeIdSpan = 128;
      const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
      int wid = 0;

      if (info.wall_edge_has_d) {
        anchor_arr.markers.push_back(
          makeSolidDot(
            "wall_edge_pts", wbase + wid++, info.wall_edge_d_x, info.wall_edge_d_y,
            kExtendR, kExtendG, kExtendB, kGarbageDotSizeM));
        appendCaptionBelow(
          anchor_arr, "wall_edge_pts", wbase + wid++,
          info.wall_edge_d_x, info.wall_edge_d_y,
          kExtendR, kExtendG, kExtendB, "D" + std::to_string(pile_num));
      }

      for (const auto & p : info.wall_edge_chain_xy) {
        if (wid >= kWallEdgeIdSpan) {
          break;
        }
        if (squaredDistanceXY(p.first, p.second, info.wall_edge_d_x, info.wall_edge_d_y) < 0.01 ||
          squaredDistanceXY(p.first, p.second, gx, gy) < 0.01 ||
          (info.extend_inserted &&
          squaredDistanceXY(p.first, p.second, info.extend_x, info.extend_y) < 0.01))
        {
          continue;
        }
        anchor_arr.markers.push_back(
          makeSolidDot(
            "wall_edge_pts", wbase + wid++, p.first, p.second,
            kExtendR, kExtendG, kExtendB, kMidDotSizeM));
      }
    }

    VizPileTrack track;
    track.pile_num = pile_num;
    track.gx = gx;
    track.gy = gy;
    if (!info.ray_chain_xy.empty()) {
      track.gx = info.ray_chain_xy.back().first;
      track.gy = info.ray_chain_xy.back().second;
    }
    track.has_e = info.extend_inserted;
    track.ex = info.extend_x;
    track.ey = info.extend_y;
    viz_tracks_.push_back(track);

  publishMarkers(garbage_pose_pub_, garbage_arr);
  publishMarkers(anchor_point_pub_, anchor_arr);
  ++viz_pile_count_;
}

void InsertGarbagePose::publishVisualization(const InsertInfo & info)
{
  if (skipVisualization()) {
    return;
  }

  visualization_msgs::msg::MarkerArray footprint_arr;
  const int pile_num = (info.dist_label > 0) ? info.dist_label : viz_pile_count_;
  const int base = pile_num * 10;
  const double gx = info.garbage.pose.pose.position.x;
  const double gy = info.garbage.pose.pose.position.y;
  const double rx = info.robot_pose.pose.position.x;
  const double ry = info.robot_pose.pose.position.y;

  if (viz_have_fail_strip_) {
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 0);
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 1);
    appendDeleteMarker(footprint_arr, "footprint_strip_fail", 2);
    viz_have_fail_strip_ = false;
  }
  appendFootprintStripMarkers(
    footprint_arr, "footprint_strip", base, rx, ry, gx, gy,
    1.00f, 0.55f, 0.05f, 0.90f, 0.22f);

  if (info.extend_inserted) {
    double line_x = gx;
    double line_y = gy;
    if (info.ray_chain_xy.size() >= 2) {
      line_x = info.ray_chain_xy.back().first;
      line_y = info.ray_chain_xy.back().second;
    }
    appendFootprintStripMarkers(
      footprint_arr, "footprint_strip", base + 2, line_x, line_y,
      info.extend_x, info.extend_y,
      0.15f, 0.45f, 0.95f, 0.85f, 0.18f);
  }
  publishMarkers(footprint_check_pub_, footprint_arr);
}

void InsertGarbagePose::visDeleteGarbageExtend()
{
  visualization_msgs::msg::MarkerArray garbage_arr;
  visualization_msgs::msg::MarkerArray anchor_arr;
  auto append_pile = [&](int pile_num) {
    if (pile_num <= 0) {
      return;
    }
    const int base = pile_num * 10;
    appendDeleteMarker(garbage_arr, "garbage", base);
    appendDeleteMarker(garbage_arr, "garbage", base + 1);
    appendDeleteMarker(garbage_arr, "obstacle_p", base);
    appendDeleteMarker(garbage_arr, "obstacle_p", base + 1);
    appendDeleteMarker(anchor_arr, "ray_chain", pile_num);
    for (int i = 0; i < 8; ++i) {
      appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i);
      appendDeleteMarker(garbage_arr, "ray_chain_g", pile_num * 10 + i + 100);
    }
    appendDeleteMarker(anchor_arr, "extend", base);
    appendDeleteMarker(anchor_arr, "extend", base + 1);
    appendDeleteMarker(anchor_arr, "extend", base + 2);
    constexpr int kWallEdgeIdBase = 8000;
    constexpr int kWallEdgeIdSpan = 128;
    const int wbase = kWallEdgeIdBase + pile_num * kWallEdgeIdSpan;
    for (int i = 0; i < kWallEdgeIdSpan; ++i) {
      appendDeleteMarker(anchor_arr, "wall_edge_pts", wbase + i);
    }
  };
  for (const auto & track : viz_tracks_) {
    append_pile(track.pile_num);
  }
  constexpr int kObstacleTextIdBase = 1000;
  for (std::size_t i = 0; i < viz_obstacle_marker_count_; ++i) {
    appendDeleteMarker(anchor_arr, "nearest_obstacle", static_cast<int>(i));
    appendDeleteMarker(
      anchor_arr, "nearest_obstacle", kObstacleTextIdBase + static_cast<int>(i));
  }
  if (garbage_pose_pub_ && !garbage_arr.markers.empty()) {
    garbage_pose_pub_->publish(garbage_arr);
  }
  if (anchor_point_pub_ && !anchor_arr.markers.empty()) {
    anchor_point_pub_->publish(anchor_arr);
  }
}

}  // namespace nav2_behavior_tree
