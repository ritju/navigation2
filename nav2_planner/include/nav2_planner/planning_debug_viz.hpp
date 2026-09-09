// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef NAV2_PLANNER__PLANNING_DEBUG_VIZ_HPP_
#define NAV2_PLANNER__PLANNING_DEBUG_VIZ_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_planner
{

/**
 * @brief through-poses / FastPath / Hybrid 裁剪的 RViz 调试可视化。
 *
 * 话题均挂在 planner_server 节点下，前缀 planning_debug/。
 * 关闭参数 publish_planning_debug 后所有发布为空操作。
 *
 * planning_debug_keep_mode:
 * - session（默认）：一次 ComputePath* 内按 via 累加 Marker，仅下次规划开始时清空。
 *   publishMarkers 只写入缓存；规划结束 endSession 整包发出（含各段 Hybrid 异色路径）。
 * - current：只显示当前 via（立即发布，会一闪而过）
 */
class PlanningDebugViz
{
public:
  PlanningDebugViz();
  ~PlanningDebugViz() = default;

  void configure(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);
  void activate();
  void deactivate();
  void cleanup();

  void setEnabled(bool enabled) {enabled_ = enabled;}
  void setFootprintStride(int stride) {footprint_stride_ = stride;}
  void setKeepMode(const std::string & mode);
  bool enabled() const {return enabled_;}
  bool accumulateSession() const {return accumulate_;}

  /** 一次 ComputePath* 开始：清空上一轮 Marker。 */
  void beginSession();
  /** 规划结束再发一帧完整 MarkerArray，避免 depth=1 只留下最后一段。 */
  void endSession();
  /** session 模式：把已缓存的 Marker 整包发出。只在 endSession 调用，不要每 via 全量 flush。 */
  void flush();

  /** 当前 via 段上下文，供 TEXT 标注。 */
  void setSegmentContext(
    unsigned int via_index,
    std::size_t via_count,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal_original);

  void publishStart(const geometry_msgs::msg::PoseStamped & pose);
  void publishOriginalGoal(const geometry_msgs::msg::PoseStamped & pose);
  void publishSnappedGoal(
    const geometry_msgs::msg::PoseStamped & original,
    const geometry_msgs::msg::PoseStamped & snapped);
  void publishAdjustedGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & reason);
  void publishPushedGoal(
    const geometry_msgs::msg::PoseStamped & original,
    const geometry_msgs::msg::PoseStamped & pushed);
  void publishDroppedVia(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & reason);

  /** 碰撞时的 base_footprint 位姿；红球画在该位姿下的 254 格（用 dx/dy 偏置）。 */
  void publishCollision(
    const geometry_msgs::msg::PoseStamped & pose,
    double dx,
    double dy,
    double sample_d,
    double segment_length);

  /** 半宽走廊矩形（S→G，不加前后悬）。blocked 时用红框，通时用绿框。 */
  void publishCorridor(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double half_width,
    bool blocked,
    const geometry_msgs::msg::PoseStamped & hit_pose);

  void publishStraightCandidate(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    int iter,
    const std::string & kind);
  void publishStraightPath(const nav_msgs::msg::Path & path);
  void publishRotatedPath(const nav_msgs::msg::Path & path);
  void publishFootprintsAlongPath(
    const nav_msgs::msg::Path & path,
    const std::string & ns,
    int stride);

  void publishHybridRaw(const nav_msgs::msg::Path & path);
  void publishHybridClipped(const nav_msgs::msg::Path & path);
  void publishIntersections(const std::vector<geometry_msgs::msg::Point> & points);

private:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using PathPub = rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>;
  using MarkerPub = rclcpp_lifecycle::LifecyclePublisher<MarkerArray>;

  bool canPublish() const;
  int markerId(int local = 0) const;
  int markerId(int local, int extra) const;
  void clearPaths();
  std::string frame() const;
  builtin_interfaces::msg::Time now() const;
  void publishPath(const PathPub::SharedPtr & pub, const nav_msgs::msg::Path & path);
  void ingestMarkers(const MarkerArray & arr);
  void publishMarkers(const MarkerArray & arr);
  void publishSessionSnapshot();
  Marker deleteAll(const std::string & ns) const;
  Marker poseArrow(
    const std::string & ns, int id,
    const geometry_msgs::msg::PoseStamped & pose,
    const std_msgs::msg::ColorRGBA & color) const;
  Marker poseSphere(
    const std::string & ns, int id,
    const geometry_msgs::msg::Point & point,
    const std_msgs::msg::ColorRGBA & color,
    double scale) const;
  Marker poseText(
    const std::string & ns, int id,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & text,
    const std_msgs::msg::ColorRGBA & color) const;
  Marker footprintStrip(
    const std::string & ns, int id,
    const geometry_msgs::msg::PoseStamped & pose,
    const std_msgs::msg::ColorRGBA & color) const;
  Marker pathStrip(
    const std::string & ns, int id,
    const nav_msgs::msg::Path & path,
    const std_msgs::msg::ColorRGBA & color) const;
  /** 按 via 下标轮换色相，相邻段颜色尽量分开。 */
  static std_msgs::msg::ColorRGBA segmentColor(unsigned int index, float alpha = 0.95f);
  nav_msgs::msg::Path twoPointPath(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const;
  static std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a = 1.0f);

  rclcpp::Logger logger_{rclcpp::get_logger("PlanningDebugViz")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  PathPub::SharedPtr straight_path_pub_;
  PathPub::SharedPtr straight_iter_pub_;
  PathPub::SharedPtr rotated_path_pub_;
  PathPub::SharedPtr hybrid_raw_pub_;
  PathPub::SharedPtr hybrid_clipped_pub_;
  MarkerPub::SharedPtr marker_pub_;

  bool enabled_{true};
  bool activated_{false};
  bool accumulate_{true};
  int footprint_stride_{0};
  unsigned int via_index_{0};
  std::size_t via_count_{0};
  std::map<std::pair<std::string, int>, Marker> session_markers_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNING_DEBUG_VIZ_HPP_
