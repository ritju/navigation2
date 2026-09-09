// Copyright 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef NAV2_PLANNER__FAST_PATH_PLANNER_HPP_
#define NAV2_PLANNER__FAST_PATH_PLANNER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "garage_utils_msgs/msg/polygons.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_planner/planning_debug_viz.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_planner
{

/**
 * @brief planner_server 前置快路径的结果原因。
 *
 * 仅内部使用，不对外暴露给 BT / Action。
 * - StraightOk：直线段可用（前进或后退），path 有点，后续不必再调 A*
 * - NeedAstar：目标已修正（或本来就可走），但直线不可用，应把 snapped_goal 交给插件
 * - GoalUnreachable：原目标占用，邻域内也找不到可通行点，不要再跑 A*
 */
enum class FastPlanReason
{
  StraightOk = 0,
  NeedAstar = 1,
  GoalUnreachable = 2
};

/**
 * @brief 前置快路径一次调用的完整返回。
 *
 * path 仅在 reason == StraightOk 时有有效 poses。
 * snapped_goal 始终是后续规划必须使用的目标：原目标可走时等于 goal，
 * 被占用时为邻域搜索到的最近可通行点。StraightOk 时可能是缩短/转线后的 G'。
 * NeedAstar 时不含失败的缩短/转线，只保留占用 snap。
 */
struct FastPlanResult
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped snapped_goal;
  FastPlanReason reason{FastPlanReason::NeedAstar};
};

/** FastPath::compute 的开关。远段应关闭 stretch/rotate，只 snap + 一次走廊检查。 */
struct FastPlanOptions
{
  bool allow_straight{true};
  bool allow_reverse{false};
  bool allow_stretch{false};
  bool allow_rotate{false};
  /** NavigateToPose / through-poses 最后一段：snap 用完整 footprint；走廊中心线通过后补前悬。 */
  bool strict_goal_footprint{false};
  /** true：snap / NeedAstar 的 G 写成 S→G 来向；false：保留传入的 goal yaw。由 server 按段决定。 */
  bool rewrite_goal_yaw_to_approach{true};
};

/**
 * @brief planner_server 同进程前置模块：目标占用 snap + 可选直线捷径。
 *
 * 直线是否启用由 allow_straight 传入。窄通道或 /enable_backward 表示
 * 允许倒车（不是必须）：前进直线失败后再试后退直线。
 * 本类订阅上述话题，供 server 查询当前倒车/窄通道状态。
 */
class FastPathPlanner
{
public:
  using FootprintChecker =
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;

  FastPathPlanner();
  ~FastPathPlanner() = default;

  /**
   * @brief 绑定 costmap / footprint，读取参数并订阅倒车、窄通道话题。
   * @param node planner_server 生命周期节点
   * @param costmap_ros 全局代价地图
   * @param footprint_checker 与 server 共用的 footprint 碰撞检测器
   * @param planner_ids 已加载的规划插件名；用于 overlay GridBased.* 旧参数
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::shared_ptr<FootprintChecker> & footprint_checker,
    const std::vector<std::string> & planner_ids);

  /** @brief 释放订阅与 costmap 引用。 */
  void cleanup();

  /**
   * @brief 先 snap，再走廊直线；近段可缩短/转线。
   * 中间 via：中心线粗检 + 半宽走廊。最后一段 / NavigateToPose：中心线通过后补前悬（放过 S 后悬）。
   * 不修改 start 的 yaw。路径点 yaw 为来向（倒车时 +π）。
   */
  FastPlanResult compute(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const FastPlanOptions & options);

  /**
   * @brief 按起点更新窄通道 latch。
   *
   * 进入：base_link 落入任一窄通道多边形（宽松）。
   * 退出：footprint 全部顶点都在所有多边形外（严格）。
   * 中间态保持上一状态，避免边界抖动。
   */
  void updateNarrowPassageLatch(const geometry_msgs::msg::PoseStamped & start);

  /** @brief 是否已收到 /enable_backward 且当前为 true。 */
  bool isBackwardActive() const;

  /**
   * @brief 本次规划是否处于窄通道模式。
   *
   * latch 为 true，或 goal 落在窄通道多边形内，即为 true。
   */
  bool isNarrowActive(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  void setGoalOccupiedTolerance(double value) {_goal_occupied_tolerance = value;}
  void setGoalSearchResolution(double value) {_goal_search_resolution = value;}
  void setStraightCheckLengthRatio(double value) {_straight_check_length_ratio = value;}
  void setStraightPathResolution(double value) {_straight_path_resolution = value;}
  void setEnableLineStretch(bool value) {enable_line_stretch_ = value;}
  void setLineStretchMax(double value) {line_stretch_max_ = value;}
  void setLineStretchGoalWindow(double value) {line_stretch_goal_window_ = value;}
  void setLineStretchAllowExtend(bool value) {line_stretch_allow_extend_ = value;}
  void setEnableLineRotate(bool value) {enable_line_rotate_ = value;}
  void setLineRotateMaxIters(int value) {line_rotate_max_iters_ = value;}
  void setLineRotateGoalShiftTol(double value) {line_rotate_goal_shift_tol_ = value;}
  void setCorridorIntrusionTol(double value)
  {
    corridor_intrusion_tol_ = std::max(0.0, value);
  }

  /** @brief 绑定调试可视化（可为空）。 */
  void setDebugViz(const std::shared_ptr<PlanningDebugViz> & viz) {debug_viz_ = viz;}

private:
  /** /enable_backward 回调。 */
  void enableBackwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /** /narrow_passages 回调：更新多边形列表；空列表则清除 latch。 */
  void narrowPassagesCallback(const garage_utils_msgs::msg::Polygons::SharedPtr msg);

  /** 是否已收到非空窄通道多边形。 */
  bool narrowPolygonsAvailable() const;
  /** map 系一点是否落在任一窄通道多边形内（射线法）。 */
  bool isPointInNarrowPassage(double x, double y) const;
  /** footprint 全部顶点是否都在所有窄通道多边形外。 */
  bool isFootprintFullyOutsideNarrowPassages(
    const geometry_msgs::msg::PoseStamped & pose) const;

  /** 根据当前机器人 footprint 刷新前后端 x 范围。 */
  void updateFootprintExtents();
  /** 直线碰撞检查步长：max(地图分辨率, 车长 * ratio)。 */
  double straightCheckStep() const;
  /** 直线路径点间距：max(参数, 1mm)。 */
  double straightPathStep() const;

  struct CorridorHit
  {
    bool blocked{false};
    bool both_sides{false};
    /** 对侧连续非 254 长度 >= 车宽。仅 blocked 且单侧时有意义。 */
    bool clearance_ok{false};
    double s{0.0};
    double L{0.0};
    double dy{0.0};
    /** 半宽侵入：halfWidth - abs(dy)。 */
    double intrusion{0.0};
    /** 对侧连续非 254 长度 (m)。 */
    double clearance_len{0.0};
    /** 发生碰撞时的 base_footprint 位姿（沿 S→G 的 s），不是 254 格子中心。 */
    geometry_msgs::msg::PoseStamped pose;
  };

  double halfWidth() const;
  double robotWidth() const;
  double inscribedRadius() const;
  bool centerlinePrecheckReliable() const;
  bool isLethalWorld(double wx, double wy) const;
  /** 从 254 格沿法向反方向量连续非 254 长度；先跨过致死再计。255/253 可过。 */
  double measureOppositeFreeLength(
    double px, double py, double nx, double ny, double dy) const;
  bool isCrossSectionFree(double x, double y, double yaw) const;
  bool footprintHitsLethal(
    double x, double y, double yaw,
    double path_s,
    bool skip_behind_start,
    double & hit_fx,
    double & hit_fy,
    double & hit_wx,
    double & hit_wy) const;
  bool findCenterlineTrigger(
    double ax, double ay, double ux, double uy, double L, double & s_out) const;
  CorridorHit scanHalfWidthBand(
    const geometry_msgs::msg::PoseStamped & start,
    double ax, double ay, double ux, double uy, double yaw, double L,
    double s_begin, double s_end) const;
  CorridorHit checkBandCorridor(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double front_overhang) const;
  CorridorHit checkCorridor(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool use_footprint_corridor) const;
  CorridorHit checkHalfWidthCorridor(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  CorridorHit checkFootprintCorridor(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool snapOccupiedGoal(
    const geometry_msgs::msg::PoseStamped & start,
    geometry_msgs::msg::PoseStamped & goal,
    bool use_footprint,
    bool rewrite_yaw_to_approach);
  nav_msgs::msg::Path buildStraightPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double heading) const;
  bool tryStretchGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & original_goal,
    geometry_msgs::msg::PoseStamped & goal,
    const CorridorHit & hit);
  bool tryRotateGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & original_goal,
    geometry_msgs::msg::PoseStamped & goal,
    CorridorHit & hit,
    bool use_footprint_corridor);

  /** 从 original 沿 original→edge 方向前进距离 d 的点。 */
  bool findPose(
    const geometry_msgs::msg::Pose2D & original_pose,
    const geometry_msgs::msg::Pose2D & edge_pose,
    double d,
    geometry_msgs::msg::Pose2D & output_pose) const;
  /** 将 (vx, vy) 归一化为单位方向。 */
  void calculateLineParam(double & x, double & y, double vx, double vy) const;
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

  rclcpp::Logger logger_{rclcpp::get_logger("FastPathPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<FootprintChecker> footprint_checker_;

  double _goal_occupied_tolerance{0.5};
  double _goal_search_resolution{0.1};
  double _straight_check_length_ratio{0.5};
  double _straight_path_resolution{0.1};
  double footprint_back_x_{0.0};
  double footprint_front_x_{0.0};
  double footprint_y_min_{0.0};
  double footprint_y_max_{0.0};
  bool enable_line_stretch_{true};
  double line_stretch_max_{0.4};
  double line_stretch_goal_window_{0.8};
  bool line_stretch_allow_extend_{false};
  bool enable_line_rotate_{true};
  int line_rotate_max_iters_{5};
  double line_rotate_goal_shift_tol_{0.5};
  double corridor_intrusion_tol_{0.08};

  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr narrow_passages_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_backward_sub_;
  mutable std::mutex narrow_polygons_mutex_;
  mutable std::mutex enable_backward_mutex_;
  std::vector<geometry_msgs::msg::Polygon> narrow_polygons_;
  bool narrow_polygons_received_{false};
  bool latched_narrow_passage_{false};
  bool enable_backward_cmd_{false};
  bool enable_backward_cmd_received_{false};

  std::shared_ptr<PlanningDebugViz> debug_viz_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__FAST_PATH_PLANNER_HPP_
