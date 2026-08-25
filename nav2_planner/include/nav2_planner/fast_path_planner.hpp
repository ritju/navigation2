// Copyright 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef NAV2_PLANNER__FAST_PATH_PLANNER_HPP_
#define NAV2_PLANNER__FAST_PATH_PLANNER_HPP_

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
 * 被占用时为邻域搜索到的最近可通行点。
 */
struct FastPlanResult
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped snapped_goal;
  FastPlanReason reason{FastPlanReason::NeedAstar};
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
   * @brief 先 snap 占用目标，再尝试直线段。
   * @param start 规划起点（全局系）
   * @param goal 原始目标
   * @param allow_straight 是否尝试直线（enable_straight_expand）
   * @param allow_reverse 是否允许倒车（窄通道或 /enable_backward）；
   *        为 true 时前进直线失败会再试后退直线
   * @return Path + snapped_goal + 失败/成功原因
   */
  FastPlanResult compute(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool allow_straight,
    bool allow_reverse);

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

  /** @brief 动态参数：目标占用邻域搜索半径 (m)。 */
  void setGoalOccupiedTolerance(double value) {_goal_occupied_tolerance = value;}
  /** @brief 动态参数：邻域搜索步长 (m)。 */
  void setGoalSearchResolution(double value) {_goal_search_resolution = value;}
  /** @brief 动态参数：footprint 向后扩展 (m)，用于 isFree。 */
  void setFootprintExtendBackX(double value) {_footprint_extend_back_x = value;}
  /** @brief 动态参数：footprint 向前扩展 (m)，用于 isFree。 */
  void setFootprintExtendFrontX(double value) {_footprint_extend_front_x = value;}
  /** @brief 动态参数：footprint 横向扩展 (m)，用于 isFree。 */
  void setFootprintExtendY(double value) {_footprint_extend_y = value;}
  /**
   * @brief 动态参数：直线碰撞采样步长 = 车长 * ratio。
   * 相对栅格分辨率取较大值，避免漏检。
   */
  void setStraightCheckLengthRatio(double value) {_straight_check_length_ratio = value;}
  /** @brief 动态参数：直线路径点间距 (m)，与碰撞步长独立。 */
  void setStraightPathResolution(double value) {_straight_path_resolution = value;}

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

  /**
   * @brief 带 footprint 纵向/横向扩展的整车可通行检查。
   *
   * LETHAL 与 INSCRIBED 都视为碰撞。越出地图视为不可通行。
   */
  bool isFree(
    const geometry_msgs::msg::PoseStamped & pose,
    double footprint_extend_back_x,
    double footprint_extend_front_x,
    double footprint_extend_y) const;

  /**
   * @brief 若 goal 占用，按切比雪夫圈由近及远搜索；最近一圈找到点即退出。
   * @return false 表示邻域内找不到可通行点
   */
  bool snapOccupiedGoal(geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 尝试生成 start→goal 直线段（前进或后退）。
   *
   * 先原地转到连线对应车头朝向，再按车长比例步长做整车碰撞检查；
   * 通过后再按 straight_path_resolution 插值出点。
   * reverse 时朝向为 line_yaw+π，前后悬扩展对调。
   */
  nav_msgs::msg::Path tryStraightPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool reverse);

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
  double _footprint_extend_back_x{0.0};
  double _footprint_extend_front_x{0.0};
  double _footprint_extend_y{0.0};
  double _straight_check_length_ratio{0.5};
  double _straight_path_resolution{0.1};
  double footprint_back_x_{0.0};
  double footprint_front_x_{0.0};

  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr narrow_passages_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_backward_sub_;
  mutable std::mutex narrow_polygons_mutex_;
  mutable std::mutex enable_backward_mutex_;
  std::vector<geometry_msgs::msg::Polygon> narrow_polygons_;
  bool narrow_polygons_received_{false};
  bool latched_narrow_passage_{false};
  bool enable_backward_cmd_{false};
  bool enable_backward_cmd_received_{false};
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__FAST_PATH_PLANNER_HPP_
