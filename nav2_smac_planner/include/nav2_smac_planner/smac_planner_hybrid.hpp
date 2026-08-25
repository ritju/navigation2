// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "nav2_costmap_2d/exceptions.hpp"


namespace nav2_smac_planner
{

class SmacPlannerHybrid : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  
  // When ignore_inscribed is true, treat only LETHAL_OBSTACLE as collision and allow INSCRIBED_INFLATED_OBSTACLE.
  // In all other cases, both LETHAL_OBSTACLE and INSCRIBED_INFLATED_OBSTACLE are treated as collision.
  bool is_free(const geometry_msgs::msg::PoseStamped &pose,
               nav2_costmap_2d::Costmap2D * costmap,
               double footprint_extend_back_x,
               double footprint_extend_front_x,
               double footprint_extend_y,
               bool ignore_inscribed = false);

private:
  /** /narrow_passages 话题回调，更新狭窄通道多边形列表 */
  void narrowPassagesCallback(const garage_utils_msgs::msg::Polygons::SharedPtr msg);
  /** /enable_backward 话题回调，外部指令切换倒车/前进规划模式 */
  void enableBackwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
  /** 预计算 DUBIN / REEDS_SHEPP 两套距离启发式缓存，供运行时快速切换 */
  void buildMotionModelCaches();
  /** 将指定运动模型及其启发式缓存应用到 NodeHybrid 与 A* */
  void applyMotionModel(const MotionModel motion_model);
  /**
   * 更新狭窄通道 latch 状态：
   * - 进入（宽松）：base_link 落在多边形内
   * - 退出（严格）：footprint 全部顶点均在所有多边形外
   * - 中间态：保持上一状态，避免边界抖动
   */
  void updateNarrowPassageLatch(const geometry_msgs::msg::PoseStamped & start);
  /** 是否已收到非空的狭窄通道多边形 */
  bool narrowPolygonsAvailable() const;
  /** 判断 map 系下一点是否落在任一狭窄通道多边形内 */
  bool isPointInNarrowPassage(double x, double y) const;
  /** 判断 footprint 是否完全在所有狭窄通道多边形外 */
  bool isFootprintFullyOutsideNarrowPassages(
    const geometry_msgs::msg::PoseStamped & pose) const;
  /** 本次全局规划是否启用窄通道模式（latch 或 goal 在通道内） */
  bool isNarrowPassagePlanningActive(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);


  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _goal_occupied_tolerance;
  double _goal_search_resolution;
  double _goal_close_to_obstacle_distance;
  double _footprint_extend_back_x, _footprint_extend_front_x, _footprint_extend_y, _costmap_resulution;
  double footprint_back_x_, footprint_front_x_;
  /** 参数仍保留，实际直线捷径改由 planner_server 决策 */
  bool _enable_straight_expand{true};
  bool _enable_straight_expand_initial{true};
  /** 狭窄通道 latch：进入宽松（base_link 在内），退出严格（footprint 全在外） */
  bool _latched_narrow_passage{false};
  /** 是否至少收到过一次 /narrow_passages 消息 */
  bool _narrow_polygons_received{false};
  /** 当前是否处于 REEDS_SHEPP 倒车规划模式 */
  bool _use_reeds_for_planning{false};
  /** DUBIN / REEDS 启发式缓存是否已构建 */
  bool _motion_model_caches_built{false};
  LookupTable _dubin_dist_heuristic_cache;
  LookupTable _reeds_dist_heuristic_cache;
  float _dubin_size_lookup_cache{0.0f};
  float _reeds_size_lookup_cache{0.0f};
  mutable std::mutex _narrow_polygons_mutex;
  std::vector<geometry_msgs::msg::Polygon> _narrow_polygons;
  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr _narrow_passages_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _enable_backward_sub;
  mutable std::mutex _enable_backward_mutex;
  bool _enable_backward_cmd_{false};
  bool _enable_backward_cmd_received_{false};
  /** When enabled, shorten Hybrid-A* limits if ROI 8-connected weighted distance is below threshold. */
  bool _enable_close_range_roi_budget{false};
  double _close_range_roi_margin{5.0};
  double _close_range_threshold_m{25.0};
  double _close_range_max_planning_time{5.0};
  int _close_range_max_iterations{250000};
  double _minimum_turning_radius_global_coords;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _roi_connectivity_path_publisher;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
