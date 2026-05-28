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
  
  bool find_pose(geometry_msgs::msg::Pose2D original_pose, geometry_msgs::msg::Pose2D edge_pose, double d, geometry_msgs::msg::Pose2D &output_pose);
  void calculate_line_param(double &x, double &y, double vx, double vy);
  // When ignore_inscribed is true, treat only LETHAL_OBSTACLE as collision and allow INSCRIBED_INFLATED_OBSTACLE.
  // In all other cases, both LETHAL_OBSTACLE and INSCRIBED_INFLATED_OBSTACLE are treated as collision.
  bool is_free(const geometry_msgs::msg::PoseStamped &pose,
               nav2_costmap_2d::Costmap2D * costmap,
               double footprint_extend_back_x,
               double footprint_extend_front_x,
               double footprint_extend_y,
               bool ignore_inscribed = false);

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
