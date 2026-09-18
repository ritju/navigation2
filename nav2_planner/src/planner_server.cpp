// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"
#include "nav2_smac_planner/hybrid_heading_hint.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace
{

struct PlanSessionStats
{
  size_t n_goals{0};
  size_t n_success{0};
  size_t n_straight{0};
  size_t n_hybrid{0};
  double t_straight_sec{0.0};
  double t_hybrid_sec{0.0};

  struct FailedGoal
  {
    unsigned int index{0};
    double x{0.0};
    double y{0.0};
    std::string reason;
  };
  std::vector<FailedGoal> failed;

  void recordKind(nav2_planner::GetPlanKind kind, double dt_sec)
  {
    if (kind == nav2_planner::GetPlanKind::Straight) {
      ++n_straight;
      t_straight_sec += dt_sec;
    } else if (kind == nav2_planner::GetPlanKind::Hybrid) {
      ++n_hybrid;
      t_hybrid_sec += dt_sec;
    }
  }

  void addFailed(
    unsigned int index, const geometry_msgs::msg::PoseStamped & goal,
    const std::string & reason)
  {
    failed.push_back({index, goal.pose.position.x, goal.pose.position.y, reason});
  }
};

std::string failedReasonFromMeta(
  nav2_planner::GetPlanKind kind, bool path_empty, bool heading_rejected)
{
  if (heading_rejected) {
    return "hybrid_heading_rejected";
  }
  if (kind == nav2_planner::GetPlanKind::Failed) {
    return "fastpath_unreachable_or_no_path";
  }
  if (kind == nav2_planner::GetPlanKind::Hybrid && path_empty) {
    return "hybrid_empty";
  }
  if (kind == nav2_planner::GetPlanKind::Straight) {
    return path_empty ? "straight_empty" : "straight_invalid";
  }
  if (kind == nav2_planner::GetPlanKind::Hybrid) {
    return "hybrid_invalid";
  }
  return "empty_or_invalid_path";
}

double wrapPi(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

double pathChordYaw(const nav_msgs::msg::Path & p)
{
  if (p.poses.size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & a = p.poses[p.poses.size() - 2].pose.position;
  const auto & b = p.poses.back().pose.position;
  return std::atan2(b.y - a.y, b.x - a.x);
}

double pathEndYaw(const nav_msgs::msg::Path & p)
{
  if (p.poses.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return tf2::getYaw(p.poses.back().pose.orientation);
}

enum class ViaRole
{
  LongInterior = 0,
  Short,
  Corner,
  Last
};

const char * viaRoleName(const ViaRole role)
{
  switch (role) {
    case ViaRole::Short: return "short";
    case ViaRole::Corner: return "corner";
    case ViaRole::Last: return "last";
    case ViaRole::LongInterior:
    default:
      return "long";
  }
}

struct ViaClass
{
  ViaRole role{ViaRole::LongInterior};
  double vertex_ang{180.0};
  double edge_len{0.0};
  double out_yaw{std::numeric_limits<double>::quiet_NaN()};
};

double polylineVertexAngleDeg(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const int idx,
  const int span)
{
  const int n = static_cast<int>(plan.size());
  const int ia = idx - span;
  const int ic = idx + span;
  if (span < 1 || ia < 0 || ic >= n || idx < 0 || idx >= n) {
    return 180.0;
  }
  const auto & p0 = plan[static_cast<size_t>(idx)].pose.position;
  const auto & p1 = plan[static_cast<size_t>(ia)].pose.position;
  const auto & p2 = plan[static_cast<size_t>(ic)].pose.position;
  const double dx_a = p0.x - p1.x;
  const double dy_a = p0.y - p1.y;
  const double dx_b = p2.x - p0.x;
  const double dy_b = p2.y - p0.y;
  const double dx_c = p2.x - p1.x;
  const double dy_c = p2.y - p1.y;
  const double length_a = std::sqrt(dx_a * dx_a + dy_a * dy_a);
  const double length_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);
  const double length_c = std::sqrt(dx_c * dx_c + dy_c * dy_c);
  constexpr double min_segment_length = 1e-6;
  if (length_a < min_segment_length || length_b < min_segment_length) {
    return 180.0;
  }
  double cos_theta =
    (length_a * length_a + length_b * length_b - length_c * length_c) /
    (2.0 * length_a * length_b);
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
  return std::acos(cos_theta) * 180.0 / M_PI;
}

std::vector<ViaClass> classifyViaRoles(
  const std::vector<geometry_msgs::msg::PoseStamped> & goals,
  int span,
  double theta_colinear_deg,
  double theta_corner_deg,
  double L_short,
  double L_footprint)
{
  const int n = static_cast<int>(goals.size());
  std::vector<ViaClass> roles(static_cast<size_t>(std::max(0, n)));
  if (n <= 0) {
    return roles;
  }
  span = std::max(1, span);
  for (int i = 0; i < n; ++i) {
    roles[static_cast<size_t>(i)].vertex_ang = polylineVertexAngleDeg(goals, i, span);
    if (i + 1 < n) {
      const auto & a = goals[static_cast<size_t>(i)].pose.position;
      const auto & b = goals[static_cast<size_t>(i + 1)].pose.position;
      roles[static_cast<size_t>(i)].out_yaw = std::atan2(b.y - a.y, b.x - a.x);
    }
  }

  struct Edge
  {
    int start{0};
    int end{0};
    double length{0.0};
    bool short_edge{false};
  };
  std::vector<Edge> edges;
  int s = 0;
  while (s < n - 1) {
    int e = s;
    double length = std::hypot(
      goals[static_cast<size_t>(s + 1)].pose.position.x -
      goals[static_cast<size_t>(s)].pose.position.x,
      goals[static_cast<size_t>(s + 1)].pose.position.y -
      goals[static_cast<size_t>(s)].pose.position.y);
    while (e + 1 < n - 1 &&
      roles[static_cast<size_t>(e + 1)].vertex_ang >= theta_colinear_deg)
    {
      e += 1;
      length += std::hypot(
        goals[static_cast<size_t>(e + 1)].pose.position.x -
        goals[static_cast<size_t>(e)].pose.position.x,
        goals[static_cast<size_t>(e + 1)].pose.position.y -
        goals[static_cast<size_t>(e)].pose.position.y);
    }
    edges.push_back({s, e + 1, length, false});
    s = e + 1;
  }

  for (auto & E : edges) {
    bool short_edge = E.length < L_short;
    if (E.length < L_footprint) {
      const bool left_corner = (E.start > 0) &&
        (roles[static_cast<size_t>(E.start)].vertex_ang < theta_corner_deg);
      const bool right_corner = (E.end < n - 1) &&
        (roles[static_cast<size_t>(E.end)].vertex_ang < theta_corner_deg);
      if (left_corner || right_corner || E.end == n - 1) {
        short_edge = true;
      }
    }
    E.short_edge = short_edge;
    for (int i = E.start; i <= E.end; ++i) {
      roles[static_cast<size_t>(i)].edge_len = E.length;
      if (short_edge) {
        roles[static_cast<size_t>(i)].role = ViaRole::Short;
      }
    }
  }

  for (int i = 1; i <= n - 2; ++i) {
    if (roles[static_cast<size_t>(i)].vertex_ang < theta_corner_deg) {
      roles[static_cast<size_t>(i)].role = ViaRole::Corner;
    }
  }
  roles[static_cast<size_t>(n - 1)].role = ViaRole::Last;
  return roles;
}

enum class HeadingTrimKind
{
  Unchanged,
  Trimmed,
  Failed
};

HeadingTrimKind trimHybridPathTailByHeading(
  nav_msgs::msg::Path & path,
  double goal_yaw,
  double heading_tol,
  double max_trim_length)
{
  if (path.poses.empty()) {
    return HeadingTrimKind::Failed;
  }
  auto yaw_ok = [&](size_t i) {
    return std::fabs(
      wrapPi(tf2::getYaw(path.poses[i].pose.orientation) - goal_yaw)) <= heading_tol;
  };
  const size_t n = path.poses.size();
  if (n == 1) {
    return yaw_ok(0) ? HeadingTrimKind::Unchanged : HeadingTrimKind::Failed;
  }
  if (yaw_ok(n - 1)) {
    return HeadingTrimKind::Unchanged;
  }
  double acc = 0.0;
  int keep = -1;
  for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
    if (i < static_cast<int>(n) - 1) {
      const auto & a = path.poses[static_cast<size_t>(i)].pose.position;
      const auto & b = path.poses[static_cast<size_t>(i + 1)].pose.position;
      acc += std::hypot(b.x - a.x, b.y - a.y);
      if (acc > max_trim_length + 1e-9) {
        break;
      }
    }
    if (yaw_ok(static_cast<size_t>(i))) {
      keep = i;
      break;
    }
  }
  if (keep < 1) {
    return HeadingTrimKind::Failed;
  }
  path.poses.resize(static_cast<size_t>(keep) + 1);
  return HeadingTrimKind::Trimmed;
}

struct HybridHeadingHintGuard
{
  ~HybridHeadingHintGuard()
  {
    nav2_smac_planner::setHybridPendingGoalHeadingTolerance(-1.0);
  }
};

void logPlanSessionStats(
  const rclcpp::Logger & logger,
  const char * tag,
  const PlanSessionStats & stats,
  double total_sec,
  size_t concat_poses)
{
  auto fmt_avg_ms = [](size_t n, double t_sec) -> std::string {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    if (n == 0) {
      oss << "0.00ms";
    } else {
      oss << (t_sec / static_cast<double>(n)) * 1000.0 << "ms";
    }
    return oss.str();
  };
  const std::string avg_s = fmt_avg_ms(stats.n_straight, stats.t_straight_sec);
  const std::string avg_h = fmt_avg_ms(stats.n_hybrid, stats.t_hybrid_sec);

  RCLCPP_INFO(
    logger,
    "[%s] summary: goals=%zu concat_poses=%zu total=%.3fs "
    "success=%zu failed=%zu straight=%zu avg=%s hybrid=%zu avg=%s",
    tag,
    stats.n_goals,
    concat_poses,
    total_sec,
    stats.n_success,
    stats.failed.size(),
    stats.n_straight,
    avg_s.c_str(),
    stats.n_hybrid,
    avg_h.c_str());

  if (stats.failed.empty()) {
    RCLCPP_INFO(logger, "[%s] failed goals: none", tag);
    return;
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  for (size_t k = 0; k < stats.failed.size(); ++k) {
    const auto & f = stats.failed[k];
    if (k > 0) {
      oss << "; ";
    }
    oss << "via=" << f.index << " (" << f.x << ", " << f.y << ") reason=" << f.reason;
  }
  RCLCPP_WARN(logger, "[%s] failed goals: %s", tag, oss.str().c_str());
}

}  // namespace

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  footprint_collision_checker_(nullptr),
  _goal_occupied_tolerance(0.5),
  _goal_search_resolution(0.1)

{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);
  declare_parameter("goal_occupied_tolerance", 0.5);
  declare_parameter("goal_search_resolution", 0.1);
  declare_parameter("enable_straight_expand", true);
  declare_parameter("straight_check_length_ratio", 0.5);
  declare_parameter("straight_path_resolution", 0.1);
  declare_parameter("publish_planning_debug", true);
  declare_parameter("planning_debug_footprint_stride", 0);
  declare_parameter("planning_debug_keep_mode", "session");

  declare_parameter("near_distance_threshold", 2.0);
  declare_parameter("near_yaw_threshold", 0.35);
  declare_parameter("enable_line_rotate", true);
  declare_parameter("line_rotate_max_iters", 5);
  declare_parameter("line_rotate_goal_shift_tol", 0.5);
  declare_parameter("corridor_intrusion_tol", 0.08);
  declare_parameter("enable_line_stretch", true);
  declare_parameter("line_stretch_max", 0.4);
  declare_parameter("line_stretch_goal_window", 0.8);
  declare_parameter("line_stretch_allow_extend", false);
  declare_parameter("rewrite_via_yaw_to_approach", true);
  declare_parameter("via_heading_tolerance", 0.35);
  declare_parameter("via_heading_trim_length", 1.0);
  declare_parameter("via_angle_span", 2);
  declare_parameter("edge_colinear_angle_deg", 135.0);
  declare_parameter("corner_angle_deg", 135.0);
  declare_parameter("short_edge_length", 2.0);
  declare_parameter("corner_sweep_scale", 1.2);
  declare_parameter("corner_snap_tolerance", 1.5);
  declare_parameter("corner_snap_enable", true);

  get_parameter("planner_plugins", planner_ids_);
  get_parameter("goal_occupied_tolerance", _goal_occupied_tolerance);
  get_parameter("goal_search_resolution", _goal_search_resolution);
  get_parameter("enable_straight_expand", enable_straight_expand_);
  get_parameter("near_distance_threshold", near_distance_threshold_);
  get_parameter("near_yaw_threshold", near_yaw_threshold_);
  get_parameter("enable_line_rotate", enable_line_rotate_);
  get_parameter("line_rotate_max_iters", line_rotate_max_iters_);
  get_parameter("line_rotate_goal_shift_tol", line_rotate_goal_shift_tol_);
  get_parameter("enable_line_stretch", enable_line_stretch_);
  get_parameter("line_stretch_max", line_stretch_max_);
  get_parameter("line_stretch_goal_window", line_stretch_goal_window_);
  get_parameter("line_stretch_allow_extend", line_stretch_allow_extend_);
  get_parameter("rewrite_via_yaw_to_approach", rewrite_via_yaw_to_approach_);
  get_parameter("via_heading_tolerance", via_heading_tolerance_);
  get_parameter("via_heading_trim_length", via_heading_trim_length_);
  get_parameter("via_angle_span", via_angle_span_);
  get_parameter("edge_colinear_angle_deg", edge_colinear_angle_deg_);
  get_parameter("corner_angle_deg", corner_angle_deg_);
  get_parameter("short_edge_length", short_edge_length_);
  get_parameter("corner_sweep_scale", corner_sweep_scale_);
  get_parameter("corner_snap_tolerance", corner_snap_tolerance_);
  get_parameter("corner_snap_enable", corner_snap_enable_);

  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
  rclcpp::Logger global_costmap_logger = rclcpp::get_logger("global_costmap.global_costmap");
  global_costmap_logger.set_level(rclcpp::Logger::Level::Warn);
}

PlannerServer::~PlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();
  footprint_collision_checker_ = std::make_shared<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  fast_path_planner_ = std::make_unique<FastPathPlanner>();
  fast_path_planner_->configure(
    node, costmap_ros_, footprint_collision_checker_, planner_ids_);
  fast_path_planner_->setCornerSweepScale(corner_sweep_scale_);
  fast_path_planner_->setCornerSnapTolerance(corner_snap_tolerance_);
  fast_path_planner_->setCornerSnapEnable(corner_snap_enable_);

  debug_viz_ = std::make_shared<PlanningDebugViz>();
  debug_viz_->configure(node, costmap_ros_);
  fast_path_planner_->setDebugViz(debug_viz_);

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available. enable_straight_expand=%s "
    "near_dist=%.2f near_yaw=%.2f via_heading_tol=%.2f via_trim_len=%.2f "
    "via_span=%d colinear_deg=%.1f corner_deg=%.1f short_edge=%.2f "
    "sweep_k=%.2f corner_snap_tol=%.2f corner_snap=%s "
    "stretch=%s rotate=%s debug=%s",
    planner_ids_concat_.c_str(),
    enable_straight_expand_ ? "true" : "false",
    near_distance_threshold_, near_yaw_threshold_,
    via_heading_tolerance_, via_heading_trim_length_,
    via_angle_span_, edge_colinear_angle_deg_, corner_angle_deg_,
    short_edge_length_, corner_sweep_scale_, corner_snap_tolerance_,
    corner_snap_enable_ ? "true" : "false",
    enable_line_stretch_ ? "true" : "false",
    enable_line_rotate_ ? "true" : "false",
    (debug_viz_ && debug_viz_->enabled()) ? "true" : "false");

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  if (debug_viz_) {
    debug_viz_->activate();
  }
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->activate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();
  if (debug_viz_) {
    debug_viz_->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();

  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  if (fast_path_planner_) {
    fast_path_planner_->cleanup();
    fast_path_planner_.reset();
  }
  if (debug_viz_) {
    debug_viz_->cleanup();
    debug_viz_.reset();
  }
  costmap_thread_.reset();
  costmap_ = nullptr;
  footprint_collision_checker_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

bool PlannerServer::validatePath(
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
     // 若全局路径划失败跳过此点继续，不终止action
    // action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %zu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_poses_->get_current_goal();

  geometry_msgs::msg::PoseStamped robot_pose;
  if (costmap_ros_->getRobotPose(robot_pose))
  {
    double footprint_cost = footprint_collision_checker_->footprintCostAtPose(robot_pose.pose.position.x, robot_pose.pose.position.y, 
                                                          tf2::getYaw(robot_pose.pose.orientation), costmap_ros_->getRobotFootprint());
    RCLCPP_INFO(get_logger(), "Robot footprint cost at current pose (%.2f, %.2f) is %f", 
                robot_pose.pose.position.x, robot_pose.pose.position.y, footprint_cost);
    if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      RCLCPP_WARN(
        get_logger(),
        "Robot is currently in collision at (%.2f, %.2f). Cannot compute path through poses.",
        robot_pose.pose.position.x, robot_pose.pose.position.y);
      action_server_poses_->terminate_current();
      return;
    }
  }
  else
  {
    RCLCPP_WARN(
      get_logger(),
      "Unable to get robot pose to check for collision.");
  }

  if (goal->goals.empty())
  {
    nav_msgs::msg::Path result_path;
    result_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    result_path.header.stamp = this->now();
    result_path.poses.clear();
    auto result = std::make_shared<ActionThroughPoses::Result>();
    result->path = result_path;
    result->planning_time = this->now() - start_time;
    action_server_poses_->succeeded_current(result);
    RCLCPP_WARN(
      get_logger(),
      "Compute path through poses requested a plan with no viapoint poses, returning.");
    return;
  }
  
  // 将 pose.z 置为0
  auto goal_poses = goal->goals;
  for (auto &pose : goal_poses) {
    pose.pose.position.z = 0.0;  // Ensure z is zero for 2D planning
  }
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;
  PlanSessionStats stats;
  stats.n_goals = goal_poses.size();
  bool summary_logged = false;
  auto log_summary = [&]() {
    if (summary_logged) {
      return;
    }
    summary_logged = true;
    logPlanSessionStats(
      get_logger(), "ThroughPoses", stats,
      (this->now() - start_time).seconds(),
      concat_path.poses.size());
  };

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }
    start.pose.position.z = 0.0;  // Ensure z is zero for 2D planning

    if (debug_viz_) {
      debug_viz_->beginSession();
    }
    auto flush_dbg = [this]() {
      if (debug_viz_) {
        debug_viz_->endSession();
      }
    };

    RCLCPP_INFO(
      get_logger(),
      "[ThroughPoses] start=(%.3f, %.3f, yaw=%.3f) goals=%zu enable_straight=%s",
      start.pose.position.x, start.pose.position.y,
      tf2::getYaw(start.pose.orientation), goal_poses.size(),
      enable_straight_expand_ ? "true" : "false");

    double L_fp = 0.5;
    if (costmap_ros_) {
      double xmin = 0.0;
      double xmax = 0.0;
      for (const auto & pt : costmap_ros_->getRobotFootprint()) {
        xmin = std::min(xmin, static_cast<double>(pt.x));
        xmax = std::max(xmax, static_cast<double>(pt.x));
      }
      L_fp = std::max(0.1, xmax - xmin);
    }
    const std::vector<ViaClass> via_roles = classifyViaRoles(
      goal_poses, via_angle_span_, edge_colinear_angle_deg_, corner_angle_deg_,
      short_edge_length_, L_fp);

    // Get consecutive paths through these points
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal_poses.size(); i++) {
      if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
        log_summary();
        flush_dbg();
        return;
      }
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        // pick the end of the last planning task as the start for the next one
        // to allow for path tolerance deviations
        // curr_start = concat_path.poses.back();
        // curr_start.header = concat_path.header;
        if (concat_path.poses.size() > 0) {
            curr_start = concat_path.poses.back();
            curr_start.header = concat_path.header;
        } else {
          curr_start = start;
        }
      }
      curr_goal = goal_poses[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        log_summary();
        flush_dbg();
        return;
      }

      const bool is_last = (i + 1u == static_cast<unsigned int>(goal_poses.size()));
      const bool near = isNearSegment(curr_start, curr_goal);
      const double orig_goal_yaw = tf2::getYaw(curr_goal.pose.orientation);
      const bool rewrite_yaw = shouldRewriteGoalYawToApproach(
        curr_start, curr_goal, is_last);

      const ViaClass via_cls = (i < via_roles.size()) ?
        via_roles[i] : ViaClass{};
      FastPlanOptions plan_opt = makeBaseFastOptions(
        curr_start, curr_goal, near || is_last, near, is_last);
      if (corner_snap_enable_) {
        if (via_cls.role == ViaRole::Corner) {
          plan_opt.strict_goal_footprint = true;
          plan_opt.use_corner_sweep = true;
          plan_opt.allow_intrusion_exempt = false;
          plan_opt.allow_stretch = false;
          plan_opt.out_yaw = via_cls.out_yaw;
          plan_opt.snap_tolerance = corner_snap_tolerance_;
        } else if (via_cls.role == ViaRole::Short) {
          plan_opt.strict_goal_footprint = true;
          plan_opt.use_corner_sweep = true;
          plan_opt.allow_intrusion_exempt = false;
          plan_opt.allow_stretch = false;
          plan_opt.out_yaw = via_cls.out_yaw;
          plan_opt.snap_tolerance = corner_snap_tolerance_;
        }
      }

      RCLCPP_INFO(
        get_logger(),
        "[ThroughPoses] via=%u/%zu start=(%.3f, %.3f, yaw=%.3f) goal=(%.3f, %.3f, yaw=%.3f) "
        "dist=%.3f near=%s last=%s rewrite_yaw=%s stretch=%s rotate=%s "
        "footprint_corridor=%s role=%s ang=%.1f edge_len=%.2f sweep=%s concat_poses=%zu",
        i, goal_poses.size(),
        curr_start.pose.position.x, curr_start.pose.position.y,
        tf2::getYaw(curr_start.pose.orientation),
        curr_goal.pose.position.x, curr_goal.pose.position.y,
        tf2::getYaw(curr_goal.pose.orientation),
        std::hypot(
          curr_goal.pose.position.x - curr_start.pose.position.x,
          curr_goal.pose.position.y - curr_start.pose.position.y),
        near ? "true" : "false",
        is_last ? "true" : "false",
        rewrite_yaw ? "true" : "false",
        plan_opt.allow_stretch ? "true" : "false",
        plan_opt.allow_rotate ? "true" : "false",
        plan_opt.strict_goal_footprint ? "true" : "false",
        viaRoleName(via_cls.role), via_cls.vertex_ang, via_cls.edge_len,
        plan_opt.use_corner_sweep ? "true" : "false",
        concat_path.poses.size());

      if (debug_viz_) {
        debug_viz_->setSegmentContext(i, goal_poses.size(), curr_start, curr_goal);
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path;
      static constexpr int kMaxStartOccupiedRetries = 50;
      int start_occupied_retries = 0;
      const std::string start_occupied_msg = "Cannot generate a plan, start is occupied!";
      const std::string start_lethal_msg =
        "Starting point in lethal space! Cannot create feasible plan.";
      GetPlanMeta last_meta;
      bool plugin_threw = false;
      const auto t_via = this->now();
      while (rclcpp::ok()) {
        if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
          log_summary();
          flush_dbg();
          return;
        }

        try {
          curr_path = getPlan(
            curr_start, curr_goal, goal->planner_id, plan_opt, &last_meta);
          break;  // planned (or at least returned something for validation)
        } catch (const std::runtime_error & ex) {
          RCLCPP_WARN(
            get_logger(),
            "%s plugin failed to plan path to goal (%.2f, %.2f): \"%s\"",
            goal->planner_id.c_str(), curr_goal.pose.position.x,
            curr_goal.pose.position.y, ex.what());

          // Recovery: if smac throws start-occupied or lethal-start, drop the last point from
          // concat_path (the next segment's start), update curr_start, and retry planning.
          const std::string ex_msg(ex.what());
          if ((ex_msg == start_occupied_msg || ex_msg == start_lethal_msg) &&
            start_occupied_retries < kMaxStartOccupiedRetries &&
            !concat_path.poses.empty())
          {
            concat_path.poses.pop_back();
            if (!concat_path.poses.empty()) {
              curr_start = concat_path.poses.back();
              curr_start.header = concat_path.header;
            } else {
              curr_start = start;
            }
            start_occupied_retries++;
            RCLCPP_WARN(
              get_logger(),
              "[computePlanThroughPoses] Recovery: smac throws start-occupied or lethal-start, drop the last point from concat_path, start_occupied_retries: %d", start_occupied_retries);
            continue;
          }

          // Other runtime errors (or we can't recover): leave curr_path empty and move on.
          curr_path = nav_msgs::msg::Path();
          plugin_threw = true;
          break;
        }
      }
      stats.recordKind(last_meta.kind, (this->now() - t_via).seconds());
      // check path for validity
      if (!validatePath(curr_goal, curr_path, goal->planner_id)) {
        const std::string reason = plugin_threw ?
          "plugin_exception" :
          failedReasonFromMeta(
            last_meta.kind, curr_path.poses.empty(), last_meta.heading_rejected);
        stats.addFailed(i, curr_goal, reason);
        RCLCPP_INFO(
          get_logger(),
          "[ThroughPoses] via=%u FAILED concat_unchanged poses=%zu goal=(%.2f, %.2f) last=%s",
          i, concat_path.poses.size(),
          curr_goal.pose.position.x,
          curr_goal.pose.position.y,
          is_last ? "true" : "false");
        if (is_last) {
          RCLCPP_ERROR(
            get_logger(),
            "[ThroughPoses] last goal failed, terminating through-poses");
          log_summary();
          flush_dbg();
          action_server_poses_->terminate_current();
          return;
        }
        continue;
      }
      stats.n_success++;

      if (last_meta.kind == GetPlanKind::Hybrid && !curr_path.poses.empty()) {
        const double request_yaw = tf2::getYaw(curr_goal.pose.orientation);
        const double path_end = pathEndYaw(curr_path);
        const double chord = pathChordYaw(curr_path);
        const auto & end_p = curr_path.poses.back().pose.position;
        const double xy_err = std::hypot(
          end_p.x - curr_goal.pose.position.x,
          end_p.y - curr_goal.pose.position.y);
        RCLCPP_INFO(
          get_logger(),
          "[ThroughPoses] via=%u Hybrid yaw orig=%.3f request=%.3f snapped=%.3f "
          "path_end=%.3f chord=%.3f d_end_orig=%.3f d_end_req=%.3f d_end_snap=%.3f "
          "d_chord_orig=%.3f d_chord_req=%.3f xy_err=%.3f end_xy=(%.3f, %.3f) "
          "rewrite=%s",
          i, orig_goal_yaw, request_yaw, last_meta.snapped_yaw,
          path_end, chord,
          wrapPi(path_end - orig_goal_yaw),
          wrapPi(path_end - request_yaw),
          wrapPi(path_end - last_meta.snapped_yaw),
          wrapPi(chord - orig_goal_yaw),
          wrapPi(chord - request_yaw),
          xy_err, end_p.x, end_p.y,
          rewrite_yaw ? "true" : "false");
      }

      RCLCPP_INFO(
          get_logger(),
          "[ThroughPoses] via=%u OK poses=%zu goal=(%.2f, %.2f) concat=%zu",
          i, curr_path.poses.size(),
          curr_goal.pose.position.x,
          curr_goal.pose.position.y,
          concat_path.poses.size() + curr_path.poses.size());

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    if (concat_path.poses.size() == 0)
    {
      log_summary();
      flush_dbg();
      action_server_poses_->terminate_current();
      return;
    }

    log_summary();

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }
    action_server_poses_->succeeded_current(result);
    flush_dbg();
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    log_summary();
    if (debug_viz_) {
      debug_viz_->endSession();
    }
    action_server_poses_->terminate_current();
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    log_summary();
    if (debug_viz_) {
      debug_viz_->endSession();
    }
    action_server_poses_->terminate_current();
  }
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_pose_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose)) {
      return;
    }

    if (debug_viz_) {
      debug_viz_->beginSession();
      debug_viz_->setSegmentContext(0, 1, start, goal_pose);
    }

    const bool near = isNearSegment(start, goal_pose);
    const double orig_goal_yaw = tf2::getYaw(goal_pose.pose.orientation);

    PlanSessionStats stats;
    stats.n_goals = 1;
    GetPlanMeta meta;
    const auto t_goal = this->now();
    result->path = getPlan(
      start, goal_pose, goal->planner_id,
      makeBaseFastOptions(start, goal_pose, true, near, true), &meta);
    stats.recordKind(meta.kind, (this->now() - t_goal).seconds());
    if (debug_viz_) {
      debug_viz_->endSession();
    }

    if (meta.kind == GetPlanKind::Hybrid && !result->path.poses.empty()) {
      const double request_yaw = tf2::getYaw(goal_pose.pose.orientation);
      const double path_end = pathEndYaw(result->path);
      const double chord = pathChordYaw(result->path);
      RCLCPP_INFO(
        get_logger(),
        "[getPlan] Hybrid yaw orig=%.3f request=%.3f snapped=%.3f "
        "path_end=%.3f chord=%.3f d_end_orig=%.3f d_end_req=%.3f d_end_snap=%.3f "
        "d_chord_orig=%.3f rewrite=%s",
        orig_goal_yaw, request_yaw, meta.snapped_yaw,
        path_end, chord,
        wrapPi(path_end - orig_goal_yaw),
        wrapPi(path_end - request_yaw),
        wrapPi(path_end - meta.snapped_yaw),
        wrapPi(chord - orig_goal_yaw),
        "false");
    }

    auto log_pose_summary = [&]() {
      logPlanSessionStats(
        get_logger(), "getPlan", stats,
        (this->now() - start_time).seconds(),
        result->path.poses.size());
    };

    if (!validatePath(goal_pose, result->path, goal->planner_id)) {
      stats.addFailed(
        0, goal_pose, failedReasonFromMeta(
          meta.kind, result->path.poses.empty(), meta.heading_rejected));
      log_pose_summary();
      action_server_pose_->terminate_current();
      return;
    }
    stats.n_success = 1;

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
    log_pose_summary();
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    RCLCPP_WARN(
      get_logger(),
      "[getPlan] failed goals: via=0 (%.3f, %.3f) reason=plugin_exception",
      goal->goal.pose.position.x, goal->goal.pose.position.y);
    action_server_pose_->terminate_current();
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    RCLCPP_WARN(
      get_logger(),
      "[getPlan] failed goals: via=0 (%.3f, %.3f) reason=plugin_exception",
      goal->goal.pose.position.x, goal->goal.pose.position.y);
    action_server_pose_->terminate_current();
  }
}

bool
PlannerServer::allowStraightExpand(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & /*goal*/)
{
  if (fast_path_planner_) {
    fast_path_planner_->updateNarrowPassageLatch(start);
  }
  return enable_straight_expand_ && static_cast<bool>(fast_path_planner_);
}

bool
PlannerServer::isNearSegment(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double dist = std::hypot(dx, dy);
  if (dist >= near_distance_threshold_) {
    return false;
  }
  const double approach = std::atan2(dy, dx);
  const double yaw_s = tf2::getYaw(start.pose.orientation);
  const double yaw_g = tf2::getYaw(goal.pose.orientation);
  auto ang_abs = [](double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
      a += 2.0 * M_PI;
    }
    return std::fabs(a - M_PI);
  };
  if (ang_abs(yaw_s - approach) >= near_yaw_threshold_) {
    return false;
  }
  if (ang_abs(yaw_g - approach) >= near_yaw_threshold_) {
    return false;
  }
  return true;
}

bool
PlannerServer::isNearByDistance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dist = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  return dist <= near_distance_threshold_;
}

bool
PlannerServer::shouldRewriteGoalYawToApproach(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool is_terminal) const
{
  if (!rewrite_via_yaw_to_approach_ || is_terminal) {
    return false;
  }
  return isNearByDistance(start, goal);
}

FastPlanOptions
PlannerServer::makeBaseFastOptions(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool allow_stretch,
  bool allow_rotate,
  bool is_terminal)
{
  FastPlanOptions options;
  options.allow_straight = allowStraightExpand(start, goal);
  if (fast_path_planner_) {
    options.allow_reverse =
      fast_path_planner_->isBackwardActive() ||
      fast_path_planner_->isNarrowActive(start, goal);
  }
  options.allow_stretch = allow_stretch && enable_line_stretch_;
  options.allow_rotate = allow_rotate && enable_line_rotate_;
  options.strict_goal_footprint = is_terminal;
  options.rewrite_goal_yaw_to_approach = shouldRewriteGoalYawToApproach(
    start, goal, is_terminal);
  return options;
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  const FastPlanOptions & options,
  GetPlanMeta * meta)
{
  if (meta) {
    meta->kind = GetPlanKind::Failed;
    meta->heading_rejected = false;
  }
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  geometry_msgs::msg::PoseStamped snapped_goal = goal;
  if (fast_path_planner_) {
    const FastPlanResult fast_result =
      fast_path_planner_->compute(start, goal, options);
    snapped_goal = fast_result.snapped_goal;
    if (fast_result.reason == FastPlanReason::StraightOk) {
      RCLCPP_INFO(
        get_logger(),
        "[getPlan] StraightOk poses=%zu start=(%.2f, %.2f, yaw=%.3f) "
        "goal=(%.2f, %.2f) snapped=(%.2f, %.2f) stretch=%s rotate=%s footprint_corridor=%s",
        fast_result.path.poses.size(),
        start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation),
        goal.pose.position.x, goal.pose.position.y,
        snapped_goal.pose.position.x, snapped_goal.pose.position.y,
        options.allow_stretch ? "true" : "false",
        options.allow_rotate ? "true" : "false",
        options.strict_goal_footprint ? "true" : "false");
      if (meta) {
        meta->kind = GetPlanKind::Straight;
        meta->snapped_yaw = tf2::getYaw(snapped_goal.pose.orientation);
      }
      return fast_result.path;
    }
    if (fast_result.reason == FastPlanReason::GoalUnreachable) {
      RCLCPP_WARN(
        get_logger(),
        "[getPlan] GoalUnreachable original=(%.2f, %.2f) snapped search failed",
        goal.pose.position.x, goal.pose.position.y);
      return nav_msgs::msg::Path();
    }
    RCLCPP_INFO(
      get_logger(),
      "[getPlan] FastPath NeedAstar, plugin=%s start_yaw=%.3f (kept) "
      "goal_yaw=%.3f snapped_goal=(%.3f, %.3f) snapped_yaw=%.3f rewrite=%s",
      planner_id.c_str(),
      tf2::getYaw(start.pose.orientation),
      tf2::getYaw(goal.pose.orientation),
      snapped_goal.pose.position.x, snapped_goal.pose.position.y,
      tf2::getYaw(snapped_goal.pose.orientation),
      options.rewrite_goal_yaw_to_approach ? "true" : "false");
  }

  if (meta) {
    meta->kind = GetPlanKind::Hybrid;
    meta->snapped_yaw = tf2::getYaw(snapped_goal.pose.orientation);
  }

  const bool near_by_dist = isNearByDistance(start, goal);
  const double search_heading_tol =
    (near_by_dist && !options.strict_goal_footprint) ? via_heading_tolerance_ : -1.0;

  nav2_core::GlobalPlanner::Ptr planner;
  if (planners_.find(planner_id) != planners_.end()) {
    planner = planners_[planner_id];
  } else if (planners_.size() == 1 && planner_id.empty()) {
    RCLCPP_WARN_ONCE(
      get_logger(), "No planners specified in action call. "
      "Server will use only plugin %s in server."
      " This warning will appear once.", planner_ids_concat_.c_str());
    planner = planners_.begin()->second;
  } else {
    RCLCPP_ERROR(
      get_logger(), "planner %s is not a valid planner. "
      "Planner names are: %s", planner_id.c_str(),
      planner_ids_concat_.c_str());
  }

  nav_msgs::msg::Path plugin_path;
  if (planner) {
    HybridHeadingHintGuard heading_guard;
    nav2_smac_planner::setHybridPendingGoalHeadingTolerance(search_heading_tol);
    RCLCPP_INFO(
      get_logger(),
      "[getPlan] createPlan heading_search=%s last=%s near_dist=%s tol=%.3f",
      search_heading_tol >= 0.0 ? "on" : "off",
      options.strict_goal_footprint ? "true" : "false",
      near_by_dist ? "true" : "false",
      search_heading_tol);
    plugin_path = planner->createPlan(start, snapped_goal);
  }

  if (!plugin_path.poses.empty()) {
    const double request_yaw = tf2::getYaw(goal.pose.orientation);
    const double snap_yaw = tf2::getYaw(snapped_goal.pose.orientation);
    const double path_end = pathEndYaw(plugin_path);
    const double chord = pathChordYaw(plugin_path);
    RCLCPP_INFO(
      get_logger(),
      "[getPlan] Hybrid yaw request=%.3f snapped=%.3f path_end=%.3f chord=%.3f "
      "d_end_req=%.3f d_end_snap=%.3f d_chord_req=%.3f d_chord_snap=%.3f "
      "d_end_chord=%.3f poses=%zu near_dist=%s heading_search=%s",
      request_yaw, snap_yaw, path_end, chord,
      wrapPi(path_end - request_yaw),
      wrapPi(path_end - snap_yaw),
      wrapPi(chord - request_yaw),
      wrapPi(chord - snap_yaw),
      wrapPi(path_end - chord),
      plugin_path.poses.size(),
      near_by_dist ? "true" : "false",
      search_heading_tol >= 0.0 ? "on" : "off");
  }

  if (debug_viz_ && !plugin_path.poses.empty()) {
    debug_viz_->publishHybridRaw(plugin_path);
    RCLCPP_INFO(
      get_logger(),
      "[getPlan] plugin path poses=%zu (raw Hybrid/GridBased, heading trim not applied yet)",
      plugin_path.poses.size());
  }

  if (!plugin_path.poses.empty()) {
    const double goal_yaw = tf2::getYaw(snapped_goal.pose.orientation);
    const size_t n_before = plugin_path.poses.size();
    const HeadingTrimKind trim_kind = trimHybridPathTailByHeading(
      plugin_path, goal_yaw, via_heading_tolerance_, via_heading_trim_length_);
    if (trim_kind == HeadingTrimKind::Trimmed) {
      RCLCPP_INFO(
        get_logger(),
        "[getPlan] heading trim poses %zu -> %zu path_end_yaw=%.3f d_end_goal=%.3f "
        "max_len=%.2f near_dist=%s last=%s",
        n_before, plugin_path.poses.size(),
        pathEndYaw(plugin_path),
        wrapPi(pathEndYaw(plugin_path) - goal_yaw),
        via_heading_trim_length_,
        near_by_dist ? "true" : "false",
        options.strict_goal_footprint ? "true" : "false");
    } else if (trim_kind == HeadingTrimKind::Failed) {
      if (near_by_dist && !options.strict_goal_footprint) {
        RCLCPP_INFO(
          get_logger(),
          "[getPlan] heading trim failed, drop intermediate via goal=(%.3f, %.3f) "
          "goal_yaw=%.3f path_end=%.3f d_end=%.3f window=%.2f",
          goal.pose.position.x, goal.pose.position.y, goal_yaw,
          pathEndYaw(plugin_path),
          wrapPi(pathEndYaw(plugin_path) - goal_yaw),
          via_heading_trim_length_);
        if (meta) {
          meta->heading_rejected = true;
        }
        return nav_msgs::msg::Path();
      }
      RCLCPP_INFO(
        get_logger(),
        "[getPlan] heading trim failed, keep path (far or last) poses=%zu "
        "path_end=%.3f d_end=%.3f",
        plugin_path.poses.size(),
        pathEndYaw(plugin_path),
        wrapPi(pathEndYaw(plugin_path) - goal_yaw));
    }
  }

  return plugin_path;
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  // unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    // float current_distance = std::numeric_limits<float>::max();
    // float closest_distance = current_distance;
    // geometry_msgs::msg::Point current_point = current_pose.pose.position;
    // for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
    //   geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

    //   current_distance = nav2_util::geometry_utils::euclidean_distance(
    //     current_point,
    //     path_point);

    //   if (current_distance < closest_distance) {
    //     closest_point_index = i;
    //     closest_distance = current_distance;
    //   }
    // }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
        RCLCPP_INFO(get_logger(), "Pose is occuppied pose.x: %f, pose.y: %f, cost: %d !", request->path.poses[i].pose.position.x, request->path.poses[i].pose.position.y, cost);
        return;
      }

      tf2::Quaternion quat;
      double roll, pitch, yaw;
      tf2::fromMsg(request->path.poses[i].pose.orientation, quat);
      quat.normalize();
      tf2::Matrix3x3 mat(quat);
      mat.getRPY(roll, pitch, yaw);
      double footprint_cost = footprint_collision_checker_->footprintCostAtPose(request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, yaw, costmap_ros_->getRobotFootprint());
      if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        response->is_valid = false;
        RCLCPP_INFO(get_logger(), "Pose footprint is occuppied pose.x: %f, pose.y: %f, cost: %d !", request->path.poses[i].pose.position.x, request->path.poses[i].pose.position.y, cost);
        return;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      } else if (name == "goal_occupied_tolerance") {
        _goal_occupied_tolerance = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setGoalOccupiedTolerance(_goal_occupied_tolerance);
        }
      } else if (name == "goal_search_resolution") {
        _goal_search_resolution = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setGoalSearchResolution(_goal_search_resolution);
        }
      } else if (name == "straight_check_length_ratio") {
        if (fast_path_planner_) {
          fast_path_planner_->setStraightCheckLengthRatio(parameter.as_double());
        }
      } else if (name == "straight_path_resolution") {
        if (fast_path_planner_) {
          fast_path_planner_->setStraightPathResolution(parameter.as_double());
        }
      } else if (name == "near_distance_threshold") {
        near_distance_threshold_ = parameter.as_double();
      } else if (name == "near_yaw_threshold") {
        near_yaw_threshold_ = parameter.as_double();
      } else if (name == "via_heading_tolerance") {
        via_heading_tolerance_ = parameter.as_double();
      } else if (name == "via_heading_trim_length") {
        via_heading_trim_length_ = parameter.as_double();
      } else if (name == "edge_colinear_angle_deg") {
        edge_colinear_angle_deg_ = parameter.as_double();
      } else if (name == "corner_angle_deg") {
        corner_angle_deg_ = parameter.as_double();
      } else if (name == "short_edge_length") {
        short_edge_length_ = parameter.as_double();
      } else if (name == "corner_sweep_scale") {
        corner_sweep_scale_ = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setCornerSweepScale(corner_sweep_scale_);
        }
      } else if (name == "corner_snap_tolerance") {
        corner_snap_tolerance_ = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setCornerSnapTolerance(corner_snap_tolerance_);
        }
      } else if (name == "line_rotate_goal_shift_tol") {
        line_rotate_goal_shift_tol_ = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setLineRotateGoalShiftTol(line_rotate_goal_shift_tol_);
        }
      } else if (name == "corridor_intrusion_tol") {
        if (fast_path_planner_) {
          fast_path_planner_->setCorridorIntrusionTol(parameter.as_double());
        }
      } else if (name == "line_stretch_max") {
        line_stretch_max_ = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setLineStretchMax(line_stretch_max_);
        }
      } else if (name == "line_stretch_goal_window") {
        line_stretch_goal_window_ = parameter.as_double();
        if (fast_path_planner_) {
          fast_path_planner_->setLineStretchGoalWindow(line_stretch_goal_window_);
        }
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "line_rotate_max_iters") {
        line_rotate_max_iters_ = static_cast<int>(parameter.as_int());
        if (fast_path_planner_) {
          fast_path_planner_->setLineRotateMaxIters(line_rotate_max_iters_);
        }
      } else if (name == "via_angle_span") {
        via_angle_span_ = static_cast<int>(parameter.as_int());
      } else if (name == "planning_debug_footprint_stride") {
        if (debug_viz_) {
          debug_viz_->setFootprintStride(static_cast<int>(parameter.as_int()));
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "enable_straight_expand") {
        enable_straight_expand_ = parameter.as_bool();
      } else if (name == "publish_planning_debug") {
        if (debug_viz_) {
          debug_viz_->setEnabled(parameter.as_bool());
        }
      } else if (name == "enable_line_rotate") {
        enable_line_rotate_ = parameter.as_bool();
        if (fast_path_planner_) {
          fast_path_planner_->setEnableLineRotate(enable_line_rotate_);
        }
      } else if (name == "enable_line_stretch") {
        enable_line_stretch_ = parameter.as_bool();
        if (fast_path_planner_) {
          fast_path_planner_->setEnableLineStretch(enable_line_stretch_);
        }
      } else if (name == "line_stretch_allow_extend") {
        line_stretch_allow_extend_ = parameter.as_bool();
        if (fast_path_planner_) {
          fast_path_planner_->setLineStretchAllowExtend(line_stretch_allow_extend_);
        }
      } else if (name == "rewrite_via_yaw_to_approach") {
        rewrite_via_yaw_to_approach_ = parameter.as_bool();
      } else if (name == "corner_snap_enable") {
        corner_snap_enable_ = parameter.as_bool();
        if (fast_path_planner_) {
          fast_path_planner_->setCornerSnapEnable(corner_snap_enable_);
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "planning_debug_keep_mode" && debug_viz_) {
        debug_viz_->setKeepMode(parameter.as_string());
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
