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


#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
std::vector<geometry_msgs::msg::Point> rotate_vec;
const char* ROTATE_POINTS = getenv("ROTATE_POINTS_FOR_GLOBAL_PLANNER");
const char* FORBIDDEN_ROTATE_AREA_POINTS = getenv("FORBIDDEN_ROTATE_AREA_POINTS_FOR_GLOBAL_PLANNER");
struct ForbiddenPoint
{
  double x1;
  double y1;
  double x2;
  double y2;
};
std::vector<ForbiddenPoint> forbidden_area_vec;

namespace nav2_planner
{

bool makeRotationGoalFromString(
  const std::string & footprint_string,
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  std::string error;
  std::vector<std::vector<float>> vvf = nav2_costmap_2d::parseVVF(footprint_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"), "Error parsing rotation goal parameter: '%s'", error.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"), "Rotation goal string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"),
      "You must specify at least one point for the robot footprint."); //NOLINT
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      geometry_msgs::msg::Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      footprint.push_back(point);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "planner_server"),
        "Points in the rotation goal specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
        static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  return true;
}
bool makeForbiddenPointsFromString(
  const std::string & forbidden_area_string,
  std::vector<ForbiddenPoint> & forbidden_area)
{
  std::string error;
  std::vector<std::vector<float>> vvf = nav2_costmap_2d::parseVVF(forbidden_area_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"), "Error parsing forbidden area parameter: '%s'", error.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"), "Forbidden area string was '%s'.", forbidden_area_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "planner_server"),
      "You must specify at least two points for forbidden area."); //NOLINT
    return false;
  }
  forbidden_area.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 4) {
      ForbiddenPoint point;
      point.x1 = vvf[i][0];
      point.y1 = vvf[i][1];
      point.x2 = vvf[i][2];
      point.y2 = vvf[i][3];
      forbidden_area.emplace_back(point);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "planner_server"),
        "Points in the forbidden area specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
        static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  return true;
}

bool IsGoalInForbiddenArea(const std::vector<ForbiddenPoint> forbidden_points, const geometry_msgs::msg::PoseStamped current_pose)
{
  for (auto point : forbidden_points)
  {
    double min_x = (point.x1 > point.x2) ? point.x2 : point.x1;
    double max_x = (point.x1 > point.x2) ? point.x1 : point.x2;
    double min_y = (point.y1 > point.y2) ? point.y2 : point.y1;
    double max_y = (point.y1 > point.y2) ? point.y1 : point.y2;
    if ((current_pose.pose.position.x > min_x && current_pose.pose.position.x < max_x) &&  
        (current_pose.pose.position.y > min_y && current_pose.pose.position.y < max_y))
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "planner_server"),
        "IsGoalInForbiddenArea: True. Current_pose.pose.position.x: %f, current_pose.pose.position.y: %f, min_x: %f," 
        "max_x: %f, min_y: %f, max_y: %f !", current_pose.pose.position.x, current_pose.pose.position.y, 
        min_x, max_x, min_y, max_y); 
      return true;
    }
  }
  return false;
}

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  rotation_goal_search_sigh_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
  if (ROTATE_POINTS != nullptr)
  {
    std::string ratate_points_string(ROTATE_POINTS);
    makeRotationGoalFromString(ratate_points_string, rotate_vec);
    for (auto goal : rotate_vec)
    {
    RCLCPP_INFO(
      get_logger(), "Goal.x: %f, goal.y: %f !", goal.x, goal.y);
    }
  }
  if (FORBIDDEN_ROTATE_AREA_POINTS != nullptr)
  {
    std::string forbidden_area_points_string(FORBIDDEN_ROTATE_AREA_POINTS);
    makeForbiddenPointsFromString(forbidden_area_points_string, forbidden_area_vec);
    for (auto point : forbidden_area_vec)
    {
    RCLCPP_INFO(
      get_logger(), "Point.x1: %f, point.y1: %f , point.x2: %f, point.y2: %f!", 
      point.x1, point.y1, point.x2, point.y2);
    }
  }
}

PlannerServer::~PlannerServer()
{
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

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

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

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
  backgoals_publisher_ = create_publisher<nav_msgs::msg::Path>("back_goals", 1);

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
  backgoals_publisher_->on_activate();
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
  backgoals_publisher_->on_deactivate();
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
  backgoals_publisher_.reset();
  tf_.reset();
  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();
  costmap_ = nullptr;
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

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
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
    (void)action_server;
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

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  if (goal->goals.size() == 0)
  {
    action_server_poses_->succeeded_current();
    return;
  }
   

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
      action_server_poses_->succeeded_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (size_t i = 0; i < goal->goals.size(); ++i) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        // curr_start = filtered_goals[i - 1];
          if (concat_path.poses.size() > 0)
          {
            curr_start = concat_path.poses.back();
            curr_start.header = concat_path.header;
          }
          else
          {
            curr_start = start;
          }
        }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      auto plan_time = steady_clock_.now();
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);
      auto duration = steady_clock_.now() - plan_time;
      RCLCPP_WARN(
        get_logger(),
        "Plan time is %f",
        duration.seconds());


      // check path for validity
      if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id)) {
        // 若全局路径划失败跳过此点继续，不终止action
        // return;
        continue;
      }
      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
      double accumulate_distance = 0;
      if (concat_path.poses.size() > 10)
      {
        for (size_t pose_num = 1; pose_num < concat_path.poses.size(); ++pose_num)
        {
          double dx = concat_path.poses.at(pose_num).pose.position.x -  concat_path.poses.at(pose_num-1).pose.position.x;
          double dy = concat_path.poses.at(pose_num).pose.position.y -  concat_path.poses.at(pose_num-1).pose.position.y;
          accumulate_distance += sqrt(pow(dx, 2) + pow(dy, 2));
          if (accumulate_distance > 10)
          {
            break;
          }
        }
      }
      if (accumulate_distance > 10)
      {
        break;
      }
    }

    if (concat_path.poses.size() == 0)
    {
      action_server_poses_->succeeded_current();
      return;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

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

    auto plan_time = steady_clock_.now();
    result->path = getPlan(start, goal_pose, goal->planner_id);
    auto duration = steady_clock_.now() - plan_time;
      RCLCPP_WARN(
        get_logger(),
        "Plan time is %f",
        duration.seconds());

    if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    auto path = planners_[planner_id]->createPlan(start, goal);
    static rclcpp::Time start_time = steady_clock_.now();
    auto rotate_goal = goal;
    rotate_goal.header.stamp = get_clock()->now();
    for (size_t i = 20; i < 40 && i < path.poses.size(); i+=2)
    {
      auto pose = path.poses.at(i);
      // 搜索机器人是否要在狭窄走廊掉头
      geometry_msgs::msg::PoseStamped goal_pose_in_base_link, current_pose;
      nav2_util::getCurrentPose(current_pose, *tf_);
      nav2_util::transformPoseInTargetFrame(pose, goal_pose_in_base_link, *tf_, costmap_ros_->getBaseFrameID());
      unsigned int mx = 0;
      unsigned int my = 0;
      double min_y = -1.5, max_y = 1.5;
      double sin_th = sin(tf2::getYaw(current_pose.pose.orientation));
      double cos_th = cos(tf2::getYaw(current_pose.pose.orientation));
      std::vector<geometry_msgs::msg::Pose2D> left_line, right_line;
      if (goal_pose_in_base_link.pose.position.x < 0)
      {
        if (IsGoalInForbiddenArea(forbidden_area_vec, current_pose))
        {
          rotation_goal_search_sigh_ = true;
          start_time = steady_clock_.now();
          goto Search_ratate_goal;
        }
        for (double x = -0.5; x <= 1.0; x += 0.5)
        {
          for (double y = -0.05; y >= -1.5; y -= 0.05) {
            double g_x = current_pose.pose.position.x + x * cos_th - y * sin_th;
            double g_y = current_pose.pose.position.y + x * sin_th + y * cos_th;
            if (costmap_->worldToMap(g_x, g_y, mx, my) && costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE){
                    if (y > min_y)
                    {
                      min_y = y;
                      geometry_msgs::msg::Pose2D left_lethal_pose;
                      left_lethal_pose.x = g_x;
                      left_lethal_pose.y = g_y;
                      left_line.emplace_back(left_lethal_pose);
                      break;
                    }
            }
          }
        }
        for (double x = -0.5; x <= 1.0; x += 0.5)
        {
          for (double y = 0.05; y <= 1.5; y += 0.05) {
            double g_x = current_pose.pose.position.x + x * cos_th - y * sin_th;
            double g_y = current_pose.pose.position.y + x * sin_th + y * cos_th;
            if (costmap_->worldToMap(g_x, g_y, mx, my) && costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE){
                    if (y < max_y)
                    {
                      max_y = y;
                      geometry_msgs::msg::Pose2D right_lethal_pose;
                      right_lethal_pose.x = g_x;
                      right_lethal_pose.y = g_y;
                      right_line.emplace_back(right_lethal_pose);
                      break;
                    }
            }
          }
        }
        if (left_line.size() != 0 && right_line.size() != 0)
        {
          rotation_goal_search_sigh_ = true;
          start_time = steady_clock_.now();
        }
        else if ((steady_clock_.now() - start_time).seconds() > 5.0)
        {
          rotation_goal_search_sigh_ = false;
        }
        RCLCPP_INFO(
            get_logger(), "rotation_goal_search_sigh_: %d!", rotation_goal_search_sigh_);
        Search_ratate_goal:
        if (rotation_goal_search_sigh_)
        {
          RCLCPP_INFO(
            get_logger(), "Obstacle around robot, move towards rotate pose!");
          // 搜索最近的掉头点
          double search_distance = std::numeric_limits<double>::infinity();
          auto search_goal = rotate_goal;
          auto search_goal_in_base_link = rotate_goal;
          if (rotate_vec.size() > 0)
          {
            for (auto point : rotate_vec)
            {
              search_goal.pose.position.x = point.x;
              search_goal.pose.position.y = point.y;
              if (nav2_util::transformPoseInTargetFrame(search_goal, search_goal_in_base_link, *tf_, costmap_ros_->getBaseFrameID()) && 
                  search_goal_in_base_link.pose.position.x > 0)
              {
                double base_link_to_search_goal_distance = sqrt(pow(search_goal_in_base_link.pose.position.x, 2) + 
                                                                pow(search_goal_in_base_link.pose.position.y, 2));
                if (base_link_to_search_goal_distance < search_distance)
                {
                  search_distance = base_link_to_search_goal_distance;
                  rotate_goal.pose.position.x = point.x;
                  rotate_goal.pose.position.y = point.y;
                }
              }
              // RCLCPP_INFO(
              //   get_logger(), "Search_goal_in_base_link.x: %f !", search_goal_in_base_link.pose.position.x);
            }
          }
          if (nav2_util::transformPoseInTargetFrame(rotate_goal, search_goal_in_base_link, *tf_, costmap_ros_->getBaseFrameID()) && 
                  search_goal_in_base_link.pose.position.x < 0)
          {
            RCLCPP_ERROR(
            get_logger(), "Cannot find a suitable goal for robot to rotate ! Please assign a rotation point for this gallery !");
            return nav_msgs::msg::Path();
          }
          path = planners_[planner_id]->createPlan(start, rotate_goal);
          return path;
        }
      }
      //
    }
    return path;
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
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
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
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
