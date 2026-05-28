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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <functional>
#include <utility>

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace
{

constexpr double kSqrt2 = 1.4142135623730950488;

inline bool roiCellTraversable(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    return false;
  }
  if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  return true;
}

class ScopedHybridAStarLimits
{
public:
  explicit ScopedHybridAStarLimits(
    AStarAlgorithm<NodeHybrid> * astar,
    int baseline_iters,
    double baseline_time)
  : astar_(astar), baseline_iters_(baseline_iters), baseline_time_(baseline_time) {}

  ~ScopedHybridAStarLimits()
  {
    if (astar_) {
      astar_->setPlanningLimits(baseline_iters_, baseline_time_);
    }
  }

  void overrideLimits(int iters, double time_s)
  {
    if (astar_) {
      astar_->setPlanningLimits(iters, time_s);
    }
  }

  ScopedHybridAStarLimits(const ScopedHybridAStarLimits &) = delete;
  ScopedHybridAStarLimits & operator=(const ScopedHybridAStarLimits &) = delete;

private:
  AStarAlgorithm<NodeHybrid> * astar_;
  int baseline_iters_;
  double baseline_time_;
};

/**
 * Dijkstra on an 8-connected ROI: orthogonal edge cost = resolution (m),
 * diagonal = sqrt(2)*resolution. Diagonal steps require both axial bridge cells free.
 * @param out_cell_path_if_connected if non-null, on success cleared and filled map cells start->goal
 * @return true iff goal reached within ROI; then *out_shortest_meters is path length.
 */
bool roi8GridShortestPathMeters(
  nav2_costmap_2d::Costmap2D * costmap,
  const unsigned int mx_s, const unsigned int my_s,
  const unsigned int mx_g, const unsigned int my_g,
  const double roi_margin_m,
  const bool allow_unknown,
  double * out_shortest_meters,
  std::vector<std::pair<unsigned int, unsigned int>> * out_cell_path_if_connected = nullptr)
{
  if (!out_shortest_meters) {
    return false;
  }

  const double res = static_cast<double>(costmap->getResolution());
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  if (size_x < 1 || size_y < 1) {
    return false;
  }

  const auto cell_ok = [&](const unsigned int mx, const unsigned int my) -> bool {
    return roiCellTraversable(costmap->getCost(mx, my), allow_unknown);
  };

  if (!cell_ok(mx_s, my_s) || !cell_ok(mx_g, my_g)) {
    return false;
  }

  const int imx_s = static_cast<int>(mx_s);
  const int imy_s = static_cast<int>(my_s);
  const int imx_g = static_cast<int>(mx_g);
  const int imy_g = static_cast<int>(my_g);

  const int margin_cells = std::max(1, static_cast<int>(std::ceil(roi_margin_m / res)));
  int rx0 = std::min(imx_s, imx_g) - margin_cells;
  int ry0 = std::min(imy_s, imy_g) - margin_cells;
  int rx1 = std::max(imx_s, imx_g) + margin_cells;
  int ry1 = std::max(imy_s, imy_g) + margin_cells;

  rx0 = std::max(rx0, 0);
  ry0 = std::max(ry0, 0);
  rx1 = std::min(rx1, size_x - 1);
  ry1 = std::min(ry1, size_y - 1);

  if (rx0 > rx1 || ry0 > ry1) {
    return false;
  }

  const int roi_w = rx1 - rx0 + 1;
  const int roi_h = ry1 - ry0 + 1;

  const int lsx = imx_s - rx0;
  const int lsy = imy_s - ry0;
  const int lgx = imx_g - rx0;
  const int lgy = imy_g - ry0;
  if (
    lsx < 0 || lsy < 0 || lsx >= roi_w || lsy >= roi_h ||
    lgx < 0 || lgy < 0 || lgx >= roi_w || lgy >= roi_h)
  {
    return false;
  }

  const std::size_t roi_cells = static_cast<std::size_t>(roi_w) * static_cast<std::size_t>(roi_h);
  const std::size_t start_lin = static_cast<std::size_t>(lsx + lsy * roi_w);
  const std::size_t goal_lin = static_cast<std::size_t>(lgx + lgy * roi_w);

  if (start_lin == goal_lin) {
    *out_shortest_meters = 0.0;
    if (out_cell_path_if_connected) {
      out_cell_path_if_connected->clear();
      out_cell_path_if_connected->emplace_back(mx_s, my_s);
    }
    return true;
  }

  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  using QElem = std::pair<double, std::size_t>;
  std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> pq;

  constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
  std::vector<double> dist(roi_cells, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parents(roi_cells, kNoParent);
  dist[start_lin] = 0.0;
  parents[start_lin] = start_lin;
  pq.emplace(0.0, start_lin);

  while (!pq.empty()) {
    const QElem top = pq.top();
    pq.pop();
    const double d_here = top.first;
    const std::size_t cur = top.second;
    if (d_here > dist[cur]) {
      continue;
    }
    if (cur == goal_lin) {
      *out_shortest_meters = d_here;
      if (out_cell_path_if_connected) {
        out_cell_path_if_connected->clear();
        std::vector<std::size_t> backwards;
        for (std::size_t at = cur; /**/; /**/) {
          backwards.push_back(at);
          if (at == start_lin) {
            break;
          }
          const std::size_t p = parents[at];
          if (p == kNoParent) {
            break;  // should not happen
          }
          at = p;
        }
        const std::size_t n_pts = backwards.size();
        out_cell_path_if_connected->reserve(n_pts);
        for (std::size_t ii = n_pts; ii-- > 0; ) {
          const std::size_t lin_idx = backwards[ii];
          const int ply = static_cast<int>(lin_idx) / roi_w;
          const int plx = static_cast<int>(lin_idx) - ply * roi_w;
          const unsigned int gmx = static_cast<unsigned int>(rx0 + plx);
          const unsigned int gmy = static_cast<unsigned int>(ry0 + ply);
          out_cell_path_if_connected->emplace_back(gmx, gmy);
        }
      }
      return true;
    }

    const int ly = static_cast<int>(cur) / roi_w;
    const int lx = static_cast<int>(cur) - ly * roi_w;
    const int gx_cell = rx0 + lx;
    const int gy_cell = ry0 + ly;

    for (int k = 0; k < 8; ++k) {
      const int ngx_i = gx_cell + kDx[k];
      const int ngy_i = gy_cell + kDy[k];
      if (ngx_i < rx0 || ngx_i > rx1 || ngy_i < ry0 || ngy_i > ry1) {
        continue;
      }
      const unsigned int ngx = static_cast<unsigned int>(ngx_i);
      const unsigned int ngy = static_cast<unsigned int>(ngy_i);
      if (!cell_ok(ngx, ngy)) {
        continue;
      }

      const int adx = kDx[k];
      const int ady = kDy[k];
      if (adx != 0 && ady != 0) {
        if (
          !cell_ok(static_cast<unsigned int>(gx_cell + adx), static_cast<unsigned int>(gy_cell)) ||
          !cell_ok(static_cast<unsigned int>(gx_cell), static_cast<unsigned int>(gy_cell + ady)))
        {
          continue;
        }
      }

      const double step = (adx != 0 && ady != 0) ? (res * kSqrt2) : res;
      const int nlx = ngx_i - rx0;
      const int nly = ngy_i - ry0;
      const std::size_t nidx = static_cast<std::size_t>(nlx + nly * roi_w);
      const double nd = d_here + step;
      if (nd < dist[nidx]) {
        dist[nidx] = nd;
        parents[nidx] = cur;
        pq.emplace(nd, nidx);
      }
    }
  }

  return false;
}

nav_msgs::msg::Path roiCellPathToNavPath(
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<std::pair<unsigned int, unsigned int>> & cells,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;

  geometry_msgs::msg::PoseStamped ps;
  ps.header = path.header;

  const std::size_t n = cells.size();
  if (n == 0) {
    return path;
  }

  path.poses.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    double wx = 0.0, wy = 0.0;
    costmap->mapToWorld(cells[i].first, cells[i].second, wx, wy);
    double yaw = 0.0;
    if (i + 1 < n) {
      double nx = 0.0, ny = 0.0;
      costmap->mapToWorld(cells[i + 1].first, cells[i + 1].second, nx, ny);
      yaw = std::atan2(ny - wy, nx - wx);
    } else if (i > 0) {
      double px = 0.0, py = 0.0;
      costmap->mapToWorld(cells[i - 1].first, cells[i - 1].second, px, py);
      yaw = std::atan2(wy - py, wx - px);
    }
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation = getWorldOrientation(static_cast<float>(yaw));
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  footprint_back_x_ = 0.0;
  footprint_front_x_ = 0.0;
  _footprint_extend_back_x = 0.0;
  _footprint_extend_front_x = 0.0;
  _footprint_extend_y = 0.0;
  _costmap_resulution = 0.05;

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".goal_occupied_tolerance", rclcpp::ParameterValue(0.5));
  _goal_occupied_tolerance = static_cast<float>(node->get_parameter(name + ".goal_occupied_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".goal_search_resolution", rclcpp::ParameterValue(0.1));
  _goal_search_resolution = static_cast<float>(node->get_parameter(name + ".goal_search_resolution").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".goal_close_to_obstacle_distance", rclcpp::ParameterValue(0.3));
  _goal_close_to_obstacle_distance = static_cast<float>(node->get_parameter(name + ".goal_close_to_obstacle_distance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".footprint_extend_back_x", rclcpp::ParameterValue(0.0));
  _footprint_extend_back_x = static_cast<float>(node->get_parameter(name + ".footprint_extend_back_x").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".footprint_extend_front_x", rclcpp::ParameterValue(0.0));
  _footprint_extend_front_x = static_cast<float>(node->get_parameter(name + ".footprint_extend_front_x").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".footprint_extend_y", rclcpp::ParameterValue(0.0));
  _footprint_extend_y = static_cast<float>(node->get_parameter(name + ".footprint_extend_y").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_close_range_roi_budget", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".enable_close_range_roi_budget", _enable_close_range_roi_budget);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".close_range_roi_margin", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".close_range_roi_margin", _close_range_roi_margin);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".close_range_threshold", rclcpp::ParameterValue(25.0));
  node->get_parameter(name + ".close_range_threshold", _close_range_threshold_m);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".close_range_max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".close_range_max_planning_time", _close_range_max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".close_range_max_iterations", rclcpp::ParameterValue(250000));
  node->get_parameter(name + ".close_range_max_iterations", _close_range_max_iterations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
  _motion_model = fromString(_motion_model_for_search);
  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);
  _roi_connectivity_path_publisher =
    node->create_publisher<nav_msgs::msg::Path>("roi_connectivity_path", 1);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  _roi_connectivity_path_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  _roi_connectivity_path_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
  _roi_connectivity_path_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  auto goal_with_tolerance = goal;
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  nav2_costmap_2d::Footprint check_footprint = _costmap_ros->getRobotFootprint();
  for (auto &point : check_footprint)
  {
    for (auto footprint : check_footprint)
    {
            footprint_back_x_ = footprint.x < footprint_back_x_ ? footprint.x : footprint_back_x_;
            footprint_front_x_ = footprint.x > footprint_front_x_ ? footprint.x : footprint_front_x_;
    }
  }
  _costmap_resulution = costmap->getResolution();

  geometry_msgs::msg::Pose2D start_pose2d;
  start_pose2d.x = start.pose.position.x;
  start_pose2d.y = start.pose.position.y;
  start_pose2d.theta = tf2::getYaw(start.pose.orientation);
  double start_footprint_cost = 0.0; 
  start_footprint_cost = _collision_checker.footprintCostAtPose(start_pose2d.x, start_pose2d.y, start_pose2d.theta, _costmap_ros->getRobotFootprint());
  if (start_footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE) 
  {
    RCLCPP_WARN(_logger, "Start pose: (%f, %f) is occupied !", start.pose.position.x, start.pose.position.y);
    throw std::runtime_error("Cannot generate a plan, start is occupied!");
  }

  try
  {
    bool is_goal_free = is_free(goal_with_tolerance, costmap, _footprint_extend_back_x, _footprint_extend_front_x, _footprint_extend_y);
    if (!is_goal_free) {
      steady_clock::time_point start = steady_clock::now();
      double goal_search_x = - _goal_occupied_tolerance;
      
      double min_dist = 1.5;
      while (goal_search_x < _goal_occupied_tolerance + 0.1)
      {
        double goal_search_y = - _goal_occupied_tolerance;
        while (goal_search_y < _goal_occupied_tolerance + 0.1)
        {
          double dist = sqrt(pow(goal_search_x, 2) + pow(goal_search_y, 2));
          if (dist > min_dist)
          {
            goal_search_y += _goal_search_resolution;
            continue;
          }
          auto search_goal = goal;
          search_goal.pose.position.x += goal_search_x;
          search_goal.pose.position.y += goal_search_y;
          is_goal_free = is_free(search_goal, costmap, _footprint_extend_back_x, _footprint_extend_front_x, _footprint_extend_y);
          if (is_goal_free)
          {
            double dist = sqrt(pow(goal_search_x, 2) + pow(goal_search_y, 2));
            if (dist < min_dist)
            {
              goal_with_tolerance = search_goal;
              min_dist = dist;
            }
          }
          goal_search_y += _goal_search_resolution;
        }
        goal_search_x += _goal_search_resolution;
      }
      if (goal.pose == goal_with_tolerance.pose)
      {
        RCLCPP_WARN(_logger, "Goal.x: %f, goal.y: %f is occupied !", goal.pose.position.x, goal.pose.position.y);
        throw std::runtime_error("Cannot generate a plan, goal is occupied!");
      }
      steady_clock::time_point end = steady_clock::now();
      duration<double> time_span = duration_cast<duration<double>>(end - start);
      RCLCPP_INFO(_logger, "Duration time in adjust path is : %f!", static_cast<double>(time_span.count()) * 1000);
      RCLCPP_INFO(_logger, "goal.x: %f, goal.y: %f !", goal.pose.position.x, goal.pose.position.y);
      RCLCPP_INFO(_logger, "goal_with_tolerance.x: %f, goal_with_tolerance.y: %f !", goal_with_tolerance.pose.position.x, goal_with_tolerance.pose.position.y);
  }
  }
  catch (const nav2_costmap_2d::IllegalPoseException & e) {
    RCLCPP_ERROR(_logger, "%s", e.what());
  } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
    RCLCPP_ERROR(_logger, "%s", e.what());
  }  
  catch (const std::runtime_error & e) {
    RCLCPP_ERROR(_logger, "%s", e.what());
  } 
  catch (...) {
    RCLCPP_ERROR(_logger, "Failed to check pose score!");
  }

  geometry_msgs::msg::Pose2D goal_pose2d;
  goal_pose2d.x = goal_with_tolerance.pose.position.x;
  goal_pose2d.y = goal_with_tolerance.pose.position.y;
  goal_pose2d.theta = tf2::getYaw(goal_with_tolerance.pose.orientation);

  double yaw = atan2(goal_pose2d.y-start_pose2d.y, goal_pose2d.x-start_pose2d.x);
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);

  // Setup message
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();
  
  double goal_footprint_cost = 0.0; 

  // 检查从起点方向旋转至终点方向是否会碰撞，如果会碰撞，跳过生成直线路径
  double start_theta = tf2::getYaw(start.pose.orientation);
  double goal_theta = tf2::getYaw(goal_with_tolerance.pose.orientation);

  double diff_theta = goal_theta - start_theta;
  diff_theta = std::fmod(diff_theta, 2 * M_PI);            // 取模到 (-2π, 2π)
  if (diff_theta > M_PI)
  {
    diff_theta -= 2 * M_PI;
  } else if(diff_theta < -M_PI)
  {
    diff_theta += 2 * M_PI;
  }
  double step = diff_theta < 0 ? -0.087 : 0.087;  // 设置步长为5度

  for (size_t i = 1 ; i < floor(fabs(diff_theta / step)); ++i) {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = start.pose.position.x;
    pose2d.y = start.pose.position.y;
    if (start_theta + step * i > M_PI) {
      pose2d.theta = start_theta + step * i - 2 * M_PI;  // 将角度限制在 (-π, π)
    } else if (start_theta + step * i < -M_PI) {
      pose2d.theta = start_theta + step * i + 2 * M_PI;  // 将角度限制在 (-π, π)
    } else {
      pose2d.theta = start_theta + step * i;
    }
    double footprint_cost = 0.0;
    footprint_cost = _collision_checker.footprintCostAtPose(pose2d.x, pose2d.y, pose2d.theta, check_footprint);
    if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(_logger, "Rotation to goal is occupied, jump to A* search!");
      goto start_a_star;
    }
  }
  
  goal_footprint_cost = _collision_checker.footprintCostAtPose(goal_pose2d.x, goal_pose2d.y, yaw, check_footprint);
  if (goal_footprint_cost != nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    try
    {
      bool is_path_free = true;
      double distance_start_to_goal = nav2_util::geometry_utils::euclidean_distance(start_pose2d, goal_pose2d);
      for (double d = _costmap_resulution; d < distance_start_to_goal + _costmap_resulution; d += _costmap_resulution)
      {
            geometry_msgs::msg::Pose2D path_pose;
            path_pose.theta = yaw;
            find_pose(start_pose2d, goal_pose2d, d, path_pose);
            geometry_msgs::msg::PoseStamped path_posestamped;
            path_posestamped.header.frame_id = _global_frame;
            path_posestamped.header.stamp = _clock->now();
            path_posestamped.pose.position.x = path_pose.x;
            path_posestamped.pose.position.y = path_pose.y;
            path_posestamped.pose.orientation = getWorldOrientation(path_pose.theta);
            if (d <= _costmap_resulution)
            {
              is_path_free = is_free(path_posestamped, costmap, _footprint_extend_back_x, -footprint_front_x_, _footprint_extend_y);
            }
            else if (d >= distance_start_to_goal - _costmap_resulution)
            {
              is_path_free = is_free(path_posestamped, costmap, -footprint_back_x_, -_footprint_extend_front_x, _footprint_extend_y);
            }
            else
            {
              is_path_free = is_free(path_posestamped, costmap, -footprint_back_x_, -footprint_front_x_, _footprint_extend_y);
            }
            if (is_path_free && d < distance_start_to_goal - _costmap_resulution)
            {
              pose.pose.position.x = path_pose.x;
              pose.pose.position.y = path_pose.y;
              plan.poses.emplace_back(pose);
            }
            else if (!is_path_free)
            {
              plan.poses.clear();
              break;
            }
      }
      if (is_path_free)
      {
        pose.pose = goal_with_tolerance.pose;
        plan.poses.emplace_back(pose);
        return plan;
      }
    }
    catch (const nav2_costmap_2d::IllegalPoseException & e) {
      RCLCPP_ERROR(_logger, "%s", e.what());
    } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
      RCLCPP_ERROR(_logger, "%s", e.what());
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(_logger, "%s", e.what());
    } catch (...) {
      RCLCPP_ERROR(_logger, "Failed to check pose score!");
    }
  }
  
  start_a_star:
  ScopedHybridAStarLimits limits_scope(_a_star.get(), _max_iterations, _max_planning_time);
  double smooth_budget_ceiling = _max_planning_time;

  unsigned int mx_s, my_s;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_s, my_s)) {
    throw std::runtime_error("Start pose is out of costmap!");
  }
  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));

  unsigned int mx_g, my_g;
  if (!costmap->worldToMap(
      goal_with_tolerance.pose.position.x, goal_with_tolerance.pose.position.y, mx_g, my_g))
  {
    throw std::runtime_error("Goal pose is out of costmap!");
  }
  double goal_orientation_bin = tf2::getYaw(goal_with_tolerance.pose.orientation) / _angle_bin_size;
  while (goal_orientation_bin < 0.0) {
    goal_orientation_bin += static_cast<float>(_angle_quantizations);
  }
  if (goal_orientation_bin >= static_cast<float>(_angle_quantizations)) {
    goal_orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int goal_orientation_bin_id = static_cast<unsigned int>(floor(goal_orientation_bin));

  const bool viz_subs = static_cast<bool>(
    _roi_connectivity_path_publisher &&
    _roi_connectivity_path_publisher->get_subscription_count() > 0);
  const bool run_roi_grid = _enable_close_range_roi_budget || viz_subs;
  std::vector<std::pair<unsigned int, unsigned int>> roi_cell_path;

  if (run_roi_grid) {
    const double start_goal_dist_m =
      nav2_util::geometry_utils::euclidean_distance(start_pose2d, goal_pose2d);
    if (start_goal_dist_m < _close_range_threshold_m) {
      double roi_path_m = std::numeric_limits<double>::infinity();
      const bool roi_connected = roi8GridShortestPathMeters(
        costmap,
        mx_s, my_s, mx_g, my_g,
        _close_range_roi_margin,
        _allow_unknown,
        &roi_path_m,
        viz_subs ? &roi_cell_path : nullptr);

      if (_enable_close_range_roi_budget) {
        const bool apply_close_budget =
          roi_connected && std::isfinite(roi_path_m) && roi_path_m < _close_range_threshold_m;

        if (apply_close_budget) {
          int close_iters =
            (_close_range_max_iterations <= 0) ? std::numeric_limits<int>::max() :
            _close_range_max_iterations;
          limits_scope.overrideLimits(close_iters, _close_range_max_planning_time);
          smooth_budget_ceiling = _close_range_max_planning_time;
        }
      }

      if (viz_subs && roi_connected && !roi_cell_path.empty()) {
        auto roi_path_msg =
          roiCellPathToNavPath(costmap, roi_cell_path, _global_frame, _clock->now());
        _roi_connectivity_path_publisher->publish(roi_path_msg);
      }
    }
  }

  _a_star->setStart(mx_s, my_s, orientation_bin_id);
  _a_star->setGoal(mx_g, my_g, goal_orientation_bin_id);

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found from (") + std::to_string(start_pose2d.x) + "," + std::to_string(start_pose2d.y) + ") to (" +
                std::to_string(goal_pose2d.x) + "," + std::to_string(goal_pose2d.y) + ")";
      } else {
        error = std::string("exceeded maximum iterations from (") + std::to_string(start_pose2d.x) + "," + std::to_string(start_pose2d.y) + ") to (" +
                std::to_string(goal_pose2d.x) + "," + std::to_string(goal_pose2d.y) + ")";
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return plan;
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Tail safety check for A* result:
  // Walk backwards from the end of the path and drop colliding poses,
  // keeping the last pose that is free. Here, when checking for collision
  // we only treat LETHAL_OBSTACLE as collision and allow INSCRIBED_INFLATED_OBSTACLE.
  if (!plan.poses.empty()) {
    int last_valid_idx = static_cast<int>(plan.poses.size()) - 1;
    bool found_valid = false;

    for (int i = last_valid_idx; i >= 0; --i) {
      if (is_free(plan.poses[i], costmap,
                  _footprint_extend_back_x,
                  _footprint_extend_front_x,
                  _footprint_extend_y,
                  /*ignore_inscribed=*/true))
      {
        last_valid_idx = i;
        found_valid = true;
        break;
      }
    }

    if (!found_valid) {
      // Path end is completely invalid – return empty plan so that higher-level logic can react.
      RCLCPP_WARN(
        _logger,
        "%s: A* produced a path whose end poses are all in collision (LETHAL); returning empty plan.",
        _name.c_str());
      plan.poses.clear();
      return plan;
    }

    if (last_valid_idx < static_cast<int>(plan.poses.size()) - 1) {
      // Trim off the colliding tail so that the final pose is guaranteed to be LETHAL-free.
      const std::size_t trimmed_count =
        static_cast<std::size_t>(plan.poses.size()) - static_cast<std::size_t>(last_valid_idx + 1);
      plan.poses.erase(plan.poses.begin() + last_valid_idx + 1, plan.poses.end());
      RCLCPP_WARN(
        _logger,
        "%s: Trimmed %zu tail pose(s) from A* path due to LETHAL collisions at the endpoint.",
        _name.c_str(), trimmed_count);
    }
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = smooth_budget_ceiling - static_cast<double>(time_span.count());
  if (time_remaining < 0.0) {
    time_remaining = 0.0;
  }

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

bool SmacPlannerHybrid::find_pose(geometry_msgs::msg::Pose2D original_pose, geometry_msgs::msg::Pose2D edge_pose, double d, geometry_msgs::msg::Pose2D &output_pose) 
{
    // 计算法向量的单位向量
    double vx = edge_pose.x - original_pose.x;
    double vy = edge_pose.y - original_pose.y;
    double param_x, param_y;
    calculate_line_param(param_x, param_y, vx, vy);

    // 计算从点P出发沿法向量方向的点
    double Qx1 = original_pose.x + param_x * d;
    double Qy1 = original_pose.y + param_y * d;

    output_pose.x = Qx1;
    output_pose.y = Qy1;
    RCLCPP_DEBUG(_logger, "original_pose x: %f, y: %f!", original_pose.x, original_pose.y);
    RCLCPP_DEBUG(_logger, "edge_pose x: %f, y: %f!", edge_pose.x, edge_pose.y);
    RCLCPP_DEBUG(_logger, "output_pose x: %f, y: %f!", output_pose.x, output_pose.y);

    return true;
}

void SmacPlannerHybrid::calculate_line_param(double &x, double &y, double vx, double vy)
{
        if (fabs(vx) < 1e-5 && fabs(vy) < 1e-5)
        {
                x = 0;
                y = 0;
                return;
        }
        double magnitude = sqrt(vx * vx + vy * vy);
        x = vx / magnitude;
        y = vy / magnitude;
        return;
}

bool SmacPlannerHybrid::is_free(const geometry_msgs::msg::PoseStamped &pose,
                                nav2_costmap_2d::Costmap2D * costmap,
                                double footprint_extend_back_x,
                                double footprint_extend_front_x,
                                double footprint_extend_y,
                                bool ignore_inscribed)
{
        try
        {
                geometry_msgs::msg::Pose2D pose2d;
                pose2d.x = pose.pose.position.x;
                pose2d.y = pose.pose.position.y;
                pose2d.theta = tf2::getYaw(pose.pose.orientation);
                double cos_th = cos(pose2d.theta);
                double sin_th = sin(pose2d.theta);
                std::vector<double> footprint_extend{0};
                if (footprint_extend_y != 0.0)
                {
                  footprint_extend.emplace_back(-footprint_extend_y);
                  footprint_extend.emplace_back(footprint_extend_y);
                }
                unsigned char footprint_cost = nav2_costmap_2d::FREE_SPACE;
                for(auto y : footprint_extend)
                {
                  for (double x = footprint_back_x_ + footprint_extend_back_x; x <= footprint_front_x_ + footprint_extend_front_x; x += _costmap_resulution) 
                  {
                          unsigned int map_x,map_y;
                          double g_x = pose2d.x + x * cos_th - y * sin_th;
                          double g_y = pose2d.y + x * sin_th + y * cos_th;
                          costmap->worldToMap(g_x, g_y, map_x, map_y);
                          footprint_cost = costmap->getCost(map_x, map_y);
                          // Default behavior: treat both LETHAL and INSCRIBED as collision.
                          // When ignore_inscribed == true (used for A* tail trimming), only treat LETHAL as collision.
                          if (footprint_cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
                              (!ignore_inscribed && footprint_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
                          {
                                  RCLCPP_DEBUG(_logger, "Footprint at (%f, %f) is occupied!", g_x, g_y);
                                  return false;
                          }
                  }
                  if (footprint_cost == nav2_costmap_2d::FREE_SPACE)
                  {
                          return true;
                  }
                }
                
        } catch (const nav2_costmap_2d::IllegalPoseException & e) {
                RCLCPP_ERROR(_logger, "%s", e.what());
                return true;
        } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
                RCLCPP_ERROR(_logger, "%s", e.what());
                return true;
        } catch (const std::runtime_error & e) {
                RCLCPP_ERROR(_logger, "%s", e.what());
                return true;
        } catch (...) {
                RCLCPP_ERROR(_logger, "Failed to check pose score!");
                return true;
        }
                return true;
}

rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".close_range_roi_margin") {
        _close_range_roi_margin = parameter.as_double();
      } else if (name == _name + ".close_range_threshold") {
        _close_range_threshold_m = parameter.as_double();
      } else if (name == _name + ".close_range_max_planning_time") {
        _close_range_max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".enable_close_range_roi_budget") {
        _enable_close_range_roi_budget = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".close_range_max_iterations") {
        _close_range_max_iterations = parameter.as_int();
      } else if (name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        _motion_model = fromString(parameter.as_string());
        if (_motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
            _motion_model_for_search.c_str());
        }
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    auto node = _node.lock();

    // Re-Initialize A* template
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
      }
    }

    // Re-Initialize collision checker
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_minimum_turning_radius_global_coords);
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
