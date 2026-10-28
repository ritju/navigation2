// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_hybrid.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
LookupTable NodeHybrid::obstacle_heuristic_lookup_table;
double NodeHybrid::travel_distance_cost = sqrt(2);
HybridMotionTable NodeHybrid::motion_table;
float NodeHybrid::size_lookup = 25;
LookupTable NodeHybrid::dist_heuristic_lookup_table;
nav2_costmap_2d::Costmap2D * NodeHybrid::sampled_costmap = nullptr;
CostmapDownsampler NodeHybrid::downsampler;
ObstacleHeuristicQueue NodeHybrid::obstacle_heuristic_queue;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/node821.html
// Model for ackermann style vehicle with minimum radius restriction
void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  unsigned int heading_bins = num_angle_quantization_in;
  if (search_info.use_curvature_state && search_info.heading_quantization > 0) {
    heading_bins = search_info.heading_quantization;
  }

  // if nothing changed, no need to re-compute primitives
  if (heading_bins == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    syncCurvature(search_info, false);
    return;
  }

  num_angle_quantization = heading_bins;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;

  // angle must meet 3 requirements:
  // 1) be increment of quantized bin size
  // 2) chord length must be greater than sqrt(2) to leave current cell
  // 3) maximum curvature must be respected, represented by minimum turning angle
  // Thusly:
  // On circle of radius minimum turning angle, we need select motion primatives
  // with chord length > sqrt(2) and be an increment of our bin size
  //
  // chord >= sqrt(2) >= 2 * R * sin (angle / 2); where angle / N = quantized bin size
  // Thusly: angle <= 2.0 * asin(sqrt(2) / (2 * R))
  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  // Now make sure angle is an increment of the quantized bin size
  // And since its based on the minimum chord, we need to make sure its always larger
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    // Search dimensions are clean multiples of quantization - this prevents
    // paths with loops in them
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  // find deflections
  // If we make a right triangle out of the chord in circle of radius
  // min turning angle, we can see that delta X = R * sin (angle)
  float delta_x = min_turning_radius * sin(angle);
  // Using that same right triangle, we can see that the complement
  // to delta Y is R * cos (angle). If we subtract R, we get the actual value
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(3);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
  syncCurvature(search_info, false);
}

void HybridMotionTable::syncCurvature(const SearchInfo & search_info, bool allow_reverse)
{
  footprint_origin_x = search_info.footprint_origin_x;
  footprint_origin_y = search_info.footprint_origin_y;
  if (!search_info.use_curvature_state) {
    curvature_enabled = false;
    index_dim3 = num_angle_quantization == 0 ? 1 : num_angle_quantization;
    curvature.enabled = false;
    return;
  }
  CurvatureLimits limits;
  limits.wheelbase = search_info.wheelbase;
  limits.max_angle = search_info.max_steering_angle;
  limits.max_rate = search_info.max_steering_rate;
  limits.max_accel = search_info.max_steering_accel;
  limits.reference_speed = search_info.reference_speed;
  limits.primitive_length = search_info.primitive_length;
  limits.resolution = search_info.costmap_resolution;
  buildCurvatureModel(limits, allow_reverse, curvature);
  curvature_enabled = true;
  index_dim3 = num_angle_quantization * static_cast<unsigned int>(curvature.bins.angles.size());
}

// http://planning.cs.uiuc.edu/node822.html
// Same as Dubin model but now reverse is valid
// See notes in Dubin for explanation
void HybridMotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  unsigned int heading_bins = num_angle_quantization_in;
  if (search_info.use_curvature_state && search_info.heading_quantization > 0) {
    heading_bins = search_info.heading_quantization;
  }

  // if nothing changed, no need to re-compute primitives
  if (heading_bins == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::REEDS_SHEPP)
  {
    syncCurvature(search_info, true);
    return;
  }

  num_angle_quantization = heading_bins;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::REEDS_SHEPP;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  float delta_x = min_turning_radius * sin(angle);
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Forward + Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Forward + Right
  projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0);  // Backward
  projections.emplace_back(-delta_x, delta_y, -increments);  // Backward + Left
  projections.emplace_back(-delta_x, -delta_y, increments);  // Backward + Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
  syncCurvature(search_info, true);
}

MotionPoses HybridMotionTable::getProjections(const NodeHybrid * node)
{
  MotionPoses projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose & motion_model = projections[i];

    // normalize theta, I know its overkill, but I've been burned before...
    const float & node_heading = node->pose.theta;
    float new_heading = node_heading + motion_model._theta;

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(
      delta_xs[i][node_heading] + node->pose.x,
      delta_ys[i][node_heading] + node->pose.y,
      new_heading);
  }

  return projection_list;
}

unsigned int HybridMotionTable::getClosestAngularBin(const double & theta)
{
  return static_cast<unsigned int>(floor(theta / static_cast<double>(bin_size))) %
         num_angle_quantization;
}

float HybridMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return bin_idx * bin_size;
}

NodeHybrid::NodeHybrid(const uint64_t index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max())
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
  cont_x = 0.f;
  cont_y = 0.f;
  cont_yaw = 0.f;
  cont_delta = 0.f;
  kappa_bin = 0;
  steer_trend = 0;
  steer_settled = true;
  has_staged = false;
  edge_samples.clear();
  connector_samples.clear();
  staged_samples.clear();
}

bool NodeHybrid::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  float theta_bin = this->pose.theta;
  if (motion_table.curvature_enabled && motion_table.num_angle_quantization > 0) {
    int heading = static_cast<int>(std::lround(theta_bin));
    if (heading < 0) {
      heading = 0;
    }
    theta_bin = static_cast<float>(
      static_cast<unsigned int>(heading) % motion_table.num_angle_quantization);
  }
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, theta_bin, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float NodeHybrid::getTraversalCost(const NodePtr & child)
{
  if (motion_table.curvature_enabled) {
    return curvatureTraversalCost(child);
  }
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }

  // this is the first node
  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
    return NodeHybrid::travel_distance_cost;
  }

  float travel_cost = 0.0;
  float travel_cost_raw =
    NodeHybrid::travel_distance_cost *
    (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost);

  if (child->getMotionPrimitiveIndex() == 0 || child->getMotionPrimitiveIndex() == 3) {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (getMotionPrimitiveIndex() == child->getMotionPrimitiveIndex()) {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (child->getMotionPrimitiveIndex() > 2) {
    // reverse direction
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const nav2_costmap_2d::Costmap2D * /*costmap*/)
{
  Coordinates node = node_coords;
  Coordinates goal = goal_coords;
  if (motion_table.curvature_enabled && motion_table.num_angle_quantization > 0) {
    const auto unpack = [](float theta) {
        int heading = static_cast<int>(std::lround(theta));
        if (heading < 0) {
          heading = 0;
        }
        return static_cast<float>(
          static_cast<unsigned int>(heading) % motion_table.num_angle_quantization);
      };
    node.theta = unpack(node.theta);
    goal.theta = unpack(goal.theta);
  }
  const float obstacle_heuristic =
    getObstacleHeuristic(node, goal, motion_table.cost_penalty);
  float dist_heuristic = getDistanceHeuristic(node, goal, obstacle_heuristic);
  float heuristic = std::max(obstacle_heuristic, dist_heuristic);
  if (motion_table.curvature_enabled && motion_table.travel_distance_reward < 1.f) {
    heuristic *= motion_table.travel_distance_reward;
  }
  return heuristic;
}

void NodeHybrid::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & size_y,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{
  // find the motion model selected
  switch (motion_model) {
    case MotionModel::DUBIN:
      motion_table.initDubin(size_x, size_y, num_angle_quantization, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      motion_table.initReedsShepp(size_x, size_y, num_angle_quantization, search_info);
      break;
    default:
      throw std::runtime_error(
              "Invalid motion model for Hybrid A*. Please select between"
              " Dubin (Ackermann forward only),"
              " Reeds-Shepp (Ackermann forward and back).");
  }

  travel_distance_cost = motion_table.projections[0]._x;
}

inline float distanceHeuristic2D(
  const unsigned int idx, const unsigned int size_x,
  const unsigned int target_x, const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::sqrt(dx * dx + dy * dy);
}

void NodeHybrid::resetObstacleHeuristic(
  nav2_costmap_2d::Costmap2D * costmap,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic. This speeds up
  // the planner considerably to search through 75% less cells with no detectable
  // erosion of path quality after even modest smoothing. The error would be no more
  // than 0.05 * normalized cost. Since this is just a search prior, there's no loss in generality
  std::weak_ptr<nav2_util::LifecycleNode> ptr;
  downsampler.on_configure(ptr, "fake_frame", "fake_topic", costmap, 2.0, true);
  downsampler.on_activate();
  sampled_costmap = downsampler.downsample(2.0);

  // Clear lookup table
  unsigned int size = sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY();
  if (obstacle_heuristic_lookup_table.size() == size) {
    // must reset all values
    std::fill(
      obstacle_heuristic_lookup_table.begin(),
      obstacle_heuristic_lookup_table.end(), 0.0);
  } else {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0);
    // must reset values for non-constructed indices
    std::fill_n(
      obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0);
  }

  obstacle_heuristic_queue.clear();
  obstacle_heuristic_queue.reserve(
    sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY());

  // Set initial goal point to queue from. Divided by 2 due to downsampled costmap.
  const unsigned int size_x = sampled_costmap->getSizeInCellsX();
  const unsigned int goal_index = floor(goal_y / 2.0) * size_x + floor(goal_x / 2.0);
  obstacle_heuristic_queue.emplace_back(
    distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);

  // initialize goal cell with a very small value to differentiate it from 0.0 (~uninitialized)
  // the negative value means the cell is in the open set
  obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
}

float NodeHybrid::getObstacleHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const double & cost_penalty)
{
  // If already expanded, return the cost
  const unsigned int size_x = sampled_costmap->getSizeInCellsX();
  // Divided by 2 due to downsampled costmap.
  const unsigned int start_y = floor(node_coords.y / 2.0);
  const unsigned int start_x = floor(node_coords.x / 2.0);
  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table[start_index];
  if (requested_node_cost > 0.0f) {
    // costs are doubled due to downsampling
    return 2.0 * requested_node_cost;
  }

  // If not, expand until it is included. This dynamic programming ensures that
  // we only expand the MINIMUM spanning set of the costmap per planning request.
  // Rather than naively expanding the entire (potentially massive) map for a limited
  // path, we only expand to the extent required for the furthest expansion in the
  // search-planning request that dynamically updates during search as needed.

  // start_x and start_y have changed since last call
  // we need to recompute 2D distance heuristic and reprioritize queue
  for (auto & n : obstacle_heuristic_queue) {
    n.first = -obstacle_heuristic_lookup_table[n.second] +
      distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(
    obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
    ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const unsigned int size_y = sampled_costmap->getSizeInCellsY();
  const float sqrt_2 = sqrt(2);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int idx, mx, my, mx_idx, my_idx;
  unsigned int new_idx = 0;

  const std::vector<int> neighborhood = {1, -1,  // left right
    size_x_int, -size_x_int,  // up down
    size_x_int + 1, size_x_int - 1,  // upper diagonals
    -size_x_int + 1, -size_x_int - 1};  // lower diagonals

  while (!obstacle_heuristic_queue.empty()) {
    idx = obstacle_heuristic_queue.front().second;
    std::pop_heap(
      obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
      ObstacleHeuristicComparator{});
    obstacle_heuristic_queue.pop_back();
    c_cost = obstacle_heuristic_lookup_table[idx];
    if (c_cost > 0.0f) {
      // cell has been processed and closed, no further cost improvements
      // are mathematically possible thanks to euclidean distance heuristic consistency
      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table[idx] = c_cost;  // set a positive value to close the cell

    my_idx = idx / size_x;
    mx_idx = idx - (my_idx * size_x);

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

      // if neighbor path is better and non-lethal, set new cost and add to queue
      if (new_idx < size_x * size_y) {
        cost = static_cast<float>(sampled_costmap->getCost(new_idx));
        if (cost >= INSCRIBED) {
          continue;
        }

        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if (mx == 0 && mx_idx >= size_x - 1 || mx >= size_x - 1 && mx_idx == 0) {
          continue;
        }
        if (my == 0 && my_idx >= size_y - 1 || my >= size_y - 1 && my_idx == 0) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        if (existing_cost <= 0.0f) {
          travel_cost =
            ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (cost_penalty * cost / 252.0f));
          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {
            // the negative value means the cell is in the open set
            obstacle_heuristic_lookup_table[new_idx] = -new_cost;
            obstacle_heuristic_queue.emplace_back(
              new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(
              obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
              ObstacleHeuristicComparator{});
          }
        }
      }
    }

    if (idx == start_index) {
      break;
    }
  }

  // return requested_node_cost which has been updated by the search
  // costs are doubled due to downsampling
  return 2.0 * requested_node_cost;
}

float NodeHybrid::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell)

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic <= 0.0) {
    // If no obstacle heuristic value, must have some H to use
    // In nominal situations, this should never be called.
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = goal_coords.theta * motion_table.num_angle_quantization;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * motion_table.num_angle_quantization;
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeHybrid::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  if (motion_model == MotionModel::DUBIN) {
    motion_table.state_space = std::make_unique<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else if (motion_model == MotionModel::REEDS_SHEPP) {
    motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    throw std::runtime_error(
            "Node attempted to precompute distance heuristics "
            "with invalid motion model!");
  }

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  unsigned int heading_bins = dim_3_size;
  if (search_info.use_curvature_state && motion_table.num_angle_quantization > 0) {
    heading_bins = motion_table.num_angle_quantization;
  }
  int dim_3_size_int = static_cast<int>(heading_bins);
  float angular_bin_size = 2 * M_PI / static_cast<float>(heading_bins);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = heading * angular_bin_size;
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(
  std::function<bool(const uint64_t &, nav2_smac_planner::NodeHybrid * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  if (motion_table.curvature_enabled) {
    getCurvatureNeighbors(NeighborGetter, collision_checker, traverse_unknown, neighbors);
    return;
  }
  uint64_t index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      motion_table.size_x, motion_table.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeHybrid::backtracePath(CoordinateVector & path)
{
  if (motion_table.curvature_enabled) {
    return backtraceCurvature(path);
  }
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    // Convert angle to radians
    path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(current_node->pose);
  // Convert angle to radians
  path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);

  return true;
}

namespace
{

bool transformPrimitive(
  float origin_x, float origin_y, float origin_yaw,
  const BodyPrimitive & primitive,
  const HybridMotionTable & table,
  GridCollisionChecker * collision_checker,
  bool traverse_unknown,
  NodeHybrid::CoordinateVector & world_samples,
  float & mean_cost)
{
  world_samples.clear();
  mean_cost = 0.f;
  if (primitive.samples.empty() || collision_checker == nullptr) {
    return false;
  }
  auto * costmap = collision_checker->getCostmap();
  const float res = static_cast<float>(table.curvature.limits.resolution);
  const float c0 = std::cos(origin_yaw);
  const float s0 = std::sin(origin_yaw);
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  float cost_sum = 0.f;
  world_samples.reserve(primitive.samples.size());
  for (const auto & sample : primitive.samples) {
    const float cell_x = origin_x + (sample.x * c0 - sample.y * s0) / res;
    const float cell_y = origin_y + (sample.x * s0 + sample.y * c0) / res;
    const float yaw = origin_yaw + sample.yaw;
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float qx = cell_x + (table.footprint_origin_x * cy - table.footprint_origin_y * sy) / res;
    const float qy = cell_y + (table.footprint_origin_x * sy + table.footprint_origin_y * cy) / res;
    const double wrapped = std::fmod(static_cast<double>(yaw), 2.0 * M_PI);
    double positive = wrapped < 0.0 ? wrapped + 2.0 * M_PI : wrapped;
    int heading = static_cast<int>(std::llround(positive / table.bin_size));
    const int n_heading = static_cast<int>(table.num_angle_quantization);
    heading %= n_heading;
    if (heading < 0) {
      heading += n_heading;
    }
    if (collision_checker->inCollision(qx, qy, static_cast<float>(heading), traverse_unknown)) {
      return false;
    }
    const int mx = static_cast<int>(std::llround(cell_x));
    const int my = static_cast<int>(std::llround(cell_y));
    if (mx < 0 || my < 0 || mx >= size_x || my >= size_y) {
      return false;
    }
    cost_sum += static_cast<float>(costmap->getCost(
      static_cast<unsigned int>(mx), static_cast<unsigned int>(my)));
    NodeHybrid::Coordinates pose;
    pose.x = cell_x;
    pose.y = cell_y;
    pose.theta = yaw;
    world_samples.push_back(pose);
  }
  mean_cost = cost_sum / static_cast<float>(world_samples.size());
  return true;
}

const BodyPrimitive * findFullPrimitive(const std::vector<BodyPrimitive> & prims, int to_bin, float length_m)
{
  for (const auto & prim : prims) {
    if (prim.to_bin == to_bin && std::abs(prim.length_m - length_m) < 1e-3f) {
      return &prim;
    }
  }
  return nullptr;
}

}  // namespace

unsigned int NodeHybrid::nearestHeadingBin(float yaw) const
{
  double wrapped = std::fmod(static_cast<double>(yaw), 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  const int n = static_cast<int>(motion_table.num_angle_quantization);
  if (n <= 0) {
    return 0;
  }
  int heading = static_cast<int>(std::llround(wrapped / motion_table.bin_size));
  heading %= n;
  if (heading < 0) {
    heading += n;
  }
  return static_cast<unsigned int>(heading);
}

float NodeHybrid::curvatureTraversalCost(const NodePtr & child) const
{
  if (!child || !child->has_staged) {
    return std::numeric_limits<float>::max() / 4.f;
  }
  const float normalized = child->staged_mean_cost / 252.f;
  const float base = child->staged_length_cells * (
    motion_table.travel_distance_reward + motion_table.cost_penalty * normalized);
  const int straight = static_cast<int>(motion_table.curvature.bins.straight_index);
  const bool opposite = steer_trend != 0 && child->staged_step_sign != 0 &&
    child->staged_step_sign != steer_trend;
  float coef = 1.f;
  if (opposite) {
    coef = motion_table.non_straight_penalty + motion_table.change_penalty;
  } else if (child->staged_kappa != straight) {
    coef = motion_table.non_straight_penalty;
  }
  if (child->staged_reverse) {
    coef *= motion_table.reverse_penalty;
  }
  return base * coef;
}

bool NodeHybrid::primitiveFree(
  const BodyPrimitive & primitive,
  GridCollisionChecker * collision_checker,
  bool traverse_unknown,
  CoordinateVector & world_samples,
  float & mean_cost) const
{
  return transformPrimitive(
    cont_x, cont_y, cont_yaw, primitive, motion_table, collision_checker,
    traverse_unknown, world_samples, mean_cost);
}

void NodeHybrid::stagePrimitive(
  const BodyPrimitive & primitive, const CoordinateVector & world_samples,
  float mean_cost, NodePtr neighbor) const
{
  neighbor->has_staged = true;
  neighbor->staged_samples = world_samples;
  neighbor->staged_x = world_samples.back().x;
  neighbor->staged_y = world_samples.back().y;
  neighbor->staged_yaw = world_samples.back().theta;
  neighbor->staged_delta = primitive.samples.back().delta;
  neighbor->staged_kappa = primitive.to_bin;
  neighbor->staged_step_sign = primitive.step_sign;
  neighbor->staged_trend = primitive.step_sign == 0 ? steer_trend : primitive.step_sign;
  neighbor->staged_length_cells =
    primitive.length_m / static_cast<float>(motion_table.curvature.limits.resolution);
  neighbor->staged_mean_cost = mean_cost;
  neighbor->staged_reverse = primitive.reverse;
  neighbor->_cell_cost = mean_cost;
}

void NodeHybrid::getCurvatureNeighbors(
  std::function<bool(const uint64_t &, NodeHybrid * &)> & validity_checker,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  const auto & model = motion_table.curvature;
  if (model.bins.angles.empty()) {
    return;
  }
  std::vector<BodyPrimitive> local;
  std::vector<const BodyPrimitive *> prims;
  if (!steer_settled) {
    const int bin = nearestCurvatureBin(cont_delta, model.bins);
    BodyPrimitive align;
    align.from_bin = bin;
    align.to_bin = bin;
    align.step_sign = 0;
    align.reverse = false;
    align.padded = false;
    integrateSteeringPrimitive(
      cont_delta, model.bins.angles[bin], 0.0, false, false, model.limits, align.samples);
    align.length_m = static_cast<float>(
      model.limits.reference_speed *
      minSteerTime(std::abs(model.bins.angles[bin] - cont_delta), model.limits));
    if (align.samples.empty()) {
      steer_settled = true;
      cont_delta = static_cast<float>(model.bins.angles[bin]);
      kappa_bin = bin;
    } else {
      local.push_back(std::move(align));
      prims.push_back(&local.back());
    }
  }
  if (steer_settled) {
    const int bin = std::clamp(
      kappa_bin, 0, static_cast<int>(model.bins.angles.size()) - 1);
    for (const auto & prim : model.forward[bin]) {
      prims.push_back(&prim);
    }
    if (motion_table.motion_model == MotionModel::REEDS_SHEPP) {
      for (const auto & prim : model.reverse_prims[bin]) {
        prims.push_back(&prim);
      }
    }
    const float dist_m = std::hypot(
      cont_x - motion_table.goal_cont_x, cont_y - motion_table.goal_cont_y) *
      static_cast<float>(model.limits.resolution);
    if (dist_m <= 2.f * static_cast<float>(model.limits.primitive_length)) {
      for (const auto & prim : model.short_forward[bin]) {
        prims.push_back(&prim);
      }
      if (motion_table.motion_model == MotionModel::REEDS_SHEPP) {
        for (const auto & prim : model.short_reverse[bin]) {
          prims.push_back(&prim);
        }
      }
    }
  }

  std::unordered_set<uint64_t> seen;
  for (const BodyPrimitive * prim : prims) {
    CoordinateVector world;
    float mean_cost = 0.f;
    if (!primitiveFree(*prim, collision_checker, traverse_unknown, world, mean_cost)) {
      continue;
    }
    const unsigned int heading = nearestHeadingBin(world.back().theta);
    const unsigned int packed = heading +
      motion_table.num_angle_quantization * static_cast<unsigned int>(prim->to_bin);
    const int ix = static_cast<int>(std::llround(world.back().x));
    const int iy = static_cast<int>(std::llround(world.back().y));
    if (ix < 0 || iy < 0) {
      continue;
    }
    const uint64_t index = getIndex(
      static_cast<unsigned int>(ix), static_cast<unsigned int>(iy), packed);
    if (!seen.insert(index).second) {
      continue;
    }
    NodePtr neighbor = nullptr;
    if (!validity_checker(index, neighbor) || neighbor->wasVisited()) {
      continue;
    }
    stagePrimitive(*prim, world, mean_cost, neighbor);
    neighbors.push_back(neighbor);
  }
}

bool NodeHybrid::reachedCurvatureGoal(
  const NodePtr & goal, float tolerance_cells, double heading_tolerance) const
{
  if (!goal || motion_table.curvature.bins.angles.empty()) {
    return false;
  }
  if (std::abs(cont_delta) > 1e-3f) {
    return false;
  }
  if (kappa_bin != static_cast<int>(motion_table.curvature.bins.straight_index)) {
    return false;
  }
  if (std::hypot(cont_x - goal->cont_x, cont_y - goal->cont_y) > tolerance_cells) {
    return false;
  }
  if (heading_tolerance >= 0.0) {
    const double err = std::abs(std::remainder(
      static_cast<double>(cont_yaw - goal->cont_yaw), 2.0 * M_PI));
    if (err > heading_tolerance) {
      return false;
    }
  }
  return true;
}

bool NodeHybrid::tryCurvatureConnector(
  const NodePtr & goal,
  GridCollisionChecker * collision_checker,
  bool traverse_unknown,
  float tolerance_cells,
  double heading_tolerance)
{
  connector_samples.clear();
  if (!steer_settled || !goal || motion_table.curvature.bins.angles.empty()) {
    return false;
  }
  const auto & model = motion_table.curvature;
  const float dist_m = std::hypot(cont_x - goal->cont_x, cont_y - goal->cont_y) *
    static_cast<float>(model.limits.resolution);
  if (dist_m > 2.f * static_cast<float>(model.limits.primitive_length)) {
    return false;
  }
  float x = cont_x;
  float y = cont_y;
  float yaw = cont_yaw;
  int bin = kappa_bin;
  const int straight = static_cast<int>(model.bins.straight_index);
  CoordinateVector chain;
  while (bin != straight) {
    const int next = bin < straight ? bin + 1 : bin - 1;
    const BodyPrimitive * prim = findFullPrimitive(
      model.forward[bin], next, static_cast<float>(model.limits.primitive_length));
    if (prim == nullptr) {
      return false;
    }
    CoordinateVector world;
    float mean_cost = 0.f;
    if (!transformPrimitive(
        x, y, yaw, *prim, motion_table, collision_checker, traverse_unknown, world, mean_cost))
    {
      return false;
    }
    chain.insert(chain.end(), world.begin(), world.end());
    x = world.back().x;
    y = world.back().y;
    yaw = world.back().theta;
    bin = next;
  }
  const float dx = goal->cont_x - x;
  const float dy = goal->cont_y - y;
  const float along = dx * std::cos(yaw) + dy * std::sin(yaw);
  const float lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
  if (std::abs(lateral) > tolerance_cells || along < -1e-3f) {
    return false;
  }
  if (heading_tolerance >= 0.0) {
    const double err = std::abs(std::remainder(
      static_cast<double>(yaw - goal->cont_yaw), 2.0 * M_PI));
    if (err > heading_tolerance) {
      return false;
    }
  }
  const float along_m = along * static_cast<float>(model.limits.resolution);
  if (along_m > 1e-3f) {
    BodyPrimitive straight_prim;
    straight_prim.to_bin = straight;
    straight_prim.step_sign = 0;
    integrateSteeringPrimitive(0.0, 0.0, along_m, true, false, model.limits, straight_prim.samples);
    straight_prim.length_m = along_m;
    CoordinateVector world;
    float mean_cost = 0.f;
    if (!transformPrimitive(
        x, y, yaw, straight_prim, motion_table, collision_checker, traverse_unknown, world,
        mean_cost))
    {
      return false;
    }
    chain.insert(chain.end(), world.begin(), world.end());
    x = world.back().x;
    y = world.back().y;
  }
  if (std::hypot(x - goal->cont_x, y - goal->cont_y) > tolerance_cells) {
    return false;
  }
  connector_samples = chain;
  if (motion_table.debug_callback) {
    std::vector<CurvatureDebugPoint> points;
    points.reserve(chain.size());
    for (const auto & sample : chain) {
      points.push_back(CurvatureDebugPoint{sample.x, sample.y, sample.theta});
    }
    motion_table.debug_callback("connector", points);
  }
  return true;
}

void NodeHybrid::commitStaged()
{
  if (!has_staged) {
    return;
  }
  cont_x = staged_x;
  cont_y = staged_y;
  cont_yaw = staged_yaw;
  cont_delta = staged_delta;
  kappa_bin = staged_kappa;
  steer_trend = staged_trend;
  steer_settled = true;
  edge_samples = staged_samples;
  pose.x = cont_x;
  pose.y = cont_y;
  pose.theta = static_cast<float>(nearestHeadingBin(cont_yaw));
  if (!motion_table.debug_callback) {
    return;
  }
  std::vector<CurvatureDebugPoint> edge;
  edge.reserve(edge_samples.size());
  for (const auto & sample : edge_samples) {
    edge.push_back(CurvatureDebugPoint{sample.x, sample.y, sample.theta});
  }
  motion_table.debug_callback("expansion", edge);

  static int publish_count = 0;
  if (++publish_count % 25 != 0 || parent == nullptr) {
    return;
  }
  std::vector<NodeHybrid *> stack;
  for (NodeHybrid * node = this; node != nullptr; node = node->parent) {
    stack.push_back(node);
  }
  std::vector<CurvatureDebugPoint> chain;
  chain.push_back(CurvatureDebugPoint{stack.back()->cont_x, stack.back()->cont_y, stack.back()->cont_yaw});
  for (int i = static_cast<int>(stack.size()) - 2; i >= 0; --i) {
    for (const auto & sample : stack[i]->edge_samples) {
      chain.push_back(CurvatureDebugPoint{sample.x, sample.y, sample.theta});
    }
  }
  motion_table.debug_callback("search_chain", chain);
}

bool NodeHybrid::backtraceCurvature(CoordinateVector & path)
{
  if (connector_samples.empty() && edge_samples.empty() && parent == nullptr) {
    Coordinates pose_out;
    pose_out.x = cont_x;
    pose_out.y = cont_y;
    pose_out.theta = cont_yaw;
    path.push_back(pose_out);
    return true;
  }
  for (int i = static_cast<int>(connector_samples.size()) - 1; i >= 0; --i) {
    path.push_back(connector_samples[i]);
  }
  NodePtr node = this;
  while (node->parent) {
    for (int i = static_cast<int>(node->edge_samples.size()) - 1; i >= 0; --i) {
      path.push_back(node->edge_samples[i]);
    }
    node = node->parent;
  }
  Coordinates start_pose;
  start_pose.x = node->cont_x;
  start_pose.y = node->cont_y;
  start_pose.theta = node->cont_yaw;
  path.push_back(start_pose);
  return !path.empty();
}

}  // namespace nav2_smac_planner
