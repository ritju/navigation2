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

#ifndef NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <cstdint>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/curvature_model.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"

namespace nav2_smac_planner
{

typedef std::vector<float> LookupTable;
typedef std::pair<double, double> TrigValues;

typedef std::pair<float, unsigned int> ObstacleHeuristicElement;
struct ObstacleHeuristicComparator
{
  bool operator()(const ObstacleHeuristicElement & a, const ObstacleHeuristicElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;

// Must forward declare
class NodeHybrid;

/**
 * @struct nav2_smac_planner::HybridMotionTable
 * @brief A table of motion primitives and related functions
 */
struct HybridMotionTable
{
  /**
   * @brief A constructor for nav2_smac_planner::HybridMotionTable
   */
  HybridMotionTable() {}

  /**
   * @brief Initializing using Dubin model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param angle_quantization_in Size of costmap in bin sizes
   * @param search_info Parameters for searching
   */
  void initDubin(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  /**
   * @brief Initializing using Reeds-Shepp model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param angle_quantization_in Size of costmap in bin sizes
   * @param search_info Parameters for searching
   */
  void initReedsShepp(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to NodeHybrid
   * @return A set of motion poses
   */
  MotionPoses getProjections(const NodeHybrid * node);

  /**
   * @brief Get the angular bin to use from a raw orientation
   * @param theta Angle in radians
   * @return bin index of closest angle to request
   */
  unsigned int getClosestAngularBin(const double & theta);

  /**
   * @brief Get the raw orientation from an angular bin
   * @param bin_idx Index of the bin
   * @return Raw orientation in radians
   */
  float getAngleFromBin(const unsigned int & bin_idx);

  MotionModel motion_model = MotionModel::UNKNOWN;
  MotionPoses projections;
  unsigned int size_x;
  unsigned int num_angle_quantization{0};
  float num_angle_quantization_float;
  float min_turning_radius;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  float travel_distance_reward;
  ompl::base::StateSpacePtr state_space;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<TrigValues> trig_values;
  bool curvature_enabled{false};
  unsigned int index_dim3{1};
  CurvatureModel curvature{};
  float goal_cont_x{0.f};
  float goal_cont_y{0.f};
  float goal_cont_yaw{0.f};
  float xy_tolerance_cells{1.f};
  float heading_tolerance{-1.f};
  float footprint_origin_x{0.f};
  float footprint_origin_y{0.f};
  CurvatureDebugCallback debug_callback{};
  void syncCurvature(const SearchInfo & search_info, bool allow_reverse);
};

/**
 * @class nav2_smac_planner::NodeHybrid
 * @brief NodeHybrid implementation for graph, Hybrid-A*
 */
class NodeHybrid
{
public:
  typedef NodeHybrid * NodePtr;
  typedef std::unique_ptr<std::vector<NodeHybrid>> Graph;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @class nav2_smac_planner::NodeHybrid::Coordinates
   * @brief NodeHybrid implementation of coordinate structure
   */
  struct Coordinates
  {
    /**
     * @brief A constructor for nav2_smac_planner::NodeHybrid::Coordinates
     */
    Coordinates() {}

    /**
     * @brief A constructor for nav2_smac_planner::NodeHybrid::Coordinates
     * @param x_in X coordinate
     * @param y_in Y coordinate
     * @param theta_in Theta coordinate
     */
    Coordinates(const float & x_in, const float & y_in, const float & theta_in)
    : x(x_in), y(y_in), theta(theta_in)
    {}

    inline bool operator==(const Coordinates & rhs)
    {
      return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
    }

    inline bool operator!=(const Coordinates & rhs)
    {
      return !(*this == rhs);
    }

    float x, y, theta;
  };

  typedef std::vector<Coordinates> CoordinateVector;

  /**
   * @brief A constructor for nav2_smac_planner::NodeHybrid
   * @param index The index of this node for self-reference
   */
  explicit NodeHybrid(const uint64_t index);

  /**
   * @brief A destructor for nav2_smac_planner::NodeHybrid
   */
  ~NodeHybrid();

  /**
   * @brief operator== for comparisons
   * @param NodeHybrid right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const NodeHybrid & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief setting continuous coordinate search poses (in partial-cells)
   * @param Pose pose
   */
  inline void setPose(const Coordinates & pose_in)
  {
    pose = pose_in;
  }

  /**
   * @brief Reset method for new search
   */
  void reset();

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  inline void setAccumulatedCost(const float & cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Sets the motion primitive index used to achieve node in search
   * @param reference to motion primitive idx
   */
  inline void setMotionPrimitiveIndex(const unsigned int & idx)
  {
    _motion_primitive_index = idx;
  }

  /**
   * @brief Gets the motion primitive index used to achieve node in search
   * @return reference to motion primitive idx
   */
  inline unsigned int & getMotionPrimitiveIndex()
  {
    return _motion_primitive_index;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline float & getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  inline bool & wasVisited()
  {
    return _was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  inline void visited()
  {
    _was_visited = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
  inline uint64_t & getIndex()
  {
    return _index;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker);

  /**
   * @brief Get traversal cost of parent node to child node
   * @param child Node pointer to child
   * @return traversal cost
   */
  float getTraversalCost(const NodePtr & child);

  /**
   * @brief Get index at coordinates
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @param width Width of costmap
   * @param angle_quantization Number of theta bins
   * @return Index
   */
  static inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    const uint64_t dim3 = angle_quantization;
    const uint64_t xx = x;
    const uint64_t yy = y;
    const uint64_t ww = width;
    return static_cast<uint64_t>(angle) + xx * dim3 + yy * ww * dim3;
  }

  /**
   * @brief Get index at coordinates
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @return Index
   */
  static inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {
    return getIndex(
      x, y, angle, motion_table.size_x,
      motion_table.index_dim3);
  }

  /**
   * @brief Get coordinates at index
   * @param index Index of point
   * @param width Width of costmap
   * @param angle_quantization Theta size of costmap
   * @return Coordinates
   */
  static inline Coordinates getCoords(
    const uint64_t & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    const uint64_t dim3 = angle_quantization;
    const uint64_t ww = width;
    return Coordinates(
      static_cast<float>((index / dim3) % ww),
      static_cast<float>(index / (dim3 * ww)),
      static_cast<float>(index % dim3));
  }

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @param costmap Costmap ptr to use
   * @return Heuristic cost between the nodes
   */
  static float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates,
    const nav2_costmap_2d::Costmap2D * costmap);

  /**
   * @brief Initialize motion models
   * @param motion_model Motion model enum to use
   * @param size_x Size of X of graph
   * @param size_y Size of y of graph
   * @param angle_quantization Size of theta bins of graph
   * @param search_info Search info to use
   */
  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  /**
   * @brief Compute the SE2 distance heuristic
   * @param lookup_table_dim Size, in costmap pixels, of the
   * each lookup table dimension to populate
   * @param motion_model Motion model to use for state space
   * @param dim_3_size Number of quantization bins for caching
   * @param search_info Info containing minimum radius to use
   */
  static void precomputeDistanceHeuristic(
    const float & lookup_table_dim,
    const MotionModel & motion_model,
    const unsigned int & dim_3_size,
    const SearchInfo & search_info);

  /**
   * @brief Compute the Obstacle heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @return heuristic Heuristic value
   */
  static float getObstacleHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const double & cost_penalty);

  /**
   * @brief Compute the Distance heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @param obstacle_heuristic Value of the obstacle heuristic to compute
   * additional motion heuristics if required
   * @return heuristic Heuristic value
   */
  static float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic);

  /**
   * @brief reset the obstacle heuristic state
   * @param costmap Costmap to use
   * @param goal_coords Coordinates to start heuristic expansion at
   */
  static void resetObstacleHeuristic(
    nav2_costmap_2d::Costmap2D * costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param validity_checker Functor for state validity checking
   * @param collision_checker Collision checker to use
   * @param traverse_unknown If unknown costs are valid to traverse
   * @param neighbors Vector of neighbors to be filled
   */
  void getNeighbors(
    std::function<bool(const uint64_t &, nav2_smac_planner::NodeHybrid * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  bool reachedCurvatureGoal(
    const NodePtr & goal, float tolerance_cells, double heading_tolerance) const;

  bool tryCurvatureConnector(
    const NodePtr & goal,
    GridCollisionChecker * collision_checker,
    bool traverse_unknown,
    float tolerance_cells,
    double heading_tolerance);

  void commitStaged();

  float cont_x{0.f};
  float cont_y{0.f};
  float cont_yaw{0.f};
  float cont_delta{0.f};
  int kappa_bin{0};
  int steer_trend{0};
  bool steer_settled{true};
  bool has_staged{false};
  float staged_x{0.f};
  float staged_y{0.f};
  float staged_yaw{0.f};
  float staged_delta{0.f};
  int staged_kappa{0};
  int staged_trend{0};
  int staged_step_sign{0};
  float staged_length_cells{0.f};
  float staged_mean_cost{0.f};
  bool staged_reverse{false};
  CoordinateVector staged_samples;
  CoordinateVector edge_samples;
  CoordinateVector connector_samples;

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(CoordinateVector & path);

  NodeHybrid * parent;
  Coordinates pose;

  // Constants required across all nodes but don't want to allocate more than once
  static double travel_distance_cost;
  static HybridMotionTable motion_table;
  // Wavefront lookup and queue for continuing to expand as needed
  static LookupTable obstacle_heuristic_lookup_table;
  static ObstacleHeuristicQueue obstacle_heuristic_queue;

  static nav2_costmap_2d::Costmap2D * sampled_costmap;
  static CostmapDownsampler downsampler;
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  static LookupTable dist_heuristic_lookup_table;
  static float size_lookup;

private:
  float _cell_cost;
  float _accumulated_cost;
  uint64_t _index;
  bool _was_visited;
  unsigned int _motion_primitive_index;

  float curvatureTraversalCost(const NodePtr & child) const;
  void getCurvatureNeighbors(
    std::function<bool(const uint64_t &, NodeHybrid * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);
  bool backtraceCurvature(CoordinateVector & path);
  bool primitiveFree(
    const BodyPrimitive & primitive,
    GridCollisionChecker * collision_checker,
    bool traverse_unknown,
    CoordinateVector & world_samples,
    float & mean_cost) const;
  void stagePrimitive(const BodyPrimitive & primitive, const CoordinateVector & world_samples,
    float mean_cost, NodePtr neighbor) const;
  unsigned int nearestHeadingBin(float yaw) const;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_
