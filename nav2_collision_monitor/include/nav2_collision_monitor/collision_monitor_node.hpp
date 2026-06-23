// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"
#include "nav2_ignore_polygon_manager/ignore_polygon_manager.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  explicit CollisionMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers,
   * creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

protected:
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Publishes output cmd_vel. If robot was stopped more than stop_pub_timeout_ seconds,
   * quit to publish 0-velocity.
   * @param robot_action Robot action to publish
   */
  void publishVelocity(const Action& robot_action);

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * is required.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(std::string& cmd_vel_in_topic, std::string& cmd_vel_out_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   * @return True if all polygons were configured successfully or false in failure case
   */
  bool configurePolygons(const std::string& base_frame_id, const tf2::Duration& transform_tolerance);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @param base_frame_id Robot base frame ID
   * @param odom_frame_id Odometry frame ID. Used as global frame to get
   * source->base time inerpolated transform.
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @return True if all sources were configured successfully or false in failure case
   */
  bool configureSources(const std::string& base_frame_id, const std::string& odom_frame_id,
                        const tf2::Duration& transform_tolerance, const rclcpp::Duration& source_timeout);

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   */
  void process(const Velocity& cmd_vel_in);

  /**
   * @brief Processes the polygon of STOP and SLOWDOWN action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processStopSlowdown(const std::shared_ptr<Polygon> polygon, const std::vector<Point>& collision_points,
                           const Velocity& velocity, Action& robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processApproach(const std::shared_ptr<Polygon> polygon, const std::vector<Point>& collision_points,
                       const Velocity& velocity, Action& robot_action) const;

  /**
   * @brief Latest local plan callback (stores path for next process() tick)
   */
  void localPlanCallback(nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief If local plan predicts footprint collision along path from start, set STOP.
   * @param path Path in path.header.frame_id (consumed for this cycle; empty path skips).
   * @param collision_points Obstacle points in base_frame_id_.
   * @param curr_time Time used for TF lookups.
   * @param plan_receive_time Time when path was received (fallback if header.stamp unset).
   * @param approach_polygon Footprint model (APPROACH polygon/circle) for point-in-footprint test.
   * @param robot_action In/out action; may be forced to STOP with zero velocity.
   * @return true if collision detected; false if path was fully checked and clear;
   *         std::nullopt if skipped (invalid input, TF failure, or plan expired per
   *         local_plan_validity_timeout_ — expiry does not STOP, only skips the check).
   */
  std::optional<bool> processLocalPlanCollision(const nav_msgs::msg::Path& path,
                                                const std::vector<Point>& collision_points,
                                                const rclcpp::Time& curr_time, const rclcpp::Time& plan_receive_time,
                                                const std::shared_ptr<Polygon>& approach_polygon, Action& robot_action);

  /**
   * @brief Prints robot action and polygon caused it (if it was)
   * @param robot_action Robot action to print
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void printAction(const Action& robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  /// @brief Ignore polygon manager shared across all sources
  std::shared_ptr<nav2_ignore_polygon_manager::IgnorePolygonManager> ignore_manager_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  /// @brief Optional subscriber for local plan footprint collision checks
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub_;

  /// @brief Latest local plan (protected by local_plan_mutex_)
  nav_msgs::msg::Path latest_local_plan_;
  /// @brief Receive time of latest_local_plan_ (same mutex)
  rclcpp::Time latest_local_plan_receive_time_;
  /// @brief Mutex for latest_local_plan_
  std::mutex local_plan_mutex_;

  /// @brief Robot base frame (cached for TF in local plan checks)
  std::string base_frame_id_;
  /// @brief Global frame (e.g. "map") for ignore manager robot position lookup
  std::string global_frame_id_;
  /// @brief TF transform tolerance
  tf2::Duration transform_tolerance_{ tf2::durationFromSec(0.0) };

  /// @brief Enable velocity-based APPROACH (TTC) processing
  bool use_velocity_approach_prediction_{ true };
  /// @brief Enable local plan footprint collision branch (STOP on hit)
  bool use_local_plan_collision_check_{ false };
  /// @brief Topic for nav_msgs/Path local plan
  std::string local_plan_topic_{ "local_plan" };
  /// @brief Forward distance along path from first pose to check (m)
  double local_plan_lookahead_distance_{ 5.0 };
  /// @brief Sample spacing along path = ratio * footprint bbox max side (m), after updatePolygon
  double local_plan_sample_spacing_ratio_{ 0.5 };
  /// @brief If >0, path older than this vs header/receive time skips local plan collision check
  rclcpp::Duration local_plan_validity_timeout_{ 0, 0 };
  /// @brief After local-plan footprint STOP, hold STOP until a new path is checked collision-free
  bool local_plan_collision_stop_latched_{ false };

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  rclcpp::Duration stop_pub_timeout_, recover_stop_timeout_;
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
