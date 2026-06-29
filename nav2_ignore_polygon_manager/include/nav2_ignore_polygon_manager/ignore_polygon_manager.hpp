#ifndef NAV2_IGNORE_POLYGON_MANAGER__IGNORE_POLYGON_MANAGER_HPP_
#define NAV2_IGNORE_POLYGON_MANAGER__IGNORE_POLYGON_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "capella_ros_msg/msg/lane_center_paths.hpp"

namespace nav2_ignore_polygon_manager
{

class IgnorePolygonManager
{
public:
  /**
   * @brief Constructor storing node and prefix only
   * @param node Weak pointer to the lifecycle node
   * @param param_prefix Prefix for parameter names (e.g. layer name)
   */
  explicit IgnorePolygonManager(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node,
                                const std::string& param_prefix = "");

  ~IgnorePolygonManager();

  /**
   * @brief Declare parameters and create subscriptions
   */
  void configure();

  /**
   * @brief Select active ignore rects based on robot position — call once per cycle
   * @param robot_x Robot x position in global frame
   * @param robot_y Robot y position in global frame
   * @param robot_z Robot z position in global frame
   */
  void update(const double& robot_x, const double& robot_y, const double& robot_z);

  /**
   * @brief Check if a point should be ignored (uses pre-selected active rects)
   * @param px Point x coordinate
   * @param py Point y coordinate
   * @param pz Point z coordinate
   * @return true if point is inside any active ignore rectangle and
   *         pz > robot_z + ignore_height_above_
   */
  bool isPointIgnored(const double& px, const double& py, const double& pz) const;

  /**
   * @brief Set ignore width dynamically (atomic, in cm)
   * @param width Ignore width in cm. 0=disabled, abs(width) used for offset
   */
  void setIgnoreWidth(int width);
  int getIgnoreWidth() const;

  /**
   * @brief Set ignore range dynamically (atomic, in meters)
   * @param range Max distance from robot to consider path segments
   */
  void setIgnoreRange(double range);
  double getIgnoreRange() const;

  /**
   * @brief Check if debug mode is enabled
   * @return true if debug point cloud output is enabled
   */
  bool getDebugMode() const;

  /**
   * @brief Publish a pre-built PointCloud2 for debug visualization.
   * Only publishes when debug_mode is enabled; otherwise returns immediately.
   * @param cloud Pre-built PointCloud2 message to publish
   */
  void publishCollisionPoints(std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud);

private:
  /**
   * @brief LaneCenterPaths subscription callback
   */
  void laneCenterPathsCallback(capella_ros_msg::msg::LaneCenterPaths::ConstSharedPtr msg);

  /**
   * @brief Check if point is inside an AABB rectangle defined by 4 corner points
   *        (pts[0]=min, pts[1]=max_x_min_y, pts[2]=max, pts[3]=min_x_max_y)
   */
  bool isPointInAABB(const double& px, const double& py, const std::vector<geometry_msgs::msg::Point32>& pts) const;

  /// @brief Node weak pointer
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  /// @brief Logger
  rclcpp::Logger logger_;

  /// @brief Parameter prefix
  std::string param_prefix_;

  /// @brief LaneCenterPaths subscriber (QoS: TransientLocal + Reliable)
  rclcpp::Subscription<capella_ros_msg::msg::LaneCenterPaths>::SharedPtr lane_center_paths_sub_;

  /// @brief Debug collision points publisher (active only when debug_mode is true)
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr collision_points_pub_;

  /// @brief Ignore width in cm (atomic for dynamic updates). 0=disabled
  std::atomic<int> ignore_width_{ 0 };

  /// @brief Ignore range in meters (atomic for dynamic updates)
  std::atomic<double> ignore_range_{ 5.0 };

  /// @brief Debug mode flag: when true, collision points are published as point cloud
  std::atomic<bool> debug_mode_{ false };

  /// @brief Height threshold above robot_z in meters; a point is ignored only if
  ///        its z > robot_z + ignore_height_above_
  std::atomic<double> ignore_height_above_{ 0.0 };

  /// @brief Robot z position in global frame (set by update(), read by isPointIgnored())
  std::atomic<double> robot_z_{ 0.0 };

  /// @brief Raw paths received from LaneCenterPaths topic
  std::vector<nav_msgs::msg::Path> ignore_paths_;
  mutable std::mutex ignore_paths_mutex_;

  /// @brief Pre-selected active ignore rectangles (updated by update())
  std::vector<geometry_msgs::msg::PolygonStamped> active_ignore_rects_;
  mutable std::mutex active_ignore_rects_mutex_;
};

}  // namespace nav2_ignore_polygon_manager

#endif  // NAV2_IGNORE_POLYGON_MANAGER__IGNORE_POLYGON_MANAGER_HPP_
