// Copyright (c) 2026
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INSERT_GARBAGE_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INSERT_GARBAGE_POSE_ACTION_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "capella_ros_msg/msg/garbage_detect.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/** 垃圾接收、后处理、插入 goals 的行为树节点 */
class InsertGarbagePose : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  /** 后处理结果：每项含 map 位姿、角点、类别 class_id */
  typedef std::vector<capella_ros_msg::msg::GarbageDetect> GarbageList;
  /** 历史垃圾队列最大长度 */
  static constexpr std::size_t kMaxHistorySize = 1;
  /** map 下去重距离阈值米，后到且更近于此的删掉 */
  static constexpr double kDedupDistanceM = 0.15;

  // 插入前采集到的全部信息，valid 为 false 时不做删点插点
  struct InsertInfo
  {
    bool valid{false};
    std::string invalid_reason;

    Goals goals;                                          // 完整 {goals}
    geometry_msgs::msg::PoseStamped robot_pose;           // 机器人当前 map 位姿
    geometry_msgs::msg::PoseStamped goala;                // 投影线起点：机器人当前位置
    geometry_msgs::msg::PoseStamped goalc;                // 投影线终点：从机器人沿路径第一个角点
    std::size_t goalc_idx{0};                             // goalc 在 goals 中的下标
    capella_ros_msg::msg::GarbageDetect garbage;          // 单堆：map 位姿、角点、class_id
    double radius_m{0.0};                                 // R = 机器人到垃圾距离
    double goald_x{0.0};                                  // 垃圾向 goala-goalc 无限直线的垂足
    double goald_y{0.0};
    double path_yaw{0.0};                                 // goala 到 goalc 方向，插入朝向用
  };

  InsertGarbagePose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Input goals list (e.g. {goals})"),
      BT::OutputPort<Goals>("output_goals", "Goals after inserting garbage poses"),
      BT::InputPort<std::string>(
        "garbage_topic", std::string("/garbage_cord"), "Garbage detection topic"),
      BT::InputPort<std::string>(
        "special_terrain_topic", std::string("/cleaning_tool_retraction_areas"),
        "Special / retraction area polygons topic"),
      BT::InputPort<std::string>(
        "footprint_topic", std::string("local_costmap/published_footprint"),
        "Robot footprint topic"),
      BT::InputPort<double>(
        "arrived_radius", 0.5, "Stop inserting when footprint enters this radius around garbage"),
      BT::InputPort<double>(
        "clip_extend_m", 1.0, "After garbage foot on path, delete goals for this distance (m)"),
      BT::InputPort<double>(
        "corner_angle_deg", 60.0, "Goals with turn angle above this are corners (deg)"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

private:
  void halt() override {}
  /** 行为树周期回调 */
  BT::NodeStatus tick() override;

  /** 垃圾检测话题回调，把垃圾放进 history_list_ */
  void garbageDetectCallback(const capella_ros_msg::msg::GarbageDetect::SharedPtr msg);

  /** 特殊清扫/禁扫区域话题回调 */
  void special_terrain_callback(const garage_utils_msgs::msg::Polygons::SharedPtr msg);

  /** footprint 话题回调 */
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  /** 接收到垃圾后的后处理函数，返回处理后的 garbage_list */
  GarbageList postProcessHistory();

  /** 接收完整的 {goals} 路径点 */
  Goals receiveGoals();

  /** 对比 goals 时间戳，外部重发任务时清空 history 和 garbage */
  void checkAndResetOnNewMission();

  /** 获取机器人当前 footprint，并转到 map */
  bool getRobotFootprintInMap(std::vector<geometry_msgs::msg::Point> & footprint_map);

  /** 判断 footprint 是否已进入垃圾附近，应停止再插入 */
  bool shouldStopInsertingGarbage(
    const capella_ros_msg::msg::GarbageDetect & garbage,
    const std::vector<geometry_msgs::msg::Point> & footprint_map,
    double arrived_radius) const;

  /** 获取插入所需的全部信息并返回 */
  InsertInfo gatherInsertInfo();

  /** 插入垃圾前按折线下标删点：clip 起点到离垃圾最近点，再从折线垂足延伸 */
  Goals clipGoalsNearGarbage(const InsertInfo & info, const Goals & goals);

  /** 剔圆内点、插入真实垃圾、统一时间戳 */
  Goals insertGarbageIntoGoals(const InsertInfo & info);

  /** 把单个垃圾从 base_link 转到 map */
  bool transformGarbageToMap(capella_ros_msg::msg::GarbageDetect & garbage) const;
  /** 判断点是否在禁扫区域内 */
  bool isPointInSpecialTerrain(double x, double y) const;
  /** 判断点是否在多边形内 */
  static bool isPointInPolygon(
    double x, double y, const geometry_msgs::msg::Polygon & polygon);
  /** 是否与已保留垃圾过近 */
  static bool isDuplicateOfKept(
    const capella_ros_msg::msg::GarbageDetect & garbage,
    const GarbageList & kept);
  /** 是否落在已到达过、不再插入的垃圾附近 */
  bool isNearReachedGarbage(double x, double y) const;
  /** 平面距离平方 */
  static double squaredDistanceXY(
    double x1, double y1, double x2, double y2);
  /** 写入 garbage_list_，满了时优先保留离机器人更近的 */
  bool tryInsertPreferCloserToRobot(
    capella_ros_msg::msg::GarbageDetect garbage,
    double robot_x, double robot_y);
  /** 点到无限直线 AB 的垂足 */
  static void projectPointToInfiniteLine(
    double px, double py,
    double ax, double ay,
    double bx, double by,
    double & out_x, double & out_y);
  /** 点在无限直线 AB 上的参数 t */
  static double lineParameterT(
    double px, double py,
    double ax, double ay,
    double bx, double by);
  /** true=非角点，false=角点；无前点用机器人坐标，无后点视为非角点 */
  bool isGoalNotCorner(
    const Goals & goals,
    std::size_t idx,
    double robot_x, double robot_y) const;
  /** 从机器人沿路径找第一个角点下标；找不到返回 false */
  bool findFirstCornerFromRobot(
    const Goals & goals,
    double robot_x, double robot_y,
    std::size_t & corner_idx) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::GarbageDetect>::SharedPtr garbage_sub_;
  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr special_terrain_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string garbage_topic_;
  std::string special_terrain_topic_;
  std::string footprint_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double arrived_radius_{0.5};
  double clip_extend_m_{1.0};
  double corner_angle_deg_{60.0};

  std::mutex history_mutex_;
  /** 原始接收缓存 */
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_;
  /** 后处理结果列表 */
  GarbageList garbage_list_;
  /** 从黑板接收到的完整 goals */
  Goals received_goals_;
  /** 当前认定的任务时间戳，与 goals 上统一 stamp 对齐 */
  rclcpp::Time mission_stamp_record_{0, 0, RCL_ROS_TIME};
  bool has_mission_stamp_{false};
  /** 本任务内已到达过、不再插入的垃圾 map 坐标 */
  std::vector<std::pair<double, double>> reached_garbage_xy_;

  mutable std::mutex special_terrain_mutex_;
  /** 禁扫区多边形 */
  std::vector<geometry_msgs::msg::Polygon> special_terrain_polygons_;

  mutable std::mutex footprint_mutex_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_footprint_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INSERT_GARBAGE_POSE_ACTION_HPP_
