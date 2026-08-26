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
#include <limits>
#include <memory>
#include <mutex>
#include <set>
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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_behavior_tree
{

/** 垃圾接收、后处理、插入 goals 的行为树节点 */
class InsertGarbagePose : public BT::ActionNodeBase
{
public:
  /** 一串带坐标系和时间戳的位姿点 */
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  /** 后处理结果：每项含 map 位姿、角点、类别 class_id */
  typedef std::vector<capella_ros_msg::msg::GarbageDetect> GarbageList;
  /** history_list_ 最大长度 */
  static constexpr std::size_t kMaxHistorySize = 10;
  /** garbage_list_ 最大长度 */
  static constexpr std::size_t kMaxGarbageSize = 6;
  /** 清扫顺序全排列上限，超过则贪心 */
  static constexpr std::size_t kSweepBruteMaxN = 6;
  /** map 下去重距离阈值米，后到且更近于此的删掉 */
  static constexpr double kDedupDistanceM = 0.4;
  /** 认「同一颗」：按下标找到 z=-1 槽后，xy 只用来确认 3.1 vs 3.11，不拿来搜附近别的堆 */
  static constexpr double kSentinelIdentityMatchM = 0.05;
  /** 本节点约定：插入的 G/E 点 pose.position.z 固定写此值，表示无任务序号的哨兵点 */
  static constexpr double kGarbageSentinelPoseZ = -1.0;
  /** 连续可视化归入同一任务的间隔阈值秒 */
  static constexpr double kVizTaskWindowSec = 2.0;
  /** base_link 下距原点小于此值的检测视为无效 */
  static constexpr double kInvalidGarbageOriginRadiusM = 0.3;
  /** from 离 G 近于此则视为已到达：不用欧氏远近选侧，沿车头在 G 后方虚设来向 */
  static constexpr double kMinExtendFromDistM = 0.5;
  /** 墙切向走廊失败后，绕该切向左右各扫到此角度 */
  static constexpr double kExtendYawSweepMaxDeg = 30.0;
  /** 切向扫角步长 */
  static constexpr double kExtendYawSweepStepDeg = 10.0;

  // 插入前采集到的全部信息，valid 为 false 时不做删点插点
  struct InsertInfo
  {
    bool valid{false};
    std::string invalid_reason;

    Goals goals;                                          // 完整 {goals}
    geometry_msgs::msg::PoseStamped robot_pose;           // 机器人当前 map 位姿
    geometry_msgs::msg::PoseStamped goala;                // 投影线起点：goaltotal_range_m 内第一个点
    geometry_msgs::msg::PoseStamped goalc;                // 投影线终点：第一角点；无角点则为前方 range 内末点
    std::size_t goalc_idx{0};                             // goalc 在 goals 中的下标
    Goals goaltotal;                                      // 本轮逻辑判定要删除的 goals最后一块删
    capella_ros_msg::msg::GarbageDetect garbage;          // 单堆：map 位姿、角点、class_id
    double radius_m{0.0};                                 // R = 机器人到垃圾距离
    double goald_x{0.0};                                  // 垃圾向 goala-goalc 无限直线的垂足
    double goald_y{0.0};
    double path_yaw{0.0};                                 // 插入朝向 / 默认伸 E；from 贴 G 时改用车头向前
    double extend_from_x{0.0};                            // 算 E 的假设车位 x：首堆当前车，其后上一堆 E
    double extend_from_y{0.0};                            // 算 E 的假设车位 y
    bool extend_inserted{false};                          // 本堆是否实际插入了延伸点 E
    double extend_used_m{0.0};                            // 实际采用的延伸距离，可与参数不同
    double extend_x{0.0};                                 // 实际写入 goals 的 E 点 x，墙切向后与 path_yaw 重算可能不同
    double extend_y{0.0};                                 // 实际写入 goals 的 E 点 y
    bool hit_mid_case{false};                             // 垂足落在段中
    bool hit_forward_case{false};                         // 前方延长线
    std::vector<std::pair<double, double>> corners_kept_xy;  // 前方延长线保留角点
    /** 离机器人远近编号 */
    int dist_label{0};

    // 每一轮 A-C 投影，给可视化用
    struct ClipRound
    {
      int round_i{0};
      double ax{0.0};
      double ay{0.0};
      double cx{0.0};
      double cy{0.0};
      double fx{0.0};
      double fy{0.0};
      double t_d{0.0};
    };
    std::vector<ClipRound> clip_rounds;
  };

  InsertGarbagePose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Input goals list (e.g. {goals})"),
      BT::OutputPort<Goals>("output_goals", "Goals after inserting garbage poses"),
      BT::OutputPort<Goals>(
        "protected_garbage",
        "Inserted G/E poses; RemovePassedGoals must not drop these as behind-the-robot"),
      BT::InputPort<std::string>(
        "garbage_topic", std::string("/garbage_cord1"), "Garbage detection topic"),
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
        "corner_angle_deg", 30.0, "Goals with turn angle above this are corners (deg)"),
      BT::InputPort<double>(
        "goaltotal_range_m", 10.0,
        "If no corner ahead, use last goal within this path distance as goalc (m)"),
      BT::InputPort<double>(
        "head_delete_robot_dist_m", 4.0,
        "Only delete from goals head forward when robot is within this distance (m) of goals[0]"),
      BT::InputPort<double>(
        "max_garbage_robot_dist_m", 5.0,
        "Ignore garbage farther than this distance (m) from robot (anti false-detect)"),
      BT::InputPort<double>(
        "garbage_merge_radius_m", 1.0,
        "Merge detections within this radius (m) of the nearest seed into one pile"),
      BT::InputPort<double>(
        "work_circle_radius_m", 10.0,
        "Accept new garbage only inside this radius around the robot pose when the first pile of a batch is accepted"),
      BT::InputPort<double>(
        "min_garbage_obstacle_clearance_m", 0.7,
        "Discard garbage if any lethal obstacle cell is within this radius (m) on local costmap"),
      BT::InputPort<std::string>(
        "local_costmap_topic", std::string("local_costmap/costmap"),
        "Local costmap OccupancyGrid topic for obstacle-info readability check"),
      BT::InputPort<bool>(
        "enable_visualization", true, "Publish insert/clip markers to RViz"),   //总开关
      BT::InputPort<bool>(
        "viz_accepted_garbage", true, "Show accepted garbage after filtering"),
      BT::InputPort<bool>(
        "viz_deleted_goals", true, "Show deleted goals as hollow black rings"),
      BT::InputPort<bool>(
        "viz_ac_points", true, "Show A/C points and labels each clip round"),
      BT::InputPort<std::string>(
        "visualization_topic", std::string("insert_garbage_pose/markers"),
        "MarkerArray topic for insert visualization"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

private:
  void halt() override {}
  /** 行为树周期回调 */
  BT::NodeStatus tick() override;

  /** 垃圾检测话题回调：转到 map；合堆半径内已见过则不进 history */
  void garbageDetectCallback(const capella_ros_msg::msg::GarbageDetect::SharedPtr msg);

  /** 特殊清扫/禁扫区域话题回调 */
  void special_terrain_callback(const garage_utils_msgs::msg::Polygons::SharedPtr msg);

  /** footprint 话题回调 */
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  /** 局部代价图话题回调 */
  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * 障碍物情况可读：点落在局部代价图内，且格子不是 unknown。
   * 失败时 reason 写入原因（可为 nullptr）。
   */
  bool isObstacleInfoReadable(double x, double y, std::string * reason) const;

  /** 局部代价图该点可通行：仅 254/255 不可过，253 可通过 */
  bool isMapPointPassableOnLocalCostmap(double x, double y) const;

  /**
   * 垃圾点周围 radius_m 内局部代价图是否有占用障碍（cell>=100）。
   * 有则视为贴墙扫不了，筛选阶段直接丢弃。
   */
  bool hasObstacleWithinRadius(double x, double y, double radius_m) const;

  /**
   * 在局部代价图上找离 (x,y) 最近的障碍格。
   * 原方向 E 进墙时用：G 与该格连线做法向，垂线方向再伸 E。
   * 只认占用格（cell>=100）；搜索半径为延伸距离再加 0.5m，不是把 E 伸那么远。
   * 成功时 ox/oy 为该格中心的 map 坐标。
   */
  bool findNearestObstaclePixel(double x, double y, double * ox, double * oy);

  /** 直线走廊无 lethal：footprint 转到行驶朝向后，半宽 0.1m 抽样并检查各顶点 */
  bool isStraightCorridorClear(
    double start_x, double start_y,
    double end_x, double end_y,
    std::string * reason) const;

  /** 接收到垃圾后的后处理函数，返回处理后的 garbage_list */
  GarbageList postProcessHistory();

  /** 接收完整的 {goals} 路径点 */
  Goals receiveGoals();

  /** 对比 goals 时间戳，外部重发任务时清空 history 和 garbage */
  void checkAndResetOnNewMission();

  /** 获取机器人当前 footprint，并转到 map */
  bool getRobotFootprintInMap(
    std::vector<geometry_msgs::msg::Point> & footprint_map) const;

  /** 获取 footprint 在 base_link 下的顶点，供走廊按行驶朝向旋转 */
  bool getRobotFootprintInBase(
    std::vector<std::pair<double, double>> & local_xy) const;

  /** 判断 footprint 是否已进入垃圾附近 */
  bool shouldStopInsertingGarbage(
    const capella_ros_msg::msg::GarbageDetect & garbage,
    const std::vector<geometry_msgs::msg::Point> & footprint_map,
    double arrived_radius,
    double robot_x, double robot_y, double robot_yaw) const;

  /**
   * 判断该 goal 是否为本节点插入的 G/E 哨兵点，而不是带序号的普通途经点。
   */
  static bool isUnindexedSentinelPoseZ(
    const geometry_msgs::msg::PoseStamped & pose_stamped_goal);

  /** footprint 是否已到达该 xy，并返回触发原因与距离 */
  struct SentinelArrivalDetail
  {
    bool arrived{false};
    bool by_vertex_radius{false};
    bool by_inside_polygon{false};
    double min_vertex_dist_m{std::numeric_limits<double>::infinity()};
  };

  SentinelArrivalDetail probeSentinelArrival(
    double gx, double gy,
    const std::vector<geometry_msgs::msg::Point> & footprint_map,
    double arrived_radius) const;

  /**
   * 扫到判定：垃圾在 footprint 内，且距 base_link 不超过 arrived_radius。
   * 避免只被 1m 多长的车头擦到就删。
   */
  bool isGarbageCoveredByFootprint(
    double gx, double gy,
    const std::vector<geometry_msgs::msg::Point> & footprint_map,
    double robot_x, double robot_y, double robot_yaw,
    double * dist_robot_m = nullptr,
    double * base_x = nullptr,
    double * base_y = nullptr) const;

  /** 本任务内按 xy 分配稳定 G 编号，重插同一堆不改号 */
  int assignStableGNum(double x, double y);
  int lookupStableGNum(double x, double y) const;
  /** E 点坐标登记所属 G 编号，footprint 删 E 时显示 E1/E2… */
  void registerStableENum(double x, double y, int g_num);
  int lookupStableENum(double x, double y) const;

  /**
   * 按下标找这堆在 {goals} 里的槽：该格 z=-1，xy 仅确认同一颗。
   * 找到返回 true 并写出 index；没有任何一格对上返回 false。
   */
  bool findUnindexedSentinelIndex(
    const Goals & goals, double x, double y, std::size_t * index_out) const;

  /**
   * 每 tick 检查全部已插堆：{goals} 里找不到这堆自己的 z=-1 槽则从 active 去掉。
   */
  std::size_t stripReachedZNeg1Goals(
    const Goals & goals,
    std::string * deleted_summary = nullptr);

  /** 每次 setOutput("output_goals") 时打日志，便于观察时机与频率 */
  void emitOutputGoals(const Goals & goals, const char * reason);

  /** 紧凑打印 goals：(x,y) 或 (x,y,-1) */
  std::string formatGoalsListCompact(const Goals & goals) const;

  /** 获取插入所需的全部信息并返回 */
  InsertInfo gatherInsertInfo(
    const Goals & goals,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double garbage_x, double garbage_y);

  /** 按 goala/goalc/goald 收集待删点到 goaltotal，再一块删除 */
  Goals clipGoalsNearGarbage(InsertInfo & info);

  /** 插入真实垃圾、统一时间戳；接回点前残留丢掉，避免扫完折返 */
  Goals insertGarbageIntoGoals(InsertInfo & info);

  /**在触发了延长线删点逻辑和检查当前角点还需不需要保护 */
  void refreshCornersOnRemaining(
    const Goals & goals,
    double robot_x, double robot_y,
    std::set<std::size_t> & delete_idx,
    std::set<std::size_t> & protected_corners,
    std::vector<std::pair<double, double>> & corners_kept_xy,
    const std::size_t * keep_idx,
    const std::size_t * also_keep_idx = nullptr) const;

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
  bool isProtectedGarbageXy(double x, double y) const;
  void addProtectedGarbageXy(double x, double y);
  void eraseProtectedGarbageXy(double x, double y);
  /** 正在清扫的堆：active 里仍占着 z=-1 槽的那一堆；中途新堆时这些点不剥、不重插 */
  bool collectInProgressKeepXy(
    const Goals & goals,
    std::vector<std::pair<double, double>> * keep_xy,
    int * keep_g_num) const;

  /** 上一堆插入点是否仍在 goals 中 */
  bool isPendingGarbageInGoals(const Goals & goals) const;

  /** 平面距离平方 */
  static double squaredDistanceXY(
    double x1, double y1, double x2, double y2);
  /** 写入 garbage_list_，满了时优先保留离机器人更近的 */
  bool tryInsertPreferCloserToRobot(
    capella_ros_msg::msg::GarbageDetect garbage,
    double robot_x, double robot_y);

  /** 合堆：距机器人最近的点当种子，到种子小于半径的并入，返回各堆代表点 */
  static GarbageList mergeGarbagePiles(
    const GarbageList & candidates,
    double robot_x, double robot_y,
    double merge_radius_m);

  /**
   * 多堆清扫顺序
   */
  std::vector<std::size_t> computeSweepOrder(
    const GarbageList & garbage_list,
    double robot_x, double robot_y,
    double robot_yaw);

  /** 按 computeSweepOrder 重排成员 garbage_list_，使 [0] 为下一堆 */
  void reorderGarbageListBySweep(
    double robot_x, double robot_y, double robot_yaw);

  /**
    找机器人最近的这个垃圾
   */
  void reorderNearestFirstThenSweep(
    double robot_x, double robot_y, double robot_yaw);

  /**
   * 导航中新堆：4-1/4-2 重排 garbage_list_。
   * 返回 true 表示 4-1
   */
  bool reorderGarbageListWithNewPile(
    double robot_x, double robot_y, double robot_yaw,
    std::size_t new_idx);

  /** 相对 before，找出本轮新入队的堆下标 */
  bool findNewGarbageIndex(
    const GarbageList & before,
    double robot_x, double robot_y,
    std::size_t & new_idx) const;

  /** 把当前 garbage_list_ 顺序记入 last_sweep_xy_ */
  void syncLastSweepXyFromList();

  /** 把已插入的 G/E 保护点写到黑板，供 RemovePassedGoals 使用 */
  void publishProtectedGarbage();

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
  /** true=非角点，false=角点；前驱用上一 goal，队首用机器人 */
  bool isGoalNotCorner(
    const Goals & goals,
    std::size_t idx,
    double robot_x, double robot_y) const;
  /** 从机器人沿路径找第一个角点下标；找不到返回 false */
  bool findFirstCornerFromRobot(
    const Goals & goals,
    double robot_x, double robot_y,
    std::size_t & corner_idx) const;
  /** 从 after_idx 之后找下一个角点；找不到返回 false */
  bool findNextCornerAfter(
    const Goals & goals,
    std::size_t after_idx,
    double robot_x, double robot_y,
    std::size_t & corner_idx) const;
  /** 从剩余队列头沿路径量 range_m 内末点 */
  bool findLastGoalWithinPathRange(
    const Goals & goals,
    double robot_x, double robot_y,
    double range_m,
    std::size_t & out_idx,
    std::size_t * nearest_seg_out = nullptr,
    std::size_t * start_idx_out = nullptr) const;

  /** 打印 garbage_list_ 当前内容，reason 为更新原因 */
  void logGarbageListState(const char * reason) const;

  /** 按开关往 RViz 发 Marker；同一任务内累加，新任务再清 */
  void publishVisualization(
    const InsertInfo & info,
    bool enable = true,
    bool viz_accepted_garbage = true,
    bool viz_deleted_goals = true,
    bool viz_ac_points = true);

  /** 新导航任务时清空本话题上全部 Marker */
  void clearMissionVisualization();
  void publishWorkCircle();
  void clearWorkCircle();
  /** 深绿工作圈 + 浅绿识别距离圈 */
  void publishRangeCircles(double robot_x, double robot_y);
  /** footprint 主动删点：黑圈 + 标签，与 clip 删点区分 */
  void publishFootprintStrippedMarkers();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::GarbageDetect>::SharedPtr garbage_sub_;
  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr special_terrain_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string garbage_topic_;
  std::string special_terrain_topic_;
  std::string footprint_topic_;
  std::string local_costmap_topic_;
  std::string visualization_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double arrived_radius_{0.5};
  double clip_extend_m_{1.0};
  double corner_angle_deg_{30.0};
  double goaltotal_range_m_{10.0};
  /** 离队头超过该距离就不删点，默认 4m */
  double head_delete_robot_dist_m_{4.0};
  /** 垃圾离机器人超过该距离则忽略，默认 5m */
  double max_garbage_robot_dist_m_{5.0};
  /** 垃圾周围该半径内有 lethal 障碍则丢弃，默认 0.7m */
  double min_garbage_obstacle_clearance_m_{0.7};
  /** 合堆半径：到种子小于该值并为一堆，默认 1.0m */
  double garbage_merge_radius_m_{1.0};
  /**
   * 沿 path_yaw 相对垃圾再插一点的距离，环境变量 GARBAGE_EXTEND_M。
   * 默认 2.0；
   */
  double garbage_extend_m_{2.0};
  double sweep_turn_weight_{0.5};
  double sweep_dist_weight_{0.5};
  double work_circle_radius_m_{10.0};
  bool has_work_circle_{false};
  double work_circle_x_{0.0};
  double work_circle_y_{0.0};
  /** 切向重构时找到的最近障碍格，RViz 画红矩形 + P{n} 文字（n 与 E{n} 同号） */
  struct VizObstaclePixel
  {
    double x{0.0};
    double y{0.0};
    int pile_num{0};
  };
  std::vector<VizObstaclePixel> viz_obstacle_pixels_;
  std::size_t viz_obstacle_marker_count_{0};
  double viz_obstacle_cell_m_{0.15};

  std::mutex history_mutex_;
  /** 原始接收缓存 */
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_;
  /** 后处理结果列表 */
  GarbageList garbage_list_;
  /** 已插入且 goals 里尚未扫过的堆；中途新堆只重排其中尚未开始的，正在扫的不重插 */
  GarbageList active_piles_;
  /** 从黑板接收到的完整 goals */
  Goals received_goals_;
  /** 当前认定的任务时间戳，与 goals 上统一 stamp 对齐 */
  rclcpp::Time mission_stamp_record_{0, 0, RCL_ROS_TIME};
  bool has_mission_stamp_{false};
  /** 本任务内已插入过、不再作为新候选的垃圾 map 坐标 */
  std::vector<std::pair<double, double>> reached_garbage_xy_;
  /** 本任务内已发布可视化的堆数 */
  int viz_pile_count_{0};
  /** footprint 删点位置，任务内累加，新任务清空 */
  struct FootprintStrippedVizPoint
  {
    double x{0.0};
    double y{0.0};
    std::string label;
  };
  std::vector<FootprintStrippedVizPoint> footprint_stripped_viz_;
  /** 本任务内各堆稳定 G 编号，避免删点全显示成 G1 */
  std::vector<std::pair<std::pair<double, double>, int>> g_num_xy_;
  /** E 点坐标 -> 所属 G 编号 */
  std::vector<std::pair<std::pair<double, double>, int>> e_num_xy_;
  int next_g_num_{1};
  /** 上一次发布可视化的时刻 */
  rclcpp::Time last_viz_time_{0, 0, RCL_ROS_TIME};
  /** 是否已发布过可视化 */
  bool has_last_viz_time_{false};
  /** 已插入且 goals 里尚未去掉的当前堆 */
  bool has_pending_garbage_{false};
  std::pair<double, double> pending_garbage_xy_{0.0, 0.0};
  /** 本周期 4-1：允许在 pending 未结束时插入新堆 */
  bool bypass_pending_insert_{false};
  /** 上次清扫顺序（map xy），供 4-1 保留其余相对次序 */
  std::vector<std::pair<double, double>> last_sweep_xy_;
  /** 上一堆假设到达点   有 E 用 E，否则用 G，供下一堆算 E；新任务/整单重排时清空 */
  bool has_last_sweep_arrive_{false};
  std::pair<double, double> last_sweep_arrive_xy_{0.0, 0.0};

  mutable std::mutex special_terrain_mutex_;
  /** 禁扫区多边形 */
  std::vector<geometry_msgs::msg::Polygon> special_terrain_polygons_;

  mutable std::mutex footprint_mutex_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_footprint_;

  mutable std::mutex local_costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_local_costmap_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INSERT_GARBAGE_POSE_ACTION_HPP_
