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
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "capella_ros_msg/msg/garbage_detect.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/msg/costmap.hpp"
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
  static constexpr std::size_t kMaxHistorySize = 15;
  /** garbage_list_ 最大长度 */
  static constexpr std::size_t kMaxGarbageSize = 6;
  /** 清扫顺序全排列上限，超过则贪心 */
  static constexpr std::size_t kSweepBruteMaxN = 6;
  /**
   * 按 xy 认「同一个已写入的点」的容差米：G/E 编号查找、已插入点保护用
   */
  static constexpr double kPointMatchDistanceM = 0.4;
  /** 认「同一颗」：按下标找到 z=-1 槽后，xy 只用来确认 3.1 vs 3.11，不拿来搜附近别的堆 */
  static constexpr double kSentinelIdentityMatchM = 0.05;
  /** 本节点约定：插入的 G/E 点 pose.position.z 固定写此值，表示无任务序号的哨兵点 */
  static constexpr double kGarbageSentinelPoseZ = -1.0;
  /** from 离 G 近于此则视为已到达：不用欧氏远近选侧，沿车头在 G 后方虚设来向 */
  static constexpr double kMinExtendFromDistM = 0.5;

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
    bool wall_edge_inserted{false};                       // 本堆是否走了贴边 D-G-E
    double wall_edge_d_x{0.0};                            // 贴边 D 点
    double wall_edge_d_y{0.0};
    std::vector<std::pair<double, double>> wall_edge_chain_xy;  // D…G…E 采样点（可视化）
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

  /** 贴边延长链：D、G、E 及中间点 */
  struct WallEdgeExtendChain
  {
    bool valid{false};
    std::string invalid_reason;
    std::vector<std::pair<double, double>> xy;
    double dx{0.0};
    double dy{0.0};
    double gx{0.0};
    double gy{0.0};
    double ex{0.0};
    double ey{0.0};
    double path_yaw{0.0};
    double extend_used_m{0.0};
    double px{0.0};
    double py{0.0};
    double normal_offset_m{0.0};
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
        "garbage_topic", std::string("/garbage_cord1"), "Garbage detection topic"),
      BT::InputPort<std::string>(
        "special_terrain_topic", std::string("/cleaning_tool_retraction_areas"),
        "Special / retraction area polygons topic"),
      BT::InputPort<std::string>(
        "footprint_topic", std::string("local_costmap/published_footprint"),
        "Robot footprint topic"),
      BT::InputPort<double>(
        "clip_extend_m", 2.5, "After garbage foot on path, delete goals for this distance (m)"),
      BT::InputPort<double>(
        "corner_angle_deg", 30.0, "Goals with turn angle above this are corners (deg)"),
      BT::InputPort<double>(
        "goaltotal_range_m", 10.0,
        "If no corner ahead, use last goal within this path distance as goalc (m)"),
      BT::InputPort<double>(
        "head_delete_robot_dist_m", 4.0,
        "Skip all goal deletes when the robot is farther than this (m) from the nearest point on input_goals"),
      BT::InputPort<double>(
        "max_garbage_robot_dist_m", 5.0,
        "Ignore garbage farther than this distance (m) from robot (anti false-detect)"),
      BT::InputPort<double>(
        "garbage_merge_radius_m", 1.0,
        "Merge detections within this radius (m) of the nearest seed into one pile"),
      BT::InputPort<double>(
        "garbage_extend_m", 2.0,
        "Along path_yaw, insert E this far past garbage (m)"),
      BT::InputPort<double>(
        "extend_max_yaw_deg", 60.0,
        "If default E is blocked, sweep this many degrees left/right (deg)"),
      BT::InputPort<double>(
        "extend_step_yaw_deg", 10.0,
        "Yaw step when sweeping left/right for a clear E (deg)"),
      BT::InputPort<double>(
        "work_circle_radius_m", 10.0,
        "Accept new garbage only inside this radius around the robot pose when the first pile of a batch is accepted"),
      BT::InputPort<bool>(
        "single_pile_insert", true,
        "If true, accept and insert one pile; while its G or E remains in the queue, drop new detections"),
      BT::InputPort<double>(
        "sweep_dist_weight", 0.5,
        "Sweep order score weight on total travel distance (2.7.3)"),
      BT::InputPort<double>(
        "sweep_turn_weight", 0.5,
        "Sweep order score weight on total |yaw| turned (2.7.3)"),
      BT::InputPort<double>(
        "wall_edge_d_extend_m", 2.0,
        "Wall-edge: each D push step along tangent (m)"),
      BT::InputPort<double>(
        "wall_edge_e_extend_m", 2.0,
        "Wall-edge: G to E extend distance along tangent (m)"),
      BT::InputPort<double>(
        "wall_edge_min_robot_dist_m", 3.0,
        "Wall-edge: keep |D-robot| at least this (m)"),
      BT::InputPort<double>(
        "wall_edge_sample_m", 0.5,
        "Wall-edge: spacing of points on D-E (m)"),
      BT::InputPort<double>(
        "wall_edge_normal_offset_m", 0.0,
        "Wall-edge: shift whole D-G-E along obstacle->garbage normal (m), + away from wall"),
      BT::InputPort<std::string>(
        "global_costmap_topic", std::string("global_costmap/costmap_raw"),
        "Global costmap topic (nav2_msgs/Costmap) for all footprint checks"),
      BT::InputPort<bool>(
        "enable_visualization", true, "Publish insert/clip markers to RViz"),   //总开关
      BT::InputPort<bool>(
        "viz_accepted_garbage", true, "Show accepted garbage after filtering"),
      BT::InputPort<double>(
        "confirm_match_dist_m", 1.0,
        "Max Euclidean distance (m) among confirm frames to count as same pile; <=0 disables multi-frame confirm"),
      BT::InputPort<int>(
        "confirm_match_num", 2,
        "Accept garbage after this many frames within confirm_match_dist_m; pose is their average"),
      BT::InputPort<double>(
        "confirm_sec_garbage_time", 1.0,
        "Drop tmp_list_ confirm entries older than this (s); <=0 disables age-based drop"),
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

  /** 垃圾检测话题回调：转到 map；单堆占用中直接丢弃新消息 */
  void garbageDetectCallback(const capella_ros_msg::msg::GarbageDetect::SharedPtr msg);

  /** 特殊清扫/禁扫区域话题回调 */
  void special_terrain_callback(const garage_utils_msgs::msg::Polygons::SharedPtr msg);

  /** footprint 话题回调：写入缓存，并喂给 Nav2 FootprintSubscriber */
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  /** 全局代价图话题回调：喂给 Nav2 CostmapSubscriber，供 tick 内 spin 后立刻可用 */
  void globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

  /** 从话题取一次 footprint，缓存在节点里；车长 = base 下 max_x-min_x */
  bool ensureCachedFootprint(std::string * reason) const;

  /**
   * 2.11：把车体轮廓放到 (x,y,yaw)，交给 nav2 的
   * CostmapTopicCollisionChecker::isCollisionFree 判定，只查全局代价图。
   * 图外 / unknown / 致命障碍为不通过。
   */
  bool isCollisionFreeAtPose(
    double x, double y, double yaw, std::string * reason,
    bool fetch_costmap_and_footprint = true) const;

  /**
   * 2.11 footprint 长条：把两端点连成一条线，车体轮廓沿线按车长步进摆放，
   * 首尾都检，朝向取连线方向，等价于扫出一个大长条矩形。
   * 垃圾点检查、延长点检查、线段检查都走这一条。
   */
  bool isFootprintSweepClear(
    double x0, double y0, double x1, double y1, std::string * reason) const;

  /**
   * 在全局代价图上找离 (x,y) 最近的致命障碍格
   */
  bool findNearestObstaclePixel(double x, double y, double * ox, double * oy);

  /** 接收到垃圾后的后处理函数，返回处理后的 garbage_list */
  GarbageList postProcessHistory();

  /** 接收完整的 {goals} 路径点 */
  Goals receiveGoals();

  /** 对比 goals 时间戳，外部重发任务时清空 history 和 garbage */
  void checkAndResetOnNewMission();

  /** 取机器人当前位姿*/
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;

  /** 取机器人当前位姿的 xy*/
  bool getRobotPoseXY(double & x, double & y, double * yaw = nullptr) const;

  /** 获取缓存的 footprint 在 base_link 下的顶点 */
  bool getRobotFootprintInBase(
    std::vector<std::pair<double, double>> & local_xy) const;

  /**
   * 判断该 goal 是否为本节点插入的 G/E 哨兵点，而不是带序号的普通途经点。
   */
  static bool isUnindexedSentinelPoseZ(
    const geometry_msgs::msg::PoseStamped & pose_stamped_goal);

  /** 本任务内按 xy 分配稳定 G 编号，重插同一堆不改号 */
  int assignStableGNum(double x, double y);
  int lookupStableGNum(double x, double y) const;
  /** E 点坐标登记所属 G 编号 */
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
   * 自己不删点，只认「哨兵已经被擦掉」这个结果。
   */
  std::size_t stripReachedZNeg1Goals(
    const Goals & goals,
    std::string * deleted_summary = nullptr);

  /**
   * 扫完判定：footprint 盖过 / 沿 G→E 经过 G，并且盖过 / 离开 E（无 E 则只看 G）。
   * 只用来从 {goals} 删掉这对哨兵，不用来判断「这堆开没开扫」。
   */
  bool isPointCoveredByRobotFootprint(
    double x, double y, const geometry_msgs::msg::PoseStamped & robot_pose) const;
  static bool isPastAlongDirection(
    double rx, double ry, double px, double py, double dir_x, double dir_y);
  bool eraseSweptSentinelsFromGoals(
    Goals & goals, const geometry_msgs::msg::PoseStamped & robot_pose);

  /** 每次 setOutput("output_goals") 时打日志，便于观察时机与频率 */
  void emitOutputGoals(const Goals & goals, const char * reason);

  /** 紧凑打印 goals：(x,y) 或 (x,y,-1) */
  std::string formatGoalsListCompact(const Goals & goals) const;

  /** 获取插入所需的全部信息并返回 */
  InsertInfo gatherInsertInfo(
    const Goals & goals,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double garbage_x, double garbage_y);

  /**
   * 原路径已被删光、A-C 撞点时：优先用已插入的 G-E 当投影线，否则车→剩余原路径 / 车→G。
   * 只补几何线，不否决插入。
   */
  bool fillAcWhenOriginalPathGone(InsertInfo * info, double gx, double gy) const;

  /**
   * 若 G→yaw 与 G→机器人同侧（点积>0），翻转 yaw，避免 E 落在车同侧。
   * 车贴 G 时不改。
   */
  static double preferExtendYawAwayFromRobot(
    double gx, double gy, double yaw,
    double robot_x, double robot_y);

  /**
   * 2.5 当前边只有一条：第一个非 z=-1 点 → 其后第一个角点。只向这条边投影。
   * 落在边上：从该点沿路径删到垂足再加 clip_extend_m；先碰到角点则停在角点并留下角点，然后停止。
   * 反延不删。正延删掉该边除尾角点外的点，重算角点和当前边后再投影。
   * 多个参考点按顺序共用一份路径副本，后一个看得到前一个删完后的新角点。
   * 下标记在调用方传入的 goals 上，由调用方一次删除。
   */
  Goals clipGoalsNearGarbage(InsertInfo & info);

  /** 按顺序对每个参考点跑 2.5，删除下标写入 delete_idx，不改 goals 本身。 */
  void clipReferencesInOrder(
    const Goals & goals,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::vector<std::pair<double, double>> & refs,
    std::set<std::size_t> & delete_idx,
    InsertInfo & info,
    bool commit_memory);

  /** 删点后变成队首的角点：车还没开上下一条边时，它仍是当前边终点 */
  bool isRememberedCorner(double x, double y) const;
  void rememberCornerXy(double x, double y) const;
  void forgetCornerXy(double x, double y) const;
  bool robotEnteredNextSide(
    const Goals & goals, std::size_t head, double robot_x, double robot_y) const;

  /** 插入真实垃圾、统一时间戳；G-E 接到剩余路径队首，角点和对边留下 */
  Goals insertGarbageIntoGoals(InsertInfo & info);

  /** 2.11：单点 footprint 能否落在该位姿 */
  bool isFootprintClearAtPose(
    double x, double y, double yaw, std::string * reason) const;

  /** 生成贴边 D、G、E 及 D-E 间隔点，含法向偏移 */
  WallEdgeExtendChain buildWallEdgeExtendChain(const InsertInfo & info);

  /** 贴墙：把延长链写入 goals */
  Goals insertWallEdgeGarbageIntoGoals(InsertInfo & info);

  /** 把单个垃圾从 base_link 转到 map */
  bool transformGarbageToMap(capella_ros_msg::msg::GarbageDetect & garbage) const;
  /** 判断点是否在禁扫区域内 */
  bool isPointInSpecialTerrain(double x, double y) const;
  /** 判断点是否在多边形内 */
  static bool isPointInPolygon(
    double x, double y, const geometry_msgs::msg::Polygon & polygon);
  /** 2.12.6 新种子是否与已有种子重复：按 garbage_merge_radius_m */
  bool isDuplicateOfKept(
    const capella_ros_msg::msg::GarbageDetect & garbage,
    const GarbageList & kept) const;
  /** 2.12.7 是否落在已处理过、不再插入的垃圾附近：按 garbage_merge_radius_m */
  bool isNearReachedGarbage(double x, double y) const;
  /** 该 xy 是否是已写入 goals 的 G/E 点，用 kPointMatchDistanceM 做几何认点 */
  bool isProtectedGarbageXy(double x, double y) const;
  void addProtectedGarbageXy(double x, double y);
  void eraseProtectedGarbageXy(double x, double y);
  /**
   * 队首第一对还在 goals 里的 G/E。车还在去 G 的路上，或 G 已出队只剩 E，都算当前堆。
   * 中途新堆只重排这对后面的，不把它剥掉。
   */
  bool collectInProgressKeepXy(
    const Goals & goals,
    std::vector<std::pair<double, double>> * keep_xy,
    int * keep_g_num) const;

  /** 平面距离平方 */
  static double squaredDistanceXY(
    double x1, double y1, double x2, double y2);
  /** 写入 garbage_list_，满了时优先保留离机器人更近的 */
  bool tryInsertPreferCloserToRobot(
    capella_ros_msg::msg::GarbageDetect garbage,
    double robot_x, double robot_y);

  /**
   * 合堆,一次性能扫掉的
   */
  static GarbageList mergeGarbagePiles(
    const GarbageList & candidates,
    double robot_x, double robot_y,
    double merge_radius_m,
    std::vector<std::vector<std::size_t>> * groups_out = nullptr);

  /**
   * 中间堆的清扫顺序。起点是 first_garbage 的延长点。
   * locked_last 非空时固定为队尾，参与延长点评分，不参与中间排列。
   */
  std::vector<std::size_t> computeSweepOrder(
    const GarbageList & garbage_list,
    double robot_x, double robot_y,
    double robot_yaw,
    const capella_ros_msg::msg::GarbageDetect * locked_last = nullptr);

  /**
   * 队首是车头前方最近的一堆，正前方没有则用离车最近的。
   * 队尾按 2.7.1 在原始路径副本上跑 2.5，取删除下标最大的一堆。
   * 中间按 computeSweepOrder 重排，评分含队尾延长点。
   */
  void reorderNearestFirstThenSweep(
    double robot_x, double robot_y, double robot_yaw);

  /** 超过 kMaxGarbageSize 时截断，保留离机器人最近的若干堆 */
  void trimGarbageListToCap(double robot_x, double robot_y);

  /** 相对 before，找出本轮新入队的堆下标 */
  bool findNewGarbageIndex(
    const GarbageList & before,
    double robot_x, double robot_y,
    std::size_t & new_idx) const;

  /** 把当前 garbage_list_ 顺序记入 last_sweep_xy_ */
  void syncLastSweepXyFromList();

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
  /** true=非角点，false=角点：z=-1 不能当角点；仅非 z=-1 点看夹角，最后一个普通途经点收尾 */
  bool isGoalNotCorner(
    const Goals & goals,
    std::size_t idx,
    double robot_x, double robot_y) const;
  /** 剩余队列第一个非 z=-1 点，工字当前长边队首 */
  std::size_t ordinaryQueueHead(const Goals & goals) const;
  /** 从当前长边队首往后找第一个角点；找不到返回 false */
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

  /** 插入一堆时画 G/E、蓝虚线、footprint 长条；首点/角点另刷 */
  void publishVisualization(
    const InsertInfo & info,
    bool enable = true,
    bool viz_accepted_garbage = true);

  /** 清空本话题上全部 Marker，并丢掉本节点可视化状态 */
  void clearMissionVisualization();
  void resetVisualizationState();
  /** 深绿工作圈 + 浅绿跟随/识别圈 */
  void publishRangeCircles(double robot_x, double robot_y);
  /** footprint 检查不通过时，把当时检查用的 footprint 框画在检查位置上 */
  void publishFootprintCheckBox(double x, double y, double yaw);
  /** 走廊检查失败：红球 + 当时检查的 footprint 长条 */
  void publishFailedSweepVisualization(double rx, double ry, double gx, double gy);
  /** 待插入 garbage_list_ 的红色小球，多了删旧 id */
  void publishPendingGarbageDots();
  /** 当前路径首点 H、角点 C：每次按最新判断覆盖，不是角点了就删 */
  void refreshClipAnchorVisualization(
    const Goals & goals, double robot_x, double robot_y);
  /** G/E 都不在 goals 里时，删掉这堆的球、虚线、长条 */
  void pruneFinishedPileVisualization(const Goals & goals);
  void deletePileVisualization(int pile_num);
  void appendDeleteMarker(
    visualization_msgs::msg::MarkerArray & arr, const std::string & ns, int id);
  void appendFootprintStripMarkers(
    visualization_msgs::msg::MarkerArray & arr,
    const std::string & ns, int id_base,
    double x0, double y0, double x1, double y1,
    float r, float g, float b, float a_line, float a_fill);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  /** 把本节点 callback group 收到的 footprint 喂进 Nav2 FootprintSubscriber */
  class FeedableFootprintSubscriber : public nav2_costmap_2d::FootprintSubscriber
  {
public:
    using FootprintSubscriber::FootprintSubscriber;
    void feed(const geometry_msgs::msg::PolygonStamped::SharedPtr & msg)
    {
      footprint_callback(msg);
    }
  };

  rclcpp::Subscription<capella_ros_msg::msg::GarbageDetect>::SharedPtr garbage_sub_;
  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr special_terrain_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_feed_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_feed_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<FeedableFootprintSubscriber> footprint_topic_sub_;
  mutable std::unique_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  std::string garbage_topic_;
  std::string special_terrain_topic_;
  std::string footprint_topic_;
  std::string global_costmap_topic_;
  std::string visualization_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double clip_extend_m_{2.5};
  double corner_angle_deg_{30.0};
  double goaltotal_range_m_{10.0};
  /** 离队头超过该距离就不删点，默认 4m */
  double head_delete_robot_dist_m_{4.0};
  /** 垃圾离机器人超过该距离则忽略，默认 5m */
  double max_garbage_robot_dist_m_{5.0};
  /** 贴边：D 每次沿切向再推的步长，默认 2m */
  double wall_edge_d_extend_m_{2.0};
  /** 贴边：G 到 E 沿切向伸出长度，默认 2m */
  double wall_edge_e_extend_m_{2.0};
  /** 贴边：D 与车最小距离 GD，默认 3m */
  double wall_edge_min_robot_dist_m_{3.0};
  /** 贴边：D-E 插点间隔，默认 0.5m */
  double wall_edge_sample_m_{0.5};
  /** 贴边：整链沿障碍→垃圾法向平移，正为离墙，默认 0 */
  double wall_edge_normal_offset_m_{0.0};
  /** 合堆半径：到种子小于该值并为一堆，默认 1.0m */
  double garbage_merge_radius_m_{1.0};
  /** 多帧确认：同堆欧氏距离上限，默认 1.0m；<=0 关闭多帧确认 */
  double confirm_match_dist_m_{1.0};
  /** 多帧确认：需凑够的帧数，默认 2；位姿取这些帧的平均 */
  int confirm_match_num_{2};
  /** tmp_list_ 待确认条目最长保留时间 (s)，默认 1.0；<=0 不按时间删除 */
  double confirm_sec_garbage_time_{1.0};
  /** 沿 path_yaw 相对垃圾再插一点的距离，默认 2.0m */
  double garbage_extend_m_{2.0};
  /** 默认 E 不通时，左右各扫到此角度，默认 60° */
  double extend_max_yaw_deg_{60.0};
  /** 扇形扫角步长，默认 10° */
  double extend_step_yaw_deg_{10.0};
  /** 2.7.3 排序得分权重：sum_score = dist_w * total_dist_m + turn_w * total_yaw_rad */
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
  /** 单堆：当前 G/E 还在 goals 里时，话题新垃圾不进 history / 待确认 / 待插列表 */
  bool single_pile_block_intake_{false};
  /** 当前占用堆的扫完闩：到 G、到 E 分开记，换堆或新任务清掉 */
  int sweep_latch_g_num_{0};
  bool sweep_seen_g_{false};
  bool sweep_seen_e_{false};
  /** 原始接收缓存 */
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_;
  /** 时间窗内待确认的垃圾位姿，满 kMaxHistorySize 丢最旧 */
  std::deque<capella_ros_msg::msg::GarbageDetect> tmp_list_;
  /** 后处理结果列表 */
  GarbageList garbage_list_;
  /** 已插入且 goals 里尚未扫过的堆；中途新堆只重排其中尚未开始的，正在扫的不重插 */
  GarbageList active_piles_;
  /** 当前认定的任务时间戳，与 goals 上统一 stamp 对齐 */
  rclcpp::Time mission_stamp_record_{0, 0, RCL_ROS_TIME};
  bool has_mission_stamp_{false};
  /** 本任务内已插入过、不再作为新候选的垃圾 map 坐标 */
  std::vector<std::pair<double, double>> reached_garbage_xy_;
  /** 本任务内已发布可视化的堆数 */
  int viz_pile_count_{0};
  /** 已画出的 footprint 检查失败框数量 */
  std::size_t viz_footprint_fail_count_{0};
  /** 已插入、还在画的 G/E，走完对应哨兵后删 */
  struct VizPileTrack
  {
    int pile_num{0};
    double gx{0.0};
    double gy{0.0};
    bool has_e{false};
    double ex{0.0};
    double ey{0.0};
  };
  std::vector<VizPileTrack> viz_tracks_;
  std::size_t viz_pending_marker_count_{0};
  bool viz_have_head_{false};
  bool viz_have_corner_{false};
  bool viz_have_fail_strip_{false};
  /** 本任务内各堆稳定 G 编号，避免删点全显示成 G1 */
  std::vector<std::pair<std::pair<double, double>, int>> g_num_xy_;
  /** E 点坐标 -> 所属 G 编号 */
  std::vector<std::pair<std::pair<double, double>, int>> e_num_xy_;
  int next_g_num_{1};
  /** 上次清扫顺序（map xy），供 4-1 保留其余相对次序 */
  std::vector<std::pair<double, double>> last_sweep_xy_;
  /** 上一堆假设到达点   有 E 用 E，否则用 G，供下一堆算 E；新任务/整单重排时清空 */
  bool has_last_sweep_arrive_{false};
  std::pair<double, double> last_sweep_arrive_xy_{0.0, 0.0};
  /** 上一堆扫向 path_yaw；到达点贴下一 G 时延续此朝向，避免退回车头导致 E∥车 */
  bool has_last_sweep_path_yaw_{false};
  double last_sweep_path_yaw_{0.0};
  /** 本任务里保留过的角点。它变成队首、且没有前一个普通点时，仍先当作 C */
  mutable std::vector<std::pair<double, double>> remembered_corner_xy_;

  mutable std::mutex special_terrain_mutex_;
  /** 禁扫区多边形 */
  std::vector<geometry_msgs::msg::Polygon> special_terrain_polygons_;

  mutable std::mutex footprint_mutex_;
  mutable bool have_cached_footprint_{false};
  /** 收到新轮廓置位，下次用时重取；取不到就继续用上一份缓存 */
  mutable bool footprint_dirty_{true};
  mutable std::vector<std::pair<double, double>> cached_footprint_base_;
  mutable double cached_robot_length_m_{0.5};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INSERT_GARBAGE_POSE_ACTION_HPP_
