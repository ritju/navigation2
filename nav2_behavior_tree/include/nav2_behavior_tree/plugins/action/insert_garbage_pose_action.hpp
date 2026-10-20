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
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
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
  /** goals 哨兵 / 已保护 G·E 坐标对齐容差 (m) */
  static constexpr double kSentinelIdentityMatchM = 0.1;
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
    std::vector<std::pair<double, double>> ray_chain_xy;  // 一条锁住的线：G1…Gn，只有链尾生成 E
    bool wall_edge_inserted{false};                       // 本堆是否走了贴边 D-G-E
    double wall_edge_d_x{0.0};                            // 贴边 D 点
    double wall_edge_d_y{0.0};
    bool wall_edge_has_d{false};                          // 这条贴墙线是否真有 D
    bool wall_edge_has_p{false};                          // 贴边用到的最近障碍格
    double wall_edge_px{0.0};
    double wall_edge_py{0.0};
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
    bool has_d{false};
    bool has_e{false};
  };

  /**
   * 一次定稿的清扫：可选 D、一串 G、可选 E。
   * 排序阶段算完，插入阶段按这里原样写入，不再重算贴墙或链尾 E。
   */
  struct SweepUnit
  {
    GarbageList piles;
    bool has_d{false};
    double dx{0.0};
    double dy{0.0};
    std::vector<std::pair<double, double>> gxy;
    bool has_e{false};
    double ex{0.0};
    double ey{0.0};
    double from_x{0.0};
    double from_y{0.0};
    double yaw{0.0};
    bool has_p{false};
    double px{0.0};
    double py{0.0};
    int head_label{0};
  };

  /**
   * 假设这一堆的清扫段。先直线 E，车体不过再左右偏头，仍不过且贴墙开着才生成 D、E。
   * 没生成的点不参与连线：有 D 有 E 用 D→E，只有 E 用 G→E，只有 D 用 D→G，都没有则只有 G。
   */
  struct AssumedSweep
  {
    bool has_d{false};
    bool has_e{false};
    double dx{0.0};
    double dy{0.0};
    double gx{0.0};
    double gy{0.0};
    double ex{0.0};
    double ey{0.0};
    double yaw{0.0};
    bool has_p{false};
    double px{0.0};
    double py{0.0};

    void endpoints(double & sx, double & sy, double & tx, double & ty) const
    {
      if (has_d && has_e) {
        sx = dx;
        sy = dy;
        tx = ex;
        ty = ey;
      } else if (has_e) {
        sx = gx;
        sy = gy;
        tx = ex;
        ty = ey;
      } else if (has_d) {
        sx = dx;
        sy = dy;
        tx = gx;
        ty = gy;
      } else {
        sx = gx;
        sy = gy;
        tx = gx;
        ty = gy;
      }
    }
  };

  AssumedSweep assumeSweepSegment(
    double from_x, double from_y, double from_yaw, double gx, double gy);

  InsertGarbagePose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Input goals list (e.g. {goals})"),
      BT::OutputPort<Goals>("output_goals", "Goals after inserting garbage poses"),
      BT::InputPort<std::string>(
        "garbage_topic", std::string("/garbage_rectangle_points"), "Garbage detection topic"),
      BT::InputPort<std::string>(
        "special_terrain_topic", std::string("/cleaning_tool_retraction_areas"),
        "Special / retraction area polygons topic"),
      BT::InputPort<std::string>(
        "footprint_topic", std::string("local_costmap/published_footprint"),
        "Robot footprint topic"),
      BT::InputPort<double>(
        "clip_extend_m", 2.5,
        "From each reference foot, delete following goals within this path distance (m)"),
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
        "G to E distance: normal insert along path_yaw; wall-edge along tangent (m)"),
      BT::InputPort<double>(
        "ray_offset_deg", 30.0,
        "Half-angle (deg) of the forward ray sector that binds garbage into one locked line"),
      BT::InputPort<double>(
        "extend_near_radius_m", 1.5,
        "If another pile is within this radius (m) of E, nudge E away or bind a line"),
      BT::InputPort<double>(
        "extend_extra_m", 1.0,
        "Extra length (m) added to garbage_extend_m when nudging E away from a nearby pile"),
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
      BT::InputPort<bool>(
        "enable_wall_edge_insert", true,
        "If true and robot->G footprint fails at insert, try wall-edge D-G-E (independent of single_pile_insert)"),
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
        "wall_edge_min_robot_dist_m", 3.0,
        "Wall-edge: keep |D-robot| at least this (m)"),
      BT::InputPort<double>(
        "wall_edge_sample_m", 0.5,
        "Wall-edge: sample spacing along D-G-E (m)"),
      BT::InputPort<double>(
        "wall_edge_normal_offset_m", 0.0,
        "Wall-edge: after D-G-E is built, shift the whole chain along P->G (m), + away from wall"),
      BT::InputPort<std::string>(
        "global_costmap_topic", std::string("global_costmap/costmap_raw"),
        "Global costmap topic (nav2_msgs/Costmap) for all footprint checks"),
      BT::InputPort<bool>(
        "enable_visualization", true, "Master switch for all insert-garbage RViz markers"),
      BT::InputPort<double>(
        "confirm_match_dist_m", 1.0,
        "Max Euclidean distance (m) among confirm frames to count as same pile"),
      BT::InputPort<int>(
        "confirm_match_num", 2,
        "Accept garbage after this many frames within confirm_match_dist_m; pose is their average"),
      BT::InputPort<double>(
        "confirm_sec_garbage_time", 1.0,
        "Drop tmp_list_ confirm entries older than this (s)"),
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

  /** 从行为树输入口读取可调参数到成员变量 */
  void getBehaviorTreeParameters();

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
   * 在全局代价图上找离 (x,y) 最近的致命障碍格，搜索半径 search_radius_m
   */
  bool findNearestObstaclePixel(
    double x, double y, double search_radius_m, double * ox, double * oy);

  /**
   * 2.12.4 贴墙进表：robot→garbage 连线在代价图上无致命障碍（不含垃圾格本身）。
   * 比 footprint 长条宽松，用于 enable_wall_edge_insert 时的候选筛选。
   */
  bool isRobotGarbageSegmentClearOfLethal(
    double robot_x, double robot_y, double gx, double gy,
    std::string * reason = nullptr) const;

  /** 接收到垃圾后的后处理函数，返回处理后的 garbage_list */
  GarbageList postProcessHistory();

  /** 接收完整的 {goals} 路径点 */
  Goals receiveGoals();

  /** 队首 stamp 变了视为新任务，清空本节点状态并更新 mission_stamp_record_ */
  void isNewMission();

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

  /** goals 里是否还有本节点插入的 z=-1 哨兵点 */
  static bool hasGoalsZpoints(const Goals & goals);

  /**
   * 按下标找这堆在 {goals} 里的槽：该格 z=-1，xy 仅确认同一颗。
   * 找到返回 true 并写出 index；没有任何一格对上返回 false。
   */
  bool findUnindexedSentinelIndex(
    const Goals & goals, double x, double y, std::size_t * index_out) const;

  /** 每次 setOutput("output_goals") 时打日志，便于观察时机与频率 */
  void emitOutputGoals(const Goals & goals, const char * reason);

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

  /** 每个参考点都在原始路径上跑 2.5，删除下标取并集写入 delete_idx，不改 goals 本身。 */
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

  /** 按最后一个已保护点或 z=-1 哨兵把 goals 切成前缀和剩余路径 */
  void splitAtLastProtected(const Goals & goals, Goals * prefix, Goals * path) const;

  /**
   * 本批删点用的尾巴还在 goals 末尾时，前缀只取尾巴之前的点。
   * 对不上时退回 splitAtLastProtected。
   */
  void splitFrozenTail(
    const Goals & goals, const Goals * frozen_tail,
    Goals * prefix, Goals * path) const;

  /**
   * 有前缀时在剩余路径上重算删点基准。
   * saved_garbage 非空时写进基准，贴墙和普通插入需要；整线插入传空。
   */
  bool prepareClipBase(
    const Goals & prefix, const Goals & path,
    double gx, double gy, double saved_yaw,
    const capella_ros_msg::msg::GarbageDetect * saved_garbage,
    InsertInfo * info, InsertInfo * frozen);

  /**
   * 用参考点判断剩余路径，把本堆要删的点写进 info->goaltotal。
   * defer_delete 非空时只把原始下标并进该集合，返回的路径不删点。
   */
  Goals clipWithRefs(
    const Goals & work,
    const std::vector<std::pair<double, double>> & refs,
    const InsertInfo & frozen,
    InsertInfo * info,
    const char * log_tag,
    std::set<std::size_t> * defer_delete);

  /** 接射线的新 G、新 E 对这份尾巴做一次并集，下标只写入 pending_delete */
  void deferOpenRayClip(
    const Goals & clip_tail,
    const std::vector<std::pair<double, double>> & refs,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    std::set<std::size_t> * pending_delete);

  /**
   * pending_delete 的下标对着 clip_tail。
   * goals 末尾仍是这份尾巴时按并集删一次，对不上则不删。返回删掉的点数。
   */
  std::size_t applyDeferredTailDelete(
    Goals * goals,
    const Goals & clip_tail,
    const std::set<std::size_t> & pending_delete);

  /** 前缀、新插入点、剩余路径拼成一条，并统一时间戳 */
  Goals assembleAndStamp(
    const Goals & prefix, const Goals & inserted, const Goals & rest);

  /** 记下这一堆扫完后的离开点和朝向，供下一堆算来向 */
  void recordSweepArrive(double x, double y, double yaw);

  enum class InsertLoopAction { Continue, BreakLoop };

  struct InsertBatch
  {
    Goals goals;
    std::size_t inserted_count{0};
    std::size_t deleted_goals_total{0};
    std::ostringstream inserted_xy;
    /** 本批开始时的普通路径，各堆的删点下标都对着它 */
    Goals clip_tail;
    /** 各堆判断完的并集，本批插完后再从 clip_tail 上删一次 */
    std::set<std::size_t> pending_delete;
  };

  /** 2.11：单点 footprint 能否落在该位姿 */
  bool isFootprintClearAtPose(
    double x, double y, double yaw, std::string * reason) const;

  /** 生成贴边 D、G、E 及 D-E 间隔点，含法向偏移 */
  WallEdgeExtendChain buildWallEdgeExtendChain(const InsertInfo & info);

  /** 把单个垃圾从 base_link 转到 map */
  bool transformGarbageToMap(capella_ros_msg::msg::GarbageDetect & garbage) const;
  /** 判断点是否在禁扫区域内 */
  bool isPointInSpecialTerrain(double x, double y) const;
  /** 判断点是否在多边形内 */
  static bool isPointInPolygon(
    double x, double y, const geometry_msgs::msg::Polygon & polygon);
  /** True if garbage is within garbage_merge_radius_m_ of any pile in existing. */
  bool isDuplicateGarbage(
    const capella_ros_msg::msg::GarbageDetect & garbage,
    const GarbageList & existing) const;
  /** 2.12.7 是否落在已处理过、不再插入的垃圾附近：按 garbage_merge_radius_m */
  bool isNearReachedGarbage(double x, double y) const;
  /** 该 xy 是否已在 reached_garbage_xy_（与插入 G/E 同 0.1m 容差） */
  bool isProtectedGarbageXy(double x, double y) const;
  void addProtectedGarbageXy(double x, double y);

public:
  /** 平面距离平方。排序用的自由函数在类外，需要能直接调用 */
  static double squaredDistanceXY(
    double x1, double y1, double x2, double y2);

private:
  /** 写入 garbage_list_，满了时优先保留离机器人更近的 */
  bool tryInsertPreferCloserToRobot(
    capella_ros_msg::msg::GarbageDetect garbage,
    double robot_x, double robot_y);

  /**
   * 合堆：每堆从离车最近的点开堆，代表坐标为成员质心
   */
  static GarbageList mergeGarbagePiles(
    const GarbageList & candidates,
    double robot_x, double robot_y,
    double merge_radius_m,
    std::vector<std::vector<std::size_t>> * groups_out = nullptr);

  /**
   * 暴力排序并返回新列表，不改 garbage_list_。
   * 正方向最近的一堆固定为队首；正方向没有则不锁队首。
   * 队尾取删点最远的一堆。中间全排列，按延长点路程和转角打分。
   */
  GarbageList forcegarbagesort(
    GarbageList piles, double pose_x, double pose_y, double pose_yaw);

  /** 点相对射线起点是否落在正方向、且夹角不超过 ray_offset_deg_ */
  bool inForwardRaySector(
    double ox, double oy, double tx, double ty, double px, double py) const;

  /**
   * 点到有向直线的夹角不超过 ray_offset_deg_（正反方向都算）。
   * proj_out 为沿 yaw 的投影，起点为 0，正方向为正。
   */
  bool onOpenRayAxis(
    double ox, double oy, double yaw, double px, double py, double * proj_out) const;

  /**
   * 反向延长只认 from 到链头这一段。
   * 车已经开上这段时，起点收到车的投影；车和 from 之间算已开过。
   * 落在 from 后面、或 G 前方的点不在这段里。
   * 参数拆开传，避免声明出现在 OpenGarbageRay 定义之前。
   */
  bool onReverseApproachSegment(
    double gx, double gy, double yaw, double fx, double fy,
    double px, double py, double robot_x, double robot_y,
    double * proj_out) const;

  /** 链被延长、新链头插到最前面之后，这条链上每个 G 的 from 改成新链头前面那个点 */
  void updateOpenRayFrom(double gx, double gy, double fx, double fy);

  /**
   * 种子一定进链。其余下标落在原点指向 (tx,ty) 的扇形里才进链。
   * 按离原点由近到远排序。除种子外没有别人时返回空。
   */
  std::vector<std::size_t> collectSectorChain(
    double ox, double oy, double tx, double ty,
    std::size_t count, std::size_t seed,
    const std::function<std::pair<double, double>(std::size_t)> & at) const;

  /** 打印已规划的清扫单元，一行一个 */
  void logSortedInsertList(double pose_x, double pose_y, double pose_yaw);

  /**
   * 多堆排序。先反复收射线链，收不出再暴力排序，再沿理想 E 收线。
   * 结果写入 planned_units_，garbage_list_ 按单元顺序展开。
   */
  void garbageSortTotal(double pose_x, double pose_y, double pose_yaw);

  /** 规划全部清扫单元：链、暴力排序、再锁链、剩下的单堆 */
  void planSweepUnits(double pose_x, double pose_y, double pose_yaw);

  /** 有 E 用 G→E，否则有 D 用 D→G，都没有则上一个点指向 G */
  static void rayFromAssumed(
    const AssumedSweep & sweep, double from_x, double from_y,
    double * ox, double * oy, double * tx, double * ty);

  /** 正前方最近的一堆；正前方没有则离当前点最近。距离用 G */
  std::size_t pickSeedIndex(
    const GarbageList & piles, double pose_x, double pose_y, double pose_yaw) const;

  /**
   * 以 seed_idx 为链头组一条链。入链看 G。
   * 射线：有 E 用 G→E，否则有 D 用 D→G，都没有则上一个点指向 G。
   * 相邻 G 扫不过就截断；链尾 E 直伸和偏头都不过就把链尾放回剩余。
   * 成功时从 remaining 删掉链上的堆。不足两堆返回 false，不改 remaining。
   */
  bool buildChainFromSeed(
    GarbageList * remaining, std::size_t seed_idx,
    double pose_x, double pose_y, double pose_yaw, SweepUnit * unit);

  /** 单堆：直伸、偏头、贴墙。E 离其余堆太近只偏开 */
  SweepUnit planSingleUnit(
    const capella_ros_msg::msg::GarbageDetect & pile,
    double pose_x, double pose_y, double pose_yaw,
    const GarbageList & others);

  /** 链尾 E：沿前一堆指向本堆直伸，不过再偏头。附近还有垃圾则只偏开 E */
  bool makeChainTailExtend(
    double prev_x, double prev_y, double gx, double gy,
    const GarbageList & others,
    double * ex, double * ey, double * yaw) const;

  /** 有 E 用 E，否则用最后一个 G，作为下一单元的上一个点 */
  static void advancePoseFromUnit(
    const SweepUnit & unit, double * pose_x, double * pose_y, double * pose_yaw);

  /** 按单元写入 D、G、E。贴墙不限于单堆 */
  InsertLoopAction insertSweepUnit(
    InsertBatch * batch,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double rx, double ry);

  /** 朝远离附近垃圾的一侧转 ray_offset_deg_，长度改为 garbage_extend_m_ + extend_extra_m_ */
  void nudgeExtendAwayFromGarbage(
    double gx, double gy, double yaw,
    double near_x, double near_y,
    double * yaw_out, double * extend_m_out) const;

  /** 从 garbage_list_ 队首删掉 count 堆 */
  void eraseGarbageFront(std::size_t count);

  /**
   * 记下已写入 goals 的 G、扫向，以及这条链的进近起点 from。
   * G 还在 goals 里就保持这条射线。has_e 为真时 (ex,ey) 是这条射线当前的延长点。
   * from 是链头前面那个点：第一堆用车位，后面的堆用上一堆的 E。
   */
  void rememberOpenGarbageRay(
    double gx, double gy, double yaw, bool has_e, double ex, double ey,
    double fx, double fy);

  /** G 和 E 都不在 goals 里时关掉这条射线。G 已过但 E 还在则保持 */
  void pruneOpenGarbageRays(const Goals & goals);

  /**
   * 射线还开着时，把落在 30° 内、且在 from 到链头这一段上的新垃圾接进这条链。
   * 车已开上这段时起点收到车的投影，车和 from 之间不折返。
   * 落在 from 后面的不当成这条链上的点，留在待插列表里当新堆。
   * G 前方仍按原来的扇形接。扫向不变，只留链尾一个 E。
   * 新链头插到最前面之后，这条链的 from 改成新链头前面那个点。
   * 返回 true 表示改写了 goals。
   * clip_refs 收这一次写进链的新 G；链尾新 E 生成了才追加。不在这里删原路径。
   */
  bool attachNewGarbageToOpenRays(
    Goals * goals, const geometry_msgs::msg::PoseStamped & robot_pose,
    std::vector<std::pair<double, double>> * clip_refs);

  /** (x,y) 是某条还开着的射线的 E，而不是 G */
  bool isOpenRayExtendPoint(double x, double y) const;

  /** 从已插入点里去掉这个坐标，挪走 E 时用 */
  void forgetProtectedGarbageXy(double x, double y);

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

  /** 插入结果：G、E、连线、障碍 P、贴墙 D。只画已经进入链式、穷举或删点的结果 */
  void visPublishGarbageExtend(const InsertInfo & info);
  /** 每拍刷新贴墙障碍红块和文字 P */
  void visPublishGarbageExtend();
  /** 删除 G、E、连线、障碍 P、贴墙点。直接发 DELETE，关可视化时也能清掉 */
  void visDeleteGarbageExtend();
  /** 插入时画车到 G、G 到 E 的 footprint 长条 */
  void publishVisualization(const InsertInfo & info);

  /** 总开关关掉时清空四个话题并跳过后续发布 */
  bool skipVisualization();
  void clearVisualizationTopics();
  void publishMarkers(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
    const visualization_msgs::msg::MarkerArray & arr);

  /** 清空四个可视化话题，并丢掉本节点可视化状态 */
  void clearMissionVisualization();
  void resetVisualizationState();
  /** 识别圈跟车；input_goals 里有 z=-1 时画工作圈，没有则删掉工作圈 */
  void visPublishRangeCircles(double robot_x, double robot_y);
  /** 删除识别圈和工作圈。直接发 DELETE，关可视化时也能清掉 */
  void visDeleteRangeCircles();
  /** footprint 检查不通过时，把当时检查用的 footprint 框画在检查位置上 */
  void publishFootprintCheckBox(double x, double y, double yaw);
  /** 走廊检查失败：红球 + 当时检查的 footprint 长条 */
  void publishFailedSweepVisualization(double rx, double ry, double gx, double gy);
  /** 每拍按当前路径覆盖 H、C。没有 H 或 C 时在同一次发布里删掉 */
  void visPublishClipAnchors(
    const Goals & goals, double robot_x, double robot_y);
  /** 删除 H、C 的球和文字。直接发 DELETE，关可视化时也能清掉 */
  void visDeleteClipAnchors();
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr workspace_circle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr anchor_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_check_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr garbage_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<FeedableFootprintSubscriber> footprint_topic_sub_;
  mutable std::unique_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  std::string garbage_topic_;
  std::string special_terrain_topic_;
  std::string footprint_topic_;
  std::string global_costmap_topic_;
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
  /** 贴边：D 与车最小距离 GD，默认 3m */
  double wall_edge_min_robot_dist_m_{3.0};
  /** 贴边：D-G-E 链上采样间距，默认 0.5m */
  double wall_edge_sample_m_{0.5};
  /** 贴边：整链沿障碍→垃圾法向往外挪，正为离墙，默认 0 */
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
  /** 射线左右张角，默认 30°；也是 E 躲开附近垃圾时转过的角度 */
  double ray_offset_deg_{30.0};
  /** E 周围这个半径内还有未插入的垃圾，就认为太近，默认 1.5m */
  double extend_near_radius_m_{1.5};
  /** 躲开时加在 garbage_extend_m 上的长度，默认 1.0m */
  double extend_extra_m_{1.0};
  /** 默认 E 不通时，左右各扫到此角度，默认 60° */
  double extend_max_yaw_deg_{60.0};
  /** 扇形扫角步长，默认 10° */
  double extend_step_yaw_deg_{10.0};
  /** 2.7.3 排序得分权重：sum_score = dist_w * total_dist_m + turn_w * total_yaw_rad */
  double sweep_turn_weight_{0.5};
  double sweep_dist_weight_{0.5};
  double work_circle_radius_m_{10.0};
  bool single_pile_insert_{true};
  bool enable_wall_edge_insert_{true};
  bool enable_visualization_{true};
  bool viz_switch_cleared_{false};
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

  /** 原始接收缓存 */
  std::deque<capella_ros_msg::msg::GarbageDetect> history_list_;
  /** 时间窗内待确认的垃圾位姿，满 kMaxHistorySize 丢最旧 */
  std::deque<capella_ros_msg::msg::GarbageDetect> tmp_list_;
  /** 后处理结果列表 */
  GarbageList garbage_list_;
  /** 当前认定的任务时间戳，与 goals 上统一 stamp 对齐 */
  rclcpp::Time mission_stamp_record_{0, 0, RCL_ROS_TIME};
  bool has_mission_stamp_{false};
  /** 本任务里已经插入或已经放弃的垃圾 map 坐标，附近不再当新垃圾插入 */
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
  bool viz_have_fail_strip_{false};
  /** 工作圈曾经生成过。取消后仍用上次圆心画圈，不随 has_work_circle_ 擦除 */
  bool viz_work_circle_center_set_{false};
  int next_g_num_{1};
  /** 本批已定稿、尚未插入的清扫单元，顺序与 garbage_list_ 展开一致 */
  std::vector<SweepUnit> planned_units_;
  /** 已写入 goals、G 还在队列里的射线。新垃圾用它判断能不能接成同一条链 */
  struct OpenGarbageRay
  {
    double gx{0.0};
    double gy{0.0};
    double yaw{0.0};
    bool has_e{false};
    double ex{0.0};
    double ey{0.0};
    /** 链头前面那个点：第一堆是车位，后面的堆是上一堆的 E。反向延长只认 from 到链头 */
    double fx{0.0};
    double fy{0.0};
  };
  std::vector<OpenGarbageRay> open_garbage_rays_;

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
