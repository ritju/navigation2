#!/usr/bin/env python3
"""TB3 联调：复位机器人位姿，打印 /waypoints，发布 /m 可视化。

用法:
  source /opt/ros/humble/setup.bash
  source /home/linux/python/capella_clean_garbage/install/capella_ros_msg/share/capella_ros_msg/local_setup.bash
  export ROS_USE_SIM_TIME=true DISPLAY=:0
  python3 zigzag_garbage_test.py

RViz MarkerArray → /m
  绿=当前 goals  黑=已删  红=垃圾  青=草稿 /waypoints

Ctrl+C 退出。
"""

from __future__ import annotations

import math
import os
import re
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
  from capella_ros_msg.msg import GarbageDetect
except ImportError:  # pragma: no cover
  GarbageDetect = None  # type: ignore

# 与 launch_tb3_insert_garbage_test.sh 的 x_pose/y_pose 对齐
START_XY = (-2.0, -1.5)
START_YAW = 0.0
ROBOT_NAME = 'turtlebot3_burger'
DEDUP_GRID_M = 0.05


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
  return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_to_yaw(z: float, w: float) -> float:
  return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def pose_key(x: float, y: float) -> Tuple[int, int]:
  g = DEDUP_GRID_M
  return (int(round(x / g)), int(round(y / g)))


class ResetRobotPose(Node):
  def __init__(self) -> None:
    from rclpy.parameter import Parameter
    super().__init__(
      'reset_robot_pose',
      parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    self.initialpose_pub = self.create_publisher(
      PoseWithCovarianceStamped, '/initialpose', 10)
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.set_entity_cli = self.create_client(SetEntityState, '/set_entity_state')
    self.reset_world_cli = self.create_client(Empty, '/reset_world')
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self._printed_labels: Set[str] = set()
    self._printed_poses: Set[Tuple[int, int]] = set()
    self._wp_count = 0

    self.waypoint_poses: List[Pose] = []
    self.current_goals: List[Pose] = []
    self.deleted_goals: List[Pose] = []
    self.garbage_xy: List[Tuple[float, float]] = []

    latched = QoSProfile(
      depth=1,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.TRANSIENT_LOCAL,
      history=HistoryPolicy.KEEP_LAST,
    )
    # RViz MarkerArray 默认 VOLATILE；用 TRANSIENT_LOCAL 常导致 Status: Error
    self.marker_pub = self.create_publisher(MarkerArray, '/m', 10)

    self.create_subscription(MarkerArray, '/waypoints', self._on_waypoints, latched)
    self.create_subscription(MarkerArray, '/waypoints', self._on_waypoints, 10)
    self.create_subscription(
      PoseArray, 'insert_garbage_pose/current_goals', self._on_current_goals, 10)
    self.create_subscription(
      PoseArray, 'insert_garbage_pose/deleted_goals', self._on_deleted_goals, 10)

    if GarbageDetect is not None:
      self.create_subscription(GarbageDetect, '/garbage_cord', self._on_garbage, 10)
    else:
      self.get_logger().warn('capella_ros_msg not found; /garbage_cord viz disabled')

    self.create_timer(0.5, self.publish_markers)

  def spin_for(self, seconds: float) -> None:
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
      rclpy.spin_once(self, timeout_sec=0.05)

  def stop_robot(self) -> None:
    self.cmd_vel_pub.publish(Twist())

  def reset_robot_pose(self) -> None:
    self.get_logger().info(
      f'Reset robot pose to start ({START_XY[0]:.2f}, {START_XY[1]:.2f}), '
      f'yaw={math.degrees(START_YAW):.1f}deg ...')
    self.stop_robot()

    if os.environ.get('ZIGZAG_RESET_WORLD', '0') == '1':
      if self.reset_world_cli.wait_for_service(timeout_sec=2.0):
        fut = self.reset_world_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('Called /reset_world')
        self.spin_for(1.0)

    q = yaw_to_quat(START_YAW)
    if self.set_entity_cli.wait_for_service(timeout_sec=5.0):
      req = SetEntityState.Request()
      req.state = EntityState()
      req.state.name = ROBOT_NAME
      req.state.reference_frame = 'world'
      req.state.pose.position.x = START_XY[0]
      req.state.pose.position.y = START_XY[1]
      req.state.pose.position.z = 0.01
      req.state.pose.orientation.z = q[2]
      req.state.pose.orientation.w = q[3]
      req.state.twist = Twist()
      fut = self.set_entity_cli.call_async(req)
      rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
      ok = False
      try:
        resp = fut.result()
        ok = bool(resp is not None and getattr(resp, 'success', True))
      except Exception:
        ok = False
      if ok:
        self.get_logger().info(
          f'Gazebo teleported {ROBOT_NAME} to ({START_XY[0]:.2f},{START_XY[1]:.2f})')
      else:
        self.get_logger().warn(
          'set_entity_state failed; AMCL initialpose only (pose may mismatch)')
      self.spin_for(0.5)
    else:
      self.get_logger().warn(
        '/set_entity_state unavailable; only resetting AMCL initialpose')

    ip = PoseWithCovarianceStamped()
    ip.header.frame_id = 'map'
    ip.pose.pose.position.x = START_XY[0]
    ip.pose.pose.position.y = START_XY[1]
    ip.pose.pose.orientation.z = q[2]
    ip.pose.pose.orientation.w = q[3]
    ip.pose.covariance[0] = 0.25
    ip.pose.covariance[7] = 0.25
    ip.pose.covariance[35] = 0.068
    for _ in range(20):
      ip.header.stamp = self.get_clock().now().to_msg()
      self.initialpose_pub.publish(ip)
      self.spin_for(0.1)
    self.spin_for(1.5)

  def lookup_robot_pose(self, timeout_sec: float = 10.0) -> Optional[Tuple[float, float, float]]:
    deadline = time.time() + timeout_sec
    while time.time() < deadline and rclpy.ok():
      try:
        tf = self.tf_buffer.lookup_transform(
          'map', 'base_link', Time(), timeout=Duration(seconds=0.2))
        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = quat_to_yaw(r.z, r.w)
        return (t.x, t.y, yaw)
      except Exception:
        rclpy.spin_once(self, timeout_sec=0.1)
    return None

  def _extract_waypoints(
    self, msg: MarkerArray
  ) -> Dict[str, Tuple[float, float, float]]:
    by_label: Dict[str, Tuple[float, float, float]] = {}
    arrows: Dict[int, Tuple[float, float, float]] = {}

    for m in msg.markers:
      if m.action in (Marker.DELETE, Marker.DELETEALL):
        continue
      x = m.pose.position.x
      y = m.pose.position.y
      yaw = quat_to_yaw(m.pose.orientation.z, m.pose.orientation.w)
      text = (m.text or '').strip()
      if m.type == Marker.TEXT_VIEW_FACING and text:
        by_label[text] = (x, y, yaw)
      elif m.type == Marker.ARROW:
        arrows[m.id] = (x, y, yaw)

    if by_label:
      return by_label

    out: Dict[str, Tuple[float, float, float]] = {}
    for i, mid in enumerate(sorted(arrows), 1):
      out[f'wp_{i}'] = arrows[mid]
    return out

  def _on_waypoints(self, msg: MarkerArray) -> None:
    wps = self._extract_waypoints(msg)

    if not wps:
      if self._printed_labels or self._printed_poses:
        self._printed_labels.clear()
        self._printed_poses.clear()
        self._wp_count = 0
        self.waypoint_poses = []
        print('--- waypoints cleared; dedup reset ---', flush=True)
        self.publish_markers()
      return

    def sort_key(label: str) -> Tuple[int, str]:
      m = re.search(r'(\d+)', label)
      return (int(m.group(1)) if m else 10**9, label)

    poses: List[Pose] = []
    for label in sorted(wps, key=sort_key):
      x, y, yaw = wps[label]
      p = Pose()
      p.position.x = x
      p.position.y = y
      q = yaw_to_quat(yaw)
      p.orientation.z = q[2]
      p.orientation.w = q[3]
      poses.append(p)

      key = pose_key(x, y)
      if label in self._printed_labels or key in self._printed_poses:
        continue
      self._printed_labels.add(label)
      self._printed_poses.add(key)
      self._wp_count += 1
      line = (
        f'WP[{self._wp_count}] {label}: '
        f'x={x:.3f}, y={y:.3f}, '
        f'yaw={yaw:.3f}rad ({math.degrees(yaw):.1f}deg)'
      )
      print(line, flush=True)
      self.get_logger().info(line)

    self.waypoint_poses = poses
    self.publish_markers()

  def _on_current_goals(self, msg: PoseArray) -> None:
    self.current_goals = list(msg.poses)
    self.publish_markers()

  def _on_deleted_goals(self, msg: PoseArray) -> None:
    self.deleted_goals = list(msg.poses)
    self.publish_markers()

  def _on_garbage(self, msg) -> None:
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    # 简单去重
    for gx, gy in self.garbage_xy:
      if (gx - x) ** 2 + (gy - y) ** 2 < 0.05 ** 2:
        return
    self.garbage_xy.append((x, y))
    print(f'GARBAGE: x={x:.3f}, y={y:.3f}', flush=True)
    self.publish_markers()

  def _sphere(
    self, ns: str, mid: int, x: float, y: float, color: ColorRGBA, scale: float = 0.22
  ) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = self.get_clock().now().to_msg()
    m.ns = ns
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.color = color
    m.lifetime = Duration(seconds=0).to_msg()
    return m

  def publish_markers(self) -> None:
    arr = MarkerArray()
    stamp = self.get_clock().now().to_msg()
    for ns in ('goals_active', 'goals_deleted', 'garbage', 'waypoints_draft', 'goals_path'):
      clear = Marker()
      clear.header.frame_id = 'map'
      clear.header.stamp = stamp
      clear.ns = ns
      clear.id = 0
      clear.action = Marker.DELETEALL
      # RViz 对 DELETEALL 的 type/scale 较挑剔，给合法默认值
      clear.type = Marker.SPHERE
      clear.scale.x = clear.scale.y = clear.scale.z = 0.1
      clear.pose.orientation.w = 1.0
      arr.markers.append(clear)

    green = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.95)
    black = ColorRGBA(r=0.05, g=0.05, b=0.05, a=0.95)
    red = ColorRGBA(r=0.95, g=0.1, b=0.1, a=0.95)
    cyan = ColorRGBA(r=0.1, g=0.75, b=0.9, a=0.85)

    mid = 0
    # 有 InsertGarbage 反馈用绿；否则用勾画的 waypoints 青色占位
    active = self.current_goals if self.current_goals else self.waypoint_poses
    color_active = green if self.current_goals else cyan
    ns_active = 'goals_active' if self.current_goals else 'waypoints_draft'
    for p in active:
      arr.markers.append(
        self._sphere(ns_active, mid, p.position.x, p.position.y, color_active))
      mid += 1

    if len(active) >= 2:
      line = Marker()
      line.header.frame_id = 'map'
      line.header.stamp = stamp
      line.ns = 'goals_path'
      line.id = 0
      line.type = Marker.LINE_STRIP
      line.action = Marker.ADD
      line.scale.x = 0.04
      line.color = ColorRGBA(r=color_active.r, g=color_active.g, b=color_active.b, a=0.5)
      line.pose.orientation.w = 1.0
      from geometry_msgs.msg import Point
      for p in active:
        pt = Point()
        pt.x = p.position.x
        pt.y = p.position.y
        pt.z = 0.05
        line.points.append(pt)
      arr.markers.append(line)

    mid = 0
    for p in self.deleted_goals:
      arr.markers.append(
        self._sphere('goals_deleted', mid, p.position.x, p.position.y, black, 0.18))
      mid += 1

    mid = 0
    for x, y in self.garbage_xy:
      arr.markers.append(self._sphere('garbage', mid, x, y, red, 0.28))
      mid += 1

    self.marker_pub.publish(arr)


def main() -> None:
  rclpy.init()
  node = ResetRobotPose()
  try:
    node.reset_robot_pose()
    print(
      f'COMMANDED_START map: x={START_XY[0]:.3f} y={START_XY[1]:.3f} '
      f'yaw_rad={START_YAW:.3f} yaw_deg={math.degrees(START_YAW):.1f}',
      flush=True)
    pose = node.lookup_robot_pose(timeout_sec=8.0)
    if pose is None:
      node.get_logger().warn(
        'map→base_link not ready (need RViz 2D Pose Estimate).')
      print(
        'TF_POSE: unavailable — in RViz click "2D Pose Estimate" near '
        f'({START_XY[0]:.1f}, {START_XY[1]:.1f}).',
        flush=True)
    else:
      x, y, yaw = pose
      print(
        f'TF_POSE map: x={x:.3f} y={y:.3f} '
        f'yaw_rad={yaw:.3f} yaw_deg={math.degrees(yaw):.1f}',
        flush=True)

    print(
      'RViz: MarkerArray topic → /m  (green=active, black=deleted, red=garbage, cyan=draft wp)\n'
      'Listening /waypoints ... Ctrl+C to quit.',
      flush=True)
    node.publish_markers()
    while rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.2)
  except KeyboardInterrupt:
    print('\nbye', flush=True)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
