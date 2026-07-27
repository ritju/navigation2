#!/usr/bin/env python3
"""Reset robot pose, collect /waypoints + /garbage_cord, print and save JSON for insert_garbage_demo.py.

Usage:
  source /opt/ros/humble/setup.bash
  source /home/linux/python/capella_clean_garbage/install/capella_ros_msg/share/capella_ros_msg/local_setup.bash
  export ROS_USE_SIM_TIME=true DISPLAY=:0
  python3 zigzag_garbage_test.py

After drawing waypoints in RViz and publishing garbage, press Ctrl+C.
The script saves a scenario JSON that can be replayed with:
  python3 insert_garbage_demo.py --replay --scenario <saved_file.json>
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
  from capella_ros_msg.msg import GarbageDetect
except ImportError:
  GarbageDetect = None

START_XY = (-2.0, -1.5)
START_YAW = 0.0
ROBOT_NAME = 'turtlebot3_burger'
DEDUP_GRID_M = 0.05
OUTPUT_JSON = Path(__file__).resolve().with_name('replay_from_rviz_log.json')


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
  return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_to_yaw(z: float, w: float) -> float:
  return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def pose_key(x: float, y: float) -> Tuple[int, int]:
  g = DEDUP_GRID_M
  return (int(round(x / g)), int(round(y / g)))


class DataCollector(Node):
  def __init__(self) -> None:
    from rclpy.parameter import Parameter
    super().__init__(
      'zigzag_data_collector',
      parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    self.initialpose_pub = self.create_publisher(
      PoseWithCovarianceStamped, '/initialpose', 10)
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.set_entity_cli = self.create_client(SetEntityState, '/set_entity_state')
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.marker_pub = self.create_publisher(MarkerArray, '/m', 10)

    self._printed_labels: Set[str] = set()
    self._printed_poses: Set[Tuple[int, int]] = set()
    self._wp_count = 0

    self.waypoints: List[Dict] = []
    self.garbage_list: List[Dict] = []
    self.robot_pose: Optional[Dict] = None

    latched = QoSProfile(
      depth=1,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.TRANSIENT_LOCAL,
      history=HistoryPolicy.KEEP_LAST,
    )
    self.create_subscription(MarkerArray, '/waypoints', self._on_waypoints, latched)
    self.create_subscription(MarkerArray, '/waypoints', self._on_waypoints, 10)

    if GarbageDetect is not None:
      self.create_subscription(GarbageDetect, '/garbage_cord', self._on_garbage, 10)
    else:
      self.get_logger().warn('capella_ros_msg not found; /garbage_cord disabled')

    self.create_timer(0.5, self._publish_viz)

  def spin_for(self, seconds: float) -> None:
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
      rclpy.spin_once(self, timeout_sec=0.05)

  def reset_robot_pose(self) -> None:
    self.get_logger().info(
      f'Reset robot pose to ({START_XY[0]:.2f}, {START_XY[1]:.2f}), '
      f'yaw={math.degrees(START_YAW):.1f}deg')
    self.cmd_vel_pub.publish(Twist())

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
      self.get_logger().info('Gazebo teleport done')
      self.spin_for(0.5)
    else:
      self.get_logger().warn('/set_entity_state unavailable; AMCL initialpose only')

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

  def _on_waypoints(self, msg: MarkerArray) -> None:
    wps = self._extract_waypoints(msg)
    if not wps:
      return

    def sort_key(label: str) -> Tuple[int, str]:
      m = re.search(r'(\d+)', label)
      return (int(m.group(1)) if m else 10**9, label)

    new_wps: List[Dict] = []
    for label in sorted(wps, key=sort_key):
      x, y, yaw = wps[label]
      new_wps.append({'x': round(x, 3), 'y': round(y, 3),
                      'yaw_deg': round(math.degrees(yaw), 1)})
      key = pose_key(x, y)
      if label not in self._printed_labels and key not in self._printed_poses:
        self._printed_labels.add(label)
        self._printed_poses.add(key)
        self._wp_count += 1
        print(f'WP[{self._wp_count}] {label}: x={x:.3f}, y={y:.3f}, '
              f'yaw={math.degrees(yaw):.1f}deg', flush=True)

    self.waypoints = new_wps

  def _extract_waypoints(self, msg: MarkerArray) -> Dict[str, Tuple[float, float, float]]:
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

  def _on_garbage(self, msg) -> None:
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    for g in self.garbage_list:
      if (g['x'] - x) ** 2 + (g['y'] - y) ** 2 < 0.05 ** 2:
        return
    self.garbage_list.append({'x': round(x, 3), 'y': round(y, 3)})
    print(f'GARBAGE[{len(self.garbage_list)}]: x={x:.3f}, y={y:.3f}', flush=True)

  def _publish_viz(self) -> None:
    arr = MarkerArray()
    stamp = self.get_clock().now().to_msg()
    for ns in ('wp_viz', 'garbage_viz', 'wp_path'):
      clear = Marker()
      clear.header.frame_id = 'map'
      clear.header.stamp = stamp
      clear.ns = ns
      clear.id = 0
      clear.action = Marker.DELETEALL
      clear.type = Marker.SPHERE
      clear.scale.x = clear.scale.y = clear.scale.z = 0.1
      clear.pose.orientation.w = 1.0
      arr.markers.append(clear)

    cyan = ColorRGBA(r=0.1, g=0.75, b=0.9, a=0.85)
    red = ColorRGBA(r=0.95, g=0.1, b=0.1, a=0.95)

    for i, wp in enumerate(self.waypoints):
      m = Marker()
      m.header.frame_id = 'map'
      m.header.stamp = stamp
      m.ns = 'wp_viz'
      m.id = i
      m.type = Marker.SPHERE
      m.action = Marker.ADD
      m.pose.position.x = wp['x']
      m.pose.position.y = wp['y']
      m.pose.position.z = 0.15
      m.pose.orientation.w = 1.0
      m.scale.x = m.scale.y = m.scale.z = 0.2
      m.color = cyan
      arr.markers.append(m)

    if len(self.waypoints) >= 2:
      from geometry_msgs.msg import Point
      line = Marker()
      line.header.frame_id = 'map'
      line.header.stamp = stamp
      line.ns = 'wp_path'
      line.id = 0
      line.type = Marker.LINE_STRIP
      line.action = Marker.ADD
      line.scale.x = 0.04
      line.color = ColorRGBA(r=0.1, g=0.75, b=0.9, a=0.5)
      line.pose.orientation.w = 1.0
      for wp in self.waypoints:
        pt = Point()
        pt.x = wp['x']
        pt.y = wp['y']
        pt.z = 0.05
        line.points.append(pt)
      arr.markers.append(line)

    for i, g in enumerate(self.garbage_list):
      m = Marker()
      m.header.frame_id = 'map'
      m.header.stamp = stamp
      m.ns = 'garbage_viz'
      m.id = i
      m.type = Marker.SPHERE
      m.action = Marker.ADD
      m.pose.position.x = g['x']
      m.pose.position.y = g['y']
      m.pose.position.z = 0.15
      m.pose.orientation.w = 1.0
      m.scale.x = m.scale.y = m.scale.z = 0.28
      m.color = red
      arr.markers.append(m)

    self.marker_pub.publish(arr)

  def save_scenario(self) -> Path:
    data = {
      'name': 'from_rviz_collection',
      'robot': self.robot_pose or {'x': START_XY[0], 'y': START_XY[1], 'yaw_deg': math.degrees(START_YAW)},
      'goals': self.waypoints,
      'garbage': self.garbage_list,
    }
    OUTPUT_JSON.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return OUTPUT_JSON


def main() -> None:
  rclpy.init()
  node = DataCollector()
  try:
    node.reset_robot_pose()
    pose = node.lookup_robot_pose(timeout_sec=8.0)
    if pose:
      x, y, yaw = pose
      node.robot_pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}
      print(f'ROBOT: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}deg', flush=True)
    else:
      node.robot_pose = {'x': START_XY[0], 'y': START_XY[1], 'yaw_deg': math.degrees(START_YAW)}
      print(f'ROBOT: TF unavailable, using start pose', flush=True)

    print('\nListening /waypoints + /garbage_cord ...')
    print('Draw waypoints in RViz, publish garbage, then Ctrl+C to save.\n', flush=True)
    while rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.2)
  except KeyboardInterrupt:
    pass
  finally:
    if node.waypoints:
      out = node.save_scenario()
      print(f'\nSaved {len(node.waypoints)} waypoints + {len(node.garbage_list)} garbage to:')
      print(f'  {out}')
      print(f'\nReplay with:')
      print(f'  python3 insert_garbage_demo.py --replay --scenario {out.name}')
    else:
      print('\nNo waypoints collected, nothing saved.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
