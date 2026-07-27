#!/usr/bin/env python3
"""把 replay_from_rviz_log.json 里保存的导航点重新发到 /waypoints，
让 zigzag_garbage_test.py 打印 WP[...]，并在 /m 上显示青色草稿点。

注意：这只是可视化草稿，不会自动开始 Nav Through Poses。
要真正导航，仍需在 RViz 里用 Nav Through Poses 勾同样路径并点 Start。

用法:
  python3 republish_saved_waypoints.py
  python3 republish_saved_waypoints.py --json replay_from_rviz_log.json --once
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

DOC = Path(__file__).resolve().parent


def yaw_to_quat(yaw: float):
  return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--json', type=Path, default=DOC / 'replay_from_rviz_log.json')
  ap.add_argument('--once', action='store_true', help='publish a few times then exit')
  ap.add_argument('--period', type=float, default=1.0)
  args = ap.parse_args()

  data = json.loads(args.json.read_text(encoding='utf-8'))
  goals = data['goals']
  if len(goals) < 2:
    raise SystemExit('need >=2 goals in json')

  rclpy.init()
  node = Node(
    'republish_saved_waypoints',
    parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
  )
  latched = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
  )
  pub = node.create_publisher(MarkerArray, '/waypoints', latched)
  pub2 = node.create_publisher(MarkerArray, '/waypoints', 10)

  for _ in range(15):
    rclpy.spin_once(node, timeout_sec=0.1)

  print(f'Loaded {len(goals)} goals from {args.json}', flush=True)
  for i, g in enumerate(goals, 1):
    print(f'  WP[{i}] x={g["x"]:.3f} y={g["y"]:.3f}', flush=True)

  count = 0
  try:
    while rclpy.ok():
      arr = MarkerArray()
      stamp = node.get_clock().now().to_msg()
      for i, g in enumerate(goals, 1):
        yaw = math.radians(float(g.get('yaw_deg', 0.0)))
        q = yaw_to_quat(yaw)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = 'waypoints'
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = float(g['x'])
        m.pose.position.y = float(g['y'])
        m.pose.position.z = 0.05
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]
        m.scale.x = 0.35
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.75, 0.9, 0.9
        arr.markers.append(m)

        t = Marker()
        t.header = m.header
        t.ns = 'waypoint_labels'
        t.id = i
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = float(g['x'])
        t.pose.position.y = float(g['y'])
        t.pose.position.z = 0.25
        t.pose.orientation.w = 1.0
        t.scale.z = 0.2
        t.color.r = t.color.g = t.color.b = 1.0
        t.color.a = 0.95
        t.text = f'wp_{i}'
        arr.markers.append(t)

      pub.publish(arr)
      pub2.publish(arr)
      count += 1
      print(f'Published /waypoints batch[{count}] n={len(goals)}', flush=True)
      rclpy.spin_once(node, timeout_sec=0.05)
      if args.once and count >= 3:
        break
      time.sleep(max(0.2, args.period))
  except KeyboardInterrupt:
    print('\nstopped', flush=True)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
