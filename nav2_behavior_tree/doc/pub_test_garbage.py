#!/usr/bin/env python3
"""手动往 /garbage_cord 发垃圾点，供 InsertGarbagePose 联调。

用法:
  python3 pub_test_garbage.py <x> <y> [yaw_deg]
  python3 pub_test_garbage.py 4.0 2.0 --loop
  python3 pub_test_garbage.py 4.0 2.0 --loop --period 0.5
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from capella_ros_msg.msg import GarbageDetect


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('x', type=float)
  p.add_argument('y', type=float)
  p.add_argument('yaw_deg', type=float, nargs='?', default=0.0)
  p.add_argument('--loop', action='store_true')
  p.add_argument('--period', type=float, default=1.0)
  return p.parse_args()


def main() -> None:
  args = parse_args()
  rclpy.init()
  node = Node(
    'pub_test_garbage',
    parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
  )
  pub = node.create_publisher(GarbageDetect, '/garbage_cord', 10)
  for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)

  yaw = math.radians(args.yaw_deg)
  count = 0
  try:
    while rclpy.ok():
      msg = GarbageDetect()
      msg.pose = PoseStamped()
      msg.pose.header.frame_id = 'map'
      msg.pose.header.stamp = node.get_clock().now().to_msg()
      msg.pose.pose.position.x = args.x
      msg.pose.pose.position.y = args.y
      msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
      msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
      msg.class_id = GarbageDetect.DRY_GARBAGE
      pub.publish(msg)
      count += 1
      node.get_logger().info(
        f'Published garbage[{count}] at map ({args.x:.3f}, {args.y:.3f}) on /garbage_cord')
      rclpy.spin_once(node, timeout_sec=0.05)
      if not args.loop:
        break
      time.sleep(max(0.05, args.period))
  except KeyboardInterrupt:
    print(f'\nstopped after {count} publishes', flush=True)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
