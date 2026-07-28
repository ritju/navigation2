#!/usr/bin/env python3
"""Reset robot to saved start, start NavigateThroughPoses, then republish garbage.

Usage:
  source /opt/ros/humble/setup.bash
  export ROS_USE_SIM_TIME=true DISPLAY=:0
  python3 repub_insert_flow.py
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from capella_ros_msg.msg import GarbageDetect
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses

DOC = Path(__file__).resolve().parent
ROBOT_NAME = 'turtlebot3_burger'
LOG = Path('/home/linux/ws2/ros_ws/navigation2/.ros_home/insert_demo_launch.log')


def quat(yaw: float):
  return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def main() -> None:
  data = json.loads((DOC / 'replay_from_rviz_log.json').read_text(encoding='utf-8'))
  rx = float(data['robot']['x'])
  ry = float(data['robot']['y'])
  ryaw = math.radians(float(data['robot'].get('yaw_deg', 0.0)))
  gx, gy = 4.0, 2.0
  if data.get('garbage'):
    gx = float(data['garbage'][0]['x'])
    gy = float(data['garbage'][0]['y'])

  rclpy.init()
  node = Node(
    'repub_insert_flow',
    parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
  )
  cmd = node.create_publisher(Twist, '/cmd_vel', 10)
  ip_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
  g_pub = node.create_publisher(GarbageDetect, '/garbage_cord', 10)
  set_cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
  if not set_cli.wait_for_service(timeout_sec=2.0):
    set_cli = node.create_client(SetEntityState, '/set_entity_state')
  nav = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')

  def spin(t: float = 0.1) -> None:
    end = time.time() + t
    while time.time() < end and rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.05)

  cmd.publish(Twist())
  qz = quat(ryaw)
  if set_cli.wait_for_service(timeout_sec=5.0):
    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = ROBOT_NAME
    req.state.reference_frame = 'world'
    req.state.pose.position.x = rx
    req.state.pose.position.y = ry
    req.state.pose.position.z = 0.01
    req.state.pose.orientation.z = qz[2]
    req.state.pose.orientation.w = qz[3]
    fut = set_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    print('gazebo teleport ok', flush=True)
  else:
    print('WARN: set_entity_state unavailable', flush=True)

  for _ in range(25):
    ip = PoseWithCovarianceStamped()
    ip.header.frame_id = 'map'
    ip.header.stamp = node.get_clock().now().to_msg()
    ip.pose.pose.position.x = rx
    ip.pose.pose.position.y = ry
    ip.pose.pose.orientation.z = qz[2]
    ip.pose.pose.orientation.w = qz[3]
    ip.pose.covariance[0] = 0.25
    ip.pose.covariance[7] = 0.25
    ip.pose.covariance[35] = 0.068
    ip_pub.publish(ip)
    spin(0.08)
  print('initialpose published', flush=True)
  spin(2.0)

  if not nav.wait_for_server(timeout_sec=30.0):
    raise SystemExit('nav action unavailable')

  goal = NavigateThroughPoses.Goal()
  for g in data['goals']:
    yaw = math.radians(float(g.get('yaw_deg', 0.0)))
    q = quat(yaw)
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(g['x'])
    p.pose.position.y = float(g['y'])
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    goal.poses.append(p)

  accepted = False
  for attempt in range(8):
    print(f'send nav attempt {attempt + 1}', flush=True)
    fut = nav.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if handle is not None and handle.accepted:
      print('nav accepted', flush=True)
      accepted = True
      break
    print('nav rejected, retry', flush=True)
    time.sleep(2.0)
  if not accepted:
    raise SystemExit('nav not accepted')

  print('wait 6s before garbage...', flush=True)
  spin(6.0)

  start_sz = LOG.stat().st_size if LOG.exists() else 0
  inserted = False
  for i in range(40):
    msg = GarbageDetect()
    msg.pose = PoseStamped()
    msg.pose.header.frame_id = 'map'
    msg.pose.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = gx
    msg.pose.pose.position.y = gy
    msg.pose.pose.orientation.w = 1.0
    msg.class_id = GarbageDetect.DRY_GARBAGE
    g_pub.publish(msg)
    print(f'garbage[{i + 1}] ({gx}, {gy})', flush=True)
    spin(0.2)
    if LOG.exists() and LOG.stat().st_size > start_sz:
      tail = LOG.read_text(errors='ignore')[-16000:]
      if (
        'InsertGarbagePose: inserted garbage' in tail
        or 'InsertGarbagePose: head-insert' in tail
      ):
        print('INSERT DETECTED in log', flush=True)
        inserted = True
        break
    time.sleep(0.5)

  print(f'DONE inserted={inserted}', flush=True)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
