#!/usr/bin/env python3
"""Read replay_from_rviz_log.json and start NavigateThroughPoses.

Usage:
  source /opt/ros/humble/setup.bash
  export ROS_USE_SIM_TIME=true DISPLAY=:0
  cd ~/ws2/ros_ws/navigation2/nav2_behavior_tree/doc
  python3 start_saved_nav.py

This sends the saved goals to /navigate_through_poses and also publishes the
saved robot pose to /initialpose first.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped as AmclPose
from nav2_msgs.action import NavigateThroughPoses


DOC = Path(__file__).resolve().parent


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
  return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SavedNavRunner(Node):
  def __init__(self) -> None:
    super().__init__(
      'start_saved_nav',
      parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
    self.amcl_pose_seen = False
    self.create_subscription(AmclPose, '/amcl_pose', self._on_amcl_pose, 10)

  def _on_amcl_pose(self, _msg: AmclPose) -> None:
    self.amcl_pose_seen = True

  def publish_initial_pose(self, x: float, y: float, yaw_deg: float) -> None:
    yaw = math.radians(yaw_deg)
    q = yaw_to_quat(yaw)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(10):
      msg.header.stamp = self.get_clock().now().to_msg()
      self.initialpose_pub.publish(msg)
      rclpy.spin_once(self, timeout_sec=0.1)
    self.get_logger().info(
      f'Published initial pose ({x:.3f}, {y:.3f}, yaw={yaw_deg:.1f}deg)')

  def wait_for_localization(self, timeout_sec: float = 30.0) -> bool:
    self.get_logger().info('Waiting for /amcl_pose after initial pose...')
    deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
      rclpy.spin_once(self, timeout_sec=0.1)
      if self.amcl_pose_seen:
        self.get_logger().info('/amcl_pose received')
        return True
    self.get_logger().warn('Timed out waiting for /amcl_pose; sending goals anyway')
    return False

  def send_goals(self, goals: list[dict]) -> bool:
    if not self.client.wait_for_server(timeout_sec=60.0):
      self.get_logger().error('navigate_through_poses action server not available')
      return False

    goal = NavigateThroughPoses.Goal()
    for g in goals:
      yaw = math.radians(float(g.get('yaw_deg', 0.0)))
      q = yaw_to_quat(yaw)
      pose = PoseStamped()
      pose.header.frame_id = 'map'
      pose.header.stamp = self.get_clock().now().to_msg()
      pose.pose.position.x = float(g['x'])
      pose.pose.position.y = float(g['y'])
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]
      goal.poses.append(pose)

    import time
    max_retries = 10
    for attempt in range(max_retries):
      self.get_logger().info(f'Sending NavigateThroughPoses with {len(goal.poses)} goals')
      future = self.client.send_goal_async(goal)
      rclpy.spin_until_future_complete(self, future)
      handle = future.result()
      if handle is not None and handle.accepted:
        self.get_logger().info('NavigateThroughPoses goal accepted')
        return True
      if attempt < max_retries - 1:
        self.get_logger().warn(
          f'Goal rejected (attempt {attempt+1}/{max_retries}), retrying in 3s...')
        time.sleep(3.0)
      else:
        self.get_logger().error('NavigateThroughPoses goal rejected after all retries')
        return False


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--json', type=Path, default=DOC / 'replay_from_rviz_log.json')
  args = parser.parse_args()

  data = json.loads(args.json.read_text(encoding='utf-8'))
  goals = data['goals']
  robot = data['robot']

  rclpy.init()
  node = SavedNavRunner()
  try:
    node.publish_initial_pose(
      float(robot['x']), float(robot['y']), float(robot.get('yaw_deg', 0.0)))
    node.wait_for_localization()
    node.send_goals(goals)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
