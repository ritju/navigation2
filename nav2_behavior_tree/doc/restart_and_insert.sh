#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export KEEP_DISPLAY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROOT="/home/linux/ws2/ros_ws/navigation2"
DOC="$ROOT/nav2_behavior_tree/doc"
LOG="$ROOT/.ros_home/insert_demo_launch.log"

echo "[0] kill old Nav2/Gazebo/RViz (keep unrelated nodes)"
pkill -9 -f 'tb3_simulation_launch.py' 2>/dev/null || true
pkill -9 -f 'component_container_isolated' 2>/dev/null || true
pkill -9 -f 'nav2_container' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
pkill -9 -f 'spawn_entity.py' 2>/dev/null || true
# 二进制名精确匹配，避免误杀自己的 pkill 命令行
for pat in gzserver gzclient rviz2; do
  pgrep -x "$pat" >/dev/null 2>&1 && pkill -9 -x "$pat" || true
done
pkill -9 -f 'zigzag_garbage_test.py' 2>/dev/null || true
pkill -9 -f 'repub_insert_flow.py' 2>/dev/null || true
pkill -9 -f 'ros2cli.daemon' 2>/dev/null || true
sleep 4
# 残留 SHM 会导致 FastDDS Failed init_port
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 1

echo "[1] launch TB3+Nav2+RViz (domain=${ROS_DOMAIN_ID}, rviz=scripts/insert_garbage_pose.rviz)"
: > "$LOG"
bash "$ROOT/scripts/launch_tb3_insert_garbage_test.sh" >> "$LOG" 2>&1 &
# navigation 激活会卡在等 odom/map TF，必须先 spawn 出机器人
spawned=0
for i in $(seq 1 90); do
  if rg -q 'Successfully spawned entity|Spawn status: True|Entity spawned' "$LOG" 2>/dev/null; then
    echo "robot spawned after ${i}s"
    spawned=1
    break
  fi
  sleep 1
done
if [[ "$spawned" -ne 1 ]]; then
  echo "WARN: spawn not confirmed yet; continue waiting for lifecycle" >&2
fi

# map 坐标系要 AMCL initial pose，否则 global_costmap 激活会一直等
set +u
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_USE_SIM_TIME=true
set -u
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = Node('restart_initpose')
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
time.sleep(0.8)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = -2.0
msg.pose.pose.position.y = -1.5
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685
for _ in range(8):
    msg.header.stamp = n.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.25)
print('early initialpose published')
n.destroy_node()
rclpy.shutdown()
PY

ready=0
for i in $(seq 1 240); do
  n=$(rg -c 'Managed nodes are active' "$LOG" 2>/dev/null || true)
  n=${n:-0}
  if [[ "$n" -ge 2 ]]; then
    echo "active count=$n after ${i}s (post-spawn wait)"
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" -ne 1 ]]; then
  echo "ERROR: Nav2 lifecycle not active in time; see $LOG" >&2
  rg -n 'Failed init_port|Successfully spawned|Managed nodes are active|have not come up|Type support' "$LOG" | tail -30 >&2 || true
  exit 1
fi
sleep 5

set +u
source /opt/ros/humble/setup.bash
source /home/linux/python/capella_clean_garbage/install/capella_ros_msg/share/capella_ros_msg/local_setup.bash
source /home/linux/python/capella_clean_garbage/install/garage_utils_msgs/share/garage_utils_msgs/local_setup.bash 2>/dev/null || true
source "$ROOT/install/nav2_behavior_tree/share/nav2_behavior_tree/local_setup.bash"
source "$ROOT/install/nav2_bt_navigator/share/nav2_bt_navigator/local_setup.bash" 2>/dev/null || true
export ROS_USE_SIM_TIME=true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set -u
echo "[env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

echo "[2] republish waypoints"
python3 "$DOC/republish_saved_waypoints.py" --once

echo "[3] reset+nav+garbage"
python3 "$DOC/repub_insert_flow.py"

echo "[4] done"
rg -n 'inserted garbage|head-insert' "$LOG" | tail -5
