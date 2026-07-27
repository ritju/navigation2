#!/usr/bin/env bash
set -eo pipefail

ROOT="/home/linux/ws2/ros_ws/navigation2"
DOC="$ROOT/nav2_behavior_tree/doc"
ROS_HOME="$ROOT/.ros_home"

mkdir -p "$ROS_HOME/log"
export HOME=/home/linux
export ROS_HOME
export ROS_USE_SIM_TIME=true
export DISPLAY="${DISPLAY:-:0}"

set +u
source /opt/ros/humble/setup.bash
source /home/linux/python/capella_clean_garbage/install/capella_ros_msg/share/capella_ros_msg/local_setup.bash 2>/dev/null || true
source /home/linux/python/capella_clean_garbage/install/garage_utils_msgs/share/garage_utils_msgs/local_setup.bash 2>/dev/null || true
set -u

echo "[0/5] kill old sim/nav2/rviz (to pick up latest params)"
pkill -9 -f 'tb3_simulation_launch.py' 2>/dev/null || true
pkill -9 -f 'gzserver|gzclient|rviz2|component_container_isolated|nav2_container' 2>/dev/null || true
pkill -9 -f 'zigzag_garbage_test.py' 2>/dev/null || true
sleep 3
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "[1/5] launch TB3 + Nav2 + RViz"
bash "$ROOT/scripts/launch_tb3_insert_garbage_test.sh" > "$ROOT/.ros_home/insert_demo_launch.log" 2>&1 &
sleep 60

echo "[2/5] republish saved waypoints (for /waypoints + viz)"
python3 "$DOC/republish_saved_waypoints.py" --once
sleep 1

echo "[3/5] publish saved garbage"
python3 "$DOC/pub_test_garbage.py" 4.0 2.0
sleep 1

echo "[4/5] start NavigateThroughPoses (blocks until goal accepted)"
python3 "$DOC/start_saved_nav.py"

echo "[5/5] start zigzag viz helper (optional)"
python3 "$DOC/zigzag_garbage_test.py" > "$ROOT/.ros_home/zigzag_viz.log" 2>&1 &
sleep 2

echo
echo "launched. logs:"
echo "  $ROOT/.ros_home/insert_demo_launch.log"
echo "  $ROOT/.ros_home/zigzag_viz.log"
