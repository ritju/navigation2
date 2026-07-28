#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export KEEP_DISPLAY=1
ROOT="/home/linux/ws2/ros_ws/navigation2"
DOC="$ROOT/nav2_behavior_tree/doc"
LOG="$ROOT/.ros_home/insert_demo_launch.log"

echo "[0] kill old"
pkill -9 -f 'tb3_simulation_launch.py' 2>/dev/null || true
pkill -9 -f 'component_container_isolated' 2>/dev/null || true
# match binary names only via pgrep then kill
for pat in gzserver gzclient rviz2; do
  pgrep -x "$pat" >/dev/null 2>&1 && pkill -9 -x "$pat" || true
done
pkill -9 -f 'zigzag_garbage_test.py' 2>/dev/null || true
sleep 3
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "[1] launch TB3+Nav2+RViz"
: > "$LOG"
bash "$ROOT/scripts/launch_tb3_insert_garbage_test.sh" >> "$LOG" 2>&1 &
for i in $(seq 1 100); do
  n=$(rg -c 'Managed nodes are active' "$LOG" 2>/dev/null || echo 0)
  if [[ "${n:-0}" -ge 2 ]]; then
    echo "active count=$n after ${i}s"
    break
  fi
  sleep 1
done
sleep 5

set +u
source /opt/ros/humble/setup.bash
source /home/linux/python/capella_clean_garbage/install/capella_ros_msg/share/capella_ros_msg/local_setup.bash
source /home/linux/python/capella_clean_garbage/install/garage_utils_msgs/share/garage_utils_msgs/local_setup.bash 2>/dev/null || true
source "$ROOT/install/nav2_behavior_tree/share/nav2_behavior_tree/local_setup.bash"
source "$ROOT/install/nav2_bt_navigator/share/nav2_bt_navigator/local_setup.bash" 2>/dev/null || true
export ROS_USE_SIM_TIME=true
set -u

echo "[2] republish waypoints"
python3 "$DOC/republish_saved_waypoints.py" --once

echo "[3] reset+nav+garbage"
python3 "$DOC/repub_insert_flow.py"

echo "[4] done"
rg -n 'inserted garbage|head-insert' "$LOG" | tail -5
