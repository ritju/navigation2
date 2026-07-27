#!/usr/bin/env bash
# 启动 TB3 仿真 + 带 InsertGarbagePose 的 Nav2，供 RViz Nav Through Poses 联调。
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PARAMS="${ROOT}/scripts/nav2_params_insert_garbage_test.yaml"
MSG_PREFIX="${CAPELLA_ROS_MSG_PREFIX:-/home/linux/python/capella_clean_garbage/install/capella_ros_msg}"
GARAGE_PREFIX="${GARAGE_UTILS_MSGS_PREFIX:-/home/linux/python/capella_clean_garbage/install/garage_utils_msgs}"

set +u
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "${KEEP_DISPLAY:-0}" != "1" ]]; then
  export DISPLAY=:0
fi
# 注意：不要设 FASTRTPS_DEFAULT_PROFILES_FILE 禁 SHM，会把 component_container 卡死

source /opt/ros/humble/setup.bash

# 只 source 单套消息 + 本仓库两个 overlay 包。
# 禁止 source ${ROOT}/install/setup.bash：它会链入坏掉的 vision_ws，导致
# "Type support not from this implementation"，BT 加载失败、目标被拒。
source "${MSG_PREFIX}/share/capella_ros_msg/local_setup.bash"
if [[ -f "${GARAGE_PREFIX}/share/garage_utils_msgs/local_setup.bash" ]]; then
  source "${GARAGE_PREFIX}/share/garage_utils_msgs/local_setup.bash"
fi
source "${ROOT}/install/nav2_behavior_tree/share/nav2_behavior_tree/local_setup.bash"
source "${ROOT}/install/nav2_bt_navigator/share/nav2_bt_navigator/local_setup.bash"

# 再保险：从路径里剔掉 vision_ws
_strip_vision() {
  local in="$1" out="" p
  IFS=':' read -ra parts <<< "$in"
  for p in "${parts[@]}"; do
    [[ -z "$p" ]] && continue
    [[ "$p" == *vision_ws* ]] && continue
    if [[ -z "$out" ]]; then out="$p"; else out="$out:$p"; fi
  done
  echo "$out"
}
export AMENT_PREFIX_PATH="$(_strip_vision "$AMENT_PREFIX_PATH")"
export CMAKE_PREFIX_PATH="$(_strip_vision "$CMAKE_PREFIX_PATH")"
export LD_LIBRARY_PATH="$(_strip_vision "$LD_LIBRARY_PATH")"
# 消息库置顶，避免误链到其它副本
export LD_LIBRARY_PATH="${MSG_PREFIX}/lib:${GARAGE_PREFIX}/lib:${LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="${MSG_PREFIX}:${GARAGE_PREFIX}:${AMENT_PREFIX_PATH}"
set -u

echo "[launch] capella_ros_msg=$(ros2 pkg prefix capella_ros_msg 2>/dev/null || echo MISSING)"
echo "[launch] nav2_behavior_tree=$(ros2 pkg prefix nav2_behavior_tree)"
echo "[launch] LD has vision? $(echo "$LD_LIBRARY_PATH" | grep -c vision_ws || true)"

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False \
  slam:=False \
  map:=/home/linux/ws2/map/blank_100x100.yaml \
  world:=/home/linux/ws2/map/blank_100x100.world \
  robot_sdf:=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  robot_name:=turtlebot3_burger \
  x_pose:=-2.00 \
  y_pose:=-1.50 \
  params_file:="${PARAMS}"
