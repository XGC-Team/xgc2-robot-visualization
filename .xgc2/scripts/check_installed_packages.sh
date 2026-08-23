#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
DESCRIPTION_PUBLISHER="/opt/ros/${ROS_DISTRO}/lib/xgc2_robot_visualization/xgc2_robot_description_publisher_node"

dpkg -s ros-noetic-xgc2-robot-visualization >/dev/null
test "$(rospack find xgc2_robot_visualization)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_robot_visualization"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/fs150_uav_visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/scout_ugv_visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/path_history.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/path_runtime.hpp"
test -f "/opt/ros/${ROS_DISTRO}/lib/libfs150_uav_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libscout_ugv_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libmecanum_ugv_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/librobot_description_runtime.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/librobot_path_runtime.so"
test -x "${DESCRIPTION_PUBLISHER}"

description_publisher_ldd="$(ldd "${DESCRIPTION_PUBLISHER}")"
if grep -F 'not found' <<<"${description_publisher_ldd}"; then
  echo "xgc2_robot_description_publisher_node has unresolved runtime libraries" >&2
  exit 1
fi

# Exercise the installed product exactly as the Session process catalog does:
# one immutable roster, namespaced descriptions, and a supervised namespaced
# robot_state_publisher. A source-tree unit test cannot prove that rosrun,
# package lookup, the installed URDF, or child-process supervision is usable.
ROSCORE_PID=""
DESCRIPTION_PID=""
cleanup_runtime_gate() {
  if [[ -n "${DESCRIPTION_PID}" ]]; then
    kill "${DESCRIPTION_PID}" 2>/dev/null || true
    wait "${DESCRIPTION_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ROSCORE_PID}" ]]; then
    kill "${ROSCORE_PID}" 2>/dev/null || true
    wait "${ROSCORE_PID}" 2>/dev/null || true
  fi
}
trap cleanup_runtime_gate EXIT

wait_for_gate() {
  local label="$1"
  shift
  for _ in $(seq 1 150); do
    if "$@" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  echo "timed out waiting for installed runtime gate: ${label}" >&2
  return 1
}

roscore >/tmp/xgc2-robot-visualization-roscore.log 2>&1 &
ROSCORE_PID=$!
wait_for_gate "ROS master" rosparam list

export XGC2_ROBOT_VISUALIZATION_ROSTER='[{"name":"gate1","namespace":"/gate1","descriptionPackage":"mecanum_description","descriptionFile":"urdf/mecanum_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}]'
rosrun xgc2_robot_visualization xgc2_robot_description_publisher_node \
  __name:=xgc2_robot_description_publisher \
  >/tmp/xgc2-robot-visualization-publisher.log 2>&1 &
DESCRIPTION_PID=$!

wait_for_gate "visual description parameter" rosparam get /gate1/visual_robot_description
wait_for_gate "state-publisher description parameter" rosparam get /gate1/robot_description
wait_for_gate "namespaced robot_state_publisher" rosnode ping -c 1 /gate1/robot_state_publisher
wait_for_gate "managed robot_state_publisher process" \
  pgrep -f '^/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher .*__ns:=/gate1'
timeout 15 rostopic echo -n 1 /xgc/robot_descriptions/ready >/dev/null
timeout 15 rostopic echo -n 1 /tf_static >/tmp/xgc2-robot-visualization-tf-static.log
grep -q 'gate1/' /tmp/xgc2-robot-visualization-tf-static.log

kill "${DESCRIPTION_PID}"
wait "${DESCRIPTION_PID}"
DESCRIPTION_PID=""
if pgrep -f '^/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher .*__ns:=/gate1' >/dev/null; then
  echo "managed robot_state_publisher survived description runtime shutdown" >&2
  exit 1
fi

echo "Installed package check passed"
