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
test -f "/opt/ros/${ROS_DISTRO}/lib/libfs150_uav_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libscout_ugv_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libmecanum_ugv_visualizer.so"
test -x "${DESCRIPTION_PUBLISHER}"

description_publisher_ldd="$(ldd "${DESCRIPTION_PUBLISHER}")"
if grep -F 'not found' <<<"${description_publisher_ldd}"; then
  echo "xgc2_robot_description_publisher_node has unresolved runtime libraries" >&2
  exit 1
fi

echo "Installed package check passed"
