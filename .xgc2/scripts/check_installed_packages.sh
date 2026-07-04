#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-robot-visualization >/dev/null
test "$(rospack find xgc2_robot_visualization)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_robot_visualization"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/fs150_uav_visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_robot_visualization/scout_ugv_visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/lib/libfs150_uav_visualizer.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libscout_ugv_visualizer.so"

echo "Installed package check passed"
