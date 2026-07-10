#!/usr/bin/env bash
set -euo pipefail

grep -q '^id: xgc2-robot-visualization$' .xgc2/product.yml
grep -q '^version: 0.1.0-2$' .xgc2/product.yml
grep -q '<name>xgc2_robot_visualization</name>' package.xml
grep -q 'ros-noetic-xgc2-fs150-description' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scout-description' .xgc2/product.yml
grep -q '<depend>fs150_description</depend>' package.xml
grep -q '<depend>scout_description</depend>' package.xml
if grep -R 'gazebo_msgs\\|mavros_msgs' include src package.xml CMakeLists.txt >/tmp/xgc2-robot-visualization-forbidden-deps.txt; then
  echo "xgc2_robot_visualization must not depend on Gazebo or MAVROS." >&2
  cat /tmp/xgc2-robot-visualization-forbidden-deps.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
