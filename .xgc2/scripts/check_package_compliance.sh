#!/usr/bin/env bash
set -euo pipefail

grep -q '^id: xgc2-robot-visualization$' .xgc2/product.yml
grep -q '^version: 0.1.0-16$' .xgc2/product.yml
grep -q '<name>xgc2_robot_visualization</name>' package.xml
grep -q 'ros-noetic-xgc2-fs150-description (>= 0.1.0-3)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-mecanum-description (>= 0.1.0-1)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scout-description' .xgc2/product.yml
grep -q '^  recommends:$' .xgc2/product.yml
grep -q '^Recommends:.*xgc2-fs150-description.*xgc2-scout-description' .xgc2/scripts/package_debs.sh
grep -q '<exec_depend>fs150_description</exec_depend>' package.xml
grep -q '<exec_depend>mecanum_description</exec_depend>' package.xml
grep -q '<exec_depend>scout_description</exec_depend>' package.xml
grep -q 'libmecanum_ugv_visualizer.so' .xgc2/scripts/package_debs.sh
grep -q '^DESCRIPTION_PUBLISHER="xgc2_robot_description_publisher_node"$' .xgc2/scripts/package_debs.sh
grep -q 'copy_required_path.*DESCRIPTION_PUBLISHER' .xgc2/scripts/package_debs.sh
grep -q 'test -x.*DESCRIPTION_PUBLISHER' .xgc2/scripts/check_installed_packages.sh
grep -q "grep -F 'not found'" .xgc2/scripts/check_installed_packages.sh
grep -q '/opt/ros/noetic/lib/xgc2_robot_visualization/xgc2_robot_description_publisher_node' .xgc2/product.yml
grep -q '^Depends:.*xgc2-mecanum-description' .xgc2/scripts/package_debs.sh
if grep -R 'gazebo_msgs\\|mavros_msgs' include src package.xml CMakeLists.txt >/tmp/xgc2-robot-visualization-forbidden-deps.txt; then
  echo "xgc2_robot_visualization must not depend on Gazebo or MAVROS." >&2
  cat /tmp/xgc2-robot-visualization-forbidden-deps.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
