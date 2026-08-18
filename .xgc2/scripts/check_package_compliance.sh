#!/usr/bin/env bash
set -euo pipefail

grep -q '^id: xgc2-robot-visualization$' .xgc2/product.yml
grep -q '^version: 0.2.0-4$' .xgc2/product.yml
grep -q '<name>xgc2_robot_visualization</name>' package.xml
grep -q 'ros-noetic-xgc2-fs150-description (>= 0.1.0-3)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-mecanum-description (>= 0.1.0-1)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scout-description' .xgc2/product.yml
grep -q '^  recommends:$' .xgc2/product.yml
grep -q '^Recommends:.*xgc2-fs150-description.*xgc2-scout-description' .xgc2/scripts/package_debs.sh
grep -q '<exec_depend>fs150_description</exec_depend>' package.xml
grep -q '<exec_depend>mecanum_description</exec_depend>' package.xml
grep -q '<exec_depend>scout_description</exec_depend>' package.xml
grep -q '<depend>roslib</depend>' package.xml
grep -q '<exec_depend>robot_state_publisher</exec_depend>' package.xml
grep -q 'libmecanum_ugv_visualizer.so' .xgc2/scripts/package_debs.sh
grep -q '^DESCRIPTION_PUBLISHER="xgc2_robot_description_publisher_node"$' .xgc2/scripts/package_debs.sh
grep -q 'copy_required_path.*DESCRIPTION_PUBLISHER' .xgc2/scripts/package_debs.sh
grep -q 'test -x.*DESCRIPTION_PUBLISHER' .xgc2/scripts/check_installed_packages.sh
grep -q "grep -F 'not found'" .xgc2/scripts/check_installed_packages.sh
grep -q '/opt/ros/noetic/lib/xgc2_robot_visualization/xgc2_robot_description_publisher_node' .xgc2/product.yml
grep -q '^Depends:.*xgc2-mecanum-description' .xgc2/scripts/package_debs.sh
grep -q '^Depends:.*ros-${ROS_DISTRO}-roslib.*ros-${ROS_DISTRO}-robot-state-publisher' .xgc2/scripts/package_debs.sh
grep -q 'catkin_make run_tests_xgc2_robot_visualization' .xgc2/scripts/build_debs_in_docker.sh
grep -q 'catkin_test_results --verbose' .xgc2/scripts/build_debs_in_docker.sh
grep -q 'xgc2-build-focal-ros-noetic:1.0.0' .xgc2/scripts/build_debs_in_docker.sh
grep -q 'XGC2_APT_OVERLAY_URL requires XGC2_DEPENDENCY_SET_DIGEST' .xgc2/scripts/build_debs_in_docker.sh
if grep -nE 'apt-get (update|install)' .xgc2/scripts/build_debs_in_docker.sh | \
  grep -Ev 'apt-get update$|/workspace/out/'; then
  echo "build dependencies must come from the XGC2 image" >&2
  exit 1
fi
[[ "$(grep -c 'apt-get update$' .xgc2/scripts/build_debs_in_docker.sh)" -eq 1 ]]
grep -q 'robot_description_runtime_test' CMakeLists.txt
grep -q 'XGC2_ROBOT_VISUALIZATION_ROSTER=' .xgc2/scripts/check_installed_packages.sh
grep -q 'rosnode ping -c 1 /gate1/robot_state_publisher' .xgc2/scripts/check_installed_packages.sh
grep -q 'pgrep -f.*robot_state_publisher' .xgc2/scripts/check_installed_packages.sh
grep -q 'rostopic echo -n 1 /xgc/robot_descriptions/ready' .xgc2/scripts/check_installed_packages.sh
grep -q 'rostopic echo -n 1 /tf_static' .xgc2/scripts/check_installed_packages.sh
grep -q '_tf_prefix:=' src/robot_description_publisher_node.cpp
if grep -q '_frame_prefix:=' src/robot_description_publisher_node.cpp; then
  echo "Noetic robot_state_publisher must use tf_prefix, not the ROS 2 frame_prefix parameter." >&2
  exit 1
fi
if grep -R 'gazebo_msgs\\|mavros_msgs' include src package.xml CMakeLists.txt >/tmp/xgc2-robot-visualization-forbidden-deps.txt; then
  echo "xgc2_robot_visualization must not depend on Gazebo or MAVROS." >&2
  cat /tmp/xgc2-robot-visualization-forbidden-deps.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
