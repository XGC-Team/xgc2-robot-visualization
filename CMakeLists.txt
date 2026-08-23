cmake_minimum_required(VERSION 3.0.2)
project(xgc2_robot_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roslib
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer robot_description_runtime robot_path_runtime
  CATKIN_DEPENDS geometry_msgs nav_msgs roslib roscpp std_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fs150_uav_visualizer
  src/fs150_uav_visualizer.cpp
)
add_dependencies(fs150_uav_visualizer
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fs150_uav_visualizer
  ${catkin_LIBRARIES}
)

add_library(scout_ugv_visualizer
  src/scout_ugv_visualizer.cpp
)
add_dependencies(scout_ugv_visualizer
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scout_ugv_visualizer
  ${catkin_LIBRARIES}
)

add_library(mecanum_ugv_visualizer
  src/mecanum_ugv_visualizer.cpp
)
add_dependencies(mecanum_ugv_visualizer
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mecanum_ugv_visualizer
  ${catkin_LIBRARIES}
)

# Strict parser for the immutable, kind-neutral Robot visualization roster.
# Keep it outside the process main so malformed/partial fleet admission is
# covered by ordinary unit tests and cannot regress behind a successful build.
add_library(robot_description_runtime
  src/robot_description_runtime.cpp
)
add_dependencies(robot_description_runtime
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(robot_description_runtime
  ${catkin_LIBRARIES}
)

add_library(robot_path_runtime
  src/path_runtime.cpp
)
add_dependencies(robot_path_runtime
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(robot_path_runtime
  ${catkin_LIBRARIES}
)

# The description publisher is a node, not a library: it turns a robot roster
# plus a per-kind visual URDF into parameters a viewer can load once by name.
add_executable(xgc2_robot_description_publisher_node
  src/robot_description_publisher_node.cpp
)
add_dependencies(xgc2_robot_description_publisher_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(xgc2_robot_description_publisher_node
  robot_description_runtime
  ${catkin_LIBRARIES}
)

install(TARGETS xgc2_robot_description_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer robot_description_runtime robot_path_runtime
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mecanum_ugv_visualizer_test test/mecanum_ugv_visualizer_test.cpp)
  if(TARGET mecanum_ugv_visualizer_test)
    target_link_libraries(mecanum_ugv_visualizer_test mecanum_ugv_visualizer ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(robot_label_anchor_test test/robot_label_anchor_test.cpp)
  if(TARGET robot_label_anchor_test)
    target_link_libraries(robot_label_anchor_test
      fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(robot_description_runtime_test test/robot_description_runtime_test.cpp)
  if(TARGET robot_description_runtime_test)
    target_link_libraries(robot_description_runtime_test robot_description_runtime ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(path_history_test test/path_history_test.cpp)
  if(TARGET path_history_test)
    target_link_libraries(path_history_test
      fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(path_runtime_test test/path_runtime_test.cpp)
  if(TARGET path_runtime_test)
    target_link_libraries(path_runtime_test robot_path_runtime ${catkin_LIBRARIES})
  endif()
endif()
