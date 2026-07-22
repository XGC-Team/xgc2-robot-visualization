cmake_minimum_required(VERSION 3.0.2)
project(xgc2_robot_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs
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

install(TARGETS fs150_uav_visualizer scout_ugv_visualizer mecanum_ugv_visualizer
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
endif()
