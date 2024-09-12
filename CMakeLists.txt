cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf2_ros
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core pluginlib roscpp tf2_ros sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dwa_planner
   src/dwa_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  base_local_planner
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES pure_pursuit_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



