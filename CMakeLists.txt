cmake_minimum_required(VERSION 2.4.6)
project(vfh_local_planner)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  pluginlib
  roscpp
  rospy
)


generate_dynamic_reconfigure_options(
  cfg/VFHPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_path_follower
#  CATKIN_DEPENDS base_local_planner costmap_2d nav_core pluginlib roscpp rospy
  DEPENDS eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  vfh_local_planner
  src/vfh_planner_ros.cpp
  src/vfh_algorithm.cc
)

target_link_libraries(vfh_local_planner
   ${catkin_LIBRARIES}
)

