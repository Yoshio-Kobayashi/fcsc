cmake_minimum_required(VERSION 2.8.3)
project(trajectory_data_handler)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  pluginlib
  roscpp
  fcsc_msgs
  rosbag
  moveit_visual_tools
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_data_handler
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    pluginlib
    roscpp
    fcsc_msgs
    rosbag
    moveit_visual_tools
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(trajectory_data_handler src/trajectory_data_handler.cpp)
target_link_libraries(trajectory_data_handler  ${catkin_LIBRARIES})
add_dependencies(trajectory_data_handler ${catkin_EXPORTED_TARGETS})

add_executable(load_trajectory_node src/load_trajectory_node.cpp)
target_link_libraries(load_trajectory_node trajectory_data_handler ${catkin_LIBRARIES})
add_dependencies(load_trajectory_node ${catkin_EXPORTED_TARGETS})
