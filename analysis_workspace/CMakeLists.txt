
cmake_minimum_required(VERSION 2.8.3)
project(analysis_workspace)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES analyse_workspace
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    moveit_ros_perception
    moveit_ros_planning_interface
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(analyse_workspace src/analyse_workspace.cpp)
target_link_libraries(analyse_workspace ${catkin_LIBRARIES})
add_dependencies(analyse_workspace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_analyse_workspace src/test_analyse_workspace.cpp)
target_link_libraries(test_analyse_workspace analyse_workspace ${catkin_LIBRARIES})
add_dependencies(test_analyse_workspace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
