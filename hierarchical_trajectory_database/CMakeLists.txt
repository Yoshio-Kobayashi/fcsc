cmake_minimum_required(VERSION 2.8.3)
project(hierarchical_trajectory_database)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dmp
  trajectory_data_handler
  moveit_core
  moveit_ros_planning_interface
  message_generation
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  GenerateTrajectory.srv
)
generate_messages(DEPENDENCIES trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hierarchical_trajectory_database
  CATKIN_DEPENDS
    roscpp
    dmp
    trajectory_data_handler
    moveit_core
    moveit_ros_planning_interface
    message_generation
    trajectory_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(hierarchical_trajectory_database src/hierarchical_trajectory_database.cpp)
target_link_libraries(hierarchical_trajectory_database ${catkin_LIBRARIES})

add_executable(generate_trajectory_server src/generate_trajectory_server.cpp)
target_link_libraries(generate_trajectory_server hierarchical_trajectory_database ${catkin_LIBRARIES})
add_dependencies(generate_trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_astar src/test_astar.cpp)
target_link_libraries(test_astar ${catkin_LIBRARIES})
add_dependencies(test_astar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
