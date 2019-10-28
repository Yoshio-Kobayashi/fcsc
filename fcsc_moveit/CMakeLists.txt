
cmake_minimum_required(VERSION 2.8.3)
project(fcsc_moveit)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_msgs
  moveit_ros_perception
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  ezgripper_cpp_driver
  fcsc_msgs
  fcsc_visual_tools
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp
  CATKIN_DEPENDS
    geometric_shapes
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_perception
    moveit_ros_planning_interface
    pluginlib
    roscpp
    fcsc_msgs
    moveit_visual_tools
    ezgripper_cpp_driver
    fcsc_visual_tools
    comm_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(smach_daihen_ur5_moveit_core
  src/smach_daihen_ur5_moveit_core.cpp
  src/action/pickup_product.cpp
  src/action/place_product.cpp
  src/action/recover_product.cpp
  src/action/detect_action.cpp
  src/action/change_sandwich_pose.cpp
  src/action/pickup_sandwich.cpp
  src/action/faceup_sandwich.cpp
  src/action/recover_sandwich.cpp
  src/action/mobile_robot_action.cpp
  src/action/detect_and_recover_sandwich.cpp
  src/grasp_planning.cpp
  src/mobile_robot.cpp
  src/robot_hand.cpp
)
target_link_libraries(smach_daihen_ur5_moveit_core ${catkin_LIBRARIES})
add_dependencies(smach_daihen_ur5_moveit_core ${catkin_EXPORTED_TARGETS})

add_executable(get_base_position_server src/get_base_position_server.cpp)
target_link_libraries(get_base_position_server ${catkin_LIBRARIES})
add_dependencies(get_base_position_server ${catkin_EXPORTED_TARGETS})

add_executable(test_get_base_position_server src/test_get_base_position_server.cpp)
target_link_libraries(test_get_base_position_server ${catkin_LIBRARIES})
add_dependencies(test_get_base_position_server ${catkin_EXPORTED_TARGETS})

add_executable(test_stocking_product src/test_stocking_product.cpp src/mobile_robot.cpp)
target_link_libraries(test_stocking_product ${catkin_LIBRARIES})
add_dependencies(test_stocking_product ${catkin_EXPORTED_TARGETS})

add_executable(test_faceup_product src/test_faceup_product.cpp)
target_link_libraries(test_faceup_product ${catkin_LIBRARIES})
add_dependencies(test_faceup_product ${catkin_EXPORTED_TARGETS})

add_executable(test_recover_product src/test_recover_product.cpp)
target_link_libraries(test_recover_product ${catkin_LIBRARIES})
add_dependencies(test_recover_product ${catkin_EXPORTED_TARGETS})

add_executable(minute_timer src/time_pub.cpp)
target_link_libraries(minute_timer ${catkin_LIBRARIES})
add_dependencies(minute_timer ${catkin_EXPORTED_TARGETS})
