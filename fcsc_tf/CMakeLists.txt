cmake_minimum_required(VERSION 2.8.3)
project(fcsc_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  fcsc_msgs
)

catkin_package(
 LIBRARIES fcsc_tf
 CATKIN_DEPENDS roscpp tf fcsc_msgs
 # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(detected_object_tf_broadcaster src/detected_object_tf_broadcaster.cpp)
target_link_libraries(detected_object_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(detected_object_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(grasped_object_tf_broadcaster src/grasped_object_tf_broadcaster.cpp)
target_link_libraries(grasped_object_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(grasped_object_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(shelf_tf_broadcaster src/shelf_tf_broadcaster.cpp)
target_link_libraries(shelf_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(shelf_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(sandwich_tf_broadcaster src/sandwich_tf_broadcaster.cpp)
target_link_libraries(sandwich_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(sandwich_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(virtual_joint_broadcaster src/virtual_joint_broadcaster.cpp)
target_link_libraries(virtual_joint_broadcaster ${catkin_LIBRARIES})
