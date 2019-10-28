cmake_minimum_required(VERSION 2.8.3)
project(fcsc_visual_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  fcsc_msgs
  moveit_visual_tools
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fcsc_visual_tools random_sandwich_generator
 # CATKIN_DEPENDS roscpp moveit_msgs fcsc_msgs moveit_visual_tools
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fcsc_visual_tools src/fcsc_visual_tools.cpp)
add_dependencies(fcsc_visual_tools ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcsc_visual_tools ${catkin_LIBRARIES})

add_library(random_sandwich_generator src/random_sandwich_generator.cpp)
add_dependencies(random_sandwich_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(random_sandwich_generator fcsc_visual_tools ${catkin_LIBRARIES})

add_executable(spawn_shelf node/spawn_shelf.cpp)
add_dependencies(spawn_shelf ${catkin_EXPORTED_TARGETS})
target_link_libraries(spawn_shelf fcsc_visual_tools ${catkin_LIBRARIES})

add_executable(spawn_sandwich node/spawn_sandwich.cpp)
add_dependencies(spawn_sandwich ${catkin_EXPORTED_TARGETS})
target_link_libraries(spawn_sandwich fcsc_visual_tools ${catkin_LIBRARIES})

add_executable(spawn_randomly_sandwich node/spawn_randomly_sandwich.cpp)
add_dependencies(spawn_randomly_sandwich ${catkin_EXPORTED_TARGETS})
target_link_libraries(spawn_randomly_sandwich fcsc_visual_tools random_sandwich_generator ${catkin_LIBRARIES})
