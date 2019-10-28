cmake_minimum_required(VERSION 2.8.3)
project(ezgripper_cpp_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp actionlib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ezgripper_cpp_driver
 CATKIN_DEPENDS roscpp actionlib
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ezgripper_interface.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ezgripper_test src/ezgripper_test src/ezgripper_interface.cpp)
target_link_libraries(ezgripper_test ${catkin_LIBRARIES})
