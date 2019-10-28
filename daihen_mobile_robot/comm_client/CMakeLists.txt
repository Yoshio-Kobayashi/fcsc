cmake_minimum_required(VERSION 2.8.3)
project(comm_client)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
)

add_service_files(
  FILES
  RobotPose.srv
  GoalPose.srv
  MoveLift.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES comm_client
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    sensor_msgs
    tf
  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(Comm_Client
  src/CommToArm.cpp
  src/CommToTransRobot.cpp
  src/DBasicTypes.cpp
  src/MyString.cpp
  src/Netif.cpp
  src/Socket.cpp
)
target_link_libraries(
   Comm_Client
   ${catkin_LIBRARIES}
 )
add_dependencies(
  Comm_Client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

 add_executable(Comm_Simulation
 src/CommToSimulation.cpp
)
target_link_libraries(
  Comm_Simulation
  ${catkin_LIBRARIES}
)
add_dependencies(
  Comm_Simulation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
