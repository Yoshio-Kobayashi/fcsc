cmake_minimum_required(VERSION 2.8.3)
project(fcsc_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_visual_tools
  fcsc_msgs
  fcsc_visual_tools
  roscpp
  tf
  # darknet_ros_msgs
  image_transport
  image_geometry
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)


catkin_package(
 # INCLUDE_DIRS include
 LIBRARIES fcsc_perception
 CATKIN_DEPENDS geometry_msgs moveit_core moveit_msgs roscpp tf ar_track_alvar_msgs fcsc_msgs  fcsc_visual_tools image_transport image_geometry pcl_ros
 # darknet_ros_msgs
 # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(detect_object_server src/object_detection_server.cpp)
target_link_libraries(detect_object_server ${catkin_LIBRARIES})
add_dependencies(detect_object_server ${catkin_EXPORTED_TARGETS})

add_executable(shelf_detector src/shelf_detector.cpp)
target_link_libraries(shelf_detector ${catkin_LIBRARIES})
add_dependencies(shelf_detector ${catkin_EXPORTED_TARGETS})

add_executable(ar_pose_to_object_pose_server src/ar_pose_to_object_pose_server.cpp)
target_link_libraries(ar_pose_to_object_pose_server ${catkin_LIBRARIES})
add_dependencies(ar_pose_to_object_pose_server ${catkin_EXPORTED_TARGETS})

add_executable(estimate_object_position src/clustering_ros_srv.cpp)
target_link_libraries(estimate_object_position ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(estimate_object_position ${catkin_EXPORTED_TARGETS})

# add_executable(estimate_sandwich_position src/estimate_sandwich_position.cpp)
# target_link_libraries(estimate_sandwich_position ${catkin_LIBRARIES})
# add_dependencies(estimate_sandwich_position ${catkin_EXPORTED_TARGETS})
