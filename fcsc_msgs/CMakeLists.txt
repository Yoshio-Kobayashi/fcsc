cmake_minimum_required(VERSION 2.8.3)
project(fcsc_msgs)

set(MSG_DEPS
  geometry_msgs
  moveit_msgs
  # object_recognition_msgs
)

set(PKG_DEPS
  pluginlib
  roscpp
  geometric_shapes
  interactive_markers
  message_generation
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

add_service_files(
  FILES
  DetectObject.srv
  GetBasePosition.srv
  finish_navigation.srv

  # smach
  GoToNearShelf.srv
  Manipulate.srv
  DetectProduct.srv
  DetectShelf.srv
  DetectSandwich.srv
  PickupProduct.srv
  PlaceProduct.srv
  PickupSandwich.srv
  PlaceSandwich.srv
  FaceupSandwich.srv
  RecoverSandwich.srv
  MoveInitialPose.srv
  ReturnProduct.srv
  SortOrderManipulation.srv
  EstimateSandwichPosition.srv
  DetectAndRecoverSandwich.srv
  EstimateObjectPosition.srv
)

add_message_files(
  FILES
  PoseStampedWithCost.msg
  RecognizedObject.msg
  RecognizedObjectArray.msg
  Grasp.msg
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})
