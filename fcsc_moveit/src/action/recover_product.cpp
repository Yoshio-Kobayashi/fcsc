#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"


bool FcscCore::returnProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  res.success = returnObjectToContainer(req.object);
  return (true);
}

// 掴んだ物体を棚に置けなかったときは諦めてコンテナに戻す。
bool FcscCore::returnObjectToContainer(fcsc_msgs::RecognizedObject object)
{
  geometry_msgs::PoseArray waypoints;
  geometry_msgs::Pose waypoint;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped current_pose;
  double z_offset = 0.1;

  target_pose = getPoseStamped(object.pose, "grasped_"+object.name, move_group.getEndEffectorLink(), listener);

  target_pose.pose.position.z += z_offset;

  asyncMoveArm(target_pose);

  waypoints.header.frame_id = target_pose.header.frame_id;
  waypoint = target_pose.pose;
  waypoint.position.z -= z_offset;
  waypoints.poses.push_back(waypoint);

  moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);

  double current_joint_pos;

  if (robot_hand.jointToPos(robot_hand.getCurrentJointValues()[0]) < robot_hand.jointToPos(robot_hand.getCurrentJointValues()[1])) {
    current_joint_pos = robot_hand.jointToPos(robot_hand.getCurrentJointValues()[0]);
  } else {
    current_joint_pos = robot_hand.jointToPos(robot_hand.getCurrentJointValues()[1]);
  }

  releaseObject(object.name, current_joint_pos + 20);
  fcsc_visual_tools.contactProduct(object.name, "container", object.pose);
  return (true);
}
