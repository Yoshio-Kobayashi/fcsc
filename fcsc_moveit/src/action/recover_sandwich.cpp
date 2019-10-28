#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"


bool FcscCore::recoverSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  fcsc_msgs::DetectObject detect_object_srv;
  geometry_msgs::PoseStamped waypoint_stamped;

  asyncMoveArm(recover_pose_name);

  move_group.setMaxVelocityScalingFactor(max_velocity);

  asyncMoveArm("fold");

  // 廃棄品のサンドイッチが棚にぶつかっても問題はない
  // したがって、あらかじめ掴んだサンドイッチをRviz上から消しておく
  detachObject(req.object.name, attach_detach_type);
  fcsc_visual_tools.deleteObject(req.object.name);
  detect_object_srv.request.delete_object_names.push_back(req.object.name);
  detect_object_client.call(detect_object_srv);

  // コンテナ上へ持っていく
  ROS_INFO("[recoverObject]:Take [%s] to container", req.object.name.c_str());
  asyncMoveArm(container_center_pose_name);

  //サンドイッチを離す
  ROS_INFO("[faceUpObject]:Release [%s]", req.object.name.c_str());
  waitForExecute();
  robot_hand.move(30);

  asyncMoveArm("fold");

  move_group.setMaxVelocityScalingFactor(normal_velocity);
  asyncMoveArm(recover_pose_name);

  res.success = true;


  return (true);
}
