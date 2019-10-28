#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

bool FcscCore::pickupProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  geometry_msgs::PoseStamped        current_pose_stamped;
  std::vector<geometry_msgs::Pose>  waypoints;
  geometry_msgs::Pose               waypoint;
  string                            object_name;
  ProductType                       product_type;
  std::vector<int>                  grasp_indices;
  bool                              success;
  std::vector<int>                  *product_placement_indices = NULL;
  int                               product_type_index;
  int                               product_placement_index;
  static bool                       is_lift_upped = false;

  ROS_INFO("[pickupProduct]:pickup [%s]", req.object.name.c_str());

  object_name = req.object.name;

  //物体の種類を設定
  if (object_name.find("onigiri") != std::string::npos) {
    product_type = ONIGIRI;
    product_placement_indices = &onigiri_placement_index;
    product_type_index = 4;
  } else if (object_name.find("drink") != std::string::npos) {
    product_type = DRINK;
    product_placement_indices = &drink_placement_index;
    product_type_index = 2;
  } else if (object_name.find("bento") != std::string::npos) {
    product_type = BENTO;
    product_placement_indices = &bento_placement_index;
    product_type_index = 0;
  } else {
    ROS_ERROR("[pickupProduct]No such a product name");
    res.success = false;
    return (true);
  }

  if (object_name.find("_A_") != std::string::npos) {
    product_placement_index = (*product_placement_indices)[0];
  } else if (object_name.find("_B_") != std::string::npos){
    product_placement_index = (*product_placement_indices)[1];
    ++product_type_index;
  } else {
    ROS_ERROR("[pickupProduct]No such a type");
    res.success = false;
    return (true);
  }

  if (product_placement_index >= object_placements[product_type_index].placements.size()) {
    ROS_ERROR("[pickupProduct]There is NO product location");
    res.success = false;
    return (true);
  }

  // move_group.setSupportSurfaceName("container");

  move_group.setMaxVelocityScalingFactor(max_velocity);
  asyncMoveArm(container_center_pose_name);
  move_group.setMaxVelocityScalingFactor(normal_velocity);

  // おにぎり、ドリンクを陳列するときは台車を上げる
  // if (!is_lift_upped && (product_type == ONIGIRI || product_type == DRINK)) {
  //   waitForExecute();
  //   // mobile_robot.moveBoard(0.09);
  //
  //   // コンテナ内の商品がロボットにめり込んでいるので、再度描画する
  //   fcsc_msgs::DetectObject detect_object_srv;
  //   detect_object_srv.request.detect = false;
  //   detect_object_srv.request.visualize = true;
  //   detect_object_client.call(detect_object_srv);
  //
  //   is_lift_upped = true;
  // }

  sortGraspPoseIndex(product_type, object_name, grasp_indices);

  res.success = false;
  for (size_t i = 0; i < grasp_indices.size(); i++) {
    product_grasps[product_type].grasps[ grasp_indices[i] ].grasp_pose.header.frame_id = object_name;
    product_grasps[product_type].grasps[ grasp_indices[i] ].pre_grasp_approach.direction.header.frame_id = object_name;
    bool success = pickup(object_name, product_grasps[product_type].grasps[ grasp_indices[i] ], moveit_msgs::Constraints(), false);
    if (success) {
      res.success = true;
      break;
    }
  }

  // asyncMoveArm(container_center_pose_name);

  return (true);
}
