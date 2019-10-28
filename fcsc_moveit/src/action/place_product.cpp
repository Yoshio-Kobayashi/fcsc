#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

bool FcscCore::placeProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ProductType        product_type;
  std::vector<int>  *product_placement_indices = NULL;
  int               *product_placement_index = NULL;
  int               product_type_index;
  bool    success = false;
  std::string shelf_board_name;
  std::string mesh_path;
  std::string return_pose_name;
  moveit_msgs::Constraints constraint;
  std::vector<moveit_msgs::PlaceLocation> locations;
  fcsc_msgs::DetectObject detect_object_srv;

  ROS_WARN("[placeObject] %s", req.object.name.c_str());
  move_group.setSupportSurfaceName("shelfA");

  // 陳列商品ごとに変数設定
  switch (req.object.type) {
    case fcsc_msgs::RecognizedObject::ONIGIRI: {
      product_type = ONIGIRI;
      mesh_path = "package://fcsc_description/mesh/onigiri_edit.stl";
      return_pose_name = "onigiri_return_pose";
      product_placement_indices = &onigiri_placement_index;
      product_type_index = 4;
      break;
    }
    case fcsc_msgs::RecognizedObject::DRINK: {
      product_type = DRINK;
      mesh_path = "package://fcsc_description/mesh/drink_edit.stl";
      return_pose_name = "masita4";
      product_placement_indices = &drink_placement_index;
      product_type_index = 2;
      break;
    }
    case fcsc_msgs::RecognizedObject::BENTO: {
      product_type = BENTO;
      mesh_path = "package://fcsc_description/mesh/bento_edit.stl";
      return_pose_name = stock_pose_name;
      product_placement_indices = &bento_placement_index;
      product_type_index = 0;
      break;
    }
  }

  // 陳列商品の種類（A/B）ごとに配置場所のインデックスを設定
  if (req.object.name.find("_A_") != std::string::npos) {
    product_placement_index = &((*product_placement_indices)[0]);
  } else if (req.object.name.find("_B_") != std::string::npos) {
    product_placement_index = &((*product_placement_indices)[1]);
    ++product_type_index ;
  } else {
    ROS_ERROR("[placeProduct]No such a type");
    res.success = false;
    return (true);
  }

  // 陳列商品の配置場所があるか確認
  if (*product_placement_index >= object_placements[product_type_index].placements.size()) {
    ROS_ERROR("[placeProduct]:There is NO product location");
    res.success = false;
    return (true);
  }

  // 商品陳列時の手先姿勢を設定
  grasp_planner.generateLocationPose(req.object.name, stocking_area[product_type].child_frame_id, &locations);
  shelf_board_name = stocking_area[product_type].parent_frame_id;

  // 手先をなるべく真下に向けた状態のまま商品を置くための制約
  if (product_type == DRINK || product_type == BENTO) {
    constraint.orientation_constraints.resize(1);
    constraint.orientation_constraints[0].header.frame_id = move_group.getCurrentPose().header.frame_id;
    constraint.orientation_constraints[0].orientation = move_group.getCurrentPose().pose.orientation;
    constraint.orientation_constraints[0].link_name = move_group.getEndEffectorLink();
    // 手先リンク座標系(gripper_link)基準のトレランス
    // x軸は手首のyaw回転
    constraint.orientation_constraints[0].absolute_x_axis_tolerance = M_PI;
    constraint.orientation_constraints[0].absolute_y_axis_tolerance = M_PI / 6;
    constraint.orientation_constraints[0].absolute_z_axis_tolerance = M_PI / 6;
    constraint.orientation_constraints[0].weight = 1.0;

    constraint.joint_constraints.resize(1);
    constraint.joint_constraints[0].joint_name = "shoulder_pan_joint";
    constraint.joint_constraints[0].position = move_group.getCurrentJointValues()[0];
    constraint.joint_constraints[0].tolerance_above = 100;
    constraint.joint_constraints[0].tolerance_below = 0;
    constraint.joint_constraints[0].weight = 1.0;
  } else {
    // constraint.joint_constraints.resize(2);
    // constraint.joint_constraints[0].joint_name = "wrist_1_joint";
    // constraint.joint_constraints[0].position = 2.0 * M_PI;
    // constraint.joint_constraints[0].tolerance_above = 0;
    // constraint.joint_constraints[0].tolerance_below = constraint.joint_constraints[0].position - M_PI;
    // constraint.joint_constraints[0].weight = 1.0;
    //
    // constraint.joint_constraints[1].joint_name = "shoulder_pan_joint";
    // constraint.joint_constraints[1].position = move_group.getCurrentJointValues()[0] + 10.0 * M_PI / 180.0;
    // constraint.joint_constraints[1].tolerance_above = 100;
    // constraint.joint_constraints[1].tolerance_below = 0;
    // constraint.joint_constraints[1].weight = 1.0;
  }

  if (product_type != ONIGIRI) {
    move_group.setNamedTarget(stock_pose_name);
  } else {
    asyncMoveArm("fold");
    move_group.setNamedTarget("onigiri_pose");
  }

  if (!asyncPlanArm(plan, 3)) {
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    // move_group.setPlanningTime(5);
    res.success = false;
    return (true);
  }

  if (!asyncExecuteArm(plan)) {
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    // move_group.setPlanningTime(5);
    res.success = false;
    return (true);
  }

  // 棚の手前に移動
  switch (product_type) {
    case DRINK: {
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = shelf_board_name;
      target_pose.pose.position.x = -0.1;
      target_pose.pose.position.y = shelf_width / 2.0;
      target_pose.pose.position.z = 0.1;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2.0, 0);
      setStartState();
      asyncMoveArm(target_pose, 3);
      break;
    }
    case ONIGIRI: {
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = shelf_board_name;
      target_pose.pose.position.x = -0.3;
      target_pose.pose.position.y = shelf_width / 2.0;
      target_pose.pose.position.z = 0.3;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI / 3, 0);
      target_pose = getPoseStamped(target_pose, "gripper_wrap_link", "gripper_link", listener);
      setStartState();
      asyncMoveArm(target_pose, 3);
      break;
    }
    case BENTO: {
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = shelf_board_name;
      target_pose.pose.position.x = -0.15;
      target_pose.pose.position.y = shelf_width / 2.0;
      target_pose.pose.position.z = 0.2;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2.0, 0);
      setStartState();
      asyncMoveArm(target_pose, 3);
      break;
    }
  }

  success = false;
  while (*product_placement_index < object_placements[product_type_index].placements.size()) {
    ROS_WARN("name:%s placement_index:%d", req.object.name.c_str(), *product_placement_index);
    for (size_t i = 0; i < locations.size(); i++) {
      locations[i].place_pose.pose.position = object_placements[product_type_index].placements[*product_placement_index].pose.position;

      if (product_type == BENTO && locations[i].id == "tilt") {
        locations[i].place_pose.pose.position.z += 0.05;
      }

      geometry_msgs::PoseStamped pout;
      transformPose(visual_frame_id_, locations[i].place_pose, pout, listener);
      if (visual_tools->publishMesh(pout.pose, mesh_path, rviz_visual_tools::BLUE)) {
        visual_tools->trigger();
      }

      success = place(req.object.name, locations[i], constraint);
      if (!success) {
        // Stop();
        continue;
      }

      switch (product_type) {
        case BENTO: {
          locations[i].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
          // locations[i].place_pose.pose.position.z -= 0.05 / 2.0;
          // fcsc_visual_tools.publishProduct(req.object.name, locations[i].place_pose);
          fcsc_visual_tools.contactProduct(req.object.name, "bento", locations[i].place_pose, *product_placement_index);
          break;
        }
        case DRINK: {
          locations[i].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
          fcsc_visual_tools.contactProduct(req.object.name, shelf_board_name, locations[i].place_pose);
          break;
        }
        case ONIGIRI: {
          locations[i].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI/2);
          fcsc_visual_tools.contactProduct(req.object.name, shelf_board_name, locations[i].place_pose);
          break;
        }
      }

      res.success = true;

      ++(*product_placement_index);
      detect_object_srv.request.delete_object_names.push_back(req.object.name);
      detect_object_client.call(detect_object_srv);

      asyncMoveArm(return_pose_name);
      move_group.setMaxVelocityScalingFactor(max_velocity);
      asyncMoveArm(stock_pose_name);
      asyncMoveArm("fold");
      move_group.setMaxVelocityScalingFactor(normal_velocity);


      return (true);
    }
    ++(*product_placement_index);
  }

  res.success = false;

  releaseObject(req.object.name, 100);
  fcsc_visual_tools.deleteObject(req.object.name);
  detect_object_srv.request.delete_object_names.push_back(req.object.name);
  detect_object_client.call(detect_object_srv);
  asyncMoveArm(stock_pose_name);
  asyncMoveArm("fold");

  return (true);
}
