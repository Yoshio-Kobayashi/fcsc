#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"


bool FcscCore::faceupSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  std::vector<visualization_msgs::Marker> baselines;
  double z_offset = 0.05;
  bool success;

  // 基準線の設定
  // 棚の手前 5cm 以内
  baselines.resize(4);
  for (size_t i = 0; i < baselines.size(); i++) {
    baselines[i].header.frame_id = shelfB_board_name;
    baselines[i].points.resize(2);
    // x = サンドイッチの奥行き + サンドイッチと棚の距離
    baselines[i].points[0].x = baselines[i].points[1].x = 0.09 + 0.01 * (i+1);
    baselines[i].points[0].y = 0.08 + 0.08;
    baselines[i].points[1].y = shelf_width - (0.08 + 0.08);
  }

  // マニピュレーション

  // 基準線上のどこかに目標位置を設定して移動
  for (size_t i = 0; i < baselines.size(); i++) {
    double particle = 20;
    geometry_msgs::Vector3 vec;

    vec.x = (baselines[i].points[1].x - baselines[i].points[0].x) / particle;
    vec.y = (baselines[i].points[1].y - baselines[i].points[0].y) / particle;

    for (double j = 0; j <= particle; j++) {
      success = false;
      // サンドイッチと棚の干渉チェックをやりたいから、何か処理をここに書く
      geometry_msgs::PoseStamped target_ee_pose;
      geometry_msgs::PoseStamped target_object_pose;

      if (req.object.pose.header.frame_id != shelfB_board_name) {
        geometry_msgs::PoseStamped ps_out;
        transformPose(shelfB_board_name, req.object.pose, ps_out, listener);
        req.object.pose = ps_out;
      }

      target_object_pose.header.frame_id = shelfB_board_name;
      target_object_pose.pose.orientation.w = 1.0;
      target_object_pose.pose.position.x = baselines[i].points[0].x + j * vec.x;
      target_object_pose.pose.position.y = baselines[i].points[0].y + j * vec.y;
      target_object_pose.pose.position.z = req.object.pose.pose.position.z + z_offset;

      target_ee_pose = getPoseStamped(target_object_pose, "grasped_"+req.object.name, move_group.getEndEffectorLink(), listener);

      if (!computeIK(target_ee_pose)) {
        continue;
      }

      // 目標位置に物体を可視化
      geometry_msgs::PoseStamped pout;
      transformPose(visual_frame_id_, target_object_pose, pout, listener);
      if (visual_tools->publishMesh(pout.pose, "package://fcsc_description/mesh/sandwich-v2.stl", rviz_visual_tools::BLUE)) {
        visual_tools->trigger();
      }

      geometry_msgs::PoseArray waypoints;
      waypoints.header.frame_id = target_ee_pose.header.frame_id;
      waypoints.poses.push_back(target_ee_pose.pose);
      // success = moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0);

      // success = asyncMoveArm(recover_pose_name);
      // if (!success) {
      //   continue;
      // }

      success = asyncMoveArm(target_ee_pose);
      if (success) {
        target_ee_pose.pose.position.z -= z_offset / 2.0;
        waypoints.poses[0] = target_ee_pose.pose;
        moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);
        break;
      }
    }
    if (success) {
      break;
    }
  }

  res.success = success;

  geometry_msgs::PoseStamped object_pose;

  releaseObject(req.object.name, 50);
  getCurrentObjectPose(req.object.name, shelfB_board_name, object_pose);
  if (req.object.pose.header.frame_id != shelfB_board_name) {
    geometry_msgs::PoseStamped ps_out;
    transformPose(shelfB_board_name, req.object.pose, ps_out, listener);
    object_pose.pose.position.z = ps_out.pose.position.z;
  }
  fcsc_visual_tools.contactProduct(req.object.name, shelfB_board_name, object_pose);

  return (true);
}

bool FcscCore::faceupSandwiches(fcsc_msgs::FaceupSandwich::Request &req, fcsc_msgs::FaceupSandwich::Response &res)
{
  fcsc_msgs::Manipulate manipulate_srv;
  fcsc_msgs::SortOrderManipulation sort_order_srv;

  asyncMoveArm(recover_pose_name);

  manipulate_srv.request.manipulation_type = fcsc_msgs::Manipulate::Request::FACEUP;

  // 棚の手前のサンドイッチから作業を優先する
  sort_order_srv.request.objects = req.sandwiches;
  sortOrderPickingSandwiches(sort_order_srv.request, sort_order_srv.response);

  for (size_t i = 0; i < sort_order_srv.response.objects.size(); i++) {
    fcsc_msgs::RecognizedObject sandwich = sort_order_srv.response.objects[i];
    if (sandwich.state == fcsc_msgs::RecognizedObject::LEFT_SIDE || sandwich.state == fcsc_msgs::RecognizedObject::RIGHT_SIDE) {
      // 倒れたサンドイッチを立てる
      fcsc_msgs::Manipulate manipulate_srv;
      manipulate_srv.request.object = sandwich;
      waitForExecute();
      if (!changeSandwichPose(manipulate_srv.request, manipulate_srv.response)) {
        move_group.setMaxVelocityScalingFactor(normal_velocity);
        continue;
      }

      move_group.setMaxVelocityScalingFactor(normal_velocity);

      bool detected = false;

      // カメラを動かして立てたサンドイッチの位置を検出する
      for (size_t k = 0; k < 3; k++) {
        geometry_msgs::PoseStamped camera_pose;
        geometry_msgs::PoseStamped target_pose;
        transformPose(shelfB_board_name, sandwich.pose, camera_pose, listener);
        switch (k) {
          case 0: {
            camera_pose.pose.position.z = 0.2;
            camera_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2.0, 0);
            break;
          }
          case 1: {
            camera_pose.pose.position.z = 0.2;
            camera_pose.pose.position.x -= 0.15;
            camera_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 3.0, 0);
            break;
          }
          case 2: {
            camera_pose.pose.position.z = 0.2;
            camera_pose.pose.position.x -= 0.15;
            camera_pose.pose.position.y = 0.45;
            camera_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 4.0, 0);
            break;
          }
        }
        ROS_ERROR("[faceupSandwiches]:[%d]", (int)k);
        target_pose = getPoseStamped(camera_pose, "camera_optical_center_line_link", "gripper_link", listener);
        target_pose.pose.position.z = 0.3;
        if (!moveArm(target_pose)) {
          continue;
        }
        ros::WallDuration(3.0).sleep();

        // マーカー認識
        fcsc_msgs::DetectObject detect_object_srv;
        detect_object_srv.request.detect = true;
        detect_object_srv.request.visualize = true;
        detect_object_client.call(detect_object_srv);

        // サンドイッチの位置を更新
        for (size_t j = 0; j < detect_object_srv.response.detected_object_array.objects.size(); j++) {
          fcsc_msgs::RecognizedObject detected_sandwich = detect_object_srv.response.detected_object_array.objects[j];
          if (detected_sandwich.name == sandwich.name && detected_sandwich.state == fcsc_msgs::RecognizedObject::BOTTOM) {
            detected = true;
            sandwich = detected_sandwich;
            break;
          }
        }// update sandwich loop
        if (detected) {
          break;
        }
      }// redetect sandwich loop
      if (!detected) {
        continue;
      }
    } else if (sandwich.state != fcsc_msgs::RecognizedObject::BOTTOM) {
      continue;
    }
    manipulate_srv.request.object = sandwich;
    pickupSandwich(manipulate_srv.request, manipulate_srv.response);
    if (!manipulate_srv.response.success) {
      continue;
    }
    asyncMoveArm(recover_pose_name);
    faceupSandwich(manipulate_srv.request, manipulate_srv.response);
    if (manipulate_srv.response.success) {
      // フェイスアップが成功すれば認識した物体情報を消しておく
      fcsc_msgs::DetectObject srv;
      srv.request.delete_object_names.push_back(sandwich.name);
      srv.request.visualize = false;
      detect_object_client.call(srv);
    }
    asyncMoveArm(recover_pose_name);
  }

  res.success = true;
  return (true);
}
