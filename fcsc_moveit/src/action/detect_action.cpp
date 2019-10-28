#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

bool FcscCore::detectProduct(fcsc_msgs::DetectProduct::Request &req, fcsc_msgs::DetectProduct::Response &res)
{
  std::vector<std::string> object_names;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  fcsc_msgs::DetectObject detect_object_srv;
  std::vector<geometry_msgs::PoseStamped> search_target_pose_v;
  bool success;

  // 偽の検出
  // コンテナ内の物体配置をあらかじめ決めているときに使用
  ROS_INFO("searchObject DAMY_DETECTION");

  asyncMoveArm(container_center_pose_name);

  detect_object_srv.request.detect = true;
  detect_object_srv.request.visualize = true;
  detect_object_srv.request.damy = true;
  detect_object_client.call(detect_object_srv);
  if (detect_object_srv.response.success == false) {
    success = false;
    return (true);
  }

  ROS_INFO("detect success");
  int object_size = detect_object_srv.response.detected_object_array.objects.size();
  for (int j = 0; j < object_size; j++) {
    std::string object_name = detect_object_srv.response.detected_object_array.objects[j].name;

    if (vectorFinder(object_names, object_name) >= 0) {
      continue;
    }

    object_names.push_back(object_name);
    res.detected_product_names.push_back(object_name);
    res.detected_object_array.objects.push_back(detect_object_srv.response.detected_object_array.objects[j]);
  }
  success = true;

  setProductPlacement(detect_object_srv.response.detected_object_array.objects);
  return (true);
}

bool FcscCore::detectShelf(fcsc_msgs::DetectShelf::Request &req, fcsc_msgs::DetectShelf::Response &res)
{
  ros::Rate loop_rate(10);
  fcsc_msgs::DetectObject detect_object_srv;
  std_msgs::Bool bool_msg;

  bool detect_success = false;

  if (use_shelf_marker_detection_) {
    // マーカ認識あり
    // マーカー認識動作を実行
    bool_msg.data = true;
    for (size_t i = 0; i < 3; i++) {
      enable_shelf_detection_publisher.publish(bool_msg);
      loop_rate.sleep();
    }

    if (getCurrentEEfPose("base_footprint").pose.position.x < 0.4) {
      asyncMoveArm("fold");
    }

    switch (req.pattern) {
      case 0: {
        // 正面
        asyncMoveArm("search_shelf_A_board_2");
        waitForExecute();
        ros::WallDuration(3).sleep();

        detect_object_srv.request.visualize = true;
        detect_shelf_client.call(detect_object_srv);

        double wrist_joint_value = move_group.getCurrentJointValues()[4];

        for (size_t i = 0; i < detect_object_srv.response.detected_object_array.objects.size(); i++) {
          std::cerr << detect_object_srv.response.detected_object_array.objects[i].name << '\n';
          if (detect_object_srv.response.detected_object_array.objects[i].name == "shelfA") {
            detect_success = true;
            break;
          }
        }

        for (size_t i = 0; i < 2 && !detect_success; i++) {
          switch (i) {
            case 0: move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value - M_PI / 12.0); break;
            case 1: move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value + M_PI / 12.0); break;
          }
          planAndExecuteArm();
          ros::WallDuration(3).sleep();
          detect_object_srv.request.visualize = true;
          detect_shelf_client.call(detect_object_srv);
          for (size_t i = 0; i < detect_object_srv.response.detected_object_array.objects.size(); i++) {
            std::cerr << detect_object_srv.response.detected_object_array.objects[i].name << '\n';
            if (detect_object_srv.response.detected_object_array.objects[i].name == "shelfA") {
              detect_success = true;
              break;
            }
          }
          if (detect_success) {
            break;
          }
        }
        break;
      }
      case 1: {
        // 正面
        asyncMoveArm("search_shelf_B_board_2");
        waitForExecute();
        // asyncMoveArm("search_shelf_2");
        // moveArm("mayoko5");
        ros::WallDuration(3).sleep();

        detect_object_srv.request.visualize = true;
        detect_shelf_client.call(detect_object_srv);

        double wrist_joint_value = move_group.getCurrentJointValues()[4];

        for (size_t i = 0; i < detect_object_srv.response.detected_object_array.objects.size(); i++) {
          std::cerr << detect_object_srv.response.detected_object_array.objects[i].name << '\n';
          if (detect_object_srv.response.detected_object_array.objects[i].name == "shelfB") {
            detect_success = true;
            break;
          }
        }

        for (size_t i = 0; i < 2 && !detect_success; i++) {
          switch (i) {
            case 0: move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value - M_PI / 8.0); break;
            case 1: move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value + M_PI / 8.0); break;
          }
          planAndExecuteArm();
          ros::WallDuration(3).sleep();
          detect_object_srv.request.visualize = true;
          detect_shelf_client.call(detect_object_srv);
          for (size_t i = 0; i < detect_object_srv.response.detected_object_array.objects.size(); i++) {
            std::cerr << detect_object_srv.response.detected_object_array.objects[i].name << '\n';
            if (detect_object_srv.response.detected_object_array.objects[i].name == "shelfB") {
              detect_success = true;
              break;
            }
          }
          if (detect_success) {
            break;
          }
        }
        break;
      }
    }

    moveArm(stock_pose_name);

    bool_msg.data = false;
    for (size_t i = 0; i < 3; i++) {
      enable_shelf_detection_publisher.publish(bool_msg);
      loop_rate.sleep();
    }

    if (!detect_success) {
      ROS_ERROR("shelf detection error");
      return (false);
    }

    ROS_WARN("shelf detect success");
    for (int i = 0; i < detect_object_srv.response.detected_object_array.objects.size(); i++) {
      fcsc_msgs::RecognizedObject obj = detect_object_srv.response.detected_object_array.objects[i];
      ROS_WARN("%s", obj.name.c_str());
      res.detected_shelf_names.push_back(obj.name);
    }
  } else {
    // マーカー認識なし
    // AI搬送車のvisual slamによる絶対座標系を利用
    geometry_msgs::PoseStamped shelf_pose;
    shelf_pose.header.frame_id = "map";

    // shelfA
    shelf_pose.pose.position.x = 1.7 + 0.435 / 2.0;
    shelf_pose.pose.position.y = 1.55 - (1.92 - 0.95 / 2.0);
    shelf_pose.pose.orientation.w = 1.0;
    fcsc_visual_tools.spawnShelf("shelfA", shelf_pose);

    // shelfB
    shelf_pose.pose.position.y = 1.55 - 0.95 / 2.0;
    fcsc_visual_tools.spawnShelf("shelfB", shelf_pose);
  }


  return (true);
}

bool FcscCore::detectSandwich(fcsc_msgs::DetectSandwich::Request &req, fcsc_msgs::DetectSandwich::Response &res)
{
  std::vector<std::string>                object_names;
  std::vector<geometry_msgs::PoseStamped> search_target_pose_v;
  fcsc_msgs::DetectObject                 detect_object_srv;
  fcsc_msgs::EstimateSandwichPosition     estimate_sandwich_pos_srv;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::Rate loop_rate(10);

  waitForExecute();
  robot_hand.move(100);
  visual_tools->deleteAllMarkers();

  // デモの場合は過去のサンドイッチの環境情報をparamから取得する
  if (req.demo == true) {
    double offset = 0.05;
    random_sandwich_generator.setRegionFrame(shelfB_board_name);
    random_sandwich_generator.setRegionRange(0+offset, shelf_depth-offset, 0+offset, shelf_width-offset);
    random_sandwich_generator.generateSandwich(8, res.sandwiches);
    for (size_t i = 0; i < res.sandwiches.size(); i++) {
      for (int j = 0; j < scrap_sandwich_ids.size(); j++) {
        if (res.sandwiches[i].name == ("sandwich_"+scrap_sandwich_ids[j])) {
          res.sandwiches[i].scrap = true;
          break;
        }
      }
      fcsc_visual_tools.spawnProduct(res.sandwiches[i].name, res.sandwiches[i].pose);
    }
    // detect_object_srv.request.damy = true;
  } else {
    for (size_t i = 0; i < 2; i++) {
      std_msgs::Bool msg;
      msg.data = true;
      enable_object_detection_publisher.publish(msg);
      loop_rate.sleep();
    }
  }

  // 初期姿勢へ移動
  if (getCurrentEEfPose("base_footprint").pose.position.x < 0.4) {
    asyncMoveArm("fold");
    asyncMoveArm(recover_pose_name);
  }

  geometry_msgs::PoseStamped ps_stamped;
  geometry_msgs::PoseStamped target_pose;

  double camera_y_offset = 0.0;

  switch (req.pattern) {
    case 0: {
      // 左〜中心〜右　奥
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = 0.2;
      ps_stamped.pose.position.y = shelf_width / 2.0 + camera_y_offset;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/3.0, 0);
      target_pose = getPoseStamped(ps_stamped, "camera_optical_center_line_link", "gripper_link", listener);
      target_pose.pose.position.z = 0.15;
      transformPose("base_footprint", target_pose, ps_stamped, listener);
      target_pose = ps_stamped;
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);

      // サンドイッチを検出するアームの目標手先位置を設定
      // 左〜中心〜右　手前
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = -0.1;
      ps_stamped.pose.position.y = shelf_width / 2.0 + 0.02 + camera_y_offset;
      ps_stamped.pose.position.z = 0.2;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/3.0, 0);
      transformPose("base_footprint", target_pose, ps_stamped, listener);
      target_pose = ps_stamped;
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);
      break;
    }
    case 1: {
      // サンドイッチを検出するアームの目標手先位置を設定
      // 左〜中心〜右　手前
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = 0.0;
      ps_stamped.pose.position.y = shelf_width / 2.0 + camera_y_offset;
      ps_stamped.pose.position.z = 0.3;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, 0);
      transformPose("base_footprint", ps_stamped, target_pose, listener);
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);
      break;
    }
    case 2: {
      // サンドイッチを検出するアームの目標手先位置を設定
      // 左〜中心〜右　手前
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = 0.0;
      ps_stamped.pose.position.y = shelf_width / 2.0 + camera_y_offset;
      ps_stamped.pose.position.z = 0.2;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/3.0, 0);
      transformPose("base_footprint", ps_stamped, target_pose, listener);
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);
      break;
    }
    case 3: {
      // 左〜中心〜右　奥
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = 0.2;
      ps_stamped.pose.position.y = shelf_width / 2.0 + camera_y_offset;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, 0);
      target_pose = getPoseStamped(ps_stamped, "camera_optical_center_line_link", "gripper_link", listener);
      target_pose.pose.position.z = 0.2;
      transformPose("base_footprint", target_pose, ps_stamped, listener);
      target_pose = ps_stamped;
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);
      break;
    }
    case 4: {
      // 左〜中心〜右　奥
      ps_stamped.header.frame_id = shelfB_board_name;
      ps_stamped.pose.position.x = 0.15;
      ps_stamped.pose.position.y = shelf_width / 2.0 + camera_y_offset;
      ps_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/3.0, 0);
      target_pose = getPoseStamped(ps_stamped, "camera_optical_center_line_link", "gripper_link", listener);
      target_pose.pose.position.z = 0.15;
      transformPose("base_footprint", target_pose, ps_stamped, listener);
      target_pose = ps_stamped;
      target_pose.pose.position.y = 0;
      search_target_pose_v.push_back(target_pose);
      break;
    }
   }


  detect_object_srv.request.detect = true;
  detect_object_srv.request.visualize = false;

  move_group.setMaxVelocityScalingFactor(max_velocity);

  // 検出動作を実行
  for (int i = 0; i < search_target_pose_v.size(); i++) {
    double wrist_joint_value;
    for (size_t j = 0; j < 3; j++) {
      ROS_INFO("search [%d]", i);

      bool success;

      if (req.demo) {
        success = true;
      } else {
        switch (j) {
          case 0:
            success = moveArm(search_target_pose_v[i]);
            wrist_joint_value = move_group.getCurrentJointValues()[4];
            break;
          case 1:
            move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value + M_PI / 6);
            success = (bool)planAndExecuteArm();
            break;
          case 2:
            move_group.setJointValueTarget("wrist_2_joint", wrist_joint_value - M_PI / 6);
            success = (bool)planAndExecuteArm();
            break;
        }
      }

      if (!success) {
        break;
      }

      if (!req.demo) {
        ros::Duration(3.0).sleep();
      }

      detect_object_client.call(detect_object_srv);
      if (detect_object_srv.response.success == false) {
        ROS_INFO("Not detected sandwich");
        continue;
      }

      // 検出したサンドイッチをresponseに格納
      ROS_INFO("Detected sandwich");
      int object_size = detect_object_srv.response.detected_object_array.objects.size();
      for (int j = 0; j < object_size; j++) {
        std::string object_name = detect_object_srv.response.detected_object_array.objects[j].name;

        // サンドイッチ以外は無視
        if (object_name.find("sandwich") == std::string::npos) {
          continue;
        }

        // 検出済みのサンドイッチは無視
        if (vectorFinder(object_names, object_name) >= 0) {
          continue;
        }

        object_names.push_back(object_name);
        res.sandwiches.push_back(detect_object_srv.response.detected_object_array.objects[j]);
      }
    }
  }

  for (size_t i = 0; i < 2; i++) {
    std_msgs::Bool msg;
    msg.data = false;
    enable_object_detection_publisher.publish(msg);
    loop_rate.sleep();
  }

  move_group.setMaxVelocityScalingFactor(normal_velocity);
  // asyncMoveArm(recover_pose_name);

  if (res.sandwiches.size() > 0) {
    ROS_INFO("visualize objects");
    detect_object_srv.request.detect = false;
    detect_object_srv.request.visualize = true;
    detect_object_client.call(detect_object_srv);
  } else {
    ROS_ERROR("Not detected sandwich");
  }


  return (true);
}
