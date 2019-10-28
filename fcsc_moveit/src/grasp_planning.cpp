//把持計画用
#include "fcsc_moveit/grasp_planning.h"

GraspPlanner::GraspPlanner()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  nh.param<bool>("use_simulator", use_simulator, false);
}

void GraspPlanner::generateGraspPose(string name, std::vector<fcsc_msgs::Grasp> *grasps)
{
  //onigiri
  if (name.find("onigiri") != std::string::npos) {
    ROS_INFO("plan grasps Onigiri");
    generateGraspPoseForOnigiri(name, grasps);
  } else if (name.find("drink") != std::string::npos) {
    ROS_INFO("plan grasps Drink");
    generateGraspPoseForDrink(name, grasps);
  } else if (name.find("bento") != std::string::npos) {
    ROS_INFO("plan grasps Bento");
    generateGraspPoseForBento(name, grasps);
  } else if (name.find("sandwich") != std::string::npos) {
    ROS_INFO("plan grasps Sandwich");
    generateGraspPoseForSandwich(name, grasps);
  }
}

void GraspPlanner::generateGraspPoseForOnigiri(string name, std::vector<fcsc_msgs::Grasp> *grasps)
{
  grasps->clear();
  grasps->resize(0);

  fcsc_msgs::Grasp grasp;

  grasp.grasp_pose.header.frame_id = name;
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(1);
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(1);
  grasp.grasp_posture.points[0].effort.resize(1);
  grasp.grasp_posture.points[0].effort[0] = 30;

  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.vector.x = 0;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 1;
  grasp.post_grasp_retreat.min_distance = 0.2;
  grasp.post_grasp_retreat.desired_distance = 0.2;

  //姿勢１（倒れている状態）
  // grasp.id = "onigiri_center_1";
  // grasp.pre_grasp_posture.points[0].positions[0] = 13.3+20;
  // grasp.grasp_posture.points[0].positions[0] = 13.3;//計測した値

  // grasp.pre_grasp_posture.points[0].positions[0] = 11.3+20;
  // grasp.grasp_posture.points[0].positions[0] = 11.3;
  // grasp.grasp_pose.pose.position.x = 0.0;
  // grasp.grasp_pose.pose.position.y = 0.0;
  // grasp.grasp_pose.pose.position.z = 0.0;
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0, M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_1_reverse_1";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0, -1.0 * M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_1_reverse_2";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.0 * M_PI/2.0, 0, -1.0 * M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_1_reverse_3";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.0 * M_PI/2.0, 0, M_PI/2.0);
  // grasps->push_back(grasp);

  //姿勢２（倒れている状態）
  // grasp.id = "onigiri_center_2";
  // grasp.pre_grasp_posture.points[0].positions[0] = 10.0+20;
  // grasp.grasp_posture.points[0].positions[0] = 10.0;
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_2_reverse_1";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -1.0 * M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_2_reverse_2";
  // grasp.grasp_pose.pose.orientationForOni = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, -1.0 * M_PI/2.0);
  // grasps->push_back(grasp);
  // grasp.id = "onigiri_center_2_reverse_3";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, M_PI/2.0);
  // grasps->push_back(grasp);

  //姿勢３（立っている状態）
  // 真上から掴む
  grasp.id = "onigiri_center_3_1";
  grasp.grasp_posture.points[0].positions[0] = 8;
  // grasp.grasp_posture.points[0].positions[0] = 11.3;
  grasp.pre_grasp_posture.points[0].positions[0] = grasp.grasp_posture.points[0].positions[0] + 20;
  grasp.grasp_pose.pose.position.x = 0;
  // grasp.grasp_pose.pose.position.x = -finger_interval / 2.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0;
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, 0);

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  grasp.pre_grasp_approach.direction.vector.x = 0;
  grasp.pre_grasp_approach.direction.vector.y = 0;
  grasp.pre_grasp_approach.direction.vector.z = -1;
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  // grasps->push_back(grasp);//2018/08/07

  // grasp.id = "onigiri_center_3_2";
  // grasp.grasp_pose.pose.position.x = finger_interval / 2.0;
  // grasps->push_back(grasp);

  // 真上から掴む（手首180度回転）
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, M_PI);
  grasp.id = "onigiri_center_4_1";
  // grasps->push_back(grasp);//2018/08/07

  /************************* FCSC2018 *************************/
  // サンドイッチの上部のフィルム部分を掴む姿勢 手先を垂直から斜めに傾ける姿勢
  grasp.id = "onigiri_edge_tilt_1";
  grasp.pre_grasp_posture.points[0].positions[0] = 40;
  // grasp.pre_grasp_posture.points[0].positions[0] = 20;
  grasp.grasp_posture.points[0].positions[0] = 0;

  grasp.grasp_pose.pose.position.x = 0.0;
  grasp.grasp_pose.pose.position.y = 0.02;
  grasp.grasp_pose.pose.position.z = onigiri_height / 2.0 + finger_interval / 2.0;
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 6, M_PI /2);

  tf::Quaternion rotation = tf::createQuaternionFromRPY(0, M_PI / 6, M_PI / 2);
  tf::Vector3 vector(1, 0, 0);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasps->push_back(grasp);

  grasp.id = "onigiri_edge_tilt_2";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 4, M_PI /2);

  rotation = tf::createQuaternionFromRPY(0, M_PI / 4, M_PI / 2);
  rotated_vector = tf::quatRotate(rotation, vector);

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasps->push_back(grasp);

  grasp.id = "onigiri_edge_top";
  grasp.pre_grasp_posture.points[0].positions[0] = 20;
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].effort[0] = 100;
  grasp.ezgripper_control = fcsc_msgs::Grasp::BOTTOM;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = onigiri_height / 2.0 + finger_interval / 2.0 + finger_width;
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI /2);

  rotation = tf::createQuaternionFromRPY(0, 0, M_PI / 2);
  rotated_vector = tf::quatRotate(rotation, vector);

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasps->push_back(grasp);

  /************************************************************/

  // grasp.id = "onigiri_center_4_2";
  // grasp.grasp_pose.pose.position.x = finger_interval / 2.0;
  // grasps->push_back(grasp);

  // grasp.id = "onigiri_center_3_reverse_1";
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, M_PI);
  // grasps->push_back(grasp);

  //姿勢４（フィルムの端）
  // double onigiri_z_offset = 0.015;
  // grasp.id = "onigiri_edge_1";
  // grasp.pre_grasp_posture.points[0].positions[0] = 20;
  // grasp.grasp_posture.points[0].positions[0] = 0;
  // grasp.grasp_pose.pose.position.x = 0.0;
  // grasp.grasp_pose.pose.position.y = -finger_interval / 2.0;
  // grasp.grasp_pose.pose.position.z = onigiri_height / 2.0 + onigiri_z_offset;
  // // grasp.grasp_pose.pose.position.z = onigiri_height * (2.0 / 3.0);
  // // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI / 2.0);
  // grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2.0, M_PI / 2.0, 0);
  // // grasps->push_back(grasp);
  // grasp.id = "onigiri_edge_2";
  // grasp.grasp_pose.pose.position.y = finger_interval / 2.0;
  // // grasps->push_back(grasp);
  // // grasp.id = "onigiri_edge_1_reverse";
  // // grasp.grasp_pose.pose.orientation = tf:createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI/2.0);
  // // grasps->push_back(grasp);
  // // grasp.id = "onigiri_edge_2";
  // // grasp.grasp_pose.pose.orientation = tf:createQuaternionMsgFromRollPitchYaw(M_PI, 0, M_PI/2.0);
  // // grasps->push_back(grasp);
  // // grasp.id = "onigiri_edge_2_reverse";
  // // grasp.grasp_pose.pose.orientation = tf:createQuaternionMsgFromRollPitchYaw(M_PI, 0, -M_PI/2.0);
  // // grasps->push_back(grasp);
}

void GraspPlanner::generateGraspPoseForDrink(string name, std::vector<fcsc_msgs::Grasp> *grasps)
{
  grasps->clear();
  grasps->resize(0);

  fcsc_msgs::Grasp grasp;
  grasp.grasp_pose.header.frame_id = name;
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(1);
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(1);
  // grasp.grasp_posture.points[0].positions[0] = 20;
  // grasp.grasp_posture.points[0].positions[0] = 23.3;
  // grasp.pre_grasp_posture.points[0].positions[0] = grasp.grasp_posture.points[0].positions[0]+20;

  // FCSC 2017/12/20
  // grasp.grasp_posture.points[0].positions[0] = 18;
  // grasp.pre_grasp_posture.points[0].positions[0] = grasp.grasp_posture.points[0].positions[0] + 22;
  grasp.grasp_posture.points[0].positions[0] = 10;
  grasp.pre_grasp_posture.points[0].positions[0] = 40;

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  grasp.pre_grasp_approach.direction.vector.x = 0;
  grasp.pre_grasp_approach.direction.vector.y = 0;
  grasp.pre_grasp_approach.direction.vector.z = -1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.min_distance = 0.1;

  grasp.post_grasp_retreat.direction.header.frame_id = "gripper_link";
  grasp.post_grasp_retreat.direction.vector.x = -1;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 0;
  grasp.post_grasp_retreat.desired_distance = 0.13;
  grasp.post_grasp_retreat.min_distance = 0.13;

  grasp.grasp_posture.points[0].effort.resize(1);
  grasp.grasp_posture.points[0].effort[0] = 50;
  for (int i = 0; i < particle_size; i++) {
    ostringstream oss;
    oss << i+1;
    grasp.id = "drink_up_" + oss.str();
    grasp.grasp_pose.pose.position.x = 0.0;
    grasp.grasp_pose.pose.position.y = 0.0;
    grasp.grasp_pose.pose.position.z = -0.01;
    // grasp.grasp_pose.pose.position.z = 0.03;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2.0, (double)i * 2.0 * M_PI / (double)particle_size);
    grasps->push_back(grasp);
  }

  // for (int i = 0; i < particle_size; i++) {
  //   ostringstream oss;
  //   oss << i + 1;
  //   grasp.id = "drink_side_" + oss.str();
  //   grasp.grasp_pose.pose.position.x = 0.0;
  //   grasp.grasp_pose.pose.position.y = 0.0;
  //   grasp.grasp_pose.pose.position.z = 0.05;
  //   grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 4, (double)i * 2.0 * M_PI / (double)particle_size);
  //   grasps->push_back(grasp);
  // }
}

void GraspPlanner::generateGraspPoseForBento(string name, std::vector<fcsc_msgs::Grasp> *grasps)
{
  // std::vector<fcsc_msgs::Grasp> grasps;

  grasps->clear();
  grasps->resize(0);

  //真上から掴む
  fcsc_msgs::Grasp grasp;
  grasp.id = "bento_top";
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(1);
  grasp.pre_grasp_posture.points[0].positions[0] = 100;
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(1);
  // close
  grasp.grasp_posture.points[0].positions[0] = 20;
  // grasp.grasp_posture.points[0].positions[0] = 0;
  // open
  // grasp.grasp_posture.points[0].positions[0] = 100;
  grasp.grasp_posture.points[0].effort.resize(1);
  grasp.grasp_posture.points[0].effort[0] = 100;

  grasp.pre_grasp_approach.direction.header.frame_id = name;
  grasp.pre_grasp_approach.direction.vector.x = 0;
  grasp.pre_grasp_approach.direction.vector.y = 0;
  grasp.pre_grasp_approach.direction.vector.z = -1;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.min_distance = 0.1;

  grasp.post_grasp_retreat.direction.header.frame_id = "gripper_link";
  grasp.post_grasp_retreat.direction.vector.x = -1;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 0;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.min_distance = 0.1;

  // if (use_simulator == false) {
  //   // grasp.grasp_posture.points[0].positions[0] = 0;
  //   // grasp.grasp_posture.points[0].positions[0] = 34.7;
  //   grasp.grasp_posture.points[0].positions[0] = 20;
  // } else {
  //   grasp.grasp_posture.points[0].positions[0] = 60;
  //   // grasp.grasp_posture.points[0].positions[0] = 100;
  // }

  // grasp.grasp_posture.points[0].positions[0] = 36.7;//計測値
  grasp.grasp_pose.header.frame_id = name;
  grasp.grasp_pose.pose.position.x = 0.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  // grasp.grasp_pose.pose.position.z = 0.0;
  // grasp.grasp_pose.pose.position.z = 0.0255;
  // grasp.grasp_pose.pose.position.z = -0.03;//指先から手のひらまでの距離
  grasp.grasp_pose.pose.position.z = -palm_to_fingertip + bento_height / 2.0 - 0.002;//指先から手のひらまでの距離 + 弁当高さ/2 + オフセット
  // grasp.grasp_pose.pose.position.z = -palm_to_fingertip + bento_height / 2.0 + 0.005;//指先から手のひらまでの距離 + 弁当高さ/2 + オフセット
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2, M_PI/2);
  grasps->push_back(grasp);
  grasp.id = "bento_top_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2, -1.0 * M_PI/2);
  grasps->push_back(grasp);


  //長辺の端をつかむ
  // for (int i=0; i<2; i++) {
  //   double toggle;
  //   fcsc_msgs::Grasp grasp;
  //   ostringstream oss;
  //
  //   if (i == 0) {
  //     toggle = -1.0;
  //   } else {
  //     toggle = 1.0;
  //   }
  //   oss << i+1;
  //   grasp.id = "bento_long_side_" + oss.str();
  //   grasp.grasp_pose.header.frame_id = name;
  //   grasp.grasp_pose.pose.position.x = toggle * (175.0 / 2.0 - 0.01);
  //   grasp.grasp_pose.pose.position.y = 0.0;
  //   grasp.grasp_pose.pose.position.z = 0.0;
  //   grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, i * M_PI);
  //   grasps->push_back(grasp);
  // }

  //短辺の端をつかむ
/*  for (int i=0; i<2; i++) {
    double toggle;
    fcsc_msgs::Grasp grasp;
    ostringstream oss;

    if (i == 0) {
      toggle = 1.0;
    } else {
      toggle = -1.0;
    }
    oss << i+1;
    grasp.id = "bento_short_side_" + oss.str();
    grasp.grasp_pose.header.frame_id = name;
    grasp.grasp_pose.pose.position.x = 0.0;
    grasp.grasp_pose.pose.position.y = toggle * (250.0 / 2.0 - 0.01);
    grasp.grasp_pose.pose.position.x = 0.0;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -1.0 * toggle * M_PI / 2.0);
    grasps->push_back(grasp);
  }*/
}

void GraspPlanner::generateGraspPoseForSandwich(string name, std::vector<fcsc_msgs::Grasp> *grasps)
{
  fcsc_msgs::Grasp grasp;
  tf::Vector3 vector(1, 0, 0);
  tf::Quaternion rotation;
  tf::Vector3 rotated_vector;
  double angle = atan(0.09/0.1);

  grasps->clear();

  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(1);
  grasp.pre_grasp_posture.points[0].positions[0] = 60;
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(1);
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].effort.resize(1);
  grasp.grasp_posture.points[0].effort[0] = 50;
  grasp.grasp_pose.header.frame_id = name;
  // grasp.grasp_pose.pose.position.x = 0.07 / 2.0;
  // grasp.grasp_pose.pose.position.y = 0.0;
  // grasp.grasp_pose.pose.position.z = 2.0 * 0.14 / 3.0;
  grasp.grasp_pose.pose.position.x = 0.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.1;

  grasp.pre_grasp_approach.desired_distance = 0.05;
  grasp.pre_grasp_approach.min_distance = 0.05;

  grasp.post_grasp_retreat.desired_distance = 0.05;
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.direction.header.frame_id = "gripper_link";
  grasp.post_grasp_retreat.direction.vector.x = -1;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 0;

  // フィルム掴む
  // サンドイッチの幅方向（y軸）を回転軸として把持姿勢を作成
  // フィルムが曲がる方向
  for (size_t i = 0; i < 5; i++) {
    stringstream ss;
    rotation = tf::createQuaternionFromRPY(M_PI / 2.0, (i+1) * M_PI/6.0, 0);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    ss << "sandwich_film_revolute" << i + 1;
    if (i == 2) {
      ss << "_top_";
    } else {
      ss << "_tilt_";
    }
    ss << i + 1;

    grasp.id = ss.str();
    grasp.grasp_pose.pose.position.x = 0.03 - (double)i * 0.015;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2.0, (i+1) * M_PI/6.0, 0);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2.0, (i+1) * M_PI/6.0, 0);
    grasps->push_back(grasp);
  }

  // フィルム掴む
  // サンドイッチの奥行き方向（x軸）を回転軸として把持姿勢を作成
  // フィルムが曲がらない方向
  grasp.pre_grasp_posture.points[0].positions[0] = 60;
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].effort[0] = 50;
  grasp.grasp_pose.pose.position.x = 0.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.12;
  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.vector.x = 0;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 1;
  for (size_t i = 0; i < 11; i++) {
    stringstream ss;
    rotation = tf::createQuaternionFromRPY(0, -M_PI / 3.0 + i * M_PI / 6.0, M_PI / 2.0);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    if (i == 5) {
      // topのときは飛ばす
      continue;
    }

    ss << "sandwich_film_tilt" << i + 1;
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI / 3.0 + i * M_PI / 6.0, M_PI / 2.0);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI / 3.0 + i * M_PI / 6.0, M_PI / 2.0);
    grasps->push_back(grasp);
  }

  // 倒れたサンドイッチの底面・パッケージ面の角部分を掴む
  grasp.pre_grasp_posture.points[0].positions[0] = 50;
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].effort[0] = 70;
  grasp.grasp_pose.pose.position.x = -0.025;
  // grasp.grasp_pose.pose.position.x = -0.045;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.0;
  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.vector.x = 0;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 1;
  for (size_t i = 0; i < 7; i++) {
    stringstream ss;
    rotation = tf::createQuaternionFromRPY(M_PI / 2.0, 0, -M_PI / 2.0 + i * M_PI / 6.0);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    ss << "sandwich_edge_" << i + 1;
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2.0, 0, -M_PI / 2.0 + i * M_PI / 6.0);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2.0, 0, -M_PI / 2.0 + i * M_PI / 6.0);
    grasps->push_back(grasp);
  }

  // 倒れたサンドイッチの背面・底面の角を掴む
  grasp.pre_grasp_posture.points[0].positions[0] = 50;
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].effort[0] = 50;
  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.vector.x = 0;
  grasp.post_grasp_retreat.direction.vector.y = 0;
  grasp.post_grasp_retreat.direction.vector.z = 1;

  tf::Transform tf_from_origin_to_slope;
  tf::Transform tf_from_slope_to_gripper;
  tf::Transform tf_from_origin_to_gripper;
  tf_from_origin_to_slope.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf_from_origin_to_slope.setRotation(tf::createQuaternionFromRPY(0, angle, 0));
  tf_from_slope_to_gripper.setOrigin(tf::Vector3(-0.06, 0, 0));

  for (size_t i = 0; i < 5; i++) {
    stringstream ss;
    tf_from_slope_to_gripper.setRotation(tf::createQuaternionFromRPY(M_PI / 2.0, 0, 4.0 * M_PI / 6.0 + i * M_PI / 6.0));
    tf_from_origin_to_gripper = tf_from_origin_to_slope * tf_from_slope_to_gripper;

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(tf::quatRotate(tf_from_origin_to_gripper.getRotation(), vector), grasp.pre_grasp_approach.direction.vector);

    for (size_t j = 0; j < 1; j++) {
      double y = 0;
      // if (j == 0)
      //   y = finger_interval / 2.0;
      // else
      //   y = -finger_interval / 2.0;

      ss << "sandwich_edge2_" << i + 1;
      grasp.id = ss.str();
      tf_from_slope_to_gripper.setRotation(tf::createQuaternionFromRPY(M_PI / 2.0, 0, 4.0 * M_PI / 6.0 + i * M_PI / 6.0));
      tf_from_origin_to_gripper = tf_from_origin_to_slope * tf_from_slope_to_gripper;
      grasp.grasp_pose.pose.position.x = tf_from_origin_to_gripper.getOrigin().getX();
      grasp.grasp_pose.pose.position.y = y;
      grasp.grasp_pose.pose.position.z = tf_from_origin_to_gripper.getOrigin().getZ();
      tf::quaternionTFToMsg(tf_from_origin_to_gripper.getRotation(), grasp.grasp_pose.pose.orientation);
      // grasps->push_back(grasp);

      ss << "_reverse";
      grasp.id = ss.str();
      tf_from_slope_to_gripper.setRotation(tf::createQuaternionFromRPY(-M_PI / 2.0, 0, 4.0 * M_PI / 6.0 + i * M_PI / 6.0));
      tf_from_origin_to_gripper = tf_from_origin_to_slope * tf_from_slope_to_gripper;
      grasp.grasp_pose.pose.position.x = tf_from_origin_to_gripper.getOrigin().getX();
      grasp.grasp_pose.pose.position.y = y;
      grasp.grasp_pose.pose.position.z = tf_from_origin_to_gripper.getOrigin().getZ();
      tf::quaternionTFToMsg(tf_from_origin_to_gripper.getRotation(), grasp.grasp_pose.pose.orientation);
      // grasps->push_back(grasp);
    }

  }

  // サンドイッチの前面側から本体を掴む
  // 背面接地と通常用
  grasp.pre_grasp_posture.points[0].positions[0] = 70;
  grasp.grasp_posture.points[0].positions[0] = 10;
  grasp.grasp_posture.points[0].effort[0] = 70;
  grasp.grasp_pose.pose.position.x = -0.01;
  // grasp.grasp_pose.pose.position.x = -0.09 / 4.0 - 0.01;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.1 / 2.0;
  // grasp.grasp_pose.pose.position.z = finger_interval / 2.0 + finger_width / 2.0;
  for (size_t i = 0; i < 4; i++) {
    stringstream ss;
    rotation = tf::createQuaternionFromRPY(0, i * M_PI / 6.0, 0);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    ss << "sandwich_front_body_" << i + 1;
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, i * M_PI / 6.0, 0);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, i * M_PI / 6.0, 0);
    grasps->push_back(grasp);
  }

  // サンドイッチの背面側から本体を掴む
  // 前面接地と通常用
  grasp.pre_grasp_posture.points[0].positions[0] = 70;
  grasp.grasp_posture.points[0].positions[0] = 10;
  grasp.grasp_posture.points[0].effort[0] = 70;
  grasp.grasp_pose.pose.position.x = -0.09 / 2.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.1 / 2.0;
  for (size_t i = 0; i < 5; i++) {
    stringstream ss;
    rotation = tf::createQuaternionFromRPY(0, -M_PI / 3.0 + i * M_PI / 6.0, M_PI);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    ss << "sandwich_back_body_" << i + 1;
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI / 3.0 + i * M_PI / 6.0, M_PI);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI / 3.0 + i * M_PI / 6.0, M_PI);
    grasps->push_back(grasp);
  }

  // 背面を接地して倒れているサンドイッチの本体を掴む 斜めからアプローチ
  grasp.id = "sandwich_back_tilt";
  grasp.pre_grasp_posture.points[0].positions[0] = 70;
  grasp.grasp_posture.points[0].positions[0] = 10;
  grasp.grasp_pose.pose.position.x = -0.09 / 4.0 - 0.01;
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = finger_interval / 2.0 + finger_width / 2.0;
  grasp.pre_grasp_approach.direction.vector.x = 1;
  grasp.pre_grasp_approach.direction.vector.y = 0;
  grasp.pre_grasp_approach.direction.vector.z = 0;
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI / 4.0, 0);
  rotation = tf::createQuaternionFromRPY(0, -M_PI / 4.0, 0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasps->push_back(grasp);

  // 背面を接地して倒れているサンドイッチの本体を掴む 斜めからアプローチ 手首反転
  grasp.id = "sandwich_back_tilt_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI / 4.0, 0);
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む
  grasp.id = "sandwich_front";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI, 0);
  rotation = tf::createQuaternionFromRPY(0, -M_PI, 0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.grasp_pose.pose.position.x = -0.09 / 3.0;
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = 0;
  // grasp.grasp_pose.pose.position.z = finger_interval / 2.0;
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む 手首反転
  grasp.id = "sandwich_front_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI, 0);
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む
  grasp.id = "sandwich_front_2";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI / 2, 0);
  rotation = tf::createQuaternionFromRPY(M_PI, -M_PI / 2, 0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.grasp_pose.pose.position.x = 0.0;
  // grasp.grasp_pose.pose.position.x = -finger_interval / 2.0;
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = 0.02;
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む 手首反転
  grasp.id = "sandwich_front_2_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI / 2, 0);
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む
  grasp.id = "sandwich_front_3";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -3 * M_PI / 4, 0);
  rotation = tf::createQuaternionFromRPY(M_PI, -3 * M_PI / 4, 0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasp.grasp_pose.pose.position.x = -0.04;
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = 0.04;
  grasps->push_back(grasp);

  // 前面を接地して倒れているサンドイッチの本体を掴む 手首反転
  grasp.id = "sandwich_front_3_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -3 * M_PI / 4, 0);
  grasps->push_back(grasp);


  // 側面を接地して倒れているサンドイッチの本体を掴む
  grasp.grasp_pose.pose.position.x = -0.09 / 3.0;
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = 0.1 / 2.0;
  for (size_t i = 0; i < 2; i++) {
    int plus_minus;
    stringstream ss;
    switch (i) {
      case 0:
        ss << "sandwich_right_side";
        plus_minus = 1;
        break;
      case 1:
        ss << "sandwich_left_side";
        plus_minus = -1;
        break;
    }
    rotation = tf::createQuaternionFromRPY(0, 0, plus_minus * M_PI / 2.0);
    rotated_vector = tf::quatRotate(rotation, vector);

    grasp.pre_grasp_approach.direction.header.frame_id = name;
    tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);

    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, plus_minus * M_PI / 2.0);
    grasps->push_back(grasp);

    ss << "_reverse";
    grasp.id = ss.str();
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, plus_minus * M_PI / 2.0);
    grasps->push_back(grasp);
  }

  // 右側面を接地して倒れているサンドイッチの本体を掴む
  grasp.id = "sandwich_right_side_2";
  grasp.grasp_pose.pose.position.x = -(0.09 / 2.0) * cos(M_PI / 2.0 - angle) * cos(M_PI / 2.0 - angle);
  grasp.grasp_pose.pose.position.y = 0;
  grasp.grasp_pose.pose.position.z = (0.09 / 2.0) * cos(M_PI / 2.0 - angle) * sin (M_PI / 2.0 - angle);
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle, 0, M_PI / 2.0);
  rotation = tf::createQuaternionFromRPY(angle, 0, M_PI / 2.0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasps->push_back(grasp);

  // 右側面を接地して倒れているサンドイッチの本体を掴む 手首反転
  grasp.id = "sandwich_right_side_2_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle+M_PI, 0, M_PI / 2.0);
  grasps->push_back(grasp);

  // 左側面を接地して倒れているサンドイッチの本体を掴む
  grasp.id = "sandwich_left_side_2";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-angle, 0, -M_PI / 2.0);
  rotation = tf::createQuaternionFromRPY(angle, 0, -M_PI / 2.0);
  rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, grasp.pre_grasp_approach.direction.vector);
  grasps->push_back(grasp);

  // 左側面を接地して倒れているサンドイッチの本体を掴む 手首反転
  grasp.id = "sandwich_left_side_2_reverse";
  grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-angle-M_PI, 0, -M_PI / 2.0);
  grasps->push_back(grasp);

  return;
}

void GraspPlanner::generateLocationPose(std::string object_name, std::string plane_frame_id,  std::vector<moveit_msgs::PlaceLocation> *locations)
{
  //onigiri
  if (object_name.find("onigiri") != std::string::npos) {
    ROS_INFO("plan grasps Onigiri");
    generateLocationPoseForOnigiri(plane_frame_id, locations);
  } else if (object_name.find("drink") != std::string::npos) {
    ROS_INFO("plan locations Drink");
    generateLocationPoseForDrink(plane_frame_id, locations);
  } else if (object_name.find("bento") != std::string::npos) {
    ROS_INFO("plan locations Bento");
    generateLocationPoseForBento(plane_frame_id, locations);
  } else if (object_name.find("sandwich") != std::string::npos) {
    ROS_INFO("plan locations Sandwich");
    generateLocationPoseForSandwich(plane_frame_id, locations);
  }
}

void GraspPlanner::generateLocationPoseForOnigiri(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations)
{
  moveit_msgs::PlaceLocation location;

  locations->clear();

  location.id = "standard";

  location.post_place_posture.points.resize(1);
  location.post_place_posture.points[0].positions.resize(1);
  location.post_place_posture.points[0].positions[0] = 100;
  location.post_place_posture.points[0].effort.resize(1);
  location.post_place_posture.points[0].effort[0] = 100;

  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI/2);

  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  location.pre_place_approach.direction.vector.x = 0;
  location.pre_place_approach.direction.vector.y = 0;
  location.pre_place_approach.direction.vector.z = -1;
  // location.pre_place_approach.direction.vector.z = -1;
  location.pre_place_approach.desired_distance = 0.03;
  // location.pre_place_approach.desired_distance = 0.05;
  location.pre_place_approach.min_distance = 0.03;
  // location.pre_place_approach.min_distance = 0.05;

  location.post_place_retreat.direction.header.frame_id = "gripper_link";
  location.post_place_retreat.direction.vector.x = -1;
  location.post_place_retreat.direction.vector.y = 0;
  location.post_place_retreat.direction.vector.z = 0;
  location.post_place_retreat.desired_distance = 0.03;
  // location.post_place_retreat.desired_distance = 0.05;
  location.post_place_retreat.min_distance = 0.03;
  // location.post_place_retreat.min_distance = 0.05;

  locations->push_back(location);

  location.id = "tilt";
  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/6, 0, -M_PI/2);
  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0, M_PI/6, 0);
  tf::Vector3 vector(1, 0, 0);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, location.pre_place_approach.direction.vector);
  locations->push_back(location);
}

void GraspPlanner::generateLocationPoseForDrink(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations)
{
  moveit_msgs::PlaceLocation location;

  locations->clear();

  location.id = "standard";

  location.post_place_posture.points.resize(1);
  location.post_place_posture.points[0].positions.resize(1);
  location.post_place_posture.points[0].effort.resize(1);
  // location.post_place_posture.points[0].positions[0] = 30;
  location.post_place_posture.points[0].positions[0] = 60;
  location.post_place_posture.points[0].effort[0] = 50;

  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  location.pre_place_approach.direction.vector.x = 0;
  location.pre_place_approach.direction.vector.y = 0;
  location.pre_place_approach.direction.vector.z = -1;
  location.pre_place_approach.desired_distance = 0.03;
  location.pre_place_approach.min_distance = 0.03;

  location.post_place_retreat.direction.header.frame_id = "gripper_link";
  location.post_place_retreat.direction.vector.x = -1;
  location.post_place_retreat.direction.vector.y = 0;
  location.post_place_retreat.direction.vector.z = 0;
  location.post_place_retreat.desired_distance = 0.03;
  location.post_place_retreat.min_distance = 0.03;

  locations->push_back(location);

  location.id = "tilt";
  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/6, 0);
  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0, M_PI/3, 0);
  tf::Vector3 vector(1, 0, 0);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, location.pre_place_approach.direction.vector);
  locations->push_back(location);
}

void GraspPlanner::generateLocationPoseForBento(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations)
{
  moveit_msgs::PlaceLocation location;

  locations->clear();

  location.id = "standard";

  location.post_place_posture.points.resize(1);
  location.post_place_posture.points[0].positions.resize(1);
  location.post_place_posture.points[0].positions[0] = 100;
  location.post_place_posture.points[0].effort.resize(1);
  location.post_place_posture.points[0].effort[0] = 50;

  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  location.pre_place_approach.direction.vector.x = 0;
  location.pre_place_approach.direction.vector.y = 0;
  location.pre_place_approach.direction.vector.z = -1;
  location.pre_place_approach.desired_distance = 0.035;
  location.pre_place_approach.min_distance = 0.035;

  location.post_place_retreat.direction.header.frame_id = "base_footprint";
  location.post_place_retreat.direction.vector.x = 0;
  location.post_place_retreat.direction.vector.y = 0;
  location.post_place_retreat.direction.vector.z = 1;
  location.post_place_retreat.desired_distance = 0.035;
  location.post_place_retreat.min_distance = 0.035;

  locations->push_back(location);

  location.id = "tilt";
  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/6, 0);
  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0, M_PI/3, 0);
  tf::Vector3 vector(1, 0, 0);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, location.pre_place_approach.direction.vector);
  locations->push_back(location);
}

void GraspPlanner::generateLocationPoseForSandwich(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations)
{
  moveit_msgs::PlaceLocation location;

  locations->clear();

  location.id = "standard";

  location.post_place_posture.points.resize(1);
  location.post_place_posture.points[0].positions.resize(1);
  location.post_place_posture.points[0].positions[0] = 30;
  location.post_place_posture.points[0].effort.resize(1);
  location.post_place_posture.points[0].effort[0] = 50;

  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  location.place_pose.pose.position.z = 0.02;

  location.pre_place_approach.direction.header.frame_id = "base_footprint";
  location.pre_place_approach.direction.vector.x = 0;
  location.pre_place_approach.direction.vector.y = 0;
  location.pre_place_approach.direction.vector.z = -1;
  location.pre_place_approach.desired_distance = 0.02;
  location.pre_place_approach.min_distance = 0.02;

  location.post_place_retreat.direction.header.frame_id = "gripper_link";
  location.post_place_retreat.direction.vector.x = -1;
  location.post_place_retreat.direction.vector.y = 0;
  location.post_place_retreat.direction.vector.z = 0;
  location.post_place_retreat.desired_distance = 0.02;
  location.post_place_retreat.min_distance = 0.02;

  locations->push_back(location);

  location.id = "tilt";
  location.place_pose.header.frame_id = plane_frame_id;
  location.place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/6, 0);
  location.place_pose.pose.position.z = 0.05;
  location.pre_place_approach.direction.header.frame_id = plane_frame_id;
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0, M_PI/3, 0);
  tf::Vector3 vector(1, 0, 0);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);
  tf::vector3TFToMsg(rotated_vector, location.pre_place_approach.direction.vector);
  locations->push_back(location);
}
