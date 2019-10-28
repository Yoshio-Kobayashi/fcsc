// 小林君作成のサンドイッチの向きを変えるプログラム

#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

// サンドイッチ名はreq.object.name
// マニピュレーションの成功(true)失敗(false)を res.successに入れる
bool FcscCore::changeSandwichPose(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{
  switch (req.object.state) {
    case fcsc_msgs::RecognizedObject::BOTTOM : {// 通常
      break;
    }
    case fcsc_msgs::RecognizedObject::BACK : {// 背面接地//完成OK
      // ros::AsyncSpinner spinner(1);
      // ros::NodeHandle nh;
      // tf::TransformListener			listener;
      // ros::ServiceClient				detect_object_client;
      // spinner.start();
      geometry_msgs::PoseStamped test_stamped;
      geometry_msgs::PoseStamped target_pose_stamped;
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::PoseStamped current_pose_stamped;
      geometry_msgs::PoseStamped waypoint_stamped;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group.setMaxVelocityScalingFactor(0.1);
      // moveit::planning_interface::MoveGroupInterface   move_group("manipulator");
      moveit::planning_interface::MoveGroupInterface   move_group2("endeffector");
      //XmlRpc::XmlRpcValue params;
      EZGripper	both_ezgripper("/ezgripper/both");
      // detect_object_client = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
      fcsc_msgs::DetectObject	detect_object_srv;
      detect_object_srv.request.detect = true;
      detect_object_srv.request.visualize = true;


      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = -0.025;
      target_pose_stamped.pose.position.y = 0;
      target_pose_stamped.pose.position.z = 0.025;
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(- M_PI, 0, 0);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      bool success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error1");
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error1");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error1 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success1 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success1");
      }


      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);

      both_ezgripper.goToPosition(1.2, 20);

      move_group2.move();

      //move_group.setStartStateToCurrentState();
      test_stamped= move_group.getCurrentPose();

      test_stamped.pose.position.z += 0.1;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error2");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error2");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error2 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success2 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success2");
      }

      /*move_group.setPoseReferenceFrame("base_footprint");

      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z + 0.1;
      waypoints[0].orientation = test_stamped.pose.orientation;
      success = (bool) move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), true);
      if (!success) {
      move_group.detachObject(req.object.name);
      ROS_INFO("plan error2");
      return(false);
      }
      move_group.execute(plan);*/

      //move_group.setStartStateToCurrentState();
      test_stamped= move_group.getCurrentPose();
      listener.waitForTransform("shelfB_board_2", test_stamped.header.frame_id, test_stamped.header.stamp, ros::Duration(2.0));
      ros::Rate rate(10);
      for (int i = 0; i < 10; i++) {
        listener.transformPose("shelfB_board_2", test_stamped, target_pose_stamped);
        rate.sleep();
      }

      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, M_PI/2, M_PI / 2);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error3");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error3");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error3 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success3 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success3");
      }

      /* wrist_joint_value = move_group.getCurrentJointValues()[5];

      move_group.setJointValueTarget("wrist_3_joint", wrist_joint_value + M_PI);
      move_group.plan(plan);
      move_group.execute(plan);

      test_stamped= move_group.getCurrentPose();*/

      test_stamped= move_group.getCurrentPose();

      test_stamped.pose.position.z -= 0.08;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error4");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error4");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error4 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success4 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success4");
      }

      /*move_group.setPoseReferenceFrame("base_footprint");

      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z - 0.08;
      waypoints[0].orientation = test_stamped.pose.orientation;
      success = (bool)move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), true);
      if (!success) {
      ROS_INFO("plan error4");
      move_group.detachObject(req.object.name);
      return(false);
      }
      move_group.execute(plan);*/



      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);

      detect_object_client.call(detect_object_srv);

      // recognize again
      moveArm(recover_pose_name);


      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = -0.045;
      target_pose_stamped.pose.position.y = 0;
      target_pose_stamped.pose.position.z = 0.03;
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI *138/180, M_PI);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error5");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error5");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error5 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success5 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success5");
      }

      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);
      both_ezgripper.goToPosition(1.2, 20);
      move_group2.move();

      //move_group.setPoseReferenceFrame("base_footprint");

      test_stamped= move_group.getCurrentPose();

      test_stamped.pose.position.z += 0.13;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error6");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error6");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error6 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success6 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success6");
      }

      /*test_stamped= move_group.getCurrentPose();
      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z + 0.2;
      waypoints[0].orientation = test_stamped.pose.orientation;
      move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), false);
      move_group.execute(plan);*/

      double wrist_joint_value = move_group.getCurrentJointValues()[5];

      move_group.setJointValueTarget("wrist_3_joint", wrist_joint_value + M_PI);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error7");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error7");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error7 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success7 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success7");
      }

      /*double wrist_joint_value = move_group.getCurrentJointValues()[5];

      test_stamped= move_group.getCurrentPose();
      listener.waitForTransform("shelfB_board_2", test_stamped.header.frame_id, test_stamped.header.stamp, ros::Duration(2.0));
      //ros::Rate rate(10);
      for (int i = 0; i < 10; i++) {
      listener.transformPose("shelfB_board_2", test_stamped, target_pose_stamped);
      rate.sleep();
      }

      if (target_pose_stamped.pose.position.y >= 0.45){
      move_group.setJointValueTarget("wrist_3_joint", wrist_joint_value - M_PI);
      } else {
      move_group.setJointValueTarget("wrist_3_joint", wrist_joint_value + M_PI);
      }
      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped);
      move_group.plan(plan);
      move_group.execute(plan);*/

      test_stamped= move_group.getCurrentPose();
      detect_object_client.call(detect_object_srv);

      test_stamped.pose.position.z -= 0.11;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error8");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error8");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error8 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success8 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success8");
      }



      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);

      detect_object_client.call(detect_object_srv);

      //test_stamped.pose.position.x -= 0.03;
      //move_group.setJointValueTarget(test_stamped);
      //move_group.move();
      moveArm(recover_pose_name);
        break;
      }


    case fcsc_msgs::RecognizedObject::FRONT : {// 前面接地 完成OK
      // ros::NodeHandle nh;
      // ros::AsyncSpinner spinner(1);
      // tf::TransformListener			listener;
      // ros::ServiceClient				detect_object_client;
      // spinner.start();
      geometry_msgs::PoseStamped test_stamped;
      geometry_msgs::PoseStamped target_pose_stamped;
      geometry_msgs::PoseStamped target_pose_stamped_2;
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::PoseStamped current_pose_stamped;
      geometry_msgs::PoseStamped waypoint_stamped;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group.setMaxVelocityScalingFactor(0.1);
      //moveit::planning_interface::MoveGroupInterface   move_group("manipulator");
      moveit::planning_interface::MoveGroupInterface   move_group2("endeffector");
      //XmlRpc::XmlRpcValue params;
      EZGripper	both_ezgripper("/ezgripper/both");
      // detect_object_client = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
      fcsc_msgs::DetectObject	detect_object_srv;
      detect_object_srv.request.detect = true;
      detect_object_srv.request.visualize = true;



      //位置を修正する
      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = -0.025;
      target_pose_stamped.pose.position.y = 0;
      target_pose_stamped.pose.position.z = 0.025;
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI * 220 / 180, 0);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      bool success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error1");
        return(false);
      }

      move_group.execute(plan);

      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);
      both_ezgripper.goToPosition(1.2, 20);
      move_group2.move();

      test_stamped= move_group.getCurrentPose();
      //move_group.setStartStateToCurrentState();

      test_stamped.pose.position.z += 0.1;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error2");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);

      /*move_group.setPoseReferenceFrame("base_footprint");

      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z + 0.15;
      waypoints[0].orientation = test_stamped.pose.orientation;
      success = (bool) move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), true);
      if (!success) {
      ROS_INFO("plan error2");
      move_group.detachObject(req.object.name);
      return(false);
      }
      move_group.execute(plan);*/

      test_stamped= move_group.getCurrentPose();
      listener.waitForTransform("shelfB_board_2", test_stamped.header.frame_id, test_stamped.header.stamp, ros::Duration(2.0));
      ros::Rate rate(10);
      for (int i = 0; i < 10; i++) {
        listener.transformPose("shelfB_board_2", test_stamped, target_pose_stamped);
        rate.sleep();
      }

      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, M_PI/2, M_PI / 2);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error3");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);

      /*move_group.setPoseReferenceFrame("base_footprint");

      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z - 0.11;
      waypoints[0].orientation = test_stamped.pose.orientation;
      success = (bool) move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), true);
      if (!success) {
      ROS_INFO("plan error4");
      move_group.detachObject(req.object.name);
      return(false);
      }
      move_group.execute(plan);*/

      test_stamped= move_group.getCurrentPose();

      test_stamped.pose.position.z -= 0.08;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error4");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);


      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);

      detect_object_client.call(detect_object_srv);
      moveArm(recover_pose_name);
      //再認識する
      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = -0.01;//-0.02
      target_pose_stamped.pose.position.y = 0;
      target_pose_stamped.pose.position.z = 0.03;//0.07
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI, 0);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setMaxVelocityScalingFactor(0.2);
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error5");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);

      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);

      both_ezgripper.goToPosition(1.5, 20);
      move_group2.move();
      move_group.setPoseReferenceFrame("base_footprint");

      /*waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x ;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z ;
      waypoints[0].orientation = test_stamped.pose.orientation;
      move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), false);
      move_group.execute(plan);*/

      //target_pose_stamped = move_group.getCurrentPose();
      transformPose("shelfB_board_2", target_pose_stamped, target_pose_stamped_2, listener);


      target_pose_stamped_2.header.frame_id = "shelfB_board_2";

      /*if (target_pose_stamped_2.pose.position.x >= 0.25){
      target_pose_stamped_2.pose.position.x -= 0.1;
      } else {
      target_pose_stamped_2.pose.position.x += 0.1;
      }
      if (target_pose_stamped_2.pose.position.y >= 0.45){
      target_pose_stamped_2.pose.position.y -= 0.1;
      } else {
      target_pose_stamped_2.pose.position.y += 0.1;
      }*/


      target_pose_stamped.pose.position.x = 0.25;//-0.02
      target_pose_stamped.pose.position.y = 0.45;
      target_pose_stamped_2.pose.position.z = 0.13;//0.07
      target_pose_stamped_2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/4,0);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped_2, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error6");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);



      double wrist_joint_value = move_group.getCurrentJointValues()[5];

      move_group.setJointValueTarget("wrist_3_joint", wrist_joint_value + M_PI);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error7");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);


      test_stamped= move_group.getCurrentPose();

      waypoints.resize(1);
      waypoints[0].position.x = test_stamped.pose.position.x;
      waypoints[0].position.y = test_stamped.pose.position.y;
      waypoints[0].position.z = test_stamped.pose.position.z - 0.11;
      waypoints[0].orientation = test_stamped.pose.orientation;
      (bool) move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, moveit_msgs::Constraints(), true);
      if (!success) {
        ROS_INFO("plan error8");
        move_group.detachObject(req.object.name);
        return(false);
      }

      move_group.execute(plan);


      detect_object_client.call(detect_object_srv);

      // [0, 1.94] (rad)
      // open: 0
      // close: 1.94
      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);

      detect_object_client.call(detect_object_srv);

      move_group.move();
      moveArm(recover_pose_name);
      break;
    }
    case fcsc_msgs::RecognizedObject::RIGHT_SIDE :// 側面接地
    {
      ros::AsyncSpinner spinner(1);
      ros::NodeHandle nh;
      // tf::TransformListener                   listener;
      // ros::ServiceClient                      detect_object_client;
      spinner.start();
      geometry_msgs::PoseStamped test_stamped;
      geometry_msgs::PoseStamped target_pose_stamped;
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::PoseStamped current_pose_stamped;
      geometry_msgs::PoseStamped waypoint_stamped;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group.setMaxVelocityScalingFactor(0.1);
      // moveit::planning_interface::MoveGroupInterface   move_group("manipulator");
      moveit::planning_interface::MoveGroupInterface   move_group2("endeffector");
      EZGripper	both_ezgripper("/ezgripper/both");
      // detect_object_client = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
      fcsc_msgs::DetectObject	detect_object_srv;
      detect_object_srv.request.detect = true;
      detect_object_srv.request.visualize = true;

      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = 0;
      target_pose_stamped.pose.position.y = 0.01;
      target_pose_stamped.pose.position.z = 0.14;
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0, - M_PI/2);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      bool success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error1");
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error1");
        for ( int i= 1; i <= 3; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error1 %d回目", (int) i+1 );
            if (i == 3){
              return(false);
            }
            continue;
          } else {
            ROS_INFO("execute  success1 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success1");
      }

      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);
      move_group.setSupportSurfaceName(shelfB_name);
      both_ezgripper.goToPosition(1.94, 10);
      move_group2.move();

      move_group.setStartStateToCurrentState();
      test_stamped.pose.position.z += 0.08;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error2");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error2");
        for ( int i= 1; i <= 3; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error2 %d回目", (int) i+1 );
            if (i == 3){
              return(false);
            }
            continue;
          } else {
            ROS_INFO("execute  success2 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success2");
      }


      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);
      moveArm(recover_pose_name);
      break;
    }
     case fcsc_msgs::RecognizedObject::LEFT_SIDE :
     {
      ros::AsyncSpinner spinner(1);
      ros::NodeHandle nh;
      // tf::TransformListener                   listener;
      // ros::ServiceClient                      detect_object_client;
      spinner.start();
      geometry_msgs::PoseStamped test_stamped;
      geometry_msgs::PoseStamped target_pose_stamped;
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::PoseStamped current_pose_stamped;
      geometry_msgs::PoseStamped waypoint_stamped;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group.setMaxVelocityScalingFactor(0.1);
      // moveit::planning_interface::MoveGroupInterface   move_group("manipulator");
      moveit::planning_interface::MoveGroupInterface   move_group2("endeffector");
      EZGripper	both_ezgripper("/ezgripper/both");
      // detect_object_client = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
      fcsc_msgs::DetectObject	detect_object_srv;
      detect_object_srv.request.detect = true;
      detect_object_srv.request.visualize = true;

      target_pose_stamped.header.frame_id = req.object.name;
      target_pose_stamped.pose.position.x = 0;
      target_pose_stamped.pose.position.y = -0.01;
      target_pose_stamped.pose.position.z = 0.14;
      target_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0,  M_PI/2);

      target_pose_stamped.header.stamp = ros::Time();
      transformPose("base_footprint", target_pose_stamped, test_stamped, listener);

      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      bool success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error1");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error1");
        for ( int i= 1; i <= 3; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error1 %d回目", (int) i+1 );
            if (i == 3){
              return(false);
            }
            continue;
          } else {
            ROS_INFO("execute  success1 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success1");
      }

      move_group.attachObject(req.object.name, move_group.getEndEffectorLink(),touch_links_);
      move_group.setSupportSurfaceName(shelfB_name);
      both_ezgripper.goToPosition(1.94, 10);
      move_group2.move();

      move_group.setStartStateToCurrentState();
      test_stamped.pose.position.z += 0.08;
      move_group.setJointValueTarget(test_stamped);
      move_group.setStartStateToCurrentState();
      success = (bool)move_group.plan(plan);
      if (!success) {
        ROS_INFO("plan error2");
        move_group.detachObject(req.object.name);
        return(false);
      }

      success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute  error2");
        for ( int i= 1; i <= 3; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute error2 %d回目", (int) i+1 );
            if (i == 3){
              return(false);
            }
            continue;
          } else {
            ROS_INFO("execute success2 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute success2");
      }
      both_ezgripper.goToPosition(0);
      move_group2.move();
      move_group.detachObject(req.object.name);
      moveArm(recover_pose_name);
      break;
    }
  }
  return (true);
}
