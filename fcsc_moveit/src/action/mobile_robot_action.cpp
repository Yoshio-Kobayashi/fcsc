#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

bool FcscCore::goToStockingBasePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  asyncMoveArm(container_center_pose_name);

  waitForExecute();

  // 固定台車位置
  geometry_msgs::PoseStamped target_base_pos;
  target_base_pos.header.frame_id = "shelfA";
  target_base_pos.pose.position.x = -1.2;
  target_base_pos.pose.position.y = 0.0;
  target_base_pos.pose.orientation.w = 1.0;
  mobile_robot.moveOnGlobalCoordinate(target_base_pos);
  res.success = true;

  // geometry_msgs::PoseStamped pose_in;
  // geometry_msgs::PoseStamped pose_out;
  //
  // pose_in.header.frame_id = "shelfA";
  // pose_in.pose.orientation.w = 1.0;
  // transformPose("base_footprint", pose_in, pose_out);
  // mobile_robot.moveOnRobotCoordinate(pose_out.pose.position.x - 1.2, pose_out.pose.position.y, tf::getYaw(pose_out.pose.orientation));

  Stop(stop);

  return (true);
}

bool FcscCore::goToFaceupBasePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  asyncMoveArm(container_center_pose_name);

  waitForExecute();

  geometry_msgs::PoseStamped target_base_pos;
  target_base_pos.header.frame_id = "shelfB";
  target_base_pos.pose.position.x = -1.2;
  target_base_pos.pose.position.y = 0.0;
  target_base_pos.pose.orientation.w = 1.0;
  mobile_robot.moveOnGlobalCoordinate(target_base_pos);
  res.success = true;

  // geometry_msgs::PoseStamped pose_in;
  // geometry_msgs::PoseStamped pose_out;
  //
  // pose_in.header.frame_id = "shelfB";
  // pose_in.pose.orientation.w = 1.0;
  // transformPose("base_footprint", pose_in, pose_out);
  // mobile_robot.moveOnRobotCoordinate(pose_out.pose.position.x - 1.2, pose_out.pose.position.y, tf::getYaw(pose_out.pose.orientation));
  // mobile_robot.moveBoard(0.01);

  return (true);
}

bool FcscCore::goToNearShelf(fcsc_msgs::GoToNearShelf::Request &req, fcsc_msgs::GoToNearShelf::Response &res)
{
  geometry_msgs::Pose2D goal_2d_pose;
  geometry_msgs::PoseStamped goal_pose;

  asyncMoveArm(container_center_pose_name);

  waitForExecute();

  // 棚の近くまで移動
  switch (req.type) {
    // 棚A(商品陳列作業)
    case 0:
            // 相対位置
            // goal_2d_pose.y = -2.0;//-1.8;
            // mobile_robot.moveOnRobotCoordinate(goal_2d_pose.x, goal_2d_pose.y, goal_2d_pose.theta);

            // 絶対位置
            goal_pose.header.frame_id = "map";
            goal_pose.pose.position.x = 0.4;
            goal_pose.pose.position.y = 1.55 - (1.92 - 0.95 / 2.0);
            goal_pose.pose.orientation.w = 1.0;
            mobile_robot.moveOnGlobalCoordinate(goal_pose);
            break;
    case 1: goal_pose.header.frame_id = "shelfA";
            goal_pose.pose.position.x = -1.4;
            goal_pose.pose.position.y = 0.0;
            goal_pose.pose.orientation.w = 1.0;
            mobile_robot.moveOnGlobalCoordinate(goal_pose);
            break;
    // 棚B(廃棄品回収・フェイスアップ作業)
    case 2:
            // 相対移動
            // goal_2d_pose.y = 1.0;
            // mobile_robot.moveOnRobotCoordinate(goal_2d_pose.x, goal_2d_pose.y, goal_2d_pose.theta);

            // map基準で相対移動
            goal_pose.header.frame_id = "map";
            goal_pose.pose.position = mobile_robot.getCurrentBasePose(goal_pose.header.frame_id).pose.position;
            goal_pose.pose.position.y += 1.0;
            goal_pose.pose.orientation.w = 1.0;
            mobile_robot.moveOnGlobalCoordinate(goal_pose);

            // 絶対位置
            // goal_pose.header.frame_id = "map";
            // goal_pose.pose.position.x = 0.4;
            // goal_pose.pose.position.y = 1.2;
            // goal_pose.pose.orientation.w = 1.0;
            // mobile_robot.moveOnGlobalCoordinate(goal_pose);

            // mobile_robot.moveBoard(0.09);
            break;
    case 3: goal_pose.header.frame_id = "shelfB";
            goal_pose.pose.position.x = -1.4;
            goal_pose.pose.position.y = 0.0;
            goal_pose.pose.orientation.w = 1.0;
            mobile_robot.moveOnGlobalCoordinate(goal_pose);
  }
  res.success = true;


  return (true);
}

bool FcscCore::goToHomePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_WARN("goToHomePosition");

  if (getCurrentEEfPose("base_footprint").pose.position.x > 0.4) {
    asyncMoveArm("fold");
  }
  asyncMoveArm(container_center_pose_name);

  geometry_msgs::PoseStamped goal_pose;

  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.5 - 1.0;
  goal_pose.pose.position.y = 1.55 + 0.4 + 0.2;
  goal_pose.pose.orientation.w = 1.0;

  ROS_WARN("moveOnGlobalCoordinate start");
  waitForExecute();
  // mobile_robot.moveOnGlobalCoordinate(mobile_robot.getInitialBasePose());
  mobile_robot.moveOnGlobalCoordinate(goal_pose);
  res.success = true;
  ROS_WARN("moveOnGlobalCoordinate finish");

  return (true);
}
