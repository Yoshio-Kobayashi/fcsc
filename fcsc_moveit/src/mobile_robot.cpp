#include "fcsc_moveit/mobile_robot.h"

double deg2rad(double deg)
{
  return (deg * M_PI / 180.0);
}

MobileRobot::MobileRobot():
board_move_group("boad"),
max_board_height(0.09),
min_board_height(0.01)
{
  ros::NodeHandle nh;

  get_mobile_base_pose_client = nh.serviceClient<comm_client::RobotPose>("/check_robot_pose");
  global_goal_pose_client     = nh.serviceClient<comm_client::GoalPose>("/action_global_goal_pose");
  local_goal_pose_client      = nh.serviceClient<comm_client::GoalPose>("/action_local_goal_pose");
  check_mobile_flag_client    = nh.serviceClient<std_srvs::Trigger>("/check_moving_flag");
  move_lift_client            = nh.serviceClient<comm_client::MoveLift>("/move_lift");

  nh.param<bool>("use_simulator", use_simulator, true);

  initial_pose = current_pose = getCurrentBasePose("map");
}

geometry_msgs::PoseStamped MobileRobot::moveOnGlobalCoordinate(geometry_msgs::PoseStamped target_pose)
{
  comm_client::GoalPose srv;
  geometry_msgs::PoseStamped goal_pose;

  transformPose("map", target_pose, goal_pose);

  srv.request.goal_pose.x = goal_pose.pose.position.x;
  srv.request.goal_pose.y = goal_pose.pose.position.y;
  srv.request.goal_pose.theta = tf::getYaw(goal_pose.pose.orientation);
  global_goal_pose_client.call(srv);

  return (current_pose);
}

geometry_msgs::PoseStamped MobileRobot::moveOnRobotCoordinate(double x, double y, double deg)
{
  comm_client::GoalPose srv;

  srv.request.goal_pose.x = x;
  srv.request.goal_pose.y = y;
  srv.request.goal_pose.theta = deg2rad(deg);

  local_goal_pose_client.call(srv);

  return (current_pose);
}

geometry_msgs::PoseStamped MobileRobot::moveBoard(double height)
{
  comm_client::MoveLift srv;

  srv.request.height.data = height;

  std::cerr << "target_height:" << srv.request.height.data << '\n';

  move_lift_client.call(srv);

  if (use_simulator) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_.joint_trajectory.joint_names.push_back("boad_joint");
    plan.trajectory_.joint_trajectory.points.resize(2);
    plan.trajectory_.joint_trajectory.points[0].positions.push_back(board_move_group.getCurrentJointValues()[0]);
    plan.trajectory_.joint_trajectory.points[0].effort.push_back(10);
    plan.trajectory_.joint_trajectory.points[1].positions.push_back(height);
    plan.trajectory_.joint_trajectory.points[1].effort.push_back(10);
    plan.planning_time_ = 0.3;
    board_move_group.execute(plan);
  }

  current_pose.header.frame_id = "map";
  current_pose.pose = srv.response.reached_pose;

  return (current_pose);
}

geometry_msgs::PoseStamped MobileRobot::getCurrentBasePose(std::string frame_id)
{
  geometry_msgs::PoseStamped tf_current_pose;
  comm_client::RobotPose srv;

  if (get_mobile_base_pose_client.call(srv)) {
    current_pose.header.frame_id = "map";
    current_pose.pose = srv.response.robot_pose;
    transformPose(frame_id, current_pose, tf_current_pose);
    return (tf_current_pose);
  }
}

geometry_msgs::PoseStamped MobileRobot::getInitialBasePose(std::string frame_id)
{
  geometry_msgs::PoseStamped tf_pose;

  transformPose(frame_id, initial_pose, tf_pose);
  return (tf_pose);
}

bool MobileRobot::isStop()
{
  std_srvs::Trigger srv;

  check_mobile_flag_client.call(srv);

  return (srv.response.success);
}

void MobileRobot::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out)
{
  //we'll just use the most recent transform available for our simple example
  ps_in.header.stamp = ros::Time();
  ros::Rate rate(10);
  while (1) {
    try{
      listener.transformPose(frame, ps_in, ps_out);
      return;
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a position from \"%s\" to \"%s\": %s", ps_in.header.frame_id.c_str(), frame.c_str(), ex.what());
    }
    rate.sleep();
  }
}
