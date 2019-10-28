#ifndef FCSC_MOBILE_ROBOT
#define FCSC_MOBILE_ROBOT
#include <ros/ros.h>
#include <iostream>
#include <comm_client/RobotPose.h>
#include <comm_client/GoalPose.h>
#include <comm_client/MoveLift.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

class MobileRobot {
public:
  MobileRobot();
  geometry_msgs::PoseStamped getCurrentBasePose(std::string frame_id="map");
  geometry_msgs::PoseStamped getInitialBasePose(std::string frame_id="map");
  geometry_msgs::PoseStamped moveOnGlobalCoordinate(geometry_msgs::PoseStamped target_pose);
  geometry_msgs::PoseStamped moveOnRobotCoordinate(double x, double y, double deg);
  geometry_msgs::PoseStamped moveBoard(double height);
  bool isStop();

private:
  ros::ServiceClient    get_mobile_base_pose_client;
  ros::ServiceClient    global_goal_pose_client;
  ros::ServiceClient    local_goal_pose_client;
  ros::ServiceClient    move_lift_client;
  ros::ServiceClient    check_mobile_flag_client;
  tf::TransformListener listener;

  moveit::planning_interface::MoveGroupInterface board_move_group;

  bool use_simulator;

  double max_board_height;
  double min_board_height;

  geometry_msgs::PoseStamped  current_pose;
  geometry_msgs::PoseStamped  initial_pose;

  void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out);
};
#endif
