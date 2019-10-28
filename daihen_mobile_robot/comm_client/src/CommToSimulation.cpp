#include<iostream>
#include<math.h>
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_srvs/Trigger.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include <tf/transform_listener.h>
#include<move_base_msgs/MoveBaseActionResult.h>

#include<comm_client/RobotPose.h>
#include<comm_client/GoalPose.h>
#include<comm_client/MoveLift.h>

#include "NetIf.h"
#include "CommToTransRobot.h"

using namespace std;

class CommSimulation{

private:
    ros::Publisher goal_pose_pub, goal_lift_pub;
    ros::Subscriber move_base_result_sub;
    ros::ServiceServer robot_pose_srv;
    ros::ServiceServer move_global_action_srv;
    ros::ServiceServer check_moving_srv;
    ros::ServiceServer move_local_action_srv;
    ros::ServiceServer move_lift_srv;
    tf::TransformListener tf_listener_; //tfを読み込む

    bool moving_flag;
    std_msgs::Float64 goal_lift;

    double max_lift_height;
    double min_lift_height;

    void sendGoalPose(std::string frame_id, geometry_msgs::Pose2D goal_pose2d);
    geometry_msgs::Pose getCurrentRobotPose();

public:
    CommSimulation();
    bool checkRobotPoseCallback(comm_client::RobotPose::Request &req, comm_client::RobotPose::Response &res);
    bool actionGlobalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res);
    bool actionLocalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res);
    bool checkMovingFlagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool moveLiftCallback(comm_client::MoveLift::Request &req, comm_client::MoveLift::Response &res);
    void resultCallback(const move_base_msgs::MoveBaseActionResult result);

};

CommSimulation::CommSimulation():
max_lift_height(0.09),
min_lift_height(0.01)
{

 	ros::NodeHandle nh;

  goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
  goal_lift_pub = nh.advertise<std_msgs::Float64>("/goal_lift", 1);

  move_base_result_sub = nh.subscribe("/move_base/result", 1, &CommSimulation::resultCallback, this);

	move_global_action_srv = nh.advertiseService("action_global_goal_pose", &CommSimulation::actionGlobalGoalPoseCallback, this);
  move_local_action_srv = nh.advertiseService("action_local_goal_pose", &CommSimulation::actionLocalGoalPoseCallback, this);
	check_moving_srv = nh.advertiseService("check_moving_flag", &CommSimulation::checkMovingFlagCallback, this);
  robot_pose_srv = nh.advertiseService("check_robot_pose", &CommSimulation::checkRobotPoseCallback, this);
  move_lift_srv = nh.advertiseService("move_lift", &CommSimulation::moveLiftCallback, this);

  moving_flag = true;
  goal_lift.data = 0.0;
}

void CommSimulation::sendGoalPose(std::string frame_id, geometry_msgs::Pose2D goal_pose2d)
{
  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = frame_id;
  goal.pose.position.x = goal_pose2d.x;
  goal.pose.position.y = goal_pose2d.y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose2d.theta);
  goal_pose_pub.publish(goal);

  moving_flag  = false;
  ros::Rate r(10);
  while(!moving_flag){
      ros::spinOnce();
      r.sleep();
  }
}

bool CommSimulation::moveLiftCallback(comm_client::MoveLift::Request &req, comm_client::MoveLift::Response &res)
{
  goal_lift = req.height;
  if (goal_lift.data > max_lift_height) {
    goal_lift.data = max_lift_height;
  } else if (goal_lift.data < min_lift_height) {
    goal_lift.data = min_lift_height;
  }
  goal_lift_pub.publish(goal_lift);
  res.reached_pose = getCurrentRobotPose();
  return (true);
}


geometry_msgs::Pose CommSimulation::getCurrentRobotPose()
{
  geometry_msgs::Pose pose;
  tf::StampedTransform robot_gl;

  tf_listener_.lookupTransform(std::string("/map"), std::string("/base_footprint"), ros::Time(0), robot_gl); //map-base_footprint間のtfを取得

  //現在のロボットの姿勢を更新
	pose.position.x = robot_gl.getOrigin().x();
	pose.position.y = robot_gl.getOrigin().y();
	pose.position.z = goal_lift.data;
	pose.orientation.x = robot_gl.getRotation().x();
	pose.orientation.y = robot_gl.getRotation().y();
	pose.orientation.z = robot_gl.getRotation().z();
	pose.orientation.w = robot_gl.getRotation().w();

  printf("now:x = %f, y = %f, theta = %f[rad], z = %f\n", pose.position.x, pose.position.y, pose.orientation.w, pose.position.z);

  return (pose);
}

bool CommSimulation::checkRobotPoseCallback(comm_client::RobotPose::Request &req, comm_client::RobotPose::Response &res){

  res.robot_pose = getCurrentRobotPose();
  return true;
}

bool CommSimulation::actionGlobalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res){
  sendGoalPose("map", req.goal_pose);
  res.reached_pose = getCurrentRobotPose();
  return true;
}

bool CommSimulation::actionLocalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res){
  sendGoalPose("base_footprint", req.goal_pose);
  res.reached_pose = getCurrentRobotPose();
  return true;
}

bool CommSimulation::checkMovingFlagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	res.success = moving_flag;
	return true;
}

void CommSimulation::resultCallback(const move_base_msgs::MoveBaseActionResult result){

    moving_flag = true;
    return;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "comm_simulation");

    CommSimulation simulation;

    ros::spin();

    return (0);

}
