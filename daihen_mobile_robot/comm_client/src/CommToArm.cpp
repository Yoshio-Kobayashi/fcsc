//===========================================================================// Name        : CommToArm.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include<iostream>
#include<math.h>
#include<std_srvs/Trigger.h>
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Pose2D.h>
#include<tf/tf.h>

#include<comm_client/RobotPose.h>
#include<comm_client/GoalPose.h>
#include<comm_client/MoveLift.h>


#include "NetIf.h"
#include "CommToTransRobot.h"

#include <sensor_msgs/JointState.h>

using namespace std;


uchar CommandCode::m_sSeqNo = 0x80;

class CommClient {

private:
	ros::ServiceServer robot_pose_srv, move_global_action_srv, check_moving_srv, move_local_action_srv, move_lift_srv;

	CommToTransRobot commToTransRobot;
	std::string ipAdress = ""; // 搬送ロボットのIPアドレス
	int port;// 搬送ロボットのポート番号
	bool moving_flag;//移動状態判別用フラグ

	double max_board_height;
	double min_board_height;

	ros::Publisher	mobile_base_pose_publisher;
	ros::Publisher 	board_joint_publisher;
	void publishBasePose(geometry_msgs::Pose pose);
	void publishBoardJoint(double height);

	bool sendMoveActionCommand(const float move_x, const float move_y, const float move_rad, const float move_z);
	void quaternionToRPY(const geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw);
	void rpyToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q);

public:
	CommClient();
	~CommClient();
	bool checkRobotPoseCallback(comm_client::RobotPose::Request &req, comm_client::RobotPose::Response &res);
	bool actionGlobalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res);
	bool actionLocalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res);
	bool moveLiftCallback(comm_client::MoveLift::Request &req, comm_client::MoveLift::Response &res);
	bool checkMovingFlagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};

CommClient::CommClient():
max_board_height(0.09),
min_board_height(0.01)
{

	ros::NodeHandle nh;
	move_global_action_srv = nh.advertiseService("action_global_goal_pose", &CommClient::actionGlobalGoalPoseCallback, this);
	move_local_action_srv = nh.advertiseService("action_local_goal_pose", &CommClient::actionLocalGoalPoseCallback, this);
	check_moving_srv = nh.advertiseService("check_moving_flag", &CommClient::checkMovingFlagCallback, this);
	robot_pose_srv = nh.advertiseService("check_robot_pose", &CommClient::checkRobotPoseCallback, this);
	move_lift_srv = nh.advertiseService("move_lift", &CommClient::moveLiftCallback, this);

	mobile_base_pose_publisher = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 10);
	board_joint_publisher = nh.advertise<sensor_msgs::JointState>("/board_joint", 10);

	ros::NodeHandle private_nh("~");
	private_nh.param("IPAdress", ipAdress, std::string("192.168.1.8"));
	private_nh.param("Port", port, 10020);
	moving_flag = true;

	// TCPクライアント接続
	cout << "connect..." << endl;
	cout << commToTransRobot.Connect(ipAdress.data(), port) << endl;
	cout << "connet OK" << endl;

	// 現在位置取得
	comm_client::RobotPose srv;
	checkRobotPoseCallback(srv.request, srv.response);
}

CommClient::~CommClient(){

	//TCP終了
	commToTransRobot.Close();
}

/**
 *	@自己位置を返すサービスサーバ
 *	req:
 *	res:	robot_pose(geometry_msgs/Pose)	ロボットの現在位置
 */
bool CommClient::checkRobotPoseCallback(comm_client::RobotPose::Request &req, comm_client::RobotPose::Response &res){

	geometry_msgs::Pose pose;

	//座標要求
	if (commToTransRobot.SendRequestPosCommand()){
		printf("*ERROR*\n");
		res.robot_pose.orientation.w = 1.0;
		return false;
	} else {
		pose.position.x = commToTransRobot.GetPosX();
		pose.position.y = commToTransRobot.GetPosY();
		pose.position.z = commToTransRobot.GetPosZ();
		pose.orientation = tf::createQuaternionMsgFromYaw((double)commToTransRobot.GetRad());

		printf("now:x = %f, y = %f, theta = %f[rad], z = %f\n", pose.position.x, pose.position.y, commToTransRobot.GetRad(), pose.position.z);
		res.robot_pose = pose;

		publishBasePose(pose);
		publishBoardJoint(pose.position.z);

		return true;
	}
}

/**
 *	@直値移動アクションコマンドを送信する
 *	move_XXX:	各パラメータの相対移動量
 */
bool CommClient::sendMoveActionCommand(const float move_x, const float move_y, const float move_rad, const float move_z){

	moving_flag = false;
	//直値移動アクション実行(ロボットの前方がX軸)
	if (commToTransRobot.SendMoveActionCommand(move_x, move_y, move_rad, move_z)){
		printf("*ERROR*\n");
		// エラー時は台車の現在位置が更新されないので，現在位置要求コマンドを送信して更新を行う
		commToTransRobot.SendRequestPosCommand();
		moving_flag = true;
		return false;
	} else {
		moving_flag = true;
		return true;
	}
}

/**
 *	@グローバル座標系で渡された目標地点を相対移動量に変換してsendMoveActionCommandを実行するサービスサーバ
 *	req:	goal_pose(geometry_msgs/Pose2D)	ロボットの目標地点
 *	res:	reached_pose(geometry_msgs/Pose)	移動完了時のロボットの現在位置
 */
bool CommClient::actionGlobalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res){

	double roll, pitch, yaw;

	if (req.height.data > max_board_height) {
		req.height.data = max_board_height;
	} else if (req.height.data < min_board_height) {
		req.height.data = min_board_height;
	}

	float x = commToTransRobot.GetPosX();
	float y = commToTransRobot.GetPosY();
	float rad = commToTransRobot.GetRad();

	float move_x = (req.goal_pose.x - x) * cos(rad) + (req.goal_pose.y - y) * sin(rad);
	float move_y = (req.goal_pose.y - y) * cos(rad) - (req.goal_pose.x - x) * sin(rad);
	// quaternionToRPY(req.goal_pose.orientation, roll, pitch, yaw);
	yaw = req.goal_pose.theta;
	float move_rad = (float)yaw - rad;
	float move_z = req.height.data - commToTransRobot.GetPosZ();

	bool flag = sendMoveActionCommand(move_x, move_y, move_rad, 0);

	geometry_msgs::Pose pose;
	pose.position.x = commToTransRobot.GetPosX();
	pose.position.y = commToTransRobot.GetPosY();
	pose.position.z = commToTransRobot.GetPosZ();
	pose.orientation = tf::createQuaternionMsgFromYaw((double)commToTransRobot.GetRad());
	printf("now:x = %f, y = %f, theta = %f[rad], z = %f\n", pose.position.x, pose.position.y, commToTransRobot.GetRad(), pose.position.z);
	res.reached_pose = pose;

	publishBasePose(pose);
	publishBoardJoint(pose.position.z);

	return flag;
}

/**
 *	@渡された相対移動量をsendMoveActionCommandに送って実行するサービスサーバ
 *	req:	goal_pose(geometry_msgs/Pose2D)	相対移動量
 *	res:	reached_pose(geometry_msgs/Pose)	移動完了時のロボットの現在位置
 */
bool CommClient::actionLocalGoalPoseCallback(comm_client::GoalPose::Request &req, comm_client::GoalPose::Response &res){

	double roll, pitch, yaw;

	if (req.height.data > max_board_height) {
		req.height.data = max_board_height;
	} else if (req.height.data < min_board_height) {
		req.height.data = min_board_height;
	}

	float move_x = req.goal_pose.x;
	float move_y = req.goal_pose.y;
	// quaternionToRPY(req.goal_pose.orientation, roll, pitch, yaw);
	yaw = req.goal_pose.theta;
	float move_rad = (float)yaw ;
	float move_z = req.height.data - commToTransRobot.GetPosZ();

	bool flag = sendMoveActionCommand(move_x, move_y, move_rad, 0);

	geometry_msgs::Pose pose;
	pose.position.x = commToTransRobot.GetPosX();
	pose.position.y = commToTransRobot.GetPosY();
	pose.position.z = commToTransRobot.GetPosZ();
	pose.orientation = tf::createQuaternionMsgFromYaw((double)commToTransRobot.GetRad());
	printf("now:x = %f, y = %f, theta = %f[rad], z = %f\n", pose.position.x, pose.position.y, commToTransRobot.GetRad(), pose.position.z);
	res.reached_pose = pose;

	publishBasePose(pose);
	publishBoardJoint(pose.position.z);

	return flag;
}

/**
 *	@ロボットの移動状態のフラグを配信するサービスサーバ
 *	req:
 *	res:	success(bool)	静止状態ならtrue、移動中ならfalse
 */
bool CommClient::checkMovingFlagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	res.success = moving_flag;
	return true;
}


bool CommClient::moveLiftCallback(comm_client::MoveLift::Request &req, comm_client::MoveLift::Response &res)
{
		if (req.height.data > max_board_height) {
			req.height.data = max_board_height;
		} else if (req.height.data < min_board_height) {
			req.height.data = min_board_height;
		}

		float move_z = req.height.data - commToTransRobot.GetPosZ();

		// mm単位の値を6mm以上でcmに切り上げる
		move_z *= pow(10, 2);
		std::cerr << "move_z:" << move_z << '\n';
		move_z = (float)(int)(move_z + 0.4);
		std::cerr << "move_z:" << move_z << '\n';
		move_z *= pow(10, -2);
		std::cerr << "move_z:" << move_z << '\n';

		std::cerr  << "[moveLiftCallback]:goal_height[" << req.height.data << "] " << "movment:[" << move_z << "]" << endl;

		bool flag = sendMoveActionCommand(0, 0, 0, move_z);

		geometry_msgs::Pose pose;
		pose.position.x = commToTransRobot.GetPosX();
		pose.position.y = commToTransRobot.GetPosY();
		pose.position.z = commToTransRobot.GetPosZ();
		pose.orientation = tf::createQuaternionMsgFromYaw((double)commToTransRobot.GetRad());
		printf("now:x = %f, y = %f, theta = %f[rad], z = %f\n", pose.position.x, pose.position.y, commToTransRobot.GetRad(), pose.position.z);
		res.reached_pose = pose;

		publishBasePose(pose);
		publishBoardJoint(pose.position.z);

		return flag;
}


/**
 *	クォータニオンをrpyに変換する
 */
void CommClient::quaternionToRPY(const geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

/**
 *	rpyをクォータニオンに変換する
 */
void CommClient::rpyToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q){
   tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
   quaternionTFToMsg(quat,q);
}

void CommClient::publishBasePose(geometry_msgs::Pose pose)
{
	geometry_msgs::Pose2D pose_2d;
  ros::Rate loop_rate(50);

  pose_2d.x = pose.position.x;
  pose_2d.y = pose.position.y;
  pose_2d.theta = tf::getYaw(pose.orientation);
  for (size_t i = 0; i < 10; i++) {
    mobile_base_pose_publisher.publish(pose_2d);
    loop_rate.sleep();
  }
}

void CommClient::publishBoardJoint(double height)
{
	ros::Rate rate(50);
  sensor_msgs::JointState joint_state;

  joint_state.header.stamp = ros::Time::now();
  joint_state.name.push_back("boad_joint");
  joint_state.position.push_back(height);
  for (size_t i = 0; i < 10; i++) {
    board_joint_publisher.publish(joint_state);
    rate.sleep();
  }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "comm_client");

    CommClient client;

    ros::spin();

    return (0);

}
