// Subscriber
//    TopicName:cmd_vel MsgType:geometry_msgs::Twist
//    TopicName:robot_pose MsgType:geometry_msgs::Pose2D

// world -> base_footprint
// /cmd_vel によるコントローラでの操作可能
// /robot_pose によるロボットの位置・姿勢を直接設定可能


#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

using namespace std;
geometry_msgs::TransformStamped robot_state;
double yaw;

void CallBack(const geometry_msgs::Twist msg)
{
  double dt;

  dt = (double)(robot_state.header.stamp - ros::Time::now()).toSec();

  //node:teleop_twist_keyboardの指令値が正負逆になっているので-1をかけている
  robot_state.transform.translation.x += (-1.0) * dt * msg.linear.x * cos(yaw);
  robot_state.transform.translation.y += (-1.0) * dt * msg.linear.x * sin(yaw);

  yaw += dt * msg.angular.z;
  robot_state.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
}

void poseCB(const geometry_msgs::Pose2D msg)
{
  robot_state.transform.translation.x = msg.x;
  robot_state.transform.translation.y = msg.y;
  robot_state.transform.rotation = tf::createQuaternionMsgFromYaw(msg.theta);
  yaw = msg.theta;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "virtual_joint_broadcaster");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber twist_sub;
  ros::Subscriber pose_sub;
  std::string world_frame;
  std::string base_frame;

  private_nh.param<std::string>("world_frame", world_frame, "map");
  private_nh.param<std::string>("base_frame", base_frame, "base_footprint");

  yaw = 0;
  twist_sub = nh.subscribe("cmd_vel", 1, CallBack);
  pose_sub = nh.subscribe("robot_pose", 1, poseCB);

	robot_state.header.stamp = ros::Time::now();
	robot_state.header.frame_id = world_frame;
	robot_state.child_frame_id = base_frame;

	robot_state.transform.translation.x = 0;
	robot_state.transform.translation.y = 0;
	robot_state.transform.translation.z = 0;

	robot_state.transform.rotation.x = 0;
	robot_state.transform.rotation.y = 0;
	robot_state.transform.rotation.z = 0;
	robot_state.transform.rotation.w = 1;

  ros::Rate loop_rate(100);
  while(ros::ok()){
    ros::spinOnce();
    robot_state.header.stamp = ros::Time::now();
  	broadcaster.sendTransform(robot_state);
  	loop_rate.sleep();
  }
}
