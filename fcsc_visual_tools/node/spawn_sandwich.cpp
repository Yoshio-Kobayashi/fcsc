#include <ros/ros.h>
#include <iostream>
#include <fcsc_visual_tools/fcsc_visual_tools.h>

using namespace std;

double deg2rad(double deg)
{
  double rad = deg * M_PI / 180.0;
  return (rad);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "spawn_sandwich");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped sandwich_pose;
  std::string sandwich_name;
  double roll, pitch, yaw;

  cerr << "usage: rosrun fcsc_moveit spawn_sandwich _sandwich_name:=[string] _frame_id:=[string] _x:=[double](m) _y:=[double](m) _roll:=[double](deg) _pitch:=[double](deg) _yaw:=[double](deg)" << endl;

  private_nh.param<std::string>("sandwich_name", sandwich_name, "sandwich");
  private_nh.param<std::string>("frame_id", sandwich_pose.header.frame_id, "shelf_board_2");
  private_nh.param<double>("x", sandwich_pose.pose.position.x, 0.0);
  private_nh.param<double>("y", sandwich_pose.pose.position.y, 0.0);
  private_nh.param<double>("roll", roll, 0);
  private_nh.param<double>("pitch", pitch, 0);
  private_nh.param<double>("yaw", yaw, 0);

  sandwich_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( deg2rad(roll), deg2rad(pitch), deg2rad(yaw) );

  moveit_visual_tools::FCSCVisualTools object_visualizer;

  object_visualizer.spawnProduct(sandwich_name, sandwich_pose);
  return 0;
}
