#include <ros/ros.h>
#include <iostream>
#include <fcsc_visual_tools/fcsc_visual_tools.h>

using namespace std;

double deg2rad(double deg)
{
  double rad = deg * M_PI / 180.0;
  // cout << rad << endl;
  return (rad);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "spawn_shelf");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped shelf_pose;
  std::string shelf_name;
  double roll, pitch, yaw;

  cerr << "usage: rosrun fcsc_moveit spawn_shelf _shelf_name:=[string] _frame_id:=[string] _x:=[double](m) _y:=[double](m) _roll:=[double](deg) _pitch:=[double](deg) _yaw:=[double](deg)" << endl;

  if (nh.hasParam("shelf_size") == false) {
    ROS_INFO("Set shelf_size");
    system("rosparam load `rospack find fcsc_description`/config/shelf_size.yaml");
  }

  private_nh.param<std::string>("shelf_name", shelf_name, "shelf");
  private_nh.param<std::string>("frame_id", shelf_pose.header.frame_id, "map");
  private_nh.param<double>("x", shelf_pose.pose.position.x, 0.75);
  private_nh.param<double>("y", shelf_pose.pose.position.y, 0.0);
  private_nh.param<double>("roll", roll, 0);
  private_nh.param<double>("pitch", pitch, 0);
  private_nh.param<double>("yaw", yaw, 0);

  shelf_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( deg2rad(roll), deg2rad(pitch), deg2rad(yaw) );

  moveit_visual_tools::FCSCVisualTools object_visualizer;

  object_visualizer.spawnShelf(shelf_name, shelf_pose);
  return 0;
}
