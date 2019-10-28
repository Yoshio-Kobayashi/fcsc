#include <ros/ros.h>
#include <iostream>
#include <fcsc_msgs/GetBasePosition.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

void createGraspPoses(std::vector<fcsc_msgs::PoseStampedWithCost> &grasp_poses)
{
  tf::TransformListener listener;

  grasp_poses.clear();
  std::vector<double> target_z;
  double shelf_width = 0.9;
  double shelf_depth = 0.4;
  double cost = 1.0;

  target_z.push_back(0.1);
  target_z.push_back(0.15);
  target_z.push_back(0.05);

  for (size_t i = 0; i < 3; i++) {
    std::stringstream ss;
    geometry_msgs::PoseStamped grasp_pose_stmp;

    ss << "shelfA_board_" << i + 1;
    grasp_pose_stmp.header.frame_id = ss.str();
    grasp_pose_stmp.header.stamp = ros::Time();
    double sign = 1.0;
    for (size_t n_pos = 0; n_pos < 4; n_pos++) {
      grasp_pose_stmp.pose.position.x = (n_pos / 2) * shelf_depth / 2;
      grasp_pose_stmp.pose.position.y = shelf_width / 2 + sign * 0.2;
      grasp_pose_stmp.pose.position.z = target_z[i];
      for (size_t n_orien = 0; n_orien < 4; n_orien++) {
        geometry_msgs::PoseStamped tmp_pose_stmp;
        grasp_pose_stmp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, n_orien * M_PI / 6, 0);
        ros::Rate rate(10);
        while (1) {
          try{
            listener.transformPose("map", grasp_pose_stmp, tmp_pose_stmp);
            break;
          }
          catch(tf::TransformException& ex){
          }
          rate.sleep();
        }
        fcsc_msgs::PoseStampedWithCost tmp_pose_cost;
        tmp_pose_cost.pose = tmp_pose_stmp;
        tmp_pose_cost.cost = cost;
        grasp_poses.push_back(tmp_pose_cost);
      }
      sign *= -1.0;
    }
  }
}

void showGraspPosesByArrow(std::vector<fcsc_msgs::PoseStampedWithCost> po)
{
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise< visualization_msgs::MarkerArray >("rviz_visual_tools", 1);
  visualization_msgs::MarkerArray markerArr;
  for (int i = 0; i < po.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "grasp_poses";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    // marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = i;

    marker.pose = po[i].pose.pose;
    markerArr.markers.push_back(marker);
  }

  ros::Rate rate(10);
  for (size_t i = 0; i < 10; i++) {
    marker_pub.publish(markerArr);
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_get_base_position_server");

  ros::NodeHandle nh;
  ros::ServiceClient client;
  fcsc_msgs::GetBasePosition srv;

  client = nh.serviceClient<fcsc_msgs::GetBasePosition>("get_base_position");

  createGraspPoses(srv.request.target_poses);

  showGraspPosesByArrow(srv.request.target_poses);

  ros::Time begin = ros::Time::now();
  client.call(srv);
  ros::Time end = ros::Time::now();

  std::cerr << (end - begin).toSec() << '\n';

  std::cout << "-- init base pos --" << '\n';
  std::cout << srv.request.init_base_position << '\n';

  std::cout << "-- optimal base pos --" << '\n';
  std::cout << srv.response.base_position << '\n';
}
