//認識物体の座標系を出す
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <fcsc_msgs/RecognizedObjectArray.h>

using namespace std;

fcsc_msgs::RecognizedObjectArray recognized_object_array_msg;

void callBack(const fcsc_msgs::RecognizedObjectArray msg)
{
  recognized_object_array_msg = msg;
  std::cerr << recognized_object_array_msg << '\n';
}

int main(int argc, char** argv){
  ros::init(argc, argv, "detected_object_tf_broadcaster");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Subscriber detected_object_array_sub;
  geometry_msgs::Pose pose;

  detected_object_array_sub = nh.subscribe("/detected_object_array", 10, callBack);


  ros::Rate rate(50.0);
  while (nh.ok()){
    int size = recognized_object_array_msg.objects.size();
    for (int i = 0; i < size; i++) {
      geometry_msgs::PoseStamped ps;
      std::string object_name;

      ps = recognized_object_array_msg.objects[i].pose;
      object_name = recognized_object_array_msg.objects[i].name;

      transform.setOrigin( tf::Vector3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) );
      transform.setRotation( tf::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ps.header.frame_id, object_name));
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
