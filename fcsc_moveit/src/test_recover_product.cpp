#include <ros/ros.h>
#include <fcsc_visual_tools/fcsc_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fcsc_msgs/DetectProduct.h>
#include <fcsc_msgs/DetectSandwich.h>
#include <fcsc_msgs/Manipulate.h>
#include <std_srvs/Trigger.h>


int main(int argc, char **argv) {
  ros::init (argc, argv, "test_recover_product");
  ros::NodeHandle nh;

  ros::ServiceClient detect_sandwich_client;
  ros::ServiceClient pickup_sandwich_client;
  ros::ServiceClient faceup_sandwich_client;
  ros::ServiceClient recover_sandwich_client;
  ros::ServiceClient goto_faceup_base_pos_client;

  fcsc_msgs::Manipulate       manipulate_srv;
  fcsc_msgs::DetectSandwich   detect_sandwich_srv;
  std_srvs::Trigger         trigger_srv;

  moveit_visual_tools::FCSCVisualTools   object_visualizer;

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  detect_sandwich_client  = nh.serviceClient<fcsc_msgs::DetectSandwich>("detect_sandwich");
  pickup_sandwich_client  = nh.serviceClient<fcsc_msgs::Manipulate>("pickup_sandwich");
  faceup_sandwich_client  = nh.serviceClient<fcsc_msgs::Manipulate>("faceup_sandwich");
  recover_sandwich_client = nh.serviceClient<fcsc_msgs::Manipulate>("recover_sandwich");
  goto_faceup_base_pos_client  = nh.serviceClient<std_srvs::Trigger>("goto_faceup_base_position");

  goto_faceup_base_pos_client.call(trigger_srv);

  detect_sandwich_srv.request.demo = true;
  detect_sandwich_client.call(detect_sandwich_srv);
  for (size_t i = 0; i < detect_sandwich_srv.response.sandwiches.size(); i++) {
    ROS_ERROR("pickup:%s", detect_sandwich_srv.response.sandwiches[i].name.c_str());
    manipulate_srv.request.object = detect_sandwich_srv.response.sandwiches[i];
    pickup_sandwich_client.call(manipulate_srv);
    if (manipulate_srv.response.success) {
      ROS_ERROR("recover:%s", detect_sandwich_srv.response.sandwiches[i].name.c_str());
      recover_sandwich_client.call(manipulate_srv);
    } else {
      move_group.setNamedTarget("masita3");
      move_group.move();
    }
  }

  return 0;
}
