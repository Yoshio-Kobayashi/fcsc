#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fcsc_msgs/DetectProduct.h>
#include <fcsc_msgs/DetectObject.h>
#include <fcsc_msgs/Manipulate.h>
#include "fcsc_moveit/mobile_robot.h"

// #include <my_ezgripper/ezgripper_interface.h>
// #include <fcsc_msgs/PickupProduct.h>
// #include <fcsc_msgs/PlaceProduct.h>
// #include <fcsc_msgs/ReturnProduct.h>

#include <std_srvs/Trigger.h>

int main(int argc, char **argv) {
  ros::init (argc, argv, "test_stocking_product");
  ros::NodeHandle nh;
  ros::ServiceClient search_product_client;
  ros::ServiceClient detect_object_client;
  ros::ServiceClient pickup_product_client;
  ros::ServiceClient place_product_client;
  ros::ServiceClient return_product_client;
  ros::ServiceClient goto_stocking_base_pos_client;

  // fcsc_msgs::PickupProduct  pickup_product_srv;
  // fcsc_msgs::PlaceProduct   place_product_srv;
  // fcsc_msgs::ReturnProduct  return_product_srv;
  fcsc_msgs::DetectProduct  detect_product_srv;
  fcsc_msgs::DetectObject   detect_object_srv;
  fcsc_msgs::Manipulate     manipulate_srv;
  std_srvs::Trigger         trigger_srv;

  MobileRobot mobile_robot;

  moveit::planning_interface::MoveGroupInterface move_mobile_group("boad");
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  ros::AsyncSpinner spinner(1);

  spinner.start();

  search_product_client  = nh.serviceClient<fcsc_msgs::DetectProduct>("detect_product");
  goto_stocking_base_pos_client  = nh.serviceClient<std_srvs::Trigger>("goto_stocking_base_position");
  detect_object_client  = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
  pickup_product_client = nh.serviceClient<fcsc_msgs::Manipulate>("pickup_product");
  place_product_client  = nh.serviceClient<fcsc_msgs::Manipulate>("place_product");
  return_product_client  = nh.serviceClient<fcsc_msgs::Manipulate>("return_product");

  // goto_stocking_base_pos_client.call(trigger_srv);

  // 台車を一番下まで下げる
  move_mobile_group.setNamedTarget("boad_down");
  move_mobile_group.move();

  // mobile_robot.moveBoard(0.0);
  // mobile_robot.moveBoard(0.1);
  // return (0);

  // ピッキングする物体情報取得
  search_product_client.call(detect_product_srv);

  for (size_t i = 0; i < detect_product_srv.response.detected_product_names.size(); i++) {
    manipulate_srv.request.object = detect_product_srv.response.detected_object_array.objects[i];

    // 上段（おにぎり）のとき再度、台車位置計画
    // static bool plan_base_pos = false;
    // if (!plan_base_pos && manipulate_srv.request.object.type == fcsc_msgs::RecognizedObject::ONIGIRI) {
    //   goto_stocking_base_pos_client.call(trigger_srv);
    //   detect_object_srv.request.detect = true;
    //   detect_object_srv.request.visualize = true;
    //   detect_object_srv.request.damy = true;
    //   detect_object_client.call(detect_object_srv);
    //   plan_base_pos = true;
    // }

    pickup_product_client.call(manipulate_srv);
    if (!manipulate_srv.response.success) {
      continue;
    }

    place_product_client.call(manipulate_srv);
    if (!manipulate_srv.response.success) {
      ROS_WARN("return product");
      manipulate_srv.request.object = detect_product_srv.response.detected_object_array.objects[i];
      return_product_client.call(manipulate_srv);
      continue;
    }

    // detect_object_srv.request.delete_object_names.push_back(product_names[i]);
  }

  // KDEL DEMO
  // move_group.setMaxVelocityScalingFactor(0.2);
  // move_group.setNamedTarget("demo_pickup");
  // move_group.move();

  return 0;
}
