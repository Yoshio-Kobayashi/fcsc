#ifndef EZGRIPPER_INTERFACE
#define EZGRIPPER_INTERFACE

#include <ros/ros.h>
#include <iostream>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;

class EZGripper {
private:
  // void connectToGripperAction(void);
  // void connectToCalibrateSrv(void);

  ros::NodeHandle nh;
  ros::ServiceClient calibrate_client;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripper_client;
  std::string grip_name;
  double grip_max;
  double grip_min;
  double grip_value;
  double grip_step;
  bool is_connected_gripper_action;

public:
  EZGripper(std::string name);
  ~EZGripper();
  void calibrate(void);
  void open(void);
  void openStep();
  void closeStep();
  void close(double max_effort);
  void hardClose(void);
  void softClose(void);
  void goToPosition(double grip_position=5.0, double grip_effort=20.0);
  void release();
  bool isConnectedGripperAction(void);
};
#endif
