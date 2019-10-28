#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include "ezgripper_cpp_driver/ezgripper_interface.h"

using namespace std;

class EZGripperJoy {
private:
  ros::NodeHandle nh;
  EZGripper *ezgripper_left;
  EZGripper *ezgripper_right;
  ros::Subscriber joy_sub;
  ros::Time last_command_end_time;
  void joyCallBack(const sensor_msgs::Joy joy);


public:
  EZGripperJoy(std::vector<std::string> gripper_names);
  ~EZGripperJoy();
  void run();
};

EZGripperJoy::EZGripperJoy(std::vector<std::string> gripper_names)
{
  joy_sub = nh.subscribe("joy", 1, &EZGripperJoy::joyCallBack, this);
  ezgripper_left = new EZGripper(gripper_names[0]);
  if (gripper_names.size() > 1) {
    ezgripper_right = new EZGripper(gripper_names[1]);
  } else {
    ezgripper_right = NULL;
  }
  last_command_end_time = ros::Time::now();
}

EZGripperJoy::~EZGripperJoy()
{
  delete ezgripper_left;
  delete ezgripper_right;
}

void EZGripperJoy::joyCallBack(const sensor_msgs::Joy joy)
{
  EZGripper* gripper;

  if (joy.buttons.size() == 0) {
    return;
  }
  if (joy.buttons[5] == 1 && ezgripper_right != NULL) {
    gripper = ezgripper_right;
  } else {
    gripper = ezgripper_left;
  }
  if ( (ros::Time::now() - last_command_end_time).toSec() > 0.2 ) {
    if (joy.buttons[0] == 1) {// A
      gripper->hardClose();
      last_command_end_time = ros::Time::now();
    }
    if (joy.buttons[3] == 1) {// Y
      gripper->softClose();
      last_command_end_time = ros::Time::now();
    }
    if (joy.buttons[1] == 1) {// B
      gripper->open();
      last_command_end_time = ros::Time::now();
    }
    if (joy.buttons[2] == 1) {// X
      gripper->release();
      last_command_end_time = ros::Time::now();
    }
    if (joy.buttons[6] == 1) {// BACK
      gripper->calibrate();
      last_command_end_time = ros::Time::now();
    }
    // if (joy.axes[7] == 1) {//十字キーの上
    //   gripper->openStep();
    //   last_command_end_time = ros::Time::now();
    // }
    // if (joy.axes[7] == -1) {//十字キーの下
    //   gripper->closeStep();
    //   last_command_end_time = ros::Time::now();
    // }
  }
}

void EZGripperJoy::run()
{
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "ezgripper_test");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  std::vector<std::string> gripper_names;

  private_nh.getParam("grippers", gripper_names);

  EZGripperJoy ezgripper_joy(gripper_names);

  ezgripper_joy.run();

  return 0;
}
