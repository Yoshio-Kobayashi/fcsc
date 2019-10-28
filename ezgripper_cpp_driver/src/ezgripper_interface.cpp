#include <ros/ros.h>
#include <iostream>
#include "ezgripper_cpp_driver/ezgripper_interface.h"

using namespace std;

EZGripper::EZGripper(std::string name):grip_name(name),is_connected_gripper_action(true)
{
  grip_max = 100.0;
  grip_min = 0.01;
  grip_value = grip_max;
  grip_step = grip_max / 30.0;
  gripper_client = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(grip_name, true);
  bool success = gripper_client->waitForServer( ros::Duration(1.0) );
  if (!success) {
    ROS_ERROR("EZGripper:Not Connected [%s] Server!", grip_name.c_str());
    is_connected_gripper_action = false;
  }
  calibrate_client = nh.serviceClient<std_srvs::Empty>(grip_name+"/calibrate");
}

EZGripper::~EZGripper()
{
  delete gripper_client;
}

void EZGripper::calibrate()
{
  std_srvs::Empty srv;

  calibrate_client.call(srv);
  grip_value = grip_max;
}

void EZGripper::openStep()
{
  control_msgs::GripperCommandGoal goal;

  grip_value -= grip_step;
  if (grip_value < grip_min) {
    grip_value = grip_min;
  }
  goal.command.position = grip_value;
  goal.command.max_effort = 50.0;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
}

void EZGripper::closeStep()
{
  control_msgs::GripperCommandGoal goal;

  grip_value += grip_step;
  if (grip_value > grip_max) {
    grip_value = grip_max;
  }

  goal.command.position = grip_value;
  goal.command.max_effort = 50.0;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
}

void EZGripper::close(double max_effort)
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = 100.0;
  // goal.command.position = 0.0;
  goal.command.max_effort = max_effort;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  grip_value = grip_min;
}

void EZGripper::hardClose()
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = 100.0;
  goal.command.max_effort = 50;
  // goal.command.max_effort = 100;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  grip_value = grip_min;
}

void EZGripper::softClose()
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = 100.0;
  goal.command.max_effort = 20;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  grip_value = grip_min;
}

void EZGripper::open()
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = 0;
  // goal.command.position = 100.0;
  goal.command.max_effort = 100;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  grip_value = grip_max;
}

void EZGripper::goToPosition(double grip_position, double grip_effort)
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = grip_position;
  goal.command.max_effort = grip_effort;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_WARN("Action finished: %s",state.toString().c_str());
  } else {
    ROS_ERROR("Action did not finish before the time out.");
  }
  grip_value = grip_position;
}

void EZGripper::release(void)
{
  control_msgs::GripperCommandGoal goal;

  goal.command.position = 0;
  goal.command.max_effort = 0;
  gripper_client->sendGoal(goal);
  bool finished_before_timeout =  gripper_client->waitForResult(ros::Duration(5.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_client->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  grip_value = grip_min;
}

bool EZGripper::isConnectedGripperAction(void)
{
  return (is_connected_gripper_action);
}
