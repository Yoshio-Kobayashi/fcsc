#include <ros/ros.h>
#include <iostream>
#include "fcsc_moveit/robot_hand.h"

RobotHand::RobotHand(std::string ee_name):
top_ezgripper("/ezgripper/top"),
bottom_ezgripper("/ezgripper/bottom"),
both_ezgripper("/ezgripper/both"),
ee_name_(ee_name),
move_group(ee_name),
max_joint_value_(1.94),
min_joint_value_(0.0)
{
}

bool RobotHand::move(double joint_position, double effort, int control)
{
  double joint_step = (max_joint_value_ - min_joint_value_) / 100.0;
  double joint_value;
  EZGripper *ezgripper_ptr;

  if (joint_position > 100 || joint_position < 0) {
    ROS_ERROR("Invalid joint_position. joint_position requires [0, 100]");
    return (false);
  }

  double jpos = max_joint_value_ - joint_step * joint_position;

  switch (control) {
    case fcsc_msgs::Grasp::BOTH:    ezgripper_ptr = &both_ezgripper;    break;
    case fcsc_msgs::Grasp::TOP:     ezgripper_ptr = &top_ezgripper;     break;
    case fcsc_msgs::Grasp::BOTTOM:  ezgripper_ptr = &bottom_ezgripper;  break;
  }

  if (ezgripper_ptr->isConnectedGripperAction()) {
    ezgripper_ptr->goToPosition(jpos, effort);
  } else {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    makePlan(plan, jpos, effort, control);
    move_group.execute(plan);
  }

  return true;
}

void RobotHand::makePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan, double jpos, double effort, int control)
{
  plan.trajectory_.joint_trajectory.points.resize(2);
  plan.trajectory_.joint_trajectory.points[0].effort.resize(2);
  plan.trajectory_.joint_trajectory.points[1].effort.resize(2);
  plan.trajectory_.joint_trajectory.points[0].positions.resize(2);
  plan.trajectory_.joint_trajectory.points[1].positions.resize(2);

  plan.trajectory_.joint_trajectory.joint_names.resize(2);
  plan.trajectory_.joint_trajectory.joint_names[0] = "bottom_ezgripper_knuckle_palm_L1_1";
  plan.trajectory_.joint_trajectory.joint_names[1] = "top_ezgripper_knuckle_palm_L1_1";

  for (size_t i = 0; i < 2; i++) {
    plan.trajectory_.joint_trajectory.points[i].effort[0] = effort;
    plan.trajectory_.joint_trajectory.points[i].effort[1] = effort;
  }

  plan.trajectory_.joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
  plan.trajectory_.joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

  plan.planning_time_ = 0.3;

  std::vector<double> current_joint_values;
  current_joint_values.push_back(move_group.getCurrentJointValues()[0]);
  current_joint_values.push_back(move_group.getCurrentJointValues()[1]);
  switch (control) {
    case fcsc_msgs::Grasp::BOTH: {
      // bottom
      plan.trajectory_.joint_trajectory.points[0].positions[0] = current_joint_values[0];
      plan.trajectory_.joint_trajectory.points[1].positions[0] = jpos;
      // top
      plan.trajectory_.joint_trajectory.points[0].positions[1] = current_joint_values[1];
      plan.trajectory_.joint_trajectory.points[1].positions[1] = jpos;
      break;
    }
    case fcsc_msgs::Grasp::TOP: {
      // bottom
      plan.trajectory_.joint_trajectory.points[0].positions[0] = current_joint_values[0];
      plan.trajectory_.joint_trajectory.points[1].positions[0] = current_joint_values[0];
      // top
      plan.trajectory_.joint_trajectory.points[0].positions[1] = current_joint_values[1];
      plan.trajectory_.joint_trajectory.points[1].positions[1] = jpos;
      break;
    }
    case fcsc_msgs::Grasp::BOTTOM: {
      // bottom
      plan.trajectory_.joint_trajectory.points[0].positions[0] = current_joint_values[0];
      plan.trajectory_.joint_trajectory.points[1].positions[0] = jpos;
      // top
      plan.trajectory_.joint_trajectory.points[0].positions[1] = current_joint_values[1];
      plan.trajectory_.joint_trajectory.points[1].positions[1] = current_joint_values[1];
      break;
    }
  }
}

std::vector<double> RobotHand::getCurrentJointValues()
{
  return (move_group.getCurrentJointValues());
}

double RobotHand::jointToPos(double joint_value)
{
  double joint_step = 100.0 / (max_joint_value_ - min_joint_value_);
  double joint_pos = joint_step * (max_joint_value_ - joint_value);
  return joint_pos;
}

double RobotHand::posToJoint(double joint_position)
{
  double joint_step = (max_joint_value_ - min_joint_value_) / 100.0;

  if (joint_position > 100) {
    joint_position = 100;
  } else if (joint_position < 0) {
    joint_position = 0;
  }

  double jpos = max_joint_value_ - joint_step * joint_position;
  return (jpos);
}
