#ifndef ROBOT_HAND_H_
#define ROBOT_HAND_H_

#include <ezgripper_cpp_driver/ezgripper_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fcsc_msgs/Grasp.h>

class RobotHand {
public:
  RobotHand(std::string ee_name);
  bool move(double joint_position, double effort=50, int control=fcsc_msgs::Grasp::BOTH);
  std::vector<double> getCurrentJointValues();
  std::string getName() { return (ee_name_); }
  double jointToPos(double joint_value);
  double posToJoint(double joint_position);

private:
  EZGripper top_ezgripper;
  EZGripper bottom_ezgripper;
  EZGripper both_ezgripper;
  moveit::planning_interface::MoveGroupInterface move_group;
  std::string ee_name_;

  double max_joint_value_;
  double min_joint_value_;

  void makePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan, double jpos, double effort, int control);
};
#endif
