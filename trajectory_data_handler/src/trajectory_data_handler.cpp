#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <fcsc_msgs/RecognizedObject.h>
#include "trajectory_data_handler/trajectory_data_handler.h"

TrajectoryDataHandler::TrajectoryDataHandler()
{
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("base_footprint", "/moveit_visual_tools"));
  // visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("map", "/visualization_marker"));
  visual_tools->setPlanningSceneTopic("/move_group/monitored_planning_scene");
  visual_tools->loadPlanningSceneMonitor();
  visual_tools->loadMarkerPub(true);
  // visual_tools->loadRobotStatePub("display_robot_state");
  // visual_tools->loadTrajectoryPub();
  // visual_tools->setManualSceneUpdating();
  visual_tools->loadSharedRobotState();
  visual_tools->enableBatchPublishing();
}

void TrajectoryDataHandler::saveTrajectoryToBag(moveit::planning_interface::MoveGroupInterface::Plan plan, std::string file_name)
{
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  std::stringstream file_name_stream;

  file_name_stream << file_name << "_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".bag";

  rosbag::Bag bag;
  bag.open(file_name_stream.str(), rosbag::bagmode::Write);

  std_msgs::Float64 planning_time;
  planning_time.data = plan.planning_time_;

  bag.write("planning_time", ros::Time::now(), planning_time);
  bag.write("start_state", ros::Time::now(), plan.start_state_);
  bag.write("trajectory", ros::Time::now(), plan.trajectory_);

  bag.close();
}

void TrajectoryDataHandler::loadTrajectoryData(const std::string& file_name, moveit::planning_interface::MoveGroupInterface::Plan &plan, bool visualize)
{
  rosbag::Bag bag;
  ROS_INFO("[loadTrajectoryData] open [%s]", file_name.c_str());
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("planning_time");
  topics.push_back("start_state");
  topics.push_back("trajectory");

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std_msgs::Float64::Ptr planning_time;
  moveit_msgs::RobotState::Ptr robot_state;
  moveit_msgs::RobotTrajectory::Ptr trajectory;

  for (rosbag::MessageInstance const m: rosbag::View(bag, rosbag::TopicQuery(topics)))
  {
    planning_time = m.instantiate<std_msgs::Float64>();
    if (planning_time != NULL) {
      plan.planning_time_ = planning_time->data;
    }

    robot_state = m.instantiate<moveit_msgs::RobotState>();
    if (robot_state != NULL) {
      plan.start_state_ = *robot_state;
    }

    trajectory = m.instantiate<moveit_msgs::RobotTrajectory>();
    if (trajectory != NULL) {
      plan.trajectory_ = *trajectory;
    }
  }

  bag.close();

  if (visualize) {
    // visual_tools->publishTrajectoryPath(plan.trajectory_, visual_tools->getSharedRobotState(), true);
    visual_tools->publishTrajectoryLine(plan.trajectory_, visual_tools->getRobotModel()->getJointModelGroup("manipulator"));
    visual_tools->trigger();
  }
}
