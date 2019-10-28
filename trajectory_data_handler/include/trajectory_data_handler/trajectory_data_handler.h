#ifndef TRAJECTORY_DATA_HANDLER_H_
#define TRAJECTORY_DATA_HANDLER_H_

#include <iostream>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

const int PLACE = 0;
const int FROM_CONTAINER_TO_SHELF = 1;

class TrajectoryDataHandler {
public:
  TrajectoryDataHandler();
  void saveTrajectoryToBag(moveit::planning_interface::MoveGroupInterface::Plan plan, std::string file_name);
  void loadTrajectoryData(const std::string& file_name, moveit::planning_interface::MoveGroupInterface::Plan &plan, bool visualize=false);
private:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
};
#endif
