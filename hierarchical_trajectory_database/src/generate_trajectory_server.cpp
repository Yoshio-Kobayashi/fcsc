#include <hierarchical_trajectory_database/hierarchical_trajectory_database.h>
#include <hierarchical_trajectory_database/GenerateTrajectory.h>

class HierarchicalTrajectoryDatabaseServer {
private:
  HierarchicalTrajectoryDatabaseHandler database_handler_;
  HierarchicalGraph trajectory_database_;
  ros::ServiceServer generate_trajectory_server_;
  bool generateTrajectory(hierarchical_trajectory_database::GenerateTrajectory::Request &req, hierarchical_trajectory_database::GenerateTrajectory::Response &res);

public:
  HierarchicalTrajectoryDatabaseServer();
  void start();
};

HierarchicalTrajectoryDatabaseServer::HierarchicalTrajectoryDatabaseServer()
{
  ros::NodeHandle nh;

  database_handler_.create(trajectory_database_);

  generate_trajectory_server_ = nh.advertiseService("generate_trajectory", &HierarchicalTrajectoryDatabaseServer::generateTrajectory, this);
}

bool HierarchicalTrajectoryDatabaseServer::generateTrajectory(hierarchical_trajectory_database::GenerateTrajectory::Request &req, hierarchical_trajectory_database::GenerateTrajectory::Response &res)
{
  Point start_state_;
  Point goal_state_;

  start_state_.values = req.start_state;
  goal_state_.values = req.goal_state;

  database_handler_.searchTrajectory(start_state_, goal_state_, trajectory_database_.depth, trajectory_database_, res.trajectory_points);

  return (true);
}

void HierarchicalTrajectoryDatabaseServer::start()
{
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hierarchical_movement_database_server");

  HierarchicalTrajectoryDatabaseServer server;

  server.start();

  return (0);
}
