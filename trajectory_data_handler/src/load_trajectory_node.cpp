#include <ros/ros.h>
#include <iostream>
#include <trajectory_data_handler/trajectory_data_handler.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>

int get_file_names(std::string dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    if (std::string(dirp->d_name) == "." || std::string(dirp->d_name) == "..") {
      continue;
    }
    std::cerr << std::string(dirp->d_name) << '\n';
    files.push_back(std::string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_trajectory_node");

  TrajectoryDataHandler handler;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  XmlRpc::XmlRpcValue params;
  std::string trajectory_file_path;
  std::vector<std::string> file_names;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("trajectory_file_list", trajectory_file_path);

  get_file_names(trajectory_file_path, file_names);

  for (size_t i = 0; i < file_names.size(); i++) {
    handler.loadTrajectoryData(trajectory_file_path+file_names[i], plan, true);
  }

  return (0);
}
