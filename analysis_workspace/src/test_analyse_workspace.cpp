#include <ros/ros.h>
#include <iostream>
#include <analysis_workspace/analyse_workspace.h>

using namespace std;

class TestWorkspaceAnalysis
{
public:
  TestWorkspaceAnalysis();
  void analyseShelfWorkspaceAll();
  void analyseShelfWorkspaceOneByOne();
  void analyseContainerWorkspace();

private:
  WorkspaceAnalyzer                       workspace_analyzer;
  moveit_msgs::WorkspaceParameters        task_space;
  std::vector<geometry_msgs::Quaternion>  orientation;
  std::stringstream                       ss;
  std::stringstream                       shelf_board_s;
  std::stringstream                       image_name;
  std::stringstream                       system_command;
  std::string                             attach_type;
  moveit_msgs::Constraints                constraint;
  moveit_msgs::OrientationConstraint      orientation_constraint;
  moveit_msgs::JointConstraint            joint_constraint;
  geometry_msgs::PoseStamped              init_pose;
  std::vector<std::string>                target_names;
  double shelf_width;
  double shelf_depth;
  double shelf_height;
  std::vector<double> installation_heights;
};

TestWorkspaceAnalysis::TestWorkspaceAnalysis():
  workspace_analyzer("manipulator_wrap"),
  // attach_type("masita")
  attach_type("mayoko")
  // attach_type("naname")
{
  ros::NodeHandle                         nh;
  XmlRpc::XmlRpcValue                     params;

  orientation.resize(1);

  target_names.push_back("mayoko");
  target_names.push_back("naname");
  target_names.push_back("masita");

  nh.getParam("shelf_size/shelfA", params);
  shelf_width = (double)params["width"];
  shelf_depth = (double)params["depth"];
  shelf_height = (double)params["height"];
  for (size_t i = 0; i < params["installation_heights"].size(); i++) {
    installation_heights.push_back((double)params["installation_heights"][i]);
  }
}

void TestWorkspaceAnalysis::analyseShelfWorkspaceOneByOne()
{
  task_space.min_corner.x = 0;
  task_space.min_corner.y = 0;
  task_space.min_corner.z = 0;
  task_space.max_corner.x = shelf_depth;
  task_space.max_corner.y = shelf_width;

  for (size_t i = 0; i < installation_heights.size(); i++) {
    if (i == installation_heights.size() - 1) {
      task_space.max_corner.z = 0.5;
    } else {
      task_space.max_corner.z = installation_heights[i+1] - installation_heights[i];
    }

    for (size_t j = 0; j < 3; j++) {
      // 棚の領域設定
      orientation[0] = tf::createQuaternionMsgFromRollPitchYaw(0, j  * M_PI / 4, 0);
      shelf_board_s.str("");
      ss.str("");
      image_name.str("");
      shelf_board_s << "shelfA_board_" << (i + 1);
      ss << attach_type << "_" << "ShelfBoard" << (i + 1) <<  "_" << j * 180 / 4;
      image_name << attach_type << "_" << "shelf_" << target_names[j];
      task_space.header.frame_id = shelf_board_s.str();

      // 制約設定
      constraint.orientation_constraints.clear();
      orientation_constraint.header.frame_id = shelf_board_s.str();
      orientation_constraint.orientation = orientation[0];
      orientation_constraint.link_name = "gripper_link";
      orientation_constraint.absolute_x_axis_tolerance = M_PI / 4;
      orientation_constraint.absolute_y_axis_tolerance = M_PI / 4;
      orientation_constraint.absolute_z_axis_tolerance = M_PI / 4;
      orientation_constraint.weight = 1.0;
      constraint.orientation_constraints.push_back(orientation_constraint);

      workspace_analyzer.analyseWorkspaceFromTaskSpace(task_space, orientation);
      workspace_analyzer.saveWorkspaceDataToDAT(ss.str());
      workspace_analyzer.saveAnalysisResultToDAT(ss.str());
      // system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
      // system(system_command.str().c_str());

      // ss << "_trajectory";
      // image_name << "_trajectory";
      // workspace_analyzer.analyseWorkspaceFromTrajectory(task_space, target_names[j], orientation[0], constraint);
      // system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
      // system(system_command.str().c_str());
      // workspace_analyzer.saveWorkspaceDataToDAT(ss.str());
      // workspace_analyzer.saveAnalysisResultToDAT(ss.str());
    }

  }
}

void TestWorkspaceAnalysis::analyseShelfWorkspaceAll()
{
  task_space.min_corner.x = 0;
  task_space.min_corner.y = 0;
  task_space.min_corner.z = 0;
  task_space.max_corner.x = shelf_depth;
  task_space.max_corner.y = shelf_width;
  task_space.max_corner.z = shelf_height - installation_heights[0];
  task_space.header.frame_id = "shelf_board_1";

  for (size_t j = 0; j < 3; j++) {
    // 棚の領域設定
    orientation[0] = tf::createQuaternionMsgFromRollPitchYaw(0, j  * M_PI / 4, 0);
    ss.str("");
    image_name.str("");
    ss << attach_type << "_" << "Shelf" <<  "_" << j * 180 / 4;
    image_name << attach_type << "_" << "shelf_" << target_names[j];

    // 制約設定
    constraint.orientation_constraints.clear();
    orientation_constraint.header.frame_id = task_space.header.frame_id;
    orientation_constraint.orientation = orientation[0];
    orientation_constraint.link_name = "gripper_link";
    orientation_constraint.absolute_x_axis_tolerance = M_PI / 4;
    orientation_constraint.absolute_y_axis_tolerance = M_PI / 4;
    orientation_constraint.absolute_z_axis_tolerance = M_PI / 4;
    orientation_constraint.weight = 1.0;
    constraint.orientation_constraints.push_back(orientation_constraint);

    workspace_analyzer.analyseWorkspaceFromTaskSpace(task_space, orientation);
    system_command.str("");
    system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
    system(system_command.str().c_str());
    workspace_analyzer.saveWorkspaceDataToDAT(ss.str());
    workspace_analyzer.saveAnalysisResultToDAT(ss.str());

    ss << "_trajectory";
    image_name << "_trajectory";
    workspace_analyzer.analyseWorkspaceFromTrajectory(task_space, target_names[j], orientation[0], constraint);
    system_command.str("");
    system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
    system(system_command.str().c_str());
    workspace_analyzer.saveWorkspaceDataToDAT(ss.str());
    workspace_analyzer.saveAnalysisResultToDAT(ss.str());
  }

}

void TestWorkspaceAnalysis::analyseContainerWorkspace()
{
  // コンテナの領域設定
  task_space.header.frame_id = "container";
  task_space.min_corner.x = -0.2;
  task_space.min_corner.y = -0.3;
  task_space.min_corner.z = 0;
  task_space.max_corner.x = 0.2;
  task_space.max_corner.y = 0.3;
  task_space.max_corner.z = 0.13 * 3;
  orientation[0] = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2, -M_PI);
  ss.str("");
  ss << attach_type << "_container";

  // 制約設定
  constraint.orientation_constraints.clear();
  joint_constraint.joint_name = "elbow_joint";
  joint_constraint.position = -M_PI / 2.0;
  joint_constraint.tolerance_above = M_PI / 2.0;
  joint_constraint.tolerance_below = M_PI / 2.0;
  joint_constraint.weight = 1.0;
  constraint.joint_constraints.push_back(joint_constraint);

  workspace_analyzer.analyseWorkspaceFromTaskSpace(task_space, orientation);

  image_name.str("");
  system_command.str("");
  // image_name << attach_type << "_container";
  // system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
  // system(system_command.str().c_str());
  workspace_analyzer.saveAnalysisResultToDAT(ss.str());
  workspace_analyzer.saveWorkspaceDataToDAT(ss.str());

  // ss << "_trajectory";
  // image_name << "_trajectory";
  // workspace_analyzer.analyseWorkspaceFromTrajectory(task_space, "masita", orientation[0], constraint, 0.05);
  //
  // system_command.str("");
  // system_command << "gnome-screenshot --window --file " << image_name.str() << ".png";
  // system(system_command.str().c_str());
  // workspace_analyzer.saveAnalysisResultToDAT(ss.str());
  // workspace_analyzer.saveWorkspaceDataToDAT(ss.str());
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "main_analysis_workspace");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TestWorkspaceAnalysis analyzer;

  analyzer.analyseShelfWorkspaceOneByOne();
  // analyzer.analyseContainerWorkspace();
  // analyzer.analyseShelfWorkspaceAll();

  return 0;
}
