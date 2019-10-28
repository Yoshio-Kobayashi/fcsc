//ワークスペース解析・表示関数
//moveit_msgs::WorkspaceParametersで解析したい領域（直方体）を指定
//その領域内で逐次IKを解いていき、解の有無を確認する
//手先の姿勢も設定する
//必要なパラメータ
//基準座標系でのバウンディングボックス
//基準座標系での手先姿勢
//離散化の粒度[m]
//ワークスペース描画用の名前
//MoveGroupの名前

#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <stdio.h>
#include <typeinfo>
#include <time.h>
#include <fstream>
#include <sstream>

// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// MoveIt!
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/WorkspaceParameters.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

#include <visualization_msgs/Marker.h>

#include <eigen_conversions/eigen_msg.h>

using namespace std;

static const std::string ROBOT_DESCRIPTION="robot_description";

const int RESULT_X = 0;
const int RESULT_Y = 1;
const int RESULT_Z = 2;

struct AnalysisResult
{
  double success_rate;//成功率
  double variance[3];
  double mean[3];
  double mean_manip;//可操作度の平均
};

class WorkspaceAnalyzer {
  public:
    WorkspaceAnalyzer(string group_name);

    void analyseWorkspaceFromTaskSpace(moveit_msgs::WorkspaceParameters task_space, std::vector<geometry_msgs::Quaternion> orientations, double step=0.01);
    void analyseWorkspaceFromJointSpace(int particle=10);
    void analyseWorkspaceFromTrajectory(moveit_msgs::WorkspaceParameters task_space, std::string target_name, geometry_msgs::Quaternion orientation, moveit_msgs::Constraints constraint, double step=0.1);

    void saveWorkspaceDataToDAT(std::string data_name);
    void saveAnalysisResultToDAT(std::string result_name);
    void saveWorkspaceDataToYAML(std::string result_name);

  private:
    ros::Publisher      marker_pub;

    robot_model_loader::RobotModelLoader            robot_model_loader;
    robot_model::RobotModelPtr                      kinematic_model;
    robot_state::JointModelGroup*                   joint_model_group;
    planning_scene_monitor::PlanningSceneMonitor    planning_scene_monitor;
    robot_state::RobotStatePtr                      robot_state;
    planning_scene::PlanningScenePtr                planning_scene;
    moveit::planning_interface::MoveGroupInterface  move_group;

    tf::TransformListener             listener;
    geometry_msgs::Vector3            rpy_msg;
    string                            group_name;
    vector<string>                    joint_names;
    std_msgs::ColorRGBA               green;
    visualization_msgs::Marker        marker_points;
    std::vector<geometry_msgs::Pose>  analysis_poses;
    AnalysisResult                    analysis_result;

    int     ik_attempts;
    double  ik_timeout;

    void setParameters();
    void transformPose(string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped &ps_out);
};
