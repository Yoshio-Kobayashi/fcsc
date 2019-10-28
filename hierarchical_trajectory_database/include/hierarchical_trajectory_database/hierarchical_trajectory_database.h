#ifndef HIERARCHICAL_TRAJECTORY_DATABASE_
#define HIERARCHICAL_TRAJECTORY_DATABASE_
#include <ros/ros.h>
#include <iostream>
#include <XmlRpcException.h>
#include <dmp/DMPTraj.h>
#include "hierarchical_trajectory_database/graph.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// ２分木の各深さで遷移グラフを作成する
// 遷移確率 を計算
// クラスk -> クラスl
// n_k:クラスk内にある状態ベクトルの数
// m_kl:クラスkからクラスｌへ遷移できる状態ベクトルの数
// double p = m_kl / n_k;

using namespace moveit::planning_interface;

class HierarchicalTrajectoryDatabaseHandler {
private:
  double thresh_;
  double dt_;
  Point start_state_;
  Point end_state_;

  moveit::planning_interface::MoveGroupInterface move_group;

  void stop();

  void showHierarchicalGraph(const HierarchicalGraph& hierarchical_graph);

  void loadTrajectorys(std::vector<std::string> file_names, std::vector<MoveGroupInterface::Plan>& trajectorys);

  // 状態行列でルートノードを作成する
  void createRootNode(std::vector<MoveGroupInterface::Plan> trajectorys, Node& node, std::vector<Point>& states);

  // ノードからグラフを作成
  void createHierarchicalGraph(std::vector<Node>& parent_nodes, int depth, HierarchicalGraph& hierarchical_graph);

  void computeMeanState(const std::vector<Point>& states, Point& mean_state);

  double computeNodeScore(const Node& node, const std::vector<Point>& states);

  void computeEdgeWeight(Graph& graph, const std::vector<Point>& states);

  double computeDistance(const Point& p1, const Point& p2);

  int findNearestNode(const std::vector<Node>& nodes, const Point& search_state);

  void convertPathsToTrajectory(const std::vector<Path>& paths, const Graph& graph, const std::vector<Point>& states, std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points);

  void convertPathsToTrajectory(const std::vector<Path>& paths, const Graph& graph, const std::vector<Point>& states, dmp::DMPTraj& trajectory);

  int countTrajectoryType(const Node& node, const std::vector<Point>& states);

  double computeDuration(const Node& node, const std::vector<Point>& states);

  void showPaths(const std::vector<Path> paths);

  void showPath(const Path& path);

  void showNodes(const std::vector<Node>& nodes);

  void showEdges(Graph& graph);

  void showWeights(const Graph& graph);

  void showNodeIds(const std::vector<int>& node_ids);

  void findPartialPath(const Node& start_node, const Node& end_node, const int desired_level, HierarchicalGraph& hierarchical_graph, std::vector<Path>& partial_paths, bool check_collision=false, moveit::core::RobotState* robot_state=NULL, robot_state::JointModelGroup* joint_model_group=NULL, planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor=NULL);

public:
  HierarchicalTrajectoryDatabaseHandler();

  void create(HierarchicalGraph& hierarchical_graph);

  void searchTrajectory(const Point& start_state, const Point& end_state, int level, HierarchicalGraph& hierarchical_graph, std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory, bool check_collision=false, moveit::core::RobotState* robot_state=NULL, robot_state::JointModelGroup* joint_model_group=NULL, planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor=NULL);
};
#endif
