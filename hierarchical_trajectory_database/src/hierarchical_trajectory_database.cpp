#include <ros/ros.h>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <XmlRpcException.h>
#include <trajectory_data_handler/trajectory_data_handler.h>
#include <dmp/dmp.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/GetDMPPlan.h>
#include "hierarchical_trajectory_database/kmeans.h"
#include "hierarchical_trajectory_database/graph.h"
#include "hierarchical_trajectory_database/my_astar_search.h"
// #include "hierarchical_trajectory_database/astar_search.h"
#include <hierarchical_trajectory_database/hierarchical_trajectory_database.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

/*function... might want it in some class?*/
int get_file_names(string dir, vector<string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    if (string(dirp->d_name) == "." || string(dirp->d_name) == "..") {
      continue;
    }
    files.push_back(string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}

// ２分木の各深さで遷移グラフを作成する
// 遷移確率 を計算
// クラスk -> クラスl
// n_k:クラスk内にある状態ベクトルの数
// m_kl:クラスkからクラスｌへ遷移できる状態ベクトルの数
// double p = m_kl / n_k;

using namespace moveit::planning_interface;

HierarchicalTrajectoryDatabaseHandler::HierarchicalTrajectoryDatabaseHandler():
move_group("manipulator")
{
}

void HierarchicalTrajectoryDatabaseHandler::create(HierarchicalGraph& hierarchical_graph)
{
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue params;

  std::vector<std::string> file_names;
  std::string trajectory_file_path;
  std::vector<MoveGroupInterface::Plan> trajectory_plans;

  private_nh.getParam("trajectory_file_path", trajectory_file_path);
  get_file_names(trajectory_file_path, file_names);

  for (size_t i = 0; i < file_names.size(); i++)
    file_names[i] = trajectory_file_path + file_names[i];

  hierarchical_graph.graphs.resize(1);
  hierarchical_graph.graphs[0].nodes.resize(1);
  hierarchical_graph.graphs[0].depth = 1;
  hierarchical_graph.depth = 1;

  std::cout << "loadTrajectorys" << '\n';
  loadTrajectorys(file_names, trajectory_plans);
  std::cout << '\n';

  std::cout << "createRootNode" << '\n';
  createRootNode(trajectory_plans, hierarchical_graph.graphs[0].nodes[0], hierarchical_graph.states);
  std::cout << '\n';

  std::cout << "createHierarchicalGraph" << '\n';
  thresh_ = 0.01;
  createHierarchicalGraph(hierarchical_graph.graphs[0].nodes, /* depth */2, hierarchical_graph);
  std::cout << '\n';

  std::cout << "showHierarchicalGraph" << '\n';
  // showHierarchicalGraph(hierarchical_graph);

  std::cout << "num of states:"<<  hierarchical_graph.states.size() << '\n';
  std::cout << "depth:" << hierarchical_graph.graphs.size() << '\n';
  std::cout << "num of nodes at deepest:" << hierarchical_graph.graphs.back().nodes.size() << '\n';
  // stop();

  // dt を計算
  double total_time = 0.0;
  double num_of_points = trajectory_plans[0].trajectory_.joint_trajectory.points.size();
  for (size_t i = 1; i < num_of_points; i++) {
    double t1 = trajectory_plans[0].trajectory_.joint_trajectory.points[i - 1].time_from_start.toSec();
    double t2 = trajectory_plans[0].trajectory_.joint_trajectory.points[i].time_from_start.toSec();
    total_time += (t2 - t1);
  }
  dt_ = total_time / (num_of_points - 1);
}

void HierarchicalTrajectoryDatabaseHandler::showHierarchicalGraph(const HierarchicalGraph& hierarchical_graph)
{
  std::cout << hierarchical_graph.states.size() << '\n';
  for (size_t i = 0; i < hierarchical_graph.graphs.size(); i++) {
    std::cout << "depth:" << hierarchical_graph.graphs[i].depth << '\n';
    const Graph* graph = &(hierarchical_graph.graphs[i]);
    for (size_t j = 0; j < graph->nodes.size(); j++) {
      const Node* node = &(graph->nodes[j]);
      std::cout << "(" << node->id << ",";
      std::cout << node->parent_id << ",";
      if (node->child_ids.size() > 0) {
        std::cout << "[";
        for (size_t k = 0; k < node->child_ids.size(); k++) {
          std::cout << node->child_ids[k] << ",";
        }
        std::cout << "]";
      }
      std::cout << ") ";
    }
    std::cout << '\n';
  }
}

void HierarchicalTrajectoryDatabaseHandler::loadTrajectorys(std::vector<std::string> file_names, std::vector<MoveGroupInterface::Plan>& trajectorys)
{
  TrajectoryDataHandler handler;

  trajectorys.resize(file_names.size());
  for (size_t i = 0; i < file_names.size(); i++) {
    std::cout << "file:" << file_names[i] << '\n';
    handler.loadTrajectoryData(file_names[i], trajectorys[i], /*visualize*/false);
  }
}

// 状態行列でルートノードを作成する
void HierarchicalTrajectoryDatabaseHandler::createRootNode(std::vector<MoveGroupInterface::Plan> trajectorys, Node& node, std::vector<Point>& states)
{
  int trajectory_size = trajectorys.size();
  int sum_point_size = 0;
  for (size_t i_traj = 0; i_traj < trajectory_size; i_traj++) {
    // 軌道を読み込む
    // 読み込んだ軌道を分解して状態(Point)を作成する
    int trajectory_point_size = trajectorys[i_traj].trajectory_.joint_trajectory.points.size();
    for (size_t j_p = 0; j_p < trajectory_point_size; j_p++) {
      trajectory_msgs::JointTrajectoryPoint traj_point = trajectorys[i_traj].trajectory_.joint_trajectory.points[j_p];
      Point state;
      state.id_point = (int)(j_p + sum_point_size);
      state.id_trajectory = (int)i_traj;
      state.id_cluster = -1;
      state.is_end = false;
      state.values.insert(state.values.end(), traj_point.positions.begin(), traj_point.positions.end());
      state.values.insert(state.values.end(), traj_point.velocities.begin(), traj_point.velocities.end());
      // 作成した状態をNodeに突っ込む
      states.push_back(state);
      node.state_ids.push_back(state.id_point);
    }
    sum_point_size += trajectory_point_size;
    states.back().is_end = true;
  }
  computeMeanState(states, node.mean_state);
}

// ノードからグラフを作成
void HierarchicalTrajectoryDatabaseHandler::createHierarchicalGraph(std::vector<Node>& parent_nodes, int depth, HierarchicalGraph& hierarchical_graph)
{
  int node_size = parent_nodes.size();
  std::vector<int> valid_node_indices;
  std::vector<int> invalid_node_indices;
  int node_id = 0;
  Graph graph;

  // std::cout << "depth:" << depth << '\n';

  // 全ノードのスコアを計算
  // std::cout << "Compute all node score" << '\n';
  for (size_t i = 0; i < node_size; i++) {
    double d = computeNodeScore(parent_nodes[i], hierarchical_graph.states);
    if (d > thresh_) {
      valid_node_indices.push_back(i);
    } else {
      invalid_node_indices.push_back(i);
    }
  }

  // クラスタリングするかしないかを決める
  // すべてのノードが閾値以下ならグラフ作成を終了
  // std::cout << "Check thresh" << '\n';
  if (invalid_node_indices.size() == node_size) {
    return;
  }

  // 閾値より大きいスコアをもつノードをクラスタリングする
  // std::cout << "Clustering" << '\n';
  int valid_node_size = valid_node_indices.size();
  for (size_t i_n = 0; i_n < valid_node_size; i_n++) {
    // ノードを２クラスにクラスタリングしてノードを作成する
    std::vector<Node> clustered_nodes(2);
    Node* parent_node = &(parent_nodes[ valid_node_indices[i_n] ]);
    KMeans kmeans(2, parent_node->state_ids.size(), 12, 100);
    // std::cout << "K-means Run:" << parent_node->state_ids.size() << '\n';
    std::vector<Point> parent_states(parent_node->state_ids.size());
    for (size_t j_s = 0; j_s < parent_node->state_ids.size(); j_s++) {
      int id = parent_node->state_ids[j_s];
      parent_states[j_s] = hierarchical_graph.states[id];
      parent_states[j_s].id_cluster = -1;
    }
    kmeans.run(parent_states);

    if (parent_node->state_ids.size() == 1) {
      parent_states[0].id_cluster = 0;
    }

    // 結果のsatesをクラスに分ける
    // std::cout << "Divide Clus" << '\n';
    int state_size = parent_states.size();
    for (size_t j_s = 0; j_s < state_size; j_s++) {
      int id_clus = parent_states[j_s].id_cluster;
      int id_state = parent_states[j_s].id_point;
      clustered_nodes[id_clus].state_ids.push_back(id_state);
    }

    // ノードの平均と親ID、子、IDの設定
    // std::cout << "computeMeanState" << '\n';

    for (size_t j_n = 0; j_n < clustered_nodes.size(); j_n++) {
      // ノードに軌道上の厳密に1つの軌道の最終状態が含まれている場合のみ、位置平均xの代わりにこの最終状態を保存。
      Point* end_state;
      int end_count = 0;
      std::vector<Point> clustered_states(clustered_nodes[j_n].state_ids.size());
      for (size_t k_s = 0; k_s < clustered_nodes[j_n].state_ids.size(); k_s++) {
        int id = clustered_nodes[j_n].state_ids[k_s];
        clustered_states[k_s] = hierarchical_graph.states[id];
        if (hierarchical_graph.states[id].is_end) {
          end_state = &(hierarchical_graph.states[id]);
          ++end_count;
        }
      }
      if (end_count == 1) {
        clustered_nodes[j_n].mean_state = *end_state;
      } else {
        computeMeanState(clustered_states, clustered_nodes[j_n].mean_state);
      }
      clustered_nodes[j_n].parent_id = parent_node->id;
      clustered_nodes[j_n].id = node_id;
      parent_node->child_ids.push_back(node_id);
      graph.nodes.push_back(clustered_nodes[j_n]);
      ++node_id;
    }
  } // i_n

  // 閾値以下のスコアをもつノードをグラフに格納する
  // std::cout << "push invalid node" << '\n';
  int invalid_node_size = invalid_node_indices.size();
  for (size_t i = 0; i < invalid_node_size; i++) {
    int invalid_index = invalid_node_indices[i];
    graph.nodes.push_back(parent_nodes[invalid_index]);
    graph.nodes.back().parent_id = parent_nodes[invalid_index].id;
    graph.nodes.back().id = node_id;
    parent_nodes[invalid_index].child_ids.push_back(node_id);
    ++node_id;
  }

  // グラフ内のノードからエッジ（遷移確率）を計算してグラフに格納する
  // std::cout << "computeEdgeWeight" << '\n';
  computeEdgeWeight(graph, hierarchical_graph.states);

  // グラフの深さ情報をグラフに格納する
  graph.depth = depth;
  hierarchical_graph.depth = depth;

  hierarchical_graph.graphs.push_back(graph);

  // 作成したノードでグラフを作成する
  createHierarchicalGraph(hierarchical_graph.graphs[depth-1].nodes, depth+1, hierarchical_graph);
}

void HierarchicalTrajectoryDatabaseHandler::computeMeanState(const std::vector<Point>& states, Point& mean_state)
{
  int num_of_states = states.size();
  std::vector<double> sum_value(12, 0.0);
  for (size_t i_s = 0; i_s < num_of_states; i_s++) {
    for (size_t j_v = 0; j_v < 12; j_v++) {
      sum_value[j_v] += states[i_s].values[j_v];
    }
  }
  mean_state.values.resize(12);
  for (size_t j_v = 0; j_v < 12; j_v++) {
    mean_state.values[j_v] = sum_value[j_v] / (double)num_of_states;
  }
}

double HierarchicalTrajectoryDatabaseHandler::computeNodeScore(const Node& node, const std::vector<Point>& states)
{
  int num_of_states = node.state_ids.size();
  double sum_distance = 0.0;
  for (size_t i_s = 0; i_s < num_of_states; i_s++) {
    int id = node.state_ids[i_s];
    sum_distance += computeDistance(states[id], node.mean_state);
  }
  return (sum_distance / (double)num_of_states);
}

void HierarchicalTrajectoryDatabaseHandler::computeEdgeWeight(Graph& graph, const std::vector<Point>& states)
{
  double num_of_transitions;

  int num_of_nodes = graph.nodes.size();
  for (size_t i_n = 0; i_n < num_of_nodes; i_n++) {
    int num_of_i_states = graph.nodes[i_n].state_ids.size();
    for (size_t j_n = 0; j_n < num_of_nodes; j_n++) {
      if (j_n == i_n) {
        continue;
      }
      num_of_transitions = 0;
      int num_of_j_states = graph.nodes[j_n].state_ids.size();
      for (size_t i_s = 0; i_s < num_of_i_states; i_s++) {
        for (size_t j_s = 0; j_s < num_of_j_states; j_s++) {
          //状態i と 状態j が同じ軌道である かつ、jがiよりもあとにある
          int id_i = graph.nodes[i_n].state_ids[i_s];
          int id_j = graph.nodes[j_n].state_ids[j_s];
          const Point* state_i = &(states[id_i]);
          const Point* state_j = &(states[id_j]);
          if (state_j->id_trajectory != state_i->id_trajectory) {
            continue;
          }
          if (state_j->id_point != (state_i->id_point + 1)) {
            continue;
          }
          ++num_of_transitions;
          break;
        }
      }
      if (num_of_transitions == 0.0) {
        continue;
      }
      double prob = num_of_transitions / (double)num_of_i_states;
      graph.nodes[i_n].edges[j_n] = 2.0 - prob;
    }
  }
}

double HierarchicalTrajectoryDatabaseHandler::computeDistance(const Point& p1, const Point& p2)
{
  if (p1.values.size() != p2.values.size()) {
    return (-1.0);
  }

  double square_dist = 0.0;
  for (size_t i = 0; i < p1.values.size(); i++) {
    double dif = p1.values[i] - p2.values[i];
    square_dist += dif * dif;
  }
  return (sqrt(square_dist));
}

int HierarchicalTrajectoryDatabaseHandler::findNearestNode(const std::vector<Node>& nodes, const Point& search_state)
{
  // HACK:全探索でやっているので、もっと早いアルゴリズムにする
  double min_dist = -1.0;
  int index;
  for (size_t i = 0; i < nodes.size(); i++) {
    double dist;
    dist = computeDistance(nodes[i].mean_state, search_state);
    if (min_dist < 0 || dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }
  return (index);
}

void HierarchicalTrajectoryDatabaseHandler::searchTrajectory(const Point& start_state, const Point& end_state, int level, HierarchicalGraph& hierarchical_graph, std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory, bool check_collision, moveit::core::RobotState* robot_state, robot_state::JointModelGroup* joint_model_group, planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor)
{
  start_state_  = start_state;
  end_state_    = end_state;

  // 1.開始点start_stateおよび終了点end_stateに最も近いところで終了するstart_nodeおよびend_nodeを見つける。
  int start_node_index  = findNearestNode(hierarchical_graph.graphs[level-1].nodes, start_state);
  int end_node_index    = findNearestNode(hierarchical_graph.graphs[level-1].nodes, end_state);

  // 2.P = FindPartialPaths（start_node、 end_node, level）
  std::vector<Path> partial_paths;
  findPartialPath(hierarchical_graph.graphs[level-1].nodes[start_node_index], hierarchical_graph.graphs[level-1].nodes[end_node_index], level, hierarchical_graph, partial_paths, check_collision, robot_state, joint_model_group, planning_scene_monitor);

  // std::cout << "start_node_index:" << start_node_index << '\n';
  // std::cout << "end_node_index:" << end_node_index << '\n';
  // showPaths(partial_paths);
  // std::cout << '\n';
  // stop();

  // 3.レベルのグラフノードのシーケンスとして特定された発見された部分経路を、状態ベクトルの位置部分の対応する平均値{xi}およびそれらの時間発展Tiに変換する
  // 各ノードのタイムスタンプを計算
  const Graph* graph = &(hierarchical_graph.graphs[level-1]);
  // convertPathsToTrajectory(partial_paths, *graph, trajectory);

  // 速度、加速度の後処理をする
  robot_state::RobotStatePtr robot_state_ptr(move_group.getCurrentState());
  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  robot_trajectory_msg.joint_trajectory.joint_names = move_group.getJointNames();
  convertPathsToTrajectory(partial_paths, *graph, hierarchical_graph.states, robot_trajectory_msg.joint_trajectory.points);
  robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), "manipulator");
  robot_trajectory.setRobotTrajectoryMsg(*robot_state_ptr, robot_trajectory_msg);
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  // trajectory_processing::IterativeSplineParameterization time_parameterization;
  time_parameterization.computeTimeStamps(robot_trajectory);
  robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);


  // 4.位置状態ベクトルの結果のシーケンスをDMPとして符号化し、したがってnTrjを得ることによって、（および結合して）結合する。
  dmp::LearnDMPFromDemo learn_dmp_srv;
  dmp::GetDMPPlan get_plan_srv;
  double k_gain = 100;
  double d_gain = 2.0 * sqrt(k_gain);

  learn_dmp_srv.request.demo.points.resize(robot_trajectory_msg.joint_trajectory.points.size());
  learn_dmp_srv.request.demo.times.resize(robot_trajectory_msg.joint_trajectory.points.size());
  for (size_t i_p = 0; i_p < robot_trajectory_msg.joint_trajectory.points.size(); i_p++) {
    learn_dmp_srv.request.demo.points[i_p].positions = robot_trajectory_msg.joint_trajectory.points[i_p].positions;
    learn_dmp_srv.request.demo.points[i_p].velocities = robot_trajectory_msg.joint_trajectory.points[i_p].velocities;
    learn_dmp_srv.request.demo.times[i_p] = robot_trajectory_msg.joint_trajectory.points[i_p].time_from_start.toSec();
  }

  // convertPathsToTrajectory(partial_paths, *graph, learn_dmp_srv.request.demo);

  // std::cout << "dmp_traj" << '\n';
  // std::cout << learn_dmp_srv.request.demo << '\n';
  // stop();

  learn_dmp_srv.request.k_gains.resize(learn_dmp_srv.request.demo.points[0].positions.size());
  learn_dmp_srv.request.d_gains.resize(learn_dmp_srv.request.demo.points[0].positions.size());
  for (size_t i = 0; i < learn_dmp_srv.request.demo.points[0].positions.size(); i++) {
    learn_dmp_srv.request.k_gains[i] = k_gain;
    learn_dmp_srv.request.d_gains[i] = d_gain;
  }
  learn_dmp_srv.request.num_bases = learn_dmp_srv.request.demo.points.size();

  dmp::learnFromDemo(
    learn_dmp_srv.request.demo,
    learn_dmp_srv.request.k_gains,
    learn_dmp_srv.request.d_gains,
    learn_dmp_srv.request.num_bases,
    learn_dmp_srv.response.dmp_list
  );
  learn_dmp_srv.response.tau = learn_dmp_srv.request.demo.times[learn_dmp_srv.request.demo.times.size()-1];

  get_plan_srv.request.x_0.insert(get_plan_srv.request.x_0.end(), start_state.values.begin(), start_state.values.begin()+6);
  get_plan_srv.request.goal.insert(get_plan_srv.request.goal.end(), end_state.values.begin(), end_state.values.begin()+6);
  get_plan_srv.request.t_0 = 0;
  get_plan_srv.request.seg_length = -1;
  get_plan_srv.request.tau = learn_dmp_srv.response.tau;
  get_plan_srv.request.dt = dt_;
  get_plan_srv.request.integrate_iter = 1;
  for (size_t i = 0; i < get_plan_srv.request.x_0.size(); i++) {
    get_plan_srv.request.x_dot_0.push_back(0.0);
    get_plan_srv.request.goal_thresh.push_back(0.01);
  }

  dmp::generatePlan(
    learn_dmp_srv.response.dmp_list,
    get_plan_srv.request.x_0,
    get_plan_srv.request.x_dot_0,
    get_plan_srv.request.t_0,
    get_plan_srv.request.goal,
    get_plan_srv.request.goal_thresh,
    get_plan_srv.request.seg_length,
    get_plan_srv.request.tau,
    get_plan_srv.request.dt,
    get_plan_srv.request.integrate_iter,
    get_plan_srv.response.plan,
    get_plan_srv.response.at_goal
  );

  // dmp::DMPTraj ==> moveit_msgs::RobotTrajectory
  // std::cout << "moveit traj" << '\n';
  trajectory.resize(get_plan_srv.response.plan.points.size());
  for (size_t i = 0; i < trajectory.size(); i++) {
    trajectory[i].positions = get_plan_srv.response.plan.points[i].positions;
    trajectory[i].velocities = get_plan_srv.response.plan.points[i].velocities;
    trajectory[i].time_from_start = ros::Duration(get_plan_srv.response.plan.times[i]);
    // std::cout << trajectory[i] << '\n';
  }
  // stop();
}

void HierarchicalTrajectoryDatabaseHandler::convertPathsToTrajectory(const std::vector<Path>& paths, const Graph& graph, const std::vector<Point>& states, std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points)
{
  // partial_pathsから軌道データ作成
  std::vector<double> durations;
  for (size_t i_p = 0; i_p < paths.size(); i_p++) {
    for (size_t j_n = 0; j_n < paths[i_p].node_indices.size(); j_n++) {
      int index = paths[i_p].node_indices[j_n];
      double duration = computeDuration(graph.nodes[index], states);
      durations.push_back(duration);
    }
  }

  trajectory_points.resize(durations.size());
  for (size_t i = 0; i < trajectory_points.size(); i++) {
    double sec;
    if (i == 0) {
      sec = 0.0;
    } else {
      sec = (durations[i] + durations[i-1]) / 2.0 + trajectory_points[i-1].time_from_start.toSec();
    }
    trajectory_points[i].time_from_start.fromSec(sec);
  }

  int traj_index = 0;
  for (size_t i_p = 0; i_p < paths.size(); i_p++) {
    for (size_t j_n = 0; j_n < paths[i_p].node_indices.size(); j_n++) {
      int index = paths[i_p].node_indices[j_n];
      const Node* node = &(graph.nodes[index]);
      trajectory_msgs::JointTrajectoryPoint* point = &(trajectory_points[traj_index]);
      point->positions.insert(point->positions.end(), node->mean_state.values.begin(), node->mean_state.values.begin() + 6);
      point->velocities.insert(point->velocities.end(), node->mean_state.values.begin() + 6, node->mean_state.values.end());
      ++traj_index;
    }
  }
}

void HierarchicalTrajectoryDatabaseHandler::convertPathsToTrajectory(const std::vector<Path>& paths, const Graph& graph, const std::vector<Point>& states, dmp::DMPTraj& trajectory)
{
  // partial_pathsから軌道データ作成
  std::vector<double> durations;
  for (size_t i_p = 0; i_p < paths.size(); i_p++) {
    for (size_t j_n = 0; j_n < paths[i_p].node_indices.size(); j_n++) {
      int index = paths[i_p].node_indices[j_n];
      double duration = computeDuration(graph.nodes[index], states);
      durations.push_back(duration);
    }
  }

  trajectory.points.resize(durations.size());
  trajectory.times.resize(durations.size());
  for (size_t i = 0; i < trajectory.times.size(); i++) {
    if (i == 0) {
      trajectory.times[i] = 0.0;
    } else {
      trajectory.times[i] = (durations[i] + durations[i-1]) / 2.0 + trajectory.times[i-1];
    }
  }

  int traj_index = 0;
  for (size_t i_p = 0; i_p < paths.size(); i_p++) {
    for (size_t j_n = 0; j_n < paths[i_p].node_indices.size(); j_n++) {
      int index = paths[i_p].node_indices[j_n];
      const Node* node = &(graph.nodes[index]);
      dmp::DMPPoint* point = &(trajectory.points[traj_index]);
      point->positions.insert(point->positions.end(), node->mean_state.values.begin(), node->mean_state.values.begin() + 6);
      point->velocities.insert(point->velocities.end(), node->mean_state.values.begin() + 6, node->mean_state.values.end());
      ++traj_index;
    }
  }
}

int HierarchicalTrajectoryDatabaseHandler::countTrajectoryType(const Node& node, const std::vector<Point>& states)
{
  std::vector<int> trajectory_types;

  for (size_t i_s = 0; i_s < node.state_ids.size(); i_s++) {
    int id = node.state_ids[i_s];
    auto itr = std::find(trajectory_types.begin(), trajectory_types.end(), states[id].id_trajectory);
    if (itr == trajectory_types.end()) {
      trajectory_types.push_back(states[id].id_trajectory);
    }
  }
  return (trajectory_types.size());
}

double HierarchicalTrajectoryDatabaseHandler::computeDuration(const Node& node, const std::vector<Point>& states)
{
  double num_of_states = node.state_ids.size();
  double num_of_trajectory_types;

  num_of_trajectory_types = (double)countTrajectoryType(node, states);

  double duration = (num_of_states / num_of_trajectory_types) * dt_;

  return (duration);
}

void HierarchicalTrajectoryDatabaseHandler::showPaths(const std::vector<Path> paths)
{
  for (size_t i_p = 0; i_p < paths.size(); i_p++) {
    std::cout << "path[" << i_p << "]:[";
    for (size_t j_n = 0; j_n < paths[i_p].node_indices.size(); j_n++) {
      std::cout << paths[i_p].node_indices[j_n] <<  " ";
    }
    std::cout << "]" << '\n';
  }
}

void HierarchicalTrajectoryDatabaseHandler::stop()
{
  int ans;
  while (1) {
    cerr << "Move? Yes:1 --- ";
    cin >> ans;
    if (!cin.fail() && ans >= 1) {
      break;
    }
    cin.clear();
    cin.ignore(10000, '\n');
  }
}

void HierarchicalTrajectoryDatabaseHandler::showPath(const Path& path)
{
  std::cout << "path:[";
  for (size_t i = 0; i < path.node_indices.size(); i++) {
    std::cout << path.node_indices[i] << " ";
  }
  std::cout << "]" << '\n';
}

void HierarchicalTrajectoryDatabaseHandler::showNodes(const std::vector<Node>& nodes)
{
  std::cout << "nodes:[";
  for (size_t i = 0; i < nodes.size(); i++) {
    std::cout << nodes[i].id  << "  ";
  }
  std::cout << "]" << '\n';
}

void HierarchicalTrajectoryDatabaseHandler::showEdges(Graph& graph)
{
  std::cout << "edges:[";
  for (size_t i_n = 0; i_n < graph.nodes.size(); i_n++) {
    for (auto itr = graph.nodes[i_n].edges.begin(); itr != graph.nodes[i_n].edges.end(); ++itr) {
      std::cout << "(" << i_n << "," << itr->first << ")=" << graph.nodes[i_n].edges[itr->first] << '\n';
    }
  }
  std::cout << "]" << '\n';
}

void HierarchicalTrajectoryDatabaseHandler::showWeights(const Graph& graph)
{
  std::cout << "weights:[";
  for (size_t i_n = 0; i_n < graph.nodes.size(); i_n++) {
    for (auto itr = graph.nodes[i_n].edges.begin(); itr !=  graph.nodes[i_n].edges.end(); ++itr) {
      std::cout << itr->first << " ";
    }
  }
  std::cout << "]" << '\n';
}

void HierarchicalTrajectoryDatabaseHandler::showNodeIds(const std::vector<int>& node_ids)
{
  std::cout << "node ids:[";
  for (size_t i = 0; i < node_ids.size(); i++) {
    std::cout << node_ids[i] << " ";
  }
  std::cout << "]" << '\n';
}

void HierarchicalTrajectoryDatabaseHandler::findPartialPath(const Node& start_node, const Node& end_node, const int desired_level, HierarchicalGraph& hierarchical_graph, std::vector<Path>& partial_paths, bool check_collision, moveit::core::RobotState* robot_state, robot_state::JointModelGroup* joint_model_group, planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor)
{
  int level = desired_level;
  std::vector<int> route_indices;
  Path path;
  path.depth = level;

  // まず desired_level で A*探索 をして経路生成
  // std::cout << "1:astar_search" << '\n';
  // std::cout << "(start, end):" << start_node.id << "," << end_node.id << '\n';
  bool found_path = my_astar::astar_search(hierarchical_graph.graphs[level - 1], start_node.id, end_node.id, path.node_indices, check_collision, robot_state, joint_model_group, planning_scene_monitor);
  // bool found_path = astar_search(hierarchical_graph.graphs[level - 1], start_node.id, end_node.id, path.node_indices);
  // std::cout << "astar end" << '\n';

  int start_node_index;
  int end_node_index;

  // if (found_path) {
  //   showPath(path);
  // }

  // 経路生成が成功するまで上の階層に移動する
  while (!found_path) {
    level = level - 1;
    // std::cout << "level:" << level << '\n';

    const Graph* graph = &(hierarchical_graph.graphs[level - 1]);

    // showNodes(graph->nodes);
    // showEdges(graph->edges);
    // showWeights(graph->edge_weights);

    path.node_indices.clear();
    path.depth = level;

    // start_state, end_stateに近い start_node, end_nodeを見つける
    // std::cout << "find start node" << '\n';
    start_node_index = findNearestNode(hierarchical_graph.graphs[level - 1].nodes, start_state_);
    end_node_index   = findNearestNode(hierarchical_graph.graphs[level - 1].nodes, end_state_);

    // 深さlevelで A*探索 pP1 = P(start_node, end_node)
    // std::cout << "(start, end):" << start_node_index << "," << end_node_index << '\n';
    // std::cout << "2:astar_search" << '\n';
    found_path = my_astar::astar_search(hierarchical_graph.graphs[level - 1], start_node_index, end_node_index, path.node_indices, check_collision, robot_state, joint_model_group, planning_scene_monitor);
    // found_path = astar_search(hierarchical_graph.graphs[level - 1], start_node_index, end_node_index, path.node_indices);
    // std::cout << "astar end" << '\n';
  }
  //stop();
  // std::cout  << '\n';
  partial_paths.push_back(path);

  while (level != desired_level) {
    level = level + 1;
    // std::vector<Path> partial_paths2;
    Graph* parent_graph = &(hierarchical_graph.graphs[level-2]);
    Graph* child_graph  = &(hierarchical_graph.graphs[level-1]);

    std::vector<std::vector<Path> > paths_vector;

    // std::cout << "level:" << level <<  " partial_paths size:" << partial_paths.size()  << " START" << '\n';

    // showEdges(child_graph->edges);
    // showWeights(child_graph->edge_weights);

    start_node_index = findNearestNode(parent_graph->nodes, start_state_);
    end_node_index   = findNearestNode(parent_graph->nodes, end_state_);

    int child_start_node_index = findNearestNode(child_graph->nodes, start_state_);
    int child_end_node_index   = findNearestNode(child_graph->nodes, end_state_);

    // std::cout << "parent_start_node_index:" << start_node_index << '\n';
    // std::cout << "parent_end_node_index:"   << end_node_index << '\n';
    // std::cout << "child start_node_index:"  << child_start_node_index << '\n';
    // std::cout << "child end_node_index:"    << child_end_node_index << '\n';
    //stop();

    for (size_t i_p = 0; i_p < partial_paths.size(); i_p++) {
      std::vector<Path> paths;

      // 親部分パスの境界ノード(パスの先頭、終端ノード)の子ノードを見つける
      Path* partial_path = &(partial_paths[i_p]);

      // std::cout << "[" << i_p << "] partial ";
      // showPath(*partial_path);
      //stop();

      // 親部分パスの先頭ノードの子ノード
      int parent_node_index = partial_path->node_indices.front();

      const std::vector<int>* child_ids;
      std::vector<int> start_node_ids;

      // もし、境界ノードがスタートノードなら、start_stateに近いノードを見つける
      if (parent_node_index == start_node_index) {
        // std::cout << "Find Start Node" << '\n';
        start_node_ids.push_back(child_start_node_index);
      } else {
        // std::cout << "Get Start Child ID" << '\n';
        start_node_ids = parent_graph->nodes[parent_node_index].child_ids;
      }

      // 親部分パスの終端ノードの子ノード
      parent_node_index = partial_path->node_indices.back();
      std::vector<int> end_node_ids;

      // もし、境界ノードがエンドノードなら、end_stateに近いノードを見つける
      if (parent_node_index == end_node_index) {
        // std::cout << "Find End Node" << '\n';
        end_node_ids.push_back(child_end_node_index);
      } else {
        // std::cout << "Get End Child ID" << '\n';
        end_node_ids = parent_graph->nodes[parent_node_index].child_ids;
      }

      // std::cout << "start ";
      // showNodeIds(start_node_ids);
      //
      // std::cout << "end ";
      // showNodeIds(end_node_ids);
      // std::cout << '\n';
      //stop();

      // 部分パスをAstarで探索するときのゴールの設定
      std::vector<int> goal_node_ids;
      for (size_t j_n = 0; j_n < partial_path->node_indices.size(); j_n++) {
        int index = partial_path->node_indices[j_n];
        const Node* parent_node = &(parent_graph->nodes[index]);
        for (size_t k_n = 0; k_n < parent_node->child_ids.size(); k_n++) {
          int goal_node_id  = parent_node->child_ids[k_n];
          goal_node_ids.push_back(goal_node_id);
        }// l_n
      }// k_n
      if (end_node_ids[0] == child_end_node_index) {
        auto itr = std::find(goal_node_ids.begin(), goal_node_ids.end(), child_end_node_index);
        if (itr == goal_node_ids.end()) {
          goal_node_ids.push_back(child_end_node_index);
        }
      }

      int not_found_end_node_count = 0;

      // start_node_idsが親部分パスの終端ノードの子ノードを持つまで回す
      // start_node_idsがend_node_idsの要素を持つまで回す
      while (true) {
        std::vector<Path> discovered_paths;

        // start_node_idsから親部分パスの子ノードまでの経路を探索する
        for (size_t j_n = 0; j_n < start_node_ids.size(); j_n++) {
          Path discovered_path;
          int start_node_id = start_node_ids[j_n];

          for (size_t k_g = 0; k_g < goal_node_ids.size(); k_g++) {
            int end_node_id = goal_node_ids[k_g];
            // std::cout << "3:astar_search" << '\n';
            // std::cout << "(start, goal):" << start_node_id << "," << end_node_id << '\n';
            found_path = my_astar::astar_search(*child_graph, start_node_id, end_node_id, discovered_path.node_indices, check_collision, robot_state, joint_model_group, planning_scene_monitor);
            // found_path = astar_search(*child_graph, start_node_id, end_node_id, discovered_path.node_indices);
            // std::cout << "astar end" << '\n';
            // パスの先頭が end_node_index だとパスの向きが逆になっているので省く
            if (discovered_path.node_indices.size() > 1 && discovered_path.node_indices[0] == end_node_index) {
              continue;
            }
            if (found_path) {
              discovered_path.depth = level;
              discovered_paths.push_back(discovered_path);
            }
          }// k_g
        }// j_n

        // std::cout  << '\n';
        // std::cout << "discovered_path" << '\n';
        // showPaths(discovered_paths);
        // std::cout << '\n';
        //stop();

        // i = 1, pP = {}, S = {};
        // すべての見つけた部分パス
        if (discovered_paths.size() > 0) {
          start_node_ids.clear();
        }
        std::vector<Path> sub_partial_paths;
        int node_index_i = 1;
        for (size_t j_p = 0; j_p < discovered_paths.size(); j_p++) {
          paths.clear();

          int node_index_j;//子部分パスの終端ノードの親である親部分パスのノードのindexつまり、パスの順番1,2,....
          int end_node_index = discovered_paths[j_p].node_indices.back();
          int parent_id = child_graph->nodes[end_node_index].parent_id;
          // 親ノードのIDからindexを見つける
          auto itr = std::find(partial_path->node_indices.begin(), partial_path->node_indices.end(), parent_id);
          if (itr == partial_path->node_indices.end()) {
            node_index_j = partial_path->node_indices.size() + 1;
          } else {
            node_index_j = std::distance(partial_path->node_indices.begin(), itr) + 1;
          }

          // std::cout << "(node_index_i, node_index_j):" << node_index_i <<  " " << node_index_j << '\n';

          if (node_index_j < node_index_i) {
            continue;
          }

          if (node_index_j > node_index_i) {
            start_node_ids.clear();
            sub_partial_paths.clear();
          }

          sub_partial_paths.push_back(discovered_paths[j_p]);

          // 子部分パスの終端ノードと同じ親ノードを持つノードを入れる
          child_ids = &(parent_graph->nodes[parent_id].child_ids);
          if (child_ids->size() == 1) {
            start_node_ids.push_back((*child_ids)[0]);
          }
          for (size_t k_n = 0; k_n < child_ids->size(); k_n++) {
            int child_id = (*child_ids)[k_n];
            // std::cout << "child_ids:" << child_id << '\n';
            if (child_id == end_node_index) {
              continue;
            }
            // すでに同じIDが格納されているか確認
            auto itr = std::find(start_node_ids.begin(), start_node_ids.end(), child_id);
            if (itr == start_node_ids.end()) {
              start_node_ids.push_back(child_id);
            }
            break;
          }// k_n
          node_index_i = node_index_j;

        } // j_p
        // std::cout << '\n';
        // std::cout << "sub_partial_path" << '\n';
        // showPaths(sub_partial_paths);
        // std::cout << '\n';

        // pathsの各パスの終端ノードがend_node_idsまで到達しているのか確認
        bool found_end_node = false;
        std::vector<bool> has_end_node(sub_partial_paths.size(), false);
        for (size_t j_p = 0; j_p < sub_partial_paths.size(); j_p++) {
          int id = sub_partial_paths[j_p].node_indices.back();
          auto itr = std::find(end_node_ids.begin(), end_node_ids.end(), id);
          if (itr != end_node_ids.end()) {
            found_end_node = true;
            has_end_node[j_p] = true;
          }
        }//j_p

        for (size_t j_p = 0; j_p < sub_partial_paths.size(); j_p++) {
          bool exist_path = false;
          if (found_end_node && !has_end_node[j_p]) {
            continue;
          }
          for (size_t k_p = 0; k_p < paths.size(); k_p++) {
            if (sub_partial_paths[j_p].node_indices == paths[k_p].node_indices) {
              exist_path = true;
              break;
            }
          }
          if (!exist_path) {
            paths.push_back(sub_partial_paths[j_p]);
          }
        }

        // std::cout << '\n';
        // std::cout << "paths" << '\n';
        // showPaths(paths);
        // std::cout << '\n';

        // end_node_id が discovered_pathsの各ノードの子ノードに含まれていない場合、
        // 探索が無限ループになるので、end_node_id １つをパスとみなしてループを抜けるようにする
        if ((paths.size() == 0) || (!found_end_node && not_found_end_node_count >= 10)) {
          std::vector<Path> path;
          for (size_t j_p = 0; j_p < not_found_end_node_count; j_p++) {
            paths_vector.pop_back();
          }
          path.resize(1);
          path[0].node_indices.resize(1);
          path[0].node_indices[0] = start_node_ids[0];
          paths_vector.push_back(path);
          path[0].node_indices[0] = end_node_ids[0];
          paths_vector.push_back(path);
          found_end_node = true;
        } else {
          paths_vector.push_back(paths);
        }

        // start_node_idsが親部分パスの終端ノードの子ノードを持つのか確認
        // start_node_idsがend_node_idsの要素を持つのか確認

        // std::cout << '\n';
        // std::cout << "start ";
        // showNodeIds(start_node_ids);

        // std::cout << "end ";
        // showNodeIds(end_node_ids);

        if (found_end_node) {
          // std::cerr << "Found End Node" << '\n';
          // stop();
          // std::cout << '\n';
          break;
        } else {
          ++not_found_end_node_count;
          // std::cerr << "Not Found End Node:" << not_found_end_node_count << '\n';
          // stop();
          // std::cout << '\n';
        }
      }// while 2
    } // i_p

    // このレベルの開始ノードと終了ノードを接続するP'の部分パスの最短シーケンスを求める
    // phase:0
    partial_paths = paths_vector[0];

    // phase:1
    if (partial_paths.size() == 2) {
      int min_index_i;
      int min_index_j;
      double min_dist = 100000;
      for (size_t i_p = 0; i_p < partial_paths.size(); i_p++) {
        int index = partial_paths[i_p].node_indices.back();
        const Node* node_i = &(child_graph->nodes[index]);
        for (size_t j_p = 0; j_p < paths_vector[1].size(); j_p++) {
          index = paths_vector[1][j_p].node_indices.front();
          const Node* node_j = &(child_graph->nodes[index]);
          double dist = computeDistance(node_i->mean_state, node_j->mean_state);
          if (dist < min_dist) {
            min_index_i = i_p;
            min_index_j = j_p;
            min_dist = dist;
          }
        }
      }
      const Path* path = &(partial_paths[min_index_i]);
      partial_paths[0] = *path;
      partial_paths[1] = paths_vector[1][min_index_j];
    } else {
      partial_paths.push_back(paths_vector[1][0]);
    }

    // phase: 2 ~ LAST
    for (size_t i_pv = 2; i_pv < paths_vector.size(); i_pv++) {
      int index = partial_paths.back().node_indices.back();
      const Node* node_i = &(child_graph->nodes[index]);
      int min_index_j;
      double min_dist = 10000;
      for (size_t j_p = 0; j_p < paths_vector[i_pv].size(); j_p++) {
        index = paths_vector[i_pv][j_p].node_indices.front();
        const Node* node_j = &(child_graph->nodes[index]);
        double dist = computeDistance(node_i->mean_state, node_j->mean_state);
        if (dist < min_dist) {
          dist = min_dist;
          min_index_j = j_p;
        }
      }
      partial_paths.push_back(paths_vector[i_pv][min_index_j]);
    }

    // std::cout << "partial_paths" << '\n';
    // showPaths(partial_paths);
    // std::cout << '\n';
    //stop();

  } // while 1
}
