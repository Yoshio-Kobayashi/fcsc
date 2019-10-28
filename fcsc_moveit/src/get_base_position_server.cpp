#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <fstream>
#include <fcsc_msgs/GetBasePosition.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcException.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "fcsc_moveit/eiquadprog.hpp"

using namespace std;

void getRPYFromQuaternion(geometry_msgs::Quaternion q, double& r, double& p, double& y)
{
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(r, p, y);
}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

class GetBasePositionServer {
private:
  tf::TransformListener listener;

  ros::ServiceServer service;

  double ik_attemps_;
  double ik_timeout_;

  Eigen::VectorXd current_q;

  std::string group_name;
  std::string base_joint_name;
  std::string base_link_name;

  robot_model_loader::RobotModelLoader          robot_model_loader;
  robot_model::RobotModelPtr                    robot_model;
  moveit::core::RobotState                      robot_state;
  robot_state::JointModelGroup*                 joint_model_group;
  planning_scene_monitor::PlanningSceneMonitor  planning_scene_monitor;
  // moveit::core::JointBoundsVector& joint_bounds_vector;
  // dynamics_solver::DynamicsSolver* dynamics_solver;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  rviz_visual_tools::RvizVisualToolsPtr     rviz_visual_tools;

  std::string visual_frame_id;

  bool computeIK(geometry_msgs::Pose target_pose, bool check_collision=true);

  void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& pose_out);

  bool planBasePosition(fcsc_msgs::GetBasePosition::Request &req, fcsc_msgs::GetBasePosition::Response &res);

  // Hill Climbing(山登り法)よる最適な台車位置の探索
  bool searchOptimalBasePositionByHC(fcsc_msgs::GetBasePosition::Request &req, fcsc_msgs::GetBasePosition::Response &res);

  // 二次計画法による台車位置の探索
  bool computeBasePositionByQP(geometry_msgs::Pose init_eef_pose, geometry_msgs::Pose target_eef_pose, geometry_msgs::Pose2D& base_position);

  // 指定された領域内をしらみつぶし探索する方法
  geometry_msgs::Pose2D searchOptimalBasePositionExhaustively(std::vector<fcsc_msgs::PoseStampedWithCost> target_poses, moveit_msgs::WorkspaceParameters search_area);

  void createNeighborBasePos(geometry_msgs::Pose2D reference_base_pos, std::vector<geometry_msgs::Pose2D>& neighbor_base_pos, double dist);
  double evaluateFromIKandManipulability(std::vector<fcsc_msgs::PoseStampedWithCost> target_poses, std::vector<fcsc_msgs::PoseStampedWithCost> visual_poses, bool visualize=true);
  // double evaluateFromIKandManipulability(std::vector<geometry_msgs::Pose> target_poses);
  int countFoundIK(std::vector<geometry_msgs::Pose> target_poses);
  void setBasePosition(geometry_msgs::Pose2D base_position);
  void transformTargetPoses(geometry_msgs::Pose2D base_position, std::vector<fcsc_msgs::PoseStampedWithCost> poses_in, std::vector<fcsc_msgs::PoseStampedWithCost>& poses_out);
  double getManipulability();

public:
  GetBasePositionServer(string arm_name);
  // ~GetBasePositionServer() { delete robot_state; };
  void start();
};

GetBasePositionServer::GetBasePositionServer(string arm_name):
  robot_model_loader("robot_description"),
  planning_scene_monitor("robot_description"),
  robot_model(robot_model_loader.getModel()),
  robot_state(robot_model),
  group_name(arm_name),
  visual_frame_id("map")
{
  ros::NodeHandle nh;

  ik_attemps_ = 1;
  ik_timeout_ = 0.05;

  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("map", "/visualization_marker"));
  visual_tools->setPlanningSceneTopic("/move_group/monitored_planning_scene");
  visual_tools->loadPlanningSceneMonitor();
  visual_tools->loadMarkerPub(true);
  visual_tools->loadRobotStatePub("display_robot_state");
  visual_tools->setManualSceneUpdating();

  rviz_visual_tools.reset(new rviz_visual_tools::RvizVisualTools(visual_frame_id, "/rviz_visual_tools"));
  rviz_visual_tools->enableBatchPublishing();

  service = nh.advertiseService("get_base_position", &GetBasePositionServer::planBasePosition, this);

  // robot_state = &(planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst());
  joint_model_group = robot_model->getJointModelGroup(group_name);

  base_joint_name = robot_model->getRootJointName();
  base_link_name = robot_model->getRootLinkName();

  std::cerr << "base_joint_name:[" << base_joint_name << "]" << '\n';
  std::cerr << "base_link_name:[" << base_link_name << "]" << '\n';

  robot_state.copyJointGroupPositions(group_name, current_q);
}

double GetBasePositionServer::getManipulability()
{
  double manipulability;

  /**********************直接計算*********************************/
  Eigen::MatrixXd J;    //ヤコビ行列
  Eigen::MatrixXd JT;   //ヤコビ行列の転置
  Eigen::MatrixXd JJT;  // j * jt

  J = robot_state.getJacobian(joint_model_group);
  // cout << J << endl;
  // exit(1);
  JT  = J.transpose();
  JJT = J * JT;
  manipulability = sqrt(JJT.determinant());

  if (manipulability == manipulability)
    return (manipulability);
  else
    return (0);
}

void GetBasePositionServer::createNeighborBasePos(geometry_msgs::Pose2D reference_base_pos, std::vector<geometry_msgs::Pose2D>& neighbor_base_pos, double dist)
{
  double direct_num = 4;
  double theta = M_PI * 2.0 / (double)direct_num;

  neighbor_base_pos.resize(direct_num);
  for (size_t i = 0; i < direct_num; i++) {
    neighbor_base_pos[i].x = reference_base_pos.x + dist * cos(theta * i + reference_base_pos.theta);
    neighbor_base_pos[i].y = reference_base_pos.y + dist * sin(theta * i + reference_base_pos.theta);
    neighbor_base_pos[i].theta = reference_base_pos.theta;
  }
}

bool GetBasePositionServer::planBasePosition(fcsc_msgs::GetBasePosition::Request &req, fcsc_msgs::GetBasePosition::Response &res)
{
  ROS_WARN("Called GetBasePositionServer");

  planning_scene_monitor.requestPlanningSceneState();

  robot_state = planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst();

  // robot_state.setToDefaultValues();

  // しらみつぶし探索で最適な台車位置を求める
  // res.base_position = searchOptimalBasePositionExhaustively(req.target_poses, req.search_area);
  // res.success = true;
  // return (true);

  // 単純な山登り法で台車位置を探索するアルゴリズム
  searchOptimalBasePositionByHC(req, res);
  return (true);

  // 二次計画問題で逆運動学を解いて台車位置を計算する
  geometry_msgs::Pose2D base_position;
  geometry_msgs::Pose init_pose;

  robot_state.setJointGroupPositions(group_name, current_q);

  const Eigen::Affine3d& end_effector_state = robot_state.getGlobalLinkTransform("gripper_link");
  tf::poseEigenToMsg(end_effector_state, init_pose);
  std::cout << "init_pose" << '\n';
  cout << init_pose << endl << endl;
  for (size_t i = 0; i < req.target_poses.size(); i++) {
    geometry_msgs::PoseStamped pin;
    geometry_msgs::PoseStamped pout;

    pin.header = req.target_poses[i].pose.header;
    pin.pose = req.target_poses[i].pose.pose;
    transformPose("map", pin, pout);
    std::cout << "target_pose" << '\n';
    cout << pout.pose << endl << endl;
    if (computeBasePositionByQP(init_pose, pout.pose, base_position)) {
      std::cout << "base_position" << '\n';
      std::cout << base_position << '\n';
      std::cout << std::endl;
    }
  }

  return (true);
}

bool GetBasePositionServer::computeIK(geometry_msgs::Pose target_pose, bool check_collision)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  collision_request.group_name = "daisha";

  bool ik_found = robot_state.setFromIK(joint_model_group, target_pose, ik_attemps_, ik_timeout_);
  if (!ik_found) {
    return (false);
  }
  if (!check_collision) {
    return (true);
  }
  planning_scene_monitor.getPlanningScene()->checkCollision(collision_request, collision_result, robot_state);
  if (collision_result.collision) {
    return (false);
  }
  return (true);
}

double GetBasePositionServer::evaluateFromIKandManipulability(std::vector<fcsc_msgs::PoseStampedWithCost> target_poses, std::vector<fcsc_msgs::PoseStampedWithCost> visual_poses, bool visualize)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  double ik_cost = 0;
  int ik_found_count = 0;
  double total_manipu = 0;
  double eval;

  for (size_t i = 0; i < target_poses.size(); i++) {
    bool success = computeIK(target_poses[i].pose.pose, /*check_collision=*/true);
    if (!success) {
      continue;
    }
    // visual_tools->publishRobotState(robot_state);
    // ros::Duration(0.5).sleep();
    // IKの成功回数 と コスト
    ++ik_found_count;
    ik_cost += target_poses[i].cost;
    total_manipu += getManipulability();
    // 描画
    if (visualize) {
      geometry_msgs::PoseStamped pin, pout;
      pin.header.frame_id = visual_poses[i].pose.header.frame_id;
      pin.pose = visual_poses[i].pose.pose;
      transformPose(visual_frame_id, pin, pout);
      rviz_visual_tools->publishArrow(pout.pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    }
  }

  rviz_visual_tools->trigger();

  if (ik_found_count == 0) {
    eval = 0;
  } else {
    // 各姿勢の可操作性の最大は1だから、可操作性の合計値を逆運動学の成功回数で割れば値が1以下になる
    eval = ik_cost + (total_manipu / (double)ik_found_count);
    // eval = (double)ik_found_count + (total_manipu / (double)ik_found_count);
  }

  return (eval);
}

int GetBasePositionServer::countFoundIK(std::vector<geometry_msgs::Pose> target_poses)
{
  int ik_found_count = 0;

  for (size_t i = 0; i < target_poses.size(); i++) {
    // IKの成功回数をカウントする
    if (computeIK(target_poses[i], /*check_collision=*/true)) {
      ik_found_count++;
    }
  }

  return (ik_found_count);
}

void GetBasePositionServer::setBasePosition(geometry_msgs::Pose2D base_position)
{
  std::vector<double> base_joint_values(3, 0.0);

  base_joint_values[0] = base_position.x;
  base_joint_values[1] = base_position.y;
  base_joint_values[2] = base_position.theta;
  robot_state.setJointPositions(base_joint_name, base_joint_values);
  robot_state.update();
}

void GetBasePositionServer::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& pose_out)
{
  ps_in.header.stamp = ros::Time();

  ros::Rate rate(10);
  while (1) {
    try{
      listener.transformPose(frame, ps_in, pose_out);
      return;
    }
    catch(tf::TransformException& ex) {
    }
    rate.sleep();
  }
}

// 目標手先位置をロボット座標系(base_footprint)基準に変換
void GetBasePositionServer::transformTargetPoses(geometry_msgs::Pose2D base_position, std::vector<fcsc_msgs::PoseStampedWithCost> poses_in, std::vector<fcsc_msgs::PoseStampedWithCost>& poses_out)
{
  tf::Transform tf_map_to_base;
  tf::Transform tf_map_to_target;
  tf::Transform tf_base_to_target;

  tf_map_to_base.setOrigin(tf::Vector3(base_position.x, base_position.y, 0));
  tf_map_to_base.setRotation(tf::createQuaternionFromYaw(base_position.theta));

  poses_out.resize(poses_in.size());
  for (size_t i = 0; i < poses_in.size(); i++) {
    geometry_msgs::PoseStamped ps;

    ps = poses_in[i].pose;
    if (ps.header.frame_id != "map") {
      geometry_msgs::PoseStamped ps_out;
      transformPose("map", ps, ps_out);
      ps = ps_out;
    }

    tf_map_to_target.setOrigin(tf::Vector3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
    tf_map_to_target.setRotation(tf::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w));

    tf_base_to_target = tf_map_to_base.inverse() * tf_map_to_target;

    poses_out[i].cost = poses_in[i].cost;
    poses_out[i].pose.header.frame_id = ps.header.frame_id;
    poses_out[i].pose.pose.position.x = tf_base_to_target.getOrigin().getX();
    poses_out[i].pose.pose.position.y = tf_base_to_target.getOrigin().getY();
    poses_out[i].pose.pose.position.z = tf_base_to_target.getOrigin().getZ();
    tf::quaternionTFToMsg(tf_base_to_target.getRotation(), poses_out[i].pose.pose.orientation);
  }
}

// 単純な山登り法で台車位置を探索するアルゴリズム
bool GetBasePositionServer::searchOptimalBasePositionByHC(fcsc_msgs::GetBasePosition::Request &req, fcsc_msgs::GetBasePosition::Response &res)
{
  geometry_msgs::Pose2D optimal_base_pos;
  std::vector<fcsc_msgs::PoseStampedWithCost> target_poses;
  // std::vector<fcsc_msgs::PoseStampedWithCost> tfed_target_poses;
  double best_eval;
  double dist;
  int n_not_found = 0;
  bool is_cost_zero = false;

  ROS_INFO("target_poses num[%d]", (int)req.target_poses.size());

  // PoseArray を base_link_name基準の pose の vector に格納
  target_poses.resize(req.target_poses.size());
  for (size_t i = 0; i < req.target_poses.size(); i++) {
    fcsc_msgs::PoseStampedWithCost target_pose;
    target_pose = req.target_poses[i];
    // map基準に変換
    if (target_pose.pose.header.frame_id != "map") {
      geometry_msgs::PoseStamped temp_pose;
      transformPose("map", target_pose.pose, temp_pose);
      target_pose.pose = temp_pose;
    }
    target_poses[i] = target_pose;
  }

  // 初期位置を最適位置に設定
  ROS_INFO("Set initial base position");
  optimal_base_pos = req.init_base_position;
  setBasePosition(optimal_base_pos);

  // transformTargetPoses(optimal_base_pos, target_poses, tfed_target_poses);

  // 初期位置でIKを解く
  // IKの成功回数をカウントする
  best_eval = evaluateFromIKandManipulability(target_poses, req.target_poses);
  std::cout << "init eval:" << best_eval << '\n';
  //countFoundIK(target_poses);

  dist = 0.1;

  while (dist <= 2.0) {
    geometry_msgs::Pose2D next_base_pos;
    std::vector<geometry_msgs::Pose2D> neighbor_base_pos;
    double next_eval;

    std::cerr << "dist:" << dist << '\n';

    // 近傍の集合を作成する
    createNeighborBasePos(optimal_base_pos, neighbor_base_pos, dist);

    // 近傍でIKを解く
    next_eval = -1;
    for (size_t i = 0; i < neighbor_base_pos.size(); i++) {
      geometry_msgs::Pose2D base_pos = neighbor_base_pos[i];
      setBasePosition(base_pos);
      // transformTargetPoses(base_pos, target_poses, tfed_target_poses);
      double eval = evaluateFromIKandManipulability(target_poses, req.target_poses);
      //countFoundIK(target_poses);
      if (next_eval < eval) {
        next_eval = eval;
        next_base_pos = base_pos;
      }
    }

    if (best_eval <= 0 && next_eval <= 0) {
      // 最適（候補）位置でも近傍位置でもIKの成功回数（コスト）が0なら探索範囲を拡大する
      dist = pow(2, n_not_found) * 0.01;
      n_not_found++;
      is_cost_zero = true;
    } else if (best_eval > next_eval) {
      res.success = true;
      res.base_position = optimal_base_pos;
      cerr << "best_eval:" << best_eval << endl;
      std::cerr <<  "base_pos:\n" << optimal_base_pos << '\n';
      // 探索範囲を縮小
      dist -= 0.01;
      if (dist < 0.01) {
        std::cerr << "success" << '\n';
        return (true);
      }
    } else {
      best_eval = next_eval;
      optimal_base_pos = next_base_pos;
      cerr << "best_eval:" << best_eval << endl;
      std::cerr <<  "base_pos:\n" << optimal_base_pos << '\n';
      if (is_cost_zero) {
        // 拡大した探索範囲を戻す
        dist = 0.1;
        is_cost_zero = false;
      }
    }
  }
  std::cerr << "false" << '\n';
  res.success = false;
  return(true);
}

// 二次計画問題による台車位置の探索
bool GetBasePositionServer::computeBasePositionByQP(geometry_msgs::Pose init_eef_pose, geometry_msgs::Pose target_eef_pose, geometry_msgs::Pose2D& base_position)
{
  static const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  static const moveit::core::JointBoundsVector& joint_bounds_vector = joint_model_group->getActiveJointModelsBounds();

  Eigen::MatrixXd I = /*2.0 */ Eigen::MatrixXd::Identity(10,10); // 単位行列
  Eigen::VectorXd g0 = Eigen::VectorXd::Zero(10); // 0ベクトル
  Eigen::MatrixXd J; // ヤコビ行列(6.10)
  Eigen::VectorXd v(6); // 手先速度ベクトル 定数
  Eigen::MatrixXd CI(7*2,10); // 干渉、関節の制約
  Eigen::VectorXd ci0 = Eigen::VectorXd::Zero(7*2); // 干渉、関節の制約
  Eigen::VectorXd qv(10); // 関節速度
  Eigen::VectorXd q(10); // 関節位置

  // CI << -1,0,0,0,0,0,0,0,0,0,
  //       0,-1,0,0,0,0,0,0,0,0,
  //       0,0,-1,0,0,0,0,0,0,0,
  //       0,0,0,-1,0,0,0,0,0,0,
  //       0,0,0,0,-1,0,0,0,0,0,
  //       0,0,0,0,0,-1,0,0,0,0,
  //       0,0,0,0,0,0,-1,0,0,0,
  //       0,0,0,0,0,0,0,-1,0,0,
  //       0,0,0,0,0,0,0,0,-1,0,
  //       0,0,0,0,0,0,0,0,0,-1,
  //       1,0,0,0,0,0,0,0,0,0,
  //       0,1,0,0,0,0,0,0,0,0,
  //       0,0,1,0,0,0,0,0,0,0,
  //       0,0,0,1,0,0,0,0,0,0,
  //       0,0,0,0,1,0,0,0,0,0,
  //       0,0,0,0,0,1,0,0,0,0,
  //       0,0,0,0,0,0,1,0,0,0,
  //       0,0,0,0,0,0,0,1,0,0,
  //       0,0,0,0,0,0,0,0,1,0,
  //       0,0,0,0,0,0,0,0,0,1;

  CI << 0,0,0,-1,0,0,0,0,0,0,
        0,0,0,0,-1,0,0,0,0,0,
        0,0,0,0,0,-1,0,0,0,0,
        0,0,0,0,0,0,-1,0,0,0,
        0,0,0,0,0,0,0,-1,0,0,
        0,0,0,0,0,0,0,0,-1,0,
        0,0,0,0,0,0,0,0,0,-1,
        0,0,0,1,0,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,1;

  // 現在の手先位置姿勢ベクトルと目標手先位置姿勢ベクトルの差をとる
  Eigen::VectorXd current_eef(6);
  Eigen::VectorXd target_eef(6);
  Eigen::VectorXd error_eef(6);
  Eigen::VectorXd init_eef(6);
  double r, p, y;

  init_eef(0) = init_eef_pose.position.x;
  init_eef(1) = init_eef_pose.position.y;
  init_eef(2) = init_eef_pose.position.z;
  getRPYFromQuaternion(init_eef_pose.orientation, r, p, y);
  init_eef(3) = r;
  init_eef(4) = p;
  init_eef(5) = y;

  target_eef(0) = target_eef_pose.position.x;
  target_eef(1) = target_eef_pose.position.y;
  target_eef(2) = target_eef_pose.position.z;
  getRPYFromQuaternion(target_eef_pose.orientation, r, p, y);
  target_eef(3) = r;
  target_eef(4) = p;
  target_eef(5) = y;

  error_eef = target_eef - init_eef;

  // サンプリングする周期を決める
  double t = 100;
  double dt = 0.01;
  double particle_num = t / dt;
  double thresh = 0.5;

  v = error_eef / t;

  q = current_q;

  // while (1) {
  for (size_t i = 0; i < particle_num; i++) {
    // ヤコビ行列Jの設定
    // 現在探索中の姿勢qをrobot_stateに登録
    robot_state.setJointGroupPositions(group_name, q);
    J = robot_state.getJacobian(robot_state.getJointModelGroup(group_name));
    // visual_tools->publishRobotState(robot_state);
    // ros::Duration(0.005).sleep();

    // 現在の手先位置・姿勢
    geometry_msgs::Pose ee_pose;
    const Eigen::Affine3d& end_effector_state = robot_state.getGlobalLinkTransform("gripper_link");
    double r, p, y;
    tf::poseEigenToMsg(end_effector_state, ee_pose);
    current_eef(0) = ee_pose.position.x;
    current_eef(1) = ee_pose.position.y;
    current_eef(2) = ee_pose.position.z;
    getRPYFromQuaternion(ee_pose.orientation, r, p, y);
    current_eef(3) = r;
    current_eef(4) = p;
    current_eef(5) = y;

    // 目標の手先位置・姿勢
    // target_eef = init_eef + error_eef * (double)(i + 1) / (double)particle_num;

    // 手先速度の決定
    // v = (target_eef - current_eef) / dt;

    // 関節速度の決定
    // 躍度最小モデル HoffArbib(t, goal, x, v, a)
    {
      static Eigen::VectorXd accel = Eigen::VectorXd::Zero(6);
      static Eigen::VectorXd pos = current_eef;
      Eigen::VectorXd jerk;
      Eigen::VectorXd target_pos;

      pos = current_eef;
      target_pos = init_eef + error_eef * (double)(i + 1) / (double)particle_num;
      jerk = -9.0 * accel / t - 36.0 * v / (t  * t) + 60.0 * (target_pos - pos) / (t * t * t);
      accel = accel + jerk * dt;
      v = v + accel * dt;
      // pos = pos + v * dt;
      t = t - dt;
    }

    // 4列目（台座の上下関節）を削除
    // removeColumn(J, 3);

    // 関節速度の制約の設定
    for (size_t j = 0; j < 6; j++) {
      double max_jvec;
      double min_jvec;
      double upper_jpos = joint_bounds_vector[j+2][0][0].max_position_;
      double lower_jpos = joint_bounds_vector[j+2][0][0].min_position_;
      double upper_jvec = joint_bounds_vector[j+2][0][0].max_velocity_;
      double lower_jvec = joint_bounds_vector[j+2][0][0].min_velocity_;

      // 関節速度の最大値を決定
      if (upper_jpos - q[j+4] < thresh) {
        max_jvec = upper_jvec * ( (upper_jpos - q[j+4]) / thresh );
      } else {
        max_jvec = upper_jvec;
      }
      ci0[j+1] = max_jvec;

      // 関節速度の最小値を決定
      if (q[j+4] - lower_jpos < thresh) {
        min_jvec = lower_jvec * ( (q[j+4] - lower_jpos) / thresh );
      } else {
        min_jvec = lower_jvec;
      }
      ci0[j+1+7] = -1.0 * min_jvec;
    }

    double cost = Eigen::solve_quadprog(I, g0, J.transpose(), -v, CI.transpose(), ci0, qv);

    if (isinf(cost)) {
      ROS_ERROR("QP failed inf");
      return (false);
    }

    if (isnan(cost)) {
      ROS_ERROR("QP failed nan");
      return (false);
    }

    // 積分
    // 矩形積分
    q = q + qv * dt;

  }

  robot_state.setJointGroupPositions(group_name, q);
  visual_tools->publishRobotState(robot_state);

  cout << "solved q" << endl;
  cout << q << endl << endl;

  base_position.x = q(0);
  base_position.y = q(1);
  base_position.theta = q(2);

  return (true);
}

geometry_msgs::Pose2D GetBasePositionServer::searchOptimalBasePositionExhaustively(std::vector<fcsc_msgs::PoseStampedWithCost> target_poses, moveit_msgs::WorkspaceParameters search_area)
{
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  std::stringstream file_name;
  std::ofstream outputfile;
  geometry_msgs::Pose2D optimal_base_pos;
  double best_eval;
  double dist = 0.01;

  file_name << "/home/kdel/workspace/ros/fcsc_ws/src/fcsc_pkgs/fcsc_moveit/data/mobile_base_position_plannning/";
  file_name << "SearchOptimalBasePosition_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".dat";

  ROS_WARN("open %s", file_name.str().c_str());
  outputfile.open(file_name.str().c_str());

  // PoseArray を base_link_name基準の pose の vector に格納
  for (size_t i = 0; i < target_poses.size(); i++) {
    // map基準に変換
    if (target_poses[i].pose.header.frame_id != "map") {
      geometry_msgs::PoseStamped temp_pose;
      transformPose("map", target_poses[i].pose, temp_pose);
      target_poses[i].pose = temp_pose;
    }
  }

  // 台車の初期位置・コスト設定
  optimal_base_pos.x = search_area.min_corner.x;
  optimal_base_pos.y = search_area.min_corner.y;
  optimal_base_pos.theta = 0;
  setBasePosition(optimal_base_pos);
  best_eval = evaluateFromIKandManipulability(target_poses, target_poses, /* visualize */false);

  // 探索開始
  outputfile << "# x y cost" << endl;
  for (double x = search_area.min_corner.x; x <= search_area.max_corner.x; x += dist) {
    std::cerr << "x:" << x << '\n';
    for (double y = search_area.min_corner.y; y <= search_area.max_corner.y; y += dist) {
      std::cerr << "  y:" << y << '\n';

      geometry_msgs::Pose2D base_pos;
      double eval;

      // 台車位置設定
      base_pos.x = x;
      base_pos.y = y;
      base_pos.theta = 0;
      setBasePosition(base_pos);

      // 評価
      eval = evaluateFromIKandManipulability(target_poses, target_poses, /* visualize */ false);

      outputfile << x << " " << y << " " << eval << endl;

      // 比較 よければ入れ替え
      if (eval > best_eval) {
        best_eval = eval;
        optimal_base_pos = base_pos;
      }
    }
  }

  outputfile.close();

  return (optimal_base_pos);
}

void GetBasePositionServer::start()
{
  ROS_INFO("get_base_position_server start");
  ros::spin();
  // ros::Rate loop_rate(100);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_base_position_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // GetBasePositionServer server("mobile_manipulator");
  GetBasePositionServer server("manipulator");
  // GetBasePositionServer server("manipulator_with_elevator");

  server.start();
  return 0;
}
