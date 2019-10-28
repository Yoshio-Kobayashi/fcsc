#include <analysis_workspace/analyse_workspace.h>
#include <ros/package.h>

WorkspaceAnalyzer::WorkspaceAnalyzer(string group_name):
  robot_model_loader(ROBOT_DESCRIPTION),
  planning_scene_monitor(ROBOT_DESCRIPTION),
  kinematic_model(robot_model_loader.getModel()),
  robot_state(new robot_state::RobotState(kinematic_model)),
  planning_scene(new planning_scene::PlanningScene(kinematic_model)),
  move_group(group_name)
{
  this->group_name = group_name;
  setParameters();
}

void WorkspaceAnalyzer::setParameters()
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  private_nh.param<std::string>("group_name", group_name, "manipulator");
  private_nh.param("roll", rpy_msg.x, 0.0);
  private_nh.param("pitch", rpy_msg.y, 0.0);
  private_nh.param("yaw", rpy_msg.z, 0.0);

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ik_attempts = 1;
  ik_timeout  = 0.1;

  joint_model_group = kinematic_model->getJointModelGroup(group_name);
  joint_names = joint_model_group->getActiveJointModelNames();
  planning_scene_monitor.requestPlanningSceneState();

  marker_points.header.frame_id = "map";
  marker_points.header.stamp = ros::Time::now();
  marker_points.ns = "workspace_points";
  marker_points.action = visualization_msgs::Marker::ADD;
  marker_points.pose.orientation.w = 1.0;
  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;
  marker_points.scale.x = 0.01;
  marker_points.scale.y = 0.01;

  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 0.25;
}

void WorkspaceAnalyzer::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out)
{
  ps_in.header.stamp = ros::Time();

  ros::Rate rate(10);
  while (1) {
    try{
      listener.transformPose(frame, ps_in, ps_out);
      return;
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a position from \"%s\" to \"%s\": %s", ps_in.header.frame_id.c_str(), frame.c_str(), ex.what());
    }
    rate.sleep();
  }
}

void WorkspaceAnalyzer::saveAnalysisResultToDAT(std::string result_name)
{
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  std::stringstream file_name;
  std::ofstream outputfile;

  file_name << ros::package::getPath("analysis_workspace") + "/data/";
  file_name << "AnalysisResult_" << result_name <<  "_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".csv";

  ROS_WARN("open %s", file_name.str().c_str());
  outputfile.open(file_name.str().c_str());

  outputfile << "success_rate,";
  outputfile << "mean_x,";
  outputfile << "mean_y,";
  outputfile << "mean_z,";
  outputfile << "variance_x,";
  outputfile << "variance_y,";
  outputfile << "variance_z," << '\n';
  outputfile << analysis_result.success_rate << ",";
  outputfile << analysis_result.mean[RESULT_X] << ",";
  outputfile << analysis_result.mean[RESULT_Y] << ",";
  outputfile << analysis_result.mean[RESULT_Z] << ",";
  outputfile << analysis_result.variance[RESULT_X] << ",";
  outputfile << analysis_result.variance[RESULT_Y] << ",";
  outputfile << analysis_result.variance[RESULT_Z] << ",";

  // outputfile << "success_rate," << analysis_result.success_rate << '\n';
  // outputfile << "mean_x," << analysis_result.mean[RESULT_X] << '\n';
  // outputfile << "mean_y," << analysis_result.mean[RESULT_Y] << '\n';
  // outputfile << "mean_z," << analysis_result.mean[RESULT_Z] << '\n';
  // outputfile << "variance_x," << analysis_result.variance[RESULT_X] << '\n';
  // outputfile << "variance_y," << analysis_result.variance[RESULT_Y] << '\n';
  // outputfile << "variance_z," << analysis_result.variance[RESULT_Z] << '\n';

  outputfile.close();
}

void WorkspaceAnalyzer::saveWorkspaceDataToDAT(std::string data_name)
{
   time_t now = time(NULL);
   struct tm *pnow = localtime(&now);
   std::stringstream file_name;
   std::ofstream outputfile;

   file_name << ros::package::getPath("analysis_workspace") + "/data/";
   file_name << "WorkspaceData_" << data_name <<  "_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".dat";

   ROS_WARN("open %s", file_name.str().c_str());
   outputfile.open(file_name.str().c_str());

   outputfile << "# x y z" << endl;
   for (int i=0; i < analysis_poses.size(); i++) {
     geometry_msgs::Point p = analysis_poses[i].position;
    outputfile << p.x << " " << p.y << " " << p.z << endl;
   }

   outputfile.close();
}

void WorkspaceAnalyzer::saveWorkspaceDataToYAML(std::string data_name)
{
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  std::stringstream file_name;
  std::ofstream outputfile;

  file_name << ros::package::getPath("analysis_workspace") + "/data/";
  file_name << "WorkspaceData_" << data_name <<  "_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".yaml";

  ROS_WARN("open %s", file_name.str().c_str());
  outputfile.open(file_name.str().c_str());

  outputfile << "poses:" << endl;
  for (size_t i = 0; i < analysis_poses.size(); i++) {
    outputfile << "  - position :" << endl;
    outputfile << "      x : " << analysis_poses[i].position.x << endl;
    outputfile << "      y : " << analysis_poses[i].position.y << endl;
    outputfile << "      z : " << analysis_poses[i].position.z << endl;
    outputfile << "    orientation :" << endl;
    outputfile << "      x : " << analysis_poses[i].orientation.x << endl;
    outputfile << "      y : " << analysis_poses[i].orientation.y << endl;
    outputfile << "      z : " << analysis_poses[i].orientation.z << endl;
    outputfile << "      w : " << analysis_poses[i].orientation.w << endl;
  }
  outputfile.close();
}

void WorkspaceAnalyzer::analyseWorkspaceFromTaskSpace(moveit_msgs::WorkspaceParameters task_space, std::vector<geometry_msgs::Quaternion> orientations, double step)
{
  collision_detection::CollisionRequest         collision_request;
  collision_detection::CollisionResult          collision_result;

  geometry_msgs::PoseStamped  end_effector_pose_stamped;
  geometry_msgs::PoseStamped  task_pose_stamped;
  int                         success_count = 0;
  int                         total_count = 0;

  planning_scene_monitor.requestPlanningSceneState();
  robot_state::RobotState& current_state  = planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst();

  task_pose_stamped.header.frame_id = task_space.header.frame_id;

  // marker_points.header.frame_id = "base_footprint";
  marker_points.header.frame_id = "map";
  marker_points.header.stamp = ros::Time::now();
  marker_points.points.clear();
  marker_points.colors.clear();

  analysis_poses.clear();

  collision_request.group_name = group_name;

  for (double z = task_space.min_corner.z; z <= task_space.max_corner.z; z += step) {
    for (double x = task_space.min_corner.x; x <= task_space.max_corner.x; x += step) {
      for (double y = task_space.min_corner.y; y <= task_space.max_corner.y; y += step) {
        for (size_t i = 0; i < orientations.size(); i++) {
          total_count++;

          task_pose_stamped.pose.position.x = x;
          task_pose_stamped.pose.position.y = y;
          task_pose_stamped.pose.position.z = z;
          task_pose_stamped.pose.orientation = orientations[i];

          transformPose("map", task_pose_stamped, end_effector_pose_stamped);
          // transformPose("base_footprint", task_pose_stamped, end_effector_pose_stamped);

          bool ik_found = current_state.setFromIK(joint_model_group, end_effector_pose_stamped.pose, ik_attempts, ik_timeout);
          if (!ik_found) {
            continue;
          }

          // collision_result.clear();
          // planning_scene_monitor.getPlanningScene()->checkCollision(collision_request, collision_result, current_state);
          // if (collision_result.collision) {
          //   continue;
          // }

          success_count++;

          // IKが成功ならデータ構造に保存
          analysis_poses.push_back(task_pose_stamped.pose);
          marker_points.points.push_back(end_effector_pose_stamped.pose.position);
          marker_points.colors.push_back(green);
        }
      }
    }

  }
  // 平均と分散を求めてみる
  double mean_x, mean_y, mean_z;
  double variance_x, variance_y, variance_z;

  mean_x = mean_y = mean_z = 0;
  for (size_t i = 0; i < analysis_poses.size(); i++) {
    mean_x += analysis_poses[i].position.x;
    mean_y += analysis_poses[i].position.y;
    mean_z += analysis_poses[i].position.z;
  }

  mean_x /= (double)analysis_poses.size();
  mean_y /= (double)analysis_poses.size();
  mean_z /= (double)analysis_poses.size();

  variance_x = variance_y = variance_z = 0;
  for (size_t i = 0; i < analysis_poses.size(); i++) {
    double diff_x, diff_y, diff_z;
    diff_x = analysis_poses[i].position.x - mean_x;
    diff_y = analysis_poses[i].position.y - mean_y;
    diff_z = analysis_poses[i].position.z - mean_z;
    variance_x += diff_x * diff_x;
    variance_y += diff_y * diff_y;
    variance_z += diff_z * diff_z;
  }

  variance_x /= (double)analysis_poses.size();
  variance_y /= (double)analysis_poses.size();
  variance_z /= (double)analysis_poses.size();

  analysis_result.success_rate = (double)success_count / (double)total_count;
  analysis_result.mean[RESULT_X] = mean_x;
  analysis_result.mean[RESULT_Y] = mean_y;
  analysis_result.mean[RESULT_Z] = mean_z;
  analysis_result.variance[RESULT_X] = variance_x;
  analysis_result.variance[RESULT_Y] = variance_y;
  analysis_result.variance[RESULT_Z] = variance_z;

  std::cout << "/***************** RESULT *****************/" << '\n';
  std::cout << "success_rate:" << analysis_result.success_rate << '\n';
  std::cout << "mean_x:" << analysis_result.mean[RESULT_X] << '\n';
  std::cout << "mean_y:" << analysis_result.mean[RESULT_Y] << '\n';
  std::cout << "mean_z:" << analysis_result.mean[RESULT_Z] << '\n';
  std::cout << "variance_x:" << analysis_result.variance[RESULT_X] << '\n';
  std::cout << "variance_y:" << analysis_result.variance[RESULT_Y] << '\n';
  std::cout << "variance_z:" << analysis_result.variance[RESULT_Z] << '\n';
  std::cout << "/******************************************/" << '\n';

  ros::Rate loop(50);
  for (int i=0; i<100; i++) {
    marker_pub.publish(marker_points);
    loop.sleep();
  }
}

void WorkspaceAnalyzer::analyseWorkspaceFromJointSpace(int particle)
{
  std::vector<double> joint_values(6);
  std::vector<double> shoulder_pan_joint_value(particle);
  std::vector<double> shoulder_lift_joint_value(particle);
  std::vector<double> elbow_joint_value(particle);
  std::vector<double> wrist_1_joint_value(particle);
  std::vector<double> wrist_2_joint_value(particle);
  std::vector<double> wrist_3_joint_value(particle);
  std::vector<geometry_msgs::Pose> end_effector_poses;
  geometry_msgs::PoseStamped end_effector_pose_stamped;

  if (particle <= 0) {
    ROS_ERROR("analyseWorkspaceFromJointSpace:[particle] is illegal value. Please enter a positive number for [particle]");
    return;
  }

  visualization_msgs::Marker arrow;
  std::vector<visualization_msgs::Marker> arrows;

  marker_points.header.frame_id = "base_footprint";
  marker_points.header.stamp = ros::Time::now();
  marker_points.points.clear();
  marker_points.colors.clear();

  arrow.header.frame_id = "base_footprint";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = "workspace_arrow";
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.z = 0.01;
  arrow.scale.x = 0.1;
  arrow.scale.y = 0.01;
  arrow.color = green;

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  const moveit::core::JointBoundsVector& joint_bounds_vector = joint_model_group->getActiveJointModelsBounds();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor(ROBOT_DESCRIPTION);
  // robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  planning_scene_monitor.requestPlanningSceneState();

  for (size_t i = 0; i < joint_names.size(); i++) {
    cout << joint_names[i].c_str() << endl;
    cout << "min:" << joint_bounds_vector[i][0][0].min_position_  << " max:" << joint_bounds_vector[i][0][0].max_position_ << endl;
    cout << endl;
  }

  // 関節変位の粒度から取りうる関節値を格納
  for (int i = 0; i < particle; i++) {
    double ratio = (double)i / (double)(particle - 1);
    shoulder_pan_joint_value[i] = joint_bounds_vector[0][0][0].min_position_ +
                                  (joint_bounds_vector[0][0][0].max_position_ - joint_bounds_vector[0][0][0].min_position_) * ratio;

    shoulder_lift_joint_value[i] = joint_bounds_vector[1][0][0].min_position_ +
                                   (joint_bounds_vector[1][0][0].max_position_ - joint_bounds_vector[1][0][0].min_position_) * ratio;

    elbow_joint_value[i] = joint_bounds_vector[2][0][0].min_position_ +
                           (joint_bounds_vector[2][0][0].max_position_ - joint_bounds_vector[2][0][0].min_position_) * ratio;

    wrist_1_joint_value[i] = joint_bounds_vector[3][0][0].min_position_ +
                             (joint_bounds_vector[3][0][0].max_position_ - joint_bounds_vector[3][0][0].min_position_) * ratio;

    wrist_2_joint_value[i] = joint_bounds_vector[4][0][0].min_position_ +
                             (joint_bounds_vector[4][0][0].max_position_ - joint_bounds_vector[4][0][0].min_position_) * ratio;

    wrist_3_joint_value[i] = joint_bounds_vector[5][0][0].min_position_ +
                             (joint_bounds_vector[5][0][0].max_position_ - joint_bounds_vector[5][0][0].min_position_) * ratio;
  }

  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header.frame_id = "base_footprint";
  for (int index1 = 0; index1 < particle; index1++) {
    for (int index2 = 0; index2 < particle; index2++) {
      for (int index3 = 0; index3 < particle; index3++) {
        for (int index4 = 0; index4 < particle; index4++) {
          for (int index5 = 0; index5 < particle; index5++) {
            for (int index6 = 0; index6 < particle; index6++) {
                // 関節変位を設定
                joint_values[0] = shoulder_pan_joint_value[index1];
                joint_values[1] = shoulder_lift_joint_value[index2];
                joint_values[2] = elbow_joint_value[index3];
                joint_values[3] = wrist_1_joint_value[index4];
                joint_values[4] = wrist_2_joint_value[index5];
                joint_values[5] = wrist_3_joint_value[index6];
                // 関節変位をもとに手先の位置・姿勢計算（順運動学）
                robot_state->setJointGroupPositions(joint_model_group, joint_values);
                robot_state->update();
                // 関節可動範囲外・干渉・衝突チェック
                collision_result.clear();
                planning_scene_monitor.getPlanningScene()->checkCollision(collision_request, collision_result, *robot_state);
                if (collision_result.collision) {
                  continue;
                }
                // 手先位置・姿勢の特徴計算（可操作性とか）
                // Eigen::Affine3d を geometry_msgs::Pose に変換
                const Eigen::Affine3d& end_effector_state = robot_state->getGlobalLinkTransform("gripper_link");
                tf::poseEigenToMsg(end_effector_state, temp_pose.pose);
                transformPose("base_footprint", temp_pose, end_effector_pose_stamped);
                // 計算結果をデータ構造に格納
                arrow.pose = end_effector_pose_stamped.pose;
                // arrows.push_back(arrow);
                marker_points.points.push_back(end_effector_pose_stamped.pose.position);
                marker_points.colors.push_back(green);
                analysis_poses.push_back(end_effector_pose_stamped.pose);
            }
          }
        }
      }
    }
  }

  ros::Rate loop(50);
  for (int i=0; i<50; i++) {
    marker_pub.publish(marker_points);
    loop.sleep();
  }
}

void WorkspaceAnalyzer::analyseWorkspaceFromTrajectory(moveit_msgs::WorkspaceParameters task_space, std::string target_name, geometry_msgs::Quaternion orientation, moveit_msgs::Constraints constraint, double step)
{
  int total_count   = 0;
  int success_count = 0;
  collision_detection::CollisionRequest         collision_request;
  collision_detection::CollisionResult          collision_result;
  geometry_msgs::PoseStamped                    end_effector_pose_stamped;
  geometry_msgs::PoseStamped                    task_pose_stamped;
  geometry_msgs::PoseStamped                    pout;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 初期姿勢を設定
  robot_state->setToDefaultValues(joint_model_group, target_name);
  move_group.setStartState(*robot_state);

  // move_group.setNamedTarget(target_name);
  // move_group.move();

  move_group.setPlanningTime(1.0);

  // 制約を設定
  move_group.setPathConstraints(constraint);

  move_group.setPoseReferenceFrame(task_space.header.frame_id);

  task_pose_stamped.header.frame_id = task_space.header.frame_id;

  marker_points.header.frame_id = "base_footprint";
  marker_points.header.stamp = ros::Time::now();
  marker_points.points.clear();
  marker_points.colors.clear();

  analysis_poses.clear();

  // 軌道の解析
  for (double z = task_space.min_corner.z; z <= task_space.max_corner.z; z += step) {
    std::cerr << "z:" << z << '\n';
    for (double x = task_space.min_corner.x; x <= task_space.max_corner.x; x += step) {
      for (double y = task_space.min_corner.y; y <= task_space.max_corner.y; y += step) {
        total_count++;

        task_pose_stamped.pose.position.x = x;
        task_pose_stamped.pose.position.y = y;
        task_pose_stamped.pose.position.z = z;
        task_pose_stamped.pose.orientation = orientation;

        transformPose("base_footprint", task_pose_stamped, end_effector_pose_stamped);

        move_group.setPoseTarget(task_pose_stamped.pose);
        bool plan_success = (bool)move_group.plan(plan);
        if (!plan_success) {
          continue;
        }

        // if ( (bool)move_group.execute(plan) == true) {
        //   exit(-1);
        // }

        success_count++;

        // IKが成功ならデータ構造に保存
        analysis_poses.push_back(task_pose_stamped.pose);
        marker_points.points.push_back(end_effector_pose_stamped.pose.position);
        marker_points.colors.push_back(green);
      }
    }
  }

  // 平均と分散を求めてみる
  double mean_x, mean_y, mean_z;
  double variance_x, variance_y, variance_z;

  mean_x = mean_y = mean_z = 0;
  for (size_t i = 0; i < analysis_poses.size(); i++) {
    mean_x += analysis_poses[i].position.x;
    mean_y += analysis_poses[i].position.y;
    mean_z += analysis_poses[i].position.z;
  }

  mean_x /= (double)analysis_poses.size();
  mean_y /= (double)analysis_poses.size();
  mean_z /= (double)analysis_poses.size();

  variance_x = variance_y = variance_z = 0;
  for (size_t i = 0; i < analysis_poses.size(); i++) {
    double diff_x, diff_y, diff_z;
    diff_x = analysis_poses[i].position.x - mean_x;
    diff_y = analysis_poses[i].position.y - mean_y;
    diff_z = analysis_poses[i].position.z - mean_z;
    variance_x += diff_x * diff_x;
    variance_y += diff_y * diff_y;
    variance_z += diff_z * diff_z;
  }

  variance_x /= (double)analysis_poses.size();
  variance_y /= (double)analysis_poses.size();
  variance_z /= (double)analysis_poses.size();

  analysis_result.success_rate = (double)success_count / (double)total_count;
  analysis_result.mean[RESULT_X] = mean_x;
  analysis_result.mean[RESULT_Y] = mean_y;
  analysis_result.mean[RESULT_Z] = mean_z;
  analysis_result.variance[RESULT_X] = variance_x;
  analysis_result.variance[RESULT_Y] = variance_y;
  analysis_result.variance[RESULT_Z] = variance_z;

  std::cout << "/***************** RESULT *****************/" << '\n';
  std::cout << "success_rate:" << analysis_result.success_rate << '\n';
  std::cout << "mean_x:" << analysis_result.mean[RESULT_X] << '\n';
  std::cout << "mean_y:" << analysis_result.mean[RESULT_Y] << '\n';
  std::cout << "mean_z:" << analysis_result.mean[RESULT_Z] << '\n';
  std::cout << "variance_x:" << analysis_result.variance[RESULT_X] << '\n';
  std::cout << "variance_y:" << analysis_result.variance[RESULT_Y] << '\n';
  std::cout << "variance_z:" << analysis_result.variance[RESULT_Z] << '\n';
  std::cout << "/******************************************/" << '\n';

  ros::Rate loop(50);
  for (int i=0; i<100; i++) {
    marker_pub.publish(marker_points);
    loop.sleep();
  }
}
