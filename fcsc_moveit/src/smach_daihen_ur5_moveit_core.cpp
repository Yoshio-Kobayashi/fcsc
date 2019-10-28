#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"

ObjectSortType object_sort_type;

FcscCore::FcscCore(string group_name, string ee_name):
  move_group(group_name),
  planning_scene_monitor(ROBOT_DESCRIPTION),
  robot_name("daihen_ur5"),
  robot_hand(ee_name),
  onigiri_placement_index(2, 0),
  drink_placement_index(2, 0),
  bento_placement_index(2, 0),
  is_executing_(false),
  is_succeeded_(true),
  timeup_(false)
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue params;

  nh.param<std::string>("visual_frame_id", visual_frame_id_, "map");

  nh.param<std::string>("recover_pose_name",          recover_pose_name,          "masita4");
  nh.param<std::string>("stock_pose_name",            stock_pose_name,            "masita2");
  nh.param<std::string>("container_center_pose_name", container_center_pose_name, "container_center");

  // visual_tools.reset(new rviz_visual_tools::RvizVisualTools(visual_frame_id_, "/visualization_marker"));
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools(visual_frame_id_, "/rviz_visual_tools"));
  visual_tools->enableBatchPublishing();

  private_nh.param<bool>("stop", stop, false);

  private_nh.param<double>("min_velocity",    min_velocity,     0.2);
  private_nh.param<double>("normal_velocity", normal_velocity,  0.4);
  private_nh.param<double>("max_velocity",    max_velocity,     0.6);

  nh.param("use_shelf_marker_detection", use_shelf_marker_detection_, true);

  private_nh.param<std::string>("attach_link", attach_link, "wrist_3_link");

  //touch_links_のリンク名を取得
  nh.getParam("touch_links", params);
  setTouchLinksParams(params);

  nh.getParam("shelf_size/shelfA/depth", shelf_depth);
  nh.getParam("shelf_size/shelfB/width", shelf_width);


  nh.getParam("stocking_area", params);
  setStockingAreaParams(params["onigiri"],  stocking_area[ONIGIRI]);
  setStockingAreaParams(params["drink"],    stocking_area[DRINK]);
  setStockingAreaParams(params["bento"],    stocking_area[BENTO]);
  setStockingAreaParams(params["sandwich"], stocking_area[SANDWICH]);

  //サンドイッチの廃棄品情報
  nh.getParam("scrap_sandwich_id", params);
  setScrapSandwichIdParams(params);
  for (size_t i = 0; i < scrap_sandwich_ids.size(); i++) {
    ROS_INFO("Scrap sandwich id[%s]", scrap_sandwich_ids[i].c_str());
  }

  grasp_planner.generateGraspPose("onigiri",  &product_grasps[ONIGIRI].grasps);
  grasp_planner.generateGraspPose("drink",    &product_grasps[DRINK].grasps);
  grasp_planner.generateGraspPose("bento",    &product_grasps[BENTO].grasps);
  grasp_planner.generateGraspPose("sandwich", &product_grasps[SANDWICH].grasps);

  /********************************** SMACH ******************************************/
  detect_product_server   = nh.advertiseService("detect_product", &FcscCore::detectProduct, this);
  detect_shelf_server     = nh.advertiseService("detect_shelf_action", &FcscCore::detectShelf, this);
  detect_sandwich_server  = nh.advertiseService("detect_sandwich", &FcscCore::detectSandwich, this);
  pickup_product_server   = nh.advertiseService("pickup_product", &FcscCore::pickupProduct, this);
  place_product_server    = nh.advertiseService("place_product", &FcscCore::placeProduct, this);
  return_product_server   = nh.advertiseService("return_product", &FcscCore::returnProduct, this);
  pickup_sandwich_server  = nh.advertiseService("pickup_sandwich", &FcscCore::pickupSandwich, this);
  faceup_standing_sandwiches_server  = nh.advertiseService("faceup_standing_sandwiches", &FcscCore::faceupSandwiches, this);
  recover_sandwich_server            = nh.advertiseService("recover_sandwich", &FcscCore::recoverSandwich, this);
  detect_and_recover_sandwich_server = nh.advertiseService("detect_and_recover_sandwich", &FcscCore::detectAndRecoverSandwich, this);
  change_sandwich_pose_server        = nh.advertiseService("change_sandwich_pose", &FcscCore::changeSandwichPose, this);
  move_initial_pose_server      = nh.advertiseService("move_initial_pose", &FcscCore::moveInitialPose, this);
  goto_stocking_base_pos_server = nh.advertiseService("goto_stocking_base_position", &FcscCore::goToStockingBasePosition, this);
  goto_faceup_base_pos_server   = nh.advertiseService("goto_faceup_base_position", &FcscCore::goToFaceupBasePosition, this);
  goto_near_shelf_server        = nh.advertiseService("goto_near_shelf", &FcscCore::goToNearShelf, this);
  goto_home_pos_server          = nh.advertiseService("goto_home_position", &FcscCore::goToHomePosition, this);
  sort_order_picking_sandwiches_server = nh.advertiseService("sort_order_picking_sandwiches", &FcscCore::sortOrderPickingSandwiches, this);
  /***********************************************************************************/

  display_trajectory_publisher      = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  enable_shelf_detection_publisher  = nh.advertise<std_msgs::Bool>("/shelf_detector/ar_track_alvar/enable_detection", 1, true);
  enable_object_detection_publisher = nh.advertise<std_msgs::Bool>("/ar_track_alvar/enable_detection", 1, true);

  trajectory_result_subscriber      = nh.subscribe("/execute_trajectory/result", 1, &FcscCore::trajectoryResultCB, this);
  time_up_subscriber                = nh.subscribe("time_finish", 1, &FcscCore::timeUpCB, this);

  planning_scene_client             = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  compute_ik_client                 = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  detect_object_client              = nh.serviceClient<fcsc_msgs::DetectObject>("detect_object");
  detect_shelf_client               = nh.serviceClient<fcsc_msgs::DetectObject>("detect_shelf");
  link_attacher_client              = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  link_detacher_client              = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

  shelfB_board_name = "shelfB_board_2";
  shelfB_name = "shelfB";

  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(normal_velocity);
  attach_detach_type = "moveit";
}

void FcscCore::trajectoryResultCB(const moveit_msgs::ExecuteTrajectoryActionResult result_msg)
{
  is_executing_ = false;
  result_msg.result.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS ? is_succeeded_ = true : is_succeeded_ = false;
  if (!is_succeeded_) {
    ROS_ERROR("[trajectoryResultCB]:%s", getMoveItErrorCodeString(result_msg.result.error_code).c_str());
  }
}

void FcscCore::timeUpCB(const std_msgs::Empty msg)
{
  ROS_WARN("timeUpCB");
  timeup_ = true;
  return;
}

bool FcscCore::pickup(std::string object_name, fcsc_msgs::Grasp grasp, const moveit_msgs::Constraints& constraints, bool check_ik)
{
  geometry_msgs::PoseArray    waypoints;
  geometry_msgs::Pose         waypoint;
  geometry_msgs::PoseStamped  visualize_pose;
  geometry_msgs::PoseStamped  pin;
  geometry_msgs::PoseStamped  pout;
  geometry_msgs::Vector3      vec;
  double                      norm;
  robot_state::RobotState     robot_state(planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst());
  moveit_msgs::RobotState     robot_state_msg;
  double                      ee_joint_position = robot_hand.posToJoint(grasp.pre_grasp_posture.points[0].positions[0]);
  bool                        success;

  // 把持姿勢の描画
  transformPose(visual_frame_id_, grasp.grasp_pose, visualize_pose, listener);
  visual_tools->publishArrow(visualize_pose.pose, rviz_visual_tools::GREEN);
  visual_tools->trigger();

  // 逆運動学で把持姿勢の確認
  if (check_ik) {
    robot_state.setJointGroupPositions(move_group.getName(), start_joint_group_positions_);
    robot_state.setVariablePosition("bottom_ezgripper_knuckle_palm_L1_1", ee_joint_position);
    robot_state.setVariablePosition("bottom_ezgripper_knuckle_palm_L1_2", ee_joint_position);
    robot_state.setVariablePosition("top_ezgripper_knuckle_palm_L1_1", ee_joint_position);
    robot_state.setVariablePosition("top_ezgripper_knuckle_palm_L1_2", ee_joint_position);
    moveit::core::robotStateToRobotStateMsg(robot_state, robot_state_msg);
    success = computeIK(grasp.grasp_pose, robot_state_msg);
    if (!success) {
      ROS_ERROR("[pickup]:IK failed");
      return (false);
    }
  }

  // ハンド調節
  waitForExecute();
  robot_hand.move(grasp.pre_grasp_posture.points[0].positions[0]);

  // 手前へ近づく
  // pre_grasp_approach と desired_distance で手先位置・姿勢を設定
  pin = grasp.grasp_pose;
  transformPose(grasp.pre_grasp_approach.direction.header.frame_id, pin, pout, listener);

  vec = grasp.pre_grasp_approach.direction.vector;
  norm = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);

  pout.pose.position.x -= grasp.pre_grasp_approach.desired_distance * vec.x / norm;
  pout.pose.position.y -= grasp.pre_grasp_approach.desired_distance * vec.y / norm;
  pout.pose.position.z -= grasp.pre_grasp_approach.desired_distance * vec.z / norm;

  move_group.setPathConstraints(constraints);
  success = moveArm(pout);
  move_group.clearPathConstraints();
  if (!success) {
    ROS_ERROR("[pickup]:moveArm failed");
    return (false);
  }

  // アプローチ
  waypoints.poses.clear();
  waypoints.header.frame_id = grasp.grasp_pose.header.frame_id;
  waypoints.poses.push_back(grasp.grasp_pose.pose);
  ROS_WARN("[pickup]:approach");
  for (size_t i = 0; i < 5; i++) {
    move_group.setMaxVelocityScalingFactor(min_velocity);
    success = moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);
    if (!success) {
      ROS_ERROR("[pickup]:Approach plan failed");
      move_group.setMaxVelocityScalingFactor(normal_velocity);
      return (false);
    }
    move_group.setMaxVelocityScalingFactor(normal_velocity);
    if (!waitForExecute()) {
      success = false;
      continue;
    }
    success = true;
    break;
  }
  if (!success) {
    ROS_ERROR("[pickup]:Approach execution failed");
    return (false);
  }

  // 掴む
  graspObject(object_name, grasp.grasp_posture.points[0].positions[0], grasp.grasp_posture.points[0].effort[0], grasp.ezgripper_control);

  // 退避
  pout = getCurrentEEfPose(grasp.post_grasp_retreat.direction.header.frame_id);

  vec = grasp.post_grasp_retreat.direction.vector;
  norm = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);

  pout.pose.position.x += grasp.post_grasp_retreat.desired_distance * vec.x / norm;
  pout.pose.position.y += grasp.post_grasp_retreat.desired_distance * vec.y / norm;
  pout.pose.position.z += grasp.post_grasp_retreat.desired_distance * vec.z / norm;

  waypoints.poses.clear();
  waypoints.header.frame_id = pout.header.frame_id;
  waypoints.poses.push_back(pout.pose);
  ROS_WARN("[pickup]:retreat");
  moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);

  return (true);
}

bool FcscCore::place(std::string object_name, moveit_msgs::PlaceLocation location, const moveit_msgs::Constraints& constraint)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  geometry_msgs::PoseArray    waypoints;
  geometry_msgs::Pose         waypoint;
  geometry_msgs::PoseStamped  pin;
  geometry_msgs::PoseStamped  pout;
  geometry_msgs::PoseStamped  place_ee_pose;
  geometry_msgs::Vector3      vec;
  double                      norm;

  // 物体の配置場所から目標手先位置設定
  place_ee_pose = getTargetPlacePoseFromObject(object_name, location.place_pose);

  // 逆運動学で姿勢の確認
  bool success = computeIK(place_ee_pose, false);
  if (!success) {
    ROS_ERROR("[place]:ik failed");
    return (false);
  }

  // 手前へ近づく
  // pre_grasp_approach と desired_distance で手先位置・姿勢を設定
  pin = place_ee_pose;
  transformPose(location.pre_place_approach.direction.header.frame_id, pin, pout, listener);

  vec = location.pre_place_approach.direction.vector;
  norm = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);

  pout.pose.position.x -= location.pre_place_approach.desired_distance * vec.x / norm;
  pout.pose.position.y -= location.pre_place_approach.desired_distance * vec.y / norm;
  pout.pose.position.z -= location.pre_place_approach.desired_distance * vec.z / norm;

  ROS_WARN("[place]:move");

  // move_group.setPlannerId("RRTstarkConfigDefault");
  // move_group.setPlanningTime(30);

  // 制約設定
  move_group.setPathConstraints(constraint);

  setStartState();

  if (!setTargetPose(pout)) {
    move_group.clearPathConstraints();
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    // move_group.setPlanningTime(5);
    ROS_ERROR("[place] setTargetPose");
    return (false);
  }

  if (!asyncPlanArm(plan)) {
  // if (!asyncPlanArm(plan, 1)) {
    move_group.clearPathConstraints();
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    // move_group.setPlanningTime(5);
    ROS_ERROR("[place] asyncPlanArm");
    return (false);
  }

  // 軌道が制約を満たしているかチェック
  bool is_path_valid = planning_scene_monitor.getPlanningScene()->isPathValid(plan.start_state_, plan.trajectory_, constraint);

  if (!is_path_valid) {
    ROS_WARN("Trajectory is invalid");
    Stop(stop);
  }

  if (!executeArm(plan)) {
    move_group.clearPathConstraints();
    // move_group.setPlannerId("RRTConnectkConfigDefault");
    // move_group.setPlanningTime(5);
    ROS_ERROR("[place] executeArm");
    return (false);
  }

  move_group.clearPathConstraints();

  // アプローチ
  move_group.setMaxVelocityScalingFactor(min_velocity);
  waypoints.poses.clear();
  waypoints.header.frame_id = place_ee_pose.header.frame_id;
  waypoints.poses.push_back(place_ee_pose.pose);
  ROS_WARN("[place]:approach");
  moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);
  move_group.setMaxVelocityScalingFactor(normal_velocity);

  // 離す
  releaseObject(object_name, location.post_place_posture.points[0].positions[0], location.post_place_posture.points[0].effort[0]);

  // 退避
  pout = getCurrentEEfPose(location.post_place_retreat.direction.header.frame_id);

  vec = location.post_place_retreat.direction.vector;
  norm = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);

  pout.pose.position.x += location.post_place_retreat.desired_distance * vec.x / norm;
  pout.pose.position.y += location.post_place_retreat.desired_distance * vec.y / norm;
  pout.pose.position.z += location.post_place_retreat.desired_distance * vec.z / norm;

  waypoints.poses.clear();
  waypoints.header.frame_id = pout.header.frame_id;
  waypoints.poses.push_back(pout.pose);
  ROS_WARN("[place]:retreat");
  moveArmCartesianPath(waypoints.header.frame_id, waypoints.poses, 0.01, 0.0, false);

  return (true);
}

bool FcscCore::moveInitialPose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  waitForExecute();

  if (getCurrentEEfPose("base_footprint").pose.position.x >= 0.4) {
    asyncMoveArm("fold");
  }

  res.success = asyncMoveArm(container_center_pose_name);

  // mobile_robot.moveBoard(0.01);


  return (true);
}

void FcscCore::getCurrentRobotStateMsg(moveit_msgs::RobotState& robot_state)
{
  planning_scene_monitor.requestPlanningSceneState();
  robot_state::RobotState current_state(planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst());
  current_state.setJointGroupPositions(move_group.getName(), start_joint_group_positions_);
  moveit::core::robotStateToRobotStateMsg(current_state, robot_state);
}

geometry_msgs::PoseStamped FcscCore::getCurrentEEfPose(std::string frame_id)
{
  geometry_msgs::PoseStamped ps_in;
  geometry_msgs::PoseStamped ps_out;

  ps_in = move_group.getCurrentPose();
  if (ps_in.header.frame_id != frame_id)
    transformPose(frame_id, ps_in, ps_out, listener);
  else
    ps_out = ps_in;
  return(ps_out);
}

bool FcscCore::getCurrentObjectPose(std::string object_name, std::string frame_id, geometry_msgs::PoseStamped& object_pose_stamped)
{
  moveit_msgs::GetPlanningScene srv;
  geometry_msgs::PoseStamped pose_stamped_tmp;

  srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  planning_scene_client.call(srv);

  for (int i = 0; i < srv.response.scene.world.collision_objects.size(); i++) {
    if (srv.response.scene.world.collision_objects[i].id != object_name) {
      continue;
    }

    object_pose_stamped.header.frame_id = srv.response.scene.world.collision_objects[i].header.frame_id;
    if (srv.response.scene.world.collision_objects[i].primitive_poses.size() > 0) {//primiveのとき
      object_pose_stamped.pose = srv.response.scene.world.collision_objects[i].primitive_poses[0];
    } else {//meshのとき
      object_pose_stamped.pose = srv.response.scene.world.collision_objects[i].mesh_poses[0];
    }

    if (object_pose_stamped.header.frame_id != object_name) {
      transformPose(frame_id, object_pose_stamped, pose_stamped_tmp, listener);
      object_pose_stamped = pose_stamped_tmp;
    }

    return (true);
  }
  return (false);
}

void FcscCore::setStartState()
{
  planning_scene_monitor.requestPlanningSceneState();
  robot_state::RobotState start_state(planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst());
  start_state.setJointGroupPositions(move_group.getName(), start_joint_group_positions_);
  move_group.setStartState(start_state);
}

bool FcscCore::computeIK(geometry_msgs::PoseStamped pose_stamped, moveit_msgs::RobotState robot_state, sensor_msgs::JointState &joint_state, bool avoid_collisions, const moveit_msgs::Constraints &constraints)
{
  moveit_msgs::GetPositionIK      ik_srv;
  moveit_msgs::PositionIKRequest  req;

  req.robot_state = robot_state;
  req.group_name = move_group.getName();
  req.avoid_collisions = avoid_collisions;
  req.pose_stamped = pose_stamped;
  req.timeout = ros::Duration(0.25);
  req.constraints = constraints;
  req.attempts = 1;
  ik_srv.request.ik_request = req;

  compute_ik_client.call(ik_srv);
  if (ik_srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    joint_state = ik_srv.response.solution.joint_state;
    return (true);
  } else {
    ROS_ERROR("[computeIK]%s", getMoveItErrorCodeString(ik_srv.response.error_code).c_str());
    return (false);
  }
}

bool FcscCore::computeIK(geometry_msgs::PoseStamped pose_stamped, moveit_msgs::RobotState robot_state, const moveit_msgs::Constraints &constraints)
{
  sensor_msgs::JointState joint_state;
  bool avoid_collisions = true;
  return (computeIK(pose_stamped, robot_state, joint_state, avoid_collisions, constraints));
}

bool FcscCore::computeIK(geometry_msgs::PoseStamped pose_stamped, bool avoid_collisions, const moveit_msgs::Constraints &constraints)
{
  moveit_msgs::RobotState robot_state;
  sensor_msgs::JointState joint_state;

  getCurrentRobotStateMsg(robot_state);
  return (computeIK(pose_stamped, robot_state, joint_state, avoid_collisions, constraints));
}

bool FcscCore::computeIK(geometry_msgs::PoseStamped pose_stamped, sensor_msgs::JointState &joint_state, const moveit_msgs::Constraints &constraints)
{
  moveit_msgs::RobotState robot_state;
  bool avoid_collisions = true;

  getCurrentRobotStateMsg(robot_state);
  return (computeIK(pose_stamped, robot_state, joint_state, avoid_collisions, constraints));
}

bool FcscCore::setStockingAreaParams(XmlRpc::XmlRpcValue &params, StockingArea &stocking_area)
{
  try
  {
    stocking_area.child_frame_id = (string)params["child_frame_id"];
    stocking_area.parent_frame_id = (string)params["parent_frame_id"];
    stocking_area.min_corner.x = (double)params["min_corner"]["x"];
    stocking_area.min_corner.y = (double)params["min_corner"]["y"];
    stocking_area.max_corner.x = (double)params["max_corner"]["x"];
    stocking_area.max_corner.y = (double)params["max_corner"]["y"];
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool FcscCore::setScrapSandwichIdParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    for (size_t i = 0; i < params.size(); i++) {
      string s;
      s = (std::string)params[i];
      // 小文字を大文字に変換
      transform(s.begin(), s.end(), s.begin(), ::toupper);
      scrap_sandwich_ids.push_back(s);
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }
  return(true);
}

geometry_msgs::PoseStamped FcscCore::getTargetPlacePoseFromObject(std::string object_name, geometry_msgs::PoseStamped target_object_pose)
{
  geometry_msgs::PoseStamped target_ee_pose;

  //作業座標系基準で目標物体位置を設定した時の手先位置・姿勢を求める
  target_ee_pose = getPoseStamped(target_object_pose, "grasped_"+object_name, move_group.getEndEffectorLink(), listener);

  return (target_ee_pose);
}

void FcscCore::setProductPlacement(std::vector<fcsc_msgs::RecognizedObject> objects)
{
  int num_bento_A = 0;
  int num_bento_B = 0;
  int num_drink_A = 0;
  int num_drink_B = 0;
  int num_onigiri_A = 0;
  int num_onigiri_B = 0;

  onigiri_placement_index[0] = 0;
  onigiri_placement_index[1] = 0;
  drink_placement_index[0] = 0;
  drink_placement_index[1] = 0;
  bento_placement_index[0] = 0;
  bento_placement_index[1] = 0;

  // どの種類の商品が何個あるのか数える
  for (size_t i = 0; i < objects.size(); i++) {
    int* num_product_A = NULL;
    int* num_product_B = NULL;
    switch (objects[i].type) {
      case fcsc_msgs::RecognizedObject::BENTO : {
        num_product_A = &num_bento_A;
        num_product_B = &num_bento_B;
        break;
      }
      case fcsc_msgs::RecognizedObject::DRINK : {
        num_product_A = &num_drink_A;
        num_product_B = &num_drink_B;
        break;
      }
      case fcsc_msgs::RecognizedObject::ONIGIRI : {
        num_product_A = &num_onigiri_A;
        num_product_B = &num_onigiri_B;
        break;
      }
      default : continue;
    }
    if (objects[i].name.find("_A_") != std::string::npos) {
      ++(*num_product_A);
    } else {
      ++(*num_product_B);
    }
  }

  object_placements.resize(6);

  // 各商品ごとに配置場所を決める
  // 弁当
  // 基準位置の設定
  geometry_msgs::PoseStamped base_placement;

  double bento_height = 0.045;

  base_placement.header.frame_id = stocking_area[BENTO].child_frame_id;
  base_placement.pose.position.x = 0.18 / 2.0 + 0.03;
  base_placement.pose.position.y = 0.9 / 2.0 - 0.2;
  base_placement.pose.position.z = bento_height;
  for (size_t i = 0; i < num_bento_A; i++) {
    geometry_msgs::PoseStamped placement = base_placement;
    switch (i) {
      // z = (積んである弁当の厚み) + (弁当の高さの中心) + (オフセット)
      case 0: placement.pose.position.z = bento_height * 0.0 + bento_height / 2.0 + 0.03; break;
      case 1: placement.pose.position.z = bento_height * 1.0 + bento_height / 2.0 + 0.03; break;
      case 2: placement.pose.position.z = bento_height * 2.0 + bento_height / 2.0 + 0.03; break;
    }
    object_placements[0].placements.push_back(placement);
  }
  // 基準位置の設定
  base_placement.pose.position.y = 0.9 / 2.0 + 0.2;
  for (size_t i = 0; i < num_bento_B; i++) {
    geometry_msgs::PoseStamped placement = base_placement;
    switch (i) {
      // z = (積んである弁当の厚み) + (弁当の高さの中心) + (オフセット)
      case 0: placement.pose.position.z = bento_height * 0.0 + bento_height / 2.0 + 0.03; break;
      case 1: placement.pose.position.z = bento_height * 1.0 + bento_height / 2.0 + 0.03; break;
      case 2: placement.pose.position.z = bento_height * 2.0 + bento_height / 2.0 + 0.03; break;
    }
    object_placements[1].placements.push_back(placement);
  }

  // ドリンク
  // 基準位置の設定
  base_placement.header.frame_id = stocking_area[DRINK].child_frame_id;
  base_placement.pose.position.x = 0.08 / 2.0 + 0.02;
  base_placement.pose.position.y = 0.9 / 2.0 - 0.05;
  base_placement.pose.position.z = 0.108 / 2.0 + 0.01;
  for (size_t i = num_drink_A; i > 0; --i) {
    geometry_msgs::PoseStamped placement = base_placement;
    placement.pose.position.x += (i - 1) * 0.1;
    // placement.pose.position.x += (i - 1) * 0.12;
    object_placements[2].placements.push_back(placement);
  }
  // 基準位置の設定
  base_placement.pose.position.y = 0.9 / 2.0 + 0.1;
  for (size_t i = num_drink_B; i > 0; --i) {
    geometry_msgs::PoseStamped placement = base_placement;
    placement.pose.position.x += (i - 1) * 0.1;
    // placement.pose.position.x += (i - 1) * 0.12;
    object_placements[3].placements.push_back(placement);
  }

  // おにぎり
  // 基準位置の設定
  base_placement.header.frame_id = stocking_area[ONIGIRI].child_frame_id;
  base_placement.pose.position.x = 0.05;
  base_placement.pose.position.y = 0.9 / 2.0 - 0.1;
  base_placement.pose.position.z = 0.06 / 2.0 - 0.02;
  for (size_t i = num_onigiri_A; i > 0; --i) {
    geometry_msgs::PoseStamped placement = base_placement;
    placement.pose.position.x += (i - 1) * 0.05;
    object_placements[4].placements.push_back(placement);
  }
  // 基準位置の設定
  base_placement.pose.position.y = 0.9 / 2.0 + 0.1;
  for (size_t i = num_onigiri_B; i > 0; --i) {
    geometry_msgs::PoseStamped placement = base_placement;
    placement.pose.position.x += (i - 1) * 0.05;
    object_placements[5].placements.push_back(placement);
  }
}

// 把持姿勢を並び替える
// 現在の手先の姿勢から回転が少ない姿勢を優先させる
// 手先の回転はYaw角のみ考慮
void FcscCore::sortGraspPoseIndex(ProductType type, std::string object_name ,std::vector<int> &grasp_indices)
{
  std::vector<ass_arr> grasp_cost_map;
  double r, p, y;
  geometry_msgs::PoseStamped pin;
  geometry_msgs::PoseStamped pout;

  grasp_indices.resize(product_grasps[type].grasps.size());

  // 手首回転（container座標系のx軸周りの回転）が少ない姿勢を優先するようにする
  for (int i = 0; i < product_grasps[type].grasps.size(); i++) {
    pin = product_grasps[type].grasps[i].grasp_pose;
    pin.header.frame_id = object_name;
    transformPose("base_footprint", pin, pout, listener);
    // transformPose("container", pin, pout);
    getRPYFromQuaternion(pout.pose.orientation, r, p, y);
    if (r < 0) r *= -1.0;
    r = fmod(r, 2.0 * M_PI);
    // 値を 0 ~ M_PI の範囲に収める
    if (r > M_PI) r -= 2.0 * (r - M_PI);
    grasp_cost_map.push_back( ass_arr(i, r) );
  }

  // 昇順でソート
  sort(grasp_cost_map.begin(), grasp_cost_map.end(), sort_less);

  int loop = 0;
  for (std::vector<ass_arr>::iterator it = grasp_cost_map.begin(); it != grasp_cost_map.end(); it++) {
    grasp_indices[loop++] = it->first;
  }
}

// 棚の手前にあるサンドイッチを優先する
bool FcscCore::sortOrderPickingSandwiches(fcsc_msgs::SortOrderManipulation::Request &req, fcsc_msgs::SortOrderManipulation::Response &res)
{
  std::vector<pose_ass_arr> sandwich_pose_ass_arr;

  // 棚の手前付近を優先させるように並べる
  for (size_t i = 0; i < req.objects.size(); i++) {
    geometry_msgs::PoseStamped ps;
    transformPose(shelfB_board_name, req.objects[i].pose, ps, listener);
    sandwich_pose_ass_arr.push_back( pose_ass_arr(i, ps.pose) );
  }
  object_sort_type = OBJECT_SORT_X;
  sort(sandwich_pose_ass_arr.begin(), sandwich_pose_ass_arr.end(), object_sort_less);

  for (std::vector<pose_ass_arr>::iterator it = sandwich_pose_ass_arr.begin(); it != sandwich_pose_ass_arr.end(); it++) {
    res.objects.push_back(req.objects[it->first]);
  }

  return (true);
}

// 把持方向でソートする
void FcscCore::sortSandwichGrasps(std::string sandwich_name, std::vector<int>& grasp_indices, int manipulation_type)
{
  std::vector<ass_arr>                    grasp_cost_map;
  collision_detection::CollisionRequest   collision_request;
  collision_detection::CollisionResult    collision_result;

  robot_state::RobotState robot_state(planning_scene_monitor.getPlanningScene()->getCurrentStateNonConst());
  const robot_state::JointModelGroup* joint_model_group(robot_state.getJointModelGroup(move_group.getName()));

  collision_request.contacts = true;
  collision_request.group_name = robot_hand.getName();
  collision_request.max_contacts = 20;

  for (size_t i = 0; i < product_grasps[SANDWICH].grasps.size(); i++) {
    product_grasps[SANDWICH].grasps[i].grasp_pose.header.frame_id = sandwich_name;

    geometry_msgs::PoseStamped target_pose;
    transformPose("map", product_grasps[SANDWICH].grasps[i].grasp_pose, target_pose, listener);

    // IKで解があるか確認
    bool ik_found = robot_state.setFromIK(joint_model_group, target_pose.pose, 1, 0.1);
    if (!ik_found) {
      continue;
    }

    // 把持位置でグリッパを閉じたとき，グリッパが陳列棚・他のサンドイッチに接触するか確認
    bool is_contact = false;
    for (double ee_pos = product_grasps[SANDWICH].grasps[i].pre_grasp_posture.points[0].positions[0]; ee_pos > product_grasps[SANDWICH].grasps[i].grasp_posture.points[0].positions[0];) {

      double ee_joint = robot_hand.jointToPos(ee_pos);
      robot_state.setVariablePosition("bottom_ezgripper_knuckle_palm_L1_1", ee_joint);
      robot_state.setVariablePosition("bottom_ezgripper_knuckle_palm_L1_2", ee_joint);
      robot_state.setVariablePosition("top_ezgripper_knuckle_palm_L1_1", ee_joint);
      robot_state.setVariablePosition("top_ezgripper_knuckle_palm_L1_2", ee_joint);

      ee_pos -= 10.0;
      if (ee_pos < product_grasps[SANDWICH].grasps[i].grasp_posture.points[0].positions[0]) {
        ee_pos = product_grasps[SANDWICH].grasps[i].grasp_posture.points[0].positions[0];
      }

      // 干渉チェック
      collision_result.clear();
      planning_scene_monitor.getPlanningScene()->checkCollision(collision_request, collision_result, robot_state);
      if (!collision_result.collision) {
        continue;
      }
      // 棚とグリッパとの干渉チェック
      // 棚とグリッパが接触している場合，その把持姿勢は実行不可能
      for(collision_detection::CollisionResult::ContactMap::iterator itr = collision_result.contacts.begin(); itr != collision_result.contacts.end(); ++itr) {
        if ( (itr->first.first.find("ezgripper") == std::string::npos && itr->first.first != sandwich_name) ||
             (itr->first.second.find("ezgripper") == std::string::npos && itr->first.second != sandwich_name) ) {
          is_contact = true;
          break;
        }
      }
      if (is_contact) {
        break;
      }
    }
    if (is_contact) {
      continue;
    }

    // map座標系のz軸とmap基準の把持姿勢のz軸の内積を計算
    // Transform設定
    tf::Transform grasp_tf(tf::Quaternion(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w), tf::Vector3(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z));

    // Matrix3x3設定
    tf::Matrix3x3 grasp_matrix(grasp_tf.getRotation().normalized());

    // map座標系のz軸と，把持方向(x軸)との内積をとる
    double cost = grasp_matrix.tdotx(tf::Vector3(0, 0, 1.0));

    // (index,可操作度)のペアでデータ構造に格納
    grasp_cost_map.push_back(ass_arr(i, cost));
  }

  // 昇順
  // cost最小（手先が地面に対して垂直な姿勢）のものを優先する
  sort(grasp_cost_map.begin(), grasp_cost_map.end(), sort_less);

  grasp_indices.clear();
  for (std::vector<ass_arr>::iterator it = grasp_cost_map.begin(); it != grasp_cost_map.end(); it++) {
    grasp_indices.push_back(it->first);
  }

  // フェイスアップの場合はフィルムを掴む把持姿勢を優先する
  if (manipulation_type == fcsc_msgs::Manipulate::Request::FACEUP) {
    std::vector<int> film_grasp_indices;
    std::vector<int> body_grasp_indices;
    for (size_t i = 0; i < grasp_indices.size(); i++) {
      if (product_grasps[SANDWICH].grasps[grasp_indices[i]].id.find("film") == std::string::npos) {
        body_grasp_indices.push_back(grasp_indices[i]);
      } else {
        film_grasp_indices.push_back(grasp_indices[i]);
      }
    }
    grasp_indices.clear();
    std::copy(film_grasp_indices.begin(), film_grasp_indices.end(), back_inserter(grasp_indices));
    grasp_indices.insert(grasp_indices.end(), body_grasp_indices.begin(), body_grasp_indices.end());
  }
}

bool FcscCore::graspObject(std::string object_name, double joint_position, double effort, int control)
{
  waitForExecute();

  bool result = true;
  result = attachObject(object_name, attach_detach_type);
  result &= robot_hand.move(joint_position, effort, control);
  return (result);
}

bool FcscCore::releaseObject(std::string object_name, double joint_position, double effort)
{
  waitForExecute();

  bool result;
  result = detachObject(object_name, attach_detach_type);
  result &= robot_hand.move(joint_position, effort);
  return (result);
}

bool FcscCore::attachObject(std::string object_name, std::string type)
{
  gazebo_ros_link_attacher::Attach attach_detach_srv;
  std::string link_name;
  bool success = true;

  if ( (type.find("moveit") != std::string::npos) || (type.find("both") != std::string::npos) ) {
    success *= move_group.attachObject(object_name, move_group.getEndEffectorLink(), touch_links_);
  }
  if (  (type.find("gazebo") != std::string::npos) || (type.find("both") != std::string::npos) ) {
    if (object_name.find("onigiri") != std::string::npos) {
      link_name = "onigiri";
    } else if (object_name.find("bento") != std::string::npos) {
      link_name = "bento";
    } else if (object_name.find("drink") != std::string::npos) {
      link_name = "drink";
    } else if (object_name.find("sandwich") != std::string::npos) {
      link_name = "sandwich";
    }

    attach_detach_srv.request.model_name_1 = robot_name;
    attach_detach_srv.request.link_name_1 = attach_link;
    attach_detach_srv.request.model_name_2 = object_name;
    attach_detach_srv.request.link_name_2 = link_name;
    link_attacher_client.call(attach_detach_srv);
    success *= attach_detach_srv.response.ok;
  }

  return (success);
}

bool FcscCore::detachObject(std::string object_name, std::string type)
{
  gazebo_ros_link_attacher::Attach attach_detach_srv;
  std::string link_name;
  bool success = true;

  if (type.find("moveit") != std::string::npos || type.find("both") != std::string::npos) {
    success *= move_group.detachObject(object_name);
  }
  if (type.find("gazebo") != std::string::npos || type.find("both") != std::string::npos) {
    if (object_name.find("onigiri") != std::string::npos) {
      link_name = "onigiri";
    } else if (object_name.find("bento") != std::string::npos) {
      link_name = "bento";
    } else if (object_name.find("drink") != std::string::npos) {
      link_name = "drink";
    }
    attach_detach_srv.request.model_name_1 = robot_name;
    attach_detach_srv.request.link_name_1 = attach_link;
    attach_detach_srv.request.model_name_2 = object_name;
    attach_detach_srv.request.link_name_2 = link_name;
    link_detacher_client.call(attach_detach_srv);
    success *= attach_detach_srv.response.ok;
  }

  return (success);
}

bool FcscCore::planArm(moveit::planning_interface::MoveGroupInterface::Plan& plan, int attempts)
{
  ROS_INFO("planArm");
  moveit_msgs::MoveItErrorCodes error_code;
  bool found_trjectory = false;

  waitForExecute();

  for (size_t i = 0; i < attempts; i++) {
    moveit::planning_interface::MoveGroupInterface::Plan current_plan;

    move_group.setStartStateToCurrentState();
    error_code = move_group.plan(current_plan);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("[planArm]%s", getMoveItErrorCodeString(error_code).c_str());
      continue;
    }

    // アームを振り回す軌道を避けるため、実行時間が短い軌道を選ぶ
    if (!found_trjectory ||
        current_plan.trajectory_.joint_trajectory.points.back().time_from_start < plan.trajectory_.joint_trajectory.points.back().time_from_start) {
      plan = current_plan;
      found_trjectory = true;
    }
  }

  return (found_trjectory);
}

bool FcscCore::asyncPlanArm(moveit::planning_interface::MoveGroupInterface::Plan& plan, int attempts)
{
  ROS_INFO("asyncPlanArm");

  moveit_msgs::MoveItErrorCodes error_code;
  bool found_trjectory = false;

  setStartState();

  for (size_t i = 0; i < attempts; i++) {
    moveit::planning_interface::MoveGroupInterface::Plan current_plan;

    error_code = move_group.plan(current_plan);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("[asyncPlanArm]:%s", getMoveItErrorCodeString(error_code).c_str());
      continue;
    }

    // アームを振り回す軌道を避けるため、実行時間が短い軌道を選ぶ
    if (!found_trjectory ||
        current_plan.trajectory_.joint_trajectory.points.back().time_from_start < plan.trajectory_.joint_trajectory.points.back().time_from_start) {
      plan = current_plan;
      found_trjectory = true;
    }
  }

  return (found_trjectory);
}

bool FcscCore::waitForExecute()
{
  ROS_INFO("[waitForExecute]:Start");

  ros::Rate loop_rate(10);
  while (is_executing_) {
    loop_rate.sleep();
  }

  ROS_INFO("[waitForExecute]:Finish");

  if (timeup_) {
    ROS_WARN("[waitForExecute]:Time's up!!!!. Go home.");
    move_group.setMaxVelocityScalingFactor(min_velocity);
    move_group.setNamedTarget("fold");
    move_group.move();
    move_group.setNamedTarget(container_center_pose_name);
    move_group.move();
    robot_hand.move(100);
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 0.5;
    goal_pose.pose.position.y = 1.55 + 0.4 + 0.2;
    goal_pose.pose.orientation.w = 1.0;
    mobile_robot.moveOnGlobalCoordinate(goal_pose);
    exit(-1);
  }

  if (!is_succeeded_) {
    ROS_ERROR("[waitForExecute]:Execution failed");
    move_group.getCurrentState()->copyJointGroupPositions(move_group.getName(), start_joint_group_positions_);
    is_succeeded_ = true;
    return (false);
  }
  return (true);
}

bool FcscCore::executeArm(moveit::planning_interface::MoveGroupInterface::Plan plan, int attemps)
{
  ROS_INFO("executeArm");
  moveit_msgs::MoveItErrorCodes error_code;

  if (!isTrajectoryContinuous(plan.trajectory_)) {
    ROS_ERROR("Trajectory is Not continuous");
  }

  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = plan.start_state_;
  display_trajectory.trajectory.push_back(plan.trajectory_);
  display_trajectory_publisher.publish(display_trajectory);

  waitForExecute();

  for (size_t i = 0; i < attemps; i++) {
    Stop(stop);
    error_code = move_group.execute(plan);
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("[executeArm]%s", getMoveItErrorCodeString(error_code).c_str());
      continue;
    }
    std::vector<double> current_joint_group_positions = plan.trajectory_.joint_trajectory.points[plan.trajectory_.joint_trajectory.points.size()-1].positions;
    start_joint_group_positions_ = current_joint_group_positions;
    return (true);
  }
  return (false);
}

bool FcscCore::asyncExecuteArm(moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  ROS_INFO("asyncExecuteArm");

  if (!isTrajectoryContinuous(plan.trajectory_)) {
    ROS_ERROR("Trajectory is Not continuous");
  }

  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = plan.start_state_;
  display_trajectory.trajectory.push_back(plan.trajectory_);
  display_trajectory_publisher.publish(display_trajectory);

  if (!waitForExecute()) {
    return (false);
  }

  std::vector<double> current_joint_group_positions = plan.trajectory_.joint_trajectory.points[plan.trajectory_.joint_trajectory.points.size()-1].positions;
  start_joint_group_positions_ = current_joint_group_positions;

  Stop(stop);

  move_group.asyncExecute(plan);
  is_executing_ = true;

  return (true);
}

bool FcscCore::planAndExecuteArm(int plan_attempts, int execute_attemps)
{
  ROS_INFO("planAndExecuteArm");
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 計画パート
  if (!planArm(plan, plan_attempts)) {
    return (false);
  }

  // 実行パート
  return (executeArm(plan, execute_attemps));
}

bool FcscCore::setTargetPose(geometry_msgs::PoseStamped pose_stamped)
{
  ROS_INFO("setTargetPose");

  geometry_msgs::PoseStamped target_pose_stamped;
  geometry_msgs::PoseStamped visualize_pose_stamped;

  // 目標手先位置を矢印で描画
  if (pose_stamped.header.frame_id != visual_frame_id_) {
    transformPose(visual_frame_id_, pose_stamped, visualize_pose_stamped, listener);
  } else {
    visualize_pose_stamped = pose_stamped;
  }
  visual_tools->publishArrow(visualize_pose_stamped.pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visual_tools->trigger();

  if (pose_stamped.header.frame_id != "base_footprint") {
    transformPose("base_footprint", pose_stamped, target_pose_stamped, listener);
  } else {
    target_pose_stamped = pose_stamped;
  }

  if (!move_group.setJointValueTarget(target_pose_stamped)) {
    ROS_ERROR("setTargetPose");
    return (false);
  }
  return (true);
}

bool FcscCore::moveArm(geometry_msgs::PoseStamped pose_stamped)
{
  ROS_INFO("moveArm");

  if (!setTargetPose(pose_stamped)) {
    return (false);
  }

  return (planAndExecuteArm());
}

bool FcscCore::moveArm(string target_name)
{
  ROS_INFO("moveArm [%s]", target_name.c_str());

  if (!move_group.setNamedTarget(target_name)) {
    ROS_ERROR("Error[moveArm]:setJointValueTarget");
    return (false);
  }

  return (planAndExecuteArm());
}

bool FcscCore::asyncMoveArm(string target_name)
{
  ROS_INFO("[asyncMoveArm]:pose name [%s]", target_name.c_str());

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (!move_group.setNamedTarget(target_name)) {
    ROS_ERROR("[asyncMoveArm]:setNamedTarget");
    return (false);
  }

  if (!asyncPlanArm(plan)) {
    ROS_ERROR("[asyncPlanArm]");
    return (false);
  }

  return (asyncExecuteArm(plan));
}

bool FcscCore::asyncMoveArm(geometry_msgs::PoseStamped pose_stamped, int plan_attempts)
{
  ROS_INFO("asyncMoveArm");
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (!setTargetPose(pose_stamped)) {
    return (false);
  }

  if (!asyncPlanArm(plan, plan_attempts)) {
    return (false);
  }

  return (asyncExecuteArm(plan));
}

bool FcscCore::moveArmCartesianPath(std::string frame_id, std::vector<geometry_msgs::Pose> waypoints, double eef_step, double jump_threshold, bool avoid_collisions, int planning_num, double valid_fraction)
{
  ROS_INFO("moveArmCartesianPath");
  moveit_msgs::Constraints path_constraints_tmp;

  return (moveArmCartesianPath(frame_id, waypoints,eef_step, jump_threshold, path_constraints_tmp, avoid_collisions, planning_num, valid_fraction));
}

bool FcscCore::moveArmCartesianPath(std::string frame_id, std::vector<geometry_msgs::Pose> waypoints, double eef_step, double jump_threshold, moveit_msgs::Constraints path_constraints, bool avoid_collisions, int planning_num, double valid_fraction)
{
  ROS_INFO("moveArmCartesianPath(constraint)");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::MoveItErrorCodes error_code;

  if (planning_num <= 0) {
    planning_num = 5;
  }

  waitForExecute();

  move_group.setPoseReferenceFrame(frame_id);
  for (int i = 0; i < planning_num; i++) {
    move_group.setStartStateToCurrentState();
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, plan.trajectory_, path_constraints, avoid_collisions, &error_code);
    if (fraction < valid_fraction) {
      ROS_ERROR("moveArmCartesianPath:failed %.2f%% acheived code[%s]", fraction * 100.0, getMoveItErrorCodeString(error_code).c_str());
      continue;
    }
    ROS_INFO("moveArmCartesianPath:%.2f%% acheived",fraction * 100.0);
    if (!isTrajectoryContinuous(plan.trajectory_)) {
      ROS_ERROR("Trajectory is Not continuous");
      return (false);
    }
    Stop(stop);
    error_code = move_group.asyncExecute(plan);
    is_executing_ = true;
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR("[moveArmCartesianPath]execute [%s]", getMoveItErrorCodeString(error_code).c_str());
      move_group.stop();
      continue;
    }
    for (size_t j = 0; j < plan.trajectory_.joint_trajectory.points[plan.trajectory_.joint_trajectory.points.size()-1].positions.size(); j++) {
      start_joint_group_positions_[j] = plan.trajectory_.joint_trajectory.points[plan.trajectory_.joint_trajectory.points.size()-1].positions[j];
    }
    return (true);
  }
  ROS_ERROR("moveArmCartesianPath:failed");
  return (false);
}

bool FcscCore::isTrajectoryContinuous(moveit_msgs::RobotTrajectory trajectory)
{

  double threshold_rad = 100.0 * M_PI / 180.0;

  for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); i++) {
    for (size_t j = 0; j < trajectory.joint_trajectory.points[0].positions.size(); j++) {
      double joint_position = trajectory.joint_trajectory.points[i].positions[j];
      double pre_joint_position = trajectory.joint_trajectory.points[i - 1].positions[j];

      // 一つ前の関節位置と比較，しきい値以上変化があるなら不連続
      if (abs(joint_position - pre_joint_position) >= threshold_rad) {
        return (false);
      }
    }
  }
  return (true);
}

bool FcscCore::setTouchLinksParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    for (int i = 0; i < params.size(); i++) {
      touch_links_.push_back(params[i]);
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {

    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }
  return true;
}

void FcscCore::serverStart()
{
  ROS_INFO("server start");

  ros::AsyncSpinner spinner(2);
  ros::Rate loop_rate_100hz(100);
  spinner.start();

  // 現在の関節位置設定
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getName(), start_joint_group_positions_);

  ros::Rate loop_rate_10hz(10);
  for (size_t i = 0; i < 2; i++) {
    std_msgs::Bool msg;
    msg.data = false;
    enable_shelf_detection_publisher.publish(msg);
    enable_object_detection_publisher.publish(msg);
    loop_rate_10hz.sleep();
  }

  while (ros::ok()) {
    loop_rate_100hz.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "fcsc_moveit_core");
  ros::NodeHandle private_nh("~");
  string group_name;
  string ee_name;

  private_nh.param<std::string>("group_name", group_name, "manipulator");
  private_nh.param<std::string>("ee_name", ee_name, "endeffector");

  FcscCore fcsc_core(group_name, ee_name);
  fcsc_core.serverStart();

  return 0;
}
