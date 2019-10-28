#ifndef SMACH_DAIHEN_UR5_MOVEIT_CORE
#define SMACH_DAIHEN_UR5_MOVEIT_CORE

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <typeinfo>
#include <algorithm>
#include <XmlRpcException.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_ros_link_attacher/Attach.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

// fcsc
#include "fcsc_moveit/grasp_planning.h"
#include "fcsc_moveit/mobile_robot.h"
#include "fcsc_moveit/robot_hand.h"

#include <fcsc_visual_tools/fcsc_visual_tools.h>
#include <fcsc_visual_tools/random_sandwich_generator.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <fcsc_msgs/GoToNearShelf.h>
#include <fcsc_msgs/DetectObject.h>
#include <fcsc_msgs/MoveInitialPose.h>
#include <fcsc_msgs/DetectShelf.h>
#include <fcsc_msgs/DetectSandwich.h>
#include <fcsc_msgs/DetectProduct.h>
#include <fcsc_msgs/GetBasePosition.h>
#include <fcsc_msgs/SortOrderManipulation.h>
#include <fcsc_msgs/Manipulate.h>
#include <fcsc_msgs/EstimateSandwichPosition.h>
#include <fcsc_msgs/Grasp.h>
#include <fcsc_msgs/DetectAndRecoverSandwich.h>
#include <fcsc_msgs/FaceupSandwich.h>

using namespace std;

static const std::string ROBOT_DESCRIPTION="robot_description";

enum ProductType {
  ONIGIRI,
  DRINK,
  BENTO,
  SANDWICH,

  NUM_OF_PRODUCT_TYPE
};

enum ObjectSortType {
  OBJECT_SORT_X,
  OBJECT_SORT_Y,
  OBJECT_SORT_Z,
  OBJECT_SORT_DIST,

  NUM_OF_OBJECT_SORT_TYPE
};

struct StockingArea {
  string child_frame_id;
  string parent_frame_id;
  geometry_msgs::Point min_corner;
  geometry_msgs::Point max_corner;
};

struct ProductGrasp {
  std::vector<fcsc_msgs::Grasp> grasps;
};

struct ObjectPlacement {
  int type;
  std::vector<geometry_msgs::PoseStamped> placements;
};

struct ScrapSandwichInfo {
  std::string name;
  bool detected;
  bool recoverd;
  int picking_count;
};


typedef pair<int, double> ass_arr;
// 昇順
static bool sort_less(const ass_arr& left, const ass_arr& right)
{
  return left.second < right.second;
}
// 降順
static bool sort_greater(const ass_arr& left, const ass_arr& right)
{
  return left.second > right.second;
}

static double distance_square(double x, double y)
{
  return (x*x + y*y);
}

typedef pair<int, geometry_msgs::Pose> pose_ass_arr;
extern ObjectSortType object_sort_type;

static void switch_sort_type(pose_ass_arr left, pose_ass_arr right, double &l_value, double &r_value)
{
  switch (object_sort_type) {
    case OBJECT_SORT_X: l_value = left.second.position.x; r_value = right.second.position.x; break;
    case OBJECT_SORT_Y: l_value = left.second.position.y; r_value = right.second.position.y; break;
    case OBJECT_SORT_Z: l_value = left.second.position.z; r_value = right.second.position.z; break;
    case OBJECT_SORT_DIST: {
      l_value = distance_square(left.second.position.x, left.second.position.y);
      r_value = distance_square(right.second.position.x, right.second.position.y);
      break;
    }
  }
}

static bool object_sort_less(const pose_ass_arr& left, const pose_ass_arr& right)
{
  double l_value, r_value;

  switch_sort_type(left, right, l_value, r_value);
  return (l_value < r_value);
}

static bool object_sort_greater(const pose_ass_arr& left, const pose_ass_arr& right)
{
  double l_value, r_value;

  switch_sort_type(left, right, l_value, r_value);
  return (l_value > r_value);
}

static void Stop(bool is_stop=true)
{
  if (!is_stop) {
    return;
  }

  int ans;
  while (ros::ok()) {
    cerr << "Move? Yes:1"  << endl;
    cin >> ans;
    if (!cin.fail() && ans >= 1) {
      break;
    }
    cin.clear();
    cin.ignore(10000, '\n');
  }
}

class FcscCore {
public:
  FcscCore(string group_name, string ee_name);
  void serverStart();

private:

  /************************** SMACH **********************************/
  // アームの作業関連
  ros::ServiceServer detect_product_server;
  ros::ServiceServer detect_shelf_server;
  ros::ServiceServer detect_sandwich_server;
  ros::ServiceServer pickup_product_server;
  ros::ServiceServer place_product_server;
  ros::ServiceServer return_product_server;
  ros::ServiceServer pickup_sandwich_server;
  ros::ServiceServer faceup_standing_sandwiches_server;
  ros::ServiceServer recover_sandwich_server;
  ros::ServiceServer detect_and_recover_sandwich_server;
  ros::ServiceServer move_initial_pose_server;
  ros::ServiceServer sort_order_picking_sandwiches_server;
  ros::ServiceServer change_sandwich_pose_server;

  // 台車の移動関連
  ros::ServiceServer goto_stocking_base_pos_server;
  ros::ServiceServer goto_faceup_base_pos_server;
  ros::ServiceServer goto_near_shelf_server;
  ros::ServiceServer goto_home_pos_server;

  bool detectProduct(fcsc_msgs::DetectProduct::Request &req, fcsc_msgs::DetectProduct::Response &res);
  bool detectShelf(fcsc_msgs::DetectShelf::Request &req, fcsc_msgs::DetectShelf::Response &res);
  bool detectSandwich(fcsc_msgs::DetectSandwich::Request &req, fcsc_msgs::DetectSandwich::Response &res);
  bool pickup(std::string object_name, fcsc_msgs::Grasp grasp, const moveit_msgs::Constraints& constraints=moveit_msgs::Constraints(), bool check_ik=true);
  bool place(std::string object_name, moveit_msgs::PlaceLocation location, const moveit_msgs::Constraints& constraint=moveit_msgs::Constraints());
  bool pickupProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool placeProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool returnProduct(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool pickupSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool faceupSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool faceupSandwiches(fcsc_msgs::FaceupSandwich::Request &req, fcsc_msgs::FaceupSandwich::Response &res);
  bool changeSandwichPose(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool recoverSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res);
  bool detectAndRecoverSandwich(fcsc_msgs::DetectAndRecoverSandwich::Request &req, fcsc_msgs::DetectAndRecoverSandwich::Response &res);
  bool moveInitialPose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool goToStockingBasePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool goToFaceupBasePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool goToNearShelf(fcsc_msgs::GoToNearShelf::Request &req, fcsc_msgs::GoToNearShelf::Response &res);
  bool goToHomePosition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool sortOrderPickingSandwiches(fcsc_msgs::SortOrderManipulation::Request &req, fcsc_msgs::SortOrderManipulation::Response &res);
  /*******************************************************************/

  ros::Publisher                          display_trajectory_publisher;
  ros::Publisher                          enable_shelf_detection_publisher;
  ros::Publisher                          enable_object_detection_publisher;

  ros::Subscriber                         trajectory_result_subscriber;
  ros::Subscriber                         time_up_subscriber;

  ros::ServiceClient                      planning_scene_client;
  ros::ServiceClient                      compute_ik_client;
  ros::ServiceClient                      detect_object_client;
  ros::ServiceClient                      detect_shelf_client;
  ros::ServiceClient                      link_attacher_client;
  ros::ServiceClient                      link_detacher_client;

  tf::TransformListener                   listener;

  moveit::planning_interface::MoveGroupInterface   move_group;
  planning_scene_monitor::PlanningSceneMonitor     planning_scene_monitor;

  MobileRobot                             mobile_robot;

  RobotHand                               robot_hand;

  GraspPlanner                            grasp_planner;

  RandomSandwichGenerator                 random_sandwich_generator;

  bool                                    is_executing_;
  bool                                    is_succeeded_;
  bool                                    timeup_;
  bool                                    use_shelf_marker_detection_;
  moveit_visual_tools::FCSCVisualTools    fcsc_visual_tools;
  rviz_visual_tools::RvizVisualToolsPtr   visual_tools;
  std::string                             robot_name;
  std::string                             attach_link;
  std::string                             visual_frame_id_;
  std::string                             container_center_pose_name;
  std::string                             stock_pose_name;
  std::string                             recover_pose_name;
  std::string                             attach_detach_type;
  std::string                             shelfB_board_name;
  std::string                             shelfB_name;
  double                                  shelf_depth;
  double                                  shelf_width;
  double                                  min_velocity;
  double                                  normal_velocity;
  double                                  max_velocity;
  ProductGrasp                            product_grasps[NUM_OF_PRODUCT_TYPE];
  StockingArea                            stocking_area[NUM_OF_PRODUCT_TYPE];
  std::vector<int>                        bento_placement_index;
  std::vector<int>                        drink_placement_index;
  std::vector<int>                        onigiri_placement_index;
  std::vector<double>                     start_joint_group_positions_;
  std::vector<std::string>                touch_links_;
  std::vector<std::string>                scrap_sandwich_ids;
  std::vector<ObjectPlacement>            object_placements;
  bool stop;


  /******************************** Moving RobotHand ************************************/
  bool graspObject(std::string object_name, double joint_position, double effort=50, int control=fcsc_msgs::Grasp::BOTH);
  bool releaseObject(std::string object_name, double joint_position, double effort=50);
  /*******************************************************************************/

  /**************************** Moving RobotArm ****************************/
  bool planArm(moveit::planning_interface::MoveGroupInterface::Plan& plan, int attemps=5);
  bool executeArm(moveit::planning_interface::MoveGroupInterface::Plan plan, int attemps=5);
  bool planAndExecuteArm(int plan_attempts=5, int execute_attemps=5);
  bool setTargetPose(geometry_msgs::PoseStamped pose_stamped);
  bool moveArm(geometry_msgs::PoseStamped pose);
  bool moveArm(string target_name);
  bool moveArmCartesianPath(std::string frame_id, std::vector<geometry_msgs::Pose> waypoints, double eef_step=0.01, double jump_threshold=0.0, bool avoid_collisions=true, int planning_num=10, double valid_fraction=0.9);
  bool moveArmCartesianPath(std::string frame_id, std::vector<geometry_msgs::Pose> waypoints, double eef_step, double jump_threshold, moveit_msgs::Constraints path_constraints, bool avoid_collisions=true, int planning_num=10, double valid_fraction=0.9);
  bool asyncPlanArm(moveit::planning_interface::MoveGroupInterface::Plan& plan, int attemps=5);
  bool asyncExecuteArm(moveit::planning_interface::MoveGroupInterface::Plan plan);
  bool asyncMoveArm(std::string target_name);
  bool asyncMoveArm(geometry_msgs::PoseStamped pose, int plan_attempts=5);
  bool waitForExecute();
  /*****************************************************************************/

  /************************ Parameter Setting ************************/
  bool setTouchLinksParams(XmlRpc::XmlRpcValue &params);
  bool setStockingAreaParams(XmlRpc::XmlRpcValue &params, StockingArea &stocking_area);
  bool setScrapSandwichIdParams(XmlRpc::XmlRpcValue &params);
  /*************************************************************/

  bool attachObject(std::string object_name, std::string type);
  bool detachObject(std::string object_name, std::string type);

  bool returnObjectToContainer(fcsc_msgs::RecognizedObject object);

  /**************************** Inverse Kinematics **********************************/
  bool computeIK(geometry_msgs::PoseStamped pose_stamped, moveit_msgs::RobotState robot_state, sensor_msgs::JointState &joint_state, bool avoid_collisions=true, const moveit_msgs::Constraints& constraints=moveit_msgs::Constraints());
  bool computeIK(geometry_msgs::PoseStamped pose_stamped, moveit_msgs::RobotState robot_state, const moveit_msgs::Constraints& constraints=moveit_msgs::Constraints());
  bool computeIK(geometry_msgs::PoseStamped pose_stamped, bool avoid_collisions=true, const moveit_msgs::Constraints& constraints=moveit_msgs::Constraints());
  bool computeIK(geometry_msgs::PoseStamped pose_stamped, sensor_msgs::JointState &joint_state, const moveit_msgs::Constraints& constraints=moveit_msgs::Constraints());
  /******************************************************************/

  geometry_msgs::PoseStamped getCurrentEEfPose(std::string frame_id="");
  void getCurrentRobotStateMsg(moveit_msgs::RobotState& robot_state);
  bool getCurrentObjectPose(std::string object_name, std::string frame_id, geometry_msgs::PoseStamped& object_pose_stamped);
  geometry_msgs::PoseStamped getTargetPlacePoseFromObject(std::string object_name, geometry_msgs::PoseStamped target_object_pose);
  void setProductPlacement(std::vector<fcsc_msgs::RecognizedObject> objects);
  void sortGraspPoseIndex(ProductType type, std::string object_name, std::vector<int> &grasp_indices);
  void setStartState();

  bool isTrajectoryContinuous(moveit_msgs::RobotTrajectory trajectory);

  void trajectoryResultCB(const moveit_msgs::ExecuteTrajectoryActionResult result_msg);

  void timeUpCB(const std_msgs::Empty msg);

  void sortSandwichGrasps(std::string sandwich_name, std::vector<int>& grasp_indices, int manipulation_type=0);
};
#endif
