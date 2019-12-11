#include <ros/ros.h>
#include <cstring>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_data_handler/trajectory_data_handler.h>
using namespace std;

int main(int argc,char** argv){
    ros::init(argc,argv, "orbit_generation");
    ros::NodeHandle node_handle;
    std::string str("drink");
    TrajectoryDataHandler handler;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface   move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface  planning_scene_interface;

    // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    // std::ostream_iterator<std::string>(std::cout, ", "));

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    geometry_msgs::Pose target_pose1;
    move_group.setPoseTarget(target_pose1);
    target_pose1.position.x = 1.2;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = (bool)move_group.plan(plan);

    if (!success) {
        ROS_INFO("plan error1");
        return(false);
    }
    
    handler.saveTrajectoryToBag(plan, str);
    
    success = (bool) move_group.execute(plan);
      if (!success) {
        ROS_INFO("execute error1");
        for ( int i= 1; i <= 5; i++) {
          move_group.setStartStateToCurrentState();
          move_group.plan(plan);
          success = (bool) move_group.execute(plan);
          if (!success){
            ROS_INFO("execute  error1 %d回目", (int) i+1 );
            continue;
          } else {
            ROS_INFO("execute  success1 %d回目", (int) i+1 );
            break;
          }
        }
      } else {
        ROS_INFO("execute  success1");
    }

    ros::shutdown();
    return 0;
}
