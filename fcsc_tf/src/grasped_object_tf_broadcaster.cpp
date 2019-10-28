//Rviz上のAttachedCollisionObjectの座標系を出す

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "grasped_object_tf_broadcaster");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::ServiceClient planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  moveit_msgs::GetPlanningScene srv;
  geometry_msgs::Pose pose;

  ros::Rate rate(50.0);
  while (nh.ok()){
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    planning_scene_client.call(srv);

    //attached_collision objectの座標系を出す
    for (int i = 0; i < srv.response.scene.robot_state.attached_collision_objects.size(); i++) {
      //rviz上にある物体の座標系を出す
      if (srv.response.scene.robot_state.attached_collision_objects[i].object.primitive_poses.size() > 0) {//primitiveのとき
        pose = srv.response.scene.robot_state.attached_collision_objects[i].object.primitive_poses[0];
      } else {//meshのとき
        pose = srv.response.scene.robot_state.attached_collision_objects[i].object.mesh_poses[0];
      }
      transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
      transform.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), srv.response.scene.robot_state.attached_collision_objects[i].object.header.frame_id, "grasped_" + srv.response.scene.robot_state.attached_collision_objects[i].object.id));
    }
    rate.sleep();
  }
  return 0;
}
