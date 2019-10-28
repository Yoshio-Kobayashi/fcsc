//MoveIt上の障害物の座標系を出す
//CollisionObject と AttachedCollisionObject

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <XmlRpcException.h>
#include <typeinfo>

using namespace std;

class SandwichTfBroadcaster
{
private:
  ros::NodeHandle nh;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  ros::ServiceClient planning_scene_client;

  void sendTransform(string parent_frame_id, string child_frame_id, geometry_msgs::Pose pose);

public:
  SandwichTfBroadcaster();
  void start();
};

SandwichTfBroadcaster::SandwichTfBroadcaster()
{
  planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
}

void SandwichTfBroadcaster::sendTransform(string parent_frame_id, string child_frame_id, geometry_msgs::Pose pose)
{
  transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  transform.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) );
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));
}

void SandwichTfBroadcaster::start()
{
  moveit_msgs::GetPlanningScene srv;
  geometry_msgs::Pose pose;
  // srv.request.components.components =  moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  while (ros::ok()) {
    //collision objectの座標系を出す
    planning_scene_client.call(srv);
    for (int i = 0; i < srv.response.scene.world.collision_objects.size(); i++) {
      bool found_sanwich = (srv.response.scene.world.collision_objects[i].id.find("sandwich") != std::string::npos);
      if (!found_sanwich) {
        continue;
      }
      std::string sandwich_id(srv.response.scene.world.collision_objects[i].id);
      if (srv.response.scene.world.collision_objects[i].mesh_poses.size() > 0) {
        pose = srv.response.scene.world.collision_objects[i].mesh_poses[0];
      } else {
        pose = srv.response.scene.world.collision_objects[i].primitive_poses[0];
      }
      sendTransform(srv.response.scene.world.collision_objects[i].header.frame_id, sandwich_id, pose);
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sandwich_tf_broadcaster");

  SandwichTfBroadcaster sandwich_tf_broadcaster;

  ROS_INFO("sandwich tf start");
  sandwich_tf_broadcaster.start();
  return 0;
}
