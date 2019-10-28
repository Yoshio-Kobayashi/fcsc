#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <XmlRpcException.h>
#include <typeinfo>
#include <std_srvs/Empty.h>

using namespace std;

const int LOWER     = 0;
const int UPPER     = 1;

const int ONIGIRI   = 0;
const int DRINK     = 1;
const int BENTO     = 2;
const int SANDWICH  = 3;

struct Shelf {
  double width;
  double depth;
  std::vector<double> installation_heights;
  double installation_height[2];
  double stopper_height;
  double thickness;
  double height;
  bool   no_upper;
};

struct StockingArea {
  string parent_frame_id;
  string child_frame_id;
  geometry_msgs::Pose pose;
};

class ShelfTfBroadcaster
{
private:
  ros::NodeHandle nh;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  ros::ServiceClient planning_scene_client;
  ros::ServiceServer update_shelf_pose_server;
  Shelf shelves[2];
  StockingArea stocking_area[4];
  bool request_update;

  void readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value);
  bool setShelfSizeParams(XmlRpc::XmlRpcValue &params, Shelf &shelf);
  bool setStockingAreaParams(XmlRpc::XmlRpcValue &params, StockingArea &stocking_area);
  void sendTransform(string parent_frame_id, string child_frame_id, geometry_msgs::Pose pose);
  bool updateCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

public:
  ShelfTfBroadcaster();
  void start();
};

void ShelfTfBroadcaster::sendTransform(string parent_frame_id, string child_frame_id, geometry_msgs::Pose pose)
{
  transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  transform.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) );
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));
}

void ShelfTfBroadcaster::readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value)
{
  if (params.hasMember(param_name))
  {
    if (params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt)
      *value = (int)params[param_name];
    else
      *value = (double)params[param_name];
  }
}

bool ShelfTfBroadcaster::setStockingAreaParams(XmlRpc::XmlRpcValue &params, StockingArea &stocking_area)
{
  try
  {
    stocking_area.parent_frame_id = (string)params["parent_frame_id"];
    stocking_area.child_frame_id = (string)params["child_frame_id"];
    stocking_area.pose.position.x = (double)params["position"]["x"];
    stocking_area.pose.position.y = (double)params["position"]["y"];
    stocking_area.pose.orientation.w = 1.0;
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ShelfTfBroadcaster::setShelfSizeParams(XmlRpc::XmlRpcValue &params, Shelf &shelf)
{
  try
  {
    readXmlParam(params, "width", &shelf.width);
    readXmlParam(params, "depth", &shelf.depth);
    readXmlParam(params, "stopper_height", &shelf.stopper_height);
    readXmlParam(params, "thickness", &shelf.thickness);
    readXmlParam(params, "height", &shelf.height);
    for (size_t i = 0; i < params["installation_heights"].size(); i++) {
      shelf.installation_heights.push_back((double)params["installation_heights"][i]);
    }
    // shelf.no_upper = (bool)params["no_upper"];
    // readXmlParam(params["installation_height"], "lower", &shelf.installation_height[LOWER]);
    // readXmlParam(params["installation_height"], "upper", &shelf.installation_height[UPPER]);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

ShelfTfBroadcaster::ShelfTfBroadcaster():
request_update(true)
{
  XmlRpc::XmlRpcValue params;

  nh.getParam("shelf_size", params);
  setShelfSizeParams(params["shelfA"], shelves[0]);
  setShelfSizeParams(params["shelfB"], shelves[1]);

  nh.getParam("stocking_area", params);
  setStockingAreaParams(params["onigiri"], stocking_area[ONIGIRI]);
  setStockingAreaParams(params["drink"], stocking_area[DRINK]);
  setStockingAreaParams(params["bento"], stocking_area[BENTO]);
  setStockingAreaParams(params["sandwich"], stocking_area[SANDWICH]);
  planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  update_shelf_pose_server = nh.advertiseService("update_shelf_pose", &ShelfTfBroadcaster::updateCB, this);
}

bool ShelfTfBroadcaster::updateCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  XmlRpc::XmlRpcValue params;
  nh.getParam("stocking_area", params);
  setStockingAreaParams(params["onigiri"], stocking_area[ONIGIRI]);
  setStockingAreaParams(params["drink"], stocking_area[DRINK]);
  setStockingAreaParams(params["bento"], stocking_area[BENTO]);
  setStockingAreaParams(params["sandwich"], stocking_area[SANDWICH]);

  request_update = true;
  return (true);
}

void ShelfTfBroadcaster::start()
{
  moveit_msgs::GetPlanningScene srv;
  geometry_msgs::Pose pose;
  Shelf shelf;
  ros::Rate loop_rate(5);
  ros::AsyncSpinner spinner(1);

  spinner.start();

  srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  while (ros::ok()) {
    //collision objectの座標系を出す
    if (request_update) {
      planning_scene_client.call(srv);
      request_update = false;
    }

    for (int i = 0; i < srv.response.scene.world.collision_objects.size(); i++) {
      //rviz上にCollisionObjectとして出ている棚の座標系を出す
      std::string shelf_id(srv.response.scene.world.collision_objects[i].id);

      if (shelf_id.find("shelfB") != std::string::npos) {
        shelf = shelves[1];
      } else if (shelf_id.find("shelf") != std::string::npos) {
        shelf = shelves[0];
      } else {
        continue;
      }

      if (srv.response.scene.world.collision_objects[i].mesh_poses.size() > 0) {
        pose = srv.response.scene.world.collision_objects[i].mesh_poses[0];
      } else {
        pose = srv.response.scene.world.collision_objects[i].primitive_poses[0];
      }
      sendTransform(srv.response.scene.world.collision_objects[i].header.frame_id, shelf_id, pose);
      for (int j = 0; j < shelf.installation_heights.size(); j++) {
        std::stringstream ss;
        ss << (j+1);
        pose.position.z = shelf.installation_heights[j];
        pose.position.x = -shelf.depth / 2.0;
        pose.position.y = -shelf.width / 2.0;
        pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
        pose.orientation.w = 1.0;
        sendTransform(shelf_id, shelf_id + "_board_" + ss.str(), pose);
      }

      if (shelf_id == "shelfA") {
        sendTransform(stocking_area[ONIGIRI].parent_frame_id, stocking_area[ONIGIRI].child_frame_id, stocking_area[ONIGIRI].pose);
        sendTransform(stocking_area[DRINK].parent_frame_id, stocking_area[DRINK].child_frame_id, stocking_area[DRINK].pose);
        sendTransform(stocking_area[BENTO].parent_frame_id, stocking_area[BENTO].child_frame_id, stocking_area[BENTO].pose);
      } else if (shelf_id == "shelfB") {
        sendTransform(stocking_area[SANDWICH].parent_frame_id, stocking_area[SANDWICH].child_frame_id, stocking_area[SANDWICH].pose);
      }
    }
    loop_rate.sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "shelf_tf_broadcaster");

  ShelfTfBroadcaster shelf_tf_broadcaster;

  ROS_INFO("shelf tf start");
  shelf_tf_broadcaster.start();
  return 0;
}
