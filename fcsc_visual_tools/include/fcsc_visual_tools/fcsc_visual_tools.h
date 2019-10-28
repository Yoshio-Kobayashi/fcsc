#ifndef FCSC_VISUAL_TOOLS
#define FCSC_VISUAL_TOOLS

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <XmlRpcException.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>
#include <fcsc_msgs/RecognizedObjectArray.h>
#include <std_srvs/Empty.h>

// tf
#include <tf/transform_listener.h>

namespace moveit_visual_tools
{
  class FCSCVisualTools
  {
    private:
      struct ShelfSize {
        double width;
        double depth;
        std::vector<double> installation_heights;
        double stopper_height;
        double stopper_thickness;
        double thickness;
        double height;
        bool   no_upper;
      };

      moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
      ros::Subscriber detected_object_array_sub;
      ros::Publisher  collision_object_pub;
      shape_msgs::Mesh onigiri_mesh;
      shape_msgs::Mesh bento_mesh;
      shape_msgs::Mesh drink_mesh;
      shape_msgs::Mesh sandwich_mesh;
      fcsc_msgs::RecognizedObjectArray object_array;
      ShelfSize shelf_sizes[2];
      ros::ServiceClient planning_scene_client;
      ros::ServiceClient update_shelf_pose_client;
      tf::TransformListener listener;

      void readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value);
      bool setShelfSizeParams(XmlRpc::XmlRpcValue &params, ShelfSize &shelf_size);
      shape_msgs::Mesh createMesh(const std::string mesh_path);
      void publishCollisionObject(const moveit_msgs::CollisionObject object_msg);
      void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out);

    public:
      FCSCVisualTools();
      void deleteObject(std::string object_name);
      void removeAllProducts(const fcsc_msgs::RecognizedObjectArray object_array);
      void spawnProduct(std::string object_name, geometry_msgs::PoseStamped object_pose);
      void spawnProduct(const fcsc_msgs::RecognizedObject object);
      void publishCollisionMesh(const geometry_msgs::Pose object_pose, const std::string object_frame_id, const std::string object_name, const shape_msgs::Mesh mesh_msg);
      void spawnShelf(std::string shelf_name, geometry_msgs::PoseStamped shelf_pose);
      void contactProduct(std::string object_name, std::string contact_object_name, geometry_msgs::PoseStamped current_object_pose, int object_count=0);
  };
}
#endif
