#ifndef FCSC_GRASP_PLANNING
#define FCSC_GRASP_PLANNING
//把持計画用

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit_msgs/GraspPlanning.h>
#include <fcsc_msgs/Grasp.h>

// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

const int particle_size         = 8;
const double finger_interval    = 0.07; // 上下のグリッパの感覚[m]
const double finger_width       = 0.03; // 指の幅[m]
const double palm_to_fingertip  = 0.093; // 手のひらから指先までの距離[m]
const double onigiri_height     = 0.07;
const double drink_height       = 0.108;
const double bento_height       = 0.045;
const double sandwich_height    = 0.1;

using namespace std;

class GraspPlanner {
private:
  string eef_link;
  string eef_wrap_link;
  bool use_simulator;

public:
  GraspPlanner ();

  void generateGraspPose(string name, std::vector<fcsc_msgs::Grasp> *grasps);
  void generateGraspPoseForOnigiri(string name, std::vector<fcsc_msgs::Grasp> *grasps);
  void generateGraspPoseForDrink(string name, std::vector<fcsc_msgs::Grasp> *grasps);
  void generateGraspPoseForBento(string name, std::vector<fcsc_msgs::Grasp> *grasps);
  void generateGraspPoseForSandwich(string name, std::vector<fcsc_msgs::Grasp> *grasps);

  void generateLocationPose(std::string object_name, std::string plane_frame_id,  std::vector<moveit_msgs::PlaceLocation> *locations);
  void generateLocationPoseForOnigiri(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations);
  void generateLocationPoseForDrink(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations);
  void generateLocationPoseForBento(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations);
  void generateLocationPoseForSandwich(string plane_frame_id, std::vector<moveit_msgs::PlaceLocation> *locations);
};
#endif
