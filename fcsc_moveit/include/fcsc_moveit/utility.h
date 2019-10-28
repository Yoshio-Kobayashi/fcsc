#ifndef FCSC_UTITILY
#define FCSC_UTITILY

#include <iostream>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <fcsc_msgs/RecognizedObject.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

template <class X> static int vectorFinder(std::vector<X> vec, X value) {
  typename std::vector<X>::iterator itr = std::find(vec.begin(), vec.end(), value);
  size_t index = std::distance( vec.begin(), itr );
  if (index != vec.size()) // 発見できたとき
    return (index);
  else // 発見できなかったとき
    return (-1);
}

static void getRPYFromQuaternion(geometry_msgs::Quaternion q, double& r, double& p, double& y)
{
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(r, p, y, 1);
}

static std::string getMoveItErrorCodeString(moveit_msgs::MoveItErrorCodes error_code)
{
  switch (error_code.val) {
    // overall behavior
    case 1: return("SUCCESS");
    case 99999: return("FAILURE");
    case -1: return("PLANNING_FAILED");
    case -2: return("INVALID_MOTION_PLAN");
    case -3: return("MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
    case -4: return("CONTROL_FAILED");
    case -5: return("UNABLE_TO_AQUIRE_SENSOR_DATA");
    case -6: return("TIMED_OUT");
    case -7: return("PREEMPTED");
    // planning & kinematics request errors
    case -10: return("START_STATE_IN_COLLISION");
    case -11: return("START_STATE_VIOLATES_PATH_CONSTRAINTS");
    case -12: return("GOAL_IN_COLLISION");
    case -13: return("GOAL_VIOLATES_PATH_CONSTRAINTS");
    case -14: return("GOAL_CONSTRAINTS_VIOLATED");
    case -15: return("INVALID_GROUP_NAME");
    case -16: return("INVALID_GOAL_CONSTRAINTS");
    case -17: return("INVALID_ROBOT_STATE");
    case -18: return("INVALID_LINK_NAME");
    case -19: return("INVALID_OBJECT_NAME");
    // system errors
    case -21: return("FRAME_TRANSFORM_FAILURE");
    case -22: return("COLLISION_CHECKING_UNAVAILABLE");
    case -23: return("ROBOT_STATE_STALE");
    case -24: return("SENSOR_INFO_STALE");
    // kinematics errors
    case -31: return("NO_IK_SOLUTION");
    default: return("undefined error code");
  }
}

static std::string getObjectStateString(fcsc_msgs::RecognizedObject obj)
{
  switch (obj.state) {
    case fcsc_msgs::RecognizedObject::STANDING:   return("STANDING");
    case fcsc_msgs::RecognizedObject::LYING:      return("LYING");
    case fcsc_msgs::RecognizedObject::BOTTOM:     return("BOTTOM");
    case fcsc_msgs::RecognizedObject::BACK:       return("BACK");
    case fcsc_msgs::RecognizedObject::RIGHT_SIDE: return("RIGHT_SIDE");
    case fcsc_msgs::RecognizedObject::LEFT_SIDE:  return("LEFT_SIDE");
    case fcsc_msgs::RecognizedObject::FRONT:      return("FRONT");
    default:                                      return("undefined state");
  }
}

// tf utility
static void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out, tf::TransformListener& listener)
{
  //we'll just use the most recent transform available for our simple example
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

static tf::StampedTransform getTransform(std::string parent_frame_id, std::string child_frame_id, tf::TransformListener& listener)
{
  tf::StampedTransform transform;
  ros::Rate rate(10);

  while (1) {
    try {
      listener.lookupTransform(parent_frame_id, child_frame_id, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
    rate.sleep();
  }

  return (transform);
}

//リンクreference_linkの位置・姿勢がreference_poseのときのリンクtarget_linkの位置・姿勢を求める
//返却するPoseStampedのframe_idはreference_poseのframe_id
static geometry_msgs::PoseStamped getPoseStamped(geometry_msgs::PoseStamped reference_pose, std::string reference_link, std::string target_link, tf::TransformListener& listener)
{
  tf::Transform transform_base_to_ref;
  geometry_msgs::PoseStamped ps;

  transform_base_to_ref.setOrigin(tf::Vector3(reference_pose.pose.position.x, reference_pose.pose.position.y, reference_pose.pose.position.z));
  transform_base_to_ref.setRotation(tf::Quaternion(reference_pose.pose.orientation.x, reference_pose.pose.orientation.y, reference_pose.pose.orientation.z, reference_pose.pose.orientation.w));

  tf::StampedTransform transform_ref_to_target(getTransform(reference_link, target_link, listener));

  tf::Transform transform_base_to_target( transform_base_to_ref * transform_ref_to_target );
  ps.header.frame_id = reference_pose.header.frame_id;
  ps.pose.position.x = transform_base_to_target.getOrigin().getX();
  ps.pose.position.y = transform_base_to_target.getOrigin().getY();
  ps.pose.position.z = transform_base_to_target.getOrigin().getZ();
  tf::quaternionTFToMsg(transform_base_to_target.getRotation(), ps.pose.orientation);

  return (ps);
}

#endif
