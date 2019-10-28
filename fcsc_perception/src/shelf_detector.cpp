// 認識したカメラ座標系の/ar_pose_marker をロボット座標系に変換してpublish

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <stdio.h>

// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <fcsc_msgs/RecognizedObjectArray.h>
#include <fcsc_msgs/DetectObject.h>

#include <XmlRpcException.h>

#include <geometry_msgs/Transform.h>

#include <fcsc_visual_tools/fcsc_visual_tools.h>

#include <std_msgs/Bool.h>

using namespace std;

struct Shelf {
  std::string name;
  geometry_msgs::PoseStamped pose;
};

int vectorFinder(std::vector<std::string> vec, std::string str) {
  std::vector<std::string>::iterator itr = std::find(vec.begin(), vec.end(), str);
  size_t index = std::distance( vec.begin(), itr );
  if (index != vec.size()) { // 発見できたとき
    return (index);
  } else { // 発見できなかったとき
    return (-1);
  }
}

void getRPYFromQuaternion(geometry_msgs::Quaternion q, double& r, double& p, double& y)
{
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(r, p, y);
}

class ShelfDetector {
public:
  ShelfDetector();
  void run();

private:
  ros::Subscriber ar_pose_marker_sub;
  ros::ServiceServer detect_shelf_server;
  ros::Publisher enable_detection_publisher;
  tf::TransformListener listener;
  ar_track_alvar_msgs::AlvarMarkers ar_marker;
  moveit_visual_tools::FCSCVisualTools object_visualizer;
  bool detected;
  int shelfA_marker_id;
  int shelfB_marker_id;
  geometry_msgs::PoseStamped shelf_pose;
  std::vector<Shelf> shelves;
  tf::Transform marker_to_shelf_tf;

  // 棚の予想姿勢
  geometry_msgs::QuaternionStamped estimated_shelf_orientation;
  // 許容誤差のしきい値
  double error_angle_thresh;

  bool detectShelfCB(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res);
  void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out);
  void arPoseCB(const ar_track_alvar_msgs::AlvarMarkers markers_msg);
};

ShelfDetector::ShelfDetector()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  detected = false;

  nh.param<int>("shelfA_marker_id", shelfA_marker_id, 100);
  nh.param<int>("shelfB_marker_id", shelfB_marker_id, 120);

  private_nh.param<double>("error_angle_thresh", error_angle_thresh, 5.0);

  ROS_INFO("[ShelfDetector]:error_angle_thresh [%f]deg", error_angle_thresh);

  error_angle_thresh *= M_PI / 180.0;

  detect_shelf_server = nh.advertiseService("detect_shelf", &ShelfDetector::detectShelfCB, this);
  ar_pose_marker_sub = nh.subscribe("shelf_detector/ar_pose_marker", 1, &ShelfDetector::arPoseCB, this);
  enable_detection_publisher = nh.advertise<std_msgs::Bool>("shelf_detector/ar_track_alvar/enable_detection", 1);

  // 棚の予想位置の設定
  estimated_shelf_orientation.header.frame_id = "base_footprint";
  estimated_shelf_orientation.quaternion = tf::createQuaternionMsgFromYaw(0.0);

  marker_to_shelf_tf.setOrigin(tf::Vector3(0, 0, -0.435/2.0));
  marker_to_shelf_tf.setRotation(tf::createQuaternionFromRPY(-M_PI/2.0, M_PI/2.0, 0));
}

void ShelfDetector::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out)
{
  //we'll just use the most recent transform available for our simple example
  ps_in.header.stamp = ros::Time();

  ros::Rate rate(10);
  // while (1) {
    try{
      listener.transformPose(frame, ps_in, ps_out);
      return;
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a position from \"%s\" to \"%s\": %s", ps_in.header.frame_id.c_str(), frame.c_str(), ex.what());
    }
    rate.sleep();
  // }
}

void ShelfDetector::run()
{
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  for (size_t i = 0; i < 2; i++) {
    enable_detection_publisher.publish(std_msgs::Bool());
    loop_rate.sleep();
  }

  while (ros::ok()) {
    loop_rate.sleep();
    // ros::spinOnce();
  }
}

void ShelfDetector::arPoseCB(const ar_track_alvar_msgs::AlvarMarkers markers_msg)
{
  ar_marker = markers_msg;
}


bool ShelfDetector::detectShelfCB(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res)
{
  double roll, pitch, yaw;
  double depth_offset = 0.0;//棚の奥行き方向の誤差 0.02
  bool detected;//すでに棚を検出しているか確認
  Shelf shelf;

  // 検出したマーカ位置から棚の位置・姿勢を計算
  for (size_t i = 0; i < ar_marker.markers.size(); i++) {
    detected = false;

    if (ar_marker.markers[i].id == shelfA_marker_id) {
      shelf.name = "shelfA";
    } else if (ar_marker.markers[i].id == shelfB_marker_id) {
      shelf.name = "shelfB";
    } else {
      continue;
    }

    tf::Transform base_to_marker_tf(tf::Quaternion(ar_marker.markers[i].pose.pose.orientation.x,
                                                   ar_marker.markers[i].pose.pose.orientation.y,
                                                   ar_marker.markers[i].pose.pose.orientation.z,
                                                   ar_marker.markers[i].pose.pose.orientation.w),
                                    tf::Vector3(ar_marker.markers[i].pose.pose.position.x,
                                                ar_marker.markers[i].pose.pose.position.y,
                                                ar_marker.markers[i].pose.pose.position.z));

    tf::Transform base_to_shelf_tf = base_to_marker_tf * marker_to_shelf_tf;

    // 検出した棚の位置・姿勢を格納
    shelf.pose.header = ar_marker.markers[i].header;
    shelf.pose.pose.position.x = base_to_shelf_tf.getOrigin().getX();
    shelf.pose.pose.position.y = base_to_shelf_tf.getOrigin().getY();
    shelf.pose.pose.position.z = base_to_shelf_tf.getOrigin().getZ();
    tf::quaternionTFToMsg(base_to_shelf_tf.getRotation().normalized(), shelf.pose.pose.orientation);

    // 棚の予想姿勢の基準座標に変換
    geometry_msgs::PoseStamped tmp_pose;
    transformPose(estimated_shelf_orientation.header.frame_id, shelf.pose, tmp_pose);
    shelf.pose = tmp_pose;
    shelf.pose.pose.position.z = 0;
    getRPYFromQuaternion(shelf.pose.pose.orientation, roll, pitch, yaw);

    // 棚の予想姿勢と計測姿勢との誤差を確認
    // しきい値よりも誤差が大きければ予想姿勢を優先
    if ( abs(tf::getYaw(estimated_shelf_orientation.quaternion) - yaw) > error_angle_thresh) {
      shelf.pose.pose.orientation = estimated_shelf_orientation.quaternion;
    } else {
      shelf.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }

    // 固定座標系に変換
    transformPose("map", shelf.pose, tmp_pose);
    shelf.pose = tmp_pose;

    for (size_t j = 0; ( j < shelves.size() ) && ( shelves[j].name == shelf.name ); j++) {
      //すでに検出していたらデータを更新
      ROS_INFO("[shelf_detector]:update shelf pose");
      detected = true;
      shelves[j] = shelf;
      break;
    }

    if (!detected) {
      shelves.push_back(shelf);
    }

  }

  if (shelves.size() > 0) {
    res.success = true;
    res.detected_object_array.objects.resize(shelves.size());
    for (size_t i = 0; i < shelves.size(); i++) {
      res.detected_object_array.objects[i].name = shelves[i].name;
      res.detected_object_array.objects[i].pose.header.frame_id = shelves[i].pose.header.frame_id;
      res.detected_object_array.objects[i].pose.pose = shelves[i].pose.pose;
      object_visualizer.spawnShelf(shelves[i].name, shelves[i].pose);
    }
  } else {
    res.success = false;
  }

  return (true);
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "shelf_detector");
  ShelfDetector detector;

  detector.run();

  return 0;
}
