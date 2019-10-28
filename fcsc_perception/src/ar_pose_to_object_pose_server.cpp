// 認識したカメラ座標系の/ar_pose_marker をロボット座標系に変換してpublish

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <stdio.h>

// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//ar_track_alvar_msgs
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <fcsc_msgs/RecognizedObjectArray.h>

#include <XmlRpcException.h>

#include <geometry_msgs/Transform.h>

#include <fcsc_msgs/DetectObject.h>

#include <std_msgs/Bool.h>

using namespace std;

const int SANDWICH_FRONT    = 0;
const int SANDWICH_BACK     = 1;
const int SANDWICH_BOTTOM   = 2;
const int SANDWICH_RIGHT    = 3;
const int SANDWICH_LEFT     = 4;

class DetectedArPoseMarker {
public:
  DetectedArPoseMarker();
  void run();

private:
  struct SandwichSize {
    double width;
    double depth;
    double height;
    double angle;
  };

  ros::ServiceServer    service;
  ros::Subscriber       ar_pose_marker_sub;
  ros::Publisher        enable_detection_pub;
  std::string           output_frame_id;
  std::string           plane_frame_id;
  std::string           depth_frame_id;
  tf::Transform             sandwich_tf[5];
  geometry_msgs::Transform  sandwich_tf_msg[5];
  ar_track_alvar_msgs::AlvarMarkers ar_marker;
  SandwichSize sandwich_size;
  tf::TransformListener listener;

  tf::StampedTransform getTransform(std::string parent_frame_id, std::string child_frame_id);
  void transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out);
  void arPoseCB(const ar_track_alvar_msgs::AlvarMarkers sub_markers_msg);
  bool arPoseToObjectPoseCB(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res);
  void arPoseToObjectPose(fcsc_msgs::RecognizedObjectArray& detected_object_array);
  void readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value);
  bool setParams(XmlRpc::XmlRpcValue &params, geometry_msgs::Transform *object_tf_msg);
};

void DetectedArPoseMarker::readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value)
{
  if (params.hasMember(param_name))
  {
    if (params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      *value = (int)params[param_name];
    } else {
      *value = (double)params[param_name];
    }
  } else {
    ROS_ERROR("No member [%s]", param_name.c_str());
  }
}

bool DetectedArPoseMarker::setParams(XmlRpc::XmlRpcValue &params, geometry_msgs::Transform *object_tf_msg)
{
  std::vector<double> v(3);
  try
  {
    object_tf_msg->translation.x = (double)params["translation"]["x"];
    object_tf_msg->translation.y = (double)params["translation"]["y"];
    object_tf_msg->translation.z = (double)params["translation"]["z"];
    v[0] = (double)params["rotation"]["roll"];
    v[1] = (double)params["rotation"]["pitch"];
    v[2] = (double)params["rotation"]["yaw"];
    object_tf_msg->rotation = tf::createQuaternionMsgFromRollPitchYaw(v[0], v[1], v[2]);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

DetectedArPoseMarker::DetectedArPoseMarker()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue params;

  // sandwich_size.width = 0.07;
  // sandwich_size.depth = 0.09;
  // sandwich_size.height = 0.14;
  sandwich_size.width = 0.08;
  sandwich_size.depth = 0.09;
  sandwich_size.height = 0.1;
  sandwich_size.angle = atan(sandwich_size.height / sandwich_size.depth);

  nh.getParam("tf_from_ar_marker_to_sandwich/markers", params);
  setParams(params["front"], &sandwich_tf_msg[SANDWICH_FRONT]);
  setParams(params["back"], &sandwich_tf_msg[SANDWICH_BACK]);
  setParams(params["bottom"], &sandwich_tf_msg[SANDWICH_BOTTOM]);
  setParams(params["right"], &sandwich_tf_msg[SANDWICH_RIGHT]);
  setParams(params["left"], &sandwich_tf_msg[SANDWICH_LEFT]);

  tf::transformMsgToTF(sandwich_tf_msg[SANDWICH_FRONT], sandwich_tf[SANDWICH_FRONT]);
  tf::transformMsgToTF(sandwich_tf_msg[SANDWICH_BACK], sandwich_tf[SANDWICH_BACK]);
  tf::transformMsgToTF(sandwich_tf_msg[SANDWICH_BOTTOM], sandwich_tf[SANDWICH_BOTTOM]);
  tf::transformMsgToTF(sandwich_tf_msg[SANDWICH_RIGHT], sandwich_tf[SANDWICH_RIGHT]);
  tf::transformMsgToTF(sandwich_tf_msg[SANDWICH_LEFT], sandwich_tf[SANDWICH_LEFT]);

  sandwich_tf[SANDWICH_FRONT]  = sandwich_tf[SANDWICH_FRONT].inverse();
  sandwich_tf[SANDWICH_BACK]   = sandwich_tf[SANDWICH_BACK].inverse();
  sandwich_tf[SANDWICH_BOTTOM] = sandwich_tf[SANDWICH_BOTTOM].inverse();
  sandwich_tf[SANDWICH_RIGHT]  = sandwich_tf[SANDWICH_RIGHT].inverse();
  sandwich_tf[SANDWICH_LEFT]   = sandwich_tf[SANDWICH_LEFT].inverse();

  private_nh.param<std::string>("output_frame_id", output_frame_id, "map");
  private_nh.param<std::string>("plane_frame_id", plane_frame_id, "base_link");
  private_nh.param<std::string>("depth_frame_id", depth_frame_id, "camera_depth_optical_frame");

  ar_pose_marker_sub   = nh.subscribe("ar_pose_marker", 1, &DetectedArPoseMarker::arPoseCB, this);
  enable_detection_pub = nh.advertise<std_msgs::Bool>("/ar_track_alvar/enable_detection", 1);
  service              = nh.advertiseService("get_object", &DetectedArPoseMarker::arPoseToObjectPoseCB, this);
}

tf::StampedTransform DetectedArPoseMarker::getTransform(std::string parent_frame_id, std::string child_frame_id)
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

void DetectedArPoseMarker::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out)
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

void DetectedArPoseMarker::run()
{
  ros::Rate loop_rate(10);

  for (size_t i = 0; i < 2; i++) {
    enable_detection_pub.publish(std_msgs::Bool());
    loop_rate.sleep();
  }

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

string getSandwichMarkerType(int sandwich_marker_type)
{
  switch (sandwich_marker_type) {
    case SANDWICH_FRONT:  return("SANDWICH_FRONT");
    case SANDWICH_BACK:   return("SANDWICH_BACK");
    case SANDWICH_BOTTOM: return("SANDWICH_BOTTOM");
    case SANDWICH_LEFT:   return("SANDWICH_LEFT");
    case SANDWICH_RIGHT:  return("SANDWICH_RIGHT");
    default:              return("UNKNOWN");
  }
}

void DetectedArPoseMarker::arPoseToObjectPose(fcsc_msgs::RecognizedObjectArray& detected_object_array)
{
  geometry_msgs::Transform object_tf_msg;
  geometry_msgs::Transform marker_tf_msg;
  tf::Transform object_tf;
  tf::Transform marker_tf;
  int markers_size = ar_marker.markers.size();

  detected_object_array.objects.clear();

  // ARマーカを１つも認識しなかった場合
  if (markers_size == 0) {
    return;
  }

  //transform from ar_marker to robot_base_frame
  static tf::StampedTransform tf_base_to_plane = getTransform(output_frame_id, plane_frame_id);

  for (int i = 0; i < markers_size; i++) {
    fcsc_msgs::RecognizedObject detected_object;
    string object_name;
    char object_state;
    char object_type;
    char sandwich_marker_type;
    int id;
    int id_upper;
    int id_lower;
    ostringstream oss;

    id = ar_marker.markers[i].id;
    id_upper = (id / 10) % 10;
    id_lower = id % 10;

    //二桁より大きいIDのマーカは商品認識には使用しないとして、スキップする
    if (id % 10 >= 10) {
      continue;
    }

    // markerのidごとに物体情報の設定
    oss << id;
    if (id_upper >= 0 && id_upper <= 7) {
      oss.str("");
      oss.clear(std::stringstream::goodbit);
      oss << (char)('A' + id_upper);
      object_name = "sandwich_" + oss.str();

      // サンドイッチのどの面のマーカを認識したのか確認
      sandwich_marker_type = id_lower;

      ROS_WARN("sandwich%s is %s", object_name.c_str(), getSandwichMarkerType(sandwich_marker_type).c_str());
      ROS_WARN("sandiwch id[%d] upper[%d] lower[%d]", id, id_upper, id_lower);

      object_tf_msg = sandwich_tf_msg[id_lower];
      object_tf = sandwich_tf[id_lower];
      object_type = fcsc_msgs::RecognizedObject::SANDWICH;
    } else {
      continue;
    }

    marker_tf_msg.translation.x = ar_marker.markers[i].pose.pose.position.x;
    marker_tf_msg.translation.y = ar_marker.markers[i].pose.pose.position.y;
    marker_tf_msg.translation.z = ar_marker.markers[i].pose.pose.position.z;
    marker_tf_msg.rotation = ar_marker.markers[i].pose.pose.orientation;
    tf::transformMsgToTF(marker_tf_msg, marker_tf);

    if (ar_marker.markers[i].header.frame_id != output_frame_id) {
      // output_frame_id -> 別基準frame_id の tf を取得
      tf::StampedTransform tf( getTransform(output_frame_id, ar_marker.markers[i].header.frame_id) );
      // 別基準frame_id -> marker を output_frame_id -> 別基準frame_id -> marker に変換
      marker_tf = tf * marker_tf;
    }

    // output_frame_id -> marker -> object に変換
    object_tf = marker_tf * object_tf;

    detected_object.name = object_name;
    detected_object.pose.header.frame_id = output_frame_id;
    detected_object.pose.pose.position.x = object_tf.getOrigin().getX();
    detected_object.pose.pose.position.y = object_tf.getOrigin().getY();
    tf::quaternionTFToMsg(object_tf.getRotation().normalized(), detected_object.pose.pose.orientation);
    detected_object.type = object_type;

    // サンドイッチの倒れ方推定
    // 底面，背面，側面（右），側面（左），前面が接地の５通り
    tf::Matrix3x3 m(object_tf.getRotation().normalized());
    tf::Vector3 normal(0.0, 0.0, 1.0);// 接地面の法線
    tf::Vector3 vec( (normal * m).normalized() );

    // 比較用ベクトル
    tf::Vector3 bottom_vec(0.0, 0.0, 1.0);
    tf::Vector3 back_vec(-1.0, 0.0, 0.0);
    tf::Vector3 side_r_vec(0.0, 1.0, 0.0);
    tf::Vector3 side_l_vec(0.0, -1.0, 0.0);
    tf::Vector3 front_vec(cos(M_PI/2 - sandwich_size.angle), 0.0, cos(M_PI - sandwich_size.angle));

    std::vector<double> dot_val(5);
    dot_val[0] = bottom_vec.dot(vec);
    dot_val[1] = back_vec.dot(vec);
    dot_val[2] = side_r_vec.dot(vec);
    dot_val[3] = side_l_vec.dot(vec);
    dot_val[4] = front_vec.dot(vec);

    std::vector<double>::iterator iter = std::max_element(dot_val.begin(), dot_val.end());
    size_t index = std::distance(dot_val.begin(), iter);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw, 2);
    std::cerr << "R:" << roll*180.0/M_PI << " P:" << pitch*180.0/M_PI << " Y:" << yaw*180.0/M_PI << '\n';
    m.getRPY(roll, pitch, yaw, 1);
    std::cerr << "R:" << roll*180.0/M_PI << " P:" << pitch*180.0/M_PI << " Y:" << yaw*180.0/M_PI << '\n';

    double z;

    switch (index) {
      case 0:// bottom
        object_state = fcsc_msgs::RecognizedObject::BOTTOM;
        roll = 0;
        pitch = 0;
        z = 0;
        break;
      case 1:// back
        object_state = fcsc_msgs::RecognizedObject::BACK;
        pitch = M_PI/2.0;
        // yaw = 0;
        z = 0;
        break;
      case 2:// right side
        object_state = fcsc_msgs::RecognizedObject::RIGHT_SIDE;
        roll = M_PI/2.0;
        pitch = 0;
        z = sandwich_size.width / 2.0;
        break;
      case 3:// left side
        object_state = fcsc_msgs::RecognizedObject::LEFT_SIDE;
        roll = -M_PI/2.0;
        pitch = 0;
        z = sandwich_size.width / 2.0;
        break;
      case 4:// front
        object_state = fcsc_msgs::RecognizedObject::FRONT;
        roll > 0 ? roll = M_PI : roll = -M_PI;
        pitch = -sandwich_size.angle;
        z = sandwich_size.height * cos(sandwich_size.angle);
        break;
    }

    detected_object.state = object_state;

    // サンドイッチの姿勢補正
    detected_object.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // サンドイッチの高さ補正
    detected_object.pose.pose.position.z = z + tf_base_to_plane.getOrigin().getZ();
    // detected_object.pose.pose.position.z = object_tf.getOrigin().getZ();

    detected_object_array.objects.push_back(detected_object);
  }
}

bool DetectedArPoseMarker::arPoseToObjectPoseCB(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res)
{
  arPoseToObjectPose(res.detected_object_array);

  if (res.detected_object_array.objects.size() > 0) {
    res.success = true;
  } else {
    res.success = false;
  }

  return (true);
}


void DetectedArPoseMarker::arPoseCB(const ar_track_alvar_msgs::AlvarMarkers sub_markers_msg)
{
  ar_marker = sub_markers_msg;
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "ar_pose_marker_to_detected_object_array");
  DetectedArPoseMarker dapm;

  dapm.run();

  return 0;
}
