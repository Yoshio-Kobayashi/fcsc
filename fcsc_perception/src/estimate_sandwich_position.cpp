#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>

// sensor_msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// image
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

// fcsc
#include <fcsc_msgs/EstimateSandwichPosition.h>

#include <visualization_msgs/Marker.h>

class SandwichPositionEstimater {
  private :
    ros::NodeHandle nh;
    image_geometry::PinholeCameraModel cam_model;
    tf::TransformListener tf_listener;
    // ros::ServiceClient darknet_ros_client;
    actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> darknet_ros_client;
    ros::ServiceServer estimate_sandwich_pos_server;
    image_transport::ImageTransport image_transport;
    image_transport::CameraSubscriber image_subscriber;
    sensor_msgs::ImageConstPtr image_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;
    std::string camera_frame_id;
    ros::Publisher marker_pub;

    bool estimateSandwichPositionCB(fcsc_msgs::EstimateSandwichPosition::Request &req, fcsc_msgs::EstimateSandwichPosition::Response &res);
    void imageCB(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);

  public :
    SandwichPositionEstimater();
    void run() {
      ros::spin();
      // ros::AsyncSpinner spinner(2);
      // ros::Rate loop_rate(100);
      //
      // spinner.start();
      // while (ros::ok()) {
      //   loop_rate.sleep();
      // }
    };

};

SandwichPositionEstimater::SandwichPositionEstimater()
    : image_transport(nh),
      darknet_ros_client("/darknet_ros/check_for_objects", true),
      camera_frame_id("camera_color_optical_frame")
{
  ros::NodeHandle private_nh("~");
  std::string image_topic;

  private_nh.param<std::string>("image_topic", image_topic, "/camera/color/image_raw");

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // darknet_ros_client = nh.serviceClient<darknet_ros_msgs::CheckForObjects>("/darknet_ros/check_for_objects");
  estimate_sandwich_pos_server = nh.advertiseService("estimate_sandwich_position", &SandwichPositionEstimater::estimateSandwichPositionCB, this);
  image_subscriber = image_transport.subscribeCamera(image_topic, 1, &SandwichPositionEstimater::imageCB, this);
}

void SandwichPositionEstimater::imageCB(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
{
  image_msg = const_cast<sensor_msgs::ImageConstPtr&>(image);
  info_msg = const_cast<sensor_msgs::CameraInfoConstPtr&>(info);
}

bool SandwichPositionEstimater::estimateSandwichPositionCB(fcsc_msgs::EstimateSandwichPosition::Request &req, fcsc_msgs::EstimateSandwichPosition::Response &res)
{
  static int count;
  darknet_ros_msgs::CheckForObjectsGoal goal;
  darknet_ros_msgs::BoundingBoxes boxes;
  ros::Rate rate(50);
  // std_msgs::Header header;

  // header.stamp = ros::Time::now();
  // // stampが最新の画像を取得するまで待つ
  // while (1) {
  //   if (image_msg->header.stamp.sec > header.stamp.sec) {
  //     break;
  //   } else if (image_msg->header.stamp.sec == header.stamp.sec && image_msg->header.stamp.nsec > header.stamp.nsec) {
  //     break;
  //   }
  //   ROS_INFO("getting current image..");
  //   rate.sleep();
  // }

  // YOLOによる画像認識
  ROS_INFO("Call YOLO");
  goal.image = *image_msg;

  int result_id = -1;

  while (result_id != goal.id) {
    goal.id = count++;
    ROS_INFO("sendGoal id[%d]", goal.id);
    darknet_ros_client.sendGoal(goal);
    darknet_ros_client.waitForResult();
    result_id = darknet_ros_client.getResult()->id;
    ROS_INFO("getResult id[%d]", darknet_ros_client.getResult()->id);
  }

  boxes = darknet_ros_client.getResult()->bounding_boxes;

  ROS_INFO("Detected [%d] sandwiches", (int)boxes.bounding_boxes.size());

  cam_model.fromCameraInfo(info_msg);

  // 棚板平面（棚座標系）の設定
  // 平面の方程式 a*x + b*y + c*z + d = 0
  ROS_INFO("Set shelf plane");
  tf::Vector3 shelf_plane_n(req.plane.coef[0], req.plane.coef[1], req.plane.coef[2]);// 法線
  double d = req.plane.coef[3];

  // tf::Vector3 shelf_plane_n(0, 0, 1.0);
  // tf::Vector3 shelf_plane_x(0, 0, 0);
  // double h = shelf_plane_n.dot(shelf_plane_x);

  // 矩形からサンドイッチの位置推定
  for (size_t i = 0; i < boxes.bounding_boxes.size(); i++) {
    // 平面と直線の交点の参考
    // https://qiita.com/edo_m18/items/c8808f318f5abfa8af1e

    // 矩形の下端の中心取得
    ROS_INFO("Set box_center");
    cv::Point2d box_center;
    box_center.x = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin) / 2.0;
    box_center.y = boxes.bounding_boxes[i].ymax;

    // 画像 -> 3D変換して直線（カメラ座標系）取得
    ROS_INFO("Project pixel to 3d");
    cv::Point3d box_center_ray;
    box_center_ray = cam_model.projectPixelTo3dRay(box_center);

    // カメラ座標系 -> 棚座標系に変換して線分取得
    ROS_INFO("Get line segment");
    geometry_msgs::PointStamped box_center_start_point; // 始点
    geometry_msgs::Vector3Stamped box_center_direct; // 方向ベクトル
    geometry_msgs::PointStamped temp_point;
    geometry_msgs::Vector3Stamped temp_vec;

    box_center_start_point.header.stamp = ros::Time();
    box_center_direct.header.stamp = ros::Time();
    box_center_start_point.header.frame_id = camera_frame_id;
    box_center_direct.header.frame_id = camera_frame_id;
    box_center_start_point.point.x = 0.0;
    box_center_start_point.point.y = 0.0;
    box_center_start_point.point.z = 0.0;
    box_center_direct.vector.x = box_center_ray.x;
    box_center_direct.vector.y = box_center_ray.y;
    box_center_direct.vector.z = box_center_ray.z;

    tf_listener.transformPoint(req.plane_frame_id, box_center_start_point, temp_point);
    box_center_start_point = temp_point;

    tf_listener.transformVector(req.plane_frame_id, box_center_direct, temp_vec);
    box_center_direct = temp_vec;

    // 平面と線分の交点計算
    tf::Vector3 box_center_x0;
    tf::Vector3 box_center_m;
    tf::Vector3 intercect_point;
    geometry_msgs::PointStamped sandwich_point;
    double t;

    box_center_x0.setX(box_center_start_point.point.x);
    box_center_x0.setY(box_center_start_point.point.y);
    box_center_x0.setZ(box_center_start_point.point.z);

    box_center_m.setX(box_center_direct.vector.x);
    box_center_m.setY(box_center_direct.vector.y);
    box_center_m.setZ(box_center_direct.vector.z);

    t = (d - shelf_plane_n.dot(box_center_x0)) / shelf_plane_n.dot(box_center_m);

    intercect_point = box_center_x0 + t * box_center_m;

    sandwich_point.header.frame_id = req.plane_frame_id;
    sandwich_point.point.x = intercect_point.getX();
    sandwich_point.point.y = intercect_point.getY();
    sandwich_point.point.z = intercect_point.getZ();
    res.sandwich_positions.push_back(sandwich_point);
  }

  // 描画
  visualization_msgs::Marker marker_points;
  std_msgs::ColorRGBA green;

  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 0.25;

  marker_points.header.frame_id = req.plane_frame_id;
  marker_points.header.stamp = ros::Time::now();
  marker_points.ns = "estimates_sandwich_points";
  marker_points.action = visualization_msgs::Marker::ADD;
  marker_points.pose.orientation.w = 1.0;
  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;
  marker_points.scale.x = 0.05;
  marker_points.scale.y = 0.05;

  for (size_t i = 0; i < res.sandwich_positions.size(); i++) {
    marker_points.points.push_back(res.sandwich_positions[i].point);
    marker_points.colors.push_back(green);
  }

  for (size_t i = 0; i < 10; i++) {
    marker_pub.publish(marker_points);
  }

  return (true);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "SandwichPositionEstimater");

    SandwichPositionEstimater SandwichPositionEstimater;

    SandwichPositionEstimater.run();

    return (0);

}
