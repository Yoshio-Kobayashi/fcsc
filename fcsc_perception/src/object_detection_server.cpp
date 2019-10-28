#include <ros/ros.h>
#include <fcsc_msgs/DetectObject.h>
#include <fcsc_visual_tools/fcsc_visual_tools.h>
#include <XmlRpcException.h>

const double onigiri_height = 0.07;
const double drink_height   = 0.108;
const double bento_height   = 0.045;

int vectorFinder(std::vector<std::string> vec, std::string str) {
  std::vector<std::string>::iterator itr = std::find(vec.begin(), vec.end(), str);
  size_t index = std::distance( vec.begin(), itr );
  if (index != vec.size()) { // 発見できたとき
    return (index);
  } else { // 発見できなかったとき
    return (-1);
  }
}

struct StockingProductInfo {
  int number;
  int start;
};

class DetectObjectServer {
public:
  DetectObjectServer();
  void start();

private:
  ros::ServiceServer service;
  ros::ServiceClient get_object_client;
  ros::Subscriber    detected_object_array_sub;
  ros::Publisher     detected_object_array_pub;
  moveit_visual_tools::FCSCVisualTools object_visualizer;

  std::vector<std::string> object_names_;
  std::vector<std::string> scrap_sandwich_ids;
  fcsc_msgs::RecognizedObjectArray detected_object_array_;

  void updateDetectedObjectArray(fcsc_msgs::RecognizedObject object);
  bool isScrapSandwich(fcsc_msgs::RecognizedObject sandwich);
  void getObject(fcsc_msgs::RecognizedObjectArray &recognized_object_array_msg);
  void detectedObjectArrayCB(fcsc_msgs::RecognizedObjectArray msg);
  bool detectObject(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res);
  void publishObject();
};

DetectObjectServer::DetectObjectServer()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue params;

  detected_object_array_sub = nh.subscribe("/object_detection_node/detected_object_array", 10, &DetectObjectServer::detectedObjectArrayCB, this);
  service                   = nh.advertiseService("detect_object", &DetectObjectServer::detectObject, this);
  detected_object_array_pub = nh.advertise<fcsc_msgs::RecognizedObjectArray>("/detected_object_array", 10);
  get_object_client         = nh.serviceClient<fcsc_msgs::DetectObject>("get_object");

  //サンドイッチの廃棄品情報を取得
  nh.getParam("scrap_sandwich_id", params);
  for (size_t i = 0; i < params.size(); i++) {
    std::string s;
    s = (std::string)params[i];
    // 小文字を大文字に変換
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    scrap_sandwich_ids.push_back(s);
  }
  for (size_t i = 0; i < scrap_sandwich_ids.size(); i++) {
    ROS_INFO("Scrap sandwich id[%s]", scrap_sandwich_ids[i].c_str());
  }
}

void DetectObjectServer::detectedObjectArrayCB(fcsc_msgs::RecognizedObjectArray msg)
{
  for (int i = 0; i < msg.objects.size(); i++) {
    // 認識済みの物体か確認
    int index = vectorFinder(object_names_, msg.objects[i].name);
    // 認識済みなら位置・姿勢だけ更新
    if (index >= 0) {
      detected_object_array_.objects[index].pose = msg.objects[i].pose;
      continue;
    }
    // あらたに認識した物体がサンドイッチなら廃棄品かどうかチェックして設定する
    if (msg.objects[i].type == fcsc_msgs::RecognizedObject::SANDWICH && isScrapSandwich(msg.objects[i])) {
      msg.objects[i].scrap = true;
    }
    // あらたに認識した物体を物体名リストに入れる
    object_names_.push_back(msg.objects[i].name);
    detected_object_array_.objects.push_back(msg.objects[i]);
  }
}

void DetectObjectServer::getObject(fcsc_msgs::RecognizedObjectArray &recognized_object_array_msg)
{
  XmlRpc::XmlRpcValue params;
  fcsc_msgs::RecognizedObject recognized_object_msg;
  std::ostringstream oss;
  std::vector<StockingProductInfo> product_info(6);
  ros::NodeHandle nh;

  recognized_object_msg.pose.header.frame_id = "container";

  system("rosparam load `rospack find fcsc_perception`/config/stocking_products.yaml");

  nh.getParam("stocking_products", params);
  product_info[0].number  = (int)params["bento"]["A"]["number"];
  product_info[1].number  = (int)params["bento"]["B"]["number"];
  product_info[2].number  = (int)params["drink"]["A"]["number"];
  product_info[3].number  = (int)params["drink"]["B"]["number"];
  product_info[4].number  = (int)params["onigiri"]["A"]["number"];
  product_info[5].number  = (int)params["onigiri"]["B"]["number"];

  // bento
  recognized_object_msg.type = fcsc_msgs::RecognizedObject::BENTO;
  recognized_object_msg.pose.pose.position.x = 0.0;
  recognized_object_msg.pose.pose.position.y = 0.0;
  recognized_object_msg.pose.pose.position.z = bento_height / 2.0;
  recognized_object_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI / 2.0);
  for (int i = 0; i < product_info[0].number + product_info[1].number; i++) {
    if (i < product_info[0].number) {
      oss << "A_" << (i + 1);
    } else {
      oss << "B_" << (i + 1 - product_info[0].number);
    }
    recognized_object_msg.name = "bento_" + oss.str();
    recognized_object_array_msg.objects.push_back(recognized_object_msg);
    recognized_object_msg.pose.pose.position.z += bento_height - 0.005;
    // recognized_object_msg.pose.pose.position.z += bento_height;
    oss.str("");
    oss.clear(std::stringstream::goodbit);
  }
  std::reverse(recognized_object_array_msg.objects.begin(), recognized_object_array_msg.objects.end());

  // drink
  recognized_object_msg.type = fcsc_msgs::RecognizedObject::DRINK;
  recognized_object_msg.pose.pose.position.x = 0.15;
  recognized_object_msg.pose.pose.position.y = -0.2;
  recognized_object_msg.pose.pose.position.z = drink_height / 2.0;
  recognized_object_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
  for (int i = 0; i < product_info[2].number + product_info[3].number; i++) {
    if (i < product_info[2].number) {
      oss << "A_" << (i + 1);
    } else {
      oss << "B_" << (i + 1 - product_info[2].number);
    }
    recognized_object_msg.name = "drink_" + oss.str();
    recognized_object_array_msg.objects.push_back(recognized_object_msg);
    recognized_object_msg.pose.pose.position.x -= 0.08;
    oss.str("");
    oss.clear(std::stringstream::goodbit);
  }

  // // onigiri
  recognized_object_msg.type = fcsc_msgs::RecognizedObject::ONIGIRI;
  recognized_object_msg.pose.pose.position.y = 0.2;

  // 立たせて置く場合
  // recognized_object_msg.pose.pose.position.x = 0.0; // FCSC2018
  // recognized_object_msg.pose.pose.position.z = onigiri_height / 2.0;
  // recognized_object_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI / 2);
  // recognized_object_msg.pose.pose.position.x -= 0.05;

  // 寝かせて置く場合
  recognized_object_msg.pose.pose.position.x = 0.18;
  recognized_object_msg.pose.pose.position.z = 0.035 / 2.0;
  recognized_object_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, 0, M_PI / 2);

  recognized_object_msg.pose.pose.position.y = (0.65 + 0.02) / 2.0 - 0.035;
  recognized_object_msg.pose.pose.position.x = 0.15;
  recognized_object_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, 0, M_PI);

  for (int i = 0; i < product_info[4].number + product_info[5].number; i++) {
    if (i < product_info[4].number) {
      oss << "A_" << (i + 1);
    } else {
      oss << "B_" << (i + 1 - product_info[4].number);
    }
    recognized_object_msg.name = "onigiri_" + oss.str();
    recognized_object_array_msg.objects.push_back(recognized_object_msg);
    recognized_object_msg.pose.pose.position.x -= 0.1;
    // recognized_object_msg.pose.pose.position.x -= 0.085;
    oss.str("");
    oss.clear(std::stringstream::goodbit);
  }
}

bool DetectObjectServer::isScrapSandwich(fcsc_msgs::RecognizedObject sandwich)
{
  if (sandwich.type != fcsc_msgs::RecognizedObject::SANDWICH) {
    return (false);
  }
  for (int j = 0; j < scrap_sandwich_ids.size(); j++) {
    if (sandwich.name == ("sandwich_"+scrap_sandwich_ids[j])) {
      return (true);
    }
  }
  return (false);
}

void DetectObjectServer::updateDetectedObjectArray(fcsc_msgs::RecognizedObject object)
{
  // すでに認識済みの物体なのか確認
  int index = vectorFinder(object_names_, object.name);
  if (index >= 0) {
    // 認識済みなら物体の位置・姿勢を更新する
    detected_object_array_.objects[index].pose = object.pose;
    detected_object_array_.objects[index].state = object.state;
  } else {
    // 初めて認識した物体をデータに追加する
    object.scrap = isScrapSandwich(object);
    object_names_.push_back(object.name);
    detected_object_array_.objects.push_back(object);
  }
}

bool DetectObjectServer::detectObject(fcsc_msgs::DetectObject::Request &req, fcsc_msgs::DetectObject::Response &res)
{
  fcsc_msgs::DetectObject get_object_srv;

  //陳列済みの物体をすべて削除
  if (req.clear == true) {
    ROS_WARN("[ObjectDetectionServer]:Clear all objects");
    object_visualizer.removeAllProducts(detected_object_array_);
    object_names_.clear();
    detected_object_array_.objects.clear();
    res.success = true;
  }

  if (req.delete_object_names.size() > 0) {
    for (int i = 0; i < req.delete_object_names.size(); i++) {
      int index = vectorFinder(object_names_, req.delete_object_names[i]);
      if (index >= 0) {
        ROS_WARN("[ObjectDetectionServer]:Delete [%s]", object_names_[index].c_str());
        object_names_.erase(object_names_.begin() + index);
        detected_object_array_.objects.erase(detected_object_array_.objects.begin() + index);
      }
    }
    res.success = true;
  }

  if (req.detect == true) {
    ROS_WARN("[ObjectDetectionServer]:Detect Object");

    if (req.damy == true) {
      ROS_WARN("[ObjectDetectionServer]:Damy Detection");
      getObject(get_object_srv.response.detected_object_array);
    } else {
      get_object_client.call(get_object_srv);
    }

    int object_size = get_object_srv.response.detected_object_array.objects.size();
    for (int i = 0; i < object_size; i++) {
      fcsc_msgs::RecognizedObject object = get_object_srv.response.detected_object_array.objects[i];
      updateDetectedObjectArray(object);
    }

    res.detected_object_array = detected_object_array_;

    if (res.detected_object_array.objects.size() > 0) {
      res.success = true;
    } else {
      res.success = false;
    }
  }

  if (req.visualize == true) {
    ROS_WARN("[ObjectDetectionServer]:Visualize objects");
    for (size_t i = 0; i < detected_object_array_.objects.size(); i++) {
      object_visualizer.spawnProduct(detected_object_array_.objects[i]);
    }
  }

  publishObject();

  return (true);
}

void DetectObjectServer::publishObject()
{
  static ros::Rate rate(50);

  for (int i = 0; i < 2; i++) {
    rate.sleep();
    detected_object_array_pub.publish(detected_object_array_);
  }
}

void DetectObjectServer::start()
{
  ROS_WARN("detect_object_server START");
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init (argc, argv, "detect_object_server");
  DetectObjectServer server;

  server.start();
  return (0);
}
