#include "fcsc_visual_tools/fcsc_visual_tools.h"
#include <iostream>

using namespace std;

void moveit_visual_tools::FCSCVisualTools::transformPose(std::string frame, geometry_msgs::PoseStamped ps_in, geometry_msgs::PoseStamped& ps_out)
{
  ps_in.header.stamp = ros::Time();
  try{
    listener.transformPose(frame, ps_in, ps_out);
    return;
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a position from \"%s\" to \"%s\": %s", ps_in.header.frame_id.c_str(), frame.c_str(), ex.what());
  }
}

shape_msgs::Mesh moveit_visual_tools::FCSCVisualTools::createMesh(const std::string mesh_path)
{
  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m, co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

  return co_mesh;
}

moveit_visual_tools::FCSCVisualTools::FCSCVisualTools()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue params;

  ROS_INFO("shelf_tf:Set param");
  nh.getParam("shelf_size", params);
  setShelfSizeParams(params["shelfA"], shelf_sizes[0]);
  setShelfSizeParams(params["shelfB"], shelf_sizes[1]);

  onigiri_mesh = createMesh("package://fcsc_description/mesh/onigiri_edit.stl");
  drink_mesh = createMesh("package://fcsc_description/mesh/drink_edit.stl");
  bento_mesh = createMesh("package://fcsc_description/mesh/bento_edit.stl");
  sandwich_mesh = createMesh("package://fcsc_description/mesh/sandwich-v3.stl");
  // sandwich_mesh = createMesh("package://fcsc_description/mesh/sandwich-v2.stl");

  collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  update_shelf_pose_client = nh.serviceClient<std_srvs::Empty>("update_shelf_pose");

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_footprint", "/visualization_marker"));
  visual_tools_->setPlanningSceneTopic("/move_group/monitored_planning_scene");
}

void moveit_visual_tools::FCSCVisualTools::readXmlParam(XmlRpc::XmlRpcValue &params, const std::string &param_name, double *value)
{
  if (params.hasMember(param_name))
  {
    if (params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt)
      *value = (int)params[param_name];
    else
      *value = (double)params[param_name];
  }
}

bool moveit_visual_tools::FCSCVisualTools::setShelfSizeParams(XmlRpc::XmlRpcValue &params, ShelfSize &shelf_size)
{
  try
  {
    readXmlParam(params, "width", &shelf_size.width);
    readXmlParam(params, "depth", &shelf_size.depth);
    readXmlParam(params, "stopper_height", &shelf_size.stopper_height);
    readXmlParam(params, "stopper_thickness", &shelf_size.stopper_thickness);
    readXmlParam(params, "thickness", &shelf_size.thickness);
    readXmlParam(params, "height", &shelf_size.height);
    for (size_t i = 0; i < params["installation_heights"].size(); i++) {
      shelf_size.installation_heights.push_back((double)params["installation_heights"][i]);
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

void moveit_visual_tools::FCSCVisualTools::publishCollisionObject(moveit_msgs::CollisionObject object_msg)
{
  for (size_t i = 0; i < 10; i++) {
    collision_object_pub.publish(object_msg);
    ros::WallDuration(0.05).sleep();
  }
}

void moveit_visual_tools::FCSCVisualTools::publishCollisionMesh(const geometry_msgs::Pose object_pose, const std::string object_frame_id, const std::string object_name, const shape_msgs::Mesh mesh_msg)
{
  moveit_msgs::CollisionObject object_msg;

  object_msg.header.frame_id = object_frame_id;
  object_msg.meshes.resize(1);
  object_msg.mesh_poses.resize(1);
  object_msg.meshes[0] = mesh_msg;
  object_msg.id = object_name;
  // object_msg.operation = moveit_msgs::CollisionObject::REMOVE;
  // publishCollisionObject(object_msg);
  object_msg.operation = moveit_msgs::CollisionObject::ADD;
  object_msg.mesh_poses[0] = object_pose;
  publishCollisionObject(object_msg);
}

void setPoseFromTransform(geometry_msgs::Pose &pose, tf::Transform transform)
{
  pose.position.x = transform.getOrigin().getX();
  pose.position.y = transform.getOrigin().getY();
  pose.position.z = transform.getOrigin().getZ();
  tf::quaternionTFToMsg(transform.getRotation().normalized(), pose.orientation);
}

void moveit_visual_tools::FCSCVisualTools::spawnShelf(std::string shelf_name, geometry_msgs::PoseStamped shelf_pose)
{
  moveit_msgs::CollisionObject shelf;
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose primitive_pose;
  tf::Transform tf_base_to_shelf;
  tf::Transform tf_shelf_to_tanaita;
  tf::Transform tf_base_to_tanaita;
  ShelfSize shelf_size;

  if (shelf_name.find("shelfB") != std::string::npos) {
    shelf_size = shelf_sizes[1];
  } else {
    shelf_size = shelf_sizes[0];
  }

  tf_base_to_shelf.setOrigin(tf::Vector3(shelf_pose.pose.position.x, shelf_pose.pose.position.y, shelf_pose.pose.position.z));
  tf_base_to_shelf.setRotation(tf::Quaternion(shelf_pose.pose.orientation.x, shelf_pose.pose.orientation.y, shelf_pose.pose.orientation.z, shelf_pose.pose.orientation.w));

  shelf.header.frame_id = shelf_pose.header.frame_id;
  shelf.id = shelf_name;
  shelf.operation = moveit_msgs::CollisionObject::ADD;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);

  //棚板(地面)
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = shelf_size.depth;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = shelf_size.width;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = shelf_size.thickness;
  primitive_pose = shelf_pose.pose;
  shelf.primitives.push_back(primitive);
  shelf.primitive_poses.push_back(primitive_pose);

  for (size_t i = 0; i < shelf_size.installation_heights.size(); i++) {
    //各段の棚板
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = shelf_size.depth;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = shelf_size.width;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = shelf_size.thickness;
    tf_shelf_to_tanaita.setOrigin(tf::Vector3(0, 0, shelf_size.installation_heights[i] - shelf_size.thickness / 2.0));
    tf_shelf_to_tanaita.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf_base_to_tanaita = tf_base_to_shelf * tf_shelf_to_tanaita;
    setPoseFromTransform(primitive_pose, tf_base_to_tanaita);
    shelf.primitives.push_back(primitive);
    shelf.primitive_poses.push_back(primitive_pose);

    //各段のランカンレール
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = shelf_size.stopper_thickness;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = shelf_size.width;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = shelf_size.stopper_height;
    tf_shelf_to_tanaita.setOrigin(tf::Vector3((-shelf_size.depth + shelf_size.stopper_thickness) / 2.0, 0.0, shelf_size.installation_heights[i] + (shelf_size.stopper_height) / 2.0));
    tf_base_to_tanaita = tf_base_to_shelf * tf_shelf_to_tanaita;
    setPoseFromTransform(primitive_pose, tf_base_to_tanaita);
    shelf.primitives.push_back(primitive);
    shelf.primitive_poses.push_back(primitive_pose);
  }

  //棚板(左)
  tf_shelf_to_tanaita.setOrigin(tf::Vector3(0.0, (shelf_size.width + shelf_size.thickness) / 2.0, shelf_size.height / 2.0));
  tf_base_to_tanaita = tf_base_to_shelf * tf_shelf_to_tanaita;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = shelf_size.depth;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = shelf_size.thickness;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = shelf_size.height;
  setPoseFromTransform(primitive_pose, tf_base_to_tanaita);
  shelf.primitives.push_back(primitive);
  shelf.primitive_poses.push_back(primitive_pose);

  //棚板(右)
  tf_shelf_to_tanaita.setOrigin(tf::Vector3(0.0, -(shelf_size.width + shelf_size.thickness) / 2.0, shelf_size.height / 2.0));
  tf_base_to_tanaita = tf_base_to_shelf * tf_shelf_to_tanaita;
  setPoseFromTransform(primitive_pose, tf_base_to_tanaita);
  shelf.primitives.push_back(primitive);
  shelf.primitive_poses.push_back(primitive_pose);

  //棚板(奥)
  tf_shelf_to_tanaita.setOrigin(tf::Vector3((shelf_size.depth + shelf_size.thickness) / 2.0, 0.0, shelf_size.height / 2.0));
  tf_base_to_tanaita = tf_base_to_shelf * tf_shelf_to_tanaita;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = shelf_size.thickness;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = shelf_size.width;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = shelf_size.height;
  setPoseFromTransform(primitive_pose, tf_base_to_tanaita);
  shelf.primitives.push_back(primitive);
  shelf.primitive_poses.push_back(primitive_pose);

  publishCollisionObject(shelf);

  std_srvs::Empty srv;
  update_shelf_pose_client.call(srv);
}

void moveit_visual_tools::FCSCVisualTools::spawnProduct(const fcsc_msgs::RecognizedObject object)
{
  spawnProduct(object.name, object.pose);
}

void moveit_visual_tools::FCSCVisualTools::spawnProduct(std::string object_name, geometry_msgs::PoseStamped object_pose)
{
  shape_msgs::Mesh mesh;

  if (object_name.find("onigiri") != std::string::npos) {
  // ROS_INFO("onigiri");
    mesh = onigiri_mesh;
  } else if (object_name.find("bento") != std::string::npos) {
  // ROS_INFO("bento");
    mesh = bento_mesh;
  } else if (object_name.find("drink") != std::string::npos) {
  // ROS_INFO("drink");
    mesh = drink_mesh;
  } else if (object_name.find("sandwich") != std::string::npos) {
    mesh = sandwich_mesh;
  } else {
    ROS_ERROR("FALSE");
  }

  ROS_INFO("spawn %s", object_name.c_str());
  publishCollisionMesh(object_pose.pose, object_pose.header.frame_id, object_name, mesh);
  // visual_tools_->setBaseFrame(object_pose.header.frame_id);
  // visual_tools_->publishCollisionMesh(object_pose.pose, object_name, mesh);
  // visual_tools_->triggerPlanningSceneUpdate();
  // ros::Duration(1).sleep();
}

void moveit_visual_tools::FCSCVisualTools::deleteObject(std::string object_name)
{
  ROS_INFO("remove %s", object_name.c_str());
  moveit_msgs::CollisionObject co_msg;

  co_msg.operation = moveit_msgs::CollisionObject::REMOVE;
  co_msg.id = object_name;
  for (size_t i = 0; i < 5; i++) {
    collision_object_pub.publish(co_msg);
    ros::WallDuration(0.1).sleep();
  }
}

void moveit_visual_tools::FCSCVisualTools::removeAllProducts(const fcsc_msgs::RecognizedObjectArray object_array)
{
  ROS_INFO("remove all products");
  int size = object_array.objects.size();
  moveit_msgs::CollisionObject co_msg;

  co_msg.operation = moveit_msgs::CollisionObject::REMOVE;
  for (size_t i = 0; i < size; i++) {
    ROS_INFO("cleanup %s", object_array.objects[i].name.c_str());
    co_msg.id = object_array.objects[i].name;
    for (size_t i = 0; i < 2; i++) {
      collision_object_pub.publish(co_msg);
    }
    // visual_tools_->cleanupCO(object_array.objects[i].name);
  }
}

void moveit_visual_tools::FCSCVisualTools::contactProduct(std::string object_name, std::string contact_object_name, geometry_msgs::PoseStamped current_object_pose, int object_count)
{
  moveit_msgs::CollisionObject co_msg;
  moveit_msgs::GetPlanningScene srv;
  geometry_msgs::PoseStamped ps_in;
  geometry_msgs::PoseStamped ps_out;
  double object_height;
  double z;
  int product_type;
  int size;
  int index;
  bool found = false;

  srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  planning_scene_client.call(srv);
  size = srv.response.scene.world.collision_objects.size();
  for (int i = 0; i < size; i++) {
    if (srv.response.scene.world.collision_objects[i].id == object_name) {
      index = i;
      found = true;
      break;
    }
  }
  if (!found) {
    ROS_ERROR("Not found %s", object_name.c_str());
  }

  //物体の種類と高さを設定
  if (object_name.find("onigiri") != std::string::npos) {
    object_height = 0.07;
    // object_height = 0.035;
    // product_type = ONIGIRI;
  } else if (object_name.find("drink") != std::string::npos) {
    object_height = 0.108;
    // product_type = DRINK;
  } else if (object_name.find("bento") != std::string::npos) {
    object_height = 0.045;
    // product_type = BENTO;
  } else if (object_name.find("sandwich") != std::string::npos) {
    object_height = 0.0;
    // product_type = SANDWICH;
  } else {
    object_height =  0.1;
  }

  if (contact_object_name.find("bento") != std::string::npos) {
    z = object_height / 2.0 + object_height * object_count;
  } else {
    z = object_height / 2.0;
  }

  co_msg.primitive_poses.resize(1);
  co_msg.header.stamp = ros::Time::now();
  co_msg.id = object_name;
  co_msg.operation = moveit_msgs::CollisionObject::MOVE;
  co_msg.primitive_poses[0].position.z = z;

  if (contact_object_name.find("bento") != std::string::npos) {
    co_msg.header.frame_id = current_object_pose.header.frame_id;
    co_msg.primitive_poses[0].orientation = current_object_pose.pose.orientation;
    co_msg.primitive_poses[0].position.x = current_object_pose.pose.position.x;
    co_msg.primitive_poses[0].position.y = current_object_pose.pose.position.y;
  } else {
    ps_in.header.frame_id = srv.response.scene.world.collision_objects[index].header.frame_id;
    ps_in.pose = srv.response.scene.world.collision_objects[index].mesh_poses[0];

    transformPose(contact_object_name, current_object_pose, ps_out);
    co_msg.primitive_poses[0].orientation = ps_out.pose.orientation;

    transformPose(contact_object_name, ps_in, ps_out);
    co_msg.header.frame_id = contact_object_name;
    co_msg.primitive_poses[0].position.x = ps_out.pose.position.x;
    co_msg.primitive_poses[0].position.y = ps_out.pose.position.y;
  }
  for (size_t i = 0; i < 2; i++) {
    collision_object_pub.publish(co_msg);
  }
}
