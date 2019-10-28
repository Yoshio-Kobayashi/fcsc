#include <ros/ros.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <fcsc_msgs/EstimateObjectPosition.h>
// #include <correspondence_grouping_ros/EstimateObjectPosition.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

// define as global variables
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//sensor_msgs::PointCloud2::Ptr in_cloud;

bool enableSwitched = false;
bool enabled = true;

//
// Switch to subscribe points
//
void enableCallback(const std_msgs::BoolConstPtr& msg)
{
  enableSwitched = (enabled != msg->data);
  enabled = msg->data;
}

//
// receive point cloud from depth camera
//
void getScene (const sensor_msgs::PointCloud2Ptr &input)
{
  // Read in the cloud data
  //pcl::PCDReader reader;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  //reader.read ("catkin_ws/src/correspondence_grouping_ros/pcd/sandwich5_shelf.pcd", *cloud);
  pcl::fromROSMsg (*input, *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
  //pcl::toROSMsg (*cloud, *in_cloud);
  //in_cloud = input;
}

//
// ROS service communication
//
bool getPosition (fcsc_msgs::EstimateObjectPosition::Request &req, fcsc_msgs::EstimateObjectPosition::Response &res)
{
  static tf::TransformListener listener;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
  //sensor_msgs::PointCloud2::Ptr filtering_cloud;

  bool success;
  // exchange camera frame to specified frame
  for (int i = 0; i < 3; i++) {
    ros::Rate loop_rate(1);
    success = pcl_ros::transformPointCloud(req.base_frame_id, *cloud, *filtered_cloud, listener);
    if (success) break;
    loop_rate.sleep();
  }
  if (!success) {
    return (false);
  }

  // exchange type
  //pcl::fromROSMsg (*filtering_cloud, *filtered_cloud);

  // check whether it's doing well so far
  std::cout << "PointCloud before filtering has: " << filtered_cloud->points.size () << " data points." << std::endl;

  // add filter to cut unnecessary scene point cloud around the target object
  pass_filter.setInputCloud(filtered_cloud);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(req.z_min, req.z_max);
  pass_filter.filter(*filtered_cloud);

  pass_filter.setInputCloud(filtered_cloud);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(req.x_min, req.x_max);
  pass_filter.filter(*filtered_cloud);

  pass_filter.setInputCloud(filtered_cloud);
  pass_filter.setFilterFieldName("y");
  pass_filter.setFilterLimits(req.y_min, req.y_max);
  pass_filter.filter(*filtered_cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> uniform_sampling;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  uniform_sampling.setInputCloud (filtered_cloud);
  uniform_sampling.setLeafSize (0.01f, 0.01f, 0.01f);
  uniform_sampling.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  pcl::PCDWriter writer;
  /*
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE); // model, SACMODEL_PLANE is the largest plane
  seg.setMethodType (pcl::SAC_RANSAC);  // method
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);  // if true, extract plane. if false, extract except plane

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  */

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (80);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud_cluster, xyz_centroid);  // calculate gravity

    // maybe still camera frame
    std::cout << "center of gravity : " << std::endl << xyz_centroid << std::endl;

    geometry_msgs::Point object_point;

    object_point.x = xyz_centroid[0];
    object_point.y = xyz_centroid[1];
    object_point.z = xyz_centroid[2];

    res.object_positions.push_back(object_point);

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
    j++;
  }
  return (true);
}

int main (int argc, char *argv[])
{
  // Initialize ROS
  ros::init (argc, argv, "clustering_ros_srv_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber enable_sub = nh.subscribe("enable_detection", 1, &enableCallback);
  ros::Subscriber sub;
  ros::ServiceServer clustering_ros_server;
  clustering_ros_server = nh.advertiseService("clustering_ros_srv", getPosition);

  ros::Rate rate(50);
  enableSwitched = true;

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    if (enableSwitched) {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud processing nodelet
      // lazy publishing policy; in CPU-scarce computer as TurtleBot's laptop this is a huge saving

      if (enabled) {
        //sub = nh.subscribe ("/camera/depth/points", 1, getScene);
        sub = nh.subscribe ("/camera/depth_registered/points", 1, getScene);
        //pub = nh.advertise<geometry_msgs::PoseStamped> ("position_out", 1);
      } else {
        sub.shutdown();
        //clustering_ros_server.shutdown();
      }
      enableSwitched = false;
    }
  }

  return (0);
}
