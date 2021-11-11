#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

using namespace std;
 
ros::Publisher pub;
 
void 
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
 
  pcl::io::savePCDFile("/home/bianjingyang/catkin_realsense/src/io_files/pointcloud_init.pcd", *cloud_msg);
  cout<<"publish point_cloud height = "<<cloud_msg->height<<endl;
  cout<<"publish point_cloud width = "<<cloud_msg->width<<endl;
}
 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sample");
  ros::NodeHandle nh;
  /*
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_VoxelGrid", 1);
  */

  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloud;
  pointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/color/points", ros::Duration(10));
 

 cloud_cb(pointCloud);

   ros::shutdown();

  return 0;
}