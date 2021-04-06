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
 

void preprocess(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  //原始的点云文件
  pcl::io::savePCDFile("./test_pointcloud_init.pcd", *cloud_msg);
  cout<<"pointcloud_init height = "<<cloud_msg->height<<endl;
  cout<<"pointcloud_init width = "<<cloud_msg->width<<endl;

  /*以下工作为滤波部分*/
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered; 
  // Convert ros type(*cloud_msg) to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  /*进行直通滤波*/ 


  /*进行体素滤波*/ 
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // build the filter
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.005, 0.005, 0.005);
  // apply filter
  sor.filter (cloud_filtered);
  // Convert pcl type to ROS data type
  sensor_msgs::PointCloud2 cloud_vog;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_vog);
 

  //滤波后的点云文件
  pcl::io::savePCDFile("./test_pointcloud_filtered.pcd", cloud_vog);
  cout<<"pointcloud_filtered height = "<<cloud_vog.height<<endl;  /*若height为1则表示点云为无序点*/
  cout<<"pointcloud_filtered width = "<<cloud_vog.width<<endl;
}
 

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "VoxelGrid");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
 
  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloud;
  pointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(10));
  if (pointCloud!=NULL)
  {
    preprocess(pointCloud);
  }
  else
  {
    ROS_ERROR("Can't sub topic /camera/depth_registered/points!");
  }
  
  ROS_INFO("successfully!");

 
  // Spin
  //ros::spin ();
}