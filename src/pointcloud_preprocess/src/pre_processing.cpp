#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

using namespace std;
 
ros::Publisher pub;
 
//回调函数，只订阅一次点云话题消息。
void preprocess(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  //保存原始点云为pcd文件
  pcl::io::savePCDFile("./test_pointcloud_init.pcd", *cloud_msg);
  cout<<"pointcloud_init height = "<<cloud_msg->height<<endl;
  cout<<"pointcloud_init width = "<<cloud_msg->width<<endl;

  /*以下工作为滤波部分*/
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  // Convert ros type(*cloud_msg) to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::PCLPointCloud2 passthrough_filtered_x;
  pcl::PCLPointCloud2 passthrough_filtered_y;
  pcl::PCLPointCloud2 passthrough_filtered_z; 
  pcl::PCLPointCloud2 voxelgrid_filtered; 
  

  /*进行X方向直通滤波*/ 
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pcl::PCLPointCloud2ConstPtr cloudPtr_x(cloud);
  pass.setInputCloud (cloudPtr_x);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.2, 0.2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (passthrough_filtered_x);
  
  /*进行Y方向直通滤波*/
  pcl::PCLPointCloud2* cloud_y = new pcl::PCLPointCloud2;
  *cloud_y = passthrough_filtered_x;
  pcl::PCLPointCloud2ConstPtr cloudPtr_y(cloud_y);
  pass.setInputCloud (cloudPtr_y);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.2, 0.2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (passthrough_filtered_y);

  /*进行Z方向直通滤波*/
  pcl::PCLPointCloud2* cloud_z = new pcl::PCLPointCloud2;
  *cloud_z = passthrough_filtered_y;
  pcl::PCLPointCloud2ConstPtr cloudPtr_z(cloud_z);
  pass.setInputCloud (cloudPtr_z);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 0.3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (passthrough_filtered_z);
 
  
  /*进行体素滤波*/ 
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // build the filter
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  *cloud2 = passthrough_filtered_z;
  pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud2); //此处的cloud2不能直接用&passthrough_filtered替代，否则会出错，必须重新创建一个对象。
  sor.setInputCloud (cloudPtr2);
  sor.setLeafSize (0.005, 0.005, 0.005);
  // apply filter
  sor.filter (voxelgrid_filtered);
  
  /*保存滤波后的点云文件*/
  // Convert pcl type to pcd data type
  sensor_msgs::PointCloud2 cloud_vog;
  pcl_conversions::moveFromPCL(voxelgrid_filtered, cloud_vog);
  pcl::io::savePCDFile("./test_pointcloud_filtered.pcd", cloud_vog);
  cout<<"pointcloud_filtered height = "<<cloud_vog.height<<endl;  /*若height为1则表示点云为无序点*/
  cout<<"pointcloud_filtered width = "<<cloud_vog.width<<endl;
}
 

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "preprocess");
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
  ROS_INFO(" Filter Successfully!");
}