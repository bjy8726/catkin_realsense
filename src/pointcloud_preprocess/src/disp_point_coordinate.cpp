#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//添加
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

using namespace std;


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "VoxelGrid");
    ros::NodeHandle nh;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("pointcloud.pcd", *cloud)==-1)
    {
            PCL_ERROR("couldn't read file pointcloud.pcd");
            return -1;
    }

    std::cout << "Loaded "
                << cloud->width*cloud->height
                << " data points from pointcloud.pcd with the following fields: "
                << std::endl;

    for (size_t i=0;i<cloud->points.size();i++)
    {
        std::cout << " " << cloud->points[i].x
                    << " " << cloud->points[i].y
                    << " " << cloud->points[i].z
                    << std::endl;
    }

    return 0;
}