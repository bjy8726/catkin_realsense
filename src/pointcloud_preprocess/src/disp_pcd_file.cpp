#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>
#include <pcl/features/normal_3d.h>

using namespace std;

/*只显示点云的坐标*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  return (viewer);
}

/*显示带颜色点云*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/*显示点云的法向量*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

string filename = "test_pointcloud_filtered.pcd";

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "VoxelGrid");
    ros::NodeHandle nh;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

    /*加载pcd文件，赋值给pcl::PointCloud<pcl::PointXYZ>类型的 指针cloud*/
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
            PCL_ERROR("couldn't read file pointcloud.pcd");
            return -1;
    }

    /*加载pcd文件，赋值给pcl::PointCloud<pcl::PointXYZRGB>类型的 指针cloud*/
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloudRGB) == -1)
    {
            PCL_ERROR("couldn't read file pointcloud.pcd");
            return -1;
    }

    std::cout << "Loaded "
                << cloud->width*cloud->height
                << " data points from pointcloud.pcd with the following fields: "
                << std::endl;

    /*输出pcd文件中各个点的坐标*/
    for (size_t i=0;i<cloud->points.size();i++)
    {
        std::cout << " " << cloud->points[i].x
                    << " " << cloud->points[i].y
                    << " " << cloud->points[i].z
                    << std::endl;
    }

    /*可视化点云*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    /*只显示xyz信息*/
    //viewer = simpleVis(cloud);

    /*显示带颜色的xyz信息*/
    viewer = rgbVis(cloudRGB);

    
    /*计算点云的法向量*/
    /*
    // 0.05Ϊ�����뾶��ȡ���Ʒ���
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloudRGB);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);
    //  0.1Ϊ�����뾶��ȡ���Ʒ���
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.1);
    ne.compute (*cloud_normals2);
    //显示点云
    viewer = normalsVis(cloudRGB, cloud_normals2);
    */

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}