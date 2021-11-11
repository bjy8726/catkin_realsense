#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/features/principal_curvatures.h>


using namespace std;
using namespace pcl::console;
typedef pcl::PointXYZ Point;
 

/*显示带颜色点云*/
void display_pcd(std::string pcd_file,std::string window_name)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

 pcl::io::loadPCDFile(pcd_file, *cloudRGB);

  pcl::visualization::PCLVisualizer viewer(window_name);
  viewer.setBackgroundColor (255, 255, 255);;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRGB);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloudRGB, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();

  viewer.spin();
}



void preprocess(const pcl::PCLPointCloud2::Ptr cloud,string output_file)
{

  /*以下工作为滤波部分*/

  pcl::PCLPointCloud2 passthrough_filtered_x;
  pcl::PCLPointCloud2 passthrough_filtered_y;
  pcl::PCLPointCloud2 passthrough_filtered_z;
  pcl::PCLPointCloud2 statistical_output; 
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
  /*
  pcl::PCLPointCloud2* cloud_z = new pcl::PCLPointCloud2;
  *cloud_z = passthrough_filtered_y;
  pcl::PCLPointCloud2ConstPtr cloudPtr_z(cloud_z);
  pass.setInputCloud (cloudPtr_z);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 0.4);
  //pass.setFilterLimitsNegative (true);
  pass.filter (passthrough_filtered_z);
  */


  /*统计学滤波:移除离群点*/
  /*
  pcl::PCLPointCloud2* cloud_statistiacal = new pcl::PCLPointCloud2;
  *cloud_statistiacal = passthrough_filtered_y;
  pcl::PCLPointCloud2ConstPtr cloudStatistiacalPtr(cloud_statistiacal);

  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> statisticalRemoval;
  statisticalRemoval.setInputCloud (cloudStatistiacalPtr);
  statisticalRemoval.setMeanK (50);
  statisticalRemoval.setStddevMulThresh (1.0);
  statisticalRemoval.filter (statistical_output);
  */
  
  /*进行体素滤波*/ 
  /*
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // build the filter
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  *cloud2 = statistical_output;
  pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud2); //此处的cloud2不能直接用&passthrough_filtered替代，否则会出错，必须重新创建一个对象。
  sor.setInputCloud (cloudPtr2);
  sor.setLeafSize (0.005, 0.005, 0.005);
  // apply filter
  sor.filter (voxelgrid_filtered);
  */
  
  /*保存滤波后的点云文件*/
  // Convert pcl type to pcd data type
  sensor_msgs::PointCloud2 cloud_vog;
  pcl_conversions::moveFromPCL(passthrough_filtered_y, cloud_vog);
  pcl::io::savePCDFile(output_file, cloud_vog);
  cout<<"pointcloud_filtered height = "<<cloud_vog.height<<endl;  /*若height为1则表示点云为无序点*/
  cout<<"pointcloud_filtered width = "<<cloud_vog.width<<endl;

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr  segmentation(string inputFile,string output_file){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile(inputFile, *cloud) == -1)
    {
        ROS_INFO("couldn't read file ");
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                         //holder
	{
		if ((cloud_HSV->points[i].v < 0.5 && cloud_HSV->points[i].h > 0))            // increase the first parameter to have better visualisation of holder
		{
			// 如果为边界颜色则不做操作，即不加入到结果集中去
		}else{
            inliers->indices.push_back(i);
        }
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*result);
    pcl::io::savePCDFile(output_file, *result);
	return result;
}



void getTargetRange(string inputFile,string outputFile){

}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "filter");
    ros::NodeHandle nh;
    string inputFile = "/home/bianjingyang/catkin_realsense/src/io_files/test.pcd";
    string outputFile = "/home/bianjingyang/catkin_realsense/src/io_files/test_filtered.pcd";
    string outputFile2 = "/home/bianjingyang/catkin_realsense/src/io_files/test_segmentation.pcd";
    string outputFile3 = "/home/bianjingyang/catkin_realsense/src/io_files/test_target";

    // 加载pcd文件并保存为inputCloud
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
    if (pcl::io::loadPCDFile(inputFile, *inputCloud) == -1)
    {
        ROS_INFO("couldn't read file ");
        return -1;
    }
    ROS_INFO("Read %s!",inputFile.c_str());
    
    // 先进行简单的直通滤波处理
    preprocess(inputCloud,outputFile);
    // 基于RGB颜色将边界点云去除
    segmentation(outputFile,outputFile2);
    
    // 基于欧式距离聚类的分割
    getTargetRange(outputFile2,outputFile3);


    // 显示分割后的点云
    //display_pcd(inputFile,"原始点云");
    display_pcd(outputFile3,"目标点云");


    return 0;
}