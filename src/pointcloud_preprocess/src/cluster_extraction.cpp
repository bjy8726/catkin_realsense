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
#include <pcl/visualization/pcl_visualizer.h>//可视化相关头文件
 
 
int main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("/home/bianjingyang/catkin_realsense/src/io_files/test_segmentation.pcd", *cloud);
  std::cout << "PointCloud before voxel filtering has: " << cloud->points.size () << " data points." << std::endl; //*
 
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  /*
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after voxel filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
 */

 
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);//将无法提取平面的点云作为cloud_filtered
 
  std::vector<pcl::PointIndices> cluster_indices;//保存每一种聚类，每一种聚类下还有具体的聚类的点
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//实例化一个欧式聚类提取对象
  ec.setClusterTolerance (0.005); // 近邻搜索的搜索半径为2cm，重要参数
  ec.setMinClusterSize (100);//设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize (2500000);//一个聚类最大点数目为25000
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (cloud);//设置输入点云
  ec.extract (cluster_indices);//将聚类结果保存至cluster_indices中
 
  //迭代访问点云索引cluster_indices，直到分割出所有聚类,一个循环提取出一类点云
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster，直到分割出所有聚类
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
 
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    pcl::io::savePCDFile(ss.str(), *cloud_cluster);
    j++;
  }
 

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3d viewer"));
  int v1(0);
  viewer->createViewPort(0,0,0.5,1,v1);
  viewer->setBackgroundColor(0,0,0,v1);
  viewer->addPointCloud(cloud,"cloud",v1);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud",v1);
 
  //第二个视口，显示分割聚类后的点云
  //读入每一个点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr view0(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr view1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr view2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr view3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr view4(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("cloud_cluster_0.pcd",*view0);
  pcl::io::loadPCDFile("cloud_cluster_1.pcd",*view1);
  pcl::io::loadPCDFile("cloud_cluster_2.pcd",*view2);
  pcl::io::loadPCDFile("cloud_cluster_3.pcd",*view3);
  pcl::io::loadPCDFile("cloud_cluster_4.pcd",*view4);
 
  int v2(0);
  viewer->createViewPort(0.5,0,1,1,v2);
  viewer->setBackgroundColor(1,1,1,v2);

  viewer->addPointCloud(view0,"view0",v2);
  viewer->addPointCloud(view1,"view1",v2);
  viewer->addPointCloud(view2,"view2",v2);
  viewer->addPointCloud(view3,"view3",v2);
  viewer->addPointCloud(view4,"view4",v2);

  std::cout<<"added!"<<std::endl;
 
  // 设置点云显示的颜色 
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"view0",v2);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"view1",v2);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"view2",v2);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,126,123,0,"view3",v2);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"view4",v2);

  while (!viewer->wasStopped())
  {
      viewer->spinOnce();
  }
  return (0);
}