#include <ros/ros.h>
// PCL specific includes
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
#include <iostream>

using namespace std;
 
ros::Publisher pub;

/*直通滤波*/
pcl::PCLPointCloud2 passthrough(pcl::PCLPointCloud2* msg)
{
    /*直通滤波*/
    pcl::PCLPointCloud2 passthrough_filtered_x; 
    pcl::PCLPointCloud2 passthrough_filtered_y; 
    pcl::PCLPointCloud2 passthrough_filtered_z; 

    /*进行X方向直通滤波*/ 
    pcl::PCLPointCloud2ConstPtr cloudPtr_x(msg);
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloudPtr_x);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.05, 0.05);
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

    return passthrough_filtered_z; 
}



/*crophull滤波*/
pcl::PointCloud<pcl::PointXYZ>::Ptr crophull(pcl::PointCloud<pcl::PointXYZ>::Ptr msg)
{
    /*设置封闭范围顶点*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0.1));
    boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1,0.1 ));
    boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1,0.1 ));

    /*构造凸包*/
    pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(boundingbox_ptr);
	hull.setDimension(2);
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surface_hull, polygons);

    /*crophull滤波*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::CropHull<pcl::PointXYZ> bb_filter;
	bb_filter.setDim(2);
	bb_filter.setInputCloud(msg);
	bb_filter.setHullIndices(polygons);
	bb_filter.setHullCloud(surface_hull);
	bb_filter.filter(*objects);
    return objects;
}

/*统计学滤波*/
pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (msg);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*objects);
    return objects;
}

/*体素滤波*/
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr msg)
{
    /*进行体素滤波*/ 
    /*定义指针时一定要记得分配指向的内存空间*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filtered(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (msg);
    sor.setLeafSize (0.005, 0.005, 0.005);
    // apply filter
    sor.filter (*voxelgrid_filtered);
    return voxelgrid_filtered;
}

void preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,string outputFile)
{


    /*直通滤波*/
    /*
    pcl::PCLPointCloud2* passthrough_filtered = new pcl::PCLPointCloud2; 
    *passthrough_filtered = passthrough(cloud);*/
    
    /*CropHull滤波*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr crophull_output(new pcl::PointCloud<pcl::PointXYZ>);
    crophull_output = crophull(inputCloud);
    
    /*统计学滤波*/
    /*有过一次测试，在CropHull滤波之后，在进行统计学滤波容易产生空洞*/
    //pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_output(new pcl::PointCloud<pcl::PointXYZ>);
    //statistical_output = statisticalRemoval(crophull_output);
    
    /*体素滤波*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_output(new pcl::PointCloud<pcl::PointXYZ>);
    voxelgrid_output = voxelGrid(crophull_output); 
    
    /*保存滤波后的点云文件*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output = voxelgrid_output;
    pcl::io::savePCDFileASCII(outputFile, *output);
    cout<<"pointcloud_filtered size = "<<output->points.size()<<endl;
} 

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "VoxelGrid");
    ros::NodeHandle nh;
    string inputFile = "test_pointcloud_init.pcd";
    string outputFile = "./888test_pointcloud_filtered.pcd";
    /*加载pcd文件并保存为inputCloud*/
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    if (pcl::io::loadPCDFile(inputFile, inputCloud) == -1)
    {
        ROS_INFO("couldn't read file ");
        return -1;
    }
    else
    {
        ROS_INFO("Read %s!",inputFile.c_str());
        /*进行预处理工作*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
        *input = inputCloud;
        preprocess(input,outputFile);
        ROS_INFO("Preprocessing is Completed! ");
    }
    return 0;
}