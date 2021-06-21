#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "surface_reconstruction/triangulation_modify.h"
#include "surface_reconstruction/visualization.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

using namespace pcl::console;
typedef pcl::PointXYZ Point;

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "bspline_fitting_and_dispersion"); 
  ros::NodeHandle n;

  std::string pcd_file;
  std::string pose_points_file;
  int mesh_resolution;
  int mesh_resolution_trajectory;

  // parameters
  n.param<std::string>("/cloud_file_cfg/pcd_file",pcd_file,"/home/bianjingyang/catkin_realsense/test_pointcloud_filtered.pcd");
  n.param("/reconstruction/mesh_resolution",mesh_resolution,128);
  n.param("/reconstruction/mesh_resolution_trajectory",mesh_resolution_trajectory,50);
  n.param<std::string>("/trajectory_planning_files/text_cam",pose_points_file,"/home/bianjingyang/catkin_realsense/src/io_files/pose_points_file");

  unsigned order (3);
  unsigned refinement (4);
  unsigned iterations (10);

  bool two_dim=true;
  
  parse_argument (argc, argv, "-o", order);
  parse_argument (argc, argv, "-rn", refinement);
  parse_argument (argc, argv, "-in", iterations);
  parse_argument (argc, argv, "-td", two_dim);

  pcl::on_nurbs::FittingSurface::Parameter params;
  params.interior_smoothness = 0.2;
  params.interior_weight = 1.0;
  params.boundary_smoothness = 0.2;
  params.boundary_weight = 0.0;


  //如果加载的文件不正确，则要退出
 
  pcl::visualization::PCLVisualizer viewer ("B样条曲面拟合");
  viewer.setBackgroundColor(0,0,0);
  viewer.setSize (800, 600);

  // ############################################################################
  // load point cloud

  printf ("Loading: %s\n", pcd_file.c_str ());
  pcl::PointCloud<Point>::Ptr cloud_init (new pcl::PointCloud<Point>);
  pcl::PCLPointCloud2 cloud2;
  pcl::on_nurbs::NurbsDataSurface data;

  if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
    throw std::runtime_error ("PCD file not found.");

  fromPCLPointCloud2 (cloud2, *cloud_init);
  PointCloud2Vector3d (cloud_init, data.interior);
  pcl::visualization::PointCloudColorHandlerCustom<Point> handler (cloud_init, 0, 255, 0);
  viewer.addPointCloud<Point> (cloud_init, handler, "cloud_init");
  printf ("%lu points in data set\n\n", cloud_init->size());

  // ############################################################################
  // fit B-spline surface

  

  // initialize
  printf ("Surface fitting ...\n\n");
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
  pcl::on_nurbs::FittingSurface fit (&data, nurbs);
  //  fit.setQuiet (false); // enable/disable debug output

  // mesh for visualization
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> mesh_vertices;
  std::string mesh_id = "mesh_nurbs";
  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);
  viewer.addPolygonMesh (mesh, mesh_id);
  std::cout<<"Before refine"<<endl<<endl;
  viewer.spinOnce (3000);
  // surface refinement
  for (unsigned i = 0; i < refinement; i++)
  {
    fit.refine (0);
    if(two_dim)fit.refine (1);
    fit.assemble (params);
    fit.solve ();
    pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
    viewer.spinOnce (500);
	  std::cout<<"refine: "<<i<<endl;
  }

  // surface fitting with final refinement level
  for (unsigned i = 0; i < iterations; i++)
  {
    fit.assemble (params);
    fit.solve ();
    pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
    viewer.spinOnce (500);
	  std::cout<<"iterations: "<<i<<endl;
  }

  // ############################################################################
  // 拟合样条曲面的边界，用 B-spline curve 来拟合

  // parameters
  pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
  curve_params.addCPsAccuracy = 5e-2;
  curve_params.addCPsIteration = 3;
  curve_params.maxCPs = 200;
  curve_params.accuracy = 1e-3;
  curve_params.iterations = 100;

  curve_params.param.closest_point_resolution = 0;
  curve_params.param.closest_point_weight = 1.0;
  curve_params.param.closest_point_sigma2 = 0.1;
  curve_params.param.interior_sigma2 = 0.00001;
  curve_params.param.smooth_concavity = 1.0;
  curve_params.param.smoothness = 1.0;

  // initialisation (circular)
  printf ("curve fitting ...\n\n");
  pcl::on_nurbs::NurbsDataCurve2d curve_data;
  curve_data.interior = data.interior_param;
  curve_data.interior_weight_function.push_back (true);
  ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);
  // curve fitting
  pcl::on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);
  // curve_fit.setQuiet (false); // enable/disable debug output
  curve_fit.fitting (curve_params);
  //visualizeCurve (curve_fit.m_nurbs, fit.m_nurbs, viewer);

  
  
  // ############################################################################
  // 三角化拟合出来的曲面
  printf ("triangulate trimmed surface ...\n\n");
  //定义一个点云my_cloud，用来存放裁剪之后曲面的 等参数化离散点
  pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr my_normals (new pcl::PointCloud<pcl::Normal>);
  double point[9];//分别存放x y z tu[0] tu[1] tu[2] tv[0] tv[1] tv[2]
  //mesh_resolution_trajectory将u/v向分别分成这些份。
  pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh,
                                                                    my_cloud, my_normals,
                                                                   mesh_resolution_trajectory,point);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr disp_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  //显示出拟合的曲面
  viewer.removePolygonMesh (mesh_id);
  viewer.addPolygonMesh (mesh, mesh_id);

  int points;
  if(my_cloud->size()== my_normals->size())
  {
    points = my_cloud->size();
    std::cout<<"points size: "<<points<<endl;
  }
  else
  {
    std::cout<<"points equal not equal to normals"<<std::endl;
    return -1;
  }

  /*显示出拟合曲面轨迹离散点(等参数法),同时保存到文件pose_points_file*/
  fstream outfile;
  outfile.open(pose_points_file,ios::out|ios::trunc);
  if(!outfile)
  {
    std::cout<<"new or open file failed"<<endl;
    return -1;
  }

  int next_counts = 0;

  vector<vector<double>> next_points;

  for (int i = 0; i < points; i++)
  {
    pcl::PointXYZRGB p;
    p.x = my_cloud->at(i).x;
    p.y = my_cloud->at(i).y;
    p.z = my_cloud->at(i).z;
    p.r = 255;
    p.g = 0;
    p.b = 0;
    disp_cloud_rgb->push_back (p);
    if(i>=1)
    {
      double dx2 = pow((my_cloud->at(i).x - my_cloud->at(i-1).x),2);
      double dy2 = pow((my_cloud->at(i).y - my_cloud->at(i-1).y),2);
      double dz2 = pow((my_cloud->at(i).z - my_cloud->at(i-1).z),2);
      double deltad = sqrt(dx2+dy2+dz2);

      if(deltad>0.05)
      {//如果两点的距离 大于一定值，说明是下一行路径点
        if(next_counts%2 == 1)
        {//当遇到奇数个NEXT时，将之前保存的一行vector逆过来输入到文件
          for(int i=next_points.size()-1;i>=0;i--)
          {
                            //x y z
            outfile<<next_points[i][0]<<"    ";
            outfile<<next_points[i][1]<<"    ";
            outfile<<next_points[i][2]<<"    ";

            //旋转矩阵==>四元数
            outfile<<next_points[i][3]<<"    ";
            outfile<<next_points[i][4]<<"    ";
            outfile<<next_points[i][5]<<"    ";
            outfile<<next_points[i][6]<<endl;
          }
          vector<vector<double>>().swap(next_points);
        }
        
        outfile<<"NEXT"<<endl;
        next_counts++;
        
      }
    }
    
    //旋转矩阵
    Eigen::Matrix3d T;
    //tu[0] tu[1] tu[2]
    Eigen::Vector3d tmp;
    tmp[0] = point[3];
    tmp[1] = point[4];
    tmp[2] = point[5];
    double length = tmp.norm();
    tmp[0] = tmp[0]/length;
    tmp[1] = tmp[1]/length;
    tmp[2] = tmp[2]/length;
    T(0,0) = tmp[0];
    T(1,0) = tmp[1];
    T(2,0) = tmp[2];
    
    //tv[0] tv[1] tv[2]
    tmp[0] = point[6];
    tmp[1] = point[7];
    tmp[2] = point[8];
    length = tmp.norm();
    tmp[0] = tmp[0]/length;
    tmp[1] = tmp[1]/length;
    tmp[2] = tmp[2]/length;
    T(0,1) = tmp[0];
    T(1,1) = tmp[1];
    T(2,1) = tmp[2];

    //u叉乘v
    tmp[0] = my_normals->at(i).normal[0];
    tmp[1] = my_normals->at(i).normal[1];
    tmp[2] = my_normals->at(i).normal[2];
    length = tmp.norm();
    tmp[0] = tmp[0]/length;
    tmp[1] = tmp[1]/length;
    tmp[2] = tmp[2]/length;
    T(0,2) = tmp[0];
    T(1,2) = tmp[1];
    T(2,2) = tmp[2];

    //旋转矩阵==>四元数
    Eigen::Quaterniond Q;
    Q = T;

    if(next_counts%2 == 0)
    {
          //x y z
      outfile<<my_cloud->at(i).x<<"    ";
      outfile<<my_cloud->at(i).y<<"    ";
      outfile<<my_cloud->at(i).z<<"    ";


      outfile<<Q.x()<<"    ";
      outfile<<Q.y()<<"    ";
      outfile<<Q.z()<<"    ";
      outfile<<Q.w()<<endl;
    }
    else{
      vector<double> pose;
      pose.push_back(my_cloud->at(i).x);
      pose.push_back(my_cloud->at(i).y);
      pose.push_back(my_cloud->at(i).z);
      pose.push_back(Q.x());
      pose.push_back(Q.y());
      pose.push_back(Q.z());
      pose.push_back(Q.w());
      next_points.push_back(pose);
      //vector<double>().swap(pose);
      std::cout<<pose[0]<<endl;
    }



    
    //viewer.removePointCloud ("test_cloud_rgb");
    //viewer.addPointCloud (test_cloud_rgb, "test_cloud_rgb");
    //viewer.spinOnce (50);
  }
  if(!next_points.empty())
  {
    for(int i=next_points.size()-1;i>=0;i--)
    {
                      //x y z
      outfile<<next_points[i][0]<<"    ";
      outfile<<next_points[i][1]<<"    ";
      outfile<<next_points[i][2]<<"    ";

      //旋转矩阵==>四元数
      outfile<<next_points[i][3]<<"    ";
      outfile<<next_points[i][4]<<"    ";
      outfile<<next_points[i][5]<<"    ";
      outfile<<next_points[i][6]<<endl;
    }
    vector<vector<double>>().swap(next_points);
  }
  outfile<<"NEXT"<<endl;
  outfile.close();


  /*将计算出的轨迹离散点显示出来，包括法向量*/
  pcl::visualization::PCLVisualizer viewer_points ("轨迹离散点");
  viewer_points.setBackgroundColor(0,0,0);
  viewer_points.setSize (800, 600);
  viewer_points.removePointCloud ("disp_cloud_rgb");
  viewer_points.addPointCloud (disp_cloud_rgb, "disp_cloud_rgb");
  //第三个参数控制法线的显示个数，第四个参数控制法线的长短
  viewer_points.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (disp_cloud_rgb, my_normals, 10, 0.5, "normals");
  viewer_points.addCoordinateSystem (1.0);
  viewer_points.initCameraParameters ();

  
  printf ("... done.\n");

  viewer.spin ();
  //viewer_points.spin();

  ros::spin();
  return 0;
}
