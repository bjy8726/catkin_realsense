#include "surface_reconstruction/visualization.h"

void
PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
    Point &p = cloud->at (i);
    if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}

void
visualizeCurve (ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  /** \brief Converts an openNURBS NurbsCurve, defined on an NurbsSurface to a sequence of points 'cloud',
    * by ELEMENT-WISE sampling of the curve according to the resolution specified.
    *  \param[in] nurbs The openNURBS surface.
    *  \param[out] cloud The actual vertices (point-cloud).
    *  \param[in] resolution number of sampling points within one NurbsCurve element. */
  pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve, surface, curve_cloud, 4);
  /*
  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
  {
    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
    std::ostringstream os;
    os << "line" << i;
    //显示坐标点连线
    //viewer.removeShape (os.str ());
    //viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
  }*/
      //显示坐标点
    viewer.removePointCloud ("curve_cloud");
    viewer.addPointCloud (curve_cloud, "curve_cloud");


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < curve.CVCount (); i++)
  {/*CVCount函数获得nurbs曲线的控制顶点数量*/
    ON_3dPoint p1;
    //获取NURBS为有理时的欧式cv
    curve.GetCV (i, p1);
    /*
    std::cout<<"x1:"<<p1.x<<endl;
    std::cout<<"y1:"<<p1.x<<endl;
    std::cout<<"z1:"<<p1.z<<endl;
    std::cout<<endl;*/
    
    double pnt[3];
    //evaluate()函数应该是给出nurbs曲线，在曲面上对应的坐标
    surface.Evaluate (p1.x, p1.y, 0, 3, pnt);
    pcl::PointXYZRGB p2;
    p2.x = float (pnt[0]);
    p2.y = float (pnt[1]);
    p2.z = float (pnt[2]);

    p2.r = 0;
    p2.g = 255;
    p2.b = 0;
    /*
    std::cout<<"x:"<<p2.x<<endl;
    std::cout<<"y:"<<p2.y<<endl;
    std::cout<<"z:"<<p2.z<<endl;
    std::cout<<endl;
    */
    curve_cps->push_back (p2);
  }
  //viewer.removePointCloud ("cloud_cps");
  //viewer.addPointCloud (curve_cps, "cloud_cps");
}