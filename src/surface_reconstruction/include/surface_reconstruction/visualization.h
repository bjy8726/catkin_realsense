#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include "surface_reconstruction/triangulation_modify.h"



typedef pcl::PointXYZ Point;

void PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);

void visualizeCurve (ON_NurbsCurve &curve,
                ON_NurbsSurface &surface,
                pcl::visualization::PCLVisualizer &viewer);