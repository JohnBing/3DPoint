//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <fstream>
//#include<unordered_set>
//#include<queue>
//#include<pcl/conversions.h>
//#include<pcl/common/geometry.h>
//using namespace std;
//
//
//int main()
//{
//
//	pcl::PolygonMesh mesh;
//	pcl::io::loadPLYFile("zhou_merge.ply", mesh);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPolygonMesh(mesh, "my");
//	viewer->spin();
//
//	return 0;
//}