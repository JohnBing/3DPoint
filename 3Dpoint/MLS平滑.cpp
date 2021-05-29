//#include <pcl/io/pcd_io.h>
//#include <pcl/surface/mls.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <boost/thread/thread.hpp>
//int
//main(int argc, char** argv)
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
//
//	pcl::io::loadPCDFile<pcl::PointXYZ>("data/part1_s.pcd", *cloud);
//	std::cout << "befor filtered:" << cloud->size();
//	
//	// Smoothing object (we choose what point types we want as input and output).
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
//	filter.setInputCloud(cloud);
//	// Use all neighbors in a radius of 3mm.
//	filter.setSearchRadius(0.001);
//	// If true, the surface and normal are approximated using a polynomial estimation
//	// (if false, only a tangent one).
//	filter.setPolynomialFit(true);
//	// We can tell the algorithm to also compute smoothed normals (optional).
//	filter.setComputeNormals(true);
//	// kd-tree object for performing searches.
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
//	filter.setSearchMethod(kdtree);
//
//	
//	filter.process(*smoothedCloud);
//	std::cout << "after filtered:" << smoothedCloud->size();
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smooth"));
//	viewer->addPointCloud<pcl::PointNormal>(smoothedCloud, "smoothed");
//	pcl::io::savePCDFile<pcl::PointNormal>("data/part1_ssmoothed.pcd", *smoothedCloud);
//	
//	viewer->spin();
//}