//#include <pcl/io/pcd_io.h>
//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <time.h>
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile<pcl::PointXYZ>("data/tai_s.pcd", *cloud);
//	std::cout <<"befor filtered"<< cloud->size();
//
//	clock_t start = clock();
//	// Uniform sampling object.
//	pcl::UniformSampling<pcl::PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	filter.setRadiusSearch(0.0005f);
//	// We need an additional object to store the indices of surviving points.
//	//pcl::PointCloud<int> keypointIndices;
//
//	filter.filter(*filteredCloud);
//
//  clock_t end = clock();
//  cout << "time" << end - start << endl;
//	std::cout << "after filtered" << filteredCloud->size();
//	pcl::io::savePCDFile("data/zhou_s_US.pcd", *filteredCloud);
//	//filter.compute(keypointIndices);
//	//pcl::copyPointCloud(*cloud, keypointIndices.points, *filteredCloud);
//
//	pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(filteredCloud);
//	viewer->spin();
//}