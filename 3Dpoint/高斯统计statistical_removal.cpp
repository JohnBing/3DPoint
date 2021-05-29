//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <time.h>
//
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	//viewer->initCameraParameters();
//	return (viewer);
//}
//
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::PCDReader reader;
//	
//	reader.read<pcl::PointXYZ>("data/zhou_origin.pcd", *cloud);
//	std::cerr << "Cloud before filtering: " << std::endl;
//	std::cerr << *cloud << std::endl;
//
//	clock_t start = clock();
//	// 创建滤波器对象
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud(cloud);
//	sor.setMeanK(200);
//	sor.setStddevMulThresh(0.5);
//	sor.filter(*cloud_filtered);
//
//	std::cerr << "Cloud after filtering: " << std::endl;
//	std::cerr << *cloud_filtered << std::endl;
//	clock_t end = clock();
//	cout << "time:" << end - start << endl;
//
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ>("data/zhou_origin_filtered.pcd", *cloud_filtered, false);
//	//sor.setNegative(true);
//	//sor.filter(*cloud_filtered);
//	//writer.write<pcl::PointXYZ>("part2_s_outliers.pcd", *cloud_filtered, false);
//
//	pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud_filtered);
//	viewer->spin();
//
//
//	system("pause");
//	return (0);
//}
