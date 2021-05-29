//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include<vector>
//using namespace std;
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
//	
//	pcl::io::loadPCDFile<pcl::PointXYZ>("calibration.pcd", *cloud);
//	pcl::io::loadPCDFile<pcl::PointXYZ>("calibration.pcd", *newcloud);
//	for (int i = 0; i < newcloud->points.size(); i++)
//	{
//		newcloud->points[i].x /= 1000;
//		newcloud->points[i].y /= 1000;
//		newcloud->points[i].z /= 1000;
//	}
//	pcl::io::savePCDFile<pcl::PointXYZ>("calibration_s.pcd", *newcloud);
//
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
//	viewer->initCameraParameters();
//
//	int v1(0);
//	viewer->createViewPort(0, 0, 0.5, 1, v1);
//	viewer->setBackgroundColor(0, 0, 255, v1);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 244, 89, 233);
//	viewer->addPointCloud(cloud, color1, "cloud_old", v1);
//
//	int v2(0);
//	viewer->createViewPort(0.5, 0, 1, 1, v2);
//	viewer->setBackgroundColor(0, 255, 255, v2);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(newcloud, 244, 89, 233);
//	viewer->addPointCloud(newcloud, color2, "cloud_new", v2);
//
//
//	viewer->spin();
//
//	return (0);
//}
//
