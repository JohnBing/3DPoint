//#include <iostream>
//#include <Eigen/Dense>
//#include <pcl/common/transforms.h>
//#include<math.h>
//#include<time.h>
//#include<pcl/visualization/pcl_visualizer.h>
//#include<pcl/io/pcd_io.h>
//
//using namespace Eigen;
//using namespace std;
//
//void tranformPCL(Vector3f line, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed)
//{
//	double a = line(0), b = line(1), c = line(2);
//	double alpha = asin(b / sqrt(b*b + c * c));
//	double beita = asin(a / sqrt(a*a + b * b + c * c));
//
//	Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
//
//	transform_x(1, 1) = cos(alpha);
//	transform_x(1, 2) = sin(alpha);
//	transform_x(2, 1) = -sin(alpha);
//	transform_x(2, 2) = cos(alpha);
//
//	transform_y(0, 0) = cos(beita);
//	transform_y(0, 2) = -sin(beita);
//	transform_y(2, 0) = sin(beita);
//	transform_y(2, 2) = cos(beita);
//
//	Eigen::Matrix4f R = transform_y * transform_x.inverse();
//
//	pcl::transformPointCloud(*cloud, *transformed, R);
//
//}
//
//int main()
//{
//
//	Vector3f line(0, 1, 0);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::io::loadPCDFile("part1_s.pcd", *cloud);
//
//	tranformPCL(line,cloud,transformed) ;
//
//	
//
//	// Visualization
//	printf("\nPoint cloud colors :  white  = original point cloud\n"
//		"                        red  = transformed point cloud\n");
//
//	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 255, 255);
//	viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed, 230, 20, 20); // Red
//	viewer.addPointCloud(transformed, transformed_cloud_color_handler, "transformed_cloud");
//	viewer.addCoordinateSystem(0.1, "cloud", 0);
//	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//	//viewer.setPosition(800, 400); // Setting visualiser window position
//	viewer.spin();
//	
//}
//
