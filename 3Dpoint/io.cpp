//#include <iostream>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/filter_indices.h>
//#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//using namespace std;
//using namespace pcl;
//int main()
//{
//	//*********************************************************************************
//	//***************************read PLY file*****************************************
//	//*********************************************************************************
//
//	cout << "read ply file...\n";
//	PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
//	PointCloud<PointXYZ>::Ptr target(new pcl::PointCloud<PointXYZ>());
//	pcl::io::loadPLYFile("1.ply", *target);
//	pcl::io::loadPLYFile("2.ply", *source);
//
//	//去除无效点
//	std:vector<int> index;
//	pcl::removeNaNFromPointCloud(*source, *source, index);
//	pcl::removeNaNFromPointCloud(*target, *target, index);
//
//	//*********************************************************************************
//	//********************visualization pointcloud*************************************
//	//*********************************************************************************
//	cout << "Visualization_PointCloud...\n";
//	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Ransac"));
//	viewer->setBackgroundColor(0, 0, 0);
//	//创建窗口
//	int vp;
//	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
//	//设置点云颜色
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source, 0, 255, 255);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 255, 255, 0);
//
//	viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source", vp);
//	viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target", vp);
//
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
//	viewer->spin();
//}
