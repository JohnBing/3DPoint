//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/features/boundary.h>
//#include <boost/make_shared.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/features/normal_3d.h>
//
//using namespace std;
//
//int
//main()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
//	cout << "加载点云" << cloud->points.size() << "个" << endl;
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//计算法向量
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setRadiusSearch(1);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	n.compute(*normals);
//
//	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;//边界特征估计
//	boundEst.setInputCloud(cloud);
//	boundEst.setInputNormals(normals);
//	boundEst.setRadiusSearch(1);
//	boundEst.setAngleThreshold(M_PI / 4);//边界判断时的角度阈值
//
//	boundEst.setSearchMethod(tree);
//	pcl::PointCloud<pcl::Boundary> boundaries;
//	boundEst.compute(boundaries);
//	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//
//		if (boundaries[i].boundary_point > 0)
//		{
//			cloud_boundary->push_back(cloud->points[i]);
//		}
//	}
//	cout << "边界点个数:" << cloud_boundary->points.size() << endl;
//	//pcl::io::savePCDFileASCII("75m2YY11.pcd", *cloud_boundary);
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("ShowCloud"));
//	int v1(0);
//	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	view->setBackgroundColor(0.3, 0.3, 0.3, v1);
//	view->addText("Raw point clouds", 10, 10, "v1_text", v1);
//	int v2(0);
//	view->createViewPort(0.5, 0.0, 1, 1.0, v2);
//	view->setBackgroundColor(0.5, 0.5, 0.5, v2);
//	view->addText("Boudary point clouds", 10, 10, "v2_text", v2);
//
//	view->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
//	view->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);
//	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
//	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
//	//view->addCoordinateSystem(1.0);
//	//view->initCameraParameters();
//	while (!view->wasStopped())
//	{
//		view->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	return 0;
//}
