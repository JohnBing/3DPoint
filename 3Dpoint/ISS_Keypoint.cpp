//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/io.h>
//#include <pcl/keypoints/iss_3d.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std;
//
//int main(int, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
//	{
//		PCL_ERROR("Could not read file\n");
//	}
//	cout << "读取点云个数: " << cloud->points.size() << endl;
//
//	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//
//	iss.setInputCloud(cloud);
//	iss.setSearchMethod(tree);
//	iss.setSalientRadius(0.1f);//设置用于计算协方差矩阵的球邻域半径
//	iss.setNonMaxRadius(0.005f);//设置非极大值抑制应用算法的半径
//	iss.setThreshold21(0.65); //设定第二个和第一个特征值之比的上限
//	iss.setThreshold32(0.5);  //设定第三个和第二个特征值之比的上限
//	iss.setMinNeighbors(10); //在应用非极大值抑制算法时，设置必须找到的最小邻居数
//	iss.setNumberOfThreads(4); //初始化调度器并设置要使用的线程数
//	iss.compute(*keypoints);
//
//	cout << "ISS_3D points 的提取结果为 " << keypoints->points.size() << endl;
//	//pcl::io::savePCDFile("keypoints_iss_3d.pcd", *keypoints, true);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D ISS"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 225, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
//	viewer->addPointCloud<pcl::PointXYZ>(keypoints, "sample cloud1");//特征点
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0.0, 0.0, "sample cloud1");
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100));
//	}	
//
//	return 0;
//}
