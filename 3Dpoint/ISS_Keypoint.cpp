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
//	cout << "��ȡ���Ƹ���: " << cloud->points.size() << endl;
//
//	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//
//	iss.setInputCloud(cloud);
//	iss.setSearchMethod(tree);
//	iss.setSalientRadius(0.1f);//�������ڼ���Э��������������뾶
//	iss.setNonMaxRadius(0.005f);//���÷Ǽ���ֵ����Ӧ���㷨�İ뾶
//	iss.setThreshold21(0.65); //�趨�ڶ����͵�һ������ֵ֮�ȵ�����
//	iss.setThreshold32(0.5);  //�趨�������͵ڶ�������ֵ֮�ȵ�����
//	iss.setMinNeighbors(10); //��Ӧ�÷Ǽ���ֵ�����㷨ʱ�����ñ����ҵ�����С�ھ���
//	iss.setNumberOfThreads(4); //��ʼ��������������Ҫʹ�õ��߳���
//	iss.compute(*keypoints);
//
//	cout << "ISS_3D points ����ȡ���Ϊ " << keypoints->points.size() << endl;
//	//pcl::io::savePCDFile("keypoints_iss_3d.pcd", *keypoints, true);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D ISS"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 225, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
//	viewer->addPointCloud<pcl::PointXYZ>(keypoints, "sample cloud1");//������
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
