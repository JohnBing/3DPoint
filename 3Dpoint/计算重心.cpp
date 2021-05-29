//#include <pcl/io/pcd_io.h>
//#include <pcl/common/centroid.h>
//
//#include <iostream>
//
//int
//main(int argc, char** argv)
//{
//	// 创建点云的对象
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	// 读取点云
//	pcl::io::loadPCDFile<pcl::PointXYZ>("part1_s.pcd", *cloud);
//	
//	//cloud->points.push_back(pcl::PointXYZ(0,1,1));
//	//cloud->points.push_back(pcl::PointXYZ(2, 3, 4));
//
//	// 创建存储点云重心的对象
//	Eigen::Vector4f centroid;
//
//	//本质上就是计算xyz三个轴的平均值
//	pcl::compute3DCentroid(*cloud, centroid);
//
//	std::cout << "The XYZ coordinates of the centroid are: ("
//		<< centroid[0] << ", "
//		<< centroid[1] << ", "
//		<< centroid[2] << ")." << std::endl;
//	system("pause");
//}