//#include <pcl/io/pcd_io.h>
//#include <pcl/common/centroid.h>
//
//#include <iostream>
//
//int
//main(int argc, char** argv)
//{
//	// �������ƵĶ���
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	// ��ȡ����
//	pcl::io::loadPCDFile<pcl::PointXYZ>("part1_s.pcd", *cloud);
//	
//	//cloud->points.push_back(pcl::PointXYZ(0,1,1));
//	//cloud->points.push_back(pcl::PointXYZ(2, 3, 4));
//
//	// �����洢�������ĵĶ���
//	Eigen::Vector4f centroid;
//
//	//�����Ͼ��Ǽ���xyz�������ƽ��ֵ
//	pcl::compute3DCentroid(*cloud, centroid);
//
//	std::cout << "The XYZ coordinates of the centroid are: ("
//		<< centroid[0] << ", "
//		<< centroid[1] << ", "
//		<< centroid[2] << ")." << std::endl;
//	system("pause");
//}