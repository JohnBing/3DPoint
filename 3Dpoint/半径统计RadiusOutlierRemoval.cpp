//#include<iostream>
//#include<pcl\point_cloud.h>
//#include<pcl\point_types.h>
//#include<pcl\io\io.h>
//#include<pcl\io\pcd_io.h>
//#include<pcl\filters\radius_outlier_removal.h>
//#include<time.h>
//using namespace std;
//int main()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("data/zhou_origin.pcd", *cloud);
//	
//	clock_t start = clock();
//
//	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//	outrem.setInputCloud(cloud);
//	outrem.setRadiusSearch(0.8);
//	outrem.setMinNeighborsInRadius(200);
//	outrem.filter(*cloud_filtered);
//
//	clock_t end = clock();
//	cout << "time:" << end - start << endl;
//
//	/*std::cerr << "cloud before filtering:" << std::endl;
//	for (size_t i = 0; i < cloud->points.size(); i++)
//		std::cerr << ' ' << cloud->points[i].x << ' ' << cloud->points[i].y << ' ' << cloud->points[i].z << std::endl;
//	std::cerr << "cloud after filtering:" << std::endl;
//	for (size_t i = 0; i < cloud_filtered->points.size(); i++)
//		std::cerr << ' ' << cloud_filtered->points[i].x << ' ' << cloud_filtered->points[i].y << ' ' << cloud->points[i].z << std::endl;*/
//	std::cerr << "cloud before filtering:" << cloud->size()<<::endl;
//	std::cerr << "cloud after filtering:" << cloud_filtered->size()<<std::endl;
//
//	pcl::io::savePCDFile("data/zhou_origin_filtered.pcd", *cloud_filtered);
//	
//	system("pause");
//	return 0;
//}