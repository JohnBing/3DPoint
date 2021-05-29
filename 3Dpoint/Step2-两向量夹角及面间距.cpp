//#include <iostream>
//#include <Eigen/Dense>
//#include <pcl/io/pcd_io.h>
////g++ mat.cpp -o mat -I/download/eigen
//using namespace Eigen;
//using namespace std;
//typedef pcl::PointXYZ PointT;
//float computeDist(pcl::PointCloud<PointT>::Ptr cloud)
//{
//	//Vector3f line1(0.061150, 0.869208, 0.490650);
//	//Vector3f  point1(140.651154,19.397173,284.8396);
//	//float D = -line1.dot(point1);
//
//	Vector3f line1(0.00617783,- 0.820735,- 0.571276);
//	float D = 147.794;
//
//	float num = 0;
//	float minValue = 100,maxValue=0;
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//		float dist = (line1.dot(point) + D) / line1.norm();
//		cout << dist << endl;
//		minValue = min(minValue, abs(dist));
//		maxValue = max(maxValue, abs(dist));
//		num += dist;
//	}
//	
//	cout << "minValue" << minValue << endl;
//	cout << "maxValue" << maxValue << endl;
//	cout << "maxValue-minValue" << maxValue-minValue << endl;
//
//	return num / cloud->points.size();
//}
//void computeAngle()
//{
//	cout << acos(1);
//	//Vector3f line1(-0.005588, 0.824198, 0.566274), line2(0.008899, 0.842967, 0.537891);//1-2
//	//Vector3f line1(0.061150, 0.869208, 0.490650), line2(0.008899, 0.842967, 0.537891);//1-3
//	Vector3f line1(-0.005588, 0.824198, 0.566274), line2(0.061150, 0.869208, 0.490650);//2-3
//	cout << acos(line1.dot(line2) / line1.norm()*line2.norm()) << endl;
//
//}
//
//
//int main()
//{
//	pcl::PCDReader reader;
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	reader.read("data/tai/tai_cloud_p2-6.pcd", *cloud);
//
//	cout<<"average:"<<computeDist(cloud);
//
//	system("pause");
//}