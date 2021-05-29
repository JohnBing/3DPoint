//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <fstream>
//#include<unordered_set>
//#include<queue>
//#include<pcl/common/geometry.h>
//#include  <time.h> 
//
//
//
//using namespace std;
//
//void kdtreeDis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int K)
//{
//	
//	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	kdtree.setInputCloud(cloud);
//
//	clock_t start = clock();  //时间起始 
//
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		auto searchPoint = cloud->points[i];
//		std::vector<int> pointIdxNKNSearch(K);
//		std::vector<float> pointNKNSquaredDistance(K);
//		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//	}
//
//	clock_t end = clock(); //时间测试结束
//	cout <<"kdtree time"<<end - start << endl; //计算打印出运行时间,单位ms
//}
//
//
//void octreeDis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int K)
//{
//	
//	//创建octree
//	float resolution = 0.128;
//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
//	
//	octree.setInputCloud(cloud);
//	octree.addPointsFromInputCloud();
//
//	clock_t start = clock();  //时间起始 
//
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		auto searchPoint = cloud->points[i];
//		std::vector<int> pointIdxNKNSearch(K);
//		std::vector<float> pointNKNSquaredDistance(K);
//		octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//	}
//
//	clock_t end = clock(); //时间测试结束
//	cout << "octree time" << end - start << endl; //计算打印出运行时间,单位ms
//}
//
//
//void arrayDis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int K)
//{
//	clock_t start = clock();  //时间起始 
//
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		vector<float> dists;
//		set<float, less<float>> minDist;
//		for (int j = 0; j < cloud->points.size(); j++)
//		{
//			float dist = pcl::geometry::squaredDistance(cloud->points[0], cloud->points[j]);
//			dists.push_back(dist);
//			if (minDist.size() >= K)
//				minDist.erase(minDist.begin());
//			minDist.insert(dist);
//		}
//		sort(dists.begin(), dists.end());
//	}
//
//	clock_t end = clock(); //时间测试结束
//	cout << "array time" << end - start << endl; //计算打印出运行时间,单位ms
//
//}
//
//
//int main()
//{
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("data/zhou_s_100000.pcd", *cloud);
//	cout << cloud->size()<<endl;
//	int K = 20;
//
//	kdtreeDis(cloud,K);
//
//	octreeDis(cloud, K);
//
//	arrayDis(cloud,K);
//
//
//	system("pause");
//
//}