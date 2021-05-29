//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include<vector>
//using namespace std;
//
//class Solution
//{
//public:
//	//计算整个点云的平均分辨率
//	double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//	{
//		double res = 0.0;
//		int n_points = 0;
//		int nres;
//		std::vector<int> indices(2);
//		std::vector<float> sqr_distances(2);
//		pcl::KdTreeFLANN<pcl::PointXYZ> tree;
//		tree.setInputCloud(cloud);
//
//		for (size_t i = 0; i < cloud->size(); ++i)
//		{
//			if (!pcl_isfinite((*cloud)[i].x))
//			{
//				continue;
//			}
//			//Considering the second neighbor since the first is the point itself.
//			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
//			if (nres == 2)
//			{
//				res += sqrt(sqr_distances[1]);
//				++n_points;
//			}
//		}
//		if (n_points != 0)
//		{
//			res /= n_points;
//		}
//		return res;
//	}
//};
//
//int main()
//{
//	pcl::PCDReader reader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	reader.read("data/tai_s.pcd", *cloud);
//
//
//	Solution obj;
//	cout<<obj.computeCloudResolution(cloud);
//	system("pause");
//}
