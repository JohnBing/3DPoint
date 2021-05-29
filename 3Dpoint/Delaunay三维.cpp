//// triangulation_test.cpp: 定义控制台应用程序的入口点。
////
//
//#include<iostream>
//#include<pcl/point_types.h>
//#include<pcl/io/pcd_io.h>
//#include<pcl/io/ply_io.h>
//#include<pcl/kdtree/kdtree_flann.h>
//#include<pcl/features/normal_3d.h>
//#include<pcl/surface/gp3.h>  //贪婪投影三角化算法
//#include<pcl/visualization/pcl_visualizer.h>
//#include<boost/math/special_functions/round.hpp>
//#include  <time.h> 
//#include  <unordered_map> 
//#include  <unordered_set> 
//#include <pcl/common/io.h>
//#include <pcl/point_types.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//#include <Eigen/Dense>
//#include <pcl/common/transforms.h>
//#include "Delaunay.h"
//
//using namespace Eigen;
//using namespace std;
//
//Eigen::Matrix4f tranformMatrix(Vector3f line)
//{
//	double a = line(0), b = line(1), c = line(2);
//	double alpha = asin(b / sqrt(b*b + c * c));
//	double beita = asin(a / sqrt(a*a + b * b + c * c));
//
//	Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
//
//	transform_x(1, 1) = cos(alpha);
//	transform_x(1, 2) = sin(alpha);
//	transform_x(2, 1) = -sin(alpha);
//	transform_x(2, 2) = cos(alpha);
//
//	transform_y(0, 0) = cos(beita);
//	transform_y(0, 2) = -sin(beita);
//	transform_y(2, 0) = sin(beita);
//	transform_y(2, 2) = cos(beita);
//
//	Eigen::Matrix4f R = transform_y * transform_x.inverse();
//
//	return R;
//
//}
//
//void flip(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	int K = 10;//搜索最近的10个点,最重要的一个阈值参数
//	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	kdtree.setInputCloud(cloud);
//	unordered_set<int> isVisited;
//	queue<int> que;
//	que.push(0);
//	isVisited.insert(0);
//	while (!que.empty())
//	{
//		int i = que.front();
//		//cout << i << endl;
//		que.pop();
//
//
//		auto searchPoint = cloud->points[i];
//		std::vector<int> pointIdxNKNSearch(K);
//		std::vector<float> pointNKNSquaredDistance(K);
//		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//		for (int next : pointIdxNKNSearch)
//		{
//			if (isVisited.count(next)) continue;
//			isVisited.insert(next);
//			que.push(next);
//			double isSameDirection = normals->points[i].normal_x*normals->points[next].normal_x + normals->points[i].normal_y*normals->points[next].normal_y + normals->points[i].normal_z*normals->points[next].normal_z;
//			//cout << isSameDirection << endl;
//			if (isSameDirection < 0)
//			{
//				normals->points[next].normal_x *= -1;
//				normals->points[next].normal_y *= -1;
//				normals->points[next].normal_z *= -1;
//			}
//		}
//	}
//}
//
//
//void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int numberPoints)
//{
//
//	srand(time(0));
//	cloud->width = numberPoints;
//	cloud->height = 1;
//	cloud->points.resize(cloud->width * cloud->height);
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
//		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
//		cloud->points[i].z = rand() / 10 / (RAND_MAX + 1.0f);
//	}
//}
//
//boost::shared_ptr<pcl::PolygonMesh> myTriangles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	//=======================================================================================================向量计算及调整
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>n;  //法线估计对象
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储法线的向量
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals);  //估计法线存储位置
//
//	flip(normals, cloud);
//
//	vector<dt::Vector2<double>> points;
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		dt::Vector2<double> t(cloud->points[i].x, cloud->points[i].y);
//		points.push_back(t);
//
//	}
//
//	int K = 5;
//	boost::shared_ptr<pcl::PolygonMesh> tri(new pcl::PolygonMesh);
//	pcl::toPCLPointCloud2(*cloud, tri->cloud);
//	for (int i = 0; i < cloud->size(); i++)
//	{
//		std::vector<int> pointIdxNKNSearch(K);
//		std::vector<float> pointNKNSquaredDistance(K);
//		tree->nearestKSearch(cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
//
//		pcl::PointCloud<pcl::PointXYZ> tmp;
//		pcl::copyPointCloud(*cloud, pointIdxNKNSearch, tmp);
//
//		//=====================投影
//		// 定义模型系数对象，并填充对应的数据Create a set of planar coefficients with X=Y=0,Z=1
//		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//		coefficients->values.resize(4);
//		coefficients->values[0] = normals->points[i].normal_x;
//		coefficients->values[1] = normals->points[i].normal_y;
//		coefficients->values[2] = normals->points[i].normal_z;
//		coefficients->values[3] = 0;
//
//		pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影滤波对象
//		proj.setModelType(pcl::SACMODEL_PLANE);//设置对象对应的投影模型
//		proj.setInputCloud(cloud);//设置输入点云
//		proj.setModelCoefficients(coefficients);//设置模型对应的系数
//
//		proj.filter(tmp);//执行投影滤波存储结果cloud_projected
//
//		//========================旋转
//		Vector3f line(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
//		Eigen::Matrix4f R = tranformMatrix(line);
//		pcl::transformPointCloud(tmp, tmp, R);
//
//		//==========================重建
//		dt::Delaunay<double> triangulation;
//		const std::vector<dt::Triangle<double>> triangles = triangulation.triangulate(points);
//		for (auto triangle : triangles)
//		{
//
//			pcl::Vertices t;
//			for (int i = 0; i < points.size(); i++)
//			{
//				if (points[i] == *triangle.a || points[i] == *triangle.b || points[i] == *triangle.c)
//				{
//					//cout << i << " ";
//					t.vertices.push_back(i);
//				}
//
//			}
//			tri->polygons.push_back(t);
//			//cout << endl;
//		}
//	}
//
//	return tri;
//}
//
//
//int main()
//{
//
//	clock_t start = clock();  //时间起始 
//	/*待测试代码*/
//	
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	
//	//pcl::io::loadPCDFile("zhou_s_jiang.pcd", *cloud);
//	generatePointCloud(cloud, 100);
//
//
//	boost::shared_ptr<pcl::PolygonMesh> triangles = myTriangles(cloud);
//
//	clock_t end = clock(); //时间测试结束
//	cout << "time:"<<(end - start) << endl; //计算打印出运行时间,单位ms
//	pcl::io::savePLYFile("myTriangles.ply", *triangles);
//
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.addPolygonMesh(*triangles);
//	viewer.spin();
//
//	return 0;
//}
