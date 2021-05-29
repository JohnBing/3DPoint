//#include <iostream>
//#include <vector>
//#include <iterator>
//#include <algorithm>
//#include <array>
//#include <random>
//#include <chrono>
//#include<pcl/visualization/pcl_visualizer.h>
//#include<pcl/surface/gp3.h>  //贪婪投影三角化算法
//#include "Delaunay.h"
//#include<unordered_map>
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
//int main(int argc, char * argv[])
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<dt::Vector2<double>> points;
//
//	generatePointCloud(cloud,10);
//	
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		dt::Vector2<double> tmp(cloud->points[i].x, cloud->points[i].y);
//		points.push_back(tmp);
//	}
//
//
//	dt::Delaunay<double> triangulation;
//	const auto start = std::chrono::high_resolution_clock::now();
//	const std::vector<dt::Triangle<double>> triangles = triangulation.triangulate(points);
//	const auto end = std::chrono::high_resolution_clock::now();
//	const std::chrono::duration<double> diff = end - start;
//
//	std::cout << triangles.size() << " triangles generated in " << diff.count()
//		<< "s\n";
//
//
//	//==============================================================生成PolygonMesh
//	pcl::PolygonMesh tri;
//	pcl::toPCLPointCloud2(*cloud, tri.cloud);
//	for (auto triangle : triangles)
//	{
//
//		pcl::Vertices tmp;
//		for (int i=0;i<points.size();i++)
//		{
//			if (points[i] == *triangle.a || points[i] == *triangle.b || points[i] == *triangle.c)
//			{
//				//cout << i<<" ";
//				tmp.vertices.push_back(i);
//			}
//			
//		}
//		//cout << endl;
//		tri.polygons.push_back(tmp);
//
//	}
//
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.addPolygonMesh(tri);
//	viewer.spin();
//
//	system("pause");
//	return 0;
//}
//
