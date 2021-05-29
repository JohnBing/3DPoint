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
//#include<unordered_map>
//#include<queue>
//#include<pcl/conversions.h>
//#include<pcl/common/geometry.h>
//using namespace std;
//
////合并两个PCLPointCloud2
//pcl::PCLPointCloud2 addPointCloud2(pcl::PCLPointCloud2 &cloud1, pcl::PCLPointCloud2 &cloud2)
//{
//	//point_step表示一个数据点占了几个数据段，row_step表示所有的数据段
//	cout <<"cloud1 point_step and row step:"<< cloud1.point_step << " " << cloud1.row_step<<endl;
//	cout << "cloud2 point_step and row step:" << cloud2.point_step << " " << cloud2.row_step << endl;
//
//	int step1 = cloud1.point_step;
//	int step2 = cloud2.point_step;
//
//	pcl::PCLPointCloud2 cloud = cloud1;
//
//	cloud.width += cloud2.width;
//	cloud.row_step = cloud.point_step*cloud.width;
//
//	
//	//将二进制cloud2的数据段有选择的添加到cloud1
//	for (int i = 0; i < cloud2.data.size(); i+=step2)
//	{
//		for (int j = i; j < i+step1; j++)
//		{
//			cloud.data.push_back(cloud2.data[j]);
//		}
//		
//	}
//
//	return cloud;
//}
//
////pcl::PCLPointCloud2 addPointCloud2(pcl::PCLPointCloud2 cloud1, pcl::PCLPointCloud2 cloud2,)
////{
////	pcl::PCLPointCloud2 cloud;
////	pcl::PointCloud<pcl::PointXYZ> c1;
////	pcl::PointCloud<pcl::PointXYZ> c2;
////	pcl::fromPCLPointCloud2<pcl::PointXYZ>(cloud1, c1);
////	pcl::fromPCLPointCloud2<pcl::PointXYZ>(cloud1, c2);
////	pcl::PointCloud<pcl::PointXYZ> c=c1 + c2;
////	pcl::toPCLPointCloud2(c, cloud);
////	return cloud;
////}
//
////合并两个mesh
//void addMesh(pcl::PolygonMesh &mesh1, pcl::PolygonMesh &mesh2)
//{
//	for (size_t i = 0; i < mesh1.cloud.fields.size(); ++i)
//	{
//		cout << "  fields[" << i << "]: ";
//		cout << std::endl;
//		cout << "    " << mesh1.cloud.fields[i] << std::endl;
//	}
//
//	for (size_t i = 0; i < mesh2.cloud.fields.size(); ++i)
//	{
//		cout << "  fields[" << i << "]: ";
//		cout << std::endl;
//		cout << "    " << mesh2.cloud.fields[i] << std::endl;
//	}
//
//	pcl::PolygonMesh mesh = mesh1;
//
//
//	int num1 = mesh1.cloud.width;
//	int num2 = mesh2.cloud.width;
//	int offset = num1;
//	cout <<"befor add:"<< num1 << " " << num2 << endl;
//
//	mesh.cloud = addPointCloud2(mesh1.cloud, mesh2.cloud);
//	
//	cout <<"after add:"<< mesh.cloud.width<<endl;
//
//	//将mesh2中的三角形添加到mesh1中
//	for (int i = 0; i < mesh2.polygons.size(); i++)
//	{
//		unsigned int i1=mesh2.polygons[i].vertices[0] + offset;
//		unsigned int i2=mesh2.polygons[i].vertices[1] + offset;
//		unsigned int i3=mesh2.polygons[i].vertices[2] + offset;
//
//		::pcl::Vertices  vertices;
//		vertices.vertices = { i1,i2,i3 };
//		mesh.polygons.push_back(vertices);
//	}
//	//cout << "===============" << endl << mesh << endl;
//	pcl::io::savePLYFile("addTest.ply", mesh);
//}
//
////检查只出现一次的边，及曲面模型的边界
//void checkEdge(pcl::PolygonMesh &mesh)
//{
//	pcl::PointCloud<pcl::PointXYZ> triangles;
//	pcl::PointCloud<pcl::PointXYZ> edgePoints;
//	cout << "Total triangles:" << mesh.polygons.size() << endl;
//
//	pcl::fromPCLPointCloud2(mesh.cloud, triangles);
//
//	map<pair<int, int>, int> edges;
//
//	for (int i = 0; i < mesh.polygons.size(); i++)
//	{
//		pair<int, int> edge1(mesh.polygons[i].vertices[0], mesh.polygons[i].vertices[1]);
//		pair<int, int> edge1_z(mesh.polygons[i].vertices[1], mesh.polygons[i].vertices[0]);
//
//		pair<int, int> edge2(mesh.polygons[i].vertices[0], mesh.polygons[i].vertices[2]);
//		pair<int, int> edge2_z(mesh.polygons[i].vertices[2], mesh.polygons[i].vertices[0]);
//
//		pair<int, int> edge3(mesh.polygons[i].vertices[1], mesh.polygons[i].vertices[2]);
//		pair<int, int> edge3_z(mesh.polygons[i].vertices[2], mesh.polygons[i].vertices[1]);
//
//		edges[edge1]++;
//		edges[edge2]++;
//		edges[edge3]++;
//
//		edges[edge1_z]++;
//		edges[edge2_z]++;
//		edges[edge3_z]++;
//	}
//	cout << "edge size:" << edges.size() << endl;
//	for (auto p : edges)
//	{
//		cout << p.second << " ";
//		//只出现过一次的边,便记录其两个端点
//		if (p.second == 1)
//		{
//			pcl::PointXYZ point1(triangles.points[p.first.first].x, triangles.points[p.first.first].y, triangles.points[p.first.first].z);
//			pcl::PointXYZ point2(triangles.points[p.first.second].x, triangles.points[p.first.second].y, triangles.points[p.first.second].z);
//			edgePoints.points.push_back(point1);
//			edgePoints.points.push_back(point2);
//		}
//	}
//	edgePoints.width = edgePoints.points.size();
//	edgePoints.height = 1;
//	pcl::io::savePCDFile("edgePoints.pcd", edgePoints);
//}
//
//int main()
//{
//
//	pcl::PolygonMesh mesh1;
//	//pcl::io::loadPLYFile("meshtest1.ply", mesh1);
//	pcl::io::loadPLYFile("zhou_s_possion_del_del.ply", mesh1);
//
//	pcl::PolygonMesh mesh2;
//	//pcl::io::loadPLYFile("meshtest2.ply", mesh2);
//	pcl::io::loadPLYFile("zhou_s_jianrui_tanlan.ply", mesh2);
//	addMesh(mesh1, mesh2);
//
//	system("pause");
//	return 0;
//}