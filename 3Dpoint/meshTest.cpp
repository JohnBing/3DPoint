//#include <iostream>
//#include <ctime>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include<pcl/surface/gp3.h>  //贪婪投影三角化算法
//#include<pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//
//
//int
//main(int argc, char** argv)
//{
//	srand(time(0));
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	cloud->width = 4;
//	cloud->height = 1;
//	cloud->points.resize(cloud->width * cloud->height);
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
//		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
//		cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
//	}
//	pcl::PolygonMesh triangles;
//	pcl::toPCLPointCloud2(*cloud, triangles.cloud);
//
//	pcl::Vertices vertices;
//	vertices.vertices = { 0,1,2 };
//	triangles.polygons.push_back(vertices);
//	vertices.vertices = { 3,1,2 };
//	triangles.polygons.push_back(vertices);
//	//vertices.vertices = { 0,3,2 };
//	//triangles.polygons.push_back(vertices);
//	//vertices.vertices = { 0,5,2 };
//	//triangles.polygons.push_back(vertices);
//	//vertices.vertices = { 0,4,2 };
//	//triangles.polygons.push_back(vertices);
//
//	triangles.header = triangles.cloud.header;
//	cout << triangles;
//	pcl::io::savePLYFile("meshtest2.ply", triangles);
//
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.addPolygonMesh(triangles);
//	viewer.spin();
//
//}