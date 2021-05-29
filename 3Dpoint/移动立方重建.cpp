//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/surface/marching_cubes_rbf.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <fstream>
//#include <iostream>
//#include <stdio.h>
//#include <string.h>
//#include <string>
//#include<unordered_set>
//#include<queue>
//using namespace std;
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
//int main(int argc, char** argv)
//{
//	//// 确定文件格式
//	//char tmpStr[100];
//	//strcpy(tmpStr, argv[1]);
//	//char* pext = strrchr(tmpStr, ‘.‘);
//	//std::string extply("ply");
//	//std::string extpcd("pcd");
//	//if (pext) {
//	//	*pext = ‘\0‘;
//	//	pext++;
//	//}
//	//std::string ext(pext);
//	////如果不支持文件格式，退出程序
//	//if (!((ext == extply) || (ext == extpcd))) {
//	//	std::cout << "文件格式不支持!" << std::endl;
//	//	std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
//	//	return(-1);
//	//}
//
//	////根据文件格式选择输入方式
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
//	//if (ext == extply) {
//	//	if (pcl::io::loadPLYFile(argv[1], *cloud) == -1) {
//	//		PCL_ERROR("Could not read ply file!\n");
//	//		return -1;
//	//	}
//	//}
//	//else {
//	//	if (C == -1) {
//	//		PCL_ERROR("Could not read pcd file!\n");
//	//		return -1;
//	//	}
//	//}
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("part1_s.pcd", *cloud);
//
//	// 估计法向量
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals); 
//
//	flip(normals, cloud);
//	//将点云和法线放到一起
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals
//
//	//创建搜索树
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//
//	//初始化MarchingCubes对象，并设置参数
//	pcl::MarchingCubes<pcl::PointNormal> *mc;
//	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
//
//	/*
//  if (hoppe_or_rbf == 0)
//	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
//  else
//  {
//	mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
//	(reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
//  }
//	*/
//
//	//创建多变形网格，用于存储结果
//	pcl::PolygonMesh mesh;
//
//	//设置MarchingCubes对象的参数
//	mc->setIsoLevel(0.0f);
//	mc->setGridResolution(50, 50, 50);
//	mc->setPercentageExtendGrid(0.0f);
//
//	mc->setInputCloud(cloud_with_normals);
//	mc->reconstruct(mesh);
//	pcl::io::savePLYFile("result.ply", mesh);
//
//	// 显示结果图
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0); //设置背景
//	viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
//	//viewer->addCoordinateSystem(1.0); //设置坐标系
//	//viewer->initCameraParameters();
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	return (0);
//}