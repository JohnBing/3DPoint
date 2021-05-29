//#include<iostream>
//#include<pcl/point_types.h>
//#include<pcl/io/pcd_io.h>
//#include<pcl/io/ply_io.h>
//#include<pcl/kdtree/kdtree_flann.h>
//#include<pcl/features/normal_3d.h>
//#include<pcl/surface/gp3.h>
//#include<pcl/visualization/pcl_visualizer.h>
//#include<boost/math/special_functions/round.hpp>
//#include  <time.h> 
//#include  <unordered_map> 
//#include  <unordered_set> 
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
//void testResult(vector<int> &parts,vector<int> &states,pcl::PolygonMesh &triangles)
//{
//	cout << "part id size:" << parts.size() << endl;
//	cout << "state size:" << states.size() << endl;
//
//	cout << "part========================" << endl;
//	unordered_map<int, int> sum;
//	for (auto num : parts)
//	{
//		sum[num]++;
//	}
//	for (auto tmp : sum)
//	{
//		cout << tmp.first << " times" << tmp.second << endl;
//	}
//
//	sum.clear();
//	cout << "state=========================" << endl;
//	for (auto num : states)
//	{
//		sum[num]++;
//	}
//	for (auto tmp : sum)
//	{
//		cout << tmp.first << " times" << tmp.second << endl;
//	}
//
//	cout << triangles.header << endl;
//	cout << "========================";
//	for (auto polygon : triangles.polygons)
//	{
//		cout << polygon << endl;
//	}
//}
//
//pcl::PolygonMesh greedy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	//=======================================================================================================向量计算及调整
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>n;  //法线估计对象
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储法线的向量
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);//
//	n.compute(*normals);  //估计法线存储位置
//
//	flip(normals, cloud);
//
//	//=======================================================================================================连接点云及其法向量
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals); //点云搜索树
//
//	//=======================================================================================================三角化
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal>gp3;  //定义三角化对象
//	pcl::PolygonMesh triangles; //定义最终三角化的网络模型
//
//	gp3.setSearchRadius(20);  //设置连接点之间的最大距离（即为三角形的最大边长）20
//	gp3.setMu(10);    //设置被样本点搜索其最近邻点的最远距离，为了适应点云密度的变化10
//	gp3.setMaximumNearestNeighbors(100); //样本点可搜索的领域个数
//	gp3.setMinimumAngle(M_PI / 4);  //设置三角化后得到的三角形内角最小角度为10°
//	gp3.setMaximumAngle(2 * M_PI / 3); //设置三角化后得到的三角形内角的最大角度为120°
//	gp3.setMaximumSurfaceAngle(M_PI / 4);  //某点法向量方向偏离样本点法线的最大角度45°
//	gp3.setNormalConsistency(true); //设置该参数保证法线朝向一致
//	gp3.setInputCloud(cloud_with_normals);  //设置输入点云为有向点云
//	gp3.setSearchMethod(tree2); //设置搜索方式
//
//	gp3.reconstruct(triangles); //重建提取三角化
//
//
//
//	//附加顶点信息
//	vector<int>parts = gp3.getPartIDs();
//	vector<int>states = gp3.getPointStates();
//	return triangles;
//	
//}
//
//int main()
//{
//
//	clock_t start = clock();  //时间起始 
//
//	
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("tai_s1.pcd", *cloud);
//	pcl::PolygonMesh triangles = greedy(cloud);
//
//
//	clock_t end = clock(); //时间测试结束
//
//	cout << "time:" << (end - start) << endl; //计算打印出运行时间,单位ms
//	
//
//	pcl::io::savePLYFile("tai_s1.ply", triangles);
//
//	pcl::visualization::PCLVisualizer viewer("viewer");
//	viewer.setBackgroundColor(255, 255, 255);
//	viewer.addPolygonMesh(triangles);
//	viewer.spin();
//
//	return 0;
//}
