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
//	gp3.setSearchRadius(5);  //设置连接点之间的最大距离（即为三角形的最大边长）
//	gp3.setMu(5);    //设置被样本点搜索其最近邻点的最远距离，为了适应点云密度的变化
//	gp3.setMaximumNearestNeighbors(30); //样本点可搜索的领域个数
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
//pcl::PolygonMesh possion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	//=======================================================================================================向量计算及调整
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象  
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线  
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals);
//	flip(normals, cloud);
//
//	//=======================================================================================================连接点云及其法向量
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//
//
//	//=======================================================================================================泊松重建
//	pcl::Poisson<pcl::PointNormal> pn;
//	pn.setSearchMethod(tree2);
//	pn.setInputCloud(cloud_with_normals);
//
//	////设置参数
//	//pn.setConfidence(true);//设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理  
//	//pn.setManifold(true);//设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加  
//	//pn.setOutputPolygons(true);//设置是否输出为多边形  
//	//pn.setIsoDivide(8);
//	//pn.setSamplesPerNode(3);//设置每个八叉树节点上最少采样点数目  
//
//	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
//	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
//	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
//	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
//	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
//	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
//	pn.setSamplesPerNode(9); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
//	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
//	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
//
//	pcl::PolygonMesh mesh;
//	pn.performReconstruction(mesh);
//
//	return mesh;
//}
//
//pcl::PolygonMesh removeTriangles(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ> &cloud,bool remove,double t)
//{
//	pcl::PointCloud<pcl::PointXYZ> triangles;
//	cout << "Total triangles:" << mesh.polygons.size() << endl;
//	cout << "Total points:" << cloud.size() << endl;
//	
//	pcl::fromPCLPointCloud2(mesh.cloud, triangles);
//	cout << triangles.points[0] << endl;
//	set<int> removed_index;
//	for (int i = 0; i < mesh.polygons.size(); i++)
//	{
//		float x = (triangles.points[mesh.polygons[i].vertices[0]].x + triangles.points[mesh.polygons[i].vertices[1]].x + triangles.points[mesh.polygons[i].vertices[2]].x) / 3;
//		float y = (triangles.points[mesh.polygons[i].vertices[0]].y + triangles.points[mesh.polygons[i].vertices[1]].y + triangles.points[mesh.polygons[i].vertices[2]].y) / 3;
//		float z = (triangles.points[mesh.polygons[i].vertices[0]].z + triangles.points[mesh.polygons[i].vertices[1]].z + triangles.points[mesh.polygons[i].vertices[2]].z) / 3;
//		pcl::PointXYZ mid(x, y, z);
//		for (auto point : cloud.points)
//		{
//			double dist = pcl::geometry::squaredDistance(mid, point);
//			//double T = 0.0001*0.0001;
//			double T = t*t;
//			if (dist <= T)
//			{
//				//cout << dist << "," << T << endl;
//				removed_index.insert(i);
//				break;
//			}
//		}
//	}
//	cout << "removed:" << removed_index.size() << endl;
//
//	pcl::PolygonMesh newMesh;
//	newMesh.cloud = mesh.cloud;
//	for (int i = 0; i < mesh.polygons.size(); i++)
//	{
//		if (remove)
//		{
//			if (removed_index.count(i))
//				continue;
//			newMesh.polygons.push_back(mesh.polygons[i]);
//		}
//		else
//		{
//			if (!removed_index.count(i))
//				continue;
//			newMesh.polygons.push_back(mesh.polygons[i]);
//		}
//		
//	}
//	return newMesh;
//}
//
//int main()
//{
//	
//	pcl::PolygonMesh mesh;
//	pcl::io::loadPLYFile("tai_jiang.ply", mesh);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_jianrui(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::io::loadPCDFile("zhou_s_jiang.pcd", *cloud);
//	pcl::io::loadPCDFile("zhou_s_jiang_jianrui.pcd", *cloud_jianrui);
//	//pcl::io::loadPLYFile("part1_s_jiang.ply", mesh);
//	
//
//	//pcl::PolygonMesh pinghuan = possion(cloud);
//	//pinghuan = removeTriangles(pinghuan, *cloud_jianrui,true);
//	//cout << "平缓部分：" << pinghuan.polygons.size() << endl;
//
//	//pcl::PolygonMesh jianrui=greedy(cloud);
//	//jianrui = removeTriangles(jianrui, *cloud_jianrui, false);
//	//cout << "尖锐部分：" << jianrui.polygons.size() << endl;
//
//
//	//mesh = removeTriangles(mesh, *cloud, false,0.001);
//	mesh= removeTriangles(mesh, *cloud_jianrui, false,0.002);//泊松用0.001
//	pcl::io::savePLYFile("zhou_s_tanlan_del.ply", mesh);
//
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPolygonMesh(mesh, "my");
//	viewer->spin();
//
//	return 0;
//}