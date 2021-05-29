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
//pcl::PolygonMesh Possion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
//	pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
//	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
//	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
//	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
//	pn.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
//	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
//	pn.setSamplesPerNode(2); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑//9
//	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
//	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
//
//	pcl::PolygonMesh mesh;
//	pn.performReconstruction(mesh);
//
//	return mesh;
//}
//
//int main()
//{
//	clock_t start = clock();  //时间起始 
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("data/4.3voxel_jiang.pcd", *cloud);
//
//	pcl::PolygonMesh mesh = Possion(cloud);
//
//	clock_t end = clock(); //时间测试结束
//	cout << "time:" << (end - start) << endl; //计算打印出运行时间,单位ms
//
//	pcl::io::savePLYFile("data/4.3voxel_jiang.ply", mesh);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPolygonMesh(mesh, "my");
//	viewer->spin();
//	return 0;
//}