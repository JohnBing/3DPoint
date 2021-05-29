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
//	int K = 10;//���������10����,����Ҫ��һ����ֵ����
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
//	//=======================================================================================================�������㼰����
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>n;  //���߹��ƶ���
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //�洢���ߵ�����
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);//
//	n.compute(*normals);  //���Ʒ��ߴ洢λ��
//
//	flip(normals, cloud);
//
//	//=======================================================================================================���ӵ��Ƽ��䷨����
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals); //����������
//
//	//=======================================================================================================���ǻ�
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal>gp3;  //�������ǻ�����
//	pcl::PolygonMesh triangles; //�����������ǻ�������ģ��
//
//	gp3.setSearchRadius(20);  //�������ӵ�֮��������루��Ϊ�����ε����߳���20
//	gp3.setMu(10);    //���ñ�����������������ڵ����Զ���룬Ϊ����Ӧ�����ܶȵı仯10
//	gp3.setMaximumNearestNeighbors(100); //��������������������
//	gp3.setMinimumAngle(M_PI / 4);  //�������ǻ���õ����������ڽ���С�Ƕ�Ϊ10��
//	gp3.setMaximumAngle(2 * M_PI / 3); //�������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��
//	gp3.setMaximumSurfaceAngle(M_PI / 4);  //ĳ�㷨��������ƫ�������㷨�ߵ����Ƕ�45��
//	gp3.setNormalConsistency(true); //���øò�����֤���߳���һ��
//	gp3.setInputCloud(cloud_with_normals);  //�����������Ϊ�������
//	gp3.setSearchMethod(tree2); //����������ʽ
//
//	gp3.reconstruct(triangles); //�ؽ���ȡ���ǻ�
//
//
//
//	//���Ӷ�����Ϣ
//	vector<int>parts = gp3.getPartIDs();
//	vector<int>states = gp3.getPointStates();
//	return triangles;
//	
//}
//
//int main()
//{
//
//	clock_t start = clock();  //ʱ����ʼ 
//
//	
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("tai_s1.pcd", *cloud);
//	pcl::PolygonMesh triangles = greedy(cloud);
//
//
//	clock_t end = clock(); //ʱ����Խ���
//
//	cout << "time:" << (end - start) << endl; //�����ӡ������ʱ��,��λms
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
