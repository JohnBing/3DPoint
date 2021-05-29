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
//	gp3.setSearchRadius(5);  //�������ӵ�֮��������루��Ϊ�����ε����߳���
//	gp3.setMu(5);    //���ñ�����������������ڵ����Զ���룬Ϊ����Ӧ�����ܶȵı仯
//	gp3.setMaximumNearestNeighbors(30); //��������������������
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
//pcl::PolygonMesh possion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	//=======================================================================================================�������㼰����
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���  
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���  
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals);
//	flip(normals, cloud);
//
//	//=======================================================================================================���ӵ��Ƽ��䷨����
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//
//
//	//=======================================================================================================�����ؽ�
//	pcl::Poisson<pcl::PointNormal> pn;
//	pn.setSearchMethod(tree2);
//	pn.setInputCloud(cloud_with_normals);
//
//	////���ò���
//	//pn.setConfidence(true);//�������ű�־��Ϊtrueʱ��ʹ�÷�������������Ϊ���Ŷ���Ϣ��false����Ҫ�Է��߽��й�һ������  
//	//pn.setManifold(true);//�������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����  
//	//pn.setOutputPolygons(true);//�����Ƿ����Ϊ�����  
//	//pn.setIsoDivide(8);
//	//pn.setSamplesPerNode(3);//����ÿ���˲����ڵ������ٲ�������Ŀ  
//
//	pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
//	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
//	pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
//	pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
//	pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
//	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
//	pn.setSamplesPerNode(9); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
//	pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
//	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
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
//	//cout << "ƽ�����֣�" << pinghuan.polygons.size() << endl;
//
//	//pcl::PolygonMesh jianrui=greedy(cloud);
//	//jianrui = removeTriangles(jianrui, *cloud_jianrui, false);
//	//cout << "���񲿷֣�" << jianrui.polygons.size() << endl;
//
//
//	//mesh = removeTriangles(mesh, *cloud, false,0.001);
//	mesh= removeTriangles(mesh, *cloud_jianrui, false,0.002);//������0.001
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