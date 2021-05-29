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
//pcl::PolygonMesh Possion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
//	pn.setConfidence(true); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
//	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
//	pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
//	pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
//	pn.setManifold(true); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
//	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
//	pn.setSamplesPerNode(2); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��//9
//	pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
//	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
//
//	pcl::PolygonMesh mesh;
//	pn.performReconstruction(mesh);
//
//	return mesh;
//}
//
//int main()
//{
//	clock_t start = clock();  //ʱ����ʼ 
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("data/4.3voxel_jiang.pcd", *cloud);
//
//	pcl::PolygonMesh mesh = Possion(cloud);
//
//	clock_t end = clock(); //ʱ����Խ���
//	cout << "time:" << (end - start) << endl; //�����ӡ������ʱ��,��λms
//
//	pcl::io::savePLYFile("data/4.3voxel_jiang.ply", mesh);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPolygonMesh(mesh, "my");
//	viewer->spin();
//	return 0;
//}