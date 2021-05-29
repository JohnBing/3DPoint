//#define _CRT_SECURE_NO_WARNINGS
//#define _SCL_SECURE_NO_WARNINGS
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <Eigen/core>
//#include <pcl/visualization/cloud_viewer.h>
//#include <vector>
//#include <pcl/features/normal_3d.h>
//#include<pcl/visualization/pcl_plotter.h>
//#include<unordered_set>
//#include <stdio.h>
//#include <stdlib.h>
////���÷�������ʾʱ�����������н��
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//
//
//using namespace std;
//
//class myNE {
//public:
//	myNE() {};
//	~myNE() {};
//
//	void setKsearch(int k)
//	{
//		K = k;
//	}
//
//	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c)
//	{
//		cloud = c;
//		kdtree.setInputCloud(cloud);
//		int cld_sz = cloud->size();
//
//	}
//
//	pcl::PointCloud<pcl::Normal>::Ptr compute()
//	{
//		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//		size_t cld_sz = cloud->size();
//		normals->resize(cld_sz);
//
//		for (int i = 0; i < cloud->points.size(); i++)
//		{
//			auto searchPoint = cloud->points[i];
//			std::vector<int> pointIdxNKNSearch(K);
//			std::vector<float> pointNKNSquaredDistance(K);
//			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//			computePointNormals(i, pointIdxNKNSearch, normals);
//		}
//		flip(normals, cloud);
//		return normals;
//	}
//
//	void plotterCurvature(pcl::PointCloud<pcl::Normal>::Ptr normals)
//	{
//		std::vector<double> vec;
//		for (auto tmp : normals->points)
//		{
//			vec.push_back(tmp.curvature);
//		}
//
//		pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter();
//
//		plotter->addHistogramData(vec, 100); //30�Ƿ�����
//
//		plotter->plot();
//	}
//
//
//public:
//	//��������
//	void flip(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//	{
//		int K = 10;//���������10����,����Ҫ��һ����ֵ����
//		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//		kdtree.setInputCloud(cloud);
//		unordered_set<int> isVisited;
//		queue<int> que;
//		que.push(0);
//		isVisited.insert(0);
//		while (!que.empty())
//		{
//			int i = que.front();
//			//cout << i << endl;
//			que.pop();
//
//
//			auto searchPoint = cloud->points[i];
//			std::vector<int> pointIdxNKNSearch(K);
//			std::vector<float> pointNKNSquaredDistance(K);
//			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//			for (int next : pointIdxNKNSearch)
//			{
//				if (isVisited.count(next)) continue;
//				isVisited.insert(next);
//				que.push(next);
//				double isSameDirection = normals->points[i].normal_x*normals->points[next].normal_x + normals->points[i].normal_y*normals->points[next].normal_y + normals->points[i].normal_z*normals->points[next].normal_z;
//				//cout << isSameDirection << endl;
//				if (isSameDirection < 0)
//				{
//					normals->points[next].normal_x *= -1;
//					normals->points[next].normal_y *= -1;
//					normals->points[next].normal_z *= -1;
//				}
//			}
//		}
//
//
//
//	}
//	//���㷨��
//	void computePointNormals(int i, std::vector<int> indexs, pcl::PointCloud<pcl::Normal>::Ptr normals)
//	{
//		size_t cld_sz = indexs.size();
//		//�������ĵ�����
//		double center_x = 0, center_y = 0, center_z = 0;
//		for (int i = 0; i < cld_sz; i++) {
//			center_x += cloud->points[indexs[i]].x;
//			center_y += cloud->points[indexs[i]].y;
//			center_z += cloud->points[indexs[i]].z;
//		}
//		center_x /= cld_sz;
//		center_y /= cld_sz;
//		center_z /= cld_sz;
//		//����Э�������
//		double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
//		for (int i = 0; i < cld_sz; i++) {
//			xx += (cloud->points[indexs[i]].x - center_x) * (cloud->points[indexs[i]].x - center_x);
//			xy += (cloud->points[indexs[i]].x - center_x) * (cloud->points[indexs[i]].y - center_y);
//			xz += (cloud->points[indexs[i]].x - center_x) * (cloud->points[indexs[i]].z - center_z);
//			yy += (cloud->points[indexs[i]].y - center_y) * (cloud->points[indexs[i]].y - center_y);
//			yz += (cloud->points[indexs[i]].y - center_y) * (cloud->points[indexs[i]].z - center_z);
//			zz += (cloud->points[indexs[i]].z - center_z) * (cloud->points[indexs[i]].z - center_z);
//		}
//		//��СΪ3*3��Э�������
//		Eigen::Matrix3f covMat(3, 3);
//		covMat(0, 0) = xx / cld_sz;
//		covMat(0, 1) = covMat(1, 0) = xy / cld_sz;
//		covMat(0, 2) = covMat(2, 0) = xz / cld_sz;
//		covMat(1, 1) = yy / cld_sz;
//		covMat(1, 2) = covMat(2, 1) = yz / cld_sz;
//		covMat(2, 2) = zz / cld_sz;
//
//		//������ֵ����������
//		Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
//		Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
//		Eigen::Matrix3f vec = es.pseudoEigenvectors();
//
//		//�ҵ���С����ֵt1
//		double t1 = val(0, 0);
//		int ii = 0;
//		if (t1 > val(1, 1)) {
//			ii = 1;
//			t1 = val(1, 1);
//
//		}
//		if (t1 > val(2, 2)) {
//			ii = 2;
//			t1 = val(2, 2);
//
//		}
//
//		//��С����ֵ��Ӧ����������v_n
//		Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
//		//����������λ��
//		v /= v.norm();
//
//		normals->points[i].normal_x = v(0);
//		normals->points[i].normal_y = v(1);
//		normals->points[i].normal_z = v(2);
//		normals->points[i].curvature = t1 / (val(0, 0) + val(1, 1) + val(2, 2));
//	}
//
//private:
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	int K;
//};
//
//
////���ּ����ƽ������
//void distinguish(pcl::PointCloud<pcl::Normal>::Ptr result)
//{
//	vector<int> pinghuan;
//	vector<int> jianrui;
//	for (int i = 0; i < result->size(); i++)
//	{
//		if (result->points[i].curvature < 0.002)
//		{
//			pinghuan.push_back(i);
//		}
//		else
//		{
//			jianrui.push_back(i);
//		}
//	}
//
//}
//
//
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 125, 125, 125); 
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud,color, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	//viewer->initCameraParameters();
//	return (viewer);
//}
//
//
//int main(int argc,char *argv[])
//{
//	double T;
//	cout << "������ָ���ֵT��һ��Ϊ0.005��" << endl;
//	cin >> T;
//		
//
//	pcl::PCDReader reader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	//reader.read("zhou_s.pcd", *cloud);
//	reader.read("data/zhou_s.pcd", *cloud);
//	cout << "cloud_size:" << cloud->size()<<endl;
//	
//	myNE ne;
//	ne.setKsearch(100);
//	ne.setInputCloud(cloud);
//	pcl::PointCloud<pcl::Normal>::Ptr result=ne.compute();
//	ne.plotterCurvature(result);
//	
//	//�����������ֳ�ƽ����ͷ�ƽ����
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pinghuan(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_pinghuan_normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_jianrui(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_jianrui_normals(new pcl::PointCloud<pcl::Normal>);
//	double sum = 0;
//	for (int i = 0; i < result->size(); i++)
//	{
//		sum += result->points[i].curvature;
//		if (result->points[i].curvature < T)
//		{
//			cloud_pinghuan->points.push_back(cloud->points[i]);
//			cloud_pinghuan_normals->points.push_back(result->points[i]);
//		}
//		else
//		{
//			cloud_jianrui->points.push_back(cloud->points[i]);
//			cloud_jianrui_normals->points.push_back(result->points[i]);
//		}
//			
//	}
//	cout << sum / result->size()<<endl;
//	ne.flip(cloud_pinghuan_normals, cloud_pinghuan);
//
//	cloud_pinghuan->width = cloud_pinghuan->points.size();
//	cloud_pinghuan->height = 1;
//	cloud_jianrui->width = cloud_jianrui->points.size();
//	cloud_jianrui->height = 1;
//	cout << *cloud_pinghuan << endl;
//	cout << *cloud_jianrui << endl;
//
//	//ȫ����ת
//	for (int i = 0; i < result->size(); i++)
//	{
//		result->points[i].normal_x *= -1;
//		result->points[i].normal_y *= -1;
//		result->points[i].normal_z *= -1;
//	}
//
//	pcl::PCDWriter writer;
//	writer.write("data/zhou_s_pinghuan.pcd", *cloud_pinghuan);
//	writer.write("data/zhou_s_jianrui.pcd", *cloud_jianrui);
//
//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//	
//	
//	//viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, result,50);
//	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5);
//
//    pcl::visualization::PCLVisualizer::Ptr ptr = simpleVis(cloud_jianrui);
//    ptr->spin();
//
//
//	viewer.spin();
//	return 0;
//}