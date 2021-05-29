//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <Eigen/core>
//#include <pcl/visualization/cloud_viewer.h>
//#include <vector>
//#include <pcl/features/normal_3d.h>
//#include<pcl/visualization/pcl_plotter.h>
//#include<unordered_set>
//#include<time.h>
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
//		clock_t start = clock();
//		flip(normals, cloud);//��K=10��������
//		clock_t end = clock();
//		cout << "�������ʱ��:"<<end - start << endl;
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
//		plotter->addHistogramData(vec, 30); //30�Ƿ�����
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
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
//	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
//{
//	//����3D���ڲ���ӵ������������  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//
//	viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1);
//
//	return (viewer);
//}
//
//int main()
//{
//	pcl::PCDReader reader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	reader.read("sphere.pcd", *cloud);
//	cout << "cloud_size:" << cloud->size() << endl;
//
//	clock_t start = clock();  //ʱ����ʼ 
//
//	myNE ne;
//	ne.setKsearch(20);
//	ne.setInputCloud(cloud);
//	pcl::PointCloud<pcl::Normal>::Ptr result = ne.compute();
//
//	clock_t end = clock(); //ʱ����Խ���
//	cout << "kdtree������ʱ��" << end - start << endl; //�����ӡ������ʱ��,��λms
//
//	//for (int i = 0; i < result->size(); i++)
//	//{
//	//	cout << result->points[i].normal_x << " " << result->points[i].normal_y << " " << result->points[i].normal_z << endl;
//	//}
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(cloud, result);
//	viewer->spin();
//	return 0;
//}