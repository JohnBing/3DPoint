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
////调用法向量显示时出错，加这两行解决
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//
//
//using namespace std;
//
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
//	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
//{
//	//创建3D窗口并添加点云其包括法线  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0); // red
//	viewer->addPointCloud<pcl::PointXYZ>(cloud,single_color, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1);
//
//	return (viewer);
//}
//
//void flip(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int k)
//{
//	int K = k;//搜索最近的10个点,最重要的一个阈值参数
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
//
//
//
//}
//
//int main(int argc, char *argv[])
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
//	std::istringstream iss;
//	string line;
//	int n = 0;
//	if (argc < 3) return -1;
//	string path = argv[1];
//	int factor = stoi(argv[2]);
//	cout << path << endl;
//	ifstream iff(path);
//	while (getline(iff, line))
//	{
//		n++;
//
//		// Trims line buffer
//		line.erase(line.find_last_not_of(" ") + 1);
//		line.erase(0, line.find_first_not_of(" "));
//		
//		// Skips comment or empty line...
//		if (line.length() == 0 || line[0] == '#')
//		{
//			continue;
//		}
//		// ...or reads position...
//		else {
//			iss.clear();
//			iss.str(line);
//			float x, y, z,nx,ny,nz;
//			if (iss >> x >> y >> z>>nx>>ny>>nz)
//			{
//				cloud->points.push_back(pcl::PointXYZ(x/factor, y/factor, z/factor));
//				normal->points.push_back(pcl::Normal(nx, ny, nz));
//			}
//
//		}
//	}
//	cloud->width = n; cloud->height = 1;
//	normal->width = n; normal->height = 1;
//	cout << n<<endl;
//	clock_t start = clock();  //时间起始 
//
//	if (argc == 4)
//	{
//		int k = stoi(argv[3]);
//		flip(normal, cloud, k);
//	}
//	
//	clock_t end = clock(); //时间测试结束
//
//	cout << "time:" << (end - start) << endl; //计算打印出运行时间,单位ms
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(cloud, normal);
//	viewer->spin();
//	return 0;
//}