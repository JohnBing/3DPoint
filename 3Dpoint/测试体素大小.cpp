//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <time.h>
//#include <string>
//#include<fstream>
//
//using namespace std;
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 125, 125, 125);
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, color, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	//viewer->initCameraParameters();
//	return (viewer);
//}
//
//
//int
//main(int argc, char** argv)
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDReader reader;
//	
//	ofstream par("paramater.txt");
//	ofstream res("result.txt");
//	string path;
//	float minleafSize = 0.002;
//	float maxleafSize = 0.02;
//	int steps = 50;
//	cout << "minleafSize,maxleafSize,steps:" << endl;
//	cin >> minleafSize >> maxleafSize >> steps;
//	//reader.read ("part1_s.pcd", *cloud);
//	reader.read("data/zhou_s.pcd", *cloud);
//
//	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
//	float tmp = (maxleafSize - minleafSize) / (steps - 1);
//	for (float i = 0; i < steps; i++)
//	{
//		clock_t start = clock();
//		float leafSize = minleafSize + i * tmp;
//		cout << leafSize << endl;
//		pcl::VoxelGrid<pcl::PointXYZ> sor;
//		sor.setInputCloud(cloud);
//		sor.setLeafSize(leafSize, leafSize, leafSize);
//		sor.filter(*cloud_filtered);
//
//		clock_t end = clock();
//		cout << "time" << end - start << endl;
//		std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//			<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
//		par << leafSize << ",";
//		res << cloud_filtered->width * cloud_filtered->height << ",";
//	}
//	par.close();
//	res.close();
//	//pcl::PCDWriter writer;
//	//writer.write ("data/zhou_s_VG.pcd", *cloud_filtered);
//
//	//pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud_filtered);
//	//viewer->spin();
//	system("pause");
//	return (0);
//}
