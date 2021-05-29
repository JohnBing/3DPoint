//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std;
//int main() {
//	//================读取点云数据=====================
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDReader reader;
//	pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
//	//reader.read<pcl::PointXYZ>("l.pcd", *cloud);
//	//================创建拟合模型=====================
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers用来存储直线上点的索引
//	pcl::SACSegmentation<pcl::PointXYZ> seg;//创建一个分割器
//	seg.setOptimizeCoefficients(true);      //可选择配置，设置模型系数需要优化
//	seg.setModelType(pcl::SACMODEL_LINE);   //设置目标几何形状
//	seg.setMethodType(pcl::SAC_RANSAC);     //拟合方法：随机采样法
//	seg.setDistanceThreshold(0.05);         //设置误差容忍范围，也就是阈值
//	seg.setMaxIterations(500);              //最大迭代次数，不设置的话默认迭代50次
//	seg.setInputCloud(cloud);               //输入点云
//	seg.segment(*inliers, *coefficients);   //拟合点云
//	//====================模型系数=====================
//	cout << "拟合直线的模型系数为：" << endl;
//	cout << "a：" << coefficients->values[0] << endl;
//	cout << "b：" << coefficients->values[1] << endl;
//	cout << "c：" << coefficients->values[2] << endl;
//	cout << "d：" << coefficients->values[3] << endl;
//	cout << "e：" << coefficients->values[4] << endl;
//	cout << "f：" << coefficients->values[5] << endl;
//
//
//	//==================提取拟合的直线=================
//	/*直线提取方法1
//	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i < inliers->indices.size(); ++i) {
//		c_plane->points.push_back(cloud->points.at(inliers->indices[i]));
//	}
//	*/
//
//	//直线提取
//	pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
//	extract.setInputCloud(cloud);    //设置输入点云
//	extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
//	extract.setNegative(false);      //false提取内点, true提取外点
//	extract.filter(*line);        //提取输出存储到c_plane2
//	pcl::io::savePCDFile("l1.pcd", *line);
//	// 点云可视化
//	pcl::visualization::PCLVisualizer viewer;
//	//viewer.addPointCloud(cloud, "cloud");  // 加载比对点云
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(line, 255, 0, 0);
//	viewer.addPointCloud(line, line_color, "line");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
//	viewer.spin();
//
//	return 0;
//}
//
