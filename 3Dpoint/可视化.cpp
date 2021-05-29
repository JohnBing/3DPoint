//#include <iostream>
//#include <boost/thread/thread.hpp>
//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>				// 法线估计类头文件声明
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//
//// --------------
//// -----Help-----
//// --------------
//void
//printUsage(const char* progName)
//{
//	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
//		<< "Options:\n"
//		<< "-------------------------------------------\n"
//		<< "-h           this help\n"
//		<< "-s           Simple visualisation example\n"
//		<< "-r           RGB colour visualisation example\n"
//		<< "-c           Custom colour visualisation example\n"
//		<< "-n           Normals visualisation example\n"
//		<< "-a           Shapes visualisation example\n"
//		<< "-v           Viewports example\n"
//		<< "\n\n";
//}
//
///*可视化单个点云*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	/*创建视图对象并定义标题栏“3D Viewer”*/
//	//boost::shared_ptr 智能共享指针，保证该指针在整个程序全局使用
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);//视窗背景色设置
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");//添加点云到视窗对象
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");//渲染属性
//	viewer->addCoordinateSystem(1.0);//显示点云的尺寸设置
//	viewer->initCameraParameters();//照相机参数设置
//	return (viewer);
//}
//
///*可视化点云颜色特征*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//渲染属性
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
///*可视化点云自定义颜色特征*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);//显示绿色
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//渲染属性
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
//
///*可视化点云法线和其他特征*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
//	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");//显示点云法线
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
///*绘制普通形状*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//
//	//------------------------------------
//	//-----Add shapes at cloud points-----
//	//------------------------------------
//	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0], cloud->points[cloud->size() - 1], "line");//点之间连线
//	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");//
//
//	//---------------------------------------
//	//-----Add shapes at other locations-----
//	//---------------------------------------
//	pcl::ModelCoefficients coeffs;
//	coeffs.values.push_back(0.0);
//	coeffs.values.push_back(0.0);
//	coeffs.values.push_back(1.0);
//	coeffs.values.push_back(0.0);
//	viewer->addPlane(coeffs, "plane");
//	coeffs.values.clear();
//	coeffs.values.push_back(0.3);
//	coeffs.values.push_back(0.3);
//	coeffs.values.push_back(0.0);
//	coeffs.values.push_back(0.0);
//	coeffs.values.push_back(1.0);
//	coeffs.values.push_back(0.0);
//	coeffs.values.push_back(5.0);
//	viewer->addCone(coeffs, "cone");
//
//	return (viewer);
//}
//
///*多视口显示点云法线*/
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
//	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->initCameraParameters();
//
//	int v1(0);
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->addText("Radius: 0.05", 10, 10, "v1 text", v1);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
//
//	int v2(0);
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
//	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
//
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//	viewer->addCoordinateSystem(1.0);
//
//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);
//
//	return (viewer);
//}
//
//int main(int argc, char** argv)
//{
//	// --------------------------------------
//	// -----———解析命令行参数————-----
//	// --------------------------------------
//	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
//	{
//		printUsage(argv[0]);
//		return 0;
//	}
//	bool simple(false), rgb(false), custom_c(false), normals(false),
//		shapes(false), viewports(false), interaction_customization(false);
//	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
//	{
//		simple = true;
//		std::cout << "Simple visualisation example\n";
//	}
//	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
//	{
//		custom_c = true;
//		std::cout << "Custom colour visualisation example\n";
//	}
//	else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
//	{
//		rgb = true;
//		std::cout << "RGB colour visualisation example\n";
//	}
//	else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
//	{
//		normals = true;
//		std::cout << "Normals visualisation example\n";
//	}
//	else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
//	{
//		shapes = true;
//		std::cout << "Shapes visualisation example\n";
//	}
//	else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
//	{
//		viewports = true;
//		std::cout << "Viewports example\n";
//	}
//	else
//	{
//		printUsage(argv[0]);
//		return 0;
//	}
//
//	// ------------------------------------
//	// -----———创造例子点云————-----
//	// ------------------------------------
//	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
//	std::cout << "Generating example point clouds.\n\n";
//	// 做一个椭圆沿着z轴方向，XYZRGB 点云的颜色会逐渐从红到绿到蓝
//	uint8_t r(255), g(15), b(15);
//	for (float z(-1.0); z <= 1.0; z += 0.05)
//	{
//		for (float angle(0.0); angle <= 360.0; angle += 5.0)
//		{
//			pcl::PointXYZ basic_point;
//			basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
//			basic_point.y = sinf(pcl::deg2rad(angle));
//			basic_point.z = z;
//			basic_cloud_ptr->points.push_back(basic_point);
//
//			pcl::PointXYZRGB point;
//			point.x = basic_point.x;
//			point.y = basic_point.y;
//			point.z = basic_point.z;
//			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
//				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//			point.rgb = *reinterpret_cast<float*>(&rgb);
//			point_cloud_ptr->points.push_back(point);
//		}
//		if (z < 0.0)
//		{
//			r -= 12;
//			g += 12;
//		}
//		else
//		{
//			g -= 12;
//			b += 12;
//		}
//	}
//	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
//	basic_cloud_ptr->height = 1;
//	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
//	point_cloud_ptr->height = 1;
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ>("customData.pcd", *basic_cloud_ptr);
//	writer.write<pcl::PointXYZRGB>("customData_RGB.pcd",*point_cloud_ptr);
//	// ----------------------------------------------------------------
//	// -----——————计算搜索半径为0.05的曲面法线————————---
//	// ----------------------------------------------------------------
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;// 创建法线估计对象 ne
//	ne.setInputCloud(point_cloud_ptr);						// 将输入数据集传递给这个对象
//	// 创建空的Kd-tree对象，并将它传递给法线估计对象
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
//	ne.setSearchMethod(tree);
//	// 存储输出数据集
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
//	// 使用半径为5cm范围内的所有邻元素
//	ne.setRadiusSearch(0.05);
//	//计算特征值
//	ne.compute(*cloud_normals1);
//
//	// ---------------------------------------------------------------
//	// -----——————计算搜索半径为0.1的曲面法线————————---
//	// ---------------------------------------------------------------
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
//	ne.setRadiusSearch(0.1);
//	ne.compute(*cloud_normals2);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	if (simple)
//	{
//		viewer = simpleVis(basic_cloud_ptr);
//	}
//	else if (rgb)
//	{
//		viewer = rgbVis(point_cloud_ptr);
//	}
//	else if (custom_c)
//	{
//		viewer = customColourVis(basic_cloud_ptr);
//	}
//	else if (normals)
//	{
//		viewer = normalsVis(point_cloud_ptr, cloud_normals2);
//	}
//	else if (shapes)
//	{
//		viewer = shapesVis(point_cloud_ptr);
//	}
//	else if (viewports)
//	{
//		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
//	}
//
//	//--------------------
//	// -----—循环---——-
//	//--------------------
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}