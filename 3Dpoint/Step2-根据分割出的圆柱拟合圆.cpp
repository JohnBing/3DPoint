//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_circle.h>
//#include <Eigen/Dense>
//#include <pcl/common/transforms.h>
//#include <pcl/sample_consensus/sac_model_cylinder.h>
//
//using namespace Eigen;
//using namespace std;
//
//typedef pcl::PointXYZ PointT;
//
//void tranformPCL(Vector3f &line, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed)
//{
//	double a = line(0), b = line(1), c = line(2);
//	double alpha = asin(b / sqrt(b*b + c * c));
//	double beita = asin(a / sqrt(a*a + b * b + c * c));
//
//	Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
//
//	transform_x(1, 1) = cos(alpha);
//	transform_x(1, 2) = sin(alpha);
//	transform_x(2, 1) = -sin(alpha);
//	transform_x(2, 2) = cos(alpha);
//
//	transform_y(0, 0) = cos(beita);
//	transform_y(0, 2) = -sin(beita);
//	transform_y(2, 0) = sin(beita);
//	transform_y(2, 2) = cos(beita);
//
//	Eigen::Matrix4f R = transform_y * transform_x.inverse();
//	cout << R << endl;
//	pcl::transformPointCloud(*cloud, *transformed, R);
//
//}
//
//void FitCenterByLeastSquares(std::vector<std::vector<double>> Points, std::vector<double> &centerP, double &radius)
//{
//	double sumX = 0, sumY = 0;
//	double sumXX = 0, sumYY = 0, sumXY = 0;
//	double sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;
//
//	for (auto p : Points)
//	{
//
//		sumX += p[0];
//		sumY += p[1];
//		sumXX += p[0] * p[0];
//		sumYY += p[1] * p[1];
//		sumXY += p[0] * p[1];
//		sumXXX += p[0] * p[0] * p[0];
//		sumXXY += p[0] * p[0] * p[1];
//		sumXYY += p[0] * p[1] * p[1];
//		sumYYY += p[1] * p[1] * p[1];
//	}
//
//	int pCount = Points.size();
//	double M1 = pCount * sumXY - sumX * sumY;
//	double M2 = pCount * sumXX - sumX * sumX;
//	double M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
//	double M4 = pCount * sumYY - sumY * sumY;
//	double M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);
//
//	double a = (M1 * M5 - M3 * M4) / (M2*M4 - M1 * M1);
//	double b = (M1 * M3 - M2 * M5) / (M2*M4 - M1 * M1);
//	double c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;
//
//	//圆心XY 半径
//	double xCenter = -0.5*a;
//	double yCenter = -0.5*b;
//	radius = 0.5 * sqrt(a * a + b * b - 4 * c);
//	centerP[0] = xCenter;
//	centerP[1] = yCenter;
//	cout << centerP[0] << centerP[1] << endl;
//	cout << radius << endl;
//}
//
//Vector3f RanSACCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//		pcl::NormalEstimation<PointT, pcl::Normal> ne;
//		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//		// 计算法向量
//		ne.setSearchMethod(tree);
//		//ne.setInputCloud(cloud);
//		ne.setInputCloud(cloud);
//		ne.setKSearch(25);
//		ne.compute(*cloud_normals);
//		//======================================================================================================
//		// Create the segmentation object for cylinder segmentation and set all the parameters
//		seg.setOptimizeCoefficients(true);
//		seg.setModelType(pcl::SACMODEL_CYLINDER);
//		seg.setMethodType(pcl::SAC_RANSAC);
//		seg.setNormalDistanceWeight(0.1);
//		seg.setMaxIterations(10000);
//		seg.setDistanceThreshold(0.005);
//		seg.setRadiusLimits(0.02, 0.03);
//		seg.setInputCloud(cloud);
//		seg.setInputNormals(cloud_normals);
//	
//		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
//		pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
//
//		seg.segment(*inliers_cylinder, *coefficients_cylinder);
//		std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//		Vector3f line;
//		line(0) = abs(coefficients_cylinder->values[3]);
//		line(1) = abs(coefficients_cylinder->values[4]);
//		line(2) = abs(coefficients_cylinder->values[5]);
//		return line;
//}
//
//int main()
//{
//	pcl::PCDReader reader;
//	pcl::PCDWriter writer;
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>);
//	//reader.read("data/part5_s_cylinder.pcd", *cloud);
//	reader.read("data/zhou_s_cylinder_3.pcd", *cloud);
//	
//	Vector3f line=RanSACCylinder(cloud);
//	cout << line<<endl;
//	tranformPCL(line, cloud, transformed);
//	writer.write("data/zhou_s_cylinder_3_transform.pcd", *transformed);
//	//====================最小二乘拟合求圆柱直径
//	pcl::PointCloud<PointT>::Ptr cloud_circle(new pcl::PointCloud<PointT>);
//	std::vector<double> centerP(2);
//	double radius;
//	float minZ = 1, maxZ = -1;
//	for (auto m : transformed->points)
//	{
//		minZ = min(m.z, minZ);
//		maxZ = max(m.z, maxZ);
//	}
//	cout << "minZ:" << minZ <<" "<< "maxZ" << maxZ << endl;
//	cout << "=================" << endl;
//	cout << "输入取值的位置：" << endl;
//	float T;
//	while (cin >> T)
//	{
//		std::vector<std::vector<double>> date;
//		for (auto m : transformed->points)
//		{
//			if (abs(m.z - T) < 0.0005)
//			{
//				cloud_circle->points.push_back(m);
//				date.push_back({ m.x,m.y });
//			}
//		}
//		cout << "circle size:" << date.size() << endl;
//		FitCenterByLeastSquares(date, centerP, radius);
//		cout << "圆心" << centerP[0] << " " << centerP[1] << endl;
//		cout << "半径" << radius << endl;
//		cout << "直径" << 2*radius << endl;
//		cout << "输入取值的位置：" << endl;
//		cout << "=================" << endl;
//		cout << endl;
//	}
//
//	pcl::visualization::PCLVisualizer viewer("3D Viewer");
//
//	int v1(0);
//	viewer.createViewPort(0, 0, 0.5, 1, v1);
//	viewer.addPointCloud(cloud_circle, "cloud_cylinder", v1);
//	viewer.addCoordinateSystem(0.001, "cloud_cylinder", v1);
//
//	int v2(0);
//	viewer.createViewPort(0.5, 0, 1, 1, v2);
//	viewer.addPointCloud(transformed,"cloud_rest",v2);
//
//	viewer.spin();
//}