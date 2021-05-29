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
//
//typedef pcl::PointXYZ PointT;
//
//
//int
//main(int argc, char** argv)
//{
//	// All the objects needed
//	pcl::PCDReader reader;
//	pcl::PCDWriter writer;
//	pcl::PassThrough<PointT> pass;
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//	pcl::ExtractIndices<PointT> extract;
//	pcl::ExtractIndices<pcl::Normal> extract_normals;
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	pcl::visualization::PCLVisualizer viewer("3D Viewer");
//
//	// Datasets
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//
//	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
//
////======================================================================================================
//	// 读数据
//	reader.read("data/zhou_s.pcd", *cloud);
//	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
//
////======================================================================================================
//	// //直通滤波只保留某个轴的某一段
//	//pass.setInputCloud(cloud);
//	//pass.setFilterFieldName("z");
//	//pass.setFilterLimits(-0.06, 0);
//	//pass.filter(*cloud_filtered);
//	//std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
//	cloud_filtered = cloud;
//
////======================================================================================================
//	// 计算法向量
//	ne.setSearchMethod(tree);
//	//ne.setInputCloud(cloud);
//	ne.setInputCloud(cloud_filtered);
//	ne.setKSearch(25);
//	ne.compute(*cloud_normals);
////======================================================================================================
//
//	// Create the segmentation object for cylinder segmentation and set all the parameters
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_CYLINDER);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setNormalDistanceWeight(0.1);
//	seg.setMaxIterations(10000);
//	seg.setDistanceThreshold(0.001);
//	seg.setRadiusLimits(0.02, 0.03);
//	seg.setInputCloud(cloud_filtered);
//	seg.setInputNormals(cloud_normals);
//
//	// Obtain the cylinder inliers and coefficients
//	seg.segment(*inliers_cylinder, *coefficients_cylinder);
//	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//
//	//Eigen::VectorXf vec1(3),vec2(3);
//	//vec1 << 0, 0, 1;
//	//vec2 << coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5];
//	//cout << vec1 * vec2.inverse() << endl;
//
////======================================================================================================提取
//	extract.setInputCloud(cloud_filtered);
//	extract.setIndices(inliers_cylinder);
//	extract.setNegative(false);
//	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
//	extract.filter(*cloud_cylinder);
//
//	pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>());
//	extract.setNegative(true);
//	extract.filter(*cloud_rest);
//
//
//	////====================最小二乘拟合求圆柱直径
//	//pcl::PointCloud<PointT>::Ptr cloud_circle(new pcl::PointCloud<PointT>);
//	//std::vector<std::vector<double>> date;
//	//std::vector<double> centerP(2);
//	//double radius;
//	//for (auto m : cloud_cylinder->points)
//	//{
//	//	if (abs(m.y-0.05)<0.0001&&m.x>0.015&&m.z>0.015)
//	//	{
//	//		cloud_circle->points.push_back(m);
//	//		date.push_back({ m.x,m.z });
//	//	}
//	//}
//	//cout << "circle size:" << date.size()<<endl;
//	//FitCenterByLeastSquares(date, centerP, radius);
//	
//	////=============ransac拟合圆柱
//	//	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
//	//	model_circle2D(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud_circle));
//	//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);
//	//	ransac.setDistanceThreshold(10);
//	//	ransac.computeModel();
//	//	//ransac.getInliers(inliers);
//	//	Eigen::VectorXf tmpSphereMatrix;
//	//	ransac.getModelCoefficients(tmpSphereMatrix);
//	//	std::cout << tmpSphereMatrix << "\n\n";
//
////======================================================================================================
//	if (cloud_cylinder->points.empty())
//		std::cerr << "Can't find the cylindrical component." << std::endl;
//	else
//	{
//		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
//		writer.write("data/zhou_s_cylinder_3.pcd", *cloud_cylinder, false);
//		writer.write("data/zhou_s_rest_3.pcd", *cloud_rest, false);
//
//		int v1(0);
//		viewer.createViewPort(0, 0, 0.5, 1, v1);
//		viewer.addPointCloud(cloud_cylinder, "cloud_cylinder", v1);
//
//		int v2(0);
//		viewer.createViewPort(0.5, 0, 1, 1, v2);
//		viewer.addPointCloud(cloud_filtered,"cloud_rest",v2);
//
//		viewer.spin();
//	}
//	system("pause");
//	return (0);
//}
