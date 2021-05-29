//#include <iostream>
//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/sample_consensus/sac_model_circle.h>
//#include <pcl/sample_consensus/sac_model_cylinder.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/features/normal_3d.h>
//
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
//typedef pcl::PointXYZ PointT;
//int
//main(int argc, char** argv)
//{
//	srand(time(NULL));
//
//	// initialize PointClouds
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("cloud_cluster_0.pcd", *cloud);
//	cout << "befor detect:" << cloud->size()<<endl;
//
//
//	//std::vector<int> inliers;
//	//// created RandomSampleConsensus object and compute the appropriated model
//	//pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr
//	//	model_circle2D(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud));
//
//	//pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);
//	//ransac.setDistanceThreshold(.01);
//	//ransac.computeModel();
//	//ransac.getInliers(inliers);
//
//
//	// Estimate point normals
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	ne.setSearchMethod(tree);
//	ne.setInputCloud(cloud);
//	ne.setKSearch(50);
//	ne.compute(*cloud_normals);
//
//
//	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
//
//	// Create the segmentation object for cylinder segmentation and set all the parameters
//	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_CYLINDER);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setNormalDistanceWeight(0.1);
//	seg.setMaxIterations(10000);
//	seg.setDistanceThreshold(0.05);
//	seg.setRadiusLimits(0, 0.1);
//	seg.setInputCloud(cloud);
//	seg.setInputNormals(cloud_normals);
//
//	// Obtain the cylinder inliers and coefficients
//	seg.segment(*inliers_cylinder, *coefficients_cylinder);
//	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//
//
//	system("pause");
//	return 0;
//}
