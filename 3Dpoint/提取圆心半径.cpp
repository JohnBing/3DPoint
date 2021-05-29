//#include <iostream>
//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/sample_consensus/sac_model_circle.h>
//
//
//int
//main(int argc, char** argv)
//{
//	srand(time(NULL));
//	// initialize PointClouds
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile("cloud_cluster_2.pcd", *cloud);
//	cout << "befor detect:" << cloud->size()<<endl;
//
//	std::vector<int> inliers;
//	// created RandomSampleConsensus object and compute the appropriated model
//	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
//		model_circle2D(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));
//
//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);
//	ransac.setDistanceThreshold(.01);
//	ransac.computeModel();
//	ransac.getInliers(inliers);
//
//
//	// copies all inliers of the model computed to another PointCloud
//	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
//	cout << "after detect:" << final->size()<<endl;
//
//	Eigen::VectorXf tmpSphereMatrix;
//	ransac.getModelCoefficients(tmpSphereMatrix);
//	std::cout << tmpSphereMatrix << "\n\n";
//	system("pause");
//	return 0;
//}
