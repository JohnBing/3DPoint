//
//#include <iostream>
//#include<pcl/io/pcd_io.h>
//#include<pcl/point_types.h>
//#include<pcl/point_cloud.h>
//#include<pcl/segmentation/sac_segmentation.h>
//#include<pcl/search/search.h>
//#include<pcl/search/kdtree.h>
//#include<pcl/features/normal_3d.h>
//#include<pcl/common/common.h>
//#include <pcl/filters/extract_indices.h>
////ransacÄâºÏ²¢ÆÀ¹À
//// use ransanc to fit cylinder
//int fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
//	// Create segmentation object for cylinder segmentation and set all the parameters
//	pcl::ModelCoefficients::Ptr coeffients_cylinder(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
//
//	//  Normals
//	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(new pcl::search::KdTree<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::Normal>::Ptr normalsFilter(new pcl::PointCloud<pcl::Normal>);
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
//	normalEstimator.setSearchMethod(tree);
//	normalEstimator.setInputCloud(cloud_in);
//	normalEstimator.setKSearch(30);
//	normalEstimator.compute(*normalsFilter);
//
//	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_CYLINDER);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	//seg.setNormalDistanceWeight(0.1);
//	seg.setNormalDistanceWeight(0.2);		// for coarse data
//	seg.setMaxIterations(1000);
//	seg.setDistanceThreshold(0.3);
//	seg.setRadiusLimits(0, 6);				// radius is within 8 centimeters
//	seg.setInputCloud(cloud_in);
//	seg.setInputNormals(normalsFilter);
//
//	// Perform segment
//	seg.segment(*inliers_cylinder, *coeffients_cylinder);
//	std::cerr << "Cylinder coefficient: " << *coeffients_cylinder << std::endl;
//
//	// Ouput extracted cylinder
//	pcl::ExtractIndices<pcl::PointXYZ> extract;
//	extract.setInputCloud(cloud_in);
//	extract.setIndices(inliers_cylinder);
//	extract.filter(*cloud_out);
//	pcl::PCDWriter wr;
//	wr.write("Extract_Cylinder.pcd", *cloud_out);
//
//	float sum_D = 0.0;
//	float sum_Ave = 0.0;
//	float x0 = coeffients_cylinder->values[0];
//	float y0 = coeffients_cylinder->values[1];
//	float z0 = coeffients_cylinder->values[2];
//	float l = coeffients_cylinder->values[3];
//	float m = coeffients_cylinder->values[4];
//	float n = coeffients_cylinder->values[5];
//	float r0 = coeffients_cylinder->values[6];
//	for (int i = 0; i < cloud_out->points.size(); i++) {
//		float x = cloud_out->points[i].x;
//		float y = cloud_out->points[i].y;
//		float z = cloud_out->points[i].z;
//		// D=part1+part2
//		float part1 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2) - pow(r0, 2);
//		float part2 = -pow(l*(x - x0) + m * (y - y0) + n * (z - z0), 2) / (l*l + m * m + n * n);
//		sum_D += pow(part1 + part2, 2);
//		sum_Ave += fabs(part1 + part2);
//	}
//	std::cerr << "The average difference is " << sum_Ave / cloud_out->points.size() << std::endl;;
//	std::cerr << "The Difference of average difference is " << sum_D / cloud_out->points.size() << std::endl;;
//	// evaluate the cylinder quation
//
//}