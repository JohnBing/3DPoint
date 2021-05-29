//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
//		cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
//		cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDReader reader;
//
//	//reader.read("table_scene_lms400_plane_0.pcd", *cloud);
//	reader.read("newPart_rest.pcd", *cloud);
//	
//
//
//	// Create a Concave Hull representation of the projected inliers
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::ConcaveHull<pcl::PointXYZ> chull;
//	chull.setInputCloud(cloud);
//	chull.setAlpha(0.1);
//	chull.reconstruct(*cloud_hull);
//
//	std::cerr << "Concave hull has: " << cloud_hull->points.size()
//		<< " data points." << std::endl;
//
//	pcl::PCDWriter writer;
//	writer.write("part_hull.pcd", *cloud_hull, false);
//
//	pcl::visualization::PCLVisualizer viewer("3D Viewer");
//	viewer.addPointCloud(cloud_hull);
//	viewer.spin();
//
//	system("pause");
//	return (0);
//}