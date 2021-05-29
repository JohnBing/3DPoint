//#include <iostream>
//#include <thread>
//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/sample_consensus/sac_model_circle.h>
//#include <pcl/sample_consensus/sac_model_circle3d.h>
//#include <pcl/sample_consensus/sac_model_cylinder.h>
//
//
//using namespace std::chrono_literals;
//
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
//int main(int argc, char** argv)
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<int> inliers;
//	pcl::PCDReader reader;
//	reader.read("data/newPart1.pcd", *cloud);
//	pcl::SampleConsensusModelCylinder<pcl::PointXYZ,pcl::Normal>::Ptr
//		model_cylinder(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud));
//
//
//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_cylinder);
//	ransac.setDistanceThreshold(.01);
//
//	ransac.computeModel();
//	ransac.getInliers(inliers);
//
//	Eigen::VectorXf tmpSphereMatrix;
//	ransac.getModelCoefficients(tmpSphereMatrix);
//	std::cout << tmpSphereMatrix << "\n\n";
//
//	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
//
//
//	pcl::visualization::PCLVisualizer::Ptr viewer;
//	viewer = simpleVis(final);
//	viewer->spin();
//
//	system("pause");
//	return 0;
//}