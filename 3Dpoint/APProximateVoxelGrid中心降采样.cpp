//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/approximate_voxel_grid.h>
//#include <time.h>
//
//pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 125, 125, 125); 
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud,color, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	//viewer->initCameraParameters();
//	return (viewer);
//}
//
//
//int
//main (int argc, char** argv)
//{
//
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCDReader reader;
//
//  //reader.read ("part1_s.pcd", *cloud);
//  reader.read("data/tai_s.pcd", *cloud);
//  
//  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
//       << " data points (" << pcl::getFieldsList (*cloud) << ").";
//
//  clock_t start = clock();
//  pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.0005f, 0.0005f, 0.0005f);
//  sor.filter (*cloud_filtered);
//  
//  clock_t end = clock();
//  cout << "time" << end - start << endl;
//
//  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
//       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
//  pcl::PCDWriter writer;
//  writer.write ("data/zhou_s_AXVG.pcd", *cloud_filtered);
//
//
//  pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud_filtered);
//  viewer->spin();
//
//  
//  system("pause");
//  return (0);
//}
