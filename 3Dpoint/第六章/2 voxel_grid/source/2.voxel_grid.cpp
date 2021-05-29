//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//
//int
//main (int argc, char** argv)
//{
//
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCDReader reader;
//
//  reader.read ("testPart1.pcd", *cloud); 
//  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
//       << " data points (" << pcl::getFieldsList (*cloud) << ").";
//
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.001f, 0.001f, 0.001f);
//  sor.filter (*cloud_filtered);
//  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
//       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
//  pcl::PCDWriter writer;
//
//  writer.write ("testPartFiltered.pcd", *cloud_filtered);
//  system("pause");
//  return (0);
//}
