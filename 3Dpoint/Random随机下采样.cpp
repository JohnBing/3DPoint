//#include <iostream>
//#include <cstdlib>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/common/transforms.h>
//
//#define PI 3.1415
//
//using namespace std;
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//unsigned int num_out_pt = 1024 * 2;
//
//PointCloudT::Ptr rotateABit(const PointCloudT::Ptr cloud_in) {
//	PointCloudT::Ptr cloud_transformed(new PointCloudT);
//	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//	float theta_deg = 30;
//	float c = cos(theta_deg*PI / 180.0);
//	float s = sin(theta_deg*PI / 180.0);
//	transform(1, 1) = c;
//	transform(3, 3) = c;
//	transform(1, 3) = -s;
//	transform(3, 1) = s;
//	pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);
//	return cloud_transformed;
//}
//
//int main(int argc, char** argv)
//{
//	PointCloudT::Ptr cloud_in(new PointCloudT), cloud_out(new PointCloudT);
//	pcl::io::loadPCDFile("data/zhou_s.pcd", *cloud_in);
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
//
//	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
//	std::vector<int> idx;
//
//	unsigned int num_pt = cloud_in->points.size();
//	cout << "Input point cloud's point number = " << num_pt << endl;
//
//	vector<int> label(num_pt,0);
//
//
//	if (num_pt < num_out_pt)
//		return (-1);
//
//	for (int i = 0; i < num_out_pt; i++) {
//		//int a_rand_num = rand() % num_pt;
//		int a_rand_num=(int)(((double)rand() / RAND_MAX) * num_pt);
//		cout << a_rand_num << endl;
//		while (1) {
//			if (label[a_rand_num] == 0) {
//				idx.push_back(a_rand_num);
//				label[i] = 1;
//				break;
//			}
//			a_rand_num = (int)(((double)rand() / RAND_MAX) * num_pt);
//			cout << a_rand_num << endl;
//		}
//	}
//
//	pcl::copyPointCloud(*cloud_in, idx, *cloud_out);
//	cout << "Ouput point cloud's point number = " << cloud_out->points.size() << endl;
//	pcl::io::savePCDFile("data/zhou_s_random_5000_.pcd", *cloud_out);
//
//	viewer->addPointCloud(rotateABit(cloud_in), "cloud_in");
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_out, 255, 0, 0);
//	viewer->addPointCloud(cloud_out, red, "cloud_out");
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce();
//	}
//
//	return (0);
//}