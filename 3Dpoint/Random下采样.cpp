//#include <pcl/filters/random_sample.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/transforms.h>
//#include <time.h>
//#define PI 3.1415
//
//#include <vector>
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
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
//int main() {
//	PointCloudT::Ptr cloud_in(new PointCloudT), cloud_out(new PointCloudT);
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
//
//	pcl::io::loadPCDFile("data/tai_s.pcd", *cloud_in);
//
//	std::cerr << *cloud_in << std::endl;
//
//	pcl::RandomSample<PointT> rs;
//	rs.setInputCloud(cloud_in);
//	//��������������   
//	rs.setSample(5000);
//	clock_t start = clock();
//	//�²����������cloud_out
//	rs.filter(*cloud_out);
//	clock_t end = clock();
//	cout << "time:" << end - start << endl;
//
//	//��ȡ�������index
//		//std::vector<int> indices;
//	//rs.filter(indices);
//
//	std::cerr << *cloud_out << std::endl;
//	pcl::io::savePCDFile("data/zhou_s_random_5000.pcd",*cloud_out);
//	//���ӻ�
//	viewer->addPointCloud(cloud_out, "cloud_out");
//
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_in, 0, 255, 255);
//	viewer->addPointCloud(rotateABit(cloud_in), red, "cloud_in");
//
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce();
//	}
//	return (1);
//}
//
