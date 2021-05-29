//#include <pcl/io/pcd_io.h>
//#include <pcl/surface/mls.h>
////对边缘进行增采样，并添加到原点云中
//
//int main(int argc, char** argv)
//{
//	// 新建点云存储对象
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr part(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::io::loadPCDFile <pcl::PointXYZ>("part1_sb.pcd",*cloud);
//	pcl::io::loadPCDFile <pcl::PointXYZ>("part1_s.pcd", *part);
//	std::cout << "befor filtered:"<<cloud->points.size() << std::endl;
//	std::cout << "befor part+:" << part->points.size() << std::endl;
//
//	// 滤波对象
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	//建立搜索对象
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
//	filter.setSearchMethod(kdtree);
//	//设置搜索邻域的半径为3cm
//	filter.setSearchRadius(0.003);
//	// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
//	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//	// 采样的半径是
//	filter.setUpsamplingRadius(0.003);
//	// 采样步数的大小
//	filter.setUpsamplingStepSize(0.002);
//	filter.process(*filteredCloud);
//
//	*part += *filteredCloud;
//	std::cout << "after filtered:" << filteredCloud->points.size() << std::endl;
//	std::cout << "after part+:" << part->points.size() << std::endl;
//	pcl::io::savePCDFile("part1_sbz.pcd", *filteredCloud);
//	pcl::io::savePCDFile("part1_sz.pcd", *part);
//
//	system("pause");
//	
//}