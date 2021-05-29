//#include <pcl/io/pcd_io.h>
//#include <pcl/surface/mls.h>
////�Ա�Ե����������������ӵ�ԭ������
//
//int main(int argc, char** argv)
//{
//	// �½����ƴ洢����
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr part(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::io::loadPCDFile <pcl::PointXYZ>("part1_sb.pcd",*cloud);
//	pcl::io::loadPCDFile <pcl::PointXYZ>("part1_s.pcd", *part);
//	std::cout << "befor filtered:"<<cloud->points.size() << std::endl;
//	std::cout << "befor part+:" << part->points.size() << std::endl;
//
//	// �˲�����
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	//������������
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
//	filter.setSearchMethod(kdtree);
//	//������������İ뾶Ϊ3cm
//	filter.setSearchRadius(0.003);
//	// Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
//	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//	// �����İ뾶��
//	filter.setUpsamplingRadius(0.003);
//	// ���������Ĵ�С
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