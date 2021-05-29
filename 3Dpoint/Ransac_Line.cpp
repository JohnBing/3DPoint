//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std;
//int main() {
//	//================��ȡ��������=====================
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDReader reader;
//	pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
//	//reader.read<pcl::PointXYZ>("l.pcd", *cloud);
//	//================�������ģ��=====================
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers�����洢ֱ���ϵ������
//	pcl::SACSegmentation<pcl::PointXYZ> seg;//����һ���ָ���
//	seg.setOptimizeCoefficients(true);      //��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
//	seg.setModelType(pcl::SACMODEL_LINE);   //����Ŀ�꼸����״
//	seg.setMethodType(pcl::SAC_RANSAC);     //��Ϸ��������������
//	seg.setDistanceThreshold(0.05);         //����������̷�Χ��Ҳ������ֵ
//	seg.setMaxIterations(500);              //�����������������õĻ�Ĭ�ϵ���50��
//	seg.setInputCloud(cloud);               //�������
//	seg.segment(*inliers, *coefficients);   //��ϵ���
//	//====================ģ��ϵ��=====================
//	cout << "���ֱ�ߵ�ģ��ϵ��Ϊ��" << endl;
//	cout << "a��" << coefficients->values[0] << endl;
//	cout << "b��" << coefficients->values[1] << endl;
//	cout << "c��" << coefficients->values[2] << endl;
//	cout << "d��" << coefficients->values[3] << endl;
//	cout << "e��" << coefficients->values[4] << endl;
//	cout << "f��" << coefficients->values[5] << endl;
//
//
//	//==================��ȡ��ϵ�ֱ��=================
//	/*ֱ����ȡ����1
//	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i < inliers->indices.size(); ++i) {
//		c_plane->points.push_back(cloud->points.at(inliers->indices[i]));
//	}
//	*/
//
//	//ֱ����ȡ
//	pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
//	extract.setInputCloud(cloud);    //�����������
//	extract.setIndices(inliers);     //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
//	extract.setNegative(false);      //false��ȡ�ڵ�, true��ȡ���
//	extract.filter(*line);        //��ȡ����洢��c_plane2
//	pcl::io::savePCDFile("l1.pcd", *line);
//	// ���ƿ��ӻ�
//	pcl::visualization::PCLVisualizer viewer;
//	//viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(line, 255, 0, 0);
//	viewer.addPointCloud(line, line_color, "line");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
//	viewer.spin();
//
//	return 0;
//}
//
