#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;
int
main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_r(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read("data/tai/turn0_jiang.pcd", *cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //����һ��PointIndices�ṹ��ָ��
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ
	seg.setOptimizeCoefficients(true); //���öԹ��Ƶ�ģ�����Ż�����
	// ��ѡ
	seg.setModelType(pcl::SACMODEL_PLANE);//���÷ָ�ģ�����
	seg.setMethodType(pcl::SAC_RANSAC);//����ʹ���Ǹ�����������Ʒ���
	seg.setMaxIterations(1000);//��������
	seg.setDistanceThreshold(0.001);//�����Ƿ�Ϊģ���ڵ�ľ�����ֵ
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	cout << *coefficients << endl;
	// �����˲�������
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	
	extract.setNegative(false);
	extract.filter(*cloud_p);
	extract.setNegative(true);//��ȡ���
	extract.filter(*cloud_r);//��������ȡ������浽cloud_r


	cout << "cloud: " << cloud->size() << endl;
	cout << "cloud_p: " << cloud_p->size() << endl;
	cout << "cloud_r: " << cloud_r->size() << endl;
	writer.write("data/tai_cloud_p.pcd", *cloud_p);
	writer.write("data/tai_cloud_r.pcd", *cloud_r);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0, 0, 0.25, 1, v1);
	viewer->setBackgroundColor(0, 0, 255, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 244, 89, 233);
	viewer->addPointCloud(cloud, color1, "cloud", v1);

	int v2(0);
	viewer->createViewPort(0.25, 0, 0.5, 1, v2);
	viewer->setBackgroundColor(0, 255, 255, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_p, 244, 89, 233);
	viewer->addPointCloud(cloud_p, color2, "cloud_p", v2);

	int v3(0);
	viewer->createViewPort(0.5, 0, 0.75, 1, v3);
	viewer->setBackgroundColor(34, 128, 0, v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud_r, 244, 89, 233);
	viewer->addPointCloud(cloud_r, color3, "cloud_r", v3);

	int v4(0);
	viewer->createViewPort(0.75, 0, 1, 1, v4);
	viewer->setBackgroundColor(0, 0, 255, v4);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color4(cloud_r, 244, 89, 233);
	viewer->addPointCloud(cloud_r, color4, "cloud_statical", v4);

	// viewer->addCoordinateSystem();//�������ϵ

	viewer->spin();

	return (0);
}

