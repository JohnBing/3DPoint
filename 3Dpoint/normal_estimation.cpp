//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
////���÷�������ʾʱ�����������н��
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//
//int
//main ()
// {
//	//���ص���
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	//pcl::io::loadPCDFile ("part.pcd", *cloud);
//	pcl::io::loadPCDFile ("newPart.pcd", *cloud);
//
//	//cloud->width = 100000;
//	//cloud->height = 1;
//	//cloud->is_dense = false;
//	//cloud->points.resize(cloud->width * cloud->height);
//	//for (size_t i = 0; i < cloud->points.size(); ++i)
//	//{
//	//	cloud->points[i].x = (rand() / (RAND_MAX + 1.0) - 0.5);
//	//	cloud->points[i].y = (rand() / (RAND_MAX + 1.0) - 0.5);
//	//	cloud->points[i].z = (rand() / (RAND_MAX + 1.0) - 0.5);
//	//	//if (i % 2 == 0)
//	//	//{
//	//	//	cloud->points[i].y = (rand() / (RAND_MAX + 1.0) - 0.5);
//	//	//	cloud->points[i].z = 0;
//	//	//}
//	//}
//	//���Ʒ���
//
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setInputCloud (cloud);
//	//����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
//	//���ڸ������������ݼ���kdtree��������
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//	ne.setSearchMethod (tree);
//	//������ݼ�
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//	//ʹ�ð뾶�ڲ�ѯ����Χ3���׷�Χ�ڵ�������Ԫ��
//	//ne.setRadiusSearch (0.03);
//	ne.setKSearch(20);
//	//��������ֵ
//	ne.setViewPoint(0.015, 0.05, 0.015);
//	//ne.useSensorOriginAsViewPoint();
//	ne.compute (*cloud_normals);
//	pcl::io::savePCDFile("nolmal.pcd", *cloud_normals);
//	cout << "cloud size" << cloud->points.size()<<endl;
//	cout << "cloud nolmal size" << cloud_normals->points.size() << endl;
//	
//	std::vector<int> index;
//	for (int i=0;i<cloud_normals->points.size();i++)
//	{
//		if (cloud_normals->points[i].curvature > 0.1)
//			index.push_back(i);
//	}
//	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, index, *final);
//
//
//	float x, y, z;
//	ne.getViewPoint(x, y, z);
//	cout << x <<" "<< y <<" "<< z <<" "<< endl;
//	pcl::io::savePCDFile("��Ե.pcd", *final);
//
//	//���߿��ӻ�
//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//	viewer.setBackgroundColor (0.0, 0.0, 0.0);
//	viewer.addPointCloud<pcl::PointXYZ>(final);
//
//	//viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals,50);
//	//viewer.addPointCloud<pcl::PointXYZ>
//	//viewer.addCoordinateSystem(1.0);
//
//	viewer.spin();
//
//
//	return 0;
//}
