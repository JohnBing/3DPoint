//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std;
//using namespace pcl;
//int main()
//{
//
//	//��ȡSTL��ʽģ��
//	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
//	reader->SetFileName("part_filtered.stl");
//	reader->Update();
//	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//	polydata = reader->GetOutput();
//	polydata->GetNumberOfPoints();
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	//��plyתpcd
//	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
//	//PCLVisualizer ��ʾԭSTL�ļ�
//	pcl::visualization::PCLVisualizer vis;
//	vis.addModelFromPolyData(polydata, "mesh", 0);
//	//����pcd�ļ�
//	pcl::io::savePCDFileASCII("part1.pcd", *cloud);
//	while (!vis.wasStopped())
//	{
//		vis.spinOnce();
//	}
//}
