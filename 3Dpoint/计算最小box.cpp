//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <Eigen/Core>
//#include <pcl/common/transforms.h>
//#include <pcl/common/common.h>
//
//
//using namespace std;
//typedef pcl::PointXYZ PointType;
//
//int main(int argc, char **argv)
//{
//	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
//
//	Eigen::Vector4f pcaCentroid;
//	pcl::compute3DCentroid(*cloud, pcaCentroid);
//	Eigen::Matrix3f covariance;
//	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
//	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
//	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //У��������䴹ֱ
//	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
//	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
//
//	std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
//	std::cout << "��������ve(3x3):\n" << eigenVectorsPCA << std::endl;
//	std::cout << "���ĵ�(4x1):\n" << pcaCentroid << std::endl;
//	/*
//	// ��һ�ּ������Э�����������ֵ�����������ķ�ʽ:ͨ��pcl�е�pca�ӿڣ����£���������õ�����������������������
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCA<pcl::PointXYZ> pca;
//	pca.setInputCloud(cloudSegmented);
//	pca.project(*cloudSegmented, *cloudPCAprojection);
//	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//������������
//	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//��������ֵ
//	*/
//	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
//	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
//	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
//	tm_inv = tm.inverse();
//
//	std::cout << "�任����tm(4x4):\n" << tm << std::endl;
//	std::cout << "������tm'(4x4):\n" << tm_inv << std::endl;
//
//	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
//	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
//
//	PointType min_p1, max_p1;   //���Ƶ����ֵ����Сֵ��
//	Eigen::Vector3f c1, c;
//	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
//	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
//
//	std::cout << "����c1(3x1):\n" << c1 << std::endl;
//
//	Eigen::Affine3f tm_inv_aff(tm_inv);
//	pcl::transformPoint(c1, c, tm_inv_aff);
//
//	Eigen::Vector3f whd, whd1;
//	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
//	whd = whd1;
//	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С
//
//	std::cout << "width1=" << whd1(0) << endl;
//	std::cout << "heght1=" << whd1(1) << endl;
//	std::cout << "depth1=" << whd1(2) << endl;
//	std::cout << "scale1=" << sc1 << endl;
//
//	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
//	const Eigen::Vector3f    bboxT1(c1);
//
//	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
//	const Eigen::Vector3f    bboxT(c);
//
//
//	//�任��ԭ��ĵ���������
//	PointType op;
//	op.x = 0.0;
//	op.y = 0.0;
//	op.z = 0.0;
//	Eigen::Vector3f px, py, pz;
//	Eigen::Affine3f tm_aff(tm);
//	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
//	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
//	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
//	PointType pcaX;
//	pcaX.x = sc1 * px(0);
//	pcaX.y = sc1 * px(1);
//	pcaX.z = sc1 * px(2);
//	PointType pcaY;
//	pcaY.x = sc1 * py(0);
//	pcaY.y = sc1 * py(1);
//	pcaY.z = sc1 * py(2);
//	PointType pcaZ;
//	pcaZ.x = sc1 * pz(0);
//	pcaZ.y = sc1 * pz(1);
//	pcaZ.z = sc1 * pz(2);
//
//	//visualization
//	pcl::visualization::PCLVisualizer viewer;
//
//
//	pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 255, 0); //����ĳ�ʼ�������
//	viewer.addPointCloud(cloud, color_handler, "cloud");
//	viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
//
//
//	viewer.addCoordinateSystem(0.5f*sc1);
//	viewer.setBackgroundColor(0.0, 0.0, 0.0);
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//
//	return 0;
//}
//
