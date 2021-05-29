////��λ�����ͼ��
//#include <pcl/range_image/range_image.h>    //���ͼ���ͷ�ļ�
//
//int main(int argc, char** argv) {
//	pcl::PointCloud<pcl::PointXYZ> pointCloud;   //������ƵĶ���
//
//	// ѭ���������Ƶ�����
//	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
//		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
//			pcl::PointXYZ point;
//			point.x = 2.0f - y;
//			point.y = y;
//			point.z = z;
//			pointCloud.points.push_back(point); //ѭ����ӵ����ݵ����ƶ���
//		}
//	}
//	pointCloud.width = (uint32_t)pointCloud.points.size();
//	pointCloud.height = 1;                                        //���õ��ƶ����ͷ��Ϣ
//	  //ʵ��һ���ʾ�����״�ĵ���
//	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
//	 //angular_resolutionΪģ�����ȴ������ĽǶȷֱ��ʣ������ͼ����һ�����ض�Ӧ�ĽǶȴ�С
//	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
//	 //max_angle_widthΪģ�����ȴ�������ˮƽ�������Ƕȣ�
//	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
//	//max_angle_heightΪģ�⴫�����Ĵ�ֱ�����������Ƕ�  ��תΪ����
//	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
//	 //�������Ĳɼ�λ��
//	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
//	//���ͼ����ѭ����ϵͳ
//	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//	float noiseLevel = 0.00;    //noise_level��ȡ���ͼ�����ʱ�����ڵ�Բ�ѯ�����ֵ��Ӱ��ˮƽ
//	float minRange = 0.0f;     //min_range������С�Ļ�ȡ���룬С����С��ȡ�����λ��Ϊ��������ä��
//	int borderSize = 1;        //border_size������ͼ��ı�Ե�Ŀ��
//
//	pcl::RangeImage rangeImage;
//	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
//		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//	std::cout << rangeImage << "\n";
//	system("pause");
//}
