//#include <iostream>
//#include <pcl/io/pcd_io.h>      //ioģ�� 
//#include <pcl/point_types.h>   //��������
//
///***********************************************************
//����һ�������ֺϲ���ʽ��
//	��һ�֣������ĺϲ�
//	�ڶ��֣����������ĺϲ�
//***********************************************************/
//int main(int argc, char** argv)
//{
//	//if (argc != 2) //��ʾ���ִ�п�ִ���ļ������������� -f ����-p
//	//{
//		//std::cerr << "please specify command line arg '-f' or '-p'" << std::endl;
//		//exit(0);
//	//}
//
//	//argv[0] = "-f";
//	////argv[1] = "-p";  //���� + ����
//	//argv[1] = "-f";  //���� + ����
//
//	//��������pcl::PointXYZ�����������ͣ��ֱ�Ϊcloud_a, cloud_b, cloud_c
//	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
//	//�洢��������ʱ��Ҫ��Normal����,Normal (float n_x, float n_y, float n_z)
//	pcl::PointCloud<pcl::Normal> n_cloud_b;           //�����ϲ��Ľ��
//	//�洢����XYZ��normal��ĵ���
//	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;    //���������ĺϲ��ṹ
//
//	// ������������
//	//����cloud_a�ĸ���Ϊ5
//	cloud_a.width = 5;
//	cloud_a.height = cloud_b.height = n_cloud_b.height = 1; //���ö�Ϊ�������
//	cloud_a.points.resize(cloud_a.width * cloud_a.height); //����
//	if (strcmp(argv[1], "-p") == 0)   //�ж��Ƿ�Ϊ����a+b=c(��������)
//	{
//		cloud_b.width = 3;   //���������������
//		cloud_b.points.resize(cloud_b.width * cloud_b.height);
//	}
//	else {  //�����ǵ��뷨��������
//		n_cloud_b.width = 5; //���������XYZ��normal������5�����ߣ��ֶμ����ӣ�
//		n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
//	}
//
//	//����ѭ�������������������涨����������͵ĵ�������
//	//��������� cloud_a
//	for (size_t i = 0; i < cloud_a.points.size(); ++i)
//	{  //cloud_a���������㣨ÿ���㶼��X Y Z �����������ֵ��
//		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//	}
//	if (strcmp(argv[1], "-p") == 0)
//		for (size_t i = 0; i < cloud_b.points.size(); ++i)
//		{   //�������a+b=c�������cloud_b������������Ϊxyz������
//			cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//			cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//			cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//		}
//	else
//		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
//		{  //�������xyz+normal=xyznormal�������n_cloud_b����5������Ϊnormal����
//			n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
//			n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
//			n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
//		}
//	/***************************************************************
//	��������������¹�����
//		һ�����������ӵ��ƻ��õ���5�����ƶ���
//			3�����루cloud_a cloud_b ��n_cloud_b��
//			2�������cloud_c  n_cloud_c��
//		����Ȼ�����Ϊ�����������cloud_a�� cloud_b����cloud_a ��n_cloud_b�������
//	****************************************************************/
//
//	//���Cloud A
//	std::cerr << "Cloud A: " << std::endl;
//	for (size_t i = 0; i < cloud_a.points.size(); ++i)
//		std::cerr << "    " << cloud_a.points[i].x
//		<< " " << cloud_a.points[i].y
//		<< " " << cloud_a.points[i].z
//		<< std::endl;
//
//	//���Cloud B
//	std::cerr << "Cloud B: " << std::endl;
//	if (strcmp(argv[1], "-p") == 0)
//		for (size_t i = 0; i < cloud_b.points.size(); ++i)  //��� Cloud_b
//			std::cerr << "    " << cloud_b.points[i].x
//			<< " " << cloud_b.points[i].y
//			<< " " << cloud_b.points[i].z
//			<< std::endl;
//	else//���n_Cloud_b
//		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
//			std::cerr << "    " << n_cloud_b.points[i].normal[0]
//			<< " " << n_cloud_b.points[i].normal[1]
//			<< " " << n_cloud_b.points[i].normal[2]
//			<< std::endl;
//
//	// Copy the point cloud data
//	if (strcmp(argv[1], "-p") == 0)
//	{
//		cloud_c = cloud_a;
//		cloud_c += cloud_b;//��cloud_a��cloud_b����һ�𴴽�cloud_c  �����
//		std::cerr << "Cloud C: " << std::endl;
//		for (size_t i = 0; i < cloud_c.points.size(); ++i)
//			std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;
//	}
//	else
//	{  //�����ֶ�  ��cloud_a�� n_cloud_b�ֶ����� һ�𴴽� p_n_cloud_c)
//		pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
//		std::cerr << "Cloud C: " << std::endl;
//		for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
//			std::cerr << "    " <<
//			p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " <<
//			p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
//	}
//	system("pause");
//	return (0);
//}
