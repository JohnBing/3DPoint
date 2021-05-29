//#include <iostream>
//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>				// 法线估计类头文件声明
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
////每次响应这种事件都会在鼠标按下的位置上生成一个文本标签
//unsigned int text_id = 0;
//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
//	void* viewer_void)
//{
//	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//	if (event.getKeySym() == "r" && event.keyDown())
//	{
//		std::cout << "r was pressed => removing all text" << std::endl;
//
//		char str[512];
//		for (unsigned int i = 0; i < text_id; ++i)
//		{
//			sprintf(str, "text#%03d", i);
//			viewer->removeShape(str);
//		}
//		text_id = 0;
//	}
//}
