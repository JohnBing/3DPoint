//HOUGH_LINE.cpp文件
#include "HOUGH_LINE.h"
HOUGH_LINE::HOUGH_LINE()
{
}

HOUGH_LINE::~HOUGH_LINE()
{
	cloud->clear();
	result_.clear();
}
void HOUGH_LINE::VoxelGrid_(float size_, PointCloud<PointT>::Ptr &voxel_cloud) {
	PointCloud<PointT>::Ptr tem_voxel_cloud(new PointCloud<PointT>);
	VoxelGrid<PointT> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(size_, size_, size_);
	vox.filter(*tem_voxel_cloud);
	cloud = tem_voxel_cloud;
	voxel_cloud = tem_voxel_cloud;
	point_num = cloud->size();
}
void HOUGH_LINE::draw_hough_spacing() {
	visualization::PCLPlotter *plot_(new visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
	plot_->setBackgroundColor(1, 1, 1);
	plot_->setTitle("hough space");
	plot_->setXTitle("angle");
	plot_->setYTitle("rho");
	vector<pair<double, double>>data_;
	double x_resolution1 = M_PI / 181;
	double a_1, b_1;
	for (int i_point = 0; i_point < point_num; i_point++)
	{
		a_1 = cloud->points[i_point].x;
		b_1 = cloud->points[i_point].y;
		for (int i = 0; i < 181; i++)
		{
			data_.push_back(make_pair(i*x_resolution1, a_1 * cos(i*x_resolution1) + b_1 * sin(i*x_resolution1)));
		}
		plot_->addPlotData(data_, "line", vtkChart::LINE);//X,Y均为double型的向量
		data_.clear();
	}
	plot_->setShowLegend(false);
	plot_->plot();//绘制曲线
}

void HOUGH_LINE::HOUGH_line(int x_setp_num, double y_resolution, int grid_point_number_threshold, vector<float>&K_, vector<float>&B_, int line_num) {
	vector<vector<int>>all_point_row_col;
	vector<int> Grid_Index;
	getMinMax3D(*cloud, point_min, point_max);
	double x_resolution = M_PI / x_setp_num;
	int raster_rows, raster_cols;
	raster_rows = ceil((M_PI - 0) / x_resolution);
	raster_cols = ceil((point_max.y + point_max.x) / y_resolution) * 2;
	MatrixXi all_row_col(raster_cols, raster_rows);//存储每个格网内的个数
	all_row_col.setZero();
	MatrixXd all_row_col_mean(raster_cols, raster_rows);//存储每个格网内的均值
	//统计每个格网内的数量，和rho均值
	all_row_col_mean.setZero();
	double a_, b_;
	for (int i_point = 0; i_point < point_num; i_point++) {
		cout << i_point << endl;
		a_ = cloud->points[i_point].x;
		b_ = cloud->points[i_point].y;
		for (int i_ = 0; i_ < x_setp_num; i_++)
		{
			double theta = 0 + i_ * x_resolution + x_resolution / 2;
			double rho = a_ * cos(theta) + b_ * sin(theta);
			//double rho = line_(theta,a_,b_);
			double idx = ceil(abs(rho / y_resolution));
			if (rho >= 0)
			{
				all_row_col(raster_cols / 2 - idx, i_) += 1;
				all_row_col_mean(raster_cols / 2 - idx, i_) += rho;
			}
			else {
				all_row_col(raster_cols / 2 + idx, i_) += 1;
				all_row_col_mean(raster_cols / 2 + idx, i_) += rho;
			}
		}
	}
	//求解邻域
	int min_num_threshold = grid_point_number_threshold;
	vector<pair<double, double>>result_jz;
	//vector<pair<double, double>>result_;
	vector<int>tem_nebor;
	vector<int>num_grid;
	for (int i_row = 0; i_row < all_row_col.rows(); i_row++)
	{
		for (int i_col = 0; i_col < all_row_col.cols(); i_col++)
		{
			if (i_row == 0)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
				}
			}
			if (i_row == all_row_col.rows() - 1)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
				}
			}
			if (i_row != all_row_col.rows() - 1 && i_row != 0)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
				}
			}
			int max_tem = *max_element(tem_nebor.begin(), tem_nebor.end());
			tem_nebor.clear();
			if (all_row_col(i_row, i_col) > max_tem)
			{
				num_grid.push_back(all_row_col(i_row, i_col));
				double tem_ = all_row_col_mean(i_row, i_col) / all_row_col(i_row, i_col);
				result_jz.push_back(make_pair(0 + i_col * x_resolution + x_resolution / 2, tem_));
				if (all_row_col(i_row, i_col) > min_num_threshold)
				{
					result_.push_back(make_pair(0 + i_col * x_resolution + x_resolution / 2, tem_));
				}
			}
		}
	}
	if (line_num != -1)
	{
		result_.clear();
		vector<size_t>idx_;
		vector_sort(num_grid, idx_);
		if (result_jz.size() < line_num)
		{
			line_num = result_jz.size();
		}
		for (int i_ = 0; i_ < line_num; i_++)
		{
			result_.push_back(result_jz[idx_[i_]]);
		}
	}
	for (int i_hough = 0; i_hough < result_.size(); i_hough++) {
		B_.push_back(result_[i_hough].second / sin(result_[i_hough].first));
		K_.push_back(-cos(result_[i_hough].first) / sin(result_[i_hough].first));
	}
}


void HOUGH_LINE::draw_hough_line() {
	visualization::PCLPlotter *plot_line(new visualization::PCLPlotter);
	plot_line->setBackgroundColor(1, 1, 1);
	plot_line->setTitle("line display");
	plot_line->setXTitle("x");
	plot_line->setYTitle("y");
	vector<double>x_, y_;
	for (int i_point = 0; i_point < point_num; i_point++)
	{
		x_.push_back(cloud->points[i_point].x);
		y_.push_back(cloud->points[i_point].y);
	}
	for (int i_hough = 0; i_hough < result_.size(); i_hough++) {
		std::vector<double> func1(2, 0);
		func1[0] = result_[i_hough].second / sin(result_[i_hough].first);
		func1[1] = -cos(result_[i_hough].first) / sin(result_[i_hough].first);
		plot_line->addPlotData(func1, point_min.x, point_max.x);
	}
	plot_line->addPlotData(x_, y_, "display", vtkChart::POINTS);//X,Y均为double型的向量
	plot_line->setShowLegend(false);
	plot_line->plot();//绘制曲线
	//plot_line->spin();
}
