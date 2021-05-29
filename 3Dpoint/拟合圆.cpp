//#include<iostream>
//#include<vector>
//#include<map>
//using namespace std;
//void FitCenterByLeastSquares(std::map<int, std::vector<double>> mapPoint, std::vector<double> &centerP, double &radius)
//{
//	double sumX = 0, sumY = 0;
//	double sumXX = 0, sumYY = 0, sumXY = 0;
//	double sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;
//
//	for (std::map<int, std::vector<double>>::iterator it = mapPoint.begin(); it != mapPoint.end(); ++it)
//	{
//		std::vector<double> p = it->second;
//
//		sumX += p[0];
//		sumY += p[1];
//		sumXX += p[0] * p[0];
//		sumYY += p[1] * p[1];
//		sumXY += p[0] * p[1];
//		sumXXX += p[0] * p[0] * p[0];
//		sumXXY += p[0] * p[0] * p[1];
//		sumXYY += p[0] * p[1] * p[1];
//		sumYYY += p[1] * p[1] * p[1];
//	}
//
//	int pCount = mapPoint.size();
//	double M1 = pCount * sumXY - sumX * sumY;
//	double M2 = pCount * sumXX - sumX * sumX;
//	double M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
//	double M4 = pCount * sumYY - sumY * sumY;
//	double M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);
//
//	double a = (M1 * M5 - M3 * M4) / (M2*M4 - M1 * M1);
//	double b = (M1 * M3 - M2 * M5) / (M2*M4 - M1 * M1);
//	double c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;
//
//	//Ô²ÐÄXY °ë¾¶
//	double xCenter = -0.5*a;
//	double yCenter = -0.5*b;
//	radius = 0.5 * sqrt(a * a + b * b - 4 * c);
//	centerP[0] = xCenter;
//	centerP[1] = yCenter;
//	cout << centerP[0] << centerP[1] << endl;
//	cout << radius;
//}
//int main()
//{
//	map<int, vector<double>> m;
//	vector<double> centerP(2);
//	double radius;
//	m[0] = { 1,0 };
//	m[1] = { 0,1 };
//	m[2] = { -1,0 };
//	FitCenterByLeastSquares(m, centerP, radius);
//	cout << centerP[0] << centerP[1] << endl;
//	cout << radius;
//	system("pause");
//}