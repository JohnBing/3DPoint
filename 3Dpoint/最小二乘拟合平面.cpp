//////https://www.cnblogs.com/zhangli07/p/12013561.html
////
//bool gFittingPlane(double *x, double *y, double *z, int n, double &a, double &b, double &c)
//{
//	int i;
//	double x1, x2, y1, y2, z1, xz, yz, xy, r;
//
//	x1 = x2 = y1 = y2 = z1 = xz = yz = xy = 0;
//	for (i = 0; i < n; i++)
//	{
//		x1 += x[i];
//		x2 += x[i] * x[i];
//		xz += x[i] * z[i];
//
//		y1 += y[i];
//		y2 += y[i] * y[i];
//		yz += y[i] * z[i];
//
//		z1 += z[i];
//		xy += x[i] * y[i];
//	}
//
//	r = gDeterm3(x2, xy, x1, xy, y2, y1, x1, y1, n);
//	if (IS_ZERO(r)) return false;
//
//	a = gDeterm3(xz, xy, x1, yz, y2, y1, z1, y1, n) / r;
//	b = gDeterm3(x2, xz, x1, xy, yz, y1, x1, z1, n) / r;
//	c = gDeterm3(x2, xy, xz, xy, y2, yz, x1, y1, z1) / r;
//
//	return true;
//}