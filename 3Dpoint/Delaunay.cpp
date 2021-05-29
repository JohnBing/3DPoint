#include"Delaunay.h"

//vector2
namespace dt {

	template<typename T>
	Vector2<T>::Vector2(const T vx, const T vy) :
		x(vx), y(vy)
	{}

	template<typename T>
	T
		Vector2<T>::dist2(const Vector2<T> &v) const
	{
		const T dx = x - v.x;
		const T dy = y - v.y;
		return dx * dx + dy * dy;
	}

	template<>
	float
		Vector2<float>::dist(const Vector2<float> &v) const
	{
		return hypotf(x - v.x, y - v.y);//求平方和的平方根
	}

	template<>
	double
		Vector2<double>::dist(const Vector2<double> &v) const
	{
		return hypot(x - v.x, y - v.y);
	}

	template<typename T>
	T
		Vector2<T>::norm2() const
	{
		return x * x + y * y;
	}

	template<typename T>
	bool
		Vector2<T>::operator ==(const Vector2<T> &v) const
	{
		return (this->x == v.x) && (this->y == v.y);
	}

	template<typename U>
	std::ostream &
		operator <<(std::ostream &str, const Vector2<U> &v)
	{
		return str << "Point x: " << v.x << " y: " << v.y;
	}

	template struct Vector2<float>;
	template struct Vector2<double>;

} // namespace dt

//edge
namespace dt {

	template<typename T>
	Edge<T>::Edge(const VertexType &v1, const VertexType &v2) :
		v(&v1), w(&v2)
	{}

	template<typename T>
	bool
		Edge<T>::operator ==(const Edge<T> &e) const
	{
		return (*(this->v) == *e.v && *(this->w) == *e.w) ||
			(*(this->v) == *e.w && *(this->w) == *e.v);
	}

	template<typename U>
	std::ostream&
		operator <<(std::ostream &str, const Edge<U> &e)
	{
		return str << "Edge " << *e.v << ", " << *e.w;
	}

	template struct Edge<float>;
	template struct Edge<double>;

} // namespace dt

//triangles
namespace dt {

	template<typename T>
	Triangle<T>::Triangle(const VertexType &v1, const VertexType &v2, const VertexType &v3) :
		a(&v1), b(&v2), c(&v3), isBad(false)
	{}

	template<typename T>
	bool
		Triangle<T>::containsVertex(const VertexType &v) const
	{
		// return p1 == v || p2 == v || p3 == v;
		return almost_equal(*a, v) || almost_equal(*b, v) || almost_equal(*c, v);
	}

	template<typename T>
	bool
		Triangle<T>::circumCircleContains(const VertexType &v) const
	{
		const T ab = a->norm2();
		const T cd = b->norm2();
		const T ef = c->norm2();

		const T ax = a->x;
		const T ay = a->y;
		const T bx = b->x;
		const T by = b->y;
		const T cx = c->x;
		const T cy = c->y;

		const T circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
		const T circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

		const VertexType circum(circum_x / 2, circum_y / 2);
		const T circum_radius = a->dist2(circum);
		const T dist = v.dist2(circum);
		return dist <= circum_radius;
	}

	template<typename T>
	bool
		Triangle<T>::operator ==(const Triangle &t) const
	{
		return	(*this->a == *t.a || *this->a == *t.b || *this->a == *t.c) &&
			(*this->b == *t.a || *this->b == *t.b || *this->b == *t.c) &&
			(*this->c == *t.a || *this->c == *t.b || *this->c == *t.c);
	}

	template<typename U>
	std::ostream&
		operator <<(std::ostream &str, const Triangle<U> &t)
	{
		return str << "Triangle:" << "\n\t" <<
			*t.a << "\n\t" <<
			*t.b << "\n\t" <<
			*t.c << '\n';
	}

	template struct Triangle<float>;
	template struct Triangle<double>;

} // namespace dt

//delaunay
namespace dt {

	template<typename T>
	const std::vector<typename Delaunay<T>::TriangleType>&
		Delaunay<T>::triangulate(std::vector<VertexType> &vertices)
	{
		// Store the vertices locally
		_vertices = vertices;

		// 求大三角形
		T minX = vertices[0].x;
		T minY = vertices[0].y;
		T maxX = minX;
		T maxY = minY;

		for (std::size_t i = 0; i < vertices.size(); ++i)
		{
			if (vertices[i].x < minX) minX = vertices[i].x;
			if (vertices[i].y < minY) minY = vertices[i].y;
			if (vertices[i].x > maxX) maxX = vertices[i].x;
			if (vertices[i].y > maxY) maxY = vertices[i].y;
		}

		const T dx = maxX - minX;
		const T dy = maxY - minY;
		const T deltaMax = std::max(dx, dy);
		const T midx = (minX + maxX) / 2;
		const T midy = (minY + maxY) / 2;

		const VertexType p1(midx - 20 * deltaMax, midy - deltaMax);
		const VertexType p2(midx, midy + 20 * deltaMax);
		const VertexType p3(midx + 20 * deltaMax, midy - deltaMax);

		// Create a list of triangles, and add the supertriangle in it
		_triangles.push_back(TriangleType(p1, p2, p3));

		for (auto p = begin(vertices); p != end(vertices); p++)
		{
			std::vector<EdgeType> polygon;

			for (auto & t : _triangles)
			{
				if (t.circumCircleContains(*p))
				{
					t.isBad = true;
					polygon.push_back(Edge<T>{*t.a, *t.b});
					polygon.push_back(Edge<T>{*t.b, *t.c});
					polygon.push_back(Edge<T>{*t.c, *t.a});
				}
			}
			//去除不符合的Delaunay三角形
			_triangles.erase(std::remove_if(begin(_triangles), end(_triangles), [](TriangleType &t) {
				return t.isBad;
			}), end(_triangles));
			//对边去重（只要发生重复都去掉）
			for (auto e1 = begin(polygon); e1 != end(polygon); ++e1)
			{
				for (auto e2 = e1 + 1; e2 != end(polygon); ++e2)
				{
					if (almost_equal(*e1, *e2))
					{
						e1->isBad = true;
						e2->isBad = true;
					}
				}
			}

			polygon.erase(std::remove_if(begin(polygon), end(polygon), [](EdgeType &e) {
				return e.isBad;
			}), end(polygon));
			//构造新的三角形进入下一轮
			for (const auto e : polygon)
				_triangles.push_back(TriangleType(*e.v, *e.w, *p));

		}

		//去除与大三角形有关的Delaunay三角形
		_triangles.erase(std::remove_if(begin(_triangles), end(_triangles), [p1, p2, p3](TriangleType &t) {
			return t.containsVertex(p1) || t.containsVertex(p2) || t.containsVertex(p3);
		}), end(_triangles));

		//压入Delaunay三角形的边
		for (const auto t : _triangles)
		{
			_edges.push_back(Edge<T>{*t.a, *t.b});
			_edges.push_back(Edge<T>{*t.b, *t.c});
			_edges.push_back(Edge<T>{*t.c, *t.a});
		}

		return _triangles;
	}

	template<typename T>
	const std::vector<typename Delaunay<T>::TriangleType>&
		Delaunay<T>::getTriangles() const
	{
		return _triangles;
	}

	template<typename T>
	const std::vector<typename Delaunay<T>::EdgeType>&
		Delaunay<T>::getEdges() const
	{
		return _edges;
	}

	template<typename T>
	const std::vector<typename Delaunay<T>::VertexType>&
		Delaunay<T>::getVertices() const
	{
		return _vertices;
	}

	template class Delaunay<float>;
	template class Delaunay<double>;

} // namespace dt