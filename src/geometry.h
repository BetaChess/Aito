#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "vecmath.h"

#include <iterator>


namespace aito
{

///////////////////////////////////////////////// Bounding Boxes

template<typename T>
class Bounds2
{
public:
	Point2<T> p_min, p_max;

public:
	constexpr Bounds2()
	{
		T minNum = std::numeric_limits<T>::min();
		T maxNum = std::numeric_limits<T>::max();
		p_min = Point2<T>(maxNum, maxNum);
		p_max = Point2<T>(minNum, minNum);
	}
	constexpr Bounds2(const Point2<T>& p) : p_min(p), p_max(p) {}
	constexpr Bounds2(const Point2<T>& p1, const Point2<T>& p2)
		: p_min(min(p1, p2)), p_max(max(p1, p2)) {}

	// Operators

	[[nodiscard]] constexpr Point2<T> operator[](size_t i) const
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return p_min;
		return p_max;
	}
	constexpr Point2<T>& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return p_min;
		return p_max;
	}

	// Public Methods
	
	[[nodiscard]] constexpr Point2<T> corner(size_t i) const
	{
		return Point2<T>(
			(*this)[i & 1].x,
			(*this)[i & 2 ? 1 : 0].y
			);
	}

	[[nodiscard]] constexpr Vec2<T> diagonal() const
	{
		return p_max - p_min;
	}

	[[nodiscard]] constexpr T surface_area() const
	{
		return (p_max.x - p_min.x) * (p_max.y - p_min.y);
	}

	[[nodiscard]] constexpr size_t maximum_extent() const
	{
		auto d = diagonal();
		// if x is more than y, return 0. Else return 1.
		return !(d.x > d.y);
	}

	[[nodiscard]] constexpr Point2<T> lerp(const Point2f& t) const
	{
		return Point2<T>(
			aito::lerp(t.x, p_min.x, p_max.x),
			aito::lerp(t.y, p_min.y, p_max.y)
			);
	}

	[[nodiscard]] constexpr Vec2<T> offset(const Point2<T>& p)
	{
		auto o = p - p_min;
		if (p_max.x > p_min.x) o.x /= p_max.x - p_min.x;
		if (p_max.y > p_min.y) o.y /= p_max.y - p_min.y;
		return o;
	}

	void bounding_sphere(Point2<T>* center_out, Float* radius_out) const
	{
		*center_out = (p_min + p_max) / 2;
		*radius_out = inside_bounds(*center_out, *this) ? distance(*center_out, p_max) : 0;
	}
	
};

template<typename T>
[[nodiscard]] constexpr Bounds2<T> bounds_union(const Bounds2<T>& b, const Point2<T>& p)
{
	return Bounds2<T>(
		min(b.p_min, p),
		max(b.p_max, p)
		);
}
template<typename T>
[[nodiscard]] constexpr Bounds2<T> bounds_union(const Bounds2<T>& b1, const Bounds2<T>& b2)
{
	return Bounds2<T>(
		min(b1.p_min, b2.p_min),
		max(b1.p_max, b2.p_max)
		);
}

template<typename T>
[[nodiscard]] constexpr Bounds2<T> bounds_intersect(const Bounds2<T>& b1, const Bounds2<T>& b2)
{
	return Bounds2<T>(
		max(b1.p_min, b2.p_min),
		min(b1.p_max, b2.p_max)
		);
}

template<typename T>
[[nodiscard]] constexpr bool bounds_overlap(const Bounds2<T>& b1, const Bounds2<T>& b2)
{
	const bool x = (b1.p_max.x >= b2.p_min.x) && (b1.p_min.x <= b2.p_max.x);
	const bool y = (b1.p_max.y >= b2.p_min.y) && (b1.p_min.y <= b2.p_max.y);
	return x && y;
}

template<typename T>
[[nodiscard]] constexpr bool inside_bounds(const Point2<T>& p, const Bounds2<T>& b)
{
	const bool x = (p.x >= b.p_min.x) && (p.x <= b.p_max.x);
	const bool y = (p.y >= b.p_min.y) && (p.y <= b.p_max.y);
	return x && y;
}

/// <summary>
/// Checks if a point "p" is inside the bounds "b", but does not consider points on the upper boundary to be inside, unlike the function "inside_bounds".
/// </summary>
template<typename T>
[[nodiscard]] constexpr bool inside_exclusive_bounds(const Point2<T>& p, const Bounds2<T>& b)
{
	const bool x = (p.x >= b.p_min.x) && (p.x < b.p_max.x);
	const bool y = (p.y >= b.p_min.y) && (p.y < b.p_max.y);
	return x && y;
}

template<typename T, typename U>
[[nodiscard]] constexpr Bounds2<T> expanded_bounds(const Bounds2<T>& b, U d)
{
	return Bounds2<T>(
		b.p_min - Vec2<T>(d, d),
		b.p_max + Vec2<T>(d, d)
		);
}

template<typename T>
class Bounds3
{
public:
	Point3<T> p_min, p_max;

public:
	constexpr Bounds3()
	{
		T minNum = std::numeric_limits<T>::min();
		T maxNum = std::numeric_limits<T>::max();
		p_min = Point3<T>(maxNum, maxNum, maxNum);
		p_max = Point3<T>(minNum, minNum, minNum);
	}
	constexpr Bounds3(const Point3<T>& p) : p_min(p), p_max(p) {}
	constexpr Bounds3(const Point3<T>& p1, const Point3<T>& p2)
		: p_min(min(p1, p2)), p_max(max(p1, p2))
	{}

	// Operators

	[[nodiscard]] constexpr Point3<T> operator[](size_t i) const
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return p_min;
		return p_max;
	}
	constexpr Point3<T>& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return p_min;
		return p_max;
	}

	// Public Methods

	[[nodiscard]] constexpr Point3<T> corner(size_t i) const
	{
		return Point3<T>(
			(*this)[i & 1].x,
			(*this)[i & 2 ? 1 : 0].y,
			(*this)[i & 4 ? 1 : 0].z
			);
	}

	[[nodiscard]] constexpr Vec3<T> diagonal() const
	{
		return p_max - p_min;
	}


	// Public Methods

	[[nodiscard]] constexpr T surface_area() const
	{
		auto d = diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	[[nodiscard]] constexpr T volume() const
	{
		auto d = diagonal();
		return d.x * d.y * d.z;
	}

	[[nodiscard]] constexpr size_t maximum_extent() const
	{
		auto d = diagonal();
		if (d.x > d.y && d.x > d.z)
			return 0;
		else if (d.y > d.z)
			return 1;
		else 
			return 2;
	}

	[[nodiscard]] constexpr Point3<T> lerp(const Point3f& t) const
	{
		return Point3<T>(
			aito::lerp(t.x, p_min.x, p_max.x),
			aito::lerp(t.y, p_min.y, p_max.y),
			aito::lerp(t.z, p_min.z, p_max.z)
			);
	}

	[[nodiscard]] constexpr Vec3<T> offset(const Point3<T>& p)
	{
		auto o = p - p_min;
		if (p_max.x > p_min.x) o.x /= p_max.x - p_min.x;
		if (p_max.y > p_min.y) o.y /= p_max.y - p_min.y;
		if (p_max.z > p_min.z) o.z /= p_max.z - p_min.z;
		return o;
	}

	void bounding_sphere(Point3<T>* center_out, Float* radius_out) const
	{
		*center_out = (p_min + p_max) / 2;
		*radius_out = inside_bounds(*center_out, *this) ? distance(*center_out, p_max) : 0;
	}

};

template<typename T>
[[nodiscard]] constexpr Bounds3<T> bounds_union(const Bounds3<T>& b, const Point3<T>& p)
{
	return Bounds3<T>(
		min(b.p_min, p),
		max(b.p_max, p)
		);
}
template<typename T>
[[nodiscard]] constexpr Bounds3<T> bounds_union(const Bounds3<T>& b1, const Bounds3<T>& b2)
{
	return Bounds3<T>(
		min(b1.p_min, b2.p_min),
		max(b1.p_max, b2.p_max)
		);
}

template<typename T>
[[nodiscard]] constexpr Bounds3<T> bounds_intersect(const Bounds3<T>& b1, const Bounds3<T>& b2)
{
	return Bounds3<T>(
		max(b1.p_min, b2.p_min),
		min(b1.p_max, b2.p_max)
		);
}

template<typename T>
[[nodiscard]] constexpr bool bounds_overlap(const Bounds3<T>& b1, const Bounds3<T>& b2)
{
	const bool x = (b1.p_max.x >= b2.p_min.x) && (b1.p_min.x <= b2.p_max.x);
	const bool y = (b1.p_max.y >= b2.p_min.y) && (b1.p_min.y <= b2.p_max.y);
	const bool z = (b1.p_max.z >= b2.p_min.z) && (b1.p_min.z <= b2.p_max.z);
	return x && y && z;
}

template<typename T>
[[nodiscard]] constexpr bool inside_bounds(const Point3<T>& p, const Bounds3<T>& b)
{
	const bool x = (p.x >= b.p_min.x) && (p.x <= b.p_max.x);
	const bool y = (p.y >= b.p_min.y) && (p.y <= b.p_max.y);
	const bool z = (p.z >= b.p_min.z) && (p.z <= b.p_max.z);
	return x && y && z;
}

/// <summary>
/// Checks if a point "p" is inside the bounds "b", but does not consider points on the upper boundary to be inside, unlike the function "inside_bounds".
/// </summary>
template<typename T>
[[nodiscard]] constexpr bool inside_exclusive_bounds(const Point3<T>& p, const Bounds3<T>& b)
{
	const bool x = (p.x >= b.p_min.x) && (p.x < b.p_max.x);
	const bool y = (p.y >= b.p_min.y) && (p.y < b.p_max.y);
	const bool z = (p.z >= b.p_min.z) && (p.z < b.p_max.z);
	return x && y && z;
}

template<typename T, typename U>
[[nodiscard]] constexpr Bounds3<T> expanded_bounds(const Bounds3<T>& b, U d)
{
	return Bounds3<T>(
		b.p_min - Vec3<T>(d, d, d),
		b.p_max + Vec3<T>(d, d, d)
		);
}

/// Typedefs

typedef Bounds2<int>	Bounds2i;
typedef Bounds2<Float>	Bounds2f;
typedef Bounds3<int>	Bounds3i;
typedef Bounds3<Float>	Bounds3f;

// Iterator begin and end definitions

class Bounds2iIterator : public std::forward_iterator_tag
{
public:
	Bounds2iIterator(const Bounds2i& b, const Point2i& pt)
		: p(pt), bounds(&b)
	{}
	Bounds2iIterator operator++()
	{
		advance();
		return *this;
	}
	Bounds2iIterator operator++(int)
	{
		Bounds2iIterator old = *this;
		advance();
		return old;
	}
	bool operator==(const Bounds2iIterator& bi) const
	{
		return p == bi.p && bounds == bi.bounds;
	}
	bool operator!=(const Bounds2iIterator& bi) const
	{
		return p != bi.p || bounds != bi.bounds;
	}

	Point2i operator*() const { return p; }

private:
	void advance()
	{
		++p.x;
		if (p.x == bounds->p_max.x)
		{
			p.x = bounds->p_min.x;
			++p.y;
		}
	}
	Point2i p;
	const Bounds2i* bounds;
};

inline Bounds2iIterator begin(const Bounds2i& b)
{
	return Bounds2iIterator(b, b.p_min);
}

inline Bounds2iIterator end(const Bounds2i& b)
{
	// Normally, the ending point is at the minimum x value and one past
	// the last valid y value.
	Point2i pEnd(b.p_min.x, b.p_max.y);
	// However, if the bounds are degenerate, override the end point to
	// equal the start point so that any attempt to iterate over the bounds
	// exits out immediately.
	if (b.p_min.x >= b.p_max.x || b.p_min.y >= b.p_max.y)
		pEnd = b.p_min;
	return Bounds2iIterator(b, pEnd);
}


///////////////////////////////////////////////// Transform


class Transform
{
private:
	Mat4 m_, m_inv_;

public:
	constexpr Transform() = default;
	constexpr Transform(const Mat4& m)
		: m_(m), m_inv_(m.get_inverse())
	{}
	constexpr Transform(const Mat4& m, const Mat4& m_inverse)
		: m_(m), m_inv_(m_inverse)
	{}

	// Public methods

	[[nodiscard]] constexpr bool is_identity() const
	{
		return (m_.m[0][0] == 1.f && m_.m[0][1] == 0.f && m_.m[0][2] == 0.f &&
				m_.m[0][3] == 0.f && m_.m[1][0] == 0.f && m_.m[1][1] == 1.f &&
				m_.m[1][2] == 0.f && m_.m[1][3] == 0.f && m_.m[2][0] == 0.f &&
				m_.m[2][1] == 0.f && m_.m[2][2] == 1.f && m_.m[2][3] == 0.f &&
				m_.m[3][0] == 0.f && m_.m[3][1] == 0.f && m_.m[3][2] == 0.f &&
				m_.m[3][3] == 1.f);
	}
	[[nodiscard]] constexpr bool has_scale() const
	{
		float la2 = (*this)(Vec3f(1, 0, 0)).length_squared();
		float lb2 = (*this)(Vec3f(0, 1, 0)).length_squared();
		float lc2 = (*this)(Vec3f(0, 0, 1)).length_squared();
#define NOT_ONE(x) ((x) < 0.999f || (x) > 1.001f)
		return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
	}
	[[nodiscard]] constexpr bool swaps_handedness() const
	{
		Float det =
			m_.m[0][0] * (m_.m[1][1] * m_.m[2][2] - m_.m[1][2] * m_.m[2][1]) -
			m_.m[0][1] * (m_.m[1][0] * m_.m[2][2] - m_.m[1][2] * m_.m[2][0]) + 
			m_.m[0][2] * (m_.m[1][0] * m_.m[2][1] - m_.m[1][1] * m_.m[2][0]);
		return det < 0;
	}

	[[nodiscard]] constexpr Transform get_inverse() const
	{
		return Transform(m_inv_, m_);
	}
	[[nodiscard]] constexpr Transform get_transpose() const
	{
		return Transform(m_.get_transpose(), m_inv_.get_transpose());
	}

	// Operators

	[[nodiscard]] inline bool operator==(const Transform& rhs) const
	{
		return m_ == rhs.m_; // No need to check if the inverse is also equal, since it should always be if the matrix is.
	}
	[[nodiscard]] inline bool operator!=(const Transform& rhs) const
	{
		return m_ != rhs.m_;
	}

	template<typename T>
	[[nodiscard]] constexpr Point3<T> operator()(const Point3<T>& p) const
	{
		T xp = m_.m[0][0] * p.x + m_.m[0][1] * p.y + m_.m[0][2] * p.z + m_.m[0][3];
		T yp = m_.m[1][0] * p.x + m_.m[1][1] * p.y + m_.m[1][2] * p.z + m_.m[1][3];
		T zp = m_.m[2][0] * p.x + m_.m[2][1] * p.y + m_.m[2][2] * p.z + m_.m[2][3];
		T wp = m_.m[3][0] * p.x + m_.m[3][1] * p.y + m_.m[3][2] * p.z + m_.m[3][3];
		if (wp == 1)
			return	Point3<T>(xp, yp, wp);
		return		Point3<T>(xp, yp, wp) / wp;
	}
	template<typename T>
	[[nodiscard]] constexpr Vec3<T> operator()(const Vec3<T>& v) const
	{
		return Vec3<T>(
			m_.m[0][0] * v.x + m_.m[0][1] * v.y + m_.m[0][2] * v.z,
			m_.m[1][0] * v.x + m_.m[1][1] * v.y + m_.m[1][2] * v.z,
			m_.m[2][0] * v.x + m_.m[2][1] * v.y + m_.m[2][2] * v.z);
	}
	template<typename T>
	[[nodiscard]] constexpr Normal3<T> operator()(const Normal3<T>& n) const
	{
		return Normal3<T>(
			m_.m[0][0] * n.x + m_.m[1][0] * n.y + m_.m[2][0] * n.z,
			m_.m[0][1] * n.x + m_.m[1][1] * n.y + m_.m[2][1] * n.z,
			m_.m[0][2] * n.x + m_.m[1][2] * n.y + m_.m[2][2] * n.z);
	}
	[[nodiscard]] constexpr Bounds3f operator()(const Bounds3f& b) const
	{
		// TODO: Do this, but more efficiently.
		const Transform& M = *this;
		Bounds3f ret(M(Point3f(b.p_min.x, b.p_min.y, b.p_min.z)));
		ret = bounds_union(ret, M(Point3f(b.p_max.x, b.p_min.y, b.p_min.z)));
		ret = bounds_union(ret, M(Point3f(b.p_min.x, b.p_max.y, b.p_min.z)));
		ret = bounds_union(ret, M(Point3f(b.p_min.x, b.p_min.y, b.p_max.z)));
		ret = bounds_union(ret, M(Point3f(b.p_min.x, b.p_max.y, b.p_max.z)));
		ret = bounds_union(ret, M(Point3f(b.p_max.x, b.p_max.y, b.p_min.z)));
		ret = bounds_union(ret, M(Point3f(b.p_max.x, b.p_min.y, b.p_max.z)));
		ret = bounds_union(ret, M(Point3f(b.p_max.x, b.p_max.y, b.p_max.z)));
		return ret;
	}
	[[nodiscard]] constexpr Ray operator()(const Ray& r) const
	{
		Vec3f oError;
		Point3f o = (*this)(r.o, &oError);
		Vec3f d = (*this)(r.d);
		// Offset ray origin to edge of error bounds and compute _tMax_
		Float lengthSquared = d.length_squared();
		Float t_max = r.t_max;
		if (lengthSquared > 0)
		{
			Float dt = dot(abs(d), oError) / lengthSquared;
			o += d * dt;
			t_max -= dt;
		}
		return Ray(o, d, t_max, r.time, r.medium);
	}
	template <typename T>
	[[nodiscard]] constexpr Point3<T> operator()(const Point3<T>& p, Vec3<T>* pError) const
	{
		// Compute transformed coordinates from point _pt_
		T xp = (m_.m[0][0] * p.x + m_.m[0][1] * p.y) + (m_.m[0][2] * p.z + m_.m[0][3]);
		T yp = (m_.m[1][0] * p.x + m_.m[1][1] * p.y) + (m_.m[1][2] * p.z + m_.m[1][3]);
		T zp = (m_.m[2][0] * p.x + m_.m[2][1] * p.y) + (m_.m[2][2] * p.z + m_.m[2][3]);
		T wp = (m_.m[3][0] * p.x + m_.m[3][1] * p.y) + (m_.m[3][2] * p.z + m_.m[3][3]);

		// Compute absolute error for transformed point
		T xAbsSum = (std::abs(m_.m[0][0] * p.x) + std::abs(m_.m[0][1] * p.y) +
					 std::abs(m_.m[0][2] * p.z) + std::abs(m_.m[0][3]));
		T yAbsSum = (std::abs(m_.m[1][0] * p.x) + std::abs(m_.m[1][1] * p.y) +
					 std::abs(m_.m[1][2] * p.z) + std::abs(m_.m[1][3]));
		T zAbsSum = (std::abs(m_.m[2][0] * p.x) + std::abs(m_.m[2][1] * p.y) +
					 std::abs(m_.m[2][2] * p.z) + std::abs(m_.m[2][3]));
		*pError = gamma(3) * Vec3<T>(xAbsSum, yAbsSum, zAbsSum);
		
		if (wp == 1)
			return Point3<T>(xp, yp, zp);
		else
			return Point3<T>(xp, yp, zp) / wp;
	}

	[[nodiscard]] constexpr Transform operator*(const Transform& t2) const
	{
		return Transform(
			m_.m * t2.m_.m,
			t2.m_inv_.m * m_inv_.m);
	}

	// Public static methods

	[[nodiscard]] static constexpr Transform make_translation(const Vec3f& d)
	{
		Mat4 m(
			1, 0, 0, d.x,
			0, 1, 0, d.y,
			0, 0, 1, d.z,
			0, 0, 0, 1);
		Mat4 m_inv(
			1, 0, 0, -d.x,
			0, 1, 0, -d.y,
			0, 0, 1, -d.z,
			0, 0, 0, 1);
		return Transform(m, m_inv);
	}
	[[nodiscard]] static constexpr Transform make_scale(Float x, Float y, Float z)
	{
		Mat4 m(
			x, 0, 0, 0,
			0, y, 0, 0,
			0, 0, z, 0,
			0, 0, 0, 1);
		Mat4 m_inv(
			1/x,  0,   0, 0,
			0,  1/y,   0, 0,
			0,    0, 1/z, 0,
			0,    0,   0, 1);
		return Transform(m, m_inv);
	}
	[[nodiscard]] static inline Transform make_rotatex(Float theta)
	{
		Float sin_t = std::sin(theta);
		Float cos_t = std::cos(theta);
		Mat4 m(1, 0, 0, 0,
			   0, cos_t, -sin_t, 0,
			   0, sin_t, cos_t, 0,
			   0, 0, 0, 1);
		return Transform(m, m.get_transpose());
	}
	[[nodiscard]] static inline Transform make_rotatey(Float theta)
	{
		Float sin_t = std::sin(theta);
		Float cos_t = std::cos(theta);
		Mat4 m(cos_t, 0, sin_t, 0,
			   0, 1, 0, 0,
			   -sin_t, 0, cos_t, 0,
			   0, 0, 0, 1);
		return Transform(m, m.get_transpose());
	}
	[[nodiscard]] static inline Transform make_rotatez(Float theta)
	{
		Float sin_t = glm::sin(theta);
		Float cos_t = std::cos(theta);
		Mat4 m(cos_t, -sin_t, 0, 0,
			   sin_t, cos_t, 0, 0,
			   0, 0, 1, 0,
			   0, 0, 0, 1);
		return Transform(m, m.get_transpose());
	}
	[[nodiscard]] static inline Transform make_rotate(Float theta, const Vec3f& axis)
	{
		Vec3f a = axis.normalized();
		Float sin_t = glm::sin(theta);
		Float cos_t = std::cos(theta);
		Mat4 m;
		m.m[0][0] = a.x * a.x + (1 - a.x * a.x) * cos_t;
		m.m[0][1] = a.x * a.y * (1 - cos_t) - a.z * sin_t;
		m.m[0][2] = a.x * a.z * (1 - cos_t) + a.y * sin_t;
		m.m[0][3] = 0;

		// Compute rotations of second and third basis vectors
		m.m[1][0] = a.x * a.y * (1.f - cos_t) + a.z * sin_t;
		m.m[1][1] = a.y * a.y + (1.f - a.y * a.y) * cos_t;
		m.m[1][2] = a.y * a.z * (1.f - cos_t) - a.x * sin_t;
		m.m[1][3] = 0;

		m.m[2][0] = a.x * a.z * (1.f - cos_t) - a.y * sin_t;
		m.m[2][1] = a.y * a.z * (1.f - cos_t) + a.x * sin_t;
		m.m[2][2] = a.z * a.z + (1.f - a.z * a.z) * cos_t;
		m.m[2][3] = 0;

		return Transform(m, m.get_transpose());
	}
	[[nodiscard]] static constexpr Transform make_lookat(const Point3f& pos, const Point3f& look, const Vec3f& up)
	{
		Mat4 m;

		m.m[0][3] = pos.x;
		m.m[1][3] = pos.y;
		m.m[2][3] = pos.z;
		m.m[3][3] = 1;

		Vec3f dir = (look - pos).normalized();
		Vec3f left = cross(up.normalized(), dir);
		if (left.length_squared() == 0)
		{
			AITO_ERROR(
				"\"up\" vector ({}, {}, {}) and viewing direction ({}, {}, {}) "
				"passed to make_lookat are pointing in the same direction.  Using "
				"the identity transformation.",
				up.x, up.y, up.z, dir.x, dir.y, dir.z);
			return Transform();
		}
		left.normalize();
		Vec3f new_up = cross(dir, left);

		m.m[0][0] = left.x;
		m.m[1][0] = left.y;
		m.m[2][0] = left.z;
		m.m[3][0] = 0;
		m.m[0][1] = new_up.x;
		m.m[1][1] = new_up.y;
		m.m[2][1] = new_up.z;
		m.m[3][1] = 0;
		m.m[0][2] = dir.x;
		m.m[1][2] = dir.y;
		m.m[2][2] = dir.z;
		m.m[3][2] = 0;

		return Transform(m.get_inverse(), m);
	}


};

}

#endif