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

}

#endif