#ifndef VECMATH_H
#define VECMATH_H

#include "aito.h"

#include <algorithm>
#include <cmath>
#include <cassert>

namespace aito
{

template <typename T>
inline bool is_NaN(const T x)
{
	return std::isnan(x);
}
template <>
inline bool is_NaN(const int x)
{
	return false;
}

///////////////////////////////////////////////// Vectors

template<typename T>
class Vec2
{
public:
	T x = 0, y = 0;

public:

	constexpr Vec2() = default;
	constexpr Vec2(T x, T y) : x(x), y(y) 
	{
		assert(!has_NaNs());
	};

	// State checking

	[[nodiscard]] constexpr bool has_NaNs() const
	{
		return is_NaN(x) || is_NaN(y);
	}

	// Mathematical operations

	[[nodiscard]] constexpr Float length_squared() const
	{
		return x * x + y * y;
	}
	[[nodiscard]] constexpr Float length() const
	{
		return std::sqrt(length_squared());
	}

	[[nodiscard]] constexpr Vec2<T> normalized() const
	{
		Float L = length();
		return *this / L;
	}
	[[nodiscard]] constexpr Vec2<T>& normalize()
	{
		Float L = length();
		*this /= L;
		return *this;
	}

	// Operators

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 1 && "Out of bounds vector access!");
		if (i == 0) return x;
		return y;
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds vector access!");
		if (i == 0) return x;
		return y;
	}

	[[nodiscard]] constexpr bool operator==(const Vec2<T>& v) const { return x == v.x && y == v.y; };
	[[nodiscard]] constexpr bool operator!=(const Vec2<T>& v) const { return x != v.x || y != v.y; };

	// Arithmetic operators
	
	[[nodiscard]] constexpr Vec2<T> operator+(const Vec2<T>& rhs) const
	{
		return Vec2<T>(x + rhs.x, y + rhs.y);
	}
	constexpr Vec2<T>& operator+=(const Vec2<T>& rhs)
	{
		x += rhs.x; y += rhs.y;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator-(const Vec2<T>& rhs) const
	{
		return Vec2<T>(x - rhs.x, y - rhs.y);
	}
	constexpr Vec2<T>& operator-=(const Vec2<T>& rhs)
	{
		x -= rhs.x; y -= rhs.y;
		return *this;
	}

	[[nodiscard]] constexpr Vec2<T> operator*(T s) const
	{
		return Vec2<T>(x * s, y * s);
	}
	constexpr Vec2<T>& operator*=(T s)
	{
		x *= s; y *= s;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Vec2<T>(x * inv, y * inv);
	}
	constexpr Vec2<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		x *= inv; y *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Vec2<T> operator-() const
	{
		return Vec2<T>(-x, -y);
	}

};

template<typename T>
[[nodiscard]] constexpr Vec2<T> abs(const Vec2<T>& v)
{
	return Vec2<T>(std::abs(v.x), std::abs(v.y));
}

template<typename T>
[[nodiscard]] constexpr T dot(const Vec2<T>& v, const Vec2<T>& w)
{
	return v.x * w.x + v.y * w.y;
}

template<typename T>
[[nodiscard]] constexpr T abs_dot(const Vec2<T>& v, const Vec2<T>& w)
{
	return std::abs(dot(v, w));
}

template<typename T>
[[nodiscard]] constexpr Vec2<T> normalized(const Vec2<T>& v)
{
	Float L = v.length();
	return v / L;
}

// Operators related to Vec2

template<typename T>
[[nodiscard]] constexpr Vec2<T> operator*(T s, const Vec2<T>& v)
{
	return v * s;
}

// Forward declare Normal3
template<typename T>
class Normal3;

template<typename T>
class Vec3
{
public:
	T x = 0, y = 0, z = 0;

public:

	constexpr Vec3() = default;
	constexpr Vec3(T x, T y, T z) : x(x), y(y), z(z)
	{
		assert(!has_NaNs());
	};
	constexpr Vec3(const Normal3<T>& n);

	// State checking

	[[nodiscard]] constexpr bool has_NaNs() const
	{
		return is_NaN(x) || is_NaN(y) || is_NaN(z);
	}

	// Mathematical operations

	[[nodiscard]] constexpr Float length_squared() const
	{
		return x * x + y * y + z * z;
	}
	[[nodiscard]] constexpr Float length() const
	{
		return std::sqrt(length_squared());
	}

	[[nodiscard]] constexpr Vec3<T> normalized() const
	{
		Float L = length();
		return *this / L;
	}
	[[nodiscard]] constexpr Vec3<T>& normalize()
	{
		Float L = length();
		*this /= L;
		return *this;
	}

	// Operators

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}

	[[nodiscard]] constexpr bool operator==(const Vec3<T>& v) const
	{
		return x == v.x && y == v.y && z == v.z;
	}
	[[nodiscard]] constexpr bool operator!=(const Vec3<T>& v) const
	{
		return x != v.x || y != v.y || z != v.z;
	}

	// Arithmetic operators

	[[nodiscard]] constexpr Vec3<T> operator+(const Vec3<T>& rhs) const
	{
		return Vec3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}
	constexpr Vec3<T>& operator+=(const Vec3<T>& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator-(const Vec3<T>& rhs) const
	{
		return Vec3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	constexpr Vec3<T>& operator-=(const Vec3<T>& rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
		return *this;
	}

	[[nodiscard]] constexpr Vec3<T> operator*(T s) const
	{
		return Vec3<T>(x * s, y * s, z * s);
	}
	constexpr Vec3<T>& operator*=(T s)
	{
		x *= s; y *= s; z *= s;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Vec3<T>(x * inv, y * inv, z * inv);
	}
	constexpr Vec3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		x *= inv; y *= inv; z *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Vec3<T> operator-() const
	{
		return Vec3<T>(-x, -y, -z);
	}
};

template<typename T>
[[nodiscard]] constexpr Vec3<T> abs(const Vec3<T>& v)
{
	return Vec3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template<typename T>
[[nodiscard]] constexpr T dot(const Vec3<T>& v, const Vec3<T>& w)
{
	return v.x * w.x + v.y * w.y + v.z * w.z;
}

template<typename T>
[[nodiscard]] constexpr T abs_dot(const Vec3<T>& v, const Vec3<T>& w)
{
	return std::abs(dot(v, w));
}

template<typename T>
[[nodiscard]] constexpr Vec3<T> normalized(const Vec3<T>& v)
{
	Float L = v.length();
	return v / L;
}

template<typename T>
[[nodiscard]] constexpr Vec3<T> cross(const Vec3<T>& v, const Vec3<T>& w)
{
	const double vx = v.x, vy = v.y, vz = v.z;
	const double wx = w.x, wy = w.y, wz = w.z;
	return Vec3<T>(
		(vy * wz) - (vz * wy),
		(vz * wx) - (vx * wz),
		(vx * wy) - (vy * wx)
		);
}

/// <summary>
/// Creates a coordinate system from a single vector. Assumes the vector is normalized.
/// </summary>
/// <param name="v1">: Basis vector</param>
/// <param name="v2_out">: First output vector</param>
/// <param name="v3_out">: Second output vector</param>
template<typename T>
constexpr void create_coordinate_system(const Vec3<T>& v1, Vec3<T>* v2_out, Vec3<T>* v3_out)
{
	if (std::abs(v1.x) > std::abs(v1.y))
		*v2_out = Vector3<T>(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
	else
		*v2_out = Vector3<T>(0, v1.z, v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
	*v3_out = cross(v1, *v2_out);
}

// Operators related to Vec3

template<typename T>
[[nodiscard]] constexpr Vec3<T> operator*(T s, const Vec3<T>& v)
{
	return v * s;
}


/// Typedefs

typedef Vec2<int>	Vec2i;
typedef Vec2<Float> Vec2f;
typedef Vec3<int>	Vec3i;
typedef Vec3<Float> Vec3f;

///////////////////////////////////////////////// Points

template<typename T>
class Point3
{
public:
	T x, y, z;
	
public:
	constexpr Point3() = default;
	constexpr Point3(T x, T y, T z) : x(x), y(y), z(z)
	{
		assert(!has_NaNs());
	};
	template<typename U>
	constexpr explicit Point3(const Point3<U>& p)
		: x(static_cast<U>(p.x)), y(static_cast<U>(p.y)), z(static_cast<U>(p.z))
	{
		assert(!has_NaNs());
	};

	template<typename U>
	explicit operator Vec3<U>() const
	{
		return Vec3<U>(x, y, z);
	}

	// State checking

	[[nodiscard]] constexpr bool has_NaNs() const
	{
		return is_NaN(x) || is_NaN(y) || is_NaN(z);
	}

	// Operators

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}

	[[nodiscard]] constexpr bool operator==(const Point3<T>& p) const
	{
		return x == p.x && y == p.y && z == p.z;
	}
	[[nodiscard]] constexpr bool operator!=(const Point3<T>& p) const
	{
		return x != p.x || y != p.y || z != p.z;
	}

	// Arithmetic operators

	[[nodiscard]] constexpr Point3<T> operator+(const Vec3<T>& v) const
	{
		return Point3<T>(x + v.x, y + v.y, z + v.z);
	}
	constexpr Point3<T>& operator+=(const Vec3<T>& v)
	{
		x += v.x; y += v.y; z += v.z;
		return *this;
	}
	[[nodiscard]] constexpr Point3<T> operator-(const Vec3<T>& v) const
	{
		return Point3<T>(x - v.x, y - v.y, z - v.z);
	}
	constexpr Point3<T>& operator-=(const Vec3<T>& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator-(const Point3<T>& p) const
	{
		return Vec3<T>(x - p.x, y - p.y, z - p.z);
	}

	[[nodiscard]] constexpr Point3<T> operator+(const Point3<T>& rhs) const
	{
		return Point3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}
	constexpr Point3<T>& operator+=(const Point3<T>& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}
	constexpr Point3<T>& operator-=(const Point3<T>& rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
		return *this;
	}

	[[nodiscard]] constexpr Point3<T> operator*(T s) const
	{
		return Point3<T>(x * s, y * s, z * s);
	}
	constexpr Point3<T>& operator*=(T s)
	{
		x *= s; y *= s; z *= s;
		return *this;
	}
	[[nodiscard]] constexpr Point3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Point3<T>(x * inv, y * inv, z * inv);
	}
	constexpr Point3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		x *= inv; y *= inv; z *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Point3<T> operator-() const
	{
		return Point3<T>(-x, -y, -z);
	}

};

template<typename T>
[[nodiscard]] constexpr Float distance_squared(const Point3<T>& p0, const Point3<T>& p1)
{
	return (p0 - p1).length_squared();
}
template<typename T>
[[nodiscard]] constexpr Float distance(const Point3<T>& p0, const Point3<T>& p1)
{
	return (p0 - p1).length();
}

template<typename T>
[[nodiscard]] constexpr Point3<T> lerp(Float t, const Point3<T>& p0, const Point3<T>& p1)
{
	return (1 - t) * p0 + t * p1;
}

/// <summary>
/// Computes the component-wise minimum of the two points.
/// </summary>
template<typename T>
[[nodiscard]] constexpr Point3<T> min(const Point3<T>& p0, const Point3<T>& p1)
{
	return Point3<T>(
		std::min(p0.x, p1.x),
		std::min(p0.y, p1.y),
		std::min(p0.z, p1.z)
		);
}
/// <summary>
/// Computes the component-wise maximum of the two points.
/// </summary>
template<typename T>
[[nodiscard]] constexpr Point3<T> max(const Point3<T>& p0, const Point3<T>& p1)
{
	return Point3<T>(
		std::max(p0.x, p1.x),
		std::max(p0.y, p1.y),
		std::max(p0.z, p1.z)
		);
}

template<typename T>
[[nodiscard]] constexpr Point3<T> floor(const Point3<T>& p)
{
	return Point3<T>(std::floor(p.x), std::floor(p.y), std::floor(p.z));
}
template<typename T>
[[nodiscard]] constexpr Point3<T> ceil(const Point3<T>& p)
{
	return Point3<T>(std::ceil(p.x), std::ceil(p.y), std::ceil(p.z));
}
template<typename T>
[[nodiscard]] constexpr Point3<T> abs(const Point3<T>& p)
{
	return Point3<T>(std::abs(p.x), std::abs(p.y), std::abs(p.z));
}

template<typename T>
[[nodiscard]] constexpr Point3<T> permute(const Point3<T>& p, size_t x, size_t y, size_t z)
{
	return Point3<T>(p[x], p[y], p[z]);
}

// Operators related to p1

template<typename T>
[[nodiscard]] constexpr Point3<T> operator*(T s, const Point3<T>& v)
{
	return v * s;
}



template<typename T>
class Point2
{
public:
	T x, y;

public:

	constexpr Point2() = default;
	constexpr Point2(T x, T y) : x(x), y(y)
	{
		assert(!has_NaNs());
	};
	template<typename U>
	constexpr explicit Point2(const Point2<U>& p) 
		: x(static_cast<U>(p.x)), y(static_cast<U>(p.y))
	{
		assert(!has_NaNs());
	};
	constexpr explicit Point2(const Point3<T>& p) : x(p.x), y(p.y)
	{
		assert(!has_NaNs());
	};
	
	template<typename U>
	explicit operator Vec2<U>() const
	{
		return Vec2<U>(x, y);
	}

	// State checking

	[[nodiscard]] constexpr bool has_NaNs() const
	{
		return is_NaN(x) || is_NaN(y);
	}

	// Operators

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return x;
		return y;
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");
		if (i == 0) return x;
		return y;
	}

	[[nodiscard]] constexpr bool operator==(const Point2<T>& p) const { return x == p.x && y == p.y; };
	[[nodiscard]] constexpr bool operator!=(const Point2<T>& p) const { return x != p.x || y != p.y; };

	// Arithmetic operators

	[[nodiscard]] constexpr Point2<T> operator+(const Vec2<T>& v) const
	{
		return Point2<T>(x + v.x, y + v.y);
	}
	constexpr Point2<T>& operator+=(const Vec2<T>& v)
	{
		x += v.x; y += v.y;
		return *this;
	}
	[[nodiscard]] constexpr Point2<T> operator-(const Vec2<T>& v) const
	{
		return Point2<T>(x - v.x, y - v.y);
	}
	constexpr Point2<T>& operator-=(const Vec2<T>& v)
	{
		x -= v.x; y -= v.y;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator-(const Point2<T>& p) const
	{
		return Vec2<T>(x - p.x, y - p.y);
	}

	[[nodiscard]] constexpr Point2<T> operator+(const Point2<T>& rhs) const
	{
		return Point2<T>(x + rhs.x, y + rhs.y);
	}
	constexpr Point2<T>& operator+=(const Point2<T>& rhs)
	{
		x += rhs.x; y += rhs.y;
		return *this;
	}
	constexpr Point2<T>& operator-=(const Point2<T>& rhs)
	{
		x -= rhs.x; y -= rhs.y;
		return *this;
	}

	[[nodiscard]] constexpr Point2<T> operator*(T s) const
	{
		return Point2<T>(x * s, y * s);
	}
	constexpr Point2<T>& operator*=(T s)
	{
		x *= s; y *= s;
		return *this;
	}
	[[nodiscard]] constexpr Point2<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Point2<T>(x * inv, y * inv);
	}
	constexpr Point2<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		x *= inv; y *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Point2<T> operator-() const
	{
		return Point2<T>(-x, -y);
	}

};

template<typename T>
[[nodiscard]] constexpr Float distance_squared(const Point2<T>& p0, const Point2<T>& p1)
{
	return (p0 - p1).length_squared();
}
template<typename T>
[[nodiscard]] constexpr Float distance(const Point2<T>& p0, const Point2<T>& p1)
{
	return (p0 - p1).length();
}

template<typename T>
[[nodiscard]] constexpr Point2<T> lerp(Float t, const Point2<T>& p0, const Point2<T>& p1)
{
	return (1 - t) * p0 + t * p1;
}

/// <summary>
/// Computes the component-wise minimum of the two points.
/// </summary>
template<typename T>
[[nodiscard]] constexpr Point2<T> min(const Point2<T>& p0, const Point2<T>& p1)
{
	return Point2<T>(
		std::min(p0.x, p1.x),
		std::min(p0.y, p1.y)
		);
}
/// <summary>
/// Computes the component-wise maximum of the two points.
/// </summary>
template<typename T>
[[nodiscard]] constexpr Point2<T> max(const Point2<T>& p0, const Point2<T>& p1)
{
	return Point2<T>(
		std::max(p0.x, p1.x),
		std::max(p0.y, p1.y)
		);
}

template<typename T>
[[nodiscard]] constexpr Point2<T> floor(const Point2<T>& p)
{
	return Point2<T>(std::floor(p.x), std::floor(p.y));
}
template<typename T>
[[nodiscard]] constexpr Point2<T> ceil(const Point2<T>& p)
{
	return Point2<T>(std::ceil(p.x), std::ceil(p.y));
}
template<typename T>
[[nodiscard]] constexpr Point2<T> abs(const Point2<T>& p)
{
	return Point2<T>(std::abs(p.x), std::abs(p.y));
}

template<typename T>
[[nodiscard]] constexpr Point2<T> permute(const Point2<T>& p, size_t x, size_t y)
{
	return Point2<T>(p[x], p[y]);
}

// Operators related to Point2

template<typename T>
[[nodiscard]] constexpr Point2<T> operator*(T s, const Point2<T>& v)
{
	return v * s;
}


/// Typedefs

typedef Point2<int>		Point2i;
typedef Point2<Float>	Point2f;
typedef Point3<int>		Point3i;
typedef Point3<Float>	Point3f;

///////////////////////////////////////////////// Surface Normals

template<typename T>
class Normal3
{
public:
	T x = 0, y = 0, z = 0;

public:

	constexpr Normal3() = default;
	constexpr Normal3(T x, T y, T z) : x(x), y(y), z(z)
	{
		assert(!has_NaNs());
	};
	explicit constexpr Normal3(const Vec3<T>& v) : x(v.x), y(v.y), z(v.z)
	{
		assert(!has_NaNs());
	};

	// State checking

	[[nodiscard]] constexpr bool has_NaNs() const
	{
		return is_NaN(x) || is_NaN(y) || is_NaN(z);
	}

	// Mathematical operations

	[[nodiscard]] constexpr Float length_squared() const
	{
		return x * x + y * y + z * z;
	}
	[[nodiscard]] constexpr Float length() const
	{
		return std::sqrt(length_squared());
	}

	[[nodiscard]] constexpr Normal3<T> normalized() const
	{
		Float L = length();
		return *this / L;
	}
	[[nodiscard]] constexpr Normal3<T>& normalize()
	{
		Float L = length();
		*this /= L;
		return *this;
	}

	// Operators

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}

	// Arithmetic operators

	[[nodiscard]] constexpr Normal3<T> operator+(const Normal3<T>& rhs) const
	{
		return Normal3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}
	constexpr Normal3<T>& operator+=(const Normal3<T>& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}
	[[nodiscard]] constexpr Normal3<T> operator-(const Normal3<T>& rhs) const
	{
		return Normal3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	constexpr Normal3<T>& operator-=(const Normal3<T>& rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
		return *this;
	}

	[[nodiscard]] constexpr Normal3<T> operator*(T s) const
	{
		return Normal3<T>(x * s, y * s, z * s);
	}
	constexpr Normal3<T>& operator*=(T s)
	{
		x *= s; y *= s; z *= s;
		return *this;
	}
	[[nodiscard]] constexpr Normal3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Normal3<T>(x * inv, y * inv, z * inv);
	}
	constexpr Normal3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		x *= inv; y *= inv; z *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Normal3<T> operator-() const
	{
		return Normal3<T>(-x, -y, -z);
	}

};

template<typename T>
constexpr Vec3<T>::Vec3(const Normal3<T>& n) : x(n.x), y(n.y), z(n.z)
{
	assert(!n.has_NaNs());
}

template<typename T>
[[nodiscard]] constexpr Normal3<T> face_forward(const Normal3<T>& n, const Vec3<T>& v)
{
	return (dot(v, n) < static_cast<T>(0)) ? -n : n;
}

template<typename T>
[[nodiscard]] constexpr Normal3<T> abs(const Normal3<T>& n)
{
	return Normal3<T>(std::abs(n.x), std::abs(n.y), std::abs(n.z));
}

template<typename T>
[[nodiscard]] constexpr T dot(const Normal3<T>& n, const Normal3<T>& w)
{
	return n.x * w.x + n.y * w.y + n.z * w.z;
}

template<typename T>
[[nodiscard]] constexpr T abs_dot(const Normal3<T>& n, const Normal3<T>& w)
{
	return std::abs(dot(n, w));
}

template<typename T>
[[nodiscard]] constexpr Normal3<T> normalized(const Normal3<T>& n)
{
	Float L = n.length();
	return n / L;
}

// Operators related to Normal3

template<typename T>
[[nodiscard]] constexpr Normal3<T> operator*(T s, const Normal3<T>& n)
{
	return n * s;
}


/// Typedefs

typedef Normal3<int>	Normal3i;
typedef Normal3<Float>	Normal3f;

///////////////////////////////////////////////// Rays

// Forward declare medium
class Medium;

class Ray
{
public:
	// Ray origin
	Point3f o{};
	// Ray direction
	Vec3f d{};
	// Ray origin medium (non-owning pointer).
	const Medium* medium;
	
	// Ray max distance
	mutable Float t_max;

	// Ray time
	Float time;
	
public:
	
	constexpr Ray() : medium(nullptr), t_max(INFINITY), time(0)
	{};
	constexpr Ray(const Point3f& o, const Vec3f& d,
				  Float t_max = INFINITY, Float time = 0,
				  const Medium* medium = nullptr)
		: o(o), d(d), medium(nullptr), t_max(t_max), time(time)
	{};

	[[nodiscard]] constexpr Point3f operator()(Float t) const
	{
		return o + d * t;
	}

};

class RayDifferential : public Ray
{
public:
	Point3f rx_o{}, ry_o{};
	Vec3f rx_d{}, ry_d{};
	
	bool has_differentials = false;

public:
	constexpr RayDifferential() : has_differentials(false)
	{}
	constexpr RayDifferential(const Point3f& o, const Vec3f& d,
							   Float t_max = INFINITY, Float time = 0,
							   const Medium* medium = nullptr)
		: Ray(o, d, t_max, time, medium), has_differentials(false)
	{}
	constexpr RayDifferential(const Ray& r)
		: Ray(r), has_differentials(false)
	{}

	// Public Methods
	
	constexpr void scale_differentials(Float s)
	{
		rx_o = o + (rx_o - o) * s;
		ry_o = o + (ry_o - o) * s;
		rx_d = d + (rx_d - d) * s;
		ry_d = d + (ry_d - d) * s;
	}
};

}

#endif