#ifndef VECMATH_H
#define VECMATH_H

#include "aito.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <array>
#include <type_traits>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>


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
private:
	glm::vec<2, T, glm::packed_highp> v_;

public:
	T& x = v_.x;
	T& y = v_.y;

public:

	constexpr Vec2() = default;
	constexpr Vec2(T x, T y) : v_(x, y)
	{
		assert(!has_NaNs());
	};
	constexpr Vec2(glm::vec<2, T, glm::packed_highp> vec) : v_(vec)
	{
		assert(!has_NaNs());
	}

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

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 1 && "Out of bounds vector access!");

		return v_[i];
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds vector access!");

		return v_[i];
	}

	constexpr Vec2<T>& operator=(const Vec2<T>& v)
	{
		v_ = v.v_;
		return *this;
	};

	[[nodiscard]] constexpr bool operator==(const Vec2<T>& v) const { return v_ == v.v_; };
	[[nodiscard]] constexpr bool operator!=(const Vec2<T>& v) const { return v_ != v.v_; };


	[[nodiscard]] constexpr Vec2<T> operator+(const Vec2<T>& rhs) const
	{
		return Vec2<T>(v_ + rhs.v_);
	}
	constexpr Vec2<T>& operator+=(const Vec2<T>& rhs)
	{
		v_ += rhs.v_;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator-(const Vec2<T>& rhs) const
	{
		return Vec2<T>(v_ - rhs.v_);
	}
	constexpr Vec2<T>& operator-=(const Vec2<T>& rhs)
	{
		v_ -= rhs.v_;
		return *this;
	}

	[[nodiscard]] constexpr Vec2<T> operator*(T s) const
	{
		return Vec2<T>(v_ * s);
	}
	constexpr Vec2<T>& operator*=(T s)
	{
		v_ *= s;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Vec2<T>(v_ * inv);
	}
	constexpr Vec2<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		v_ *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Vec2<T> operator-() const
	{
		return Vec2<T>(-v_);
	}

	// Friends
	template<typename T>
	friend class Point2;

	template<typename T>
	[[nodiscard]] friend constexpr Vec2<T> abs(const Vec2<T>& v);
	template<typename T>
	[[nodiscard]] friend constexpr T dot(const Vec2<T>& v, const Vec2<T>& w);
};

template<typename T>
[[nodiscard]] constexpr Vec2<T> abs(const Vec2<T>& v)
{
	return glm::abs(v.v_);
}

template<typename T>
[[nodiscard]] constexpr T dot(const Vec2<T>& v, const Vec2<T>& w)
{
	return glm::dot(v.v_, w.v_);
}

template<typename T>
[[nodiscard]] constexpr T abs_dot(const Vec2<T>& v, const Vec2<T>& w)
{
	return abs(dot(v, w));
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
private:
	glm::vec<3, T, glm::packed_highp> v_;

public:
	T& x = v_.x;
	T& y = v_.y;
	T& z = v_.z;

public:

	constexpr Vec3() = default;
	constexpr Vec3(T x, T y, T z) : v_(x, y, z)
	{
		assert(!has_NaNs());
	};
	constexpr Vec3(glm::vec<3, T, glm::packed_highp> vec) : v_(vec)
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

	[[nodiscard]] constexpr T operator[](size_t i) const
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		
		return v_[i];
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");
		
		return v_[i];
	}

	constexpr Vec3<T>& operator=(const Vec3<T>& v)
	{
		v_ = v.v_;
		return *this;
	};

	[[nodiscard]] constexpr bool operator==(const Vec3<T>& v) const { return v_ == v.v_; };
	[[nodiscard]] constexpr bool operator!=(const Vec3<T>& v) const { return v_ != v.v_; };


	[[nodiscard]] constexpr Vec3<T> operator+(const Vec3<T>& rhs) const
	{
		return Vec3<T>(v_ + rhs.v_);
	}
	constexpr Vec3<T>& operator+=(const Vec3<T>& rhs)
	{
		v_ += rhs.v_;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator-(const Vec3<T>& rhs) const
	{
		return Vec3<T>(v_ - rhs.v_);
	}
	constexpr Vec3<T>& operator-=(const Vec3<T>& rhs)
	{
		v_ -= rhs.v_;
		return *this;
	}

	[[nodiscard]] constexpr Vec3<T> operator*(T s) const
	{
		return Vec3<T>(v_ * s);
	}
	constexpr Vec3<T>& operator*=(T s)
	{
		v_ *= s;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Vec3<T>(v_ * inv);
	}
	constexpr Vec3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		v_ *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Vec3<T> operator-() const
	{
		return Vec3<T>(-v_);
	}

	// Friends
	template<typename T>
	friend class Point3;

	template<typename T>
	[[nodiscard]] friend constexpr Vec3<T> abs(const Vec3<T>& v);
	template<typename T>
	[[nodiscard]] friend constexpr T dot(const Vec3<T>& v, const Vec3<T>& w);
	template<typename T>
	[[nodiscard]] friend constexpr Vec3<T> cross(const Vec3<T>& v, const Vec3<T>& w);
};


template<typename T>
[[nodiscard]] constexpr Vec3<T> abs(const Vec3<T>& v)
{
	return glm::abs(v.v_);
}

template<typename T>
[[nodiscard]] constexpr T dot(const Vec3<T>& v, const Vec3<T>& w)
{
	return glm::dot(v.v_, w.v_);
}

template<typename T>
[[nodiscard]] constexpr T abs_dot(const Vec3<T>& v, const Vec3<T>& w)
{
	return std::abs(dot(v, w));
}

template<typename T>
[[nodiscard]] constexpr Vec3<T> cross(const Vec3<T>& v, const Vec3<T>& w)
{
	
	return Vec3<T>(glm::cross(v.v_, w.v_));
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
		*v2_out = Vec3<T>(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
	else
		*v2_out = Vec3<T>(0, v1.z, v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
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
private:
	glm::vec<3, T, glm::packed_highp> v_;

public:
	T& x = v_.x;
	T& y = v_.y;
	T& z = v_.z;

public:
	constexpr Point3() = default;
	constexpr Point3(T x, T y, T z) : v_(x, y, z)
	{
		assert(!has_NaNs());
	};
	constexpr Point3(glm::vec<3, T, glm::packed_highp> vec) : v_(vec)
	{
		assert(!has_NaNs());
	};
	template<typename U>
	constexpr explicit Point3(const Point3<U>& p)
		: v_(p.v_)
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
		
		return v_[i];
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");

		return v_[i];
	}

	constexpr Point3<T>& operator=(const Point3<T>& p)
	{
		v_ = p.v_;
		return *this;
	};

	[[nodiscard]] constexpr bool operator==(const Point3<T>& p) const
	{
		return v_ == p.v_;
	}
	[[nodiscard]] constexpr bool operator!=(const Point3<T>& p) const
	{
		return v_ != p.v_;
	}

	// Arithmetic operators

	[[nodiscard]] constexpr Point3<T> operator+(const Vec3<T>& v) const
	{
		return Point3<T>(v_ + v.v_);
	}
	constexpr Point3<T>& operator+=(const Vec3<T>& v)
	{
		v_ += v.v_;
		return *this;
	}
	[[nodiscard]] constexpr Point3<T> operator-(const Vec3<T>& v) const
	{
		return Point3<T>(v_ - v.v_);
	}
	constexpr Point3<T>& operator-=(const Vec3<T>& v)
	{
		v_ -= v.v_;
		return *this;
	}
	[[nodiscard]] constexpr Vec3<T> operator-(const Point3<T>& p) const
	{
		return Vec3<T>(v_ - p.v_);
	}

	[[nodiscard]] constexpr Point3<T> operator+(const Point3<T>& rhs) const
	{
		return Point3<T>(v_ + rhs.v_);
	}
	constexpr Point3<T>& operator+=(const Point3<T>& rhs)
	{
		v_ += rhs.v_;
		return *this;
	}
	constexpr Point3<T>& operator-=(const Point3<T>& rhs)
	{
		v_ -= rhs.v_;
		return *this;
	}

	[[nodiscard]] constexpr Point3<T> operator*(T s) const
	{
		return Point3<T>(v_ * s);
	}
	constexpr Point3<T>& operator*=(T s)
	{
		v_ *= s;
		return *this;
	}
	[[nodiscard]] constexpr Point3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Point3<T>(v_ * s);
	}
	constexpr Point3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		v_ *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Point3<T> operator-() const
	{
		return Point3<T>(-v_);
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

// Operators related to Point3

template<typename T>
[[nodiscard]] constexpr Point3<T> operator*(T s, const Point3<T>& v)
{
	return v * s;
}


template<typename T>
class Point2
{
private:
	glm::vec<2, T, glm::packed_highp> v_;

public:
	T& x = v_.x;
	T& y = v_.y;

public:

	constexpr Point2() = default;
	constexpr Point2(T x, T y) : v_(x, y)
	{
		assert(!has_NaNs());
	};
	constexpr Point2(glm::vec<2, T, glm::packed_highp> vec) : v_(vec)
	{
		assert(!has_NaNs());
	};
	template<typename U>
	constexpr explicit Point2(const Point2<U>& p)
		: v_(static_cast<U>(p.x), static_cast<U>(p.y))
	{
		assert(!has_NaNs());
	};
	constexpr explicit Point2(const Point3<T>& p) : v_(p.x, p.y)
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
		
		return v_[i];
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 1 && "Out of bounds access!");

		return v_[i];
	}

	constexpr Point2<T>& operator=(const Point2<T>& p)
	{
		v_ = p.v_;
		return *this;
	};

	[[nodiscard]] constexpr bool operator==(const Point2<T>& p) const { return v_ == p.v_; };
	[[nodiscard]] constexpr bool operator!=(const Point2<T>& p) const { return v_ != p.v_; };

	// Arithmetic operators

	[[nodiscard]] constexpr Point2<T> operator+(const Vec2<T>& v) const
	{
		return Point2<T>(v_ + v.v_);
	}
	constexpr Point2<T>& operator+=(const Vec2<T>& v)
	{
		v_ += v.v_;
		return *this;
	}
	[[nodiscard]] constexpr Point2<T> operator-(const Vec2<T>& v) const
	{
		return Point2<T>(v_ - v.v_);
	}
	constexpr Point2<T>& operator-=(const Vec2<T>& v)
	{
		v_ -= v.v_;
		return *this;
	}
	[[nodiscard]] constexpr Vec2<T> operator-(const Point2<T>& p) const
	{
		return Vec2<T>(v_ - p.v_);
	}

	[[nodiscard]] constexpr Point2<T> operator+(const Point2<T>& rhs) const
	{
		return Point2<T>(v_ + rhs.v_);
	}
	constexpr Point2<T>& operator+=(const Point2<T>& rhs)
	{
		v_ += rhs.v_;
		return *this;
	}
	constexpr Point2<T>& operator-=(const Point2<T>& rhs)
	{
		v_ -= rhs.v_;
		return *this;
	}

	[[nodiscard]] constexpr Point2<T> operator*(T s) const
	{
		return Point2<T>(v_ * s);
	}
	constexpr Point2<T>& operator*=(T s)
	{
		v_ *= s;
		return *this;
	}
	[[nodiscard]] constexpr Point2<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Point2<T>(v_ * inv);
	}
	constexpr Point2<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		v_ *= inv;
		return *this;
	}

	[[nodiscard]] constexpr Point2<T> operator-() const
	{
		return Point2<T>(-v_);
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
private:
	glm::vec<3, T, glm::packed_highp> v_;

public:
	T& x = v_.x;
	T& y = v_.y;
	T& z = v_.z;

public:

	constexpr Normal3() = default;
	constexpr Normal3(T x, T y, T z) : v_(x, y, z)
	{
		assert(!has_NaNs());
	};
	constexpr Normal3(glm::vec<3, T, glm::packed_highp> vec) : v_(vec)
	{
		assert(!has_NaNs());
	};
	explicit constexpr Normal3(const Vec3<T>& v) : v_(v.x, v.y, v.z)
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
		
		return v_[i];
	}
	constexpr T& operator[](size_t i)
	{
		assert(i >= 0 && i <= 2 && "Out of bounds vector access!");

		return v_[i];
	}

	// Arithmetic operators

	[[nodiscard]] constexpr Normal3<T> operator+(const Normal3<T>& rhs) const
	{
		return Normal3<T>(v_ + rhs.v_);
	}
	constexpr Normal3<T>& operator+=(const Normal3<T>& rhs)
	{
		v_ += rhs.v_;
		return *this;
	}
	[[nodiscard]] constexpr Normal3<T> operator-(const Normal3<T>& rhs) const
	{
		return Normal3<T>(v_ - rhs.v_);
	}
	constexpr Normal3<T>& operator-=(const Normal3<T>& rhs)
	{
		v_ -= rhs.v_;
		return *this;
	}

	[[nodiscard]] constexpr Normal3<T> operator*(T s) const
	{
		return Normal3<T>(v_ * s);
	}
	constexpr Normal3<T>& operator*=(T s)
	{
		v_ *= s;
		return *this;
	}
	[[nodiscard]] constexpr Normal3<T> operator/(T s) const
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		return Normal3<T>(v_ * inv);
	}
	constexpr Normal3<T>& operator/=(T s)
	{
		assert(s != 0);
		Float inv = static_cast<Float>(1) / static_cast<Float>(s);

		v_ *= s;
		return *this;
	}

	[[nodiscard]] constexpr Normal3<T> operator-() const
	{
		return Normal3<T>(-v_);
	}

};

template<typename T>
constexpr Vec3<T>::Vec3(const Normal3<T>& n) : v_(n.x, n.y, n.z)
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


// Operators related to Normal3

template<typename T>
[[nodiscard]] constexpr Normal3<T> operator*(T s, const Normal3<T>& n)
{
	return n * s;
}


/// Typedefs

typedef Normal3<int>	Normal3i;
typedef Normal3<Float>	Normal3f;

///////////////////////////////////////////////// 4 by 4 Matrix

class Mat4
{
private:
	using matType = glm::mat<4, 4, Float, glm::packed_highp>;

	// Keep in mind that this is a glm matrix, and it thus uses column major notation.
	matType m_{1};

public:
	constexpr Mat4() = default;
	constexpr Mat4(glm::mat<4, 4, Float, glm::packed_highp> matrix) : m_(matrix)
	{}
	// TAKES IN THE ARGUMENTS IN ROW MAJOR FORMAT FOR READABILITY.
	constexpr Mat4(Float t00, Float t01, Float t02, Float t03,
		 Float t10, Float t11, Float t12, Float t13,
		 Float t20, Float t21, Float t22, Float t23,
		 Float t30, Float t31, Float t32, Float t33)
		: m_(t00, t10, t20, t30,
			t01, t11, t21, t31,
			t02, t12, t22, t32,
			t03, t13, t23, t33)
	{};

	// Public methods

	inline Mat4& transpose()
	{
		m_ = glm::transpose(m_);
		return *this;
	}
	[[nodiscard]] constexpr Mat4 get_transpose() const
	{
		return glm::transpose(m_);
	};

	[[nodiscard]] constexpr Mat4 get_inverse() const
	{
		return glm::inverse(m_);
	};

	// Operators

	template<typename T>
	struct rowStruct
	{
	private:
		std::array<T* const, 4> v;

	public:
		constexpr rowStruct(std::conditional_t<std::is_const_v<T>, const matType, matType>& m, matType::length_type index)
			: v({ &m[0][index], &m[1][index], &m[2][index], &m[3][index] }),
			x(m[0][index]),
			y(m[1][index]),
			z(m[2][index]),
			w(m[3][index])
		{}

		T& x;
		T& y;
		T& z;
		T& w;

		constexpr const T& operator[](matType::row_type::length_type index) const
		{
			return *v[index];
		}
		constexpr T& operator[](matType::row_type::length_type index)
		{
			return *v[index];
		}

		constexpr operator glm::vec4() const
		{
			return {x, y, z, w};
		}

		constexpr rowStruct<T>& operator=(const glm::vec4& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;

			return *this;
		}
	};
	typedef rowStruct<Float> row_ret_t;
	typedef const rowStruct<const Float> row_ret_c_t;


	constexpr glm::vec4 operator[](matType::length_type index) const
	{
		return row_ret_c_t(m_, index);
	}
	constexpr row_ret_t operator[](matType::length_type index)
	{
		return row_ret_t(m_, index);
	}


	[[nodiscard]] inline bool operator==(const Mat4& rhs) const
	{
		return m_ == rhs.m_;
	}
	[[nodiscard]] inline bool operator!=(const Mat4& rhs) const
	{
		return m_ != rhs.m_;
	}

	[[nodiscard]] constexpr Mat4 operator*(const Mat4& rhs) const
	{
		return m_ * rhs.m_;
	}
	constexpr Mat4& operator*=(const Mat4& rhs)
	{
		*this = (*this) * rhs;
		return *this;
	};

};

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