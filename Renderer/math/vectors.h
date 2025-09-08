#pragma once
#include <math.h>
#include <ostream>

// Forward declarations to allow cross-type constructors
class Vec2i;
class Vec2u;
class Vec3;

class Vec2
{
public:
	union {
		struct {
			float x;
			float y;
		};
		float coords[2];
	};

	// Constructors
	Vec2();
	Vec2(float a);
	Vec2(float x, float y);

	// conversion constructors
	Vec2(const Vec2i& v);
	Vec2(const Vec2u& v);
	Vec2(const Vec3& v);

	// Arithmetic operators
	Vec2 operator+(const Vec2& v) const;
	Vec2 operator-(const Vec2& v) const;
	Vec2 operator+=(const Vec2& v);
	Vec2 operator-=(const Vec2& v);

	Vec2 operator*(const Vec2& v) const;
	Vec2 operator/(const Vec2& v) const;
	Vec2 operator*=(const Vec2& v);
	Vec2 operator/=(const Vec2& v);

	Vec2 operator+(const float& v) const;
	Vec2 operator-(const float& v) const;
	Vec2 operator+=(const float& v);
	Vec2 operator-=(const float& v);

	Vec2 operator*(const float& v) const;
	Vec2 operator/(const float& v) const;
	Vec2 operator*=(const float& v);
	Vec2 operator/=(const float& v);

	// Unary negation
	Vec2 operator-() const;

	// Comparison operators
	bool operator<=(const Vec2& other) const;
	bool operator>=(const Vec2& other) const;
	bool operator<(const Vec2& other) const;
	bool operator>(const Vec2& other) const;

	bool operator<=(const float& v) const;
	bool operator>=(const float& v) const;
	bool operator<(const float& v) const;
	bool operator>(const float& v) const;

	// Vector operations
	float lengthSq() const;
	float length() const;
	Vec2 normalize() const;
	float dot(Vec2 v) const;
	float cross(Vec2 v) const; // Corrected: 2D cross product returns a scalar

	friend std::ostream& operator<<(std::ostream& os, const Vec2& v);
};

class Vec2i
{
public:
	union {
		struct {
			int x;
			int y;
		};
		int coords[2];
	};

	// Constructors
	Vec2i();
	Vec2i(int a);
	Vec2i(int x, int y);

	//conversion constructors
	Vec2i(const Vec2& v);
	Vec2i(const Vec2u& v);
	Vec2i(const Vec3& v);

	// Arithmetic operators
	Vec2i operator+(const Vec2i& v) const;
	Vec2i operator-(const Vec2i& v) const;
	Vec2i operator+=(const Vec2i& v);
	Vec2i operator-=(const Vec2i& v);

	Vec2i operator*(const float& v) const;
	Vec2i operator/(const float& v) const;
	Vec2i operator*=(const float& v);
	Vec2i operator/=(const float& v);

	// Unary negation
	Vec2i operator-() const;

	// Comparison operators
	bool operator<=(const Vec2i& other) const;
	bool operator>=(const Vec2i& other) const;
	bool operator<(const Vec2i& other) const;
	bool operator>(const Vec2i& other) const;

	// Min/Max
	Vec2i Min(const Vec2i& other);
	Vec2i Max(const Vec2i& other);

	friend std::ostream& operator<<(std::ostream& os, const Vec2i& v);
};

class Vec2u
{
public:
	union {
		struct {
			unsigned int x; // Corrected type
			unsigned int y; // Corrected type
		};
		unsigned int coords[2]; // Corrected type
	};

	// Constructors
	Vec2u();
	Vec2u(unsigned int a);
	Vec2u(unsigned int x, unsigned int y);

	// conversion constructors
	Vec2u(const Vec2& v);
	Vec2u(const Vec2i& v);
	Vec2u(const Vec3& v);

	// Arithmetic operators
	Vec2u operator+(const Vec2u& v) const;
	Vec2u operator-(const Vec2u& v) const;
	Vec2u operator+=(const Vec2u& v);
	Vec2u operator-=(const Vec2u& v);

	Vec2u operator*(const float& v) const;
	Vec2u operator/(const float& v) const;
	Vec2u operator*=(const float& v);
	Vec2u operator/=(const float& v);

	// Comparison operators
	bool operator<=(const Vec2u& other) const;
	bool operator>=(const Vec2u& other) const;
	bool operator<(const Vec2u& other) const;
	bool operator>(const Vec2u& other) const;
	bool operator<(const Vec2i& other) const;
	bool operator>(const Vec2i& other) const;

	// Min/Max
	Vec2u Min(const Vec2u& other);
	Vec2u Max(const Vec2u& other);

	friend std::ostream& operator<<(std::ostream& os, const Vec2u& v);
};

class Vec3
{
public:
	union {
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float coords[4];
	};

	// Constructors
	Vec3();
	Vec3(float a);
	Vec3(float x, float y, float z);
	Vec3(float x, float y, float z, float w);

	// conversion constructors
	Vec3(const Vec2& v);
	Vec3(const Vec2i& v);
	Vec3(const Vec2u& v);

	// Arithmetic operators
	Vec3 operator+(const Vec3& v) const;
	Vec3 operator-(const Vec3& v) const;
	Vec3 operator+=(const Vec3& v);
	Vec3 operator-=(const Vec3& v);

	Vec3 operator*(const Vec3& v) const;
	Vec3 operator*=(const Vec3& v);

	Vec3 operator*(const float& v) const;
	Vec3 operator/(const float& v) const;
	Vec3 operator*=(const float& v);
	Vec3 operator/=(const float& v);

	// Unary negation
	Vec3 operator-() const;

	// Vector operations
	Vec3 perspectiveDivide() const;
	float lengthSq() const;
	float length() const;
	Vec3 normalize() const;
	float dot(Vec3 v) const;
	Vec3 cross(Vec3 v) const;

	friend std::ostream& operator<<(std::ostream& os, const Vec3& v);
};

// Global helper functions
float Dot(const Vec3 v1, const Vec3 v2);
Vec3 Cross(const Vec3& v1, const Vec3& v2);
Vec3 Max(const Vec3& a, const Vec3& b);
Vec3 Min(const Vec3& a, const Vec3& b);