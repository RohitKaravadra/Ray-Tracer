#pragma once
#include <math.h>

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
	Vec2()
	{
		x = 0;
		y = 0;
	}
	Vec2(float _a)
	{
		x = _a;
		y = _a;
	}
	Vec2(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	// arithmetic operators
	Vec2 operator+(const Vec2 v) const { return Vec2(x + v.x, y + v.y); }
	Vec2 operator-(const Vec2 v) const { return Vec2(x - v.x, y - v.y); }
	Vec2 operator+=(const Vec2 v) { x += v.x; y += v.y; return *this; }
	Vec2 operator-=(const Vec2 v) { x -= v.x; y -= v.y; return *this; }

	Vec2 operator*(const Vec2 v) const { return Vec2(x * v.x, y * v.y); }
	Vec2 operator/(const Vec2 v) const { return Vec2(x / v.x, y / v.y); }
	Vec2 operator*=(const Vec2 v) { x *= v.x; y *= v.y; return *this; }
	Vec2 operator/=(const Vec2 v) { x /= v.x; y /= v.y; return *this; }

	Vec2 operator+(const float v) const { return Vec2(x + v, y + v); }
	Vec2 operator-(const float v) const { return Vec2(x - v, y - v); }
	Vec2 operator+=(const float v) { x += v; y += v; return *this; }
	Vec2 operator-=(const float v) { x -= v; y -= v; return *this; }

	Vec2 operator*(const float v) const { return Vec2(x * v, y * v); }
	Vec2 operator/(const float v) const { return Vec2(x / v, y / v); }
	Vec2 operator*=(const float v) { x *= v; y *= v; return *this; }
	Vec2 operator/=(const float v) { x /= v; y /= v; return *this; }

	// unary negation
	Vec2 operator-() const { return Vec2(-x, -y); }

	// comparision operators
	bool operator<=(const Vec2& other) const { return x <= other.x && y <= other.y; }
	bool operator>=(const Vec2& other) const { return x >= other.x && y >= other.y; }
	bool operator<(const Vec2& other) const { return x < other.x && y < other.y; }
	bool operator>(const Vec2& other) const { return x > other.x && y > other.y; }

	bool operator<=(const float v) const { return x <= v && y <= v; }
	bool operator>=(const float v) const { return x >= v && y >= v; }
	bool operator<(const float v) const { return x < v && y < v; }
	bool operator>(const float v) const { return x > v && y > v; }

	float lengthSq() const { return ((x * x) + (y * y)); }
	float length() const { return sqrtf((x * x) + (y * y)); }

	Vec2 normalize() const
	{
		float l = 1.0f / sqrtf((x * x) + (y * y));
		return Vec2(x * l, y * l);
	}

	float dot(Vec2 v) const { return ((x * v.x) + (y * v.y)); }
	Vec2 cross(Vec2 v) const { return Vec2((y * v.x) - (x * v.y), (x * v.y) - (y * v.x)); }
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
	Vec2i()
	{
		x = 0;
		y = 0;
	}
	Vec2i(int _a)
	{
		x = _a;
		y = _a;
	}
	Vec2i(int _x, int _y)
	{
		x = _x;
		y = _y;
	}

	// arithmetic operators
	Vec2i operator+(const Vec2i v) const { return Vec2i(x + v.x, y + v.y); }
	Vec2i operator-(const Vec2i v) const { return Vec2i(x - v.x, y - v.y); }
	Vec2i operator+=(const Vec2i v) { x += v.x; y += v.y; return *this; }
	Vec2i operator-=(const Vec2i v) { x -= v.x; y -= v.y; return *this; }

	Vec2i operator*(const float v) const { return Vec2i(x * v, y * v); }
	Vec2i operator/(const float v) const { return Vec2i(x / v, y / v); }
	Vec2i operator*=(const float v) { x *= v; y *= v; return *this; }
	Vec2i operator/=(const float v) { x /= v; y /= v; return *this; }

	// unary negation
	Vec2i operator-() const { return Vec2i(-x, -y); }

	// comparision operators
	bool operator<=(const Vec2i& other) const { return x <= other.x && y <= other.y; }
	bool operator>=(const Vec2i& other) const { return x >= other.x && y >= other.y; }
	bool operator<(const Vec2i& other) const { return x < other.x && y < other.y; }
	bool operator>(const Vec2i& other) const { return x > other.x && y > other.y; }

	Vec2i Min(const Vec2i& other) { return Vec2i(x < other.x ? x : other.x, y < other.y ? y : other.y); }
	Vec2i Max(const Vec2i& other) { return Vec2i(x > other.x ? x : other.x, y > other.y ? y : other.y); }
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
	Vec3()
	{
		x = 0;
		y = 0;
		z = 0;
		w = 1.0f;
	}
	Vec3(float _x, float _y, float _z)
	{
		x = _x;
		y = _y;
		z = _z;
		w = 1.0f;
	}
	Vec3(float _x, float _y, float _z, float _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}
	Vec3 operator+(const Vec3 v) const
	{
		return Vec3(x + v.x, y + v.y, z + v.z);
	}
	Vec3 operator-(const Vec3 v) const
	{
		return Vec3(x - v.x, y - v.y, z - v.z);
	}
	Vec3 operator*(const float v) const
	{
		return Vec3(x * v, y * v, z * v);
	}
	Vec3 operator/(const float v) const
	{
		return Vec3(x / v, y / v, z / v, w / v);
	}
	Vec3 operator*(const Vec3 v) const
	{
		return Vec3(x * v.x, y * v.y, z * v.z);
	}
	Vec3 perspectiveDivide() const
	{
		return Vec3(x / w, y / w, z / w, 1.0f / w);
	}
	Vec3 operator-() const { return Vec3(-x, -y, -z); }
	float lengthSq()
	{
		return ((x * x) + (y * y) + (z * z));
	}
	float length()
	{
		return sqrtf((x * x) + (y * y) + (z * z));
	}
	Vec3 normalize() const
	{
		float l = 1.0f / sqrtf((x * x) + (y * y) + (z * z));
		return Vec3(x * l, y * l, z * l);
	}
	float dot(Vec3 v) const
	{
		return ((x * v.x) + (y * v.y) + (z * v.z));
	}
	Vec3 cross(Vec3 v) const
	{
		return Vec3((y * v.z) - (z * v.y), (z * v.x) - (x * v.z), (x * v.y) - (y * v.x));
	}
};

static float Dot(const Vec3 v1, const Vec3 v2)
{
	return ((v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z));
}

static Vec3 Cross(const Vec3& v1, const Vec3& v2)
{
	return Vec3((v1.y * v2.z) - (v1.z * v2.y), (v1.z * v2.x) - (v1.x * v2.z), (v1.x * v2.y) - (v1.y * v2.x));
}

static Vec3 Max(Vec3 a, Vec3 b)
{
	return Vec3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
}

static Vec3 Min(Vec3 a, Vec3 b)
{
	return Vec3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
}