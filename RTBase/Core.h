#pragma once

// Do not change this code!

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory.h>
#include <vector>
#include <algorithm>

// Stop warnings about M_PI being a double
#pragma warning( disable : 4244)

#define SQ(x) (x * x)

class Colour
{
public:
	float r;
	float g;
	float b;
	Colour() { r = 0; g = 0; b = 0; }
	Colour(float _r, float _g, float _b)
	{
		r = _r;
		g = _g;
		b = _b;
	}
	Colour(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a)
	{
		r = (float)_r / 255.0f;
		g = (float)_g / 255.0f;
		b = (float)_b / 255.0f;
	}
	void ToRGB(unsigned char& cr, unsigned char& cg, unsigned char& cb)
	{
		cr = (unsigned char)(r * 255);
		cg = (unsigned char)(g * 255);
		cb = (unsigned char)(b * 255);
	}
	Colour operator+(const Colour& colour) const
	{
		Colour c;
		c.r = r + colour.r;
		c.g = g + colour.g;
		c.b = b + colour.b;
		return c;
	}
	Colour operator-(const Colour& colour) const
	{
		Colour c;
		c.r = r - colour.r;
		c.g = g - colour.g;
		c.b = b - colour.b;
		return c;
	}
	Colour operator*(const Colour& colour) const
	{
		Colour c;
		c.r = r * colour.r;
		c.g = g * colour.g;
		c.b = b * colour.b;
		return c;
	}
	Colour operator/(const Colour& colour) const
	{
		Colour c;
		c.r = r / colour.r;
		c.g = g / colour.g;
		c.b = b / colour.b;
		return c;
	}
	Colour operator*(const float v) const
	{
		Colour c;
		c.r = r * v;
		c.g = g * v;
		c.b = b * v;
		return c;
	}
	Colour operator/(const float v) const
	{
		Colour c;
		c.r = r / v;
		c.g = g / v;
		c.b = b / v;
		return c;
	}
	float Lum()
	{
		return ((0.2126f * r) + (0.7152f * g) + (0.0722f * b));
	}
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

struct Vertex
{
	Vec3 p;
	Vec3 normal;
	float u;
	float v;
};

class Matrix
{
public:
	union
	{
		float a[4][4];
		float m[16];
	};
	Matrix() { identity(); }
	Matrix(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33)
	{
		a[0][0] = m00;
		a[0][1] = m01;
		a[0][2] = m02;
		a[0][3] = m03;
		a[1][0] = m10;
		a[1][1] = m11;
		a[1][2] = m12;
		a[1][3] = m13;
		a[2][0] = m20;
		a[2][1] = m21;
		a[2][2] = m22;
		a[2][3] = m23;
		a[3][0] = m30;
		a[3][1] = m31;
		a[3][2] = m32;
		a[3][3] = m33;
	}
	void identity()
	{
		memset(m, 0, 16 * sizeof(float));
		m[0] = 1.0f;
		m[5] = 1.0f;
		m[10] = 1.0f;
		m[15] = 1.0f;
	}
	Matrix transpose()
	{
		return Matrix(a[0][0], a[1][0], a[2][0], a[3][0],
			a[0][1], a[1][1], a[2][1], a[3][1],
			a[0][2], a[1][2], a[2][2], a[3][2],
			a[0][3], a[1][3], a[2][3], a[3][3]);
	}
	float& operator[](int index)
	{
		return m[index];
	}
	static Matrix translation(const Vec3& v)
	{
		Matrix mat;
		mat.a[0][3] = v.x;
		mat.a[1][3] = v.y;
		mat.a[2][3] = v.z;
		return mat;
	}
	static Matrix scaling(const Vec3& v)
	{
		Matrix mat;
		mat.m[0] = v.x;
		mat.m[5] = v.y;
		mat.m[10] = v.z;
		return mat;
	}
	Matrix mul(const Matrix& matrix) const
	{
		Matrix ret;

		ret.m[0] = m[0] * matrix.m[0] + m[1] * matrix.m[4] + m[2] * matrix.m[8] + m[3] * matrix.m[12];
		ret.m[1] = m[0] * matrix.m[1] + m[1] * matrix.m[5] + m[2] * matrix.m[9] + m[3] * matrix.m[13];
		ret.m[2] = m[0] * matrix.m[2] + m[1] * matrix.m[6] + m[2] * matrix.m[10] + m[3] * matrix.m[14];
		ret.m[3] = m[0] * matrix.m[3] + m[1] * matrix.m[7] + m[2] * matrix.m[11] + m[3] * matrix.m[15];
		ret.m[4] = m[4] * matrix.m[0] + m[5] * matrix.m[4] + m[6] * matrix.m[8] + m[7] * matrix.m[12];
		ret.m[5] = m[4] * matrix.m[1] + m[5] * matrix.m[5] + m[6] * matrix.m[9] + m[7] * matrix.m[13];
		ret.m[6] = m[4] * matrix.m[2] + m[5] * matrix.m[6] + m[6] * matrix.m[10] + m[7] * matrix.m[14];
		ret.m[7] = m[4] * matrix.m[3] + m[5] * matrix.m[7] + m[6] * matrix.m[11] + m[7] * matrix.m[15];
		ret.m[8] = m[8] * matrix.m[0] + m[9] * matrix.m[4] + m[10] * matrix.m[8] + m[11] * matrix.m[12];
		ret.m[9] = m[8] * matrix.m[1] + m[9] * matrix.m[5] + m[10] * matrix.m[9] + m[11] * matrix.m[13];
		ret.m[10] = m[8] * matrix.m[2] + m[9] * matrix.m[6] + m[10] * matrix.m[10] + m[11] * matrix.m[14];
		ret.m[11] = m[8] * matrix.m[3] + m[9] * matrix.m[7] + m[10] * matrix.m[11] + m[11] * matrix.m[15];
		ret.m[12] = m[12] * matrix.m[0] + m[13] * matrix.m[4] + m[14] * matrix.m[8] + m[15] * matrix.m[12];
		ret.m[13] = m[12] * matrix.m[1] + m[13] * matrix.m[5] + m[14] * matrix.m[9] + m[15] * matrix.m[13];
		ret.m[14] = m[12] * matrix.m[2] + m[13] * matrix.m[6] + m[14] * matrix.m[10] + m[15] * matrix.m[14];
		ret.m[15] = m[12] * matrix.m[3] + m[13] * matrix.m[7] + m[14] * matrix.m[11] + m[15] * matrix.m[15];

		return ret;
	}
	Matrix operator*(const Matrix& matrix)
	{
		return mul(matrix);
	}
	Vec3 mulVec(const Vec3& v)
	{
		return Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]),
			(v.x * m[4] + v.y * m[5] + v.z * m[6]),
			(v.x * m[8] + v.y * m[9] + v.z * m[10]));
	}
	Vec3 mulPoint(const Vec3& v)
	{
		Vec3 v1 = Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]) + m[3],
			(v.x * m[4] + v.y * m[5] + v.z * m[6]) + m[7],
			(v.x * m[8] + v.y * m[9] + v.z * m[10]) + m[11]);
		return v1;
	}
	Vec3 mulPointAndPerspectiveDivide(const Vec3& v)
	{
		Vec3 v1 = Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]) + m[3],
			(v.x * m[4] + v.y * m[5] + v.z * m[6]) + m[7],
			(v.x * m[8] + v.y * m[9] + v.z * m[10]) + m[11]);
		float w;
		w = (m[12] * v.x) + (m[13] * v.y) + (m[14] * v.z) + m[15];
		w = 1.0f / w;
		return (v1 * w);
	}
	Matrix operator=(const Matrix& matrix)
	{
		memcpy(m, matrix.m, sizeof(float) * 16);
		return (*this);
	}
	Matrix invert() // Unrolled inverse from MESA library
	{
		Matrix inv;
		inv[0] = m[5] * m[10] * m[15] -
			m[5] * m[11] * m[14] -
			m[9] * m[6] * m[15] +
			m[9] * m[7] * m[14] +
			m[13] * m[6] * m[11] -
			m[13] * m[7] * m[10];
		inv[4] = -m[4] * m[10] * m[15] +
			m[4] * m[11] * m[14] +
			m[8] * m[6] * m[15] -
			m[8] * m[7] * m[14] -
			m[12] * m[6] * m[11] +
			m[12] * m[7] * m[10];
		inv[8] = m[4] * m[9] * m[15] -
			m[4] * m[11] * m[13] -
			m[8] * m[5] * m[15] +
			m[8] * m[7] * m[13] +
			m[12] * m[5] * m[11] -
			m[12] * m[7] * m[9];
		inv[12] = -m[4] * m[9] * m[14] +
			m[4] * m[10] * m[13] +
			m[8] * m[5] * m[14] -
			m[8] * m[6] * m[13] -
			m[12] * m[5] * m[10] +
			m[12] * m[6] * m[9];
		inv[1] = -m[1] * m[10] * m[15] +
			m[1] * m[11] * m[14] +
			m[9] * m[2] * m[15] -
			m[9] * m[3] * m[14] -
			m[13] * m[2] * m[11] +
			m[13] * m[3] * m[10];
		inv[5] = m[0] * m[10] * m[15] -
			m[0] * m[11] * m[14] -
			m[8] * m[2] * m[15] +
			m[8] * m[3] * m[14] +
			m[12] * m[2] * m[11] -
			m[12] * m[3] * m[10];
		inv[9] = -m[0] * m[9] * m[15] +
			m[0] * m[11] * m[13] +
			m[8] * m[1] * m[15] -
			m[8] * m[3] * m[13] -
			m[12] * m[1] * m[11] +
			m[12] * m[3] * m[9];
		inv[13] = m[0] * m[9] * m[14] -
			m[0] * m[10] * m[13] -
			m[8] * m[1] * m[14] +
			m[8] * m[2] * m[13] +
			m[12] * m[1] * m[10] -
			m[12] * m[2] * m[9];
		inv[2] = m[1] * m[6] * m[15] -
			m[1] * m[7] * m[14] -
			m[5] * m[2] * m[15] +
			m[5] * m[3] * m[14] +
			m[13] * m[2] * m[7] -
			m[13] * m[3] * m[6];
		inv[6] = -m[0] * m[6] * m[15] +
			m[0] * m[7] * m[14] +
			m[4] * m[2] * m[15] -
			m[4] * m[3] * m[14] -
			m[12] * m[2] * m[7] +
			m[12] * m[3] * m[6];
		inv[10] = m[0] * m[5] * m[15] -
			m[0] * m[7] * m[13] -
			m[4] * m[1] * m[15] +
			m[4] * m[3] * m[13] +
			m[12] * m[1] * m[7] -
			m[12] * m[3] * m[5];
		inv[14] = -m[0] * m[5] * m[14] +
			m[0] * m[6] * m[13] +
			m[4] * m[1] * m[14] -
			m[4] * m[2] * m[13] -
			m[12] * m[1] * m[6] +
			m[12] * m[2] * m[5];
		inv[3] = -m[1] * m[6] * m[11] +
			m[1] * m[7] * m[10] +
			m[5] * m[2] * m[11] -
			m[5] * m[3] * m[10] -
			m[9] * m[2] * m[7] +
			m[9] * m[3] * m[6];
		inv[7] = m[0] * m[6] * m[11] -
			m[0] * m[7] * m[10] -
			m[4] * m[2] * m[11] +
			m[4] * m[3] * m[10] +
			m[8] * m[2] * m[7] -
			m[8] * m[3] * m[6];
		inv[11] = -m[0] * m[5] * m[11] +
			m[0] * m[7] * m[9] +
			m[4] * m[1] * m[11] -
			m[4] * m[3] * m[9] -
			m[8] * m[1] * m[7] +
			m[8] * m[3] * m[5];
		inv[15] = m[0] * m[5] * m[10] -
			m[0] * m[6] * m[9] -
			m[4] * m[1] * m[10] +
			m[4] * m[2] * m[9] +
			m[8] * m[1] * m[6] -
			m[8] * m[2] * m[5];
		float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
		if (det == 0)
		{
			// This code should never be called. Add error handling if it is
			identity();
			det = 1.0f;
		}
		det = 1.0f / det;
		for (int i = 0; i < 16; i++)
		{
			inv[i] = inv[i] * det;
		}
		return inv;
	}
	static Matrix lookAt(const Vec3& from, const Vec3& to, const Vec3& up)
	{
		Matrix mat;
		Vec3 dir = (from - to).normalize();
		Vec3 left = up.cross(dir).normalize();
		Vec3 newUp = dir.cross(left);
		mat.a[0][0] = left.x;
		mat.a[0][1] = left.y;
		mat.a[0][2] = left.z;
		mat.a[1][0] = newUp.x;
		mat.a[1][1] = newUp.y;
		mat.a[1][2] = newUp.z;
		mat.a[2][0] = dir.x;
		mat.a[2][1] = dir.y;
		mat.a[2][2] = dir.z;
		mat.a[0][3] = -from.dot(left);
		mat.a[1][3] = -from.dot(newUp);
		mat.a[2][3] = -from.dot(dir);
		mat.a[3][3] = 1;
		return mat;
	}
	static Matrix perspective(const float n, const float f, float aspect, const float fov) // FOV in degrees, outputs transposed Matrix for DX
	{
		Matrix pers;
		memset(pers.m, 0, sizeof(float) * 16);
		float t = 1.0f / (tanf(fov * 0.5f * 3.141592654f / 180.0f));
		pers.a[0][0] = t / aspect;
		pers.a[1][1] = t;
		pers.a[2][2] = -f / (f - n);
		pers.a[2][3] = -(f * n) / (f - n);
		pers.a[3][2] = -1.0f;
		return pers;
	}
	static Matrix rotateX(float theta)
	{
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);
		mat.m[5] = ct;
		mat.m[6] = st;
		mat.m[9] = -st;
		mat.m[10] = ct;
		return mat;
	}
	static Matrix rotateY(float theta)
	{
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);
		mat.m[0] = ct;
		mat.m[2] = -st;
		mat.m[8] = st;
		mat.m[10] = ct;
		return mat;
	}
	static Matrix rotateZ(float theta)
	{
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);
		mat.m[0] = ct;
		mat.m[1] = st;
		mat.m[4] = -st;
		mat.m[5] = ct;
		return mat;
	}
};

class Frame
{
public:
	Vec3 u;
	Vec3 v;
	Vec3 w;
	void fromVector(const Vec3& n)
	{
		// Gram-Schmit
		w = n.normalize();
		if (fabsf(w.x) > fabsf(w.y))
		{
			float l = 1.0f / sqrtf(w.x * w.x + w.z * w.z);
			u = Vec3(w.z * l, 0.0f, -w.x * l);
		} else
		{
			float l = 1.0f / sqrtf(w.y * w.y + w.z * w.z);
			u = Vec3(0, w.z * l, -w.y * l);
		}
		v = Cross(w, u);
	}
	void fromVectorTangent(const Vec3& n, const Vec3& t)
	{
		w = n.normalize();
		u = t.normalize();
		v = Cross(w, u);
	}
	Vec3 toLocal(const Vec3& vec) const
	{
		return Vec3(Dot(vec, u), Dot(vec, v), Dot(vec, w));
	}
	Vec3 toWorld(const Vec3& vec) const
	{
		return ((u * vec.x) + (v * vec.y) + (w * vec.z));
	}
};

class SphericalCoordinates
{
public:
	static Vec3 sphericalToWorld(float theta, float phi)
	{
		return Vec3(cosf(phi) * sinf(theta), sinf(phi) * sinf(theta), cosf(theta));
	}
	static float sphericalTheta(const Vec3& wi)
	{
		return acosf(wi.z);
	}
	static float sphericalPhi(const Vec3& wi)
	{
		float p = atan2f(wi.y, wi.x);
		return (p < 0.0f) ? p + (2.0f * M_PI) : p;
	}
};

template<typename T>
T& use()
{
	static T t;
	return t;
}