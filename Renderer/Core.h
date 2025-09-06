#pragma once

// Do not change this code!

#define _USE_MATH_DEFINES
#include "math/vectors.h"
#include "math/matrix.h"

// Stop warnings about M_PI being a double
#pragma warning( disable : 4244)

#define SQ(x) (x * x)

template <typename T>
constexpr const T& clamp(const T& value, const T& low, const T& high)
{
	return (value < low) ? low : (value > high) ? high : value;
}

class Color
{
public:
	union
	{
		struct
		{
			float r, g, b;
		};
		float rgb[3];
	};

	Color() { r = 0; g = 0; b = 0; }
	Color(float a) {
		r = g = b = a;
	}
	Color(float _r, float _g, float _b)
	{
		r = _r;
		g = _g;
		b = _b;
	}
	Color(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a)
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
	Color operator+(const Color& colour) const
	{
		Color c;
		c.r = r + colour.r;
		c.g = g + colour.g;
		c.b = b + colour.b;
		return c;
	}
	Color operator-(const Color& colour) const
	{
		Color c;
		c.r = r - colour.r;
		c.g = g - colour.g;
		c.b = b - colour.b;
		return c;
	}
	Color operator*(const Color& colour) const
	{
		Color c;
		c.r = r * colour.r;
		c.g = g * colour.g;
		c.b = b * colour.b;
		return c;
	}
	Color operator*=(const Color& colour) const
	{
		Color c;
		c.r = r * colour.r;
		c.g = g * colour.g;
		c.b = b * colour.b;
		return c;
	}
	Color operator/(const Color& colour) const
	{
		Color c;
		c.r = r / colour.r;
		c.g = g / colour.g;
		c.b = b / colour.b;
		return c;
	}
	Color operator*(const float v) const
	{
		Color c;
		c.r = r * v;
		c.g = g * v;
		c.b = b * v;
		return c;
	}
	Color operator/(const float v) const
	{
		Color c;
		c.r = r / v;
		c.g = g / v;
		c.b = b / v;
		return c;
	}
	float Lum()
	{
		return ((0.2126f * r) + (0.7152f * g) + (0.0722f * b));
	}

	Color normalize()
	{
		float l = 1.0f / sqrtf((r * r) + (g * g) + (b * b));
		return Color(r * l, g * l, b * l);
	}
};

struct Vertex
{
	Vec3 p;
	Vec3 normal;
	float u;
	float v;
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
		}
		else
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