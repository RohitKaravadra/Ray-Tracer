#pragma once

// Do not change this code!

#define _USE_MATH_DEFINES
#include "math/vectors.h"
#include "math/matrix.h"
#include "math/color.h"

constexpr auto EPSILON = 1e-3f;
constexpr auto EPSILON2 = 1e-7f;

// Stop warnings about M_PI being a double
#pragma warning( disable : 4244)

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