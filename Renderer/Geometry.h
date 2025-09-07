#pragma once

#include "Core.h"
#include "Sampling.h"
#include <stack>

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		// find time of intersection
		t = (d - n.dot(r.o)) / (n.dot(r.dir));

		// check if intersection is in front of ray
		return t >= 0;
	}
};

#define EPSILON 0.001f

class Triangle
{
	Vec3 invertNormal(const Vec3& n)
	{
		// Create a world frame from normal vector
		Frame tangentSpace;
		tangentSpace.fromVector(n);

		return tangentSpace.toWorld(Vec3(0, 0, -1)); // Invert Z direction
	}

public:
	Vertex vertices[3];

	Vec3 e1;	// Edge 1
	Vec3 e2;	// Edge 2

	Vec3 n;		// Geometric Normal
	float area; // Triangle area
	float d;	// For ray triangle if needed

	unsigned int materialIndex;

	// precomputed values for AABB
	Vec3 centre;
	Vec3 maxP;
	Vec3 minP;

	int lightIndex; // Index of the light if this triangle is a light source

	float epsilon = 1e-8f;

	Triangle()
	{
		materialIndex = 0;
		e1 = Vec3(0, 0, 0);
		e2 = Vec3(0, 0, 0);
		n = Vec3(0, 0, 0);
		d = 0.0f;
		area = 0.0f;
		maxP = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		minP = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	Triangle(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		lightIndex = -1; // Default to no light source

		materialIndex = _materialIndex;

		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;

		e1 = vertices[0].p - vertices[2].p;
		e2 = vertices[1].p - vertices[2].p;

		n = e1.cross(e2).normalize();
		d = Dot(n, vertices[0].p);
		area = e1.cross(e2).length() * 0.5f;

		maxP = Max(vertices[0].p, Max(vertices[1].p, vertices[2].p));
		minP = Min(vertices[0].p, Min(vertices[1].p, vertices[2].p));
		centre = minP + (maxP - minP) * 0.5f;
	}

	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;

		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;

		e1 = vertices[0].p - vertices[2].p;
		e2 = vertices[1].p - vertices[2].p;

		n = e1.cross(e2).normalize();
		d = Dot(n, vertices[0].p);
		area = e1.cross(e2).length() * 0.5f;

		maxP = Max(vertices[0].p, Max(vertices[1].p, vertices[2].p));
		minP = Min(vertices[0].p, Min(vertices[1].p, vertices[2].p));
		centre = minP + (maxP - minP) * 0.5f;
	}

	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		// moeller trumbore intersection algorithm

		Vec3 p = Cross(r.dir, e2);
		float det = p.dot(e1);

		// parellel ray check
		if (std::fabs(det) < epsilon)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[2].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && fabs(u) > epsilon) || (u > 1 && fabs(u - 1) > epsilon))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && fabs(v) > epsilon) || (u + v > 1 && fabs(u + v - 1) > epsilon))
			return false;

		t = e2.dot(p) * invDet;

		if (t < epsilon)
			return false;

		return true;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}

	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		// generate random samples
		float r1 = sampler->next();
		float r2 = sampler->next();

		// calculate barycentric coordinates
		float rootr1 = sqrtf(r1);
		float alpha = 1 - rootr1;
		float beta = r2 * rootr1;
		float gamma = 1 - (alpha + beta);

		// calculate interpolated coordinates 
		Vec3 p = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		// calculate pdf
		pdf = 1 / area;

		return p;
	}

	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}

	bool isOnEdge(const Vec3& a, const Vec3& b, const Vec3& p, float w) const
	{
		Vec3 ab = b - a;
		Vec3 ap = p - a;

		float abLenSq = ab.lengthSq();
		if (abLenSq < epsilon) return false; // degenerate edge

		Vec3 cross = ab.cross(ap);
		float distanceSq = cross.lengthSq() / abLenSq;

		// If point is too far from the line, it's not on the "wide" edge
		if (distanceSq > SQ(w)) {
			return false;
		}

		// Dot product check: is point between a and b?
		float dot = ab.dot(ap);
		if (dot < 0.0f || dot > abLenSq) {
			return false;
		}

		return true; // Within edge bounds and close enough to the line
	}

	bool isOnEdge(const Vec3& p, float w = 0.01f) const
	{
		// Check if the point is on any of the triangle's edges
		return isOnEdge(vertices[0].p, vertices[1].p, p, w) ||
			isOnEdge(vertices[1].p, vertices[2].p, p, w) ||
			isOnEdge(vertices[2].p, vertices[0].p, p, w);
	}

	void invertNormals()
	{
		vertices[0].normal = invertNormal(vertices[0].normal);
		vertices[1].normal = invertNormal(vertices[1].normal);
		vertices[2].normal = invertNormal(vertices[2].normal);
		n = invertNormal(n);
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	Vec3 centre;

	AABB()
	{
		reset();
	}

	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);

		centre = min + (max - min) * 0.5f;
	}

	void extend(const AABB& b)
	{
		extend(b.max);
		extend(b.min);
	}

	// Add code here
	bool rayAABB(const Ray& r, float& t) const
	{
		// find points of intersection
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		// swaping for correct entry and exit points
		Vec3 entry = Min(tmin, tmax);
		Vec3 exit = Max(tmin, tmax);

		// find time of intersection for entry and exit point
		float tentry = max(max(entry.x, entry.y), entry.z);
		float texit = min(min(exit.x, exit.y), exit.z);

		if (tentry > texit || texit < 0)
			return false;

		t = (tentry >= 0) ? tentry : texit;		// handle case where ray origin is inside of bounding box
		return true;
	}

	// Add code here
	bool rayAABB(const Ray& r) const
	{
		// find points of intersection
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		// swaping for correct entry and exit points
		Vec3 entry = Min(tmin, tmax);
		Vec3 exit = Max(tmin, tmax);

		// find time of intersection for entry and exit point
		float tentry = max(max(entry.x, entry.y), entry.z);
		float texit = min(min(exit.x, exit.y), exit.z);

		return !(tentry > texit || texit < 0);
	}

	// Add code here
	float area() const
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}

	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float a = r.dir.dot(r.dir);
		float b = 2 * r.dir.dot(l);
		float c = l.dot(l) - SQ(radius);

		float dis = b * b - 4 * a * c;

		if (dis < 0)						// no intersection
			return false;
		else if (dis == 0)					// one intersection
			t = -0.5 * b / a;
		else								// two intersection
		{
			float q = (b > 0) ?
				-0.5 * (b + sqrtf(dis)) :
				-0.5 * (b - sqrtf(dis));

			t = min(q / a, c / q);
		}

		return true;
	}
};
