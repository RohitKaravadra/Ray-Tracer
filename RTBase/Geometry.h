#pragma once

#include "Core.h"
#include "Sampling.h"
#include <conio.h>
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
		t = (d - n.dot(r.o)) / (n.dot(r.dir));
		return t >= 0;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;

	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[1].p - vertices[0].p;
		e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}

	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 p = Cross(r.dir, e2);
		float det = p.dot(e1);

		// parellel ray check
		if (std::abs(det) < EPSILON)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[0].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
			return false;

		t = e2.dot(p) * invDet;

		if (t < EPSILON)
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
		return Vec3(0, 0, 0);
	}

	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;

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
	}

	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		// find points of intersection
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		// swaping for correct entry and exit points
		Vec3 entry = Min(tmin, tmax);
		Vec3 exit = Max(tmin, tmax);

		// find time of intersection for entry and exit point
		float tentry = std::max(std::max(entry.x, entry.y), entry.z);
		float texit = std::min(std::min(exit.x, exit.y), exit.z);

		if (tentry > texit || texit < 0)
			return false;

		t = (tentry >= 0) ? tentry : texit;		// handle case where ray origin is inside of bounding box
		return true;
	}

	// Add code here
	bool rayAABB(const Ray& r)
	{
		// find points of intersection
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		// swaping for correct entry and exit points
		Vec3 entry = Min(tmin, tmax);
		Vec3 exit = Max(tmin, tmax);

		// find time of intersection for entry and exit point
		float tentry = std::max(std::max(entry.x, entry.y), entry.z);
		float texit = std::min(std::min(exit.x, exit.y), exit.z);

		if (tentry > texit || texit < 0)
			return false;

		return true;
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}

	Vec3 center() const { return min + (max - min) / 2.0f; }
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
				-0.5 * (b + sqrt(dis)) :
				-0.5 * (b - sqrt(dis));

			t = std::min(q / a, c / q);
		}

		return true;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

struct BVHNode
{
	AABB bounds;
	std::vector<int> triIndices;
	unsigned int child_l, child_r;

	BVHNode() :child_l(0), child_r(0) {}

	bool isLeaf() const { return child_l == 0 && child_r == 0; }

	void addTringle(int index, Triangle& tri)
	{
		bounds.extend(tri.vertices[0].p);
		bounds.extend(tri.vertices[1].p);
		bounds.extend(tri.vertices[2].p);
		triIndices.emplace_back(index);
	}
};

class BVH
{
	Triangle* triangles;
	std::vector<BVHNode> data;
	const int maxDepth = 22;

	bool splitNode(unsigned int node, BVHNode& child_l, BVHNode& child_r)
	{
		// get aabb bounds data
		Vec3 boundsCenter = data[node].bounds.center();
		Vec3 boundsSize = data[node].bounds.max - data[node].bounds.min;

		//find max axis for bounding box
		int axis = 0;
		if (boundsSize.coords[axis] < boundsSize.coords[1])
			axis = 1;
		if (boundsSize.coords[axis] < boundsSize.coords[2])
			axis = 2;

		// splitting triangles into child nodes
		for (int index : data[node].triIndices)
		{
			Vec3 triCenter = triangles[index].centre();
			BVHNode& child = triCenter.coords[axis] < boundsCenter.coords[axis] ? child_l : child_r;
			child.addTringle(index, triangles[index]);
		}

		// check if both nodes have some triangles if not no split happened
		return !(child_l.triIndices.empty() || child_r.triIndices.empty());
	}

	void split(unsigned int node, std::vector<Triangle>& triangles, int depth)
	{
		// check for max depth reached (leaf node)
		if (depth >= maxDepth)
		{
			std::cout << "Child Node with " << data[node].triIndices.size() << " Triangles\n";
			return;
		}

		// create child nodes
		BVHNode child_l, child_r;
		if (!splitNode(node, child_l, child_r))
		{
			std::cout << "Child Node with " << data[node].triIndices.size() << " Triangles\n";
			return;
		}

		std::cout << child_l.triIndices.size() << ":" << child_r.triIndices.size() << "->";

		// update data
		data.emplace_back(child_l);
		data.emplace_back(child_r);

		// update nodes child indices
		data[node].child_l = data.size() - 2;
		data[node].child_r = data.size() - 1;

		// recursive call to child splitting
		depth++;
		split(data[node].child_l, triangles, depth);
		split(data[node].child_r, triangles, depth);

		// clear this nodes triangle indices if not leaf node
		data[node].triIndices.clear();
	}

public:

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, AABB bounds)
	{
		// set triangles
		triangles = &inputTriangles[0];

		// clear data if any
		data.clear();

		// add root node to the data
		data.emplace_back(BVHNode());
		data[0].bounds = bounds;
		for (int i = 0; i < inputTriangles.size(); i++)
			data[0].triIndices.emplace_back(i);

		// begin recursive split operation
		split(0, inputTriangles, 0);
		std::cout << "BVH construction complete with " << data[0].triIndices.size() << " triangles\n";
	}

	void traverse(unsigned int node, const Ray& ray, IntersectionData& intersection)
	{
		// check for leaf node to terminate recursion
		if (data[node].isLeaf())
		{
			// check all triangles in leaf node
			for (int index : data[node].triIndices)
			{
				float t, u, v;
				if (triangles[index].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = index;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}

		// recursive traversal call to child nodes
		traverse(data[node].child_l, ray, intersection);
		traverse(data[node].child_r, ray, intersection);
	}

	IntersectionData traverse(const Ray& ray)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;

		// traverse tree from root node
		traverse(0, ray, intersection);

		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
