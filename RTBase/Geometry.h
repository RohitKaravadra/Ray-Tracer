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
		// find time of intersection
		t = (d - n.dot(r.o)) / (n.dot(r.dir));

		// check if intersection is in front of ray
		return t >= 0;
	}
};

#define EPSILON 0.001f

class Triangle
{
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

	const float epsilon = 1e-7f;

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
		Vec3 p = Cross(r.dir, e2);
		float det = p.dot(e1);

		// parellel ray check
		if (std::abs(det) < epsilon)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[2].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && abs(u) > epsilon) || (u > 1 && abs(u - 1) > epsilon))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && abs(v) > epsilon) || (u + v > 1 && abs(u + v - 1) > epsilon))
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
		float rootr1 = sqrt(r1);
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

class BVHTree
{
	struct Node
	{
		AABB bounds;
		unsigned int start, end;
		unsigned int child_l, child_r;

		Node(unsigned int start, unsigned int end) :start(start), end(end), child_l(0), child_r(0) {}
	};

	Triangle* triangles;		// list of triangles
	std::vector<Node> nodes;	// list of nodes
	unsigned int* indices;		// list of triangle indices for nodes

	const int maxDepth = 50;	// maximum depth of BVH

	float calcCost(float pArea, float lArea, float rArea, unsigned int lNum, unsigned int rNum)
	{
		return TRAVERSE_COST + TRIANGLE_COST * (lArea * lNum + rArea * rNum) / pArea;
	}

	float evaluateSplit(unsigned int node, unsigned int axis, float splitPos)
	{
		AABB boundA, boundB;				// bound of left and right child
		unsigned int numA = 0, numB = 0;	// number of triangles in left and right child

		// calculate bound of left and right child
		for (unsigned int i = nodes[node].start; i < nodes[node].end; i++)
		{
			unsigned int index = indices[i];

			float triCenter = triangles[index].centre.coords[axis];

			if (triCenter < splitPos)
			{
				boundA.extend(triangles[index].maxP);
				boundA.extend(triangles[index].minP);
				numA++;
			}
			else
			{
				boundB.extend(triangles[index].maxP);
				boundB.extend(triangles[index].minP);
				numB++;
			}
		}

		// calculate and return cost
		return calcCost(nodes[node].bounds.area(), boundA.area(), boundB.area(), numA, numB);
	}

	bool createSplit(unsigned int node, unsigned int axis, float splitPos)
	{
		unsigned int rStart = nodes[node].start;
		// split the triangles
		for (unsigned int i = nodes[node].start; i < nodes[node].end; i++)
		{
			float triCenter = triangles[indices[i]].centre.coords[axis];
			bool rChild = triCenter > splitPos;
			if (rChild)
			{
				std::swap(indices[i], indices[rStart]);
				rStart++;
			}
		}

		// check for no split
		if (rStart == nodes[node].start || rStart == nodes[node].end)
			return false;

		// create child nodes
		Node child_l(nodes[node].start, rStart);
		Node child_r(rStart, nodes[node].end);

		// calculate bounds of child nodes
		for (unsigned int i = child_l.start; i < child_l.end; i++)
		{
			child_l.bounds.extend(triangles[indices[i]].maxP);
			child_l.bounds.extend(triangles[indices[i]].minP);
		}
		for (unsigned int i = child_r.start; i < child_r.end; i++)
		{
			child_r.bounds.extend(triangles[indices[i]].maxP);
			child_r.bounds.extend(triangles[indices[i]].minP);
		}

		// add child nodes to node list
		nodes.emplace_back(child_l);
		nodes.emplace_back(child_r);

		// update parent nodes child indices
		nodes[node].child_l = nodes.size() - 2;
		nodes[node].child_r = nodes.size() - 1;

		return true;
	}

	bool splitNode(unsigned int node)
	{
		// check for max triangles in node (no more split)
		if (nodes[node].end - nodes[node].start <= MAXNODE_TRIANGLES)
			return false;

		const unsigned int numTest = 5;			// number of split test
		unsigned int bestAxis = 0;				// best axis to split
		float bestPos = 0;						// best position to split	
		float bestCost = FLT_MAX;				// best cost to split

		float invNumTest = 1 / (float)numTest;	// inverse of number of test

		// check for each axis
		for (int axis = 0; axis < 3; axis++)
		{
			// calculate bound size
			float boundsStart = nodes[node].bounds.min.coords[axis];
			float boundsEnd = nodes[node].bounds.max.coords[axis];

			// check for each split
			for (int i = 0; i < numTest; i++)
			{
				float splitT = (i + 1) * invNumTest;

				float pos = boundsStart + (boundsEnd - boundsStart) * splitT;	// calculate split position
				float cost = evaluateSplit(node, axis, pos);					// calculate cost

				if (cost < bestCost)											// update best cost
				{
					bestCost = cost;
					bestPos = pos;
					bestAxis = axis;
				}
			}
		}

		// create child nodes
		if (createSplit(node, bestAxis, bestPos))
			return true;

		return false;
	}

	unsigned int split(unsigned int node, int depth = 0)
	{
		// check for max depth reached (leaf node)
		if (depth >= maxDepth)
			return depth;

		// if no split possible return (leaf node)
		if (!splitNode(node))
			return depth;

		// recursive call to child splitting
		depth++;
		unsigned int depth1 = split(nodes[node].child_l, depth);
		unsigned int depth2 = split(nodes[node].child_r, depth);

		// return maximum depth (for debugging)
		return std::max(depth1, depth2);
	}

public:
	~BVHTree()
	{
		if (indices)
			delete[] indices;
	}

	void build(std::vector<Triangle>& inputTriangles, AABB bounds)
	{
		// set triangles
		triangles = &inputTriangles[0];

		std::cout << "Total Triangles in scene is " << inputTriangles.size() << std::endl;

		// clear data if any
		nodes.clear();

		// add root node to the data
		indices = new unsigned int[inputTriangles.size()];
		for (unsigned int i = 0; i < inputTriangles.size(); i++)
			indices[i] = i;

		nodes.emplace_back(Node(0, inputTriangles.size()));
		nodes[0].bounds = bounds;

		// split the root node (recursive call)
		unsigned int depth = split(0);

		// calculate statistics
		unsigned int trices = 0;
		unsigned int totalNodes = 0;
		unsigned int maxTrice = 0;

		for (auto& node : nodes)
		{
			// check for leaf node
			if (node.child_l == 0 && node.child_r == 0)
			{
				unsigned int tri = node.end - node.start;
				trices += tri;
				if (tri > maxTrice)
					maxTrice = tri;
				totalNodes++;
			}
		}

		// print statistics
		std::cout << "\n------: BVH Info :------\n";
		std::cout << "Total depth                 - " << depth << "/" << maxDepth << std::endl;
		std::cout << "Total nodes                 - " << nodes.size() << std::endl;
		std::cout << "Total Triangles             - " << trices << std::endl;
		std::cout << "Average triangle per node   - " << float(trices) / totalNodes << std::endl;
		std::cout << "Maximum triangles in a node - " << maxTrice << "\n\n";
	}

	void traverse(const Ray& ray, IntersectionData& intersection)
	{
		// check for intersection with root node for early exit
		if (!nodes[0].bounds.rayAABB(ray))
			return;

		std::stack<unsigned int> stack;	// stack for node traversal
		stack.push(0);					// push root node to stack	

		// traverse tree
		while (!stack.empty())
		{
			// pop top node from stack
			const Node& node = nodes[stack.top()];
			stack.pop();

			// check for leaf node to terminate recursion
			if (node.child_l == 0 && node.child_r == 0)
			{
				// check for intersection with triangles
				for (unsigned int i = node.start; i < node.end; i++)
				{
					float t, u, v;
					unsigned int index = indices[i];

					// check for intersection with triangle
					if (triangles[index].rayIntersect(ray, t, u, v))
					{
						// update intersection data
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
			}
			else
			{
				// recursive traversal call to child nodes
				if (nodes[node.child_l].bounds.rayAABB(ray))
					stack.push(node.child_l);
				if (nodes[node.child_r].bounds.rayAABB(ray))
					stack.push(node.child_r);
			}
		}
	}

	IntersectionData traverse(const Ray& ray)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;

		// traverse tree from root node
		traverse(ray, intersection);

		return intersection;
	}

	bool traverseVisible(const Ray& ray, const float maxT)
	{
		// check for intersection with root node for early exit
		if (!nodes[0].bounds.rayAABB(ray))
			return false;

		std::stack<unsigned int> stack;	// stack for node traversal
		stack.push(0);					// push root node to stack

		// traverse tree
		while (!stack.empty())
		{
			// pop top node from stack
			const Node& node = nodes[stack.top()];
			stack.pop();

			// check for leaf node to terminate recursion
			if (node.child_l == 0 && node.child_r == 0)
			{
				// check for intersection with triangles
				for (unsigned int i = node.start; i < node.end; i++)
				{
					float t, u, v;
					unsigned int index = indices[i];

					// check for intersection with triangle
					if (triangles[index].rayIntersect(ray, t, u, v))
						if (t <= maxT)
							return true;
				}
			}
			else
			{
				// recursive traversal call to child nodes
				if (nodes[node.child_l].bounds.rayAABB(ray))
					stack.push(node.child_l);
				if (nodes[node.child_r].bounds.rayAABB(ray))
					stack.push(node.child_r);
			}
		}

		return false;
	}
};
