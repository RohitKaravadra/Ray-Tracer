#pragma once

#include "Geometry.h"
#include <iostream>
#include <atomic>
#include <thread>
#include <vector>
#include <stack>
#include <algorithm>
#include <cmath>

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;

	IntersectionData() : ID(0), t(FLT_MAX), alpha(0), beta(0), gamma(0) {}
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 15
#define PARALLEL_BUILD_THRESHOLD 1024

class BVHTree
{
	// bvh node structure
	struct Node
	{
		AABB bounds;
		unsigned int start;
		unsigned int end;
		unsigned int child_l;
		unsigned int child_r;
		unsigned int flags;			// 0: leaf, 1: internal

		Node() = default;
		Node(unsigned int start, unsigned int end) :
			start(start), end(end), child_l(0), child_r(0), flags(0) {
		}
	};

	Triangle* triangles;			// list of triangles
	std::vector<Node> nodes;		// list of nodes
	unsigned int* indices;			// list of triangle indices for nodes

	const int maxDepth = 50;
	const float invBuildBins = 1.0f / (float)BUILD_BINS;

	// Thread-safe node allocation
	std::atomic<unsigned int> nextNodeIndex{ 1 };

	float calcCost(float pArea, float lArea, float rArea, unsigned int lNum, unsigned int rNum)
	{
		if (pArea <= 0.0f) return FLT_MAX;
		return TRAVERSE_COST + TRIANGLE_COST * (lArea * lNum + rArea * rNum) / pArea;
	}

	// SAH-based split evaluation
	float evaluateSplitSAH(unsigned int node, unsigned int axis, float splitPos)
	{
		AABB leftBounds, rightBounds;
		unsigned int leftCount = 0, rightCount = 0;

		for (unsigned int i = nodes[node].start; i < nodes[node].end; i++)
		{
			unsigned int index = indices[i];
			Vec3 centroid = triangles[index].centre;

			if (centroid.coords[axis] < splitPos)
			{
				leftBounds.extend(triangles[index].maxP);
				leftBounds.extend(triangles[index].minP);
				leftCount++;
			}
			else
			{
				rightBounds.extend(triangles[index].maxP);
				rightBounds.extend(triangles[index].minP);
				rightCount++;
			}
		}

		if (leftCount == 0 || rightCount == 0)
			return FLT_MAX;

		return calcCost(nodes[node].bounds.area(),
			leftBounds.area(),
			rightBounds.area(),
			leftCount,
			rightCount);
	}

	bool createSplit(unsigned int node, unsigned int axis, float splitPos)
	{
		unsigned int leftStart = nodes[node].start;
		unsigned int rightStart = nodes[node].end;

		// Partition triangles
		unsigned int i = leftStart;
		unsigned int j = rightStart - 1;

		while (i <= j)
		{
			while (i < rightStart &&
				triangles[indices[i]].centre.coords[axis] < splitPos)
			{
				i++;
			}

			while (j > leftStart &&
				triangles[indices[j]].centre.coords[axis] >= splitPos)
			{
				j--;
			}

			if (i <= j)
			{
				std::swap(indices[i], indices[j]);
				i++;
				j--;
			}
		}

		unsigned int splitIndex = i;
		if (splitIndex == nodes[node].start || splitIndex == nodes[node].end)
			return false;

		// Allocate nodes atomically
		unsigned int leftIndex = nextNodeIndex.fetch_add(2);
		unsigned int rightIndex = leftIndex + 1;

		// Ensure we have enough space
		if (rightIndex >= nodes.size())
		{
			nodes.resize(rightIndex + 1);
		}

		// Create child nodes
		Node leftChild(nodes[node].start, splitIndex);
		Node rightChild(splitIndex, nodes[node].end);

		// Compute bounds for left child
		for (unsigned int k = leftChild.start; k < leftChild.end; k++)
		{
			unsigned int index = indices[k];
			leftChild.bounds.extend(triangles[index].maxP);
			leftChild.bounds.extend(triangles[index].minP);
		}

		// Compute bounds for right child
		for (unsigned int k = rightChild.start; k < rightChild.end; k++)
		{
			unsigned int index = indices[k];
			rightChild.bounds.extend(triangles[index].maxP);
			rightChild.bounds.extend(triangles[index].minP);
		}

		nodes[leftIndex] = leftChild;
		nodes[rightIndex] = rightChild;

		// Update parent
		nodes[node].child_l = leftIndex;
		nodes[node].child_r = rightIndex;
		nodes[node].flags = 1;		// Mark as internal node

		return true;
	}

	// Parallel splitting function
	void splitNodeParallel(unsigned int node, int depth)
	{
		if (depth >= maxDepth ||
			(nodes[node].end - nodes[node].start) <= MAXNODE_TRIANGLES)
			return;

		// Find best split using SAH
		unsigned int bestAxis = 0;
		float bestPos = 0;
		float bestCost = FLT_MAX;

		for (int axis = 0; axis < 3; axis++)
		{
			float boundsStart = nodes[node].bounds.min.coords[axis];
			float boundsEnd = nodes[node].bounds.max.coords[axis];
			float boundsSize = boundsEnd - boundsStart;

			if (boundsSize <= 0.0f) continue;

			for (int i = 1; i < BUILD_BINS; i++)
			{
				float splitPos = boundsStart + boundsSize * (i * invBuildBins);
				float cost = evaluateSplitSAH(node, axis, splitPos);

				if (cost < bestCost)
				{
					bestCost = cost;
					bestPos = splitPos;
					bestAxis = axis;
				}
			}
		}

		if (bestCost == FLT_MAX) return;

		if (createSplit(node, bestAxis, bestPos))
		{
			unsigned int leftChild = nodes[node].child_l;
			unsigned int rightChild = nodes[node].child_r;

			// Recursive splitting - use threads for large nodes
			unsigned int leftSize = nodes[leftChild].end - nodes[leftChild].start;
			unsigned int rightSize = nodes[rightChild].end - nodes[rightChild].start;

			if (leftSize > PARALLEL_BUILD_THRESHOLD && rightSize > PARALLEL_BUILD_THRESHOLD)
			{
				std::thread leftThread([&]() { splitNodeParallel(leftChild, depth + 1); });
				std::thread rightThread([&]() { splitNodeParallel(rightChild, depth + 1); });
				leftThread.join();
				rightThread.join();
			}
			else if (leftSize > PARALLEL_BUILD_THRESHOLD)
			{
				std::thread leftThread([&]() { splitNodeParallel(leftChild, depth + 1); });
				splitNodeParallel(rightChild, depth + 1);
				leftThread.join();
			}
			else if (rightSize > PARALLEL_BUILD_THRESHOLD)
			{
				std::thread rightThread([&]() { splitNodeParallel(rightChild, depth + 1); });
				splitNodeParallel(leftChild, depth + 1);
				rightThread.join();
			}
			else
			{
				splitNodeParallel(leftChild, depth + 1);
				splitNodeParallel(rightChild, depth + 1);
			}
		}
	}

	// Single-threaded splitting for comparison
	unsigned int splitSingleThreaded(unsigned int node, int depth = 0)
	{
		if (depth >= maxDepth)
			return depth;

		if ((nodes[node].end - nodes[node].start) <= MAXNODE_TRIANGLES)
			return depth;

		// Find best split using SAH
		unsigned int bestAxis = 0;
		float bestPos = 0;
		float bestCost = FLT_MAX;

		for (int axis = 0; axis < 3; axis++)
		{
			float boundsStart = nodes[node].bounds.min.coords[axis];
			float boundsEnd = nodes[node].bounds.max.coords[axis];
			float boundsSize = boundsEnd - boundsStart;

			if (boundsSize <= 0.0f) continue;

			for (int i = 1; i < BUILD_BINS; i++)
			{
				float splitPos = boundsStart + boundsSize * (i * invBuildBins);
				float cost = evaluateSplitSAH(node, axis, splitPos);

				if (cost < bestCost)
				{
					bestCost = cost;
					bestPos = splitPos;
					bestAxis = axis;
				}
			}
		}

		if (bestCost == FLT_MAX) return depth;

		if (createSplit(node, bestAxis, bestPos))
		{
			depth++;
			unsigned int depth1 = splitSingleThreaded(nodes[node].child_l, depth);
			unsigned int depth2 = splitSingleThreaded(nodes[node].child_r, depth);
			return std::max(depth1, depth2);
		}

		return depth;
	}

public:
	~BVHTree()
	{
		if (indices)
			delete[] indices;
	}

	void build(std::vector<Triangle>& inputTriangles, AABB bounds, bool useParallel = true)
	{
		triangles = &inputTriangles[0];
		unsigned int numTriangles = inputTriangles.size();

		std::cout << "Total Triangles in scene     : " << numTriangles << std::endl;
		std::cout << "Building BVH.................." << std::endl;

		// Clear data if any
		nodes.clear();
		nextNodeIndex.store(1);

		// Pre-allocate nodes array
		nodes.resize(numTriangles * 2);

		// Create indices array
		indices = new unsigned int[numTriangles];
		for (unsigned int i = 0; i < numTriangles; i++)
			indices[i] = i;

		// Create root node
		nodes[0] = Node(0, numTriangles);
		nodes[0].bounds = bounds;

		// Compute root bounds
		for (unsigned int i = 0; i < numTriangles; i++)
		{
			nodes[0].bounds.extend(triangles[i].maxP);
			nodes[0].bounds.extend(triangles[i].minP);
		}

		float buildTime = (float)clock() / CLOCKS_PER_SEC;

		// Build the tree
		if (useParallel && numTriangles > PARALLEL_BUILD_THRESHOLD)
			splitNodeParallel(0, 0);
		else
			splitSingleThreaded(0);

		buildTime = ((float)clock() / CLOCKS_PER_SEC) - buildTime;

		// Trim nodes to actual size
		unsigned int actualNodeCount = nextNodeIndex.load();
		nodes.resize(actualNodeCount);

		// Calculate statistics
		unsigned int trices = 0;
		unsigned int totalNodes = 0;
		unsigned int maxTrice = 0;
		unsigned int leafNodes = 0;

		for (auto& node : nodes)
		{
			if (node.flags == 0) // Leaf node
			{
				unsigned int tri = node.end - node.start;
				trices += tri;
				if (tri > maxTrice)
					maxTrice = tri;
				totalNodes++;
				leafNodes++;
			}
		}

		// Print statistics
		std::cout << "\n  -----------: BVH Info :-----------  \n";
		std::cout << "Total nodes                  : " << nodes.size() << std::endl;
		std::cout << "Leaf nodes                   : " << leafNodes << std::endl;
		std::cout << "Total Triangles              : " << trices << std::endl;
		std::cout << "Average triangles per node   : " << float(trices) / leafNodes << std::endl;
		std::cout << "Maximum triangles in a node  : " << maxTrice << std::endl;
		std::cout << "BVH build time               : " << buildTime << " seconds\n";
		std::cout << "Build method                 : " << (useParallel ? "Parallel" : "Single-threaded") << "\n\n";
	}

	void traverse(const Ray& ray, IntersectionData& intersection) const
	{
		if (!nodes[0].bounds.rayAABB(ray))
			return;

		// Use thread-local stack for better performance
		thread_local std::vector<unsigned int> stack;
		stack.clear();
		stack.reserve(64);
		stack.push_back(0);

		while (!stack.empty())
		{
			unsigned int nodeIndex = stack.back();
			stack.pop_back();
			const Node& node = nodes[nodeIndex];

			if (node.flags == 0) // Leaf node
			{
				for (unsigned int i = node.start; i < node.end; i++)
				{
					unsigned int index = indices[i];
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
			}
			else // Internal node
			{
				unsigned int left = node.child_l;
				unsigned int right = node.child_r;

				float tLeft, tRight;
				bool hitLeft = nodes[left].bounds.rayAABB(ray, tLeft);
				bool hitRight = nodes[right].bounds.rayAABB(ray, tRight);

				// Push farther node first (process closer ones first)
				if (hitLeft && hitRight)
				{
					if (tLeft < tRight)
					{
						stack.push_back(right);
						stack.push_back(left);
					}
					else
					{
						stack.push_back(left);
						stack.push_back(right);
					}
				}
				else if (hitLeft)
				{
					stack.push_back(left);
				}
				else if (hitRight)
				{
					stack.push_back(right);
				}
			}
		}
	}

	IntersectionData traverse(const Ray& ray) const
	{
		IntersectionData intersection;
		traverse(ray, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const float maxT) const
	{
		if (!nodes[0].bounds.rayAABB(ray))
			return true;

		thread_local std::vector<unsigned int> stack;
		stack.clear();
		stack.reserve(64);
		stack.push_back(0);

		while (!stack.empty())
		{
			unsigned int nodeIndex = stack.back();
			stack.pop_back();
			const Node& node = nodes[nodeIndex];

			if (node.flags == 0) // Leaf node
			{
				for (unsigned int i = node.start; i < node.end; i++)
				{
					unsigned int index = indices[i];
					float t, u, v;

					if (triangles[index].rayIntersect(ray, t, u, v))
						if (t <= maxT)
							return false;
				}
			}
			else // Internal node
			{
				unsigned int left = node.child_l;
				unsigned int right = node.child_r;

				if (nodes[left].bounds.rayAABB(ray))
					stack.push_back(left);
				if (nodes[right].bounds.rayAABB(ray))
					stack.push_back(right);
			}
		}

		return true;
	}
};