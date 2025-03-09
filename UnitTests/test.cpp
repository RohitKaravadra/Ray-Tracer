#include "pch.h"

TEST(AABBTest, RayIntersectsAABB)
{
	AABB aabb;
	aabb.min = Vec3(1, 1, 1);
	aabb.max = Vec3(3, 3, 3);
	Ray ray(Vec3(0, 0, 0), Vec3(1, 1, 1));

	// Test: Ray (0, 0, 0) to (1, 1, 1) should intersect the AABB
	ASSERT_TRUE(aabb.rayAABB(ray));
}

TEST(AABBTest, RayDoesNotIntersectAABB)
{
	AABB aabb;
	aabb.min = Vec3(1, 1, 1);
	aabb.max = Vec3(3, 3, 3);
	Ray ray(Vec3(0, 0, 0), Vec3(-1, -1, -1));

	// Test: Ray (0, 0, 0) to (-1, -1, -1) should not intersect the AABB
	ASSERT_FALSE(aabb.rayAABB(ray));
}

TEST(AABBTest, RayOriginInsideAABB)
{
	AABB aabb;
	aabb.min = Vec3(1, 1, 1);
	aabb.max = Vec3(3, 3, 3);
	Ray ray(Vec3(2, 2, 2), Vec3(1, 1, 1));

	// Test: Ray origin inside AABB (should intersect)
	ASSERT_TRUE(aabb.rayAABB(ray));
}

TEST(RaySphere, RayIntersectsSphere)
{
	Sphere s;
	s.init(Vec3(3, 3, 3), 1);
	Ray ray(Vec3(0, 0, 0), Vec3(1, 1, 1));
	float t;

	// Test: Ray origin inside AABB (should intersect)
	ASSERT_TRUE(s.rayIntersect(ray, t));
}

TEST(RaySphere, RayDoesNotIntersectsSphere)
{
	Sphere s;
	s.init(Vec3(3, 3, 3), 1);
	Ray ray(Vec3(0, 0, 0), Vec3(-1, 1, 1));
	float t;

	// Test: Ray origin inside AABB (should intersect)
	ASSERT_FALSE(s.rayIntersect(ray, t));
}

TEST(AABBTest, CorrectCenter)
{
	AABB aabb;
	aabb.min = Vec3(0, 0, 0);
	aabb.max = Vec3(4, 4, 4);
	// Test: Ray origin inside AABB (should intersect)
	ASSERT_TRUE(aabb.center() == Vec3(2, 2, 2));
}

TEST(AABBTest, IncorrectCenter)
{
	AABB aabb;
	aabb.min = Vec3(0, 0, 0);
	aabb.max = Vec3(5, 5, 5);
	// Test: Ray origin inside AABB (should intersect)
	ASSERT_FALSE(aabb.center() == Vec3(2, 2, 2));
}