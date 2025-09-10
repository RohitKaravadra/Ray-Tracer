#pragma once
#include "Geometry.h"

class SceneCamera
{
public:
	Matrix projMat;		// Projection matrix
	Matrix viewMat;		// View matrix

	Matrix invProjMat;	// Inverse of projection matrix
	Matrix invViewMat;	// Inverse of view matrix

	Vec2 size; // Size of the screen in pixels

	Vec3 pos;	// Camera position in world space
	Vec3 dir;	// Camera forward direction in world space

	float Afilm;	// Area of the film

	void init(Matrix ProjectionMatrix, Vec2i _size)
	{
		projMat = ProjectionMatrix;
		invProjMat = ProjectionMatrix.invert();
		size = Vec2(_size.x, _size.y);

		float Wlens = (2.0f / ProjectionMatrix.a[1][1]);
		float aspect = ProjectionMatrix.a[0][0] / ProjectionMatrix.a[1][1];
		float Hlens = Wlens * aspect;
		Afilm = Wlens * Hlens;
	}

	void updateView(Matrix V)
	{
		viewMat = V;
		invViewMat = V.invert();
		pos = viewMat.mulPoint(Vec3(0, 0, 0));
		dir = invProjMat.mulPointAndPerspectiveDivide(Vec3(0, 0, 1));
		dir = viewMat.mulVec(dir);
		dir = dir.normalize();
	}

	Ray generateRay(const Vec2& p)
	{
		Vec2 prime = p / size;
		prime.y = 1.0f - prime.y;		// flip y coordinate
		prime = (prime * 2.0f) - 1.0f;	// NDC space

		Vec3 dir(prime.x, prime.y, 1.0f);	// point on near plane in view space

		dir = invProjMat.mulPoint(dir);
		dir = viewMat.mulVec(dir);

		return Ray(pos, dir.normalize());
	}

	bool projectOntoCamera(const Vec3& p, Vec2& sp)
	{
		Vec3 pview = invViewMat.mulPoint(p);
		Vec3 pproj = projMat.mulPointAndPerspectiveDivide(pview);

		sp = (Vec2(pproj.x, pproj.y) + 1.0f) * 0.5f;

		if (sp < 0 || sp > 1)
			return false;

		sp.y = 1.0f - sp.y;
		sp *= size;

		return true;
	}
};