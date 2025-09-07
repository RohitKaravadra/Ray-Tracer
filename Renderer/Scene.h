#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Bvh.h"

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

class Scene
{

	Light* sampleLight(Sampler* sampler, float& pmf) const
	{
		pmf = 1 / (float)lights.size();		// probability mass function
		unsigned int i = min(sampler->next() * lights.size(), lights.size() - 1.0f);
		return lights[i];
	}

public:
	std::vector<Triangle> triangles;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;
	Light* background = NULL;
	BVHTree bvh;
	SceneCamera camera;
	AABB bounds;

	~Scene()
	{
		delete background;
	}

	void build()
	{
		// Add BVH building code here
		bvh.build(triangles, bounds);

		// Do not touch the code below this line!
		// Build light list
		for (int i = 0; i < triangles.size(); i++)
		{
			if (materials[triangles[i].materialIndex]->isLight())
			{
				AreaLight* light = new AreaLight();
				light->init(&triangles[i], lights.size(),
					materials[triangles[i].materialIndex]->emission);
				lights.push_back(light);
			}
		}
	}

	IntersectionData traverseAll(const Ray& ray)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		for (int i = 0; i < triangles.size(); i++)
		{
			float t;
			float u;
			float v;
			if (triangles[i].rayIntersect(ray, t, u, v))
			{
				if (t < intersection.t)
				{
					intersection.t = t;
					intersection.ID = i;
					intersection.alpha = u;
					intersection.beta = v;
					intersection.gamma = 1.0f - (u + v);
				}
			}
		}
		return intersection;
	}

	IntersectionData traverse(const Ray& ray)
	{
		//return traverseAll(ray);
		return bvh.traverse(ray);
	}

	LightSample sampleLight(Sampler* sampler) const
	{
		LightSample sample;
		sample.light = sampleLight(sampler, sample.pmf);

		sample.isNull = sample.light == nullptr;
		if (!sample.isNull)
		{
			sample.isArea = sample.light->isArea();
			sample.p = sample.light->sample(sampler, sample.emitted, sample.pdf);

			// adjust position if not an area light (i.e background light)
			if (!sample.isArea)
				sample.p = use<SceneBounds>().sceneCentre + (sample.p * use<SceneBounds>().sceneRadius);

			// get the normal at the sample position
			// if area light , use the triangle normal
			sample.n = sample.light->normal(sample.p.normalize());
		}

		return sample;
	}

	float getLightPdf(const int& lightIndex, const Vec3& wi) const
	{
		if (lightIndex < 0 || lightIndex >= lights.size())
			return 0.0f;

		return lights[lightIndex]->PDF(wi) / lights.size();
	}

	float getLightPdf(const Vec3& wi) const
	{
		float backgroundPdf = background->PDF(wi);
		if (lights.size() > 1)
			backgroundPdf /= lights.size();

		return backgroundPdf;
	}

	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background)
	{
		for (int i = 0; i < meshTriangles.size(); i++)
		{
			triangles.push_back(meshTriangles[i]);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}
		for (int i = 0; i < meshMaterials.size(); i++)
		{
			materials.push_back(meshMaterials[i]);
		}
		background = _background;
		if (background->totalIntegratedPower() > 0)
		{
			lights.push_back(background);
		}
	}
	bool visible(const Vec3& p1, const Vec3& p2)
	{
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length() - (2.0f * EPSILON);
		dir = dir.normalize();
		ray.init(p1 + (dir * EPSILON), dir);
		return bvh.traverseVisible(ray, maxT);
	}
	Color emit(Triangle* light, ShadingData shadingData, Vec3 wi)
	{
		return materials[light->materialIndex]->emit(shadingData, wi);
	}
	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX)
		{
			shadingData.lightIndex = triangles[intersection.ID].lightIndex;
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.sNormal) < 0)
				{
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0)
				{
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		}
		else
		{
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};