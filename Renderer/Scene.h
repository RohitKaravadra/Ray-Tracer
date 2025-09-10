#pragma once

#include "Sampling.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Bvh.h"
#include "Camera.h"

struct SurfaceData
{
	Vec3 p;			// Intersection position
	Vec3 n;			// Geometric normal
	float t;		// Ray parameter at intersection
	BSDF* bsdf;		// Material at the intersection
	int lightIndex; // Index of the light if this is a light source

	ShadingData shadingData;

	SurfaceData() = default;
	SurfaceData(Vec3 _p, Vec3 _n)
	{
		p = _p;
		n = _n;
		bsdf = NULL;
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

	SceneCamera camera;

	BVHTree bvh;
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
				light->init(&triangles[i], materials[triangles[i].materialIndex]->emission, lights.size());
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
	bool visible(const Vec3& p1, const Vec3& p2) const
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
	SurfaceData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		SurfaceData surfaceData;
		ShadingData& shadingData = surfaceData.shadingData;

		if (intersection.t < FLT_MAX)
		{
			surfaceData.lightIndex = triangles[intersection.ID].lightIndex;
			surfaceData.p = ray.at(intersection.t);
			surfaceData.n = triangles[intersection.ID].gNormal();
			surfaceData.bsdf = materials[triangles[intersection.ID].materialIndex];

			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.n, shadingData.uv);

			shadingData.wo = -ray.dir;

			if (surfaceData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.n) < 0)
					shadingData.n = -shadingData.n;
				if (Dot(shadingData.wo, surfaceData.n) < 0)
					surfaceData.n = -surfaceData.n;
			}
			shadingData.frame.fromVector(shadingData.n);
			surfaceData.t = intersection.t;
		}
		else
		{
			shadingData.wo = -ray.dir;
			surfaceData.t = intersection.t;
		}

		return surfaceData;
	}
};