#pragma once

#include "Algorithm.h"

class InstantRadiosity : public AlgorithmBase
{
	struct PathData
	{
		Ray& r;
		Sampler*& sampler;
		Color pathThroughput;

		Color Le;

		PathData(Ray& _r, Sampler*& _sampler) : r(_r), sampler(_sampler)
		{
			pathThroughput = Color(1, 1, 1);
			Le = Color(0, 0, 0);
		}
	};

	// vertual point light
	struct VPL
	{
		ShadingData shadingData;
		Color Le;
	};

	std::vector<VPL> vpls;

	void VPLTracePath(PathData& path, std::vector<VPL>& vplList, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= data.settings.maxBounces)
			return;

		// Traverse the scene to find an intersection
		IntersectionData intersection = data.scene->traverse(path.r);
		ShadingData shadingData = data.scene->calculateShadingData(intersection, path.r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
				return;

			// Sample new direction
			Color bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, path.sampler, bsdf, pdf);

			// Update path throughput
			path.pathThroughput = path.pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			VPL vpl;
			vpl.shadingData = shadingData;
			vpl.Le = path.pathThroughput * path.Le;

			// update vpls list
			vplList.emplace_back(vpl);

			// Russian Roulette for termination
			float russianRouletteProbability = min(path.pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < path.sampler->next())
				return;

			path.pathThroughput = path.pathThroughput / russianRouletteProbability;

			// Create new ray
			path.r.init(shadingData.x + (wi * EPSILON), wi);

			// Continue tracing the path recursively
			VPLTracePath(path, vplList, depth + 1);
		}
	}

	void traceVPLs(unsigned int id, std::vector<VPL>& vplList)
	{
		Sampler* sampler = data.samplers[id];

		int total = data.settings.vplRaysPerTile * data.numThreads;

		for (unsigned int i = 0; i < data.settings.vplRaysPerTile; i++)
		{
			// Sample a light
			LightSample light = data.scene->sampleLight(sampler);
			float lightPdf = light.pdf * light.pmf;

			// sample direction from light
			float pdfDir;
			Vec3 wi = light.light->sampleDirectionFromLight(sampler, pdfDir);

			ShadingData shadingData;

			Color Le = light.emitted / (lightPdf * pdfDir);
			// normalize light if area light
			if (light.isArea)
				Le = Le * max(Dot(wi, light.n), 0);

			VPL vpl;
			vpl.shadingData = ShadingData(light.p, light.n);
			vpl.Le = Le;

			// update vpls list
			vplList.emplace_back(vpl);

			Ray ray(light.p + (wi * EPSILON), wi);
			Color pathThroughput(1.0f, 1.0f, 1.0f);

			PathData path(ray, sampler);
			path.Le = Le;
			VPLTracePath(path, vplList);
		}
	}

	Color radiosityComputeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Color(0.0f, 0.0f, 0.0f);
		}

		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, shadingData.wo);
		}

		Color accumulated(0.0f, 0.0f, 0.0f);

		unsigned int total = vpls.size();
		for (unsigned int i = 0; i < total; i++)
		{
			//unsigned int index = (unsigned int)(sampler->next() * (vpls.size() - 1));
			VPL vpl = vpls[i];

			// Calculate G Term
			Vec3 wi = vpl.shadingData.x - shadingData.x;
			float lengthSq = wi.lengthSq();
			wi = wi.normalize();

			float gTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) *
				max(-Dot(wi, vpl.shadingData.sNormal), 0.0f)) / lengthSq;

			// Shade if visible
			if (gTerm > 0 && data.scene->visible(shadingData.x, vpl.shadingData.x))
				accumulated = accumulated + shadingData.bsdf->evaluate(shadingData, wi) * vpl.Le * gTerm;
		}
		return accumulated / total;
	}

	float radiosityDebug(const Vec3& p, float& i)
	{
		const float rSq = SQ(0.03f);
		for (const VPL& vpl : vpls)
		{
			float lSq = (vpl.shadingData.x - p).lengthSq();
			if (lSq < rSq)
			{
				i = lSq / rSq;
				return true;
			}
		}
		return false;
	}

	Color radiosityLightPass(Ray r, Sampler* sampler)
	{
		// Traverse the scene to find an intersection
		IntersectionData intersection = data.scene->traverse(r);
		ShadingData shadingData = data.scene->calculateShadingData(intersection, r);

		if (data.settings.debug)
		{
			float i;
			if (radiosityDebug(shadingData.x, i))
				return Color(1.0f, 0.0f, 0.0f) * (1.0f - i) + Color(1.0f, 1.0f, 0.0f) * i;
		}

		return shadingData.t < FLT_MAX ? radiosityComputeDirect(shadingData, sampler) :
			data.scene->background->evaluate(r.dir);
	}

	void radiosityVplPass()
	{
		// list fo newly generated vpls by each thread
		std::vector<std::vector<VPL>> vplLists(data.numThreads);

		// generated new vpls using multi threading
		for (int i = 0; i < data.numThreads; i++)
			data.threads[i] = new std::thread(&InstantRadiosity::traceVPLs, this, i, std::ref(vplLists[i]));

		for (int i = 0; i < data.numThreads; i++)
		{
			data.threads[i]->join();
			delete data.threads[i];
		}

		// clear past vpls
		vpls.clear();

		// merge all vpls in single list
		for (const auto& v : vplLists) {
			vpls.insert(vpls.end(), v.begin(), v.end());
		}
	}

	void renderTile(const Vec2i& start, const Vec2i& end, const Sampler* sampler)
	{
		for (unsigned int y = start.y; y < end.y; y++)
		{
			for (unsigned int x = start.x; x < end.x; x++)
			{
				// jittered pixel coordinates
				Vec2 pixel = Vec2(x, y);
				pixel += Vec2(data.samplers[0]->next(),
					data.samplers[0]->next());
				Ray ray = data.scene->camera.generateRay(pixel);

				Color col = radiosityLightPass(ray, data.samplers[0]);
				data.film->splat(pixel, col);
			}
		}
	}

	void processTiles(unsigned int id)
	{
		unsigned int i;
		while ((i = data.tileCounter.fetch_add(1)) < data.totalTiles)
		{
			Vec2i start = Vec2i((i % data.totalXTiles), (i / data.totalXTiles));
			start *= data.tileSize;

			Vec2i end = start + Vec2i(data.tileSize, data.tileSize);
			end = end.Min(Vec2i(data.film->width, data.film->height));

			renderTile(start, end, data.samplers[id]);
		}
	}

	void renderMT()
	{
		data.tileCounter.store(0);

		// process all tiles
		for (int i = 0; i < data.numThreads; i++)
			data.threads[i] = new std::thread(&InstantRadiosity::processTiles, this, i);

		for (int i = 0; i < data.numThreads; i++)
		{
			data.threads[i]->join();
			delete data.threads[i];
		}
	}
public:
	InstantRadiosity(RENDERER_DATA& _data) : AlgorithmBase(_data) {
	}

	void render() override
	{
		data.film->incrementSPP();

		radiosityVplPass();

		if (data.settings.useMultithreading)
			renderMT();
		else
			renderTile(Vec2i(0, 0), Vec2i(data.film->width, data.film->height), data.samplers[0]);
	}
};