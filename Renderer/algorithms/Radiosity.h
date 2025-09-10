#pragma once

#include "Algorithm.h"


/// <summary>
/// Instant Radiosity implementation.
/// </summary>
class Radiosity : public AlgorithmBase
{
	/// <summary>
	/// Path data structure to hold ray, sampler, and path throughput information.
	/// </summary>
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

	/// <summary>
	/// VPL (Virtual Point Light) structure to hold shading data and emitted radiance.
	/// </summary>
	struct VPL
	{
		Vec3 p;
		ShadingData shadingData;
		Color Le;
	};

	const float minDistSqPerc = 0.01f;	// percentage of scene size for min distance squared
	const int totalRays = 20.0f;		// total rays per frame

	float minDistSq;					// minimum distance squared for VPL contribution
	int raysPerThread;					// rays per thread
	std::vector<VPL> vpls;				// list of VPLs

	// ##################################################################################
	// RADIOSITY METHODS
	// ##################################################################################

	void VPLTracePath(PathData& path, std::vector<VPL>& vplList, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= data.settings.maxBounces)
			return;

		// Russian roulette termination
		if (depth > 2)
		{
			float rrProbability = min(path.pathThroughput.Lum(), 0.95f);
			if (path.sampler->next() >= rrProbability) return;
			path.pathThroughput = path.pathThroughput / rrProbability;
		}

		// Traverse the scene to find an intersection
		IntersectionData intersection = data.scene->traverse(path.r);
		SurfaceData surfaceData = data.scene->calculateShadingData(intersection, path.r);
		ShadingData& shadingData = surfaceData.shadingData;

		if (surfaceData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (surfaceData.bsdf->isLight() || surfaceData.bsdf->isPureSpecular())
				return;

			// Sample new direction
			Color bsdf;
			float pdf;
			Vec3 wi = surfaceData.bsdf->sample(shadingData, path.sampler, bsdf, pdf);

			// Update path throughput
			path.pathThroughput = path.pathThroughput * bsdf * fabsf(wi.dot(shadingData.n)) / pdf;

			// If the path throughput is too low, stop
			if (path.pathThroughput.Lum() < EPSILON) return;

			// Create a new VPL at the intersection point and add to the list
			VPL vpl;
			vpl.p = surfaceData.p;
			vpl.shadingData = shadingData;
			vpl.Le = path.pathThroughput * path.Le;
			vplList.emplace_back(vpl);

			// Create new ray
			path.r.init(surfaceData.p + (wi * EPSILON), wi);

			// Continue tracing the path recursively
			VPLTracePath(path, vplList, depth + 1);
		}
	}

	void traceVPLs(unsigned int id, std::vector<VPL>& vplList)
	{
		Sampler* sampler = data.samplers[id];

		for (unsigned int i = 0; i < raysPerThread; i++)
		{
			// Sample a light
			LightSample light = data.scene->sampleLight(sampler);
			float lightPdf = light.pdf * light.pmf;

			// sample direction from light
			float pdfDir;
			Vec3 wi = light.light->sampleDirectionFromLight(sampler, pdfDir);

			SurfaceData shadingData;

			Color Le = light.emitted / (lightPdf * pdfDir);
			// normalize light if area light
			if (light.isArea)
				Le = Le * max(Dot(wi, light.n), 0);

			VPL vpl;
			vpl.p = light.p;
			vpl.shadingData = ShadingData(light.n);
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

	Color radiosityComputeDirect(const SurfaceData& surfaceData, Sampler* sampler)
	{
		const ShadingData& shadingData = surfaceData.shadingData;

		// Is surface is specular we cannot computing direct lighting
		if (surfaceData.bsdf->isPureSpecular() == true)
		{
			return Color(0.0f, 0.0f, 0.0f);
		}

		if (surfaceData.bsdf->isLight())
		{
			return surfaceData.bsdf->emit(shadingData, shadingData.wo);
		}

		Color accumulated(0.0f, 0.0f, 0.0f);

		unsigned int total = vpls.size();
		for (unsigned int i = 0; i < total; i++)
		{
			//unsigned int index = (unsigned int)(sampler->next() * (vpls.size() - 1));
			VPL vpl = vpls[i];

			// Calculate G Term
			Vec3 wi = vpl.p - surfaceData.p;
			float lengthSq = wi.lengthSq();
			wi = wi.normalize();

			float gTerm = (max(Dot(wi, shadingData.n), 0.0f) *
				max(-Dot(wi, vpl.shadingData.n), 0.0f)) / max(lengthSq, minDistSq);

			// Shade if visible
			if (gTerm > 0 && data.scene->visible(surfaceData.p, vpl.p))
				accumulated = accumulated + surfaceData.bsdf->evaluate(shadingData, wi) * vpl.Le * gTerm;
		}
		return accumulated / total;
	}

	// simple debug function to visualize vpl density
	float radiosityDebug(const Vec3& p, float& i)
	{
		const float rSq = SQ(0.03f);
		for (const VPL& vpl : vpls)
		{
			float lSq = (vpl.p - p).lengthSq();
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
		SurfaceData surfaceData = data.scene->calculateShadingData(intersection, r);

		if (data.settings.debug)
		{
			float i;
			if (radiosityDebug(surfaceData.p, i))
				return Color(1.0f, 0.0f, 0.0f) * (1.0f - i) + Color(1.0f, 1.0f, 0.0f) * i;
		}

		return surfaceData.t < FLT_MAX ? radiosityComputeDirect(surfaceData, sampler) :
			data.scene->background->evaluate(r.dir);
	}

	void radiosityVplPass()
	{
		// list fo newly generated vpls by each thread
		std::vector<std::vector<VPL>> vplLists(data.numThreads);

		// generated new vpls using multi threading
		for (int i = 0; i < data.numThreads; i++)
			data.threads[i] = new std::thread(&Radiosity::traceVPLs, this, i, std::ref(vplLists[i]));

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

	// ##################################################################################
	// RENDERING METHODS
	// ##################################################################################

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
			end = end.Min(data.film->size);

			renderTile(start, end, data.samplers[id]);
		}
	}

	void renderMT()
	{
		data.tileCounter.store(0);

		// process all tiles
		for (int i = 0; i < data.numThreads; i++)
			data.threads[i] = new std::thread(&Radiosity::processTiles, this, i);

		for (int i = 0; i < data.numThreads; i++)
		{
			data.threads[i]->join();
			delete data.threads[i];
		}
	}

	void process() override
	{
		radiosityVplPass();

		if (data.settings.useMultithreading)
			renderMT();
		else
			renderTile(Vec2i(0, 0), data.film->size, data.samplers[0]);
	}
public:
	Radiosity(RENDERER_DATA& _data) : AlgorithmBase(_data)
	{
		minDistSq = use<SceneBounds>().sceneRadius * minDistSqPerc;
		raysPerThread = max((float)totalRays / (float)data.numThreads, 1);
	}
};