#pragma once

#include "Algorithm.h"


/// <summary>
/// Light Tracing algorithm implementation.
/// </summary>
class LightTracing : public AlgorithmBase
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

	// ##################################################################################
	// LIGHT TRACING METHODS
	// ##################################################################################

	void connectToCamera(Vec3 p, Vec3 n, Color col)
	{
		Vec2 sp; //screen position
		// project point on camera if possible
		if (data.scene->camera.projectOntoCamera(p, sp))
		{
			Vec3 toCamDir = (data.scene->camera.pos - p);
			float lengthSq = toCamDir.lengthSq();
			toCamDir = toCamDir.normalize();

			float cosThetaS = Dot(toCamDir, n);
			float cosThetaC = Dot(toCamDir, data.scene->camera.dir);

			float gTerm = max(cosThetaS, 0.0f) * max(-cosThetaC, 0.0f) / lengthSq;

			if (gTerm > 0)
			{
				if (data.scene->visible(p, data.scene->camera.pos))
				{
					float cosThetaSq = SQ(cosThetaC);
					float we = 1.0f / (cosThetaSq * cosThetaSq * data.scene->camera.Afilm);

					data.film->splat(sp, col * we * gTerm);
				}
			}
		}
	}

	void lightTracePath(PathData& path, int depth = 0)
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

			// connect to camera and draw pixel
			Vec3 wi = (data.scene->camera.pos - shadingData.x).normalize();
			Color col = path.pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * path.Le;
			connectToCamera(shadingData.x, shadingData.sNormal, col);

			// Russian Roulette for termination
			float russianRouletteProbability = min(path.pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < path.sampler->next())
				return;
			path.pathThroughput = path.pathThroughput / russianRouletteProbability;

			// Sample new direction
			Color bsdf;
			float pdf;
			wi = shadingData.bsdf->sample(shadingData, path.sampler, bsdf, pdf);

			// Update path throughput
			path.pathThroughput = path.pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			// Create new ray
			path.r.init(shadingData.x + (wi * EPSILON), wi);

			// Continue tracing the path recursively
			lightTracePath(path, depth + 1);
		}
	}

	void lightTrace(Sampler* sampler)
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

		// connect to camera to draw light
		connectToCamera(light.p, light.n, Le);

		Ray ray(light.p + (wi * EPSILON), wi);
		Color pathThroughput(1.0f, 1.0f, 1.0f);

		PathData path(ray, sampler);
		path.Le = Le;
		lightTracePath(path, 0);
	}

	// ##################################################################################
	// RENDERING METHODS
	// ##################################################################################

	void renderTile(const Vec2i& start, const Vec2i& end, Sampler* sampler)
	{
		for (unsigned int y = start.y; y < end.y; y++)
			for (unsigned int x = start.x; x < end.x; x++)
				lightTrace(sampler);
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
			data.threads[i] = new std::thread(&LightTracing::processTiles, this, i);

		for (int i = 0; i < data.numThreads; i++)
		{
			data.threads[i]->join();
			delete data.threads[i];
		}
	}

	void process() override
	{
		if (data.settings.useMultithreading)
			renderMT();
		else
			renderTile(Vec2i(0, 0), data.film->size, data.samplers[0]);
	}
public:
	LightTracing(RENDERER_DATA& data) : AlgorithmBase(data) {}
};