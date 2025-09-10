#pragma once

#include "Algorithm.h"

/// <summary>
/// Path Tracing algorithm with Multiple Importance Sampling (MIS) and Russian Roulette termination.
/// Implements adaptive sampling based on tile variance.
/// </summary>
class PathTracing : public AlgorithmBase
{
	/// <summary>
	/// Path data structure to hold information about the current path segment.
	/// </summary>
	struct PathData
	{
		Ray& r;
		Sampler*& sampler;
		Color pathThroughput;

		float prevBsdfPdf;
		bool prevSpecular;

		PathData(Ray& _r, Sampler*& _sampler) : r(_r), sampler(_sampler)
		{
			pathThroughput = Color(1, 1, 1);
			prevBsdfPdf = 0.0f;
			prevSpecular = true;
		}
	};

	// percentage of total SPP to use as initial samples
	const float initSppAmnt = 0.2f;
	int initSpp;			// initial number of samples per tile

	// number of samples per tile for adaptive sampling
	std::vector<unsigned int> tileSamples;

	// Power heuristic for MIS
	float powerHeuristic(float pdfA, float pdfB)
	{
		pdfA *= pdfA;
		pdfB *= pdfB;
		return pdfA / (pdfA + pdfB);
	}

	// ##################################################################################
	// PATH TRACING METHODS
	// ##################################################################################

	Color computeDirect(const SurfaceData& surfaceData, Sampler* sampler)
	{
		const ShadingData& shadingData = surfaceData.shadingData;
		if (surfaceData.bsdf->isPureSpecular())
		{
			return Color(0.0f, 0.0f, 0.0f);
		}

		// Light sampling part
		LightSample light = data.scene->sampleLight(sampler);
		if (light.isNull)
			return Color(0.0f, 0.0f, 0.0f);

		Vec3 wi = light.p - surfaceData.p;
		float lengthSq = light.isArea ? wi.lengthSq() : 1.0f;
		wi = wi.normalize();

		float cosThetaSurface = max(Dot(wi, shadingData.n), 0.0f);
		float gTerm = cosThetaSurface / lengthSq;

		if (light.isArea)
		{
			float cosThetaLight = max(-Dot(wi, light.n), 0.0f);
			gTerm *= cosThetaLight;
		}

		if (gTerm > 0.0f && data.scene->visible(surfaceData.p, light.p))
		{
			Color bsdfVal = surfaceData.bsdf->evaluate(shadingData, wi);
			float bsdfPdf = surfaceData.bsdf->PDF(shadingData, wi);

			float lightPdf = light.pdf * light.pmf;

			// Balance heuristic for MIS
			float misWeight = powerHeuristic(lightPdf, bsdfPdf);

			return (light.emitted * bsdfVal * gTerm * misWeight) / lightPdf;
		}

		return Color(0.0f, 0.0f, 0.0f);
	}

	Color pathTrace(PathData& path, int depth = 0)
	{
		IntersectionData intersection = data.scene->traverse(path.r);
		SurfaceData surfaceData = data.scene->calculateShadingData(intersection, path.r);
		ShadingData& shadingData = surfaceData.shadingData;

		if (surfaceData.t < FLT_MAX)
		{
			// Handle light hit
			if (surfaceData.bsdf->isLight())
			{
				path.pathThroughput = path.pathThroughput * surfaceData.bsdf->emit(shadingData, path.r.dir);

				// if last bsdf was pure specular, we cannot use MIS
				if (path.prevSpecular)
					return path.pathThroughput;

				// MIS weight calculation
				float lightPdf = data.scene->getLightPdf(surfaceData.lightIndex, path.r.dir);
				float misWeight = powerHeuristic(path.prevBsdfPdf, lightPdf);

				return path.pathThroughput * misWeight;
			}

			// Direct lighting (with MIS)
			Color direct = path.pathThroughput * computeDirect(surfaceData, path.sampler);

			if (depth >= data.settings.maxBounces)
				return direct;

			// Russian roulette termination
			if (depth > 2)
			{
				float rrProbability = min(path.pathThroughput.Lum(), 0.95f);
				if (path.sampler->next() >= rrProbability) return direct;
				path.pathThroughput = path.pathThroughput / rrProbability;
			}

			// Sample BSDF for next path segment
			Color bsdf;
			float bsdfPdf;
			Vec3 wi = surfaceData.bsdf->sample(shadingData, path.sampler, bsdf, bsdfPdf);

			// Update path throughput
			float cosTheta = abs(Dot(wi, shadingData.n));
			path.pathThroughput = (path.pathThroughput * bsdf * cosTheta) / bsdfPdf;

			// Update path data
			path.r.init(surfaceData.p + (wi * EPSILON), wi);
			path.prevBsdfPdf = surfaceData.bsdf->PDF(shadingData, path.r.dir);
			path.prevSpecular = surfaceData.bsdf->isPureSpecular();

			// Indirect lighting (recursive call)
			Color indirect = pathTrace(path, depth + 1);

			// Combine direct and indirect lighting
			return direct + indirect;
		}

		// Missed scene - return background

		path.pathThroughput = path.pathThroughput * data.scene->background->evaluate(path.r.dir);

		if (depth == 0 || path.prevSpecular)
			return path.pathThroughput;

		float lightPdf = data.scene->getLightPdf(path.r.dir);
		// MIS weight calculation
		float misWeight = powerHeuristic(path.prevBsdfPdf, lightPdf);

		return path.pathThroughput * misWeight;
	}

	Color pathTrace(Ray& r, Sampler* sampler)
	{
		PathData path(r, sampler);
		return pathTrace(path);
	}

	// ##################################################################################
	// ADAPTIVE SAMPLING METHOD
	// ##################################################################################

	void calculateTileSamples()
	{
		for (unsigned int i = 0; i < data.totalTiles; i++)
		{
			Vec2i start = Vec2i((i % data.totalXTiles), (i / data.totalXTiles));
			start *= data.tileSize;

			Vec2i end = start + Vec2i(data.tileSize, data.tileSize);
			end = end.Min(data.film->size);

			std::vector<float> lums = data.film->getLums(start, end);

			// Compute average luminance
			float total = 0.0f;
			for (float lum : lums)
				total += lum;

			float mean = (lums.empty()) ? 0.0f : total / lums.size();

			// Compute variance
			float variance = 0.0f;
			for (float lum : lums)
				variance += (lum - mean) * (lum - mean);

			variance = (lums.empty()) ? 0.0f : variance / lums.size();
			float weight = clamp(variance / (variance + mean * mean + EPSILON), EPSILON, 1.0f); // Example weighting formula

			tileSamples[i] = (unsigned int)(data.film->SPP + (data.settings.totalSPP - data.film->SPP) * weight);
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

				Color col = pathTrace(ray, data.samplers[0]);
				data.film->splat(pixel, col);
			}
		}
	}

	void processTiles(unsigned int id)
	{
		unsigned int i;
		while ((i = data.tileCounter.fetch_add(1)) < data.totalTiles)
		{
			bool isTileRendered = data.film->SPP > initSpp &&
				data.film->SPP > tileSamples[i];

			if (isTileRendered)
				continue;

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
			data.threads[i] = new std::thread(&PathTracing::processTiles, this, i);

		for (int i = 0; i < data.numThreads; i++)
		{
			data.threads[i]->join();
			delete data.threads[i];
		}

		// calculate samples of tiles for adaptive sampling
		if (data.film->SPP == initSpp)
			calculateTileSamples();
	}

	void process() override
	{
		if (data.settings.useMultithreading)
			renderMT();
		else
			renderTile(Vec2i(0, 0), data.film->size, data.samplers[0]);
	}

public:
	PathTracing(RENDERER_DATA& data) : AlgorithmBase(data)
	{
		tileSamples.resize(data.totalTiles, 1);
		initSpp = (int)max(data.settings.totalSPP * initSppAmnt, 1.f);
	}

	int getSpp(int index) override
	{
		return initSpp < data.film->SPP ? min(tileSamples[index], data.film->SPP) : data.film->SPP;
	}

	void draw() override
	{
		unsigned char r, g, b;
		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < data.film->size.y; y++)
		{
			sppY = (y / data.tileSize) * data.totalXTiles;
			for (unsigned int x = 0; x < data.film->size.x; x++)
			{
				spp = getSpp(sppY + x / data.tileSize);
				data.film->tonemap(x, y, r, g, b, spp, data.settings.toneMap);
				data.canvas->draw(y * data.film->size.x + x, r, g, b);
			}
		}
	}
};