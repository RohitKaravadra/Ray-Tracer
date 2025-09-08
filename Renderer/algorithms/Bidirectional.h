#pragma once

#include "Algorithm.h"

/// <summary>
/// Bidirectional Path Tracing algorithm with Multiple Importance Sampling (MIS) and Russian Roulette termination.
/// </summary>
class Bidirectional : public AlgorithmBase
{
	// percentage of total SPP to use as initial samples
	const float initSppAmnt = 0.2f;
	int initSpp;			// initial number of samples per tile

	// number of samples per tile for adaptive sampling
	std::vector<unsigned int> tileSamples;

	// ##################################################################################
	// BIDIRECTIONAL METHODS
	// ##################################################################################

	struct VertexData
	{
		Vec3 p;						// Position of the vertex
		BSDF* bsdf;					// BSDF evaluated value
		Color throughput;			// Throughput up to this vertex
		ShadingData shadingData;	// Shading data at the vertex

		VertexData() : p(0), bsdf(nullptr), throughput(1) {}
	};

	void generatePath(Ray& r, Sampler* sampler, Color& throughput, std::vector<VertexData>& path, int depth = 0, bool reverse = false)
	{
		if (data.settings.maxBounces < depth)
			return;

		IntersectionData intersection = data.scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			// Russian roulette termination
			float rrProbability = min(max(throughput.Lum(), 0.05f), 0.95f);
			if (sampler->next() >= rrProbability) return;
			throughput /= rrProbability;

			SurfaceData surfaceData = data.scene->calculateShadingData(intersection, r);
			ShadingData& shadingData = surfaceData.shadingData;

			// Sample BSDF for next path segment
			Color bsdf;
			float bsdfPdf;
			Vec3 wi = surfaceData.bsdf->sample(shadingData, sampler, bsdf, bsdfPdf);

			// update throughput
			throughput *= bsdf * fabsf(Dot(wi, shadingData.n)) / bsdfPdf;

			// terminate if throughput is too low
			if (throughput.Lum() < EPSILON) return;

			// create vertex
			VertexData vertex;
			vertex.p = surfaceData.p;
			vertex.shadingData = shadingData;
			vertex.throughput = throughput;
			vertex.bsdf = surfaceData.bsdf;

			path.emplace_back(vertex);
			if (surfaceData.bsdf->isLight())
				return;

			r.init(surfaceData.p + (wi * EPSILON), wi);
			generatePath(r, sampler, throughput, path, depth + 1);
		}
	}

	bool generateLightPath(Sampler* sampler, std::vector<VertexData>& path)
	{
		// sampel light source
		LightSample light = data.scene->sampleLight(sampler);
		if (light.isNull) return false;

		// sample direction and create ray
		float dirPdf;
		Vec3 dir = light.light->sampleDirectionFromLight(sampler, dirPdf);
		Ray lightRay(light.p + light.n * EPSILON, dir);

		Color throughput = light.emitted * fabsf(Dot(light.n, dir)) / (light.pmf * light.pdf * dirPdf);

		VertexData vertex;
		vertex.p = light.p;
		vertex.shadingData.n = light.n;
		vertex.throughput = throughput;
		path.emplace_back(vertex);

		generatePath(lightRay, sampler, throughput, path);

		return true;
	}

	bool generateCameraPath(Ray& r, Sampler* sampler, std::vector<VertexData>& path, Color& emitted)
	{
		Color throughput(1.0f);

		VertexData vertex;
		vertex.p = r.o;
		vertex.shadingData.n = r.dir; // Camera normal aligned with ray direction
		vertex.throughput = throughput;
		path.emplace_back(vertex);

		IntersectionData intersection = data.scene->traverse(r);

		if (intersection.t < FLT_MAX)
		{
			SurfaceData surfaceData = data.scene->calculateShadingData(intersection, r);
			ShadingData& shadingData = surfaceData.shadingData;

			// Sample BSDF for next path segment
			Color bsdf;
			float bsdfPdf;
			Vec3 wi = surfaceData.bsdf->sample(shadingData, sampler, bsdf, bsdfPdf);

			// update throughput
			throughput *= bsdf * fabsf(Dot(wi, shadingData.n)) / bsdfPdf;

			// create vertex
			VertexData vertex;
			vertex.p = surfaceData.p;
			vertex.shadingData = shadingData;
			vertex.throughput = throughput;
			vertex.bsdf = surfaceData.bsdf;

			path.emplace_back(vertex);
			if (surfaceData.bsdf->isLight())
			{
				emitted = surfaceData.bsdf->emission;
				return true;
			}

			r.init(surfaceData.p + (wi * EPSILON), wi);
			generatePath(r, sampler, throughput, path, 1);
		}

		return false;
	}

	Color computePath(Ray& ray, Sampler* sampler)
	{
		std::vector<VertexData> cameraPath;
		Color emitted(1.0f);
		if (generateCameraPath(ray, sampler, cameraPath, emitted))
			return emitted;
		if (cameraPath.empty()) return Color(0.0f);

		std::vector<VertexData> lightPath;
		if (!generateLightPath(sampler, lightPath))
			return Color(0.0f);

		Color result(0);

		// Try all (s, t) connections
		for (int s = 0; s < (int)cameraPath.size(); s++)
		{
			for (int t = 0; t < (int)lightPath.size(); t++)
			{
				const auto& cv = cameraPath[s];
				const auto& lv = lightPath[t];

				// skip if specular → you can’t directly connect through delta BSDFs
				if ((cv.bsdf && cv.bsdf->isPureSpecular()) ||
					lv.bsdf && lv.bsdf->isPureSpecular())
					continue;

				// shadow ray test
				if (!data.scene->visible(cv.p, lv.p))
					continue;

				// geometry term
				Vec3 d = lv.p - cv.p;
				float dist2 = d.lengthSq();
				d /= sqrt(dist2);
				float cosC = max(0.f, Dot(cv.shadingData.n, d));
				float cosL = max(0.f, Dot(lv.shadingData.n, -d));
				float G = (cosC * cosL) / dist2;

				Color camContrib = cv.bsdf ? cv.bsdf->evaluate(cv.shadingData, d) : 1;
				Color lightContrib = lv.bsdf ? lv.bsdf->evaluate(lv.shadingData, -d) : 1;

				// combine throughputs
				Color contrib = cv.throughput * camContrib * G * lightContrib * lv.throughput;

				// TODO: apply MIS weight (for now weight = 1)
				result += contrib;
			}
		}

		return result;
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

				Color col = computePath(ray, data.samplers[0]);
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
			data.threads[i] = new std::thread(&Bidirectional::processTiles, this, i);

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
	Bidirectional(RENDERER_DATA& data) : AlgorithmBase(data)
	{
		tileSamples.resize(data.totalTiles, 1);
		initSpp = max((int)(data.settings.totalSPP * initSppAmnt), 1);
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