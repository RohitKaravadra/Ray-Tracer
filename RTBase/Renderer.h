#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

#define MAX_DEPTH 10

enum DRAWMODE
{
	DM_NORMALS,
	DM_ALBEDO,
	DM_DIRECT,
	DM_PATH_TRACE,
	DM_LIGHT_TRACE
};

struct MISData
{
	Colour bsdf;
	float pdf;
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom** samplers;
	std::thread** threads;
	int numProcs;
	unsigned int numThreads;

	unsigned int totalTiles;
	unsigned int totalXTiles;
	const unsigned int tileSize = 16;

	std::atomic<unsigned int> tileCounter;
	std::vector<unsigned int> tileSamples;

	DRAWMODE drawMode = DM_ALBEDO;
	TONEMAP toneMap = TM_LINEAR;

	unsigned int initSamples = 20;
	unsigned int totalSamples = 2000;

	~RayTracer()
	{
		std::cout << "Cleaning Ray Tracer..." << std::endl;

		// clean threads
		if (threads != nullptr)
			delete[] threads;

		// clean samplers
		if (samplers != nullptr)
		{
			for (unsigned int i = 0; i < numThreads; i++)
				delete samplers[i];
			delete[] samplers;
		}

		// clean film
		if (film != nullptr)
			delete film;

	}

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, unsigned int _numThreads = 3)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;

		// calculate number of threads according to available processors
		numThreads = max(1, min(_numThreads, numProcs));

		// create threads and samplers for each thread
		threads = new std::thread * [numThreads];
		samplers = new MTRandom * [numThreads];

		// assign different seeds to each sampler
		// Linear Congruential Generator used for seed
		// x + 1 = [a * (x - 1) + c] % m
		// where a = 48271, c = 0
		int m = pow(2, 32) - 1;
		for (unsigned int i = 0; i < numThreads; i++)
			samplers[i] = new MTRandom((48271 * (i + 1)) % m);

		totalXTiles = (canvas->getWidth() + tileSize - 1) / tileSize;
		float totalYTiles = (canvas->getHeight() + tileSize - 1) / tileSize;
		totalTiles = totalXTiles * totalYTiles;
		tileSamples.resize(totalTiles);

		tileCounter.store(0);
	}
	void clear()
	{
		film->clear();
	}

	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		// project point on camera if possible
		if (scene->camera.projectOntoCamera(p, x, y))
		{
			// check if point is visible from camera
			if (!scene->visible(p, scene->camera.origin))
				return;

			Vec3 toCamDir = (scene->camera.origin - p).normalize();
			float cosTheta = Dot(toCamDir, n);
			float we = 1 / (SQ(cosTheta) * SQ(cosTheta) * scene->camera.Afilm);

			film->splat(x, y, col * we);
		}
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth = 0)
	{

		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light, stop
			if (shadingData.bsdf->isLight())
				return;

			// Is surface is specular we cannot computing direct lighting
			if (shadingData.bsdf->isPureSpecular() == true)
				return;

			// If the hit surface is visible to the camera, connect it
			connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput);

			// Max recursion depth check
			if (depth >= MAX_DEPTH)
				return;

			// Russian Roulette for termination
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() >= russianRouletteProbability)
				return;

			pathThroughput = pathThroughput / russianRouletteProbability;

			// Sample new direction for indirect lighting
			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// Update path throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			// Create new ray
			r.init(shadingData.x + wi, wi);

			// Continue tracing the path recursively
			lightTracePath(r, pathThroughput, Le, sampler, depth + 1);
		}
	}

	void lightTrace(Sampler* sampler)
	{
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		// Sample a point on the light
		float pdfPos, pdfDir;
		Colour emitted;
		Vec3 p = light->samplePositionFromLight(sampler, pdfPos);
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDir);

		Colour Le = light->evaluate(-wi) / (pdfDir * pdfPos);

		connectToCamera(p, p + wi, Le);

		Ray ray(p, wi);
		Colour pathThroughput(1.0f, 1.0f, 1.0f);

		lightTracePath(ray, pathThroughput, Le, sampler);
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler, float misPdf = 0)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);

		if (light->isArea())
		{
			// Calculate G Term
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();

			float gTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) *
				max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;

			if (gTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					float weight = pdf / (pdf + misPdf);
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * gTerm * weight / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate G Term
			Vec3 wi = p.normalize();
			float gTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (gTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					float weight = pdf / (pdf + misPdf);
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * gTerm * weight / (pmf * pdf);
				}
			}
		}

		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, float misPdf = 0, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return canHitLight ? pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo)
					: Colour(0.0f, 0.0f, 0.0f);
			}

			// max depth reached
			if (depth >= MAX_DEPTH)
				return pathThroughput * computeDirect(shadingData, sampler);

			// russian roulette
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			if (sampler->next() < russianRouletteProbability)
				pathThroughput = pathThroughput / russianRouletteProbability;
			else
				return pathThroughput * computeDirect(shadingData, sampler);

			// sample new direction
			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// calculate throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;
			// create new ray
			r.init(shadingData.x + (wi * EPSILON), wi);

			// calculate direct lighting with MIS
			Colour direct = pathThroughput * computeDirect(shadingData, sampler, pdf);

			// trace new ray
			return direct + pathTrace(r, pathThroughput, depth + 1, sampler, pdf, shadingData.bsdf->isPureSpecular());
		}

		if (depth <= 0)
			return scene->background->evaluate(r.dir) * pathThroughput;

		float pdf = scene->background->PDF(shadingData, r.dir);
		float weight = pdf / (pdf + misPdf);
		return scene->background->evaluate(r.dir) * pathThroughput * weight / pdf;
	}

	Colour pathTrace(Ray& r, Sampler* sampler)
	{
		Colour pathThroughput(1.0f, 1.0f, 1.0f);
		return pathTrace(r, pathThroughput, 0, sampler);
	}

	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				if (drawMode == DM_LIGHT_TRACE)
				{
					lightTrace(samplers[0]);
				}
				else
				{
					float px = x + (drawMode == DM_PATH_TRACE ? samplers[0]->next() : 0.5f);
					float py = y + (drawMode == DM_PATH_TRACE ? samplers[0]->next() : 0.5f);
					Ray ray = scene->camera.generateRay(px, py);

					Colour col;
					switch (drawMode)
					{
					case DM_NORMALS:
						col = viewNormals(ray);
						break;
					case DM_ALBEDO:
						col = albedo(ray);
						break;
					case DM_DIRECT:
						col = direct(ray, samplers[0]);
						break;
					case DM_PATH_TRACE:
						col = pathTrace(ray, samplers[0]);
						break;
					}

					film->splat(px, py, col);
				}

				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, toneMap);

				canvas->draw(x, y, r, g, b);
			}
		}
	}

	unsigned int calculateSamples()
	{
		unsigned int i;
		while ((i = tileCounter.fetch_add(1)) < totalTiles)
		{
			unsigned int startx = (i % totalXTiles) * tileSize;
			unsigned int starty = (i / totalXTiles) * tileSize;

			unsigned int endx = min(startx + tileSize, film->width);
			unsigned int endy = min(starty + tileSize, film->height);
			std::vector<float> lums = film->getLums(startx, starty, endx, endy);

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

			float weight = variance / (variance + mean * mean + EPSILON); // Example weighting formula

			tileSamples[i] = totalSamples * weight + initSamples;
		}
	}

	void processTile(unsigned int id)
	{
		unsigned int i;
		while ((i = tileCounter.fetch_add(1)) < totalTiles)
		{
			unsigned int startx = (i % totalXTiles) * tileSize;
			unsigned int starty = (i / totalXTiles) * tileSize;

			unsigned int endx = min(startx + tileSize, film->width);
			unsigned int endy = min(starty + tileSize, film->height);

			for (unsigned int y = starty; y < endy; y++)
			{
				for (unsigned int x = startx; x < endx; x++)
				{
					unsigned char r, g, b;

					// only calculate if enought samples are not calculated
					if (film->SPP <= initSamples || film->SPP < tileSamples[i])
					{
						if (drawMode == DM_LIGHT_TRACE)
						{
							lightTrace(samplers[0]);
						}
						else
						{
							float px = x + (drawMode == DM_PATH_TRACE ? samplers[id]->next() : 0.5f);
							float py = y + (drawMode == DM_PATH_TRACE ? samplers[id]->next() : 0.5f);
							Ray ray = scene->camera.generateRay(px, py);

							Colour col;
							switch (drawMode)
							{
							case DM_NORMALS:
								col = viewNormals(ray);
								break;
							case DM_ALBEDO:
								col = albedo(ray);
								break;
							case DM_DIRECT:
								col = direct(ray, samplers[id]);
								break;
							case DM_PATH_TRACE:
								col = pathTrace(ray, samplers[id]);
								break;
							}
							film->splat(px, py, col);
						}
						film->tonemap(x, y, r, g, b, film->SPP, toneMap);
					}
					else
						film->tonemap(x, y, r, g, b, tileSamples[i], toneMap);

					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}

	void renderMT()
	{
		film->incrementSPP();

		tileCounter.store(0);

		for (int i = 0; i < numThreads; i++)
			threads[i] = new std::thread(&RayTracer::processTile, this, i);

		for (int i = 0; i < numThreads; i++)
		{
			threads[i]->join();
			delete threads[i];
		}

		if (film->SPP == initSamples)
		{
			tileCounter.store(0);
			for (int i = 0; i < numThreads; i++)
				threads[i] = new std::thread(&RayTracer::calculateSamples, this);

			for (int i = 0; i < numThreads; i++)
			{
				threads[i]->join();
				delete threads[i];
			}
		}
	}

	int getSPP() const
	{
		return film->SPP;
	}

	void saveHDR(std::string filename)
	{
		film->save(filename);
	}

	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};