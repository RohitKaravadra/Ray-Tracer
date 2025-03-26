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

#define MAX_DEPTH 5

enum DRAWMODE
{
	DM_NORMALS = 0,
	DM_ALBEDO = 1,
	DM_DIRECT = 2,
	DM_PATHTRACE = 3
};

struct LastData
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

	DRAWMODE drawMode = DM_ALBEDO;

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

		totalXTiles = canvas->getWidth() / tileSize;
		totalTiles = totalXTiles * canvas->getHeight() / tileSize;

		tileCounter.store(0);

		clear();
	}

	void clear()
	{
		film->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
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

			float gTerm = (max(Dot(wi, shadingData.sNormal) * -Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;

			if (gTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * gTerm / (pmf * pdf);
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
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * gTerm / (pmf * pdf);
				}
			}
		}

		return Colour(0.0f, 0.0f, 0.0f);
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
		return scene->background->evaluate(shadingData, r.dir);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, LastData& lastData, bool canHitLight = true)
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

			// calculate direct lighting
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			if (depth > MAX_DEPTH) return direct;

			// russian roulette
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			if (sampler->next() < russianRouletteProbability)
				pathThroughput = pathThroughput / russianRouletteProbability;
			else
				return direct;

			// sample new direction
			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// calculate throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;
			// create new ray
			r.init(shadingData.x + (wi * EPSILON), wi);

			// store last data
			lastData.bsdf = bsdf;
			lastData.pdf = pdf;

			// trace new ray
			return direct + pathTrace(r, pathThroughput, depth + 1, sampler, lastData, shadingData.bsdf->isPureSpecular());
		}

		Colour bkColour = scene->background->evaluate(shadingData, r.dir);
		if (depth <= 0)
			return bkColour;

		float bkPdf = scene->background->PDF(shadingData, r.dir);

		float total = lastData.pdf + bkPdf;
		float w1 = bkPdf / total;
		float w2 = lastData.pdf / total;
		return (bkColour * w1 + lastData.bsdf * w2) * pathThroughput;
	}

	Colour pathTrace(unsigned int x, unsigned int y, Sampler* sampler)
	{
		Ray ray = scene->camera.generateRay(x + sampler->next(), y + sampler->next());

		Colour pathThroughput(1.0f, 1.0f, 1.0f);
		LastData lastData;
		return pathTrace(ray, pathThroughput, 0, sampler, lastData);
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
		return scene->background->evaluate(shadingData, r.dir);
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
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);

				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				Colour col = direct(ray, samplers[0]);
				//Colour col = pathTrace(x, y, samplers[0]);

				film->splat(px, py, col);
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);

				canvas->draw(x, y, r, g, b);
			}
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
					float px = x + 0.5f;
					float py = y + 0.5f;
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
					case DM_PATHTRACE:
						col = pathTrace(x, y, samplers[id]);
						break;
					}

					film->splat(px, py, col);
					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b);

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
	}

	int getSPP()
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