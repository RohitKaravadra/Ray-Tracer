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
#include <mutex>

class RayTracer
{

public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;

	int numProcs;
	unsigned int numThreads;

	unsigned int totalTiles;
	unsigned int totalXTiles;
	const unsigned int tileSize = 32;
	std::atomic<unsigned int> tileCounter;

	std::mutex mtx;

	~RayTracer()
	{
		std::cout << "Cleaning Ray Tracer..." << std::endl;

		if (threads != nullptr)
			delete[] threads;

		if (samplers != nullptr)
			delete[] samplers;

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

		numThreads = max(1, min(_numThreads, numProcs));
		threads = new std::thread * [numThreads];
		samplers = new MTRandom[numThreads];

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
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		return Colour(0.0f, 0.0f, 0.0f);
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
				Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void processTile()
	{
		unsigned int i;
		while ((i = tileCounter.fetch_add(1)) < totalTiles)
		{
			unsigned int startx = (i % totalXTiles) * tileSize;
			unsigned int starty = (int(i) / int(totalXTiles)) * tileSize;

			unsigned int endx = min(startx + tileSize, film->width);
			unsigned int endy = min(starty + tileSize, film->height);

			for (unsigned int y = starty; y < endy; y++)
			{
				for (unsigned int x = startx; x < endx; x++)
				{
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					//if (!scene->bounds.rayAABB(ray))
					//	continue;

					//Colour col = viewNormals(ray);
					Colour col = albedo(ray);
					film->splat(px, py, col);
					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					canvas->draw(x, y, r, g, b);
				}
			}
			//mtx.lock();
			//canvas->present();
			//mtx.unlock();
		}
	}

	/// <summary>
	/// Render using multiple threads and segments
	/// </summary>
	/// <param name="numThreads">Number of threads to use (uses between 1 and max threads processor supports)</param>
	void renderMT()
	{
		numThreads = max(1, min(numThreads, numProcs));

		film->incrementSPP();
		tileCounter.store(0);

		for (int i = 0; i < numThreads; i++)
			threads[i] = new std::thread(&RayTracer::processTile, this);

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