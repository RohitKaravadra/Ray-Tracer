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

#define MAX_DEPTH 5

enum DRAWMODE
{
	DM_NORMALS,
	DM_ALBEDO,
	DM_DIRECT,
	DM_PATH_TRACE,
	DM_LIGHT_TRACE,
	DM_INSTANT_RADIOSITY
};

struct SETTINGS
{
	DRAWMODE drawMode;
	TONEMAP toneMap;
	IMAGE_FILTER filter;

	bool TileBasedAdaptiveSampling;
	bool canHitLight;

	unsigned int numThreads;
	unsigned int initSPP;
	unsigned int totalSPP;

	SETTINGS()
	{
		drawMode = DM_NORMALS;
		toneMap = TM_NONE;
		filter = FT_BOX;

		TileBasedAdaptiveSampling = false;
		canHitLight = false;

		numThreads = 3;
		initSPP = 10;
		totalSPP = 8192;
	}
};

struct VPL
{
	ShadingData shadingData;
	Colour Le;
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

	std::vector<VPL> vpls;
	std::mutex vplMtx;

	SETTINGS settings;

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

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, SETTINGS _settings)
	{
		scene = _scene;
		canvas = _canvas;
		settings = _settings;

		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, settings.filter);
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;

		if (!settings.TileBasedAdaptiveSampling)
			settings.initSPP = settings.totalSPP;

		setMultithreading(settings.numThreads);
	}

	void setMultithreading(unsigned int _numThreads)
	{
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

		// calculate number of tiles
		totalXTiles = (canvas->getWidth() + tileSize - 1) / tileSize;
		float totalYTiles = (canvas->getHeight() + tileSize - 1) / tileSize;
		totalTiles = totalXTiles * totalYTiles;

		// create samples for each tile
		tileSamples.resize(totalTiles);

		tileCounter.store(0);
	}

	void clear()
	{
		film->clear();
	}

	// RADIOSITY #####################################################################################################

	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, std::vector<VPL>& vplList, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= MAX_DEPTH)
			return;

		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
				return;

			// Russian Roulette for termination
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < sampler->next())
				return;

			pathThroughput = pathThroughput / russianRouletteProbability;

			// Sample new direction
			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// Update path throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			VPL vpl;
			vpl.shadingData = shadingData;
			vpl.Le = pathThroughput * Le;

			// update vpls list
			vplList.emplace_back(vpl);

			// Create new ray
			r.init(shadingData.x, wi);

			// Continue tracing the path recursively
			VPLTracePath(r, pathThroughput, Le, sampler, vplList, depth + 1);
		}
	}

	void traceVPLs(unsigned int id, std::vector<VPL>& vplList)
	{
		Sampler* sampler = samplers[id];

		int count = 1;
		int total = count * numThreads;

		for (unsigned int i = 0; i < count; i++)
		{
			// Sample a light
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);

			float pdfPos, pdfDir;

			// Sample a point on the light
			Vec3 p = light->samplePositionFromLight(sampler, pdfPos);
			// sample direction from light
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDir);

			ShadingData shadingData;
			// calculate light emmision
			Vec3 nLight = light->normal(shadingData, wi);

			float cosTheta = Dot(wi, nLight);
			Colour Le = light->evaluate(-wi) * cosTheta / (pmf * pdfPos * pdfDir);

			VPL vpl;
			vpl.shadingData = ShadingData(p, nLight);
			vpl.Le = Le;

			// update vpls list
			vplList.emplace_back(vpl);

			Ray ray(p, wi);
			Colour pathThroughput(1.0f, 1.0f, 1.0f);

			VPLTracePath(ray, pathThroughput, Le, sampler, vplList);
		}
	}

	Colour radiosityComputeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		if (shadingData.bsdf->isLight())
		{
			return settings.canHitLight ? shadingData.bsdf->emit(shadingData, shadingData.wo)
				: Colour(0.0f, 0.0f, 0.0f);
		}

		Colour accumulated(0.0f, 0.0f, 0.0f);

		for (unsigned int i = 0; i < vpls.size(); i++)
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
			if (gTerm > 0 && scene->visible(shadingData.x, vpl.shadingData.x))
				accumulated = accumulated + shadingData.bsdf->evaluate(shadingData, wi) * vpl.Le * gTerm;
		}
		return accumulated / vpls.size();
	}

	Colour radiosityLightPass(Ray r, Sampler* sampler)
	{
		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		return shadingData.t < FLT_MAX ? radiosityComputeDirect(shadingData, sampler) :
			scene->background->evaluate(r.dir);
	}

	void radiosityVplPass()
	{
		// list fo newly generated vpls by each thread
		std::vector<std::vector<VPL>> vplLists(numThreads);

		// generated new vpls using multi threading
		for (int i = 0; i < numThreads; i++)
			threads[i] = new std::thread(&RayTracer::traceVPLs, this, i, std::ref(vplLists[i]));

		for (int i = 0; i < numThreads; i++)
		{
			threads[i]->join();
			delete threads[i];
		}

		// clear past vpls
		vpls.clear();

		// merge all vpls in single list
		for (const auto& v : vplLists) {
			vpls.insert(vpls.end(), v.begin(), v.end());
		}
	}

	//##############################################################################################################

	// LIGHT TRACING #####################################################################################################

	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		// project point on camera if possible
		if (scene->camera.projectOntoCamera(p, x, y))
		{
			Vec3 toCamDir = (scene->camera.origin - p);
			float lengthSq = toCamDir.lengthSq();
			toCamDir = toCamDir.normalize();

			float cosThetaS = Dot(toCamDir, n);
			float cosThetaC = Dot(toCamDir, scene->camera.viewDirection);

			float gTerm = max(cosThetaS, 0.0f) * max(-cosThetaC, 0.0f) / lengthSq;

			if (gTerm > 0)
			{
				if (scene->visible(p, scene->camera.origin))
				{
					float cosThetaSq = SQ(cosThetaC);
					float we = 1.0f / (cosThetaSq * cosThetaSq * scene->camera.Afilm);

					film->splat(x, y, col * we * gTerm);
				}
			}
		}
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= MAX_DEPTH)
			return;

		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
				return;

			// Russian Roulette for termination
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < sampler->next())
				return;

			pathThroughput = pathThroughput / russianRouletteProbability;

			Vec3 wi = (scene->camera.origin - shadingData.x).normalize();
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;
			connectToCamera(shadingData.x, shadingData.sNormal, col);

			// Sample new direction
			Colour bsdf;
			float pdf;
			wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// Update path throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			// Create new ray
			r.init(shadingData.x + (wi * EPSILON), wi);

			// Continue tracing the path recursively
			lightTracePath(r, pathThroughput, Le, sampler, depth + 1);
		}
	}

	void lightTrace(Sampler* sampler)
	{
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		float pdfPos, pdfDir;

		// Sample a point on the light
		Vec3 p = light->samplePositionFromLight(sampler, pdfPos);
		// sample direction from light
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDir);

		ShadingData shadingData;
		// calculate light emmision
		Vec3 nLight = light->normal(shadingData, wi);

		float cosTheta = Dot(wi, nLight);
		Colour Le = light->evaluate(-wi) / (pmf * pdfPos * pdfDir);

		// connect to camera to draw light
		if (settings.canHitLight)
			connectToCamera(p, nLight, Le);

		Ray ray(p, wi);
		Colour pathThroughput(1.0f, 1.0f, 1.0f);

		lightTracePath(ray, pathThroughput, Le * cosTheta, sampler);
	}

	// #############################################################################################################

	// PATH TRACE #####################################################################################################

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
			float lengthSq = wi.lengthSq();
			wi = wi.normalize();

			float gTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) *
				max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / lengthSq;

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

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight, float misPdf = 0)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		//return scene->background->evaluate(r.dir);

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
			canHitLight = settings.canHitLight && shadingData.bsdf->isPureSpecular();
			return direct + pathTrace(r, pathThroughput, depth + 1, sampler, canHitLight, pdf);
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
		return pathTrace(r, pathThroughput, 0, sampler, settings.canHitLight);
	}

	// ###################################################################################################################

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
		if (settings.drawMode == DM_INSTANT_RADIOSITY)
			radiosityVplPass();
		else
		{
			for (unsigned int y = 0; y < film->height; y++)
			{
				for (unsigned int x = 0; x < film->width; x++)
				{
					if (settings.drawMode == DM_LIGHT_TRACE)
					{
						lightTrace(samplers[0]);
					}
					else
					{
						float px = x + (settings.drawMode == DM_PATH_TRACE ? samplers[0]->next() : 0.5f);
						float py = y + (settings.drawMode == DM_PATH_TRACE ? samplers[0]->next() : 0.5f);
						Ray ray = scene->camera.generateRay(px, py);

						Colour col;
						switch (settings.drawMode)
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
					film->tonemap(x, y, r, g, b, settings.toneMap);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}

	// TILE BASED ADAPTIVE SAMPLING ########################################################################################

	void calculateTileVariance()
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
			float weight = clamp(variance / (variance + mean * mean + EPSILON), EPSILON, 1.0f); // Example weighting formula

			tileSamples[i] = (settings.totalSPP - settings.initSPP) * weight;
		}
	}

	void calculateTileSamples()
	{
		std::vector<std::vector<std::pair<int, float>>> varLists(numThreads);
		tileCounter.store(0);

		// calculate variance for each tile
		for (int i = 0; i < numThreads; i++)
			threads[i] = new std::thread(&RayTracer::calculateTileVariance, this);

		for (int i = 0; i < numThreads; i++)
		{
			threads[i]->join();
			delete threads[i];
		}
	}

	// #####################################################################################################################

	// MULTI THREADING #####################################################################################################

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
					unsigned int spp = film->SPP;

					if (settings.drawMode == DM_LIGHT_TRACE)
					{
						lightTrace(samplers[id]);
					}
					else if (film->SPP <= settings.initSPP || film->SPP < tileSamples[i])
					{
						// only calculate if enought samples are not calculated
						float px = x + (settings.drawMode == DM_PATH_TRACE ? samplers[id]->next() : 0.5f);
						float py = y + (settings.drawMode == DM_PATH_TRACE ? samplers[id]->next() : 0.5f);
						Ray ray = scene->camera.generateRay(px, py);

						Colour col;
						switch (settings.drawMode)
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
						case DM_INSTANT_RADIOSITY:
							col = radiosityLightPass(ray, samplers[id]);
						}
						film->splat(px, py, col);
					}
					else
						spp = tileSamples[i];

					// tonemap and draw pixel
					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b, spp, settings.toneMap);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}

	void renderMT()
	{
		film->incrementSPP();
		tileCounter.store(0);

		// check for radiosity vpl pass
		if (settings.drawMode == DM_INSTANT_RADIOSITY)
			radiosityVplPass();

		// process all tiles
		for (int i = 0; i < numThreads; i++)
			threads[i] = new std::thread(&RayTracer::processTile, this, i);

		for (int i = 0; i < numThreads; i++)
		{
			threads[i]->join();
			delete threads[i];
		}

		// calculate samples of tiles for adaptive sampling
		if (film->SPP == settings.initSPP && settings.drawMode != DM_LIGHT_TRACE)
			calculateTileSamples();
	}

	// ##################################################################################################################

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