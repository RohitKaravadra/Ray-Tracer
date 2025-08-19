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

#include "Denoiser.h"
#include "Settings.h"

// vertual point light
struct VPL
{
	ShadingData shadingData;
	Color Le;
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom** samplers;					// samplers for multithreading
	std::thread** threads;					// threads for multithreading
	int numProcs;							// number of processors
	unsigned int numThreads;				// number of threads

	unsigned int totalTiles;				// number of tiles
	unsigned int totalXTiles;				// number of tiles in x direction	
	const unsigned int tileSize = 16;		// size of each tile

	std::atomic<unsigned int> tileCounter;	// number of tiles processed
	std::vector<unsigned int> tileSamples;	// number of samples per tile

	std::vector<VPL> vpls;					// list of VPLs

	SETTINGS settings;						// settings for the ray tracer

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

		// only use adaptive sampling for path tracing
		settings.adaptiveSampling = settings.adaptiveSampling && settings.algorithm == AL_PATH_TRACE;
		if (!(settings.adaptiveSampling && settings.useMultithreading))
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
		tileSamples.resize(totalTiles, 1);

		tileCounter.store(0);
	}

	void clear()
	{
		film->clear();
	}

	// RADIOSITY #####################################################################################################

	void VPLTracePath(Ray& r, Color pathThroughput, Color Le, Sampler* sampler, std::vector<VPL>& vplList, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= settings.maxBounces)
			return;

		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
				return;

			// Sample new direction
			Color bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			// Update path throughput
			pathThroughput = pathThroughput * bsdf * fabsf(wi.dot(shadingData.sNormal)) / pdf;

			VPL vpl;
			vpl.shadingData = shadingData;
			vpl.Le = pathThroughput * Le;

			// update vpls list
			vplList.emplace_back(vpl);

			// Russian Roulette for termination
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < sampler->next())
				return;

			pathThroughput = pathThroughput / russianRouletteProbability;

			// Create new ray
			r.init(shadingData.x + (wi * EPSILON), wi);

			// Continue tracing the path recursively
			VPLTracePath(r, pathThroughput, Le, sampler, vplList, depth + 1);
		}
	}

	void traceVPLs(unsigned int id, std::vector<VPL>& vplList)
	{
		Sampler* sampler = samplers[id];

		int total = settings.vplRaysPerTile * numThreads;

		for (unsigned int i = 0; i < settings.vplRaysPerTile; i++)
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
			Vec3 nLight = light->normal(wi);

			float cosTheta = Dot(wi, nLight);
			Color Le = light->evaluate(-wi) * cosTheta / (pmf * pdfPos);

			VPL vpl;
			vpl.shadingData = ShadingData(p, nLight);
			vpl.Le = Le;

			// update vpls list
			vplList.emplace_back(vpl);

			Ray ray(p, wi);
			Color pathThroughput(1.0f, 1.0f, 1.0f);

			VPLTracePath(ray, pathThroughput, Le, sampler, vplList);
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
			if (gTerm > 0 && scene->visible(shadingData.x, vpl.shadingData.x))
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
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);


		if (settings.debug)
		{
			float i;
			if (radiosityDebug(shadingData.x, i))
				return Color(1.0f, 0.0f, 0.0f) * (1.0f - i) + Color(1.0f, 1.0f, 0.0f) * i;
		}

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

	void connectToCamera(Vec3 p, Vec3 n, Color col)
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

	void lightTracePath(Ray& r, Color pathThroughput, Color Le, Sampler* sampler, int depth = 0)
	{
		// Max recursion depth check
		if (depth >= settings.maxBounces)
			return;

		// Traverse the scene to find an intersection
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)  // If the ray hits something
		{
			// If the hit surface is another light or pure specular, stop
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
				return;

			// connect to camera and draw pixel
			Vec3 wi = (scene->camera.origin - shadingData.x).normalize();
			Color col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;
			connectToCamera(shadingData.x, shadingData.sNormal, col);

			// Russian Roulette for termination
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (russianRouletteProbability < sampler->next())
				return;
			pathThroughput = pathThroughput / russianRouletteProbability;

			// Sample new direction
			Color bsdf;
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
		Vec3 nLight = light->normal(wi);

		float cosTheta = Dot(wi, nLight);
		Color Le = light->evaluate(-wi) / (pmf * pdfPos);

		// connect to camera to draw light
		connectToCamera(p, nLight, Le);

		// normalize light if area light
		if (light->isArea())
			Le = Le * cosTheta;

		Ray ray(p, wi);
		Color pathThroughput(1.0f, 1.0f, 1.0f);

		lightTracePath(ray, pathThroughput, Le, sampler);
	}

	// #############################################################################################################

	// PATH TRACE #####################################################################################################

	inline float powerHeuristic(float pdfA, float pdfB)
	{
		float a2 = pdfA * pdfA;
		float b2 = pdfB * pdfB;

		if (a2 + b2 == 0.0f)
			return 0.0f;

		return a2 / (a2 + b2);
	}

	Color computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular())
		{
			return Color(0.0f, 0.0f, 0.0f);
		}

		// Light sampling part
		LightSample light = scene->sampleLightPoint(sampler);
		if (light.isNull)
			return Color(0.0f, 0.0f, 0.0f);

		Vec3 wi = light.p - shadingData.x;
		float lengthSq = light.isArea ? wi.lengthSq() : 1.0f;
		wi = wi.normalize();

		float cosThetaSurface = max(Dot(wi, shadingData.sNormal), 0.0f);
		float gTerm = cosThetaSurface / lengthSq;

		if (light.isArea)
		{
			float cosThetaLight = max(-Dot(wi, light.n), 0.0f);
			gTerm *= cosThetaLight;
		}

		if (gTerm > 0.0f && scene->visible(shadingData.x, light.p))
		{
			Color bsdfVal = shadingData.bsdf->evaluate(shadingData, wi);
			float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);

			float lightPdf = light.pdf * light.pmf;

			// Balance heuristic for MIS
			float misWeight = powerHeuristic(light.pdf, bsdfPdf);

			return (light.emitted * bsdfVal * gTerm * misWeight) / (lightPdf);
		}

		return Color(0.0f, 0.0f, 0.0f);
	}

	Color pathTrace(Ray& r, Color& pathThroughput, int depth, Sampler* sampler, float prevBsdfPdf, bool canHitLight)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			// Handle light hit
			if (shadingData.bsdf->isLight())
			{
				pathThroughput = pathThroughput * shadingData.bsdf->emit(shadingData, r.dir);

				// if last bsdf was pure specular, we cannot use MIS
				if (canHitLight)
					return pathThroughput;

				float lightPdf = scene->getLightPdf(shadingData.lightIndex, r.dir);

				// MIS weight calculation
				float misWeight = powerHeuristic(lightPdf, prevBsdfPdf);

				return pathThroughput * misWeight;
			}

			// Direct lighting (with MIS)
			Color direct = pathThroughput * computeDirect(shadingData, sampler);

			// Russian roulette termination
			if (depth >= settings.maxBounces)
			{
				return direct;
			}

			float rrProbability = min(max(pathThroughput.Lum(), 0.05f), 0.95f);
			if (sampler->next() >= rrProbability)
			{
				return direct;
			}
			pathThroughput = pathThroughput / rrProbability;

			// Sample BSDF for next path segment
			Color bsdf;
			float bsdfPdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, bsdfPdf);

			// Update path throughput
			float cosTheta = abs(Dot(wi, shadingData.sNormal));
			pathThroughput = (pathThroughput * bsdf * cosTheta) / bsdfPdf;

			// Spawn next ray
			r.init(shadingData.x + (wi * EPSILON), wi);

			// Indirect lighting (recursive call)
			Color indirect = pathTrace(r, pathThroughput, depth + 1, sampler, bsdfPdf, shadingData.bsdf->isPureSpecular());

			// Combine direct and indirect lighting
			return direct + indirect;
		}

		// Missed scene - return background
		return scene->background->evaluate(r.dir) * pathThroughput;
	}

	Color pathTrace(Ray& r, Sampler* sampler)
	{
		Color pathThroughput(1.0f, 1.0f, 1.0f);
		return pathTrace(r, pathThroughput, 0, sampler, 0, true);
	}

	// ###################################################################################################################

	Color direct(Ray& r, Sampler* sampler)
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

	Color albedo(Ray& r)
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

	Color viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Color(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Color(0.0f, 0.0f, 0.0f);
	}

	void drawPoint(Vec3 point, Color col)
	{
		Vec3 dir = (point - scene->camera.origin).normalize();
		bool isInFront = dir.dot(scene->camera.viewDirection) > 0.0f;

		if (isInFront)
		{
			float px, py;
			if (scene->camera.projectOntoCamera(point, px, py))
				film->splat(px, py, col);
		}
	}

	void lightDebug(Sampler* sampler)
	{
		LightSample light = scene->sampleLightPoint(sampler);

		if (!light.isNull)
			drawPoint(light.p, light.emitted / (light.pdf * light.pmf));
	}

	void render()
	{
		film->incrementSPP();

		if (settings.algorithm == AL_INSTANT_RADIOSITY && settings.render)
			radiosityVplPass();

		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				if (settings.drawMode == DM_LIGHTS)
				{
					lightDebug(samplers[0]);
					continue;
				}

				if (!settings.render)
				{
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					Color col;
					switch (settings.drawMode)
					{
					case DM_ALBEDO:
						col = albedo(ray);
						break;
					case DM_NORMALS:
						col = viewNormals(ray);
						break;
					case DM_DIRECT:
						col = direct(ray, samplers[0]);
						break;
					}

					film->splat(px, py, col);
				}
				else
				{
					if (settings.algorithm == AL_LIGHT_TRACE)
					{
						lightTrace(samplers[0]);
						continue;
					}
					else
					{
						float px = x + samplers[0]->next();
						float py = y + samplers[0]->next();
						Ray ray = scene->camera.generateRay(px, py);

						Color col;

						switch (settings.algorithm)
						{
						case AL_PATH_TRACE:
							col = pathTrace(ray, samplers[0]);
							break;
						case AL_INSTANT_RADIOSITY:
							col = radiosityLightPass(ray, samplers[0]);
						}

						film->splat(px, py, col);
					}
				}
			}
		}
	}

	// TILE BASED ADAPTIVE SAMPLING ########################################################################################

	void calculateTileSamples()
	{
		for (unsigned int i = 0; i < totalTiles; i++)
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

			tileSamples[i] = (unsigned int)(film->SPP + (settings.totalSPP - film->SPP) * weight);
		}
	}

	// #####################################################################################################################

	// MULTI THREADING #####################################################################################################

	void processTile(unsigned int id)
	{
		unsigned int i;
		while ((i = tileCounter.fetch_add(1)) < totalTiles)
		{
			bool isTileRendered = film->SPP > settings.initSPP &&
				film->SPP > tileSamples[i] &&
				settings.algorithm != AL_LIGHT_TRACE;

			if (isTileRendered)
				continue;

			unsigned int startx = (i % totalXTiles) * tileSize;
			unsigned int starty = (i / totalXTiles) * tileSize;

			unsigned int endx = min(startx + tileSize, film->width);
			unsigned int endy = min(starty + tileSize, film->height);

			for (unsigned int y = starty; y < endy; y++)
			{
				for (unsigned int x = startx; x < endx; x++)
				{
					if (settings.drawMode == DM_LIGHTS)
					{
						lightDebug(samplers[id]);
						continue;
					}

					if (!settings.render)
					{
						float px = x + 0.5f;
						float py = y + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);

						Color col;
						switch (settings.drawMode)
						{
						case DM_ALBEDO:
							col = albedo(ray);
							break;
						case DM_NORMALS:
							col = viewNormals(ray);
							break;
						case DM_DIRECT:
							col = direct(ray, samplers[id]);
							break;
						}
						film->splat(px, py, col);
					}
					else
					{
						if (settings.algorithm == AL_LIGHT_TRACE)
						{
							lightTrace(samplers[id]);
							continue;
						}
						else
						{
							float px = x + samplers[id]->next();
							float py = y + samplers[id]->next();
							Ray ray = scene->camera.generateRay(px, py);

							Color col;

							switch (settings.algorithm)
							{
							case AL_PATH_TRACE:
								col = pathTrace(ray, samplers[id]);
								break;
							case AL_INSTANT_RADIOSITY:
								col = radiosityLightPass(ray, samplers[id]);
							}

							film->splat(px, py, col);
						}
					}
				}
			}
		}
	}

	void renderMT()
	{
		if (settings.debug || settings.drawMode == DM_LIGHTS)
		{
			film->clear();
			film->SPP = 1;
		}
		else
			film->incrementSPP();

		tileCounter.store(0);

		// check for radiosity vpl pass
		if (settings.algorithm == AL_INSTANT_RADIOSITY && settings.render)
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
		if (film->SPP == settings.initSPP && settings.algorithm != AL_LIGHT_TRACE)
			calculateTileSamples();
	}

	// ##################################################################################################################

	void createAOV(AOV& aov)
	{
		aov = AOV(film->width, film->height);

		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < aov.height; y++)
		{
			sppY = (y / tileSize) * totalXTiles;
			for (unsigned int x = 0; x < aov.width; x++)
			{
				// calculate index
				unsigned int index = y * aov.width + x;

				sppIndex = sppY + x / tileSize;
				spp = settings.adaptiveSampling && settings.initSPP < film->SPP ? min(tileSamples[sppIndex], film->SPP) : film->SPP;

				// set colour
				Color col = film->film[index] / (float)spp;
				memcpy(&aov.color[index * 3], &col.rgb, sizeof(float) * 3);

				// create ray
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);

				// set albedo 
				col = albedo(ray);
				memcpy(&aov.albedo[index * 3], &col.rgb, sizeof(float) * 3);

				// set normals
				col = viewNormals(ray);
				memcpy(&aov.normal[index * 3], &col.rgb, sizeof(float) * 3);
			}
		}
	}

	void draw()
	{
		unsigned char r, g, b;
		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < film->height; y++)
		{
			sppY = (y / tileSize) * totalXTiles;
			for (unsigned int x = 0; x < film->width; x++)
			{
				sppIndex = sppY + x / tileSize;
				spp = settings.adaptiveSampling && settings.initSPP < film->SPP ? min(tileSamples[sppIndex], film->SPP) : film->SPP;
				film->tonemap(x, y, r, g, b, spp, settings.toneMap);
				canvas->draw(y * film->width + x, r, g, b);
			}
		}
	}

	void draw(const AOV& aov)
	{
		Color col;
		unsigned char r, g, b;
		unsigned int index, total = film->height * film->width;

		for (unsigned int i = 0; i < total; i++)
		{
			index = i * 3;
			film->tonemap(aov.output[index],
				aov.output[index + 1],
				aov.output[index + 2],
				r, g, b, settings.toneMap);
			canvas->draw(i, r, g, b);
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

	void cycleDrawMode()
	{
		if (settings.render)
			return;

		clear();

		if (settings.drawMode == DM_ALBEDO)
			settings.drawMode = DM_NORMALS;
		else if (settings.drawMode == DM_NORMALS)
			settings.drawMode = DM_DIRECT;
		else if (settings.drawMode == DM_DIRECT)
			settings.drawMode = DM_LIGHTS;
		else
			settings.drawMode = DM_ALBEDO;
	}

	void cycleAlgorithm()
	{
		if (!settings.render)
			return;

		clear();

		if (settings.algorithm == AL_PATH_TRACE)
			settings.algorithm = AL_LIGHT_TRACE;
		else if (settings.algorithm == AL_LIGHT_TRACE)
			settings.algorithm = AL_INSTANT_RADIOSITY;
		else
			settings.algorithm = AL_PATH_TRACE;

		if (settings.adaptiveSampling && settings.algorithm != AL_PATH_TRACE)
			settings.initSPP = settings.totalSPP;
	}

	void toggleRender()
	{
		clear();
		settings.render = !settings.render;
	}
};