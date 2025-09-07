#pragma once

#include "Imaging.h"
#include <thread>
#include <functional>

#include "Denoiser.h"
#include "algorithms/PathTracing.h"
#include "algorithms/LightTracing.h"
#include "algorithms/Radiosity.h"

/// <summary>
/// Manages and selects different rendering algorithms for use in rendering operations.
/// </summary>
class AlgorithmFactory
{
	std::map<ALGORITHM, std::unique_ptr<Algorithm>> algorithms;
	ALGORITHM currentAlgorithm;
public:
	AlgorithmFactory(RENDERER_DATA& data)
	{
		algorithms[AL_PATH_TRACE] = std::make_unique<PathTracing>(data);
		algorithms[AL_LIGHT_TRACE] = std::make_unique<LightTracing>(data);
		algorithms[AL_INSTANT_RADIOSITY] = std::make_unique<Radiosity>(data);
		currentAlgorithm = AL_PATH_TRACE;
	}

	void render() { algorithms[currentAlgorithm]->render(); }
	void draw() { algorithms[currentAlgorithm]->draw(); }
	int getSpp(int index) { return algorithms[currentAlgorithm]->getSpp(index); }
	void setAlgorithm(ALGORITHM algo) { currentAlgorithm = algo; }
};


/// <summary>
/// Renderer class that handles rendering operations using different algorithms.
/// </summary>
class Renderer
{
	RENDERER_DATA data;							// renderer data used by all algorithms
	std::unique_ptr<AlgorithmFactory> factory;	// algorithm factory

	// ##################################################################################
	// DATA SETUP
	// ##################################################################################

	void setTileData()
	{
		// set tile size
		data.tileSize = 32;

		// calculate number of tiles
		data.totalXTiles = (data.film->size.x + data.tileSize - 1) / data.tileSize;
		float totalYTiles = (data.film->size.y + data.tileSize - 1) / data.tileSize;
		data.totalTiles = data.totalXTiles * totalYTiles;

		data.tileCounter.store(0);
	}

	void setMultithreadingData(unsigned int _numThreads)
	{
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		data.numProcs = sysInfo.dwNumberOfProcessors;

		// calculate number of threads according to available processors
		data.numThreads = max(1, min(_numThreads, data.numProcs));

		// create threads and samplers for each thread
		data.threads = new std::thread * [data.numThreads];
		data.samplers = new MTRandom * [data.numThreads];

		// assign different seeds to each sampler
		// Linear Congruential Generator used for seed
		// x + 1 = [a * (x - 1) + c] % m
		// where a = 48271, c = 0
		int m = pow(2, 32) - 1;
		for (unsigned int i = 0; i < data.numThreads; i++)
			data.samplers[i] = new MTRandom((48271 * (i + 1)) % m);
	}

	// ##################################################################################
	// DEBUG DRAW MODES
	// ##################################################################################

	Color albedo(Ray& r)
	{
		IntersectionData intersection = data.scene->traverse(r);
		ShadingData shadingData = data.scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return data.scene->background->evaluate(r.dir);
	}

	Color viewNormals(Ray& r)
	{
		IntersectionData intersection = data.scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = data.scene->calculateShadingData(intersection, r);
			return Color(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Color(0.0f, 0.0f, 0.0f);
	}

	void drawPoint(Vec3 point, Color col)
	{
		Vec3 dir = (point - data.scene->camera.pos).normalize();
		bool isInFront = dir.dot(data.scene->camera.dir) > 0.0f;

		if (isInFront)
		{
			Vec2 p;
			if (data.scene->camera.projectOntoCamera(point, p))
				data.film->splat(p, col);
		}
	}

	void lightDebug(Sampler* sampler)
	{
		LightSample light = data.scene->sampleLight(sampler);

		if (!light.isNull)
			drawPoint(light.p, light.emitted / (light.pdf * light.pmf));
	}

	// ##################################################################################
	// DEBUG RENDERING
	// ##################################################################################

	void renderTile(const Vec2i& start, const Vec2i& end, const Sampler* sampler)
	{
		for (unsigned int y = start.y; y < end.y; y++)
		{
			for (unsigned int x = start.x; x < end.x; x++)
			{
				if (data.settings.drawMode == DM_LIGHTS)
				{
					lightDebug((Sampler*)sampler);
					continue;
				}

				// jittered pixel coordinates
				Vec2 pixel = Vec2(x, y) + 0.5f;
				Ray ray = data.scene->camera.generateRay(pixel);

				Color col;

				switch (data.settings.drawMode)
				{
				case DM_ALBEDO:
					col = albedo(ray);
					break;
				default:
					col = viewNormals(ray);
				}

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

	void debug()
	{
		data.film->clear();

		if (data.settings.useMultithreading)
		{
			data.tileCounter.store(0);

			// process all tiles
			for (int i = 0; i < data.numThreads; i++)
				data.threads[i] = new std::thread(&Renderer::processTiles, this, i);

			for (int i = 0; i < data.numThreads; i++)
			{
				data.threads[i]->join();
				delete data.threads[i];
			}
		}
		else
			renderTile(Vec2i(0, 0), data.film->size, data.samplers[0]);
	}

	// ##################################################################################
	// DENOISING
	// ##################################################################################

	void createAOV(AOV& aov)
	{
		aov = AOV(data.film->size);

		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < aov.size.y; y++)
		{
			sppY = (y / data.tileSize) * data.totalXTiles;
			for (unsigned int x = 0; x < aov.size.x; x++)
			{
				// calculate index
				unsigned int index = y * aov.size.x + x;

				sppIndex = sppY + x / data.tileSize;
				spp = factory->getSpp(sppIndex);

				// set colour
				Color col = data.film->film[index] / (float)spp;
				memcpy(&aov.color[index * 3], &col.rgb, sizeof(float) * 3);

				// create ray
				Vec2 pixel = Vec2(x + 0.5f, y + 0.5f);
				Ray ray = data.scene->camera.generateRay(pixel);

				// set albedo 
				col = albedo(ray);
				memcpy(&aov.albedo[index * 3], &col.rgb, sizeof(float) * 3);

				// set normals
				col = viewNormals(ray);
				memcpy(&aov.normal[index * 3], &col.rgb, sizeof(float) * 3);
			}
		}
	}

	void drawDenoised()
	{
		Color col;
		unsigned char r, g, b;
		unsigned int index, total = data.film->size.y * data.film->size.x;

		for (unsigned int i = 0; i < total; i++)
		{
			index = i * 3;
			data.film->tonemap(data.aov.output[index],
				data.aov.output[index + 1],
				data.aov.output[index + 2],
				r, g, b, data.settings.toneMap);
			data.canvas->draw(i, r, g, b);
		}
	}

	// ##################################################################################

public:

	Renderer(Scene* _scene, GamesEngineeringBase::Window* _canvas, SETTINGS _settings)
	{
		data.scene = _scene;
		data.canvas = _canvas;
		data.settings = _settings;

		data.film = new Film();
		data.film->init(data.scene->camera.size, data.settings.filter);

		setMultithreadingData(data.settings.numThreads);
		setTileData();

		factory = std::make_unique<AlgorithmFactory>(data);
	}

	~Renderer()
	{
		std::cout << "Cleaning Ray Tracer..." << std::endl;
	}

	// ##################################################################################
	// GETTERS
	// ##################################################################################

	bool isCompleted() const { return data.isCompleted; }
	bool isRendering() const { return data.isRendering; }
	std::string getAlgorithm() const
	{
		switch (data.settings.algorithm)
		{
		case AL_PATH_TRACE:return "Path Tracing";
		case AL_LIGHT_TRACE:return "Light Tracing";
		case AL_INSTANT_RADIOSITY:return "Instant Radiosity";
		default:return "Unknown";
		}
	}

	bool getProgress(float& progress, int& spp) const
	{
		if (!data.isRendering)
			return false;

		spp = data.film->SPP;
		progress = (float)spp / data.settings.totalSPP;
		return true;
	}

	// ##################################################################################
	// PUBLIC RENDERING INTERFACE
	// ##################################################################################

	void clear()
	{
		data.film->clear();
		data.isCompleted = false;
		data.isDenoised = false;
	}

	// returns true if rendering is in progress
	void render()
	{
		if (data.isRendering)
			factory->render();
		else
			debug();
	}

	void draw()
	{
		if (data.isRendering)
		{
			if (data.isDenoised)
				drawDenoised();
			else
				factory->draw();
		}
		else
		{
			unsigned char r, g, b;
			for (unsigned int y = 0; y < data.film->size.y; y++)
			{
				for (unsigned int x = 0; x < data.film->size.x; x++)
				{
					int spp = data.film->SPP > 0 ? data.film->SPP : 1;
					data.film->tonemap(x, y, r, g, b, spp, data.settings.toneMap);
					data.canvas->draw(y * data.film->size.x + x, r, g, b);
				}
			}
		}
	}

	void denoise()
	{
		if (data.isDenoised || !data.settings.denoise)
			return;

		// create AOVs
		createAOV(data.aov);
		// denoise
		Denoiser denoiser(data.aov.size.x, data.aov.size.y);
		denoiser.denoise(data.aov);
		data.isDenoised = true;
	}

	// ##################################################################################
	// IMAGE SAVING
	// ##################################################################################

	void saveHDR(std::string filename) { data.film->save(filename); }

	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), data.canvas->getWidth(), data.canvas->getHeight(), 3, data.canvas->getBackBuffer(), data.canvas->getWidth() * 3);
	}

	// ##################################################################################
	// SETTINGS MODIFICATION
	// ##################################################################################


	void cycleDrawMode()
	{
		if (data.isRendering)
			return;

		clear();

		if (data.settings.drawMode == DM_ALBEDO)
			data.settings.drawMode = DM_NORMALS;
		else if (data.settings.drawMode == DM_NORMALS)
			data.settings.drawMode = DM_LIGHTS;
		else
			data.settings.drawMode = DM_ALBEDO;
	}

	void cycleAlgorithm()
	{
		if (data.isRendering)
			return;

		if (data.settings.algorithm == AL_PATH_TRACE)
			data.settings.algorithm = AL_LIGHT_TRACE;
		else if (data.settings.algorithm == AL_LIGHT_TRACE)
			data.settings.algorithm = AL_INSTANT_RADIOSITY;
		else
			data.settings.algorithm = AL_PATH_TRACE;

		factory->setAlgorithm(data.settings.algorithm);
	}

	void toggleRender()
	{
		clear();
		data.isRendering = !data.isRendering;
	}
};