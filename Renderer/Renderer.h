#pragma once

#include "Imaging.h"
#include <thread>
#include <functional>

#include "Denoiser.h"
#include "algorithms/PathTracing.h"
#include "algorithms/LightTracing.h"
#include "algorithms/Radiosity.h"
#include "algorithms/Bidirectional.h"

/// <summary>
/// Manages and selects different rendering algorithms for use in rendering operations.
/// </summary>
class AlgorithmFactory {
	std::map<ALGORITHM, std::shared_ptr<Algorithm>> algorithms;

public:

	std::weak_ptr<Algorithm> getAlgorithm(RENDERER_DATA& data) {
		ALGORITHM algorithm = data.settings.algorithm;

		// check if algorithm exists
		auto it = algorithms.find(algorithm);
		if (it != algorithms.end())
			return it->second;

		// if not, create a new one
		switch (algorithm) {
		case AL_PATH_TRACE:
			algorithms[algorithm] = std::make_shared<PathTracing>(data);
			break;
		case AL_LIGHT_TRACE:
			algorithms[algorithm] = std::make_shared<LightTracing>(data);
			break;
		case AL_INSTANT_RADIOSITY:
			algorithms[algorithm] = std::make_shared<Radiosity>(data);
			break;
		case AL_BIDIRECTIONAL:
			algorithms[algorithm] = std::make_shared<Bidirectional>(data);
			break;
		default:
			return !algorithms.empty()
				? algorithms.begin()->second
				: std::weak_ptr<Algorithm>();
		}

		return algorithms[algorithm];
	}
};

/// <summary>
/// Renderer class that handles rendering operations using different algorithms.
/// </summary>
class Renderer
{
	RENDERER_DATA data;							// renderer data used by all algorithms
	std::unique_ptr<AlgorithmFactory> factory;	// algorithm factory
	std::weak_ptr<Algorithm> currAlgorithm;		// current algorithm

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
		SurfaceData surfaceData = data.scene->calculateShadingData(intersection, r);
		ShadingData& shadingData = surfaceData.shadingData;
		if (surfaceData.t < FLT_MAX)
		{
			if (surfaceData.bsdf->isLight())
			{
				return surfaceData.bsdf->emit(shadingData, shadingData.wo);
			}
			return surfaceData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return data.scene->background->evaluate(r.dir);
	}

	Color viewNormals(Ray& r)
	{
		IntersectionData intersection = data.scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			SurfaceData surfaceData = data.scene->calculateShadingData(intersection, r);
			ShadingData& shadingData = surfaceData.shadingData;
			return Color(fabsf(shadingData.n.x), fabsf(shadingData.n.y), fabsf(shadingData.n.z));
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

				if (auto algo = currAlgorithm.lock())
					spp = algo.get()->getSpp(sppIndex);
				else
					throw std::runtime_error(" No algorithm assigned ");

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

		factory = std::make_unique<AlgorithmFactory>();
		currAlgorithm = factory->getAlgorithm(data);
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
		case AL_BIDIRECTIONAL:return "Bidirectional";
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
		{
			if (auto algo = currAlgorithm.lock())
				algo.get()->render();
			else
				tui::print(tui::color::yellow("WARNING: No algorithm Assigned, can't render"));
		}
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
			{
				if (auto algo = currAlgorithm.lock())
					algo.get()->draw();
				else
					tui::print(tui::color::yellow("WARNING: No algorithm Assigned, can't draw"));
			}
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

	void saveHDR(std::string filename)
	{
		if (auto algo = currAlgorithm.lock())
		{
			int total = data.film->size.x * data.film->size.y;
			Color* hdrpixels = new Color[total];

			int sppY, spp, pixelY;
			for (unsigned int y = 0; y < data.film->size.y; y++)
			{
				pixelY = y * data.film->size.x;
				sppY = (y / data.tileSize) * data.totalXTiles;
				for (unsigned int x = 0; x < data.film->size.x; x++)
				{
					spp = max(algo.get()->getSpp(sppY + x / data.tileSize), 1);
					hdrpixels[pixelY + x] = data.film->film[pixelY + x] / (float)spp;
				}
			}

			stbi_write_hdr(filename.c_str(), data.film->size.x, data.film->size.y, 3, (float*)hdrpixels);
			delete[] hdrpixels;
		}
	}

	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), data.canvas->getWidth(), data.canvas->getHeight(), 3, data.canvas->getBackBuffer(), data.canvas->getWidth() * 3);
	}

	// ##################################################################################
	// SETTINGS MODIFICATION
	// ##################################################################################


	void cycleDrawMode()
	{
		if (data.isRendering) return;
		clear();
		data.settings.drawMode = (DRAW_MODE)((data.settings.drawMode + 1) % DRAW_MODE_COUNT);
	}

	void cycleAlgorithm()
	{
		if (data.isRendering) return;
		data.settings.algorithm = (ALGORITHM)((data.settings.algorithm + 1) % ALGORITHM_COUNT);
		currAlgorithm = factory->getAlgorithm(data);
	}

	void toggleRender()
	{
		clear();
		data.isRendering = !data.isRendering;
	}
};