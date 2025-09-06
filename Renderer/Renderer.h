#pragma once

#include "Imaging.h"
#include <thread>
#include <functional>

#include "Denoiser.h"
#include "algorithms/PathTracing.h"
#include "algorithms/LightTracing.h"
#include "algorithms/InstantRadiosity.h"

class Renderer
{
public:
	RENDERER_DATA data;		// multithreading data

	AlgorithmBase* algorithm; // current rendering algorithm

	~Renderer()
	{
		std::cout << "Cleaning Ray Tracer..." << std::endl;
		delete algorithm;
	}

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, SETTINGS _settings)
	{
		data.scene = _scene;
		data.canvas = _canvas;
		data.settings = _settings;

		data.film = new Film();
		data.film->init((unsigned int)data.scene->camera.width, (unsigned int)data.scene->camera.height, data.settings.filter);

		// only use adaptive sampling for path tracing
		data.settings.adaptiveSampling = data.settings.adaptiveSampling && data.settings.algorithm == AL_PATH_TRACE;
		if (!(data.settings.adaptiveSampling && data.settings.useMultithreading))
			data.settings.initSPP = data.settings.totalSPP;

		setMultithreadingData(data.settings.numThreads);
		setTileData();

		algorithm = new InstantRadiosity(data);
	}

	void setTileData()
	{
		// set tile size
		data.tileSize = 16;

		// calculate number of tiles
		data.totalXTiles = (data.canvas->getWidth() + data.tileSize - 1) / data.tileSize;
		float totalYTiles = (data.canvas->getHeight() + data.tileSize - 1) / data.tileSize;
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

	void clear()
	{
		data.film->clear();
	}

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
		Vec3 dir = (point - data.scene->camera.origin).normalize();
		bool isInFront = dir.dot(data.scene->camera.viewDirection) > 0.0f;

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
			end = end.Min(Vec2i(data.film->width, data.film->height));

			renderTile(start, end, data.samplers[id]);
		}
	}

	void debug()
	{
		clear();

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
			renderTile(Vec2i(0, 0), Vec2i(data.film->width, data.film->height), data.samplers[0]);
	}

	void render()
	{
		if (data.settings.render)
			algorithm->render();
		else
			debug();
	}

	// ##################################################################################################################

	void createAOV(AOV& aov)
	{
		aov = AOV(data.film->width, data.film->height);

		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < aov.height; y++)
		{
			sppY = (y / data.tileSize) * data.totalXTiles;
			for (unsigned int x = 0; x < aov.width; x++)
			{
				// calculate index
				unsigned int index = y * aov.width + x;

				sppIndex = sppY + x / data.tileSize;
				spp = data.film->SPP;
				// spp = data.settings.adaptiveSampling && data.settings.initSPP < data.film->SPP ? min(tileSamples[sppIndex], film->SPP) : film->SPP;

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

	void draw()
	{
		unsigned char r, g, b;
		int sppY, spp, sppIndex;
		for (unsigned int y = 0; y < data.film->height; y++)
		{
			sppY = (y / data.tileSize) * data.totalXTiles;
			for (unsigned int x = 0; x < data.film->width; x++)
			{
				sppIndex = sppY + x / data.tileSize;
				spp = data.film->SPP > 0 ? data.film->SPP : 1;
				// spp = data.settings.adaptiveSampling && data.settings.initSPP < data.film->SPP ? min(tileSamples[sppIndex], film->SPP) : film->SPP;
				data.film->tonemap(x, y, r, g, b, spp, data.settings.toneMap);
				data.canvas->draw(y * data.film->width + x, r, g, b);
			}
		}
	}

	void draw(const AOV& aov)
	{
		Color col;
		unsigned char r, g, b;
		unsigned int index, total = data.film->height * data.film->width;

		for (unsigned int i = 0; i < total; i++)
		{
			index = i * 3;
			data.film->tonemap(aov.output[index],
				aov.output[index + 1],
				aov.output[index + 2],
				r, g, b, data.settings.toneMap);
			data.canvas->draw(i, r, g, b);
		}
	}

	int getSPP() const
	{
		return data.film->SPP;
	}

	void saveHDR(std::string filename)
	{
		data.film->save(filename);
	}

	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), data.canvas->getWidth(), data.canvas->getHeight(), 3, data.canvas->getBackBuffer(), data.canvas->getWidth() * 3);
	}

	void cycleDrawMode()
	{
		if (data.settings.render)
			return;

		clear();

		if (data.settings.drawMode == DM_ALBEDO)
			data.settings.drawMode = DM_NORMALS;
		else if (data.settings.drawMode == DM_NORMALS)
			data.settings.drawMode = DM_DIRECT;
		else if (data.settings.drawMode == DM_DIRECT)
			data.settings.drawMode = DM_LIGHTS;
		else
			data.settings.drawMode = DM_ALBEDO;
	}

	void cycleAlgorithm()
	{
		if (!data.settings.render)
			return;

		clear();

		if (data.settings.algorithm == AL_PATH_TRACE)
			data.settings.algorithm = AL_LIGHT_TRACE;
		else if (data.settings.algorithm == AL_LIGHT_TRACE)
			data.settings.algorithm = AL_INSTANT_RADIOSITY;
		else
			data.settings.algorithm = AL_PATH_TRACE;

		if (data.settings.adaptiveSampling && data.settings.algorithm != AL_PATH_TRACE)
			data.settings.initSPP = data.settings.totalSPP;
	}

	void toggleRender()
	{
		clear();
		data.settings.render = !data.settings.render;
	}
};