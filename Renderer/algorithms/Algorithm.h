#pragma once

#include "../Scene.h"
#include "../Settings.h"
#include "../GamesEngineeringBase.h"

/// <summary>
/// Renderer data structure holding all necessary information for rendering.
/// Shared among different algorithms.
/// </summary>
struct RENDERER_DATA
{
	Scene* scene;				// pointer to the scene (handled by scene manager do not delete)
	GamesEngineeringBase::Window* canvas;
	Film* film;

	MTRandom** samplers;		// samplers for multithreading
	std::thread** threads;		// threads for multithreading
	int numProcs;				// number of processors
	unsigned int numThreads;	// number of threads

	unsigned int totalTiles;	// number of tiles
	unsigned int totalXTiles;	// number of tiles in x direction	
	unsigned int tileSize;		// size of each tile
	std::atomic<unsigned int> tileCounter;	// counter to process tiles

	bool isRendering; 			// flag to indicate if rendering is in progress
	bool isCompleted;			// flag to indicate if rendering is completed
	bool isDenoised;			// flag to indicate if denoising is completed

	AOV aov;					// active AOV

	SETTINGS settings;			// rendering settings

	~RENDERER_DATA()
	{
		// clean film
		delete film;

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
	}
};

// Algorithm interface
class Algorithm
{
public:
	virtual void render() = 0;				// render a single iteration
	virtual int getSpp(int index) = 0;		// get the current samples per pixel
	virtual void draw() = 0;				// draw the current image to the canvas
};

// Base class for all algorithms to inherit from
class AlgorithmBase : public Algorithm
{
	/// <summary>
	/// Performs a rendering step, updating the sample count and checking for completion.
	/// calls the process() method to perform the actual rendering work.
	/// </summary>
	void render() override
	{
		if (data.isCompleted)
			return;

		data.film->incrementSPP();
		process();

		// check if rendering is complete
		if (!data.isCompleted)
			data.isCompleted = data.film->SPP >= data.settings.totalSPP;
	}

protected:
	RENDERER_DATA& data;
	AlgorithmBase(RENDERER_DATA& data) : data(data) {}

	/// <summary>
	/// Pure virtual function that processes data or performs an operation. Must be implemented by derived classes.
	/// </summary>
	virtual void process() = 0;

	/// <summary>
	/// Renders the film data onto the canvas by applying tone mapping and drawing each pixel.
	/// Can be overridden by derived classes for custom drawing behavior.
	/// </summary>
	virtual void draw() override
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

	/// <summary>
	/// Returns the samples per pixel (SPP) value from the film data.
	/// </summary>
	/// <param name="index">The index parameter.</param>
	/// <returns>The samples per pixel (SPP) value as an integer.</returns>
	virtual int getSpp(int index) { return data.film->SPP; }
};