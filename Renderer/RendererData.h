#pragma once

#include "Scene.h"
#include "GamesEngineeringBase.h"
#include "Settings.h"

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