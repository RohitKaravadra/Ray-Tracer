#pragma once

#include "Imaging.h"

enum ALGORITHM
{
	AL_PATH_TRACE,
	AL_LIGHT_TRACE,
	AL_INSTANT_RADIOSITY
};

enum DRAW_MODE
{
	DM_NORMALS,
	DM_ALBEDO,
	DM_LIGHTS
};

struct SETTINGS
{
	ALGORITHM algorithm;
	DRAW_MODE drawMode;
	TONEMAP toneMap;
	IMAGE_FILTER filter;

	bool useMultithreading;			// multithreading enabled
	bool debug;
	bool denoise;					// set to true to denoise the image
	bool saveRenders;				// save rendered and denoised images

	unsigned int numThreads;		// number of threads for multithreading
	unsigned int maxBounces;		// max number of bounces for path tracing

	unsigned int totalSPP;			// total samples per pixel

	SETTINGS()
	{
		algorithm = AL_PATH_TRACE;
		drawMode = DM_NORMALS;
		toneMap = TM_NONE;
		filter = FT_BOX;

		useMultithreading = false;
		debug = false;
		denoise = false;
		saveRenders = false;

		numThreads = 3;
		maxBounces = 5;

		totalSPP = 8192;
	}
};

