#pragma once

#include "Imaging.h"
#include "TerminalUI.h"

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

	// ostream operator using TerminalUI formatting
	friend std::ostream& operator<<(std::ostream& os, const SETTINGS& settings)
	{
		os << tui::color::green(tui::format(
			"=============== Settings ============== \n",
			"  Tone Map       :", (settings.toneMap == TM_NONE ? "None" :
				settings.toneMap == TM_LINEAR ? "Linear" :
				settings.toneMap == TM_LINEAR_EXPOSURE ? "Linear with Exposure" :
				settings.toneMap == TM_REINHARD_GLOBAL ? "Reinhard Global" : "Filmic"), "\n",
			"  Filter         :", (settings.filter == FT_BOX ? "Box" :
				settings.filter == FT_GAUSSIAN ? "Gaussian" : "Mitchell-Netravali"), "\n\n",
			"  Multithreading :", (settings.useMultithreading ? "Enabled" : "Disabled"), "\n",
			"  Denoise        :", (settings.denoise ? "Enabled" : "Disabled"), "\n",
			"  Save Renders   :", (settings.saveRenders ? "Enabled" : "Disabled"), "\n\n",
			"  Threads        :", settings.numThreads, "\n",
			"  Max Bounces    :", settings.maxBounces, "\n",
			"  Total SPP      :", settings.totalSPP, "\n",
			"======================================= \n"));
		return os;
	}
};

