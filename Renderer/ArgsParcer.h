#pragma once
#include <string>
#include <unordered_map>
#include <filesystem>
#include <algorithm>

#include "Core.h"
#include "Settings.h"
#include "TerminalUI.h"

static class ArgsParser
{
	enum ARG_KEYS
	{
		ARG_UNKNOWN,
		ARG_SCENE,
		ARG_SPP,
		ARG_BOUNCES,
		ARG_THREADS,
		ARG_DENOISE,
		ARG_SAVERENDER,
		ARG_FILTER,
		ARG_TONEMAP
	};

	static std::string getKeyName(ARG_KEYS key)
	{
		switch (key)
		{
		case ARG_SPP: return "-spp";
		case ARG_SCENE: return "-scene";
		case ARG_FILTER: return "-filter";
		case ARG_BOUNCES: return "-bounces";
		case ARG_THREADS: return "-threads";
		case ARG_DENOISE: return "-denoise";
		case ARG_TONEMAP: return "-toneMap";
		case ARG_SAVERENDER: return "-saveRender";
		default: return "unknown";
		}
	}

	static ARG_KEYS getKeyType(std::string key)
	{
		// convert key to lower case
		std::transform(key.begin(), key.end(), key.begin(), ::tolower);

		if (key == "-spp") return ARG_SPP;
		if (key == "-scene") return ARG_SCENE;
		if (key == "-filter") return ARG_FILTER;
		if (key == "-bounces") return ARG_BOUNCES;
		if (key == "-threads") return ARG_THREADS;
		if (key == "-tonemap") return ARG_TONEMAP;
		if (key == "-denoise") return ARG_DENOISE;
		if (key == "-saverender") return ARG_SAVERENDER;

		return ARG_UNKNOWN; // default
	}

	static bool validateKeyValue(ARG_KEYS key, std::string value)
	{
		switch (key)
		{
		case ARG_SCENE:
			return !value.empty() && std::filesystem::exists(value);
		case ARG_SPP:
			try {
				int intValue = std::stoi(value);
				return intValue >= 10 && intValue <= 100000;
			}
			catch (...) { return false; }
			break;
		case ARG_BOUNCES:
			try {
				int intValue = std::stoi(value);
				return intValue >= 2 && intValue <= 100;
			}
			catch (...) { return false; }
			break;
		case ARG_THREADS:
			try {
				int intValue = std::stoi(value);
				return intValue >= 1 && intValue <= 64;
			}
			catch (...) { return false; }
			break;
		case ARG_DENOISE:
			return value == "0" || value == "1";
		case ARG_SAVERENDER:
			return value == "0" || value == "1";
		case ARG_FILTER:
			try {
				int intValue = std::stoi(value);
				return intValue >= 0 && intValue <= 2;
			}
			catch (...) { return false; }
			break;
		case ARG_TONEMAP:
			try {
				int intValue = std::stoi(value);
				return intValue >= 0 && intValue <= 4;
			}
			catch (...) { return false; }
			break;
		default:
			return false;
		}
		return false;
	}

	static std::unordered_map<ARG_KEYS, std::string> generateArgs(int argc, char* argv[])
	{
		// generate a map of key-value pairs from command line arguments
		std::unordered_map<ARG_KEYS, std::string> argsMap;

		ARG_KEYS key;
		std::string value;
		for (int i = 1; i < argc; i++)
		{
			// get key type and value
			key = getKeyType(argv[i]);
			value = (i + 1) < argc ? argv[i + 1] : "";

			// only add valid key-value pairs
			if (validateKeyValue(key, value))
				argsMap[getKeyType(argv[i])] = value;
		}
		return argsMap;
	}

	static void printHelp()
	{
		// print usage with all options using tui
		std::cout << tui::format(
			"Usage: \"Ray Tracer.exe\" [options] from terminal for custom inputs\n",
			"Options:\n",
			"  -scene <path>       : Path to the scene file (required)\n",
			"  -spp <int>          : Total samples per pixel (default: 100, range: 10-100000)\n",
			"  -bounces <int>      : Maximum bounces for path tracing (default: 5, range: 2-100)\n",
			"  -threads <int>      : Number of threads for multithreading (default: 8, range: 1-64)\n",
			"  -denoise <0|1>      : Enable denoising (1 to enable, 0 to disable, default: 0)\n",
			"  -saveRenders <0|1>  : Save rendered and denoised images (1 to enable, 0 to disable, default: 0)\n",
			"  -filter <0-2>       : Image filter type (0: Box, 1: Gaussian, 2: Mitchell-Netravali; default: 0)\n",
			"  -toneMap <0-4>      : Tone mapping operator (0: None, 1: Linear, 2: Linear with Exposure, 3: Reinhard Global, 4: Filmic; default: 3)\n",
			"\nExample:\n",
			"  RayTracer -scene \"scenes/cornell-box\" -spp 500 -bounces 10 -threads 4 -denoise 1 -saveRenders 1 -filter 1 -toneMap 4\n\n"
		);
	}
public:

	static std::string parse(int argc, char* argv[], SETTINGS& settings)
	{
		// argumnt contains the scene name and settings
		// example: -scene "scenes/cornell-box" -spp 100 -bounces 5 -threads 4 -denoise 1 -saveRenders 1 -numThreads 8 -filter 0 -toneMap 3 -drawMode 2 -algorithm 0
		// create settings and return scene path if valid else return null

		if (argc < 2)
		{
			printHelp();
			return "";
		}

		std::unordered_map<ARG_KEYS, std::string> argsMap = generateArgs(argc, argv);

		// update settings
		for (const auto& [key, value] : argsMap)
		{
			switch (key)
			{
			case ARG_SPP:
				settings.totalSPP = clamp(std::stoi(value), 10, 100000);
				break;
			case ARG_BOUNCES:
				settings.maxBounces = clamp(std::stoi(value), 2, 100);
				break;
			case ARG_THREADS:
				settings.numThreads = clamp(std::stoi(value), 1, 64);
				settings.useMultithreading = settings.numThreads > 1;
				break;
			case ARG_DENOISE:
				settings.denoise = value == "1";
				break;
			case ARG_SAVERENDER:
				settings.saveRenders = value == "1";
				break;
			case ARG_FILTER:
				settings.filter = (IMAGE_FILTER)clamp(std::stoi(value), 0, 2);
				break;
			case ARG_TONEMAP:
				settings.toneMap = (TONEMAP)clamp(std::stoi(value), 0, 4);
				break;
			default:
				break;
			}
		}

		// check if scene path is valid and return it
		return argsMap.find(ARG_SCENE) != argsMap.end() ? argsMap[ARG_SCENE] : "";
	}
};