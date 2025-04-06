

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

SETTINGS createSettings()
{
	SETTINGS settings;

	settings.algorithm = AL_PATH_TRACE;
	settings.drawMode = DM_ALGORITHM;
	settings.toneMap = TM_LINEAR;
	settings.filter = FT_BOX;

	settings.canHitLight = true;
	settings.debug = false;
	settings.denoise = true;

	settings.useMultithreading = true;
	settings.useMis = true;

	settings.adaptiveSampling = true;
	settings.initSPP = 20;
	settings.totalSPP = 200;

	settings.numThreads = 20;
	settings.maxBounces = 5;
	settings.vplRaysPerTile = 1;

	return settings;
}

// scene names
const std::string scenes[] = { "scenes/cornell-box",		// 0
							   "scenes/bathroom",			// 1
							   "scenes/bathroom2",			// 2
							   "scenes/bedroom",			// 3
							   "scenes/classroom",			// 4
							   "scenes/coffee",				// 5
							   "scenes/dining-room",		// 6		
							   "scenes/glass-of-water",		// 7
							   "scenes/house",				// 8
							   "scenes/kitchen",			// 9
							   "scenes/living-room",		// 10		
							   "scenes/living-room-2",		// 11
							   "scenes/living-room-3",		// 12
							   "scenes/Sibenik",			// 13	
							   "scenes/staircase",			// 14	
							   "scenes/staircase2",			// 15		
							   "scenes/Terrain",			// 16	
							   "scenes/veach-bidir",		// 17		
							   "scenes/veach-mis",			// 18	
							   "scenes/MaterialsScene",		// 19
							   "scenes/car2",				// 20
							   "scenes/materialball",		// 21
							   "scenes/teapot-full",		// 22
							   "scenes/Sponza",				// 23
};

// current scene number
const unsigned int sceneNum = 19;

int main(int argc, char* argv[])
{
	SETTINGS settings = createSettings();

	std::string sceneName = scenes[sceneNum];
	std::string filename = scenes[sceneNum] + "_GI.hdr";

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				}
				else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			}
			else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				settings.totalSPP = stoi(pair.second);
			}
		}
	}

	// Load scene and camera
	RTCamera viewcamera;
	std::cout << "Loading scene: " << scenes[sceneNum] << std::endl;
	Scene* scene = loadScene(sceneName, viewcamera);

	// Create canvas
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", 1.0f);

	// Create ray tracer
	RayTracer rt;
	rt.init(scene, &canvas, settings);	// 10 threads

	// Create timer
	GamesEngineeringBase::Timer timer;
	float totalTime = 0;

	bool running = true;
	bool completed = false;
	AOV aov;

	std::cout << "\n\n\n\n";
	while (running)
	{
		canvas.checkInput(); // Check for input

		// Check if the user wants to quit
		if (canvas.isQuitRequested() || canvas.keyPressed(VK_ESCAPE))
		{
			running = false;
			continue;
		}

		// Update camera and check if it has changed (reset if it has)
		if (viewcamera.update(canvas))
		{
			rt.clear();
			totalTime = 0;
			if (completed)
			{
				std::cout << "\n\n\n\n";
				completed = false;
			}
		}

		canvas.clear();

		if (!completed)
		{
			// Time how long a render call takes
			timer.reset();

			if (settings.useMultithreading)
				rt.renderMT();
			else
				rt.render();

			float t = timer.dt();

			totalTime += t; // update total time

			// Write stats to console
			std::cout << "\033[F\033[F\033[F\033[F";
			std::cout << "Samples    : " << rt.getSPP() << "            \n";
			std::cout << "Time       : " << t << "                      \n";
			std::cout << "FPS        : " << (t > 0 ? 1.0f / t : FLT_MAX) << "              \n";
			std::cout << "Total time : " << std::roundf(totalTime) << " sec                \n";
		}

		if (canvas.keyPressed('P'))
		{
			rt.savePNG(filename);
		}

		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}

		if (!completed && settings.totalSPP <= rt.getSPP() && settings.drawMode == DM_ALGORITHM)
		{
			completed = true;

			rt.saveHDR(filename);
			if (settings.denoise)
			{
				rt.createAOV(aov);
				Denoiser denoiser(aov.width, aov.height);
				denoiser.denoise(aov);
			}
		}

		if (completed && settings.denoise)
			rt.draw(aov);
		else
			rt.draw();

		canvas.present();
	}

	delete scene;

	return 0;
}