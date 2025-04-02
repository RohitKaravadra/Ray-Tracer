

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include <iomanip>

void runTests()
{
	// Add test code here
}

std::string formatTime(int seconds) {
	int hours = seconds / 3600;
	int minutes = (seconds % 3600) / 60;
	int secs = seconds % 60;

	std::ostringstream formattedTime;
	formattedTime << std::setw(2) << std::setfill('0') << hours << ":"
		<< std::setw(2) << std::setfill('0') << minutes << ":"
		<< std::setw(2) << std::setfill('0') << secs;

	return formattedTime.str();
}

int main(int argc, char* argv[])
{
	// Add call to tests if required
	// runTests()

	// scene names
	std::string scenes[] = { "cornell-box",		// 0
							"bathroom",			// 1
							"bathroom2",		// 2
							"bedroom",			// 3
							"classroom",		// 4
							"coffee",			// 5
							"dining-room",		// 6		
							"glass-of-water",	// 7
							"house",			// 8
							"kitchen",			// 9
							"living-room",		// 10		
							"living-room-2",	// 11
							"living-room-3",	// 12
							"Sibenik",			// 13	
							"staircase",		// 14	
							"staircase2",		// 15		
							"Terrain",			// 16	
							"veach-bidir",		// 17		
							"veach-mis",		// 18	
							"MaterialsScene"	// 19 
	};

	// Initialize default parameters
	unsigned int sceneNum = 19;
	bool multiThreaded = true;

	SETTINGS settings;
	settings.drawMode = DM_PATH_TRACE;
	settings.toneMap = TM_LINEAR;
	settings.filter = FT_BOX;

	settings.canHitLight = true;
	settings.TileBasedAdaptiveSampling = false;
	settings.initSPP = 10;
	settings.totalSPP = 8192;

	settings.numThreads = 10;
	settings.maxBounces = 5;

	std::string sceneName = "scenes/" + scenes[sceneNum];
	std::string filename = "GI.hdr";

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

	std::cout << "\n\n\n\n";
	bool running = true;

	while (running)
	{
		canvas.checkInput(); // Check for input

		// Check if the user wants to quit
		if (canvas.isQuitRequested() || canvas.keyPressed(VK_ESCAPE))
		{
			running = false;
			continue;
		}

		canvas.clear();

		// Update camera and check if it has changed (reset if it has)
		if (viewcamera.update(canvas))
		{
			rt.clear();
			totalTime = 0;
		}

		// Time how long a render call takes
		timer.reset();

		if (multiThreaded)
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
		std::cout << "Total time : " << formatTime(totalTime) << "                     \n";

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
		if (settings.totalSPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			running = false;
		}
		canvas.present();
	}

	delete scene;

	return 0;
}