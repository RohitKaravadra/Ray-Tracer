

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

	// Initialize default parameters
	std::string sceneName = "scenes/dining-room";
	std::string filename = "GI.hdr";
	unsigned int SPP = 8192;

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
				SPP = stoi(pair.second);
			}
		}
	}

	// Load scene and camera
	RTCamera viewcamera;
	Scene* scene = loadScene(sceneName, viewcamera);

	// Create canvas
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", 1.0f);

	// Create ray tracer
	RayTracer rt;
	rt.init(scene, &canvas, 10);	// 10 threads

	// Create timer
	GamesEngineeringBase::Timer timer;
	float totalTime = 0;

	std::cout << "\n\n\n\n";
	bool running = true;

	while (running)
	{
		canvas.checkInput(); // Check for input

		// Check if the user wants to quit
		if (canvas.isQuit() || canvas.keyPressed(VK_ESCAPE))
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
		rt.renderMT();
		float t = timer.dt();

		totalTime += t; // update total time

		// Write stats to console
		std::cout << "\033[F\033[F\033[F\033[F";
		std::cout << "Samples    : " << rt.getSPP() << std::endl;
		std::cout << "Time       : " << t << std::endl;
		std::cout << "FPS        : " << (t > 0 ? 1.0f / t : FLT_MAX) << std::endl;
		std::cout << "Total time : " << formatTime(totalTime) << std::endl;

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
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			running = false;
		}
		canvas.present();
	}

	delete scene;

	return 0;
}