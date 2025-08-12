

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include "SceneManager.h"

// create settings
SETTINGS createSettings()
{
	SETTINGS settings;

	settings.render = false;

	settings.drawMode = DM_ALBEDO;
	settings.algorithm = AL_PATH_TRACE;
	settings.toneMap = TM_REINHARD_GLOBAL;
	settings.filter = FT_BOX;

	settings.debug = false;
	settings.denoise = true;

	settings.useMultithreading = true;
	settings.useMis = true;

	settings.adaptiveSampling = true;
	settings.initSPP = 15;
	settings.totalSPP = 500;

	settings.numThreads = 20;
	settings.maxBounces = 5;
	settings.vplRaysPerTile = 1;

	return settings;
}

int main()
{
	SceneManager sceneManager;
	sceneManager.load(SCENES::BATHROOM);
	SETTINGS settings = createSettings();

	// Create canvas
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)sceneManager.curScene->camera.width, (unsigned int)sceneManager.curScene->camera.height, "Tracer", 1.0f);

	// Create ray tracer
	RayTracer rt;
	rt.init(sceneManager.curScene, &canvas, settings);

	// Create timer
	GamesEngineeringBase::Timer timer;
	float totalTime = 0;
	float lastInputTime = 0;
	float renderStartTime = 0;
	float renderTime = 0;
	float deltaTime = 0;

	bool running = true;
	bool completed = false;	// flag to check if the render is completed
	AOV aov;

	// Reset function to reset the render start time and completed flag
	auto reset = [&]()
		{
			renderStartTime = totalTime; // reset render start time
			if (completed)
			{
				std::cout << "\n\n\n\n\n";
				completed = false;	// reset completed flag
			}
		};

	std::cout << "\n\n\n\n\n";
	while (running)
	{
		deltaTime = timer.dt();
		totalTime += deltaTime;

		canvas.checkInput(); // Check for input

		// Check if the user wants to quit
		if (canvas.isQuitRequested() || canvas.keyPressed(VK_ESCAPE))
		{
			running = false;
			continue;
		}

		// Update camera and check if it has changed (reset if it has)
		if (!rt.settings.render && sceneManager.viewcamera->update(canvas))
		{
			rt.clear();
			reset();
		}

		if (totalTime - lastInputTime > 0.1f)
		{
			bool inputChanged = true;

			if (canvas.keyPressed('R'))
				rt.toggleRender();
			else if (canvas.keyPressed(VK_SPACE))
				rt.cycleDrawMode();
			else if (canvas.keyPressed(VK_TAB))
				rt.cycleAlgorithm();
			else
				inputChanged = false;

			if (inputChanged)
			{
				reset();
				lastInputTime = totalTime; // reset timer for last input
			}
		}

		canvas.clear();

		if (!completed)
		{
			if (settings.useMultithreading)
				rt.renderMT();
			else
				rt.render();

			float finalTime = completed ? renderTime : totalTime - renderStartTime;
			float progress = rt.getSPP() * 100 / settings.totalSPP;

			// Write stats to console
			std::cout << "\033[F\033[F\033[F\033[F\033[F";
			std::cout << "Progress   : " << progress << "%                                                \n";
			std::cout << "Samples    : " << rt.getSPP() << "                                              \n";
			std::cout << "Time       : " << deltaTime << "                                                \n";
			std::cout << "FPS        : " << (deltaTime > 0 ? 1.0f / deltaTime : FLT_MAX) << "             \n";
			std::cout << "Total time : " << std::roundf(totalTime) << " sec                               \n";

			if (settings.totalSPP <= rt.getSPP() && settings.render)
			{
				completed = true;

				// denoising
				//rt.saveHDR(filename);
				if (settings.denoise)
				{
					rt.createAOV(aov);
					Denoiser denoiser(aov.width, aov.height);
					denoiser.denoise(aov);
				}
			}

			rt.draw();
		}
		else
		{
			// draw the image
			if (settings.denoise)
				rt.draw(aov);
			else
				rt.draw();
		}

		//if (canvas.keyPressed('P'))
		//{
		//	rt.savePNG(filename);
		//}
		//
		//if (canvas.keyPressed('L'))
		//{
		//	size_t pos = filename.find_last_of('.');
		//	std::string ldrFilename = filename.substr(0, pos) + ".png";
		//	rt.savePNG(ldrFilename);
		//}

		canvas.present();
	}

	return 0;
}