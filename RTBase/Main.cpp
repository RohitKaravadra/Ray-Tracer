

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

	settings.algorithm = AL_LIGHT_TRACE;
	settings.drawMode = DM_ALGORITHM;
	settings.toneMap = TM_LINEAR;
	settings.filter = FT_BOX;

	settings.canHitLight = true;
	settings.debug = false;
	settings.denoise = true;

	settings.useMultithreading = true;
	settings.useMis = true;

	settings.adaptiveSampling = true;
	settings.initSPP = 10;
	settings.totalSPP = 200;

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

	bool running = true;
	bool completed = false;	// to check if the render is completed
	AOV aov;

	std::cout << "\n\n\n\n\n";
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
		if (sceneManager.viewcamera->update(canvas))
		{
			rt.clear();
			totalTime = 0;
			if (completed)
			{
				std::cout << "\n\n\n\n\n";
				completed = false;	// reset completed flag
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

			float progress = rt.getSPP() * 100 / settings.totalSPP;

			// Write stats to console
			std::cout << "\033[F\033[F\033[F\033[F\033[F";
			std::cout << "Progress   : " << progress << "%              \n";
			std::cout << "Samples    : " << rt.getSPP() << "            \n";
			std::cout << "Time       : " << t << "                      \n";
			std::cout << "FPS        : " << (t > 0 ? 1.0f / t : FLT_MAX) << "              \n";
			std::cout << "Total time : " << std::roundf(totalTime) << " sec                \n";
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

		if (!completed && settings.totalSPP <= rt.getSPP() && settings.drawMode == DM_ALGORITHM)
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

		// draw the image
		if (completed && settings.denoise)
			rt.draw(aov);
		else
			rt.draw();

		canvas.present();
	}

	return 0;
}