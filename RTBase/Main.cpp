

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include "SceneManager.h"

// create settings
static SETTINGS createSettings()
{
	SETTINGS settings;

	settings.render = false;
	settings.saveRenders = false;

	settings.drawMode = DM_ALBEDO;
	settings.algorithm = AL_PATH_TRACE;
	settings.toneMap = TM_REINHARD_GLOBAL;
	settings.filter = FT_BOX;

	settings.debug = false;
	settings.denoise = true;

	settings.useMultithreading = true;
	settings.useMis = true;

	settings.adaptiveSampling = true;
	settings.initSPP = 50;
	settings.totalSPP = 600;

	settings.numThreads = 8;
	settings.maxBounces = 6;
	settings.vplRaysPerTile = 1;

	return settings;
}

class Stats
{
	float totalTime;
	float lastInputTime;
	float renderStartTime;
	float renderTime;
	float deltaTime;

	bool completed;	// flag to check if the render is completed
public:

	Stats()
	{
		totalTime = 0;
		lastInputTime = 0;
		renderStartTime = 0;
		renderTime = 0;
		deltaTime = 0;
		completed = false;
	}

	void reset()
	{
		renderStartTime = totalTime; // reset render start time
		if (completed)
		{
			std::cout << "\n\n\n\n\n";
			completed = false;	// reset completed flag
		}
	}

	void update(float dt)
	{
		deltaTime = dt;
		totalTime += deltaTime;
	}

	void updateLastInputTime() {
		lastInputTime = totalTime; // reset timer for last input
	}

	void onCompletion()
	{
		if (!completed)
		{
			completed = true;
			renderTime = totalTime - renderStartTime;
		}
	}

	bool canInput() const { return totalTime - lastInputTime > 0.1f; }

	bool isCompleted() const { return completed; }

	void print(Renderer& rt) const
	{
		float finalTime = completed ? renderTime : totalTime - renderStartTime;
		float progress = rt.settings.render ? rt.getSPP() * 100 / rt.settings.totalSPP : 0;

		// Write stats to console
		std::cout << "\033[F\033[F\033[F\033[F\033[F";
		std::cout << "Progress   : " << progress << "%                                                \n";
		std::cout << "Samples    : " << rt.getSPP() << "                                              \n";
		std::cout << "Time       : " << deltaTime << "                                                \n";
		std::cout << "FPS        : " << (deltaTime > 0 ? 1.0f / deltaTime : FLT_MAX) << "             \n";
		std::cout << "Total time : " << std::roundf(totalTime) << " sec                               \n";
	}
};

static void saveRender(Renderer& rt, const std::string& filename)
{
	if (!rt.settings.saveRenders)
		return;

	const wchar_t* rendersFolder = L"Renders";
	CreateDirectory(L"Renders", NULL);

	std::string renderFolderStr = std::string(rendersFolder, rendersFolder + wcslen(rendersFolder));
	std::string filepath = renderFolderStr + "/" + filename;
	rt.savePNG(filepath);
}

int main()
{
	SceneManager sceneManager;
	sceneManager.load(SCENES::CORNELL_BOX, "scenes");

	// Create canvas
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)sceneManager.curScene->camera.width, (unsigned int)sceneManager.curScene->camera.height, "Tracer", 1.0f);

	// Create ray tracer
	Renderer rt;
	rt.init(sceneManager.curScene, &canvas, createSettings());


	// Create timer
	GamesEngineeringBase::Timer timer;
	Stats stats;

	bool running = true;
	AOV aov;

	std::cout << "\n\n\n\n\n";
	while (running)
	{
		stats.update(timer.dt());

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
			stats.reset();
		}

		if (stats.canInput())
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
				stats.reset();
				stats.updateLastInputTime();
			}
		}

		canvas.clear();

		if (!stats.isCompleted())
		{
			if (rt.settings.useMultithreading)
				rt.renderMT();
			else
				rt.render();

			stats.print(rt);

			if (rt.settings.totalSPP <= rt.getSPP() && rt.settings.render)
			{
				stats.onCompletion();

				rt.draw();
				saveRender(rt, sceneManager.currentSceneName + "_render.png");

				// denoising
				if (rt.settings.denoise)
				{
					rt.createAOV(aov);
					Denoiser denoiser(aov.width, aov.height);
					denoiser.denoise(aov);
				}

				rt.draw(aov);
				saveRender(rt, sceneManager.currentSceneName + "_render_denoised.png");
			}
			else
				rt.draw();
		}
		else
		{
			// draw the image
			if (rt.settings.denoise)
				rt.draw(aov);
			else
				rt.draw();
		}

		canvas.present();
	}

	return 0;
}