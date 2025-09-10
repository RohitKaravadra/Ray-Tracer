
#define NOMINMAX
#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#include "SceneManager.h"
#include "GamesEngineeringBase.h"
#include "ArgsParcer.h"

/// <summary>
/// Creates and returns a SETTINGS object initialized with default values.
/// </summary>
/// <returns>A SETTINGS object with its fields set to default configuration values.</returns>
static SETTINGS createSettings()
{
	SETTINGS settings;

	settings.saveRenders = false;
	settings.debug = false;
	settings.denoise = false;

	settings.drawMode = DM_ALBEDO;
	settings.algorithm = AL_PATH_TRACE;
	settings.toneMap = TM_REINHARD_GLOBAL;
	settings.filter = FT_BOX;

	settings.useMultithreading = true;
	settings.numThreads = 8;

	settings.totalSPP = 100;
	settings.maxBounces = 5;

	return settings;
}

/// <summary>
/// Stats class to keep track of rendering statistics and performance metrics.
/// </summary>
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
		if (completed) {
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
			std::cout << "Render completed in " << std::roundf(renderTime) << " seconds.\n";
		}
	}

	bool canInput() const { return totalTime - lastInputTime > 0.1f; }

	void print(float progress, int spp, std::string alg, bool rendering) const
	{
		// Write stats to console
		tui::restoreCursor();
		tui::clearRest();
		if (rendering)
			tui::print(tui::color::cyan(tui::format("Algorithm  : ", alg,
				" (Progress   : ", std::to_string((int)(progress * 100)), " % |",
				" Samples   : ", std::to_string(spp), " )")));
		else
			tui::print(tui::color::cyan(tui::format("Algorithm  : ", alg, " (change - TAB , render - R) ")));
	}
};

/// <summary>
/// Saves the current render from the given renderer to a PNG file in the 'Renders' directory.
/// </summary>
/// <param name="rt">Reference to the Renderer object whose output will be saved.</param>
/// <param name="filename">The name of the PNG file to save the render as.</param>
static void saveRender(Renderer& rt, const std::string& filename)
{
	const wchar_t* rendersFolder = L"Renders";
	CreateDirectory(L"Renders", NULL);

	std::string renderFolderStr = std::string(rendersFolder, rendersFolder + wcslen(rendersFolder));
	std::string filepath = renderFolderStr + "/" + filename;
	rt.savePNG(filepath);
}


/// <summary>
/// Entry point for the ray-tracing application. Initializes the scene, window, renderer, and main loop for user interaction and rendering.
/// </summary>
/// <param name="argc">The number of command-line arguments.</param>
/// <param name="argv">An array of command-line argument strings.</param>
/// <returns>Returns 0 upon successful execution.</returns>
int main(int argc, char* argv[])
{
	tui::clear();
	tui::hideCursor();

	SETTINGS settings = createSettings();
	std::string scenePath = ArgsParser::parse(argc, argv, settings);
	std::cout << settings;

	// Load scene
	SceneManager sceneManager;
	if (scenePath.empty()) sceneManager.load(SCENES::CORNELL_BOX, "scenes");
	else sceneManager.load(scenePath);

	// Create canvas
	GamesEngineeringBase::Window canvas;
	Vec2u screenSize = sceneManager.curScene->camera.size;
	canvas.create(screenSize.x, screenSize.y, "Ray-Tracer", 1.0f);

	// Create ray tracer
	Renderer rt(sceneManager.curScene, &canvas, settings);

	// Create timer
	GamesEngineeringBase::Timer timer;
	Stats stats;

	bool running = true;
	AOV aov;

	tui::saveCursor();
	stats.print(0, 0, "", false);
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
		if (!rt.isRendering() && sceneManager.viewcamera->update(canvas))
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

		if (!rt.isCompleted())
		{
			rt.render();

			float progress = 0;
			int spp = 0;
			rt.getProgress(progress, spp);
			stats.print(progress, spp,
				rt.getAlgorithm(), rt.isRendering());

			if (rt.isCompleted())
			{
				stats.onCompletion();
				rt.denoise();
			}
		}

		rt.draw();
		canvas.present();
	}

	tui::showCursor();
	tui::clear();
	return 0;
}