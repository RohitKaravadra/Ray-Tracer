#pragma once
#include <string>
#include "SceneLoader.h"
#include <filesystem>

static enum SCENES
{
	CORNELL_BOX = 0,
	BATHROOM,
	BATHROOM2,
	BEDROOM,
	CLASSROOM,
	COFFEE,
	DINING_ROOM,
	GLASS_OF_WATER,
	HOUSE,
	KITCHEN,
	LIVING_ROOM,
	LIVING_ROOM_2,
	LIVING_ROOM_3,
	SIBENIK,
	STAIRCASE,
	STAIRCASE2,
	TERRAIN,
	VEACH_BIDIR,
	VEACH_MIS,
	MATERIALS_SCENE,
	CAR2,
	MATERIALBALL,
	TEAPOT_FULL,
	SPONZA
};

class SceneManager
{
	// scene names
	const std::string scenes[24] = {
									"cornell-box",		// 0
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
									"MaterialsScene",	// 19
									"car2",				// 20
									"materialball",		// 21
									"teapot-full",		// 22
									"Sponza",			// 23
	};

	void unload()
	{
		if (curScene)
		{
			delete curScene;
			curScene = nullptr;
		}
	}

public:
	Scene* curScene = nullptr;
	RTCamera* viewcamera;
	std::string currentSceneName = "";

	SceneManager() {
		viewcamera = new RTCamera();
	}

	bool load(SCENES scene, std::string foldername)
	{
		unload();
		std::string sceneName = scenes[scene];
		return load(foldername + "/" + sceneName);
	}

	bool load(std::string sceneFolderPath)
	{
		unload();

		// validate if path exists
		if (!std::filesystem::exists(sceneFolderPath))
		{
			std::cout << "Scene does not exist: " << sceneFolderPath << std::endl;
			return false;
		}

		// extract scene name from path
		currentSceneName = sceneFolderPath.substr(sceneFolderPath.find_last_of("/\\") + 1);
		std::cout << "Loading scene: " << sceneFolderPath << std::endl;
		curScene = loadScene(sceneFolderPath, *viewcamera);
		return true;
	}

	~SceneManager()
	{
		unload();
		if (viewcamera)
		{
			delete viewcamera;
			viewcamera = nullptr;
		}
	}
};