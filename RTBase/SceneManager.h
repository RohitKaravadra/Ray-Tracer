#pragma once
#include <string>
#include "SceneLoader.h"

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
	const std::string scenes[24] = { "scenes/cornell-box",		// 0
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

public:

	Scene* curScene = nullptr;
	RTCamera* viewcamera;

	SceneManager()
	{
		viewcamera = new RTCamera();
	}

	void unload()
	{
		if (curScene)
		{
			delete curScene;
			curScene = nullptr;
		}
	}

	void load(SCENES scene)
	{
		unload();
		std::string sceneName = scenes[scene];
		std::cout << "Loading scene: " << sceneName << std::endl;
		curScene = loadScene(sceneName, *viewcamera);
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