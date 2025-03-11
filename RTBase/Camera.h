#pragma once
#include "Core.h"
#include "Renderer.h"
#include "GamesEngineeringBase.h"

using GamesEngineeringBase::Window;

class NewCamera
{
	Vec3 from;
	Vec3 to;
	Vec3 up;

	Camera* camera = NULL;

	float moveSpeed = 1.0f;
	float rotSpeed = 5.0f;

	NewCamera()
	{
		rotSpeed = 5.0f;
	}

	void updateMatrix()
	{
		Matrix V = Matrix::lookAt(from, to, up);
		V = V.invert();
		camera->updateView(V);
	}

	void update(Window* canvas)
	{
		int movex = 0, movey = 0, movez = 0, rotx = 0, roty = 0;

		if (canvas->keyPressed('D'))
			movex++;
		if (canvas->keyPressed('A'))
			movex--;
		if (canvas->keyPressed('W'))
			movey++;
		if (canvas->keyPressed('S'))
			movey--;
		if (canvas->keyPressed('E'))
			movez++;
		if (canvas->keyPressed('Q'))
			movez--;
	}
};
