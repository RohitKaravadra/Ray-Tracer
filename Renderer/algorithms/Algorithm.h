#pragma once
#include "../RendererData.h"

// base class for all rendering algorithms
class AlgorithmBase
{
protected:
	RENDERER_DATA& data;

	AlgorithmBase(RENDERER_DATA& data) : data(data) {
	}
public:
	virtual void render() = 0;
};