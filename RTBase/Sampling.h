#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
	float epsilon = 1e-7f;
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;

	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		float r = dist(generator);
		return r < epsilon ? epsilon : r;
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(r1);
		float phy = M_PI * 2 * r2;
		float st = sinf(theta);
		return Vec3(st * cosf(phy), st * sinf(phy), r1);
	}

	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return 1.0f / (2 * M_PI);
	}

	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		r1 = sqrtf(r1);
		float theta = acosf(r1);
		float phy = M_PI * 2 * r2;
		float st = sinf(theta);
		return Vec3(st * cosf(phy), st * sinf(phy), r1);
	}

	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return wi.z / M_PI;
	}

	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		r1 = 1 - 2 * r1;
		float theta = acosf(r1);
		float phy = M_PI * 2 * r2;
		float st = sinf(theta);
		return Vec3(st * cosf(phy), st * sinf(phy), r1);
	}

	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / (4 * M_PI);
	}
};