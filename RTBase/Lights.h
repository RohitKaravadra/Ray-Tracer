#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

struct LightSample
{
	Vec3 p;				// position of the light sample
	Vec3 n;				// normal at the light sample position

	bool isNull;		// true if the light sample is null (no light)
	bool isArea;		// true if the light sample is from an area light

	float pdf;			// probability density function value for the sample
	float pmf;			// probability mass function value for the sample

	Colour emitted;		// emission colour of the light sample
};

class Light
{
public:
	virtual Vec3 sample(Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;

	Vec3 sample(Sampler* sampler, Colour& emittedColour, float& pdf) override
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}

	bool isArea()
	{
		return true;
	}

	Vec3 normal(const Vec3& wi) override
	{
		return triangle->gNormal();
	}

	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}

	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// sample from cosine hemisphere
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(Sampler* sampler, Colour& reflectedColour, float& pdf) override
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

// TabulatedDistribution is used to store the luminance values of the environment map
class TabulatedDistribution
{
public:
	unsigned int width;
	unsigned int height;

	float invWidth;
	float invHeight;

	float totalLum;
	float avgLum;

	std::vector<float> lum;
	std::vector<float> cdfRows;					// marginal PDF
	std::vector<std::vector<float>> cdfCols;	// conditional PDF

	void clear()
	{
		lum.clear();
		cdfRows.clear();
		cdfCols.clear();
	}

	void init(Texture* txt)
	{
		clear();

		std::cout << "Creating Tabulated Distribution..." << std::endl;

		width = txt->width;
		height = txt->height;

		invWidth = 1.0f / (float)(width);
		invHeight = 1.0f / (float)(height);

		lum.resize(width * height);
		cdfRows.resize(height);
		cdfCols.resize(height, std::vector<float>(width));

		totalLum = 0.0f;
		for (unsigned int y = 0; y < height; y++)
		{
			float sinTheta = sinf(M_PI * ((float)y + 0.05f) / (float)(height - 1));
			float rowSum = 0.0f;
			for (unsigned int x = 0; x < width; x++)
			{
				unsigned int index = y * width + x;
				lum[index] = txt->texels[index].Lum() * sinTheta;	// weight the luminance by the sin of the angle
				cdfCols[y][x] = lum[index];							// cdfCols is the marginal PDF
				rowSum += lum[index];
			}
			cdfRows[y] = rowSum;	// cdfRows is the conditional PDF
			totalLum += rowSum;
		}
		avgLum = totalLum * invHeight * invWidth; // average luminance

		float accumCol = 0.0f, accumRow = 0.0f;
		for (unsigned y = 0; y < height; y++)
		{
			accumCol = 0.0f;
			for (unsigned int x = 0; x < width; x++)
			{
				accumCol += cdfCols[y][x] / cdfRows[y];
				cdfCols[y][x] = accumCol;	// normalize the marginal PDF
			}
			accumRow += cdfRows[y] / totalLum;
			cdfRows[y] = accumRow; // normalize the conditional PDF
		}

		std::cout << "Tabulated Distribution created..." << std::endl;
	}

	static int binarySearch(const std::vector<float>& cdf, float value)
	{
		const int n = static_cast<int>(cdf.size());
		if (n == 0) return 0;

		int left = 0;
		int right = n - 1;

		while (left < right)
		{
			int mid = left + (right - left) / 2;
			if (cdf[mid] < value)
				left = mid + 1;
			else
				right = mid;
		}

		return left;
	}

	float getPdf(int row, int col) const
	{
		if (row < 0 || row >= height || col < 0 || col >= width)
			return 0.0f;

		// Compute marginal PDF for row
		float marginalPdf = (row == 0) ? cdfRows[row] : (cdfRows[row] - cdfRows[row - 1]);

		// Compute conditional PDF for column
		float conditionalPdf = (col == 0) ? cdfCols[row][col] : (cdfCols[row][col] - cdfCols[row][col - 1]);

		// Combined PDF needs to account for the 2D domain size
		float pdf = marginalPdf * conditionalPdf * (width * height);

		return std::max(pdf, EPSILON);
	}

	float getPdf(float u, float v) const
	{
		// Clamp and scale to [0,1]
		u = std::max(0.0f, std::min(1.0f - 1e-6f, u));
		v = std::max(0.0f, std::min(1.0f - 1e-6f, v));

		int row = static_cast<int>(v * height);
		int col = static_cast<int>(u * width);

		return getPdf(row, col);
	}

	Vec3 sample(Sampler* sampler, float& u, float& v, float& pdf) const
	{
		// Sample row (marginal)
		float rand1 = sampler->next();
		int row = binarySearch(cdfRows, rand1);
		row = std::max(0, std::min(row, static_cast<int>(height) - 1));

		// Sample column (conditional)
		float rand2 = sampler->next();
		int col = binarySearch(cdfCols[row], rand2);
		col = std::max(0, std::min(col, static_cast<int>(width) - 1));

		// Compute PDF
		pdf = getPdf(row, col);

		// Compute UV coordinates (with jittering for better stratification)
		u = (col + sampler->next()) * invWidth;
		v = (row + sampler->next()) * invHeight;

		// Convert to spherical coordinates
		float phi = u * 2.0f * M_PI;
		float theta = v * M_PI;  // More accurate than acos(1-2v)

		// Convert to Cartesian direction
		float sinTheta = sinf(theta);
		return Vec3(
			sinTheta * cosf(phi),
			cosf(theta),
			sinTheta * sinf(phi)
		);
	}

	Vec3 sample(Sampler* sampler, float& pdf) const
	{
		float u, v;
		return sample(sampler, u, v, pdf);
	}

	Vec3 sample(Sampler* sampler) const
	{
		float pdf, u, v;
		return sample(sampler, u, v, pdf);
	}

	float getLum(float u, float v) const
	{
		u = std::max(0.0f, std::min(1.0f - 1e-6f, u));
		v = std::max(0.0f, std::min(1.0f - 1e-6f, v));

		int row = static_cast<int>(v * height);
		int col = static_cast<int>(u * width);

		if (row < 0 || row >= height || col < 0 || col >= width)
			return 0.0f;

		return lum[row * width + col];
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	TabulatedDistribution tabDist;	// tabulated distribution for importance sampling

	const bool useTabulated = true;

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		tabDist.init(env);
	}

	Vec3 sampleSpherical(Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;
	}

	Vec3 sampleTabulated(Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float u, v;
		Vec3 wi = tabDist.sample(sampler, u, v, pdf);
		reflectedColour = evaluate(wi);
		return wi;
	}

	Vec3 sample(Sampler* sampler, Colour& reflectedColour, float& pdf) override
	{
		return useTabulated ? sampleTabulated(sampler, reflectedColour, pdf) :
			sampleSpherical(sampler, reflectedColour, pdf);
	}

	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;

		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		if (!useTabulated)
			return SamplingDistributions::uniformHemispherePDF(wi);

		// sample from tabulated distribution
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;

		return tabDist.getPdf(u, v);
	}

	bool isArea() {
		return false;
	}

	Vec3 normal(const Vec3& wi) {
		return -wi;
	}

	float totalIntegratedPower() {
		return tabDist.avgLum * 4.0f * M_PI;
	}

	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = tabDist.sample(sampler, pdf);
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		return p;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// sample from uniform sphere
		Vec3 wi = tabDist.sample(sampler, pdf);
		return wi;
	}
};