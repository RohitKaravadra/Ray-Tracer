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

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
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

	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
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
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
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
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
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

	static int binarySearch(const std::vector<float>& cdf, float value) {

		const int n = static_cast<int>(cdf.size());

		if (value <= cdf[0]) return 0;
		if (value >= cdf[n - 1]) return n - 1;

		int left = 0, right = static_cast<int>(cdf.size()) - 1;

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

	float getPdf(unsigned int row, unsigned int col)
	{
		// Marginal PDF (row) and conditional PDF (column)
		float mPDF = (row == 0) ? cdfRows[row] : (cdfRows[row] - cdfRows[row - 1]);
		float cPDF = (col == 0) ? cdfCols[row][col] : (cdfCols[row][col] - cdfCols[row][col - 1]);

		// Normalize by pixel count
		float pdf = (mPDF * cPDF) * (width * height);
		return pdf < EPSILON ? EPSILON : pdf;
	}

	float getPdf(float u, float v)
	{
		unsigned int row = v * (height - 1);
		unsigned int col = u * (width - 1);

		return getPdf(row, col);
	}


	Vec3 sample(Sampler* sampler, float& u, float& v, float& pdf)
	{
		unsigned int row = binarySearch(cdfRows, sampler->next());		// calculate row
		unsigned int col = binarySearch(cdfCols[row], sampler->next());	// calculate column

		// calculate pdf
		pdf = getPdf(row, col);

		// calculate uv
		u = (float)col * invWidth;
		v = (float)row * invHeight;

		// Convert (u, v) to spherical coordinates
		float phi = u * 2.0f * M_PI;
		float theta = acosf(1.0f - 2.0f * v);

		float sinTheta = sinf(theta);

		// Convert (theta, phi) to Cartesian direction
		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = sinTheta * sinf(phi);
		wi.z = cosf(theta);

		return wi;
	}

	Vec3 sample(Sampler* sampler)
	{
		int row = binarySearch(cdfRows, sampler->next());		// calculate row
		int col = binarySearch(cdfCols[row], sampler->next());	// calculate column

		// Convert (u, v) to spherical coordinates
		float phi = col * invWidth * 2.0f * M_PI;
		float theta = acosf(1.0f - 2.0f * row * invHeight);

		float sinTheta = sinf(theta);

		// Convert (theta, phi) to Cartesian direction
		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = sinTheta * sinf(phi);
		wi.z = cosf(theta);

		return wi;
	}

	float getLum(float u, float v)
	{
		int row = v * (height - 1);
		int col = u * (width - 1);
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

	Vec3 sampleSpherical(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;
	}

	Vec3 sampleTabulated(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float u, v;
		Vec3 wi = tabDist.sample(sampler, u, v, pdf);
		reflectedColour = evaluate(wi);
		return wi;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		return useTabulated ? sampleTabulated(shadingData, sampler, reflectedColour, pdf) :
			sampleSpherical(shadingData, sampler, reflectedColour, pdf);
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

	bool isArea()
	{
		return false;
	}

	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}

	float totalIntegratedPower()
	{
		return tabDist.avgLum * 4.0f * M_PI;

		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}

	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));

		return p;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// sample from uniform sphere
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		return wi;
	}
};