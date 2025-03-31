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
		// Add code to sample a direction from the light
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


class TabulatedDistribution
{
public:
	unsigned int width;
	unsigned int height;

	float totalLum;
	std::vector<float> luminance;
	std::vector<float> cdfRows;
	std::vector<std::vector<float>> cdfCols;

	void clear()
	{
		luminance.clear();
		cdfRows.clear();
		cdfCols.clear();
	}

	void init(Texture* txt)
	{
		clear();

		std::cout << "Creating TabulatedDistribution..." << std::endl;

		width = txt->width;
		height = txt->height;

		luminance.resize(width * height);
		cdfRows.resize(height);
		cdfCols.resize(height, std::vector<float>(width));

		// Compute luminance and row sums
		totalLum = 0.0f;
		float invhPI = 1.0f / ((float)height * M_PI);

		for (int i = 0; i < height; i++)
		{
			float st = sinf(M_PI * (float)i / (float)height);  // calculate Sin-weighting
			float rowSum = 0.0f;

			for (int n = 0; n < width; n++)
			{
				unsigned int index = (i * width) + n;
				luminance[index] = txt->texels[index].Lum() * st;
				rowSum += luminance[index];
				cdfCols[i][n] = rowSum; // Store prefix sum (not normalized yet)
			}

			cdfRows[i] = rowSum; // Store row sum
			totalLum += rowSum;
		}

		// Normalize cdfCols (convert to cumulative distribution)
		for (int i = 0; i < height; i++)
		{
			if (cdfRows[i] > 0.0f)
			{
				float invRowSum = 1.0f / cdfRows[i];

				for (int n = 0; n < width; n++)
					cdfCols[i][n] *= invRowSum;

				cdfCols[i][width - 1] = 1.0f; // Ensure the last value is exactly 1.0
			}
		}

		// Convert cdfRows into a proper cumulative distribution
		float accumulated = 0.0f;
		float invTotalLum = 1.0f / totalLum;
		for (int i = 0; i < height; i++)
		{
			accumulated += cdfRows[i] * invTotalLum;
			cdfRows[i] = accumulated;
		}
		cdfRows[height - 1] = 1.0f; // Ensure the last value is exactly 1.0

		std::cout << "TabulatedDistribution created..." << std::endl;
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

	float getPdf(float u, float v)
	{
		unsigned int row = v * (height - 1);
		unsigned int col = u * (width - 1);

		// Marginal PDF (row) and conditional PDF (column)
		float mPDF = (row == 0) ? cdfRows[row] : (cdfRows[row] - cdfRows[row - 1]);
		float cPDF = (col == 0) ? cdfCols[row][col] : (cdfCols[row][col] - cdfCols[row][col - 1]);

		// Normalize by pixel count
		float pdf = (mPDF * cPDF) * (width * height);
		return pdf < EPSILON ? EPSILON : pdf;
	}

	Vec3 sample(Sampler* sampler, float& u, float& v, float& pdf)
	{
		int row = binarySearch(cdfRows, sampler->next());		// calculate row
		int col = binarySearch(cdfCols[row], sampler->next());	// calculate column

		// calculate uv
		v = row / (float)height;
		u = col / (float)width;

		// calculate pdf
		pdf = getPdf(u, v);

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
		float phi = col * 2.0f * M_PI / (float)width;
		float theta = acosf(1.0f - 2.0f * row / (float)height);

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
		return luminance[row * width + col];
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	TabulatedDistribution tabDist;

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
		return sampleSpherical(shadingData, sampler, reflectedColour, pdf);
		//return sampleTabulated(shadingData, sampler, reflectedColour, pdf);
	}

	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		//return Colour(1.0f, 1.0f, 1.0f) * tabDist.cdfCols[int(v * (tabDist.height - 1))][int(u * (tabDist.width - 1))];
		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformHemispherePDF(wi);
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
		return tabDist.totalLum * 4.0f * M_PI / (float)(env->width * env->height);

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
		//Vec3 p = tabDist.sample(sampler) * 1000.0f;
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// sample fromtabulated distribution
		//float u, v;
		//Vec3 wi = tabDist.sample(sampler, u, v, pdf);
		//return wi;

		// sample from uniform sphere
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		return wi;
	}
};