#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"
#include "GamesEngineeringBase.h"

using GamesEngineeringBase::Window;

#pragma warning( disable : 4244)

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
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
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
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
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
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
};

class TabulatedDistribution
{
public:
	unsigned int width;
	unsigned int height;

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
		float totalLum = 0.0f;
		float invhPI = 1.0f / ((float)height * M_PI);

		for (int i = 0; i < height; i++)
		{
			float st = sinf(((float)i + 0.5f) / (float)height * M_PI);  // calculate Sin-weighting
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

	static int binarySearch(const std::vector<float>& cdf, int size, float value)
	{
		int left = 0, right = size - 1;

		while (left < right)
		{
			int mid = (left + right) / 2;

			if (cdf[mid] < value)
				left = mid + 1;
			else
				right = mid;
		}

		return left < size ? left : size - 1;
	}

	float getPdf(int row, int col)
	{
		float mPDF = (row == 0) ? cdfRows[row] : (cdfRows[row] - cdfRows[row - 1]);
		float cPDF = (col == 0) ? cdfCols[row][col] : (cdfCols[row][col] - cdfCols[row][col - 1]);
		float pdf = (mPDF * cPDF) * width * height;

		// ensure pdf is not zero
		return pdf < EPSILON ? EPSILON : pdf;
	}

	Vec3 sample(Sampler* sampler, float& u, float& v, float& pdf)
	{
		int row = binarySearch(cdfRows, height, sampler->next());		// calculate row
		int col = binarySearch(cdfCols[row], width, sampler->next());	// calculate column

		// calculate uv
		v = (row + 0.5f) / height;
		u = (col + 0.5f) / width;

		// calculate pdf
		pdf = getPdf(row, col);

		// Convert (u, v) to spherical coordinates
		float phi = u * 2.0f * M_PI;   // Longitude (φ) ∈ [0, 2π]
		float theta = v * M_PI;        // Latitude (θ) ∈ [0, π]

		// Normalize the PDF
		pdf *= sinf(theta); // Adjust for Jacobian of spherical sampling

		// Convert (θ, φ) to Cartesian direction
		Vec3 wi;
		wi.x = sinf(theta) * cosf(phi);
		wi.y = cosf(theta);
		wi.z = sinf(theta) * sinf(phi);

		return wi;
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

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi1 = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		float pdf1 = SamplingDistributions::uniformSpherePDF(wi1);
		Colour le1 = evaluate(shadingData, wi1);

		// for testing
		pdf = pdf1;
		reflectedColour = le1;
		return wi1;

		// tabulated distribution sampling
		float u, v, pdf2;
		Vec3 wi2 = tabDist.sample(sampler, u, v, pdf2);
		Colour le2 = evaluate(shadingData, wi2);

		// calculate weights
		float invPdfSum = 1 / (pdf2 + pdf1);
		float w2 = pdf2 * invPdfSum;
		float w1 = pdf1 * invPdfSum;

		// calculate pdf
		pdf = (pdf2 * w2) + (pdf1 * w1);

		// calculate reflected colour
		reflectedColour = (le2 * w2) + (le1 * w1);

		// return direction
		return (sampler->next() < 0.5f) ? wi2 : wi1;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
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
};