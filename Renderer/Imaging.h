#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

// Constants for Filmic tonemap
enum IMAGE_FILTER
{
	FT_BOX,
	FT_GAUSSIAN,
	FT_MITCHELL_NETRAVALI
};

class Texture
{
public:
	Color* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Color[1];
		texels[0] = Color(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Color[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Color(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Color[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Color(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Color sample(const float tu, const float tv) const
	{
		Color tex;
		float u = max(0.0f, fabsf(tu)) * width;
		float v = max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Color s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = max(0.0f, fabsf(tu)) * width;
		float v = max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 1.0f && fabsf(y) <= 1.0f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter
{
	static constexpr int radii = 2;
	static constexpr float alpha = 2.5f;
	const float t2 = std::exp(-alpha * (radii * radii));

public:
	float Gaussian(float d) const
	{
		return std::exp(-alpha * (d * d)) - t2;
	}

	float filter(float x, float y) const
	{
		return Gaussian(x) * Gaussian(y);
	}

	int size() const
	{
		return radii;
	}
};

class MitchellNetravaliFilter : public ImageFilter
{
	const float B = 1.0f / 3.0f;
	const float C = 1.0f / 3.0f;

	const float a1 = (1.0f / 6.0f) * (12 - 9 * B - 6 * C);
	const float a2 = (-18 + 12 * B + 6 * C);
	const float a3 = (6 - 2 * B);

	const float b1 = (1.0f / 6.0f) * (-B - 6 * C);
	const float b2 = (6 * B + 30 * C);
	const float b3 = (-12 * B - 48 * C);
	const float b4 = (8 * B + 24 * C);

public:
	float MitchellNetravali(float d) const
	{
		d = fabs(d);
		if (d >= 2) return 0;

		float ds = d * d, dc = ds * d;
		if (d >= 0 && d < 1)
			return a1 * dc + a2 * ds + a3;
		else
			return b1 * dc + b2 * ds + b3 * d + b4;
	}

	float filter(float x, float y) const
	{
		return MitchellNetravali(x) * MitchellNetravali(y);
	}

	int size() const
	{
		return 2;
	}
};

// Tonemapping functions
enum TONEMAP
{
	TM_NONE,
	TM_LINEAR,
	TM_LINEAR_EXPOSURE,
	TM_REINHARD_GLOBAL,
	TM_FILMIC
};

class Film
{
	void none(float& r, float& g, float& b)
	{
		r *= 255;
		g *= 255;
		b *= 255;
	}

	// Linear tonemap
	void liner(float& r, float& g, float& b)
	{
		r = powf(max(r, 0.0f), inv2p2) * 255;
		g = powf(max(g, 0.0f), inv2p2) * 255;
		b = powf(max(b, 0.0f), inv2p2) * 255;
	}

	// Linear tonemap with exposure
	void linerWithExposure(float& r, float& g, float& b, float exposure = 1.0f)
	{
		const float e = std::pow(2.0f, exposure * inv2p2);
		r = powf(max(r, 0.0f), inv2p2) * e * 255;
		g = powf(max(g, 0.0f), inv2p2) * e * 255;
		b = powf(max(b, 0.0f), inv2p2) * e * 255;
	}

	// Reinhard global tonemap
	void ReinhardGlobal(float& r, float& g, float& b)
	{
		r = powf(max(r / (1.0f + r), 0.0f), inv2p2) * 255;
		g = powf(max(g / (1.0f + g), 0.0f), inv2p2) * 255;
		b = powf(max(b / (1.0f + b), 0.0f), inv2p2) * 255;
	}

	float CX(float x) const
	{
		return std::fabs((x * (A * x + CB) + DE) / (x * (A * x + B) + DF) - EbF);
	}

	// Filmic tonemap
	void filmic(float& r, float& g, float& b)
	{
		r = CX(r) * invCW * 255.0f;
		g = CX(g) * invCW * 255.0f;
		b = CX(b) * invCW * 255.0f;
	}

	// Set the filter to be used
	void setFilter(IMAGE_FILTER _filter)
	{
		if (filter != nullptr)
			delete filter;

		switch (_filter)
		{
		case FT_BOX:filter = new BoxFilter();
			break;
		case FT_GAUSSIAN:filter = new GaussianFilter();
			break;
		case FT_MITCHELL_NETRAVALI:filter = new MitchellNetravaliFilter();
			break;
		}
	}

public:
	Color* film;
	Vec2u size;
	int SPP;
	ImageFilter* filter;

	// for Filmic tonemap
	const float A = 0.15f, B = 0.5f, C = 0.1f, D = 0.2f, E = 0.02f, F = 0.3f, W = 11.2f;
	const float CB = C * B, DE = D * E, DF = D * F, EbF = E / F;
	const float invCW = 1.0f / ((W * (A * W + CB) + DE) / (W * (A * W + B) + DF) - EbF);

	const float inv2p2 = 1.0f / 2.2f;

	~Film()
	{
		delete[] film;
		delete filter;
	}

	void splat(const Vec2& p, const Color& L)
	{
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations 
		unsigned int used = 0;
		float total = 0;
		int _size = filter->size();
		for (int i = -_size; i <= _size; i++)
		{
			for (int j = -_size; j <= _size; j++)
			{
				Vec2i sp = Vec2i(p.x + j, p.y + i);
				if (sp >= 0 && sp < size)
				{
					indices[used] = (sp.y * size.x) + sp.x;
					filterWeights[used] = filter->filter(sp.x - p.x, sp.y - p.y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}

	// Tonemap the pixel
	void tonemap(float fr, float fg, float fb, unsigned char& r, unsigned char& g, unsigned char& b, TONEMAP toneMap = TM_LINEAR)
	{
		switch (toneMap)
		{
		case TM_NONE:none(fr, fg, fb);
			break;
		case TM_LINEAR:liner(fr, fg, fb);
			break;
		case TM_LINEAR_EXPOSURE:linerWithExposure(fr, fg, fb);
			break;
		case TM_REINHARD_GLOBAL:ReinhardGlobal(fr, fg, fb);
			break;
		case TM_FILMIC:filmic(fr, fg, fb);
		}

		r = min(fr, 255.f);
		g = min(fg, 255.f);
		b = min(fb, 255.f);
	}

	// Tonemap the pixel
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, int spp, TONEMAP toneMap = TM_LINEAR)
	{
		Color pixel = film[(y * size.x) + x] / (float)spp;

		float fr = max(pixel.r, 0.0f);
		float fg = max(pixel.g, 0.0f);
		float fb = max(pixel.b, 0.0f);

		tonemap(fr, fg, fb, r, g, b, toneMap);
	}

	// Get the luminance of a pixels from the film with the given coordinates
	std::vector<float> getLums(const Vec2i& start, const Vec2i& end)
	{
		std::vector<float> lums;

		for (unsigned int x = start.x; x < end.x; x++)
			for (unsigned int y = start.y; y < end.y; y++)
				lums.emplace_back(film[y * size.x + x].Lum());

		return lums;
	}

	// Do not change any code below this line
	void init(Vec2u _size, IMAGE_FILTER _filter)
	{
		size = _size;
		film = new Color[size.x * size.y];
		clear();
		setFilter(_filter);
	}

	void clear()
	{
		memset(film, 0, size.x * size.y * sizeof(Color));
		SPP = 0;
	}

	void incrementSPP()
	{
		SPP++;
	}

	void save(std::string filename)
	{
		Color* hdrpixels = new Color[size.x * size.y];
		for (unsigned int i = 0; i < (size.x * size.y); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), size.x, size.y, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};