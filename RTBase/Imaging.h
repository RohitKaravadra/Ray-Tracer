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
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
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
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
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
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
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
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
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
		Colour s[4];
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
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
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
		r = powf(std::max(r, 0.0f), inv2p2) * 255;
		g = powf(std::max(g, 0.0f), inv2p2) * 255;
		b = powf(std::max(b, 0.0f), inv2p2) * 255;
	}

	// Linear tonemap with exposure
	void linerWithExposure(float& r, float& g, float& b, float exposure = 1.0f)
	{
		const float e = std::pow(2.0f, exposure * inv2p2);
		r = powf(std::max(r, 0.0f), inv2p2) * e * 255;
		g = powf(std::max(g, 0.0f), inv2p2) * e * 255;
		b = powf(std::max(b, 0.0f), inv2p2) * e * 255;
	}

	// Reinhard global tonemap
	void ReinhardGlobal(float& r, float& g, float& b)
	{
		r = powf(std::max(r / (1.0f + r), 0.0f), inv2p2) * 255;
		g = powf(std::max(g / (1.0f + g), 0.0f), inv2p2) * 255;
		b = powf(std::max(b / (1.0f + b), 0.0f), inv2p2) * 255;
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
	Colour* film;
	unsigned int width;
	unsigned int height;
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

	void splat(const float x, const float y, const Colour& L)
	{
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations 
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
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

		r = std::min(fr, 255.f);
		g = std::min(fg, 255.f);
		b = std::min(fb, 255.f);
	}

	// Tonemap the pixel
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, int spp, TONEMAP toneMap = TM_LINEAR)
	{
		Colour pixel = film[(y * width) + x] / (float)spp;

		float fr = std::max(pixel.r, 0.0f);
		float fg = std::max(pixel.g, 0.0f);
		float fb = std::max(pixel.b, 0.0f);

		tonemap(fr, fg, fb, r, g, b, toneMap);
	}

	// Get the luminance of a pixels from the film with the given coordinates
	std::vector<float> getLums(unsigned int startx, unsigned int starty, unsigned int endx, unsigned int endy)
	{
		std::vector<float> lums;

		for (unsigned int x = startx; x < endx; x++)
			for (unsigned int y = starty; y < endy; y++)
				lums.emplace_back(film[y * width + x].Lum());

		return lums;
	}

	// Do not change any code below this line
	void init(int _width, int _height, IMAGE_FILTER _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		setFilter(_filter);
	}

	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}

	void incrementSPP()
	{
		SPP++;
	}

	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};