#pragma once
#include "OpenImageDenoise/oidn.hpp"
#include <iostream>
#include "TerminalUI.h"

// denoising AOVs
// Arbitrary Output Variable
struct AOV
{
	std::vector<float> albedo;
	std::vector<float> normal;
	std::vector<float> color;
	std::vector<float> output;

	Vec2u size;

	AOV() = default;

	AOV(const Vec2u& size) : size(size)
	{
		unsigned int totalSize = size.x * size.y * 3;
		albedo.resize(totalSize, 0);
		normal.resize(totalSize, 0);
		color.resize(totalSize, 0);
		output.resize(totalSize, 0);
	}
};

// Denoiser class using OpenImageDenoise (OIDN)
class Denoiser
{
	// OIDN device and filter handles
	OIDNDevice device = nullptr;
	OIDNFilter filter = nullptr;

	// Buffers for color, albedo, normal, and output
	OIDNBuffer colorBuf = nullptr;
	OIDNBuffer albedoBuf = nullptr;
	OIDNBuffer normalBuf = nullptr;
	OIDNBuffer outputBuf = nullptr;

	// Image dimensions and data size
	unsigned int width = 0;
	unsigned int height = 0;

	size_t dataSize = 0;
	bool initialized = false;	// Flag to check if OIDN is initialized

	// Initialize OIDN buffers
	void initializeBuffers()
	{
		colorBuf = oidnNewBuffer(device, dataSize);
		albedoBuf = oidnNewBuffer(device, dataSize);
		normalBuf = oidnNewBuffer(device, dataSize);
		outputBuf = oidnNewBuffer(device, dataSize);

		if (!colorBuf || !albedoBuf || !normalBuf || !outputBuf)
		{
			throw std::runtime_error("Failed to create OIDN buffers");
		}
	}

	// Cleanup function to release OIDN resources
	void cleanup()
	{
		if (filter) oidnReleaseFilter(filter);
		if (device) oidnReleaseDevice(device);
		if (colorBuf) oidnReleaseBuffer(colorBuf);
		if (albedoBuf) oidnReleaseBuffer(albedoBuf);
		if (normalBuf) oidnReleaseBuffer(normalBuf);
		if (outputBuf) oidnReleaseBuffer(outputBuf);

		filter = nullptr;
		device = nullptr;
		colorBuf = nullptr;
		albedoBuf = nullptr;
		normalBuf = nullptr;
		outputBuf = nullptr;
		initialized = false;
	}

	// Disable copying
	Denoiser(const Denoiser&) = delete;
	Denoiser& operator=(const Denoiser&) = delete;

public:
	Denoiser(unsigned int _width, unsigned int _height)
	{
		width = _width;
		height = _height;
		dataSize = width * height * 3 * sizeof(float);

		try
		{
			std::cout << "Initializing OIDN..." << std::endl;

			// Create OIDN device
			device = oidnNewDevice(OIDN_DEVICE_TYPE_DEFAULT);
			if (!device) throw std::runtime_error("Failed to create OIDN device");

			// Set device parameters
			oidnCommitDevice(device);

			// Create OIDN filter
			filter = oidnNewFilter(device, "RT");
			if (!filter) throw std::runtime_error("Failed to create OIDN filter");

			// Set filter parameters
			initializeBuffers();

			initialized = true;
			tui::print(tui::color::green("OIDN initialized successfully"));
		}
		catch (const std::exception& e)
		{
			tui::print(tui::color::red(tui::format("ERROR: ", e.what())));
			cleanup();
			throw;
		}
	}

	void denoise(AOV& aov)
	{
		// Check if OIDN is initialized
		if (!initialized)
		{
			tui::print(tui::color::red("ERROR: Denoiser not initialized"));
			return;
		}

		try
		{
			tui::print(tui::color::yellow("Starting denoising..."));

			// Copy data to buffers
			oidnWriteBuffer(colorBuf, 0, dataSize, &aov.color[0]);
			oidnWriteBuffer(albedoBuf, 0, dataSize, &aov.albedo[0]);
			oidnWriteBuffer(normalBuf, 0, dataSize, &aov.normal[0]);

			// Set filter images (using non-shared version for safety)
			oidnSetFilterImage(filter, "color", colorBuf, OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0);
			oidnSetFilterImage(filter, "albedo", albedoBuf, OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0);
			oidnSetFilterImage(filter, "normal", normalBuf, OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0);
			oidnSetFilterImage(filter, "output", outputBuf, OIDN_FORMAT_FLOAT3, width, height, 0, 0, 0);

			// Set filter parameters
			oidnSetFilterBool(filter, "hdr", true);
			oidnSetFilterBool(filter, "cleanAux", true);

			// Execute denoising
			oidnCommitFilter(filter);
			oidnExecuteFilter(filter);

			// Check for errors
			const char* errorMessage = nullptr;
			if (oidnGetDeviceError(device, &errorMessage) != OIDN_ERROR_NONE) {
				throw std::runtime_error(errorMessage ? errorMessage : "Unknown denoising error");
			}

			// Copy results back
			oidnReadBuffer(outputBuf, 0, dataSize, &aov.output[0]);

			tui::print(tui::color::green("Denoising completed successfully"));
		}
		catch (const std::exception& e) {
			tui::print(tui::color::red(tui::format("ERROR: ", e.what())));
			throw;
		}
	}

	~Denoiser()
	{
		cleanup();
	}
};


