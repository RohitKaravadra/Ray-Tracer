#pragma once
#include "OpenImageDenoise/oidn.hpp"
#include <iostream>

// ANSI color codes
#define ANSI_COLOR_RED     "\033[31m"
#define ANSI_COLOR_GREEN   "\033[32m"
#define ANSI_COLOR_YELLOW  "\033[33m"
#define ANSI_COLOR_BLUE    "\033[34m"
#define ANSI_COLOR_RESET   "\033[0m"

// denoising AOVs
// Arbitrary Output Variable
struct AOV
{
	std::vector<float> albedo;
	std::vector<float> normal;
	std::vector<float> color;
	std::vector<float> output;

	unsigned int width;
	unsigned int height;

	AOV() = default;

	AOV(unsigned int _width, unsigned int _height)
	{
		width = _width;
		height = _height;

		albedo.resize(width * height * 3, 0);
		normal.resize(width * height * 3, 0);
		color.resize(width * height * 3, 0);
		output.resize(width * height * 3, 0);
	}
};

class Denoiser {
	OIDNDevice device = nullptr;
	OIDNFilter filter = nullptr;

	OIDNBuffer colorBuf = nullptr;
	OIDNBuffer albedoBuf = nullptr;
	OIDNBuffer normalBuf = nullptr;
	OIDNBuffer outputBuf = nullptr;

	unsigned int width = 0;
	unsigned int height = 0;
	size_t dataSize = 0;
	bool initialized = false;

	void initializeBuffers() {
		colorBuf = oidnNewBuffer(device, dataSize);
		albedoBuf = oidnNewBuffer(device, dataSize);
		normalBuf = oidnNewBuffer(device, dataSize);
		outputBuf = oidnNewBuffer(device, dataSize);

		if (!colorBuf || !albedoBuf || !normalBuf || !outputBuf) {
			throw std::runtime_error("Failed to create OIDN buffers");
		}
	}

	void cleanup() {
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

		try {
			std::cout << "Initializing OIDN..." << std::endl;

			device = oidnNewDevice(OIDN_DEVICE_TYPE_DEFAULT);
			if (!device) throw std::runtime_error("Failed to create OIDN device");

			oidnCommitDevice(device);

			filter = oidnNewFilter(device, "RT");
			if (!filter) throw std::runtime_error("Failed to create OIDN filter");

			initializeBuffers();

			initialized = true;
			std::cout << ANSI_COLOR_GREEN << "OIDN initialized successfully" << ANSI_COLOR_RESET << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << ANSI_COLOR_RED << "ERROR: " << e.what() << ANSI_COLOR_RESET << std::endl;
			cleanup();
			throw;
		}
	}

	void denoise(AOV& aov)
	{
		if (!initialized) {
			std::cerr << ANSI_COLOR_RED << "ERROR: Denoiser not initialized" << ANSI_COLOR_RESET << std::endl;
			return;
		}

		try {
			std::cout << ANSI_COLOR_YELLOW << "Starting denoising..." << ANSI_COLOR_RESET << std::endl;

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

			std::cout << ANSI_COLOR_GREEN << "Denoising completed successfully" << ANSI_COLOR_RESET << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << ANSI_COLOR_RED << "ERROR: " << e.what() << ANSI_COLOR_RESET << std::endl;
			throw;
		}
	}

	~Denoiser() {
		cleanup();
	}
};


