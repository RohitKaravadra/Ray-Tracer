# Ray Tracer

## About

This project is a CPU-based physically-based ray tracer built from scratch, implementing several global illumination techniques to achieve realistic image synthesis. It features:

- Path Tracing with Monte Carlo integration
- Light Tracing (reverse path tracing)
- Instant Radiosity with Virtual Point Lights (VPLs)
- Support for a variety of materials: Diffuse, Mirror, Glass, Glossy, and Plastic
- Environment Lighting using a tabulated distribution and Multiple Importance Sampling (MIS)

Optimizations for performance and quality:
- BVH (Bounding Volume Hierarchy) for fast ray-scene intersections
- Multi-threaded tile-based rendering
- Tile-based adaptive sampling for efficient resource usage
- Intel Open Image Denoiser for post-process noise reduction

The ray tracer is entirely CPU-based, focusing on clean implementation, extensibility, and high-quality offline rendering. 
It serves as both a learning project and a foundation for further experimentation in physically-based rendering.

![Image1](Readme/Image1.png)
![Image2](Readme/Image2.png)
