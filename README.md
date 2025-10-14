# 🌌 Ray Tracer  
### _A Physically-Based Global Illumination Renderer (CPU-Based)_

<p align="center">
  <img src="https://img.shields.io/badge/Language-C%2B%2B17-blue?style=for-the-badge&logo=c%2B%2B">
  <img src="https://img.shields.io/badge/Renderer-Type%3A%20Path%20Tracer-green?style=for-the-badge&logo=opengl">
  <img src="https://img.shields.io/badge/Platform-Windows%20%7C%20CPU%20Renderer-lightgrey?style=for-the-badge">
  <img src="https://img.shields.io/badge/University-Warwick%20WMG-orange?style=for-the-badge">
</p>

<p align="center">
  <img src="https://github.com/RohitKaravadra/Ray-Tracer/blob/main/Readme/Image2.png?raw=true" alt="Ray Tracer Example" width="100%" style="border-radius:10px; object-fit:cover;">
</p>

---

## 📘 Overview

This project is a **physically-based CPU ray tracer** developed for the *Advanced Graphics* coursework during my **MSc in Games Engineering** at the University of Warwick.  
It was built upon the excellent template provided by [**MSCGamesTom**](https://github.com/MSCGamesTom) and later expanded into a complete **global illumination framework**.

The focus is on:
- 🧩 **Clean architecture**
- ⚙️ **Extensibility**
- 🌈 **High-quality offline rendering**

It serves as both a **learning tool** and a **foundation for experimental physically-based rendering research**.

---

## 🚀 Features

### ✳️ Algorithms
| Algorithm | Status | Description |
|------------|---------|-------------|
| **Path Tracing** | ✅ | Core physically-based global illumination |
| **Light Tracing** | ✅ | Light-source driven sampling |
| **Instant Radiosity** | ✅ | Efficient global illumination approximation |
| **Bidirectional Path Tracing** | ⚠️ | In progress |
| **Progressive Photon Mapping** | 🔴 | Planned |
| **Metropolis Light Transport (MLT)** | 🔴 | Planned |

---

### ⚡ Optimizations
| Optimization | Status |
|---------------|---------|
| **Multithreading** | ✅ |
| **Bounding Volume Hierarchy (BVH)** | ✅ |
| **Tile-Based Adaptive Sampling** | ✅ |
| **Multiple Importance Sampling (MIS)** | ✅ |
| **Tabulated Distributions** | ✅ |
| **Intel Open Image Denoiser Integration** | ✅ |
| **GPU Implementation** | 🔴 Planned |

---

### 🧱 Material Models
| Material Type | Status | Notes |
|----------------|---------|-------|
| **Diffuse (Cosine, Oren–Nayar)** | ✅ | Standard and rough diffuse BRDFs |
| **Mirror / Perfect Specular** | ✅ | Ideal reflection |
| **Glass / Dielectric** | ✅ | Refraction and total internal reflection |
| **Glossy (Conductor)** | ✅ | Metallic highlights using complex IOR |
| **Plastic (Phong)** | ✅ | Simple layered BRDF |
| **Layered / Coated Materials** | 🔴 | Planned for future |

---

## 🧾 Instructions

Scenes are **not included** in this repository.  
You can download them separately from:  
👉 [**Ray-Tracer-Scenes**](https://github.com/RohitKaravadra/Ray-Tracer-Scenes.git)

Once downloaded:
1. Create a `scene` folder inside your **RayTracer** directory.  
2. Place the scene files inside the folder.  

You can also grab precompiled builds from the **[Releases Section](https://github.com/RohitKaravadra/Ray-Tracer/releases)**  
or run the latest development version manually via Visual Studio.

---

## ⌨️ Controls

| Key | Action |
|-----|--------|
| **TAB** | Cycle through available algorithms |
| **SPACE** | Cycle through draw modes (Albedo, Normals, Light Samples) |
| **R** | Toggle render on/off |

---

## ⚒️ Command-Line Options

| Option | Description |
|---------|-------------|
| `-scene <path>` | Path to the scene file (**required**) |
| `-spp <int>` | Samples per pixel (**default:** 100, range: 10–100000) |
| `-bounces <int>` | Max light bounces (**default:** 5, range: 2–100) |
| `-threads <int>` | Number of threads (**default:** 8, range: 1–64) |
| `-denoise <0\|1>` | Enable denoising (**1:** yes, **0:** no, default: 0) |
| `-saveRenders <0\|1>` | Save rendered images (**1:** yes, **0:** no, default: 0) |
| `-filter <0–2>` | Image filter (**0:** Box, **1:** Gaussian, **2:** Mitchell–Netravali; default: 0) |
| `-toneMap <0–4>` | Tone mapping (**0:** None, **1:** Linear, **2:** Linear+Exposure, **3:** Reinhard, **4:** Filmic; default: 3) |

---

## 🧪 Example Usage

Run the ray tracer directly from terminal:
```bash
RayTracer -scene "scenes/cornell-box" -spp 500 -bounces 10 -threads 4 -denoise 1 -saveRenders 1 -filter 1 -toneMap 4
```
## 💬 Acknowledgements

> **Based on:** Template and framework by [MSCGamesTom](https://github.com/MSCGamesTom)  
> **Developed for:** Advanced Graphics coursework — *University of Warwick (WMG)*  
> **Supervisor:** Prof. Thomas Bashford-Rogers — guidance and insights into global illumination methods

## 🎥 Renders

<p align="center">
  <img src="https://github.com/RohitKaravadra/Ray-Tracer/blob/main/Readme/Image1.png?raw=true" alt="Ray Tracer Example" width="100%" style="border-radius:10px; object-fit:cover;">
</p>
