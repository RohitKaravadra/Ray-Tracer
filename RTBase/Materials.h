#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		float eta = (cosTheta > 0.0f) ? (iorExt / iorInt) : (iorInt / iorExt);

		// Calculate sin2ThetaI (refraction angle)
		float sin2ThetaT = eta * eta * (1.0f - cosTheta * cosTheta);

		// Total Internal Reflection check
		if (sin2ThetaT > 1.0f)
			return 1.0f; // Total internal reflection

		// Calculate cosThetaT (refracted angle)
		float cosThetaT = std::sqrtf(1.0f - sin2ThetaT);

		// Compute the parallel and perpendicular Fresnel reflection coefficients
		float rParallel = (cosTheta - eta * cosThetaT) / (cosTheta + eta * cosThetaT);
		float rPerpendicular = (eta * cosTheta - cosThetaT) / (eta * cosTheta + cosThetaT);

		// Return the averaged Fresnel term
		return 0.5f * (rParallel * rParallel + rPerpendicular * rPerpendicular);
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Calculate sin2ThetaI (refraction angle)
		float sin2ThetaI = 1.0f - cosTheta * cosTheta;

		// Total Internal Reflection check
		if (sin2ThetaI > 1.0f)
			return Colour(1.0f, 1.0f, 1.0f); // Total internal reflection

		float eta = ior.Lum();
		// Calculate cosThetaT (refracted angle)
		float cosThetaT = std::sqrtf(1.0f - eta * eta * sin2ThetaI);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		return 1.0f;
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 localWi = shadingData.frame.toLocal(wi);
		return localWi.z / M_PI;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		// reflected direction
		Vec3 wr(-localWo.x, -localWo.y, localWo.z);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;
		pdf = 1.0f;

		return shadingData.frame.toWorld(wr);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / shadingData.frame.toLocal(wi).z;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		// sample albedo
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		float cosThetaI = localWo.z;

		// Compute Fresnel reflection coefficient
		float F = ShadingHelper::fresnelDielectric(cosThetaI, intIOR, extIOR);
		float eta = cosThetaI > 0.0f ? (extIOR / intIOR) : (intIOR / extIOR);
		// Compute sin square theta using Snell's Law
		float sin2ThetaT = eta * eta * (1.0f - cosThetaI * cosThetaI);

		Vec3 wi;
		if (sin2ThetaT > 1.0f - EPSILON || sampler->next() < F) // Reflect
		{
			wi = Vec3(-localWo.x, -localWo.y, localWo.z);
			pdf = F;
			reflectedColour = reflectedColour * pdf;
		}
		else // Refract
		{
			float cosThetaT = sqrtf(1.0f - sin2ThetaT);
			wi = localWo * eta;
			wi.z += (eta * cosThetaI - cosThetaT) * (localWo.z > 0.0f ? -1.0f : 1.0f);
			pdf = 1.0f - F;
			reflectedColour = reflectedColour * pdf * eta * eta;
		}

		return shadingData.frame.toWorld(wi.normalize());
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		float cosThetaI = shadingData.frame.toLocal(wi).z;
		return ShadingHelper::fresnelDielectric(cosThetaI, intIOR, extIOR);
	}

	bool isPureSpecular()
	{
		return true;
	}

	bool isTwoSided()
	{
		return false;
	}

	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// calculate phong exponential
		float e = alphaToPhongExponent();

		// sample theta and calculate cos theta
		float theta = acosf(powf(sampler->next(), 1.0f / (e + 1.0f)));
		// sample phy
		float phi = 2 * M_PI * sampler->next();

		float sinTheta = sinf(theta);
		Vec3 wl(sinTheta * cosf(phi), sinTheta * sinf(phi), cosf(theta));

		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr(-localWo.x, -localWo.y, localWo.z);

		// Construct a local frame aligned with reflection direction
		Frame frame;
		frame.fromVector(wr);

		Vec3 wi = frame.toWorld(wl);

		pdf = powf(std::max(0.0f, Dot(wr, wi)), e) * (e + 0.1f) / (2 * M_PI);
		float specular = powf(std::max(0.0f, Dot(wr, wi)), e) * (e + 0.2f) / (2 * M_PI);
		float F = ShadingHelper::fresnelDielectric(localWo.z, intIOR, extIOR);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		reflectedColour = reflectedColour * ((1.0f - F) + F * specular);

		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Vec3 wr(-localWo.x, -localWo.y, localWo.z);
		float e = alphaToPhongExponent();

		float specular = powf(std::max(0.0f, Dot(wr, localWi)), e) * (e + 0.2f) / (2 * M_PI);
		Colour reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		float F = ShadingHelper::fresnelDielectric(localWo.z, intIOR, extIOR);
		return reflectedColour * ((1.0f - F) + F * specular);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Vec3 wr(-localWo.x, -localWo.y, localWo.z);
		float e = alphaToPhongExponent();

		return powf(std::max(0.0f, Dot(wr, localWi)), e) * (e + 0.1f) / (2 * M_PI);
	}

	bool isPureSpecular()
	{
		return false;
	}

	bool isTwoSided()
	{
		return true;
	}

	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};