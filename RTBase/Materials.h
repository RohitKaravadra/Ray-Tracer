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

// Physical Constants
static const Colour Gold_Eta(0.17f, 0.35f, 1.5f);
static const Colour Silver_Eta(0.14f, 0.16f, 0.13f);
static const Colour Copper_Eta(0.26f, 0.67f, 1.1f);
static const Colour Aluminium_Eta(1.5f, 0.9f, 0.6f);

static const Colour Gold_K(3.1f, 2.7f, 1.9f);
static const Colour Silver_K(4.1f, 2.3f, 3.1f);
static const Colour Copper_K(3.7f, 2.4f, 2.4f);
static const Colour Aluminium_K(7.0f, 6.0f, 4.7f);

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float eta)
	{
		// Calculate sin2ThetaI (refraction angle)
		float sin2ThetaT = eta * eta * (1.0f - cosTheta * cosTheta);

		// Total Internal Reflection check
		if (sin2ThetaT >= 1.0f)
			return 1.0f;

		// Calculate cosThetaT (refracted angle)
		float cosThetaT = std::sqrtf(1.0f - sin2ThetaT);

		// Compute the parallel and perpendicular Fresnel reflection coefficients
		float fParl = (cosTheta - eta * cosThetaT) / (cosTheta + eta * cosThetaT);
		float fPerp = (eta * cosTheta - cosThetaT) / (eta * cosTheta + cosThetaT);

		// Return the averaged Fresnel term
		return  (fParl * fParl + fPerp * fPerp) * 0.5f;
	}

	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		cosTheta = clamp(cosTheta, 0.0f, 1.0f);

		Colour eta2k2 = ior * ior + k * k;

		Colour cos2Theta = Colour(1.0f, 1.0f, 1.0f) * cosTheta * cosTheta;
		Colour sin2Theta = Colour(1.0f, 1.0f, 1.0f) * (1.0f - cosTheta * cosTheta);

		Colour fParl = (eta2k2 * cos2Theta - ior * 2 * cosTheta + sin2Theta) /
			(eta2k2 * cos2Theta + ior * 2 * cosTheta + sin2Theta);
		Colour fPerp = (eta2k2 - ior * 2 * cosTheta + cos2Theta) /
			(eta2k2 + ior * 2 * cosTheta + cos2Theta);

		// Return the averaged Fresnel term
		return (fParl * fParl + fPerp * fPerp) * 0.5f;
	}

	static float lambdaGGX(Vec3 wi, float alpha)
	{
		float cosTheta = fabsf(wi.z);
		float cos2Theta = cosTheta * cosTheta;
		float tan2Theta = (1.0f - cos2Theta) / cos2Theta;

		return (sqrtf(1.0f + alpha * alpha * tan2Theta) - 1.0f) * 0.5f;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		float g1Wi = 1.0f / (1.0f + lambdaGGX(wi, alpha));
		float g1Wo = 1.0f / (1.0f + lambdaGGX(wo, alpha));
		return g1Wi * g1Wo;
	}

	static float Dggx(Vec3 h, float alpha)
	{
		float cos2Theta = h.z * h.z;
		float alpha2 = alpha * alpha;
		float denom = cos2Theta * (alpha2 - 1.0f) + 1.0f;
		return alpha2 / (M_PI * denom * denom);
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

	// Physical Constants for Fresnel
	const Colour eta = Aluminium_Eta;
	const Colour k = Aluminium_K;

	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		// reflected direction
		Vec3 wi(-localWo.x, -localWo.y, localWo.z);

		Colour F = ShadingHelper::fresnelConductor(fabs(localWo.z), eta, k);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F / fabs(wi.z);
		pdf = 1.0f;

		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(shadingData.frame.toLocal(wi).z);
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
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 wi;
		if (alpha < EPSILON)
		{
			// reflected direction
			wi = Vec3(-localWo.x, -localWo.y, localWo.z);

			Colour F = ShadingHelper::fresnelConductor(fabsf(localWo.z), eta, k);
			reflectedColour = reflectedColour * F / fabs(wi.z);
			pdf = 1.0f;
		}
		else
		{
			float r1 = sampler->next();
			float r2 = sampler->next();

			float cosTheta = sqrtf((1 - r1) / (r1 * (alpha * alpha - 1.0f) + 1.0f));
			float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
			float phi = 2 * M_PI * r2;

			Vec3 wm(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);

			wi = (-localWo + wm * Dot(localWo, wm) * 2.0f).normalize();

			float ggx = ShadingHelper::Gggx(wi, localWo, alpha);
			float D = ShadingHelper::Dggx(wm, alpha);

			Colour F = ShadingHelper::fresnelConductor(fabsf(wi.dot(wm)), eta, k);

			reflectedColour = reflectedColour * F * ggx * D / (4 * fabs(localWo.z) * fabs(wi.z));
			pdf = D * fabsf(cosTheta / (4 * Dot(localWo, wm)));
		}

		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// treat as mirror
		if (alpha < EPSILON)
			return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(shadingData.frame.toLocal(wi).z);

		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Vec3 wm = (localWo + localWi).normalize();

		Colour F = ShadingHelper::fresnelConductor(fabsf(localWi.dot(wm)), eta, k);

		float D = ShadingHelper::Dggx(wm, alpha);
		float ggx = ShadingHelper::Gggx(localWi, localWo, alpha);

		return albedo->sample(shadingData.tu, shadingData.tv) * F * D * ggx / (4.0f * fabsf(localWo.z) * fabsf(localWi.z));
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// treat as mirror
		if (alpha < EPSILON)
			return 0.0f;

		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Vec3 wm = (localWo + localWi).normalize();
		float D = ShadingHelper::Dggx(wm, alpha);

		return D * fabsf(wm.z / (4 * Dot(localWo, wm)));
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

	float eta;
	float invEta;

	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;

		eta = extIOR / intIOR;
		invEta = 1.0f / eta;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		// sample albedo
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		float cosThetaI = fabsf(localWo.z);
		int transmitDir = localWo.z > 0.0f ? -1 : 1;

		// Compute Fresnel reflection coefficient
		float eta = transmitDir < 0 ? this->eta : invEta;
		float F = ShadingHelper::fresnelDielectric(cosThetaI, eta);

		Vec3 wi;
		if (F > sampler->next()) // Reflect
		{
			wi = Vec3(-localWo.x, -localWo.y, localWo.z);
			pdf = F;
			reflectedColour = reflectedColour;
		}
		else // Refract
		{
			// Compute sin square theta using Snell's Law
			float sin2ThetaT = eta * eta * (1.0f - cosThetaI * cosThetaI);
			float cosThetaT = sqrtf(1.0f - sin2ThetaT);

			wi = Vec3(-eta * localWo.x, -eta * localWo.y, transmitDir * cosThetaT);

			pdf = 1.0f - F;
			reflectedColour = reflectedColour * eta * eta;
		}

		reflectedColour = reflectedColour * pdf / fabsf(wi.z);
		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
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

	float A, B;

	OrenNayarBSDF() = default;

	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;

		A = 1.0f - (sigma * sigma / (2.0f * ((sigma * sigma) + 0.33f)));
		B = (0.45 * sigma * sigma) / (sigma * sigma + 0.09f);
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		float thetaO = SphericalCoordinates::sphericalTheta(localWo);
		float phiO = SphericalCoordinates::sphericalPhi(localWo);

		float r1 = sqrtf(sampler->next());
		float thetaI = acosf(r1);
		float phiI = M_PI * 2 * sampler->next();
		float sinTheta = sinf(thetaI);

		Vec3 wi(sinTheta * cosf(phiI), sinTheta * sinf(phiI), r1);
		float demon = (A + B * std::max(cosf(phiI - phiO), 0.0f) * sinf(std::max(thetaI, thetaO) * tanf(std::min(thetaI, thetaO))));

		pdf = fabsf(wi.z) / M_PI;
		reflectedColour = (albedo->sample(shadingData.tu, shadingData.tv) / M_PI) * demon;

		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		float thetaO = SphericalCoordinates::sphericalTheta(localWo);
		float phiO = SphericalCoordinates::sphericalPhi(localWo);

		float thetaI = SphericalCoordinates::sphericalTheta(localWi);
		float phiI = SphericalCoordinates::sphericalPhi(localWi);

		float demon = (A + B * std::max(cosf(phiI - phiO), 0.0f) * sinf(std::max(thetaI, thetaO) * tanf(std::min(thetaI, thetaO))));

		return (albedo->sample(shadingData.tu, shadingData.tv) / M_PI) * demon;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return fabsf(wiLocal.z) / M_PI;
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

	float eta;
	float e; // Phong exponent

	PlasticBSDF() = default;

	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);

		eta = extIOR / intIOR;
		e = (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Colour col = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 wi, wr(-localWo.x, -localWo.y, localWo.z);

		float F = ShadingHelper::fresnelDielectric(std::abs(localWo.z), eta);

		if (sampler->next() < F)
		{
			float cosTheta = powf(sampler->next(), 1.0f / (e + 1.0f));
			float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
			float phi = 2.0f * M_PI * sampler->next();

			Vec3 wl(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
			Frame frame;
			frame.fromVector(wr);

			wi = frame.toWorld(wl);
		}
		else
			wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		float cosAlpha = std::max(0.0f, wr.dot(wi));

		float diffPdf = (1.0f - F) * fabsf(wi.z) / M_PI;
		float specPdf = F * (e + 1.0f) * powf(cosAlpha, e) / (2.0f * M_PI);

		float diff = (1.0f - F) / M_PI;
		float spec = F * (e + 2.0f) * powf(cosAlpha, e) / (2.0f * M_PI);

		pdf = diffPdf + specPdf;
		reflectedColour = col * (diff + spec);

		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Colour col = albedo->sample(shadingData.tu, shadingData.tv);

		float F = ShadingHelper::fresnelDielectric(std::abs(localWi.z), eta);

		Vec3 wr(-localWo.x, -localWo.y, localWo.z);
		float cosAlpha = std::max(0.0f, wr.dot(localWi));

		float diff = (1.0f - F) / M_PI;
		float spec = F * (e + 2.0f) * powf(cosAlpha, e) / (2.0f * M_PI);

		return col * (diff + spec);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);

		Vec3 wr(-localWo.x, -localWo.y, localWo.z);

		float cosTheta = fabsf(localWi.z);
		float cosAlpha = std::max(0.0f, wr.dot(localWi));

		float F = ShadingHelper::fresnelDielectric(cosTheta, eta);

		float diffPdf = (1.0f - F) * cosTheta / M_PI;
		float specPdf = F * (e + 1.0f) * powf(cosAlpha, e) / (2.0f * M_PI);

		return  specPdf + diffPdf;
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