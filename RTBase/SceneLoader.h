#pragma once

// Do not change any of this!

#include "GEMLoader.h"
#include "Renderer.h"

class RTCamera
{
public:
	Vec3 from;
	Vec3 to;
	Vec3 up;
	Camera* camera = NULL;
	float movespeed = 1.0f;
	float rotspeed = 5.0f;
	RTCamera()
	{
		rotspeed = 5.0f;
	}
	void forward()
	{
		Vec3 dir = to - from;
		dir = dir.normalize() * movespeed;
		from = from + dir;
		to = from + dir;
		updateCamera();
	}
	void back()
	{
		Vec3 dir = to - from;
		dir = dir.normalize() * movespeed;
		from = from - dir;
		to = from + dir;
		updateCamera();
	}
	void left()
	{
		Vec3 dir = to - from;
		dir = dir.normalize();

		float rad = rotspeed * (M_PI / 180.0f);
		float cosTheta = cosf(rad);
		float sinTheta = sinf(rad);
		Vec3 k = up;
		Vec3 rotated = (dir * cosTheta) + (k.cross(dir) * sinTheta) + (k * (k.dot(dir) * (1 - cosTheta)));
		dir.x = rotated.x;
		dir.y = rotated.y;
		dir.z = rotated.z;
		to = from + dir;
		updateCamera();
	}
	void right()
	{
		Vec3 dir = to - from;
		dir = dir.normalize();
		float rad = -rotspeed * (M_PI / 180.0f);
		float cosTheta = cosf(rad);
		float sinTheta = sinf(rad);
		Vec3 k = up;
		Vec3 rotated = (dir * cosTheta) + (k.cross(dir) * sinTheta) + (k * (k.dot(dir) * (1 - cosTheta)));
		dir.x = rotated.x;
		dir.y = rotated.y;
		dir.z = rotated.z;
		to = from + dir;
		updateCamera();
	}
	void flyUp()
	{
		Vec3 dir = up * movespeed;
		from = from + dir;
		to = to + dir;
		updateCamera();
	}
	void flyDown()
	{
		Vec3 dir = up * movespeed;
		from = from - dir;
		to = to - dir;
		updateCamera();
	}
	void updateCamera()
	{
		Matrix V = Matrix::lookAt(from, to, up);
		V = V.invert();
		camera->updateView(V);
	}
};

static RTCamera viewcamera;

Texture* loadTexture(std::string filename, std::map<std::string, Texture*>& textureManager)
{
	if (textureManager.find(filename) != textureManager.end())
	{
		return textureManager[filename];
	}
	Texture* texture = new Texture();
	texture->load(filename);
	textureManager.insert({ filename, texture });
	return texture;
}

void loadInstance(std::string sceneName, std::vector<Triangle>& meshTriangles, std::vector<BSDF*>& meshMaterials, GEMLoader::GEMInstance& instance, std::map<std::string, Texture*>& textureManager)
{
	GEMLoader::GEMModelLoader loader;
	std::vector<GEMLoader::GEMMesh> meshes;
	loader.load(sceneName + "/" + instance.meshFilename, meshes);
	// Add Material
	BSDF* material = NULL;
	if (instance.material.find("bsdf").getValue("") == "diffuse")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		material = new DiffuseBSDF(loadTexture(filename, textureManager));
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "orennayar")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		material = new OrenNayarBSDF(loadTexture(filename, textureManager), instance.material.find("alpha").getValue(1.0f));
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "glass")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		float intIOR = instance.material.find("intIOR").getValue(1.33f);
		float extIOR = instance.material.find("extIOR").getValue(1.0f);
		material = new GlassBSDF(loadTexture(filename, textureManager), intIOR, extIOR);
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "mirror")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		material = new MirrorBSDF(loadTexture(filename, textureManager));
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "plastic")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		Colour eta;
		Colour k;
		instance.material.find("eta").getValuesAsVector3(eta.r, eta.g, eta.b);
		instance.material.find("k").getValuesAsVector3(k.r, k.g, k.b);
		float roughness = instance.material.find("roughness").getValue(1.0f);
		material = new ConductorBSDF(loadTexture(filename, textureManager), eta, k, roughness);
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "dielectric")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		float intIOR = instance.material.find("intIOR").getValue(1.33f);
		float extIOR = instance.material.find("extIOR").getValue(1.0f);
		material = new GlassBSDF(loadTexture(filename, textureManager), intIOR, extIOR);
		float roughness = instance.material.find("roughness").getValue(1.0f);
		if (roughness < 0.001f)
		{
			material = new GlassBSDF(loadTexture(filename, textureManager), intIOR, extIOR);
		} else
		{
			material = new DielectricBSDF(loadTexture(filename, textureManager), intIOR, extIOR, roughness);
		}
		meshMaterials.push_back(material);
	}
	if (instance.material.find("bsdf").getValue("") == "conductor")
	{
		std::string filename = sceneName + "/" + instance.material.find("reflectance").getValue("");
		Colour eta;
		Colour k;
		instance.material.find("eta").getValuesAsVector3(eta.r, eta.g, eta.b);
		instance.material.find("k").getValuesAsVector3(k.r, k.g, k.b);
		float roughness = instance.material.find("roughness").getValue(1.0f);
		material = new ConductorBSDF(loadTexture(filename, textureManager), eta, k, roughness);
		meshMaterials.push_back(material);
	}
	if (instance.material.find("emission").getValue("") != "")
	{
		Colour emission;
		instance.material.find("emission").getValuesAsVector3(emission.r, emission.g, emission.b);
		material->addLight(emission);
	}
	if (instance.material.find("coatingThickness").getValue(0) > 0)
	{
		BSDF* base = material;
		Colour sigmaa;
		instance.material.find("coatingSigmaA").getValuesAsVector3(sigmaa.r, sigmaa.g, sigmaa.b);
		float intIOR = instance.material.find("coatingIntIOR").getValue(1.33f);
		float extIOR = instance.material.find("coatingExtIOR").getValue(1.0f);
		float thickness = instance.material.find("coatingThickness").getValue(0.0f);
		material = new LayeredBSDF(base, sigmaa, thickness, intIOR, extIOR);
	}
	if (material == NULL)
	{
		// Flag there is an issue but keep loading as the rest of the scene might be fine
		std::cout << "Error in loading" << std::endl;
		return;
	}
	int materialIndex = (int)meshMaterials.size() - 1;
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	Matrix transform;
	memcpy(transform.m, instance.w.m, 16 * sizeof(float));
	Matrix vecTransform = transform.invert();
	vecTransform = vecTransform.transpose();
	for (int i = 0; i < meshes.size(); i++)
	{
		for (int n = 0; n < meshes[i].verticesStatic.size(); n++)
		{
			Vertex v;
			v.p.x = meshes[i].verticesStatic[n].position.x;
			v.p.y = meshes[i].verticesStatic[n].position.y;
			v.p.z = meshes[i].verticesStatic[n].position.z;
			v.normal.x = meshes[i].verticesStatic[n].normal.x;
			v.normal.y = meshes[i].verticesStatic[n].normal.y;
			v.normal.z = meshes[i].verticesStatic[n].normal.z;
			// Transform
			v.p = transform.mulPoint(v.p);
			v.normal = vecTransform.mulVec(v.normal);
			v.normal = v.normal.normalize();
			v.u = meshes[i].verticesStatic[n].u;
			v.v = meshes[i].verticesStatic[n].v;
			vertices.push_back(v);
		}
		int offset = (int)indices.size();
		for (int n = 0; n < meshes[i].indices.size(); n++)
		{
			indices.push_back(offset + meshes[i].indices[n]);
		}
	}
	for (int i = 0; i < indices.size(); i = i + 3) {
		Triangle t;
		t.init(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]], materialIndex);
		if (t.area > 0)
		{
			meshTriangles.push_back(t);
		}
	}
}

Scene* loadScene(std::string sceneName)
{
	Scene* scene = new Scene();
	GEMLoader::GEMScene gemscene;
	gemscene.load(sceneName + "/scene.json");
	int width = gemscene.findProperty("width").getValue(1920);
	int height = gemscene.findProperty("height").getValue(1080);
	float fov = gemscene.findProperty("fov").getValue(45.0f);
	Matrix P = Matrix::perspective(0.001f, 10000.0f, (float)width / (float)height, fov);
	Vec3 from;
	Vec3 to;
	Vec3 up;
	gemscene.findProperty("from").getValuesAsVector3(from.x, from.y, from.z);
	gemscene.findProperty("to").getValuesAsVector3(to.x, to.y, to.z);
	gemscene.findProperty("up").getValuesAsVector3(up.x, up.y, up.z);
	Matrix V = Matrix::lookAt(from, to, up);
	V = V.invert();
	int flip = gemscene.findProperty("flipX").getValue(0);
	if (flip == 1)
	{
		P.a[0][0] = -P.a[0][0];
	}
	scene->camera.init(P, width, height);
	scene->camera.updateView(V);

	viewcamera.from = from;
	viewcamera.to = to;
	viewcamera.up = up;
	viewcamera.camera = &scene->camera;

	std::vector<Triangle> meshTriangles;
	std::vector<BSDF*> meshMaterials;
	std::vector<unsigned int> lights;
	std::map<std::string, Texture*> textureManager;
	for (int i = 0; i < gemscene.instances.size(); i++)
	{
		loadInstance(sceneName, meshTriangles, meshMaterials, gemscene.instances[i], textureManager);
	}
	Light* background;
	if (gemscene.findProperty("envmap").getValue("") != "")
	{
		Texture* env = loadTexture(sceneName + "/" + gemscene.findProperty("envmap").getValue(""), textureManager);
		background = new EnvironmentMap(env);
	} else
	{
		background = new BackgroundColour(Colour(0.0f, 0.0f, 0.0f));
	}
	scene->init(meshTriangles, meshMaterials, background);
	viewcamera.movespeed = (scene->bounds.max - scene->bounds.min).length() * 0.05f;
	scene->build();
	use<SceneBounds>().sceneCentre = (scene->bounds.max + scene->bounds.min) * 0.5f;
	use<SceneBounds>().sceneRadius = (scene->bounds.max - use<SceneBounds>().sceneCentre).length();
	return scene;
}