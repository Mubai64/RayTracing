#pragma once
#include "Geometry.h"

namespace PathTracing
{

vec3 radiance(const Ray& r, int depth)
{
	double t = 1e20;			// distance to intersection
	int id = -1;				// id of intersected object
	if (!intersect(r, t, id))	// if miss, return black
		return vec3(0.,0.,0.);

	const Sphere& obj = sen[id];        // the hit object

	vec3 x = r.getPosition(t);
	vec3 n = normalize(x - obj.O);
	vec3 nl = dot(n, r.D) < 0. ? n : n * -1.;
	vec3 f = obj.m.Albedo;

	return n;

	double p = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; // max refl

	if (++depth > 5)
	{
		if (randx() < p)
			f = f * (1 / p);
		else
			return obj.m.Emissive; //R.R.
	}

	if (depth > 100)
		return obj.m.Emissive;

	if (obj.m.roughness > 0) // Ideal DIFFUSE reflection 
	{
		double theta = randx() * 0.5 * PI * obj.m.roughness;
		double phi = randx() * 2. * PI;

		Ray ref(x, spheredir(n, theta, phi));

		return obj.m.Emissive + f * radiance(ref, depth);
	}
	else if (obj.m.roughness == 0 && obj.m.refractive == 0 ) // Ideal SPECULAR reflection
	{
		return obj.m.Emissive + f * radiance(Ray(x, reflect(r.D, n)), depth);
	}
	else // Ideal dielectric REFRACTION
	{
		Ray reflRay(x, reflect(r.D, n));
		bool into = dot(n, nl) > 0;                // Ray from outside going in?
		double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = dot(r.D, nl), cos2t;
		if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)    // Total internal reflection
			return obj.m.Emissive + f * radiance(reflRay, depth);

		vec3 tdir = normalize(r.D * nnt - n * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t))));
		double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), c = 1 - (into ? -ddn : dot(tdir, n));
		double Re = R0 + (1 - R0) * c * c * c * c * c, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
		return obj.m.Emissive + f * (depth > 2 ? (randx() < P ?   // Russian roulette
			radiance(reflRay, depth) * RP : radiance(Ray(x, tdir), depth) * TP) :
			radiance(reflRay, depth) * Re + radiance(Ray(x, tdir), depth) * Tr);
	}
};

void render()
{
	double offset = 1.0f / SIZE;
	static const size_t samplecount = 1;

	for (size_t i = 0; i < SIZE; i++)
	{
		for (size_t j = 0; j < SIZE; j++)
		{
			vec3 sum(0.,0.,0.);
			for (size_t s = 0; s < samplecount; s++)
			{
				Ray r = cam.generateRay(offset * (i + randx()*0.5 - 0.25), offset * (j + randx() * 0.5 - 0.25));
				sum = sum + radiance(r, 0);
			}

			framebuffer[i][j] = (uint32_t)(sum / (double)samplecount);
		}
	}

}

}