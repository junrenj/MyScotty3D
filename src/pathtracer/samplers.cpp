
#include "samplers.h"
#include "../util/rand.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling
	float x = rng.unit();
	float y = rng.unit();
    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()

    return Vec2{x * size.x, y * size.y};
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec2 Circle::sample(RNG &rng) const {
	//A3EC - bokeh - circle sampling

    // Return a point selected uniformly at random from a circle defined by its
	// center and radius.
    // Useful function: rng.unit()

    return Vec2{};
}

float Circle::pdf(Vec2 at) const {
	//A3EC - bokeh - circle pdf

	// Return the pdf of sampling the point 'at' for a circle defined by its
	// center and radius.

    return 1.f;
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler
	Hemisphere::Uniform sampler;
	Vec3 dir = sampler.sample(rng);
	if(rng.coin_flip(0.5f))
		dir.y = -dir.y;

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
    return dir;
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.
    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
	_pdf.resize(w * h);
	_cdf.resize(w * h);

	float totalWeight = 0.0f;
	for (uint32_t y = 0; y < h; y++)
	{
		for (uint32_t x = 0; x < w; x++)
		{
			Spectrum color = image.at(x, y);
			float brightness = color.luma();
			float weight = brightness * sinf(PI_F *(1.0f - (y + 0.5f)) / h); 

			_pdf[y* w + x] = weight;
			totalWeight += weight;
		}
	}

	float accum = 0.0f;
	for (size_t i = 0; i < _pdf.size(); i++)
	{
		_pdf[i] /= totalWeight;
		_cdf[i] = _pdf[i] + accum;
		accum = _cdf[i];
	}
	_cdf.back() = 1.0f;
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) 
	{
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
		Sphere::Uniform sampler;
		Vec3 dir = sampler.sample(rng);
    	return dir;
	} 
	else 
	{
		// Step 2: Importance sampling
		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
		float random = rng.unit();
		auto it = std::upper_bound(_cdf.begin(), _cdf.end(), random);
		int distance = (int)std::distance(_cdf.begin(), it);
		int y = distance / w;
		int x = distance % w;

		float u = (x + 0.5f) / w;
		float v = 1.0f - (y + 0.5f) / h;

		float phi = 2 * PI_F * u;
		float the_ta = PI_F * v;

		float xc = sinf(the_ta) * cosf(phi);
		float yc = cosf(the_ta);
		float zc = sinf(the_ta) * sinf(phi);
    	return Vec3(xc, yc, zc).unit();
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
	float pdf = 0.0f;
    if(!IMPORTANCE_SAMPLING) 
	{
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		Sphere::Uniform sampler;
		pdf = sampler.pdf(dir);
	} 
	else 
	{
		// A3T7 - image sampler importance sampling pdf
		// What is the PDF of this distribution at a particular direction?
		float theta = acosf(std::clamp(dir.y, -1.0f, 1.0f));
		float phi = atan2f(dir.z, dir.x);
		if (phi < 0.0f) 
			phi += 2.0f * PI_F;

		float u = phi / (2.0f * PI_F);
		float v = theta / PI_F;

		int x = int(u * w);
		int y = int((1.0f - v) * h);

		size_t index = y * w + x;

		float sin_theta = sinf(theta);
		if(sin_theta <= 0.0f)
			sin_theta = 1e-6f;
		pdf =  _pdf[index] * w * h / (2 * PI_F * PI_F * sinf(theta));
	}
	return pdf;
}

} // namespace Samplers
