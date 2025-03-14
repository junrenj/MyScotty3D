
#include "shape.h"
#include "../geometry/util.h"

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
	auto CheckInDis_Bound = [](float t, Vec2 dis_bound)->bool
	{
		return t >= dis_bound.x && t <= dis_bound.y;
	};

	Vec3 O = ray.point;
	Vec3 D = ray.dir;
	float a = D.norm() * D.norm();
	float b = 2 * dot(O, D);
	float c = O.norm() * O.norm() - radius * radius;
	float delta = b * b - 4.0f * a * c;
	float t1 = (-b - sqrt(delta))/ (2.0f * a);
	float t2 = (-b + sqrt(delta))/ (2.0f * a);

	float t = 0.0f;
	bool hasIntersectPoint = false;
	if(CheckInDis_Bound(t1, ray.dist_bounds))
	{
		t = t1;
		hasIntersectPoint = true;
	}
	else if(CheckInDis_Bound(t2, ray.dist_bounds))
	{
		t = t2;
		hasIntersectPoint = true;
	}

    PT::Trace ret;
    ret.origin = ray.point;
    ret.hit = (delta >= 0.0f) && hasIntersectPoint;       // was there an intersection?
    ret.distance = t;   // at what distance did the intersection occur?
    ret.position = O + t * D; // where was the intersection?
    ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
	ret.uv = uv(ret.position); 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
    return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
