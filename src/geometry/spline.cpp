
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...

	// special case 1 : no knot
	if(knots.size() == 0)	
		return T();
	// special case 2 : one knot
	if(knots.size() == 1)	
		return knots.begin()->second;
	
	// special case 3 : time < first knot
	if(time <= knots.begin()->first)
		return  knots.begin()->second;

	// special case 4 : time > last knot
	if(time >= (std::prev(knots.end()))->first)
		return  (std::prev(knots.end()))->second;

	T p0, p1, p2, p3;
	float t0, t1, t2, t3;
	auto it = knots.upper_bound(time);
	p2 = it->second;
	t2 = it->first;
	p1 = std::prev(it)->second;
	t1 = std::prev(it)->first;

	if(std::next(it) != knots.end())
	{
		p3 = std::next(it)->second;
		t3 = std::next(it)->first;
	}
	else
	{
		p3 = 2.0f * p2 - p1;
		t3 = 2.0f * t2 - t1;
	}

	if(std::prev(it) != knots.begin())
	{
		p0 = std::prev(std::prev(it))->second;
		t0 = std::prev(std::prev(it))->first;
	}
	else
	{
		p0 = 2.0f * p1 - p2;
		t0 = 2.0f * t1 - t2;
	}

	T m0 = (p2 - p0) / (t2 - t0);
	T m1 = (p3 - p1) / (t3 - t1);

	float timeLerp = (time - t1) / (t2 - t1);

	return cubic_unit_spline(timeLerp, p1, p2, m0, m1);
}

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.

	float h00 = 2.0f * pow(time, 3.0f) - 3.0f * time * time + 1.0f;
	float h10 = pow(time, 3.0f) - 2.0f * time * time + time;
	float h01 = -2.0f * pow(time, 3.0f) + 3 * time * time;
	float h11 = pow(time, 3.0f) - time * time;

	T value = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;

	return value;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
