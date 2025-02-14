// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"
template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).
	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}


	std::vector<Vec3> const &samples = framebuffer.sample_pattern.centers_and_weights;
	std::vector<ClippedVertex> supersampleVertex(clipped_vertices.size()); 
	for (uint32_t s = 0; s < samples.size(); s++)
	{
		//--------------------------
		// rasterize primitives:
		std::vector<Fragment> fragments;

		// helper used to put output of rasterization functions into fragments:
		auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };

		for (int j = 0; j < clipped_vertices.size(); j++)
		{
			// SUPERSAMPLING
			float offsetX = samples[s].x - 0.5f;
			float offsetY = samples[s].y - 0.5f;
			supersampleVertex[j] = clipped_vertices[j];
	
			supersampleVertex[j].fb_position.x += offsetX;
			supersampleVertex[j].fb_position.y += offsetY;
		}

		// actually do rasterization:
		if constexpr (primitive_type == PrimitiveType::Lines) {
			for (uint32_t i = 0; i + 1 < supersampleVertex.size(); i += 2) {
				rasterize_line(supersampleVertex[i], supersampleVertex[i + 1], emit_fragment);
			}
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			for (uint32_t i = 0; i + 2 < supersampleVertex.size(); i += 3) {
				rasterize_triangle(supersampleVertex[i], supersampleVertex[i + 1], supersampleVertex[i + 2], emit_fragment);
			}
		} else {
			static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
		}

			//--------------------------
		// depth test + shade + blend fragments:
		uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
								// (indicates something is wrong with clipping)
		for (auto const& f : fragments) 
		{

			// fragment location (in pixels):
			int32_t x = (int32_t)std::floor(f.fb_position.x);
			int32_t y = (int32_t)std::floor(f.fb_position.y);

			// if clipping is working properly, this condition shouldn't be needed;
			// however, it prevents crashes while you are working on your clipping functions,
			// so we suggest leaving it in place:
			if (x < 0 || (uint32_t)x >= framebuffer.width || 
				y < 0 || (uint32_t)y >= framebuffer.height) {
				++out_of_range;
				continue;
			}

			// local names that refer to destination sample in framebuffer:
			float& fb_depth = framebuffer.depth_at(x, y, s);
			Spectrum& fb_color = framebuffer.color_at(x, y, s);


			// depth test:
			if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
				// "Always" means the depth test always passes.
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
				// "Never" means the depth test never passes.
				continue; //discard this fragment
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) 
			{
				// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
				// A1T4: Depth_Less
				// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
				if(fb_depth < f.fb_position.z)
					continue; // discard
			} else {
				static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
			}

			// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
			if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
				fb_depth = f.fb_position.z;
			}

			// shade fragment:
			ShadedFragment sf;
			sf.fb_position = f.fb_position;
			Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

			// write color to framebuffer if color writes aren't disabled:
			if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
				// blend fragment:
				if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
					fb_color = sf.color;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
					// A1T4: Blend_Add
					// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
					fb_color += sf.color * sf.opacity;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
					// A1T4: Blend_Over
					// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
					// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
					fb_color = fb_color * (1 - sf.opacity) + sf.color * sf.opacity; 
				} else {
					static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
				}
			}
			if (out_of_range > 0) {
				if constexpr (primitive_type == PrimitiveType::Lines) {
					warn("Produced %d fragments outside framebuffer; this indicates something is likely "
							"wrong with the clip_line function.",
							out_of_range);
				} else if constexpr (primitive_type == PrimitiveType::Triangles) {
					warn("Produced %d fragments outside framebuffer; this indicates something is likely "
							"wrong with the clip_triangle function.",
							out_of_range);
				}
			}
		}
		
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	// Using Sutherland-Hodgman 
	/*std::vector<ShadedVertex> input = {va, vb, vc};
	std::vector<ShadedVertex> output;
	auto IntersectPoint = [](ShadedVertex v1, ShadedVertex v2, float t)-> ShadedVertex
	{	
		ShadedVertex sv;
		sv.clip_position = v1.clip_position * t + v2.clip_position * (1 - t);
		for (int i = 0; i < sv.attributes.size(); i++)
		{
			sv.attributes[i] = v1.attributes[i] * t + v2.attributes[i] * (1 - t);
		}
		return sv;
	};

	auto EdgeFunc = [](float x1, float x2, float y1, float y2, float x, float y) -> float 
	{
		return (x2 - x1)(y - y1) - (y2 - y1)(x - x1);
	};

	int w = va.clip_position.w;
	bool insides[2];
	// start from A
	insides[0] = va.clip_position.x >= -w && va.clip_position.x <= w && va.clip_position.y <= w && va.clip_position.y >= -w;
	if(insides[0])
		output.push_back(va);
	
	// left side
	*/


	

	 emit_vertex(va);
	 emit_vertex(vb);
	 emit_vertex(vc);
}

// -------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) 
	{
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}
	// A1T2: rasterize_line

	// TODO: Check out the block comment above this function for more information on how to fill in
	// this function!
	// The OpenGL specification section 3.5 may also come in handy.
	
	// get screen pos
	Vec3 v0 = va.fb_position;
	Vec3 v1 = vb.fb_position;
	int x0 = static_cast<int>(v0.x);
	int x1 = static_cast<int>(v1.x);
	int y0 = static_cast<int>(v0.y);
	int y1 = static_cast<int>(v1.y);
	float z0 = v0.z;
	float z1 = v1.z;
	
	int dx = x1 - x0;
	int dy = y1 - y0;
	float dz = z1 - z0;

	int stepX = 1;
	int stepY = 1;

	bool isSwap = false;

	// ensure x1 > x0
	if(x1 < x0)
	{
		std::swap(v0,v1);
		std::swap(x0,x1);
		std::swap(y0,y1);
		std::swap(z0,z1);
		dx = x1 - x0;
	    dy = y1 - y0;
		dz = z1 - z0;
		isSwap = true;
	}

	if(y1 < y0)
	{
		stepY = -stepY;
		dy = abs(dy);
	}

	float slope = (float)dy / dx;
	
	int errX = 2 * dy - dx;
	int errY = 2 * dx - dy;

	dz = dz / std::max(dx, dy);

	int x = x0;
	int y = y0;

	auto FloatEqual = [](float a, float target) -> bool
	{
		return fabs(a - target) <= 1e-9;
	};

	auto DiamondExitEnd = [&](int x, int y, float x1, float y1, float k) -> bool 
	{
		float solution;
		int count = 0;
		float b;

		if(FloatEqual((float)x, x1))
			return false;

		if(std::isinf(k))	// in case k = inf
		{
			b = x1 - x;
			if(b <= 0.5f)
			{
				solution = b + 0.5f;
				if( solution > 0.5f && solution <= y1 && solution < 1.0f)
					count ++;
				solution = 0.5f - b;
				if( solution > 0 && solution <= y1 && solution < 0.5f)
					count ++;
			}
			else
			{
				solution = b - 0.5f;
				if( solution > 0 && solution <= y1  && solution < 0.5f)
					count ++;
				solution = 1.5f - b;
				if( solution > 0.5f && solution <= y1 && solution < 1.0f)
					count ++;
			}
		}
		else
		{
			b = k * (y1 - y) / (x1 - x);
			// true shaded false not shaded
			// if intersection point >= 2 the line must pass through whole diamond
			// y = x + 0.5f and y = x - 0.5f
			if(k != 1)
			{
				// y = x + 0.5f
				solution = (0.5f - b) / (k - 1);
				if(solution >= 0 && solution < 0.5f && solution <= (x1 - x))
					count++;
				// y = x - 0.5f
				solution = (-0.5f - b) / (k - 1);
				if(solution >= 0.5f && solution < 1 && solution <= (x1 - x))
					count++;
			}
			else
			{
				if(FloatEqual(b, 0.5f) && FloatEqual(x1, x + 0.5f))
					return true;
				else if(FloatEqual(b, -0.5f) && FloatEqual(x1, x + 1.0f) && !FloatEqual(x1, x + 0.5f))
					return true;
			}
			// y = -x + 1.5f and y = -x + 0.5f
			if(k != -1)
			{
				// y = -x + 1.5f
				solution = (1.5f - b) / (k + 1);
				if(solution > 0.5f && solution < 1.0f && solution <=(x1 - x))
					count++;
				// y = -x + 0.5f
				solution = (0.5f - b) / (k + 1);
				if(solution >= 0 && solution <= 0.5f && solution <= (x1 - x))
					count++;
			}
			else
			{
				if(FloatEqual(b, 1.5f))
					return false;
				else if(FloatEqual(b, 0.5f) && y1 - y < 0)
					return true;
			}
		}
		return count >= 2;
	};

	while(true)
	{
		float centerX = x + 0.5f;
		float centerY = y + 0.5f;

		Fragment frag;
		frag.fb_position = Vec3(centerX, centerY, z0);
		frag.derivatives.fill(Vec2(0.0f, 0.0f));
		frag.attributes = va.attributes;

		if(x == x0 && y == y0)
		{
			if(isSwap)
			{
				// check startPoint instead of end point
				if(DiamondExitEnd(x, y, v0.x, v0.y, (v1.x - v0.x)/(v1.y - v0.y)))
					emit_fragment(frag);
			}
		}
		if(x == x1 && y == y1)	
		{
			if(!isSwap)
			{
				// end Point Check
				if(DiamondExitEnd(x, y, v1.x, v1.y, slope))
					emit_fragment(frag);
			}
			else
			{
				emit_fragment(frag);
			}
			break;
		}
		else
			emit_fragment(frag);

		// X is main
		if(errX < 0)
			errX += 2 * dy;
		else
		{
			// change (y++)
			errX += 2*(dy -dx);
			y += stepY;
		}

		// Y is main
		if(errY < 0)
			errY += 2*dx;
		else
		{
			// change x++
			errY += 2*(dx - dy);
			x += stepX;
		}
		z0 += dz;
	}
}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// NOTE: it is okay to restructure this function to allow these tasks to use the
	//  same code paths. Be aware, however, that all of them need to remain working!
	//  (e.g., if you break Flat while implementing Correct, you won't get points
	//   for Flat.)
	
	// Boundbox
	int minX = static_cast<int>(std::floor(std::min({va.fb_position.x, vb.fb_position.x, vc.fb_position.x})));
	int minY = static_cast<int>(std::floor(std::min({va.fb_position.y, vb.fb_position.y, vc.fb_position.y})));
	int maxX = static_cast<int>(std::ceil(std::max({va.fb_position.x, vb.fb_position.x, vc.fb_position.x})));
	int maxY = static_cast<int>(std::ceil(std::max({va.fb_position.y, vb.fb_position.y, vc.fb_position.y})));

	auto TopLeftEdge = [](Vec2 a, Vec2 b)-> bool
	{
		bool b1 = a.y == b.y && a.x > b.x;	// true mean it is the top edge false mean it is the bottom
		bool b2 = a.y < b.y; // because it is clockwise, so b.y > a.y, true mean it is left edge
		return b1 || b2 ;
	};

	// ab * ap
	auto EdgeFunction = [&](Vec2 a, Vec2 b, Vec2 p) -> float
	{
		float edgeValue = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
		if(edgeValue == 0)
		{
			// mean point Q is on the edge. So we have to check whether the edge is the top left
			if(TopLeftEdge(a, b))
				edgeValue = -10000;
		}
    	return edgeValue;
	};


	Vec2 a = Vec2(va.fb_position.x, va.fb_position.y);
	Vec2 b = Vec2(vb.fb_position.x, vb.fb_position.y);
	Vec2 c = Vec2(vc.fb_position.x, vc.fb_position.y);

	float area = EdgeFunction(a, b, c);

	// just lines
	if(area == 0)
		return;

	// method 1:using changes of barycentric cooridinate
	// for (int y = minY; y <= maxY; y++)
	// {
	// 	for (int x = minX; x <= maxX; x++)
	// 	{
	// 		// clockwise 
	// 		Vec2 target = Vec2(x + 0.5f, y + 0.5f);
	// 		float alpha = EdgeFunction(b, c, target);
	// 		float beta = EdgeFunction(c, a, target);
	// 		float gamma = EdgeFunction(a, b, target);

	// 		// check top left rule
	// 		if(alpha == -10000 || beta == -10000 || gamma == -10000)
	// 		 	continue;

	// 		if(alpha >= 0 && beta >= 0 && gamma >= 0 || alpha <= 0 && beta <= 0 && gamma <= 0)
	// 		{
	// 			Fragment frag;
	// 			Vec2 derivative;

	// 			alpha /= area;
	// 			beta /= area;
	// 			gamma = 1 - alpha - beta;

	// 			frag.fb_position = Vec3(target.x, target.y, alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z);
				
	// 			float alpha_deX = vb.fb_position.y - vc.fb_position.y;
	// 			float beta_deX = vc.fb_position.y - va.fb_position.y;
	// 			float gamma_deX = va.fb_position.y - vb.fb_position.y;
	// 			float alpha_deY = vc.fb_position.x - vb.fb_position.x;
	// 			float beta_deY =  va.fb_position.x - vc.fb_position.x;
	// 			float gamma_deY = vb.fb_position.x -  va.fb_position.x;
					
	// 			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) 
	// 			{
	// 				frag.attributes = va.attributes;
	// 				for(int i = 0; i < frag.derivatives.size(); i++)
	// 				{
	// 					frag.derivatives[i] = Vec2(0.0f, 0.0f);
	// 					emit_fragment(frag);
	// 					continue;
	// 				}
	// 			}
	// 			else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) 
	// 			{
	// 				// A1T5: screen-space smooth triangles
	// 				for (int i = 0; i < va.attributes.size(); i++)
	// 				{
	// 					frag.attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
	// 				}
						
	// 			} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) 
	// 			{
	// 				// 1/wp = alpha/wa + beta/wb + gamma/wc
	// 				float wpInverse = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;
	// 				float wp = 1 / wpInverse;
	// 				for (int i = 0; i < va.attributes.size(); i++)
	// 				{
	// 					float attribute = alpha * va.attributes[i] * va.inv_w 
	// 											+ beta * vb.attributes[i] * vb.inv_w
	// 											+ gamma * vc.attributes[i] * vc.inv_w;
	// 					frag.attributes[i] = wp * attribute;
	// 				}
	// 			}

	// 			for (int i = 0; i < frag.derivatives.size(); i++)
	// 			{
	// 				derivative.x = alpha_deX * va.attributes[i] + beta_deX * vb.attributes[i] + gamma_deX * vc.attributes[i];
	// 				derivative.y = alpha_deY * va.attributes[i] + beta_deY * vb.attributes[i] + gamma_deY * vc.attributes[i];
	// 				frag.derivatives[i] = derivative / area;
	// 			}
	// 			emit_fragment(frag);
	// 		}
	// 	}
	// }


	// method 2 : 
	// using p(x+1, y) - p(x, y) and p(x, y + 1)  - p(x, y)
	for (int y = minY; y < maxY; y++)
	{
	 	for (int x = minX; x < maxX; x++)
	 	{
			// 2x2 block for d
			Fragment frags[3];
			bool validate = false;
			for (int j = 0; j < 3; j++)
			{
				Vec2 target = Vec2(x + 0.5f, y + 0.5f);
				if(j == 1)
				{
					target += Vec2(1.0f, 0.0f);
				}
				else if(j == 2)
				{
					target += Vec2(0.0f, 1.0f);
				}
				// clockwise 
				float alpha = EdgeFunction(b, c, target);
				float beta = EdgeFunction(c, a, target);
				float gamma = EdgeFunction(a, b, target);
	
				alpha /= area;
				beta /= area;
				gamma = 1 - alpha - beta;
	
				frags[j].fb_position = Vec3(target.x, target.y, alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z);
				
				if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) 
				{
					frags[j].attributes = va.attributes;
				}
				else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) 
				{
					// A1T5: screen-space smooth triangles
					for (int i = 0; i < va.attributes.size(); i++)
					{
						frags[j].attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
					}
					
				} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) 
				{
					// 1/wp = alpha/wa + beta/wb + gamma/wc
					float wpInverse = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;
					float wp = 1 / wpInverse;
					for (int i = 0; i < va.attributes.size(); i++)
					{
						float attribute = alpha * va.attributes[i] * va.inv_w 
												+ beta * vb.attributes[i] * vb.inv_w
												+ gamma * vc.attributes[i] * vc.inv_w;
						frags[j].attributes[i] = wp * attribute;
					}
				}
				if(j == 0)
				{
					// point insides the triangle
					if(alpha >= 0 && beta >= 0 && gamma >= 0 || alpha <= 0 && beta <= 0 && gamma <= 0)
						validate = true;
					// check top left rule
					if(alpha == -10000 || beta == -10000 || gamma == -10000)
						validate = false;
				}
			}

			for (int i = 0; i < frags[0].derivatives.size(); i++)
			{
				frags[0].derivatives[i] = Vec2(frags[1].attributes[i] - frags[0].attributes[i], frags[2].attributes[i] - frags[0].attributes[i]);
			}

			if(validate)
			 	emit_fragment(frags[0]);
		}
	}
		
}


//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;