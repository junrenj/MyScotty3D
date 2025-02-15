
#include "texture.h"
#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	uint32_t ix = uint32_t(std::floor(x));
	uint32_t iy = uint32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, uint32_t(image.w) - 1);
	iy = std::min(iy, uint32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f) - 0.5f;
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f) - 0.5f;
	Spectrum finalCol;
	Spectrum col0;
	Spectrum col1;
	uint32_t x0 = uint32_t(std::max(0.0f, std::floor(x)));
	uint32_t y0 = uint32_t(std::max(0.0f, std::floor(y)));
	x0 = std::min(x0, image.w - 1);
	y0 = std::min(y0, image.h - 1);
	uint32_t x1 = std::min(x0 + 1, image.w - 1);
	uint32_t y1 = std::min(y0 + 1, image.h - 1);

	float tx = x - x0;
	float ty = y - y0;
	// Lerp X
	col0 = image.at(x0, y0) * (1 - tx) + image.at(x1, y0) * tx;
	col1 = image.at(x0, y1) * (1 - tx) + image.at(x1, y1) * tx;
	// Lerp Y
	finalCol = col0 * (1 - ty) + col1 * ty;
	
	return finalCol;
}


Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'
	float goodLod = std::clamp(lod, 0.0f, (float)levels.size());
	if(goodLod == 0.0f)
	{
		return sample_bilinear(base, uv);
	}
	else
	{
		uint32_t low = static_cast<uint32_t>(std::min(std::floor(goodLod), (float)levels.size()));
		uint32_t high = static_cast<uint32_t>(low + 1u) > static_cast<uint32_t>(levels.size()) ?  static_cast<uint32_t>(levels.size()) : static_cast<uint32_t>(low + 1u);
		float t = goodLod - low;
		Spectrum s1 = goodLod < 1.0f ? sample_bilinear(base, uv) : sample_bilinear(levels[low - 1], uv);
		Spectrum s2 = sample_bilinear(levels[high - 1], uv);

		return s1 * (1 - t) + s2 * t;
	}
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		int dstW = dst.w;
		int dstH = dst.h;
		std::vector<Spectrum> dstPixels(dstW * dstH);

		for (int y = 0; y < dstH; y++)
		{
			for (int x = 0; x < dstW; x++)
			{
				int x0 =  2 * x;
				int y0 = 2 * y;
				int x1 = x0 + 1 < (int)src.w - 1 ? x0 + 1 : src.w - 1;
				int y1 = y0 + 1 < (int)src.h - 1 ? y0 + 1 : src.h - 1;

				int index_1 = y0 * src.w + x0;
				int index_2 = y0 * src.w + x1;
				int index_3 = y1 * src.w + x0;
				int index_4 = y1 * src.w + x1;

				int index_dst = y * dst.w + x;

				dstPixels[index_dst] = (src.data().at(index_1) + src.data().at(index_2)
									  + src.data().at(index_3) + src.data().at(index_4)) / 4;
				
			}
		}
		dst = HDR_Image(dstW, dstH, dstPixels);

		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.

	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (image.w == 0 && image.h == 0) return Spectrum();
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
