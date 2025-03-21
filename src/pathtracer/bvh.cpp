#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"
#include <iostream>

#include <stack>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh
	// Keep these
    nodes.clear();
    primitives = std::move(prims);

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	//TODO
	if(primitives.empty())
		return;

	auto build_repeated = [&](auto& self, BVHBuildData data, size_t max_leaf_size)->size_t
	{
		size_t bucket_num = data.range < 16 ? data.range : 16;
		std::vector<SAHBucketData> buckets(bucket_num);
		size_t index = nodes.size(); 
		nodes.emplace_back();
		nodes[index].size = data.range;
		nodes[index].start = data.start;

		// Step 1: Caculate bbox
		BBox bb;
		for (size_t i = data.start; i < data.start + data.range; i++)
		{
			bb.enclose(primitives[i].bbox());
		}
		nodes[index].bbox = bb;
		
		Vec3 range = bb.max - bb.min;
		int t_axis = 0;
		if(range.x > range.y && range.x > range.z)
			t_axis = 0;
		else if(range.y > range.x && range.y > range.z)
			t_axis = 1;
		else 
			t_axis = 2;

		// Step 2: decide whether it is a leaf 
		if(data.range <= max_leaf_size)
		{
			nodes[index].l = 0;
			nodes[index].r = 0;
 			return index;
		}

		// Step 3.0 init bucket
		for (size_t i = 0; i < bucket_num; i++)
		{
			buckets[i].bb = BBox();
			buckets[i].num_prims = 0;
		}

		float axis_min = bb.min[t_axis];
		float axis_max = bb.max[t_axis];
		float bucket_width = (axis_max - axis_min) / (float)bucket_num;
		size_t data_End = data.start + data.range;

		// Step 3 :: assign primitive to target bucket
		for (size_t i = data.start; i < data_End; i++)
		{
			int bucket_idx = std::min((int)bucket_num - 1, (int)((primitives[i].bbox().center()[t_axis] - axis_min) / bucket_width));;
			buckets[bucket_idx].bb.enclose(primitives[i].bbox());
			buckets[bucket_idx].num_prims++;
		}
		
		// Step 4: SAH algorithm
		float lowest_cost = std::numeric_limits<float>::max();
		size_t bestSplit_Bucket = 0;
		BBox left_bb;
		size_t N_left = 0;
		size_t N_right = data.range;

		for (size_t idx = 1; idx < bucket_num - 1; idx++)
		{
			if(buckets[idx].num_prims == 0) 
				continue;
			BBox right_bb;
			left_bb.enclose(buckets[idx].bb);
			for (size_t i = idx + 1; i < bucket_num - 1; i++)
			{
				right_bb.enclose(buckets[i].bb);
			}
			
			N_left += buckets[idx].num_prims;
			N_right -= buckets[idx].num_prims;

			if(N_left == 0 || N_right == 0)
				continue;

			// Caculate lowest cost
			float S_left = left_bb.surface_area();
			float S_right = right_bb.surface_area();
			float cost = S_left * (float)N_left + S_right * (float)N_right;

			if(cost < lowest_cost)
			{
				lowest_cost = cost;
				bestSplit_Bucket = idx;
			}
		}

		auto middle = std::partition(primitives.begin() + data.start, primitives.begin() + data_End, 
		[&](const Primitive& p)
		{
			int bucket_idx = std::min((int)bucket_num - 1, (int)(std::floor((p.bbox().center()[t_axis] - axis_min) / bucket_width)));
			return bucket_idx < bestSplit_Bucket;
		});


		size_t bestSplit_idx = (size_t)std::max((int)std::distance(primitives.begin(), middle), (int)data.start + 1);
		
		BVHBuildData left_data = BVHBuildData(data.start, bestSplit_idx - data.start, nodes.size());
		size_t left_child = 0;
		if(left_data.range >= 1)
			left_child = self(self, left_data, max_leaf_size);
			
		BVHBuildData right_data = BVHBuildData(bestSplit_idx, data_End - bestSplit_idx, nodes.size());
		size_t right_child = 0;
		if(right_data.range >= 1)
			right_child = self(self, right_data, max_leaf_size);

		nodes[index].l = left_child;
		nodes[index].r = right_child;
		return index;
	};

	BVHBuildData newData = BVHBuildData(0, primitives.size(), 0);
	build_repeated(build_repeated, newData, max_leaf_size);
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
	Trace t;
	Vec2 times = Vec2(ray.dist_bounds.x, ray.dist_bounds.y);
	if(nodes.empty())
		return t;
	
	auto FindClosestHit = [&](auto& self, const Node& node, const Ray& ray)-> Trace
	{
		Trace ret;
		ret.distance = std::numeric_limits<float>::max();
		Vec2 localTimes_L = times;
		Vec2 localTimes_R = times;
		if(node.bbox.hit(ray, times))
		{
			if(node.is_leaf())
			{
				for (size_t i = node.start; i < node.start + node.size; i++)
				{
					Trace newRet = primitives[i].hit(ray);
					std::cout <<"isLeaf" <<std::endl;
					if(newRet.hit)
					{
						ret = Trace::min(newRet, ret);
					}
				}
				return ret;
			}
			else
			{
				size_t first = node.l;
				size_t second = node.r;
				bool leftHit;
				bool rightHit;
				leftHit = nodes[first].bbox.hit(ray, localTimes_L);
				rightHit = nodes[second].bbox.hit(ray, localTimes_R);

				if(leftHit && rightHit)
				{
					if(localTimes_L.x > localTimes_R.x)
						std::swap(first, second);
					
						ret = self(self, nodes[first], ray);
						if(!ret.hit)
						{
							ret = self(self, nodes[second], ray);
						}
				}
				else if(leftHit)
					ret = self(self, nodes[first], ray);
				else if(rightHit)
					ret = self(self, nodes[second], ray);	
				return ret;
			}
		}
		else
			return ret;
	};

    t = FindClosestHit(FindClosestHit, nodes[0], ray);
	return t;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
