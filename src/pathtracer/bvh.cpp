#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

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

	// //TODO
	if(primitives.empty())
		return;

	std::function<void(BVHBuildData, size_t)> build_repeated;
	build_repeated = [&](BVHBuildData data, size_t max_leaf_size)
	{
		if(data.range <= max_leaf_size)
			return;
		
		size_t index = data.node;

		// Step 1: Caculate bbox
		BBox bb = nodes[index].bbox;

		size_t leftSize_best = 0;
		size_t rightSize_best = 0;
		BBox left_bb_best;
		BBox right_bb_best;
		int axis_best = 0;
		float partition_best = 0.0f;
		float cost_lowest = FLT_MAX;
		
		size_t bucket_num = data.range < 16 ? data.range : 16;

		for (int axis = 0; axis < 3; axis++)
		{
			float bucket_w = (bb.max - bb.min)[axis] / float(bucket_num);
			std::vector<SAHBucketData> buckets(bucket_num);
			if (bucket_w < 1e-6f) 
				bucket_w = 1e-6f; 

			for (size_t p_idx = data.start; p_idx < data.start + data.range; p_idx++)
			{
				float center = primitives[p_idx].bbox().center()[axis];
				size_t bucket_idx = static_cast<size_t>((center - bb.min[axis]) / bucket_w);
				
				if (bucket_idx >= bucket_num) 
					bucket_idx = bucket_num - 1;
			
				buckets[bucket_idx].bb.enclose(primitives[p_idx].bbox());
				buckets[bucket_idx].num_prims++;
			}

			for (size_t bucketSplit_idx = 1; bucketSplit_idx < bucket_num; bucketSplit_idx++)
			{
				BBox left_bb;
				BBox right_bb;
				size_t N_left = 0;
				size_t N_right = 0;
				for (size_t l = 0; l < bucketSplit_idx; l++)
				{
					left_bb.enclose(buckets[l].bb);
					N_left += buckets[l].num_prims;
				}
				for (size_t r = bucketSplit_idx; r < bucket_num; r++)
				{
					right_bb.enclose(buckets[r].bb);
					N_right += buckets[r].num_prims;
				}
				float cost = left_bb.surface_area() * (float)N_left + right_bb.surface_area() * (float)N_right;
				
				if(cost < cost_lowest)
				{
					cost_lowest = cost;
					leftSize_best = N_left;
					rightSize_best = N_right;
					left_bb_best = left_bb;
					right_bb_best = right_bb;
					partition_best = bb.min[axis] + bucket_w * bucketSplit_idx;
					axis_best = axis;
				}
			}
		}

		std::partition(primitives.begin() + data.start, primitives.begin() + data.start + data.range, 
		[axis_best, partition_best](Primitive& p)
		{
			return p.bbox().center()[axis_best] <= partition_best;
		});
		
		nodes[index].l = new_node(left_bb_best, data.start, leftSize_best, 0, 0);
		nodes[index].r = new_node(right_bb_best, data.start + leftSize_best, rightSize_best, 0, 0);

		BVHBuildData left_data = BVHBuildData(data.start, leftSize_best, nodes[index].l);
		build_repeated(left_data, max_leaf_size);
		BVHBuildData right_data = BVHBuildData(data.start + leftSize_best, rightSize_best, nodes[index].r);
		build_repeated(right_data, max_leaf_size);
	};
	BBox bb_root;
	for (size_t i = 0; i < primitives.size(); i++)
	{
		bb_root.enclose(primitives[i].bbox());
	}
	
	new_node(bb_root,0,primitives.size(),0,0);
	BVHBuildData newData = BVHBuildData(0, primitives.size(), 0);
	build_repeated(newData, max_leaf_size);
	
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
	Vec2 times = ray.dist_bounds;
	if(nodes.empty())
		return t;
	
	auto FindClosestHit = [&](auto& self, const Node& node, const Ray& ray, Vec2 newTime)-> Trace
	{
		Trace ret;
		ret.distance = FLT_MAX;
		if(node.bbox.hit(ray, newTime))
		{
			if(node.is_leaf())
			{
				for (size_t i = node.start; i < node.start + node.size; i++)
				{
					Trace newRet = primitives[i].hit(ray);
					if(newRet.hit && newRet.distance < ret.distance)
						ret = newRet;
				}
				return ret;
			}
			else
			{
				size_t first = node.l;
				size_t second = node.r;
				bool leftHit;
				bool rightHit;
				Vec2 localTimes_L = newTime;
				Vec2 localTimes_R = newTime;
				leftHit = nodes[first].bbox.hit(ray, localTimes_L);
				rightHit = nodes[second].bbox.hit(ray, localTimes_R);

				if(leftHit && rightHit)
				{
					// test the cloest one first
					if(localTimes_L.x > localTimes_R.x)
					{
						std::swap(first, second);
						std::swap(localTimes_L, localTimes_R);
					}
					
					Trace leftTrace = self(self, nodes[first], ray, localTimes_L);
					if(leftTrace.hit)
						ret = leftTrace;

					Trace rightTrace = self(self, nodes[second], ray, localTimes_R);
                    if (rightTrace.hit && rightTrace.distance <= ret.distance)
                        ret = rightTrace;
				}
				else if(leftHit)
					ret = self(self, nodes[first], ray, localTimes_L);
				else if(rightHit)
					ret = self(self, nodes[second], ray, localTimes_R);	
				return ret;
			}
		}
		else
			return ret;
	};

    t = FindClosestHit(FindClosestHit, nodes[0], ray, times);
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
