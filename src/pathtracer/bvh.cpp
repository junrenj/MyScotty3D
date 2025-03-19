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

	//TODO
	if(primitives.empty())
		return;

	auto build_repeated = [&](auto& self, BVHBuildData data, size_t max_leaf_size)->size_t
	{
		size_t index = nodes.size(); 
		nodes.emplace_back();
		nodes[index].size = data.range;

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
			nodes[index].start = data.start;
			nodes[index].size = data.range;
			nodes[index].l = 0;
			nodes[index].r = 0;
 			return index;
		}
		
		// Step 3: SAH algorithm
		size_t best_split = data.start + 1;
		float lowest_cost = std::numeric_limits<float>::max();
		
		// Devide 3 axis
		for (int i = 0; i < 3; i++)
		{
			// Sort all primitives according to one axis
			std::sort(primitives.begin() + data.start, primitives.begin() + data.start + data.range, 
			[i](const Primitive& a, const Primitive& b)-> bool
			{
				return a.bbox().center()[i] > b.bbox().center()[i];
			});
			
			// Test all split
			for (size_t split_index = data.start + 1; split_index < data.start + data.range; split_index++)
			{
				BBox left_bb;
				BBox right_bb;
				// Left bb
				for (size_t left = data.start; left < split_index; left++)
				{
					left_bb.enclose(primitives[left].bbox());
				}
				// Right bb
				for (size_t right = split_index; right < data.start + data.range; right++)
				{
					right_bb.enclose(primitives[right].bbox());
				}

				// C = C_t + paCa + pbCb
				// C' = SaNa + SbNb
				float N_left = (float)(split_index - data.start);
				float N_right = (float)data.range - N_left;
				float S_left = left_bb.surface_area();
				float S_right = right_bb.surface_area();

				// Caculate lowest cost
				float cost = S_left * N_left + S_right * N_right;
				if(cost < lowest_cost)
				{
					lowest_cost = cost;
					best_split = split_index;
				}
			}
		}
		
		// Sort by best axis
		std::sort(primitives.begin() + data.start, primitives.begin() + data.start + data.range, 
		[t_axis](const Primitive& a, const Primitive& b)-> bool
		{
			return a.bbox().center()[t_axis] > b.bbox().center()[t_axis];
		});

		BVHBuildData left_data = BVHBuildData(data.start, best_split - data.start, nodes.size());
		size_t left_child = self(self, left_data, max_leaf_size);
		BVHBuildData right_data = BVHBuildData(best_split, data.start + data.range - best_split, nodes.size());
		size_t right_child = self(self, right_data, max_leaf_size);

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
		if(node.bbox.hit(ray, times))
		{
			if(node.is_leaf())
			{
				for (size_t i = node.start; i < node.start + node.size; i++)
				{
					ret = primitives[i].hit(ray);
					if(ret.hit)
						return ret;
				}
				return ret;
			}
			else
			{
				size_t first = node.l;
				size_t second = node.r;
				bool leftHit;
				bool rightHit;
				leftHit = nodes[first].bbox.hit(ray, times);
				rightHit = nodes[second].bbox.hit(ray, times);

				if(leftHit && rightHit)		// right is closer
				{
					ret = self(self, nodes[second], ray);
					if(nodes[first].bbox.hit(ray, times))
					{
						ret = self(self, nodes[first], ray);
					}
				}
				else if(!leftHit && !rightHit)// No intersect
					return ret;
				else if(!leftHit && rightHit)	// only right intersect
					ret = self(self, nodes[second], ray);
				else	// left is closer
				{
					ret = self(self, nodes[first], ray);
					if(nodes[second].bbox.hit(ray, times))
					{
						ret = self(self, nodes[second], ray);
					}
				}	
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
