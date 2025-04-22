#include <unordered_set>
#include "skeleton.h"
#include "test.h"

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.

	for (size_t i = 0; i < bones.size(); i++)
	{
		Mat4 mat;
		BoneIndex parent = bones[i].parent;
		if(parent == -1U)
		{
			mat = Mat4::translate(this->base);
		}
		else
		{
			mat = bind[parent] * Mat4::translate(bones[parent].extent);
		}
		bind.emplace_back(mat);
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;
}

std::vector< Mat4 > Skeleton::current_pose() const {
    //A4T2a: bone-to-skeleton transformations in the current pose
	std::vector< Mat4 > currentPose;
	for (size_t i = 0; i < bones.size(); i++)
	{
		Mat4 mat;
		BoneIndex parent = bones[i].parent;
		Vec3 axis[3];
		bones[i].compute_rotation_axes(&axis[0], &axis[1], &axis[2]);
		Mat4 R =  Mat4::angle_axis(bones[i].pose.z, axis[2]) * Mat4::angle_axis(bones[i].pose.y, axis[1]) * Mat4::angle_axis(bones[i].pose.x, axis[0]);
		if(parent == -1U)
		{
			mat = Mat4::translate(this->base + this->base_offset) * R;
		}
		else
		{
			mat = currentPose[parent] * Mat4::translate(bones[parent].extent) * R;
		}
		currentPose.emplace_back(mat);
	}

	return currentPose;
}

std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
    //A4T2b: IK gradient

    // Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});

	//TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	//remember bone.compute_rotation_axes() -- should be useful here, too!
	std::vector< Mat4 > bindPose = bind_pose();
	std::vector< Mat4 > currentPose = current_pose();

	for (const Handle& handle : handles)
	{
		if(!handle.enabled)
			continue;
		BoneIndex tipIndex = handle.bone;
		const Bone &tipBone = bones[tipIndex];

		Vec3 p = currentPose[tipIndex] * tipBone.extent;
		Vec3 err = p - handle.target;
		
		BoneIndex b = tipIndex;

		while (b != -1U)
		{
			const Bone &bone = bones[b];
			BoneIndex parent = bone.parent;
			Mat4 xf_x, xf_y, xf_z, mat_t;

			if(parent == -1U)
				mat_t = Mat4::translate(base + base_offset);
			else
				mat_t = currentPose[parent] * Mat4::translate(bones[parent].extent);
			
			xf_z = mat_t;
			xf_y = mat_t * Mat4::angle_axis(bone.pose.z, Vec3(0,0,1));
			xf_x = xf_y * Mat4::angle_axis(bone.pose.y, Vec3(0,1,0));

			Vec3 r = currentPose[b] * Vec3(0,0,0);

			Vec3 x_WS = xf_x.rotate(Vec3(1, 0, 0));
			Vec3 y_WS = xf_y.rotate(Vec3(0, 1, 0));
			Vec3 z_WS = xf_z.rotate(Vec3(0, 0, 1));
			
			Vec3 x = cross(x_WS, p - r);
			Vec3 y = cross(y_WS, p - r);
			Vec3 z = cross(z_WS, p - r);

			gradient[b].x += dot(x, err);
			gradient[b].y += dot(y, err);
			gradient[b].z += dot(z, err);
			
			b = bone.parent;
		}
	}
	
	assert(gradient.size() == bones.size());
	
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//check which handles are enabled
	//run `steps` iterations
	float stepSize = 0.05f;
	float min_err = 1e-6f;
	for (uint32_t i = 0; i < steps; i++)
	{
		float loss = 0.0f;
		for (const Handle& handle : handles)
		{
			if(handle.enabled)
			{
				Vec3 p = current_pose()[handle.bone] * bones[handle.bone].extent;
				Vec3 h = handle.target;
				loss += 0.5f * (p - h).norm_squared();
			}
		}
		if(loss <= min_err)
			return true;

		std::vector<Vec3> gradient = gradient_in_current_pose();
		bool isNearZero = true;
		for (size_t g = 0; g < gradient.size(); g++)
		{
			float norm = gradient[g].norm();
			if(norm > min_err)
				isNearZero = false;
		}

		if(isNearZero)
			return true;
		
		for (size_t j = 0; j < bones.size(); j++)
		{
			bones[j].pose -= stepSize * gradient[j];
		}
	}
	
	//call gradient_in_current_pose() to compute d loss / d pose
	//add ...

	//if at a local minimum (e.g., gradient is near-zero), return 'true'.
	//if run through all steps, return `false`.
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)
	Vec3 ab = b - a;
	Vec3 ap = p - a;
	float d = dot(ab, ab);
	float t = dot(ap, ab) / d;
	if(t <= 0.0f)
		return a;
	else if(t >= 1.0f)
		return b;
	else
    	return a + t * ab;
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)
	std::vector< Mat4 > bind = bind_pose();
	for(auto &vertex : mesh.vertices)
	{
		Vec3 pos = vertex.position;
		float totalWeight = 0.0f;
		std::vector< Halfedge_Mesh::Vertex::Bone_Weight > newBoneWeight;
		for (uint32_t i = 0; i < bones.size(); i++)
		{
			Vec3 a = bind[i] * Vec3(0,0,0);
			Vec3 b = bind[i] * bones[i].extent;

			Vec3 p = closest_point_on_line_segment(a, b, pos);
			float distance = (pos - p).norm();

			float w_ij = std::max(0.0f, (bones[i].radius - distance) / bones[i].radius); 

			totalWeight += w_ij;

			Halfedge_Mesh::Vertex::Bone_Weight boneWeight;
			boneWeight.bone = i;
			boneWeight.weight = w_ij;
			newBoneWeight.emplace_back(boneWeight);
		}

		if(totalWeight == 0.0f)
			continue;
		else
		{
			vertex.bone_weights.clear();

			for (auto& boneWeight : newBoneWeight)
			{
				boneWeight.weight /= totalWeight;
				vertex.bone_weights.emplace_back(boneWeight);
			}

		}
	}
}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());
	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector< Mat4 > bind_to_current;
	bind_to_current.reserve(bind.size());
	for (size_t i = 0; i < bind.size(); i++)
	{
		bind_to_current.emplace_back(current[i] * bind[i].inverse());
	}

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) 
	{
		//NOTE: vertices with empty bone_weights should remain in place.
		if(vi->bone_weights.empty())
		{
			skinned_positions.emplace(vi, vi->position);
		}
		else
		{
			Vec3 blend = Vec3(0.0f, 0.0f, 0.0f);

			for (auto boneWeight : vi->bone_weights)
			{
				blend += boneWeight.weight * (bind_to_current[boneWeight.bone] * vi->position);
			}

			skinned_positions.emplace(vi, blend);
		}

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted
			if(h->face->boundary)
			{
				h = h->twin->next;
				continue;
			}

			Vec3 normal = h->corner_normal;
			if(vi->bone_weights.empty())
				skinned_normals.emplace(h, normal.normalize());
			else
			{
				Vec3 blend_N = Vec3(0.0f, 0.0f, 0.0f);
				for (auto boneWeight : vi->bone_weights)
				{
					Mat4 R = Mat4::transpose(bind_to_current[boneWeight.bone].inverse());
					blend_N += boneWeight.weight * R.rotate(normal);
				}

				skinned_normals.emplace(h, blend_N.normalize());
			}

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	Indexed_Mesh result; 
	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> idxs;
	for (Halfedge_Mesh::FaceCRef f = mesh.faces.begin(); f != mesh.faces.end(); f++) 
	{
		if (f->boundary) 
			continue;

		//every corner gets its own copy of a vertex:
		uint32_t corners_begin = static_cast<uint32_t>(verts.size());
		Halfedge_Mesh::HalfedgeCRef h = f->halfedge;
		do {
			Indexed_Mesh::Vert v;
			v.pos = skinned_positions.at(h->vertex);
			v.norm = skinned_normals.at(h);
			v.uv = h->corner_uv;
			v.id = f->id;
			
			verts.emplace_back(v);
			h = h->next;
		} while (h != f->halfedge);
		uint32_t corners_end = static_cast<uint32_t>(verts.size());

		//divide face into a triangle fan:
		for (size_t i = corners_begin + 1; i + 1 < corners_end; i++) 
		{
			idxs.emplace_back(corners_begin);
			idxs.emplace_back(static_cast<uint32_t>(i));
			idxs.emplace_back(static_cast<uint32_t>(i+1));
		}
	}
	result.vertices() = std::move(verts);
	result.indices() = std::move(idxs);
	return result;
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
