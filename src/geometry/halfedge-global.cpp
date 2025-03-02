#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>

#include <iostream>

// for projected the vertex to 2D plane
enum TargetCoordinate
{
	XY,
	YZ,
	ZX
};

/*
 * triangulate: split all non-boundary faces into triangles.
 *
 * Works on all valid meshes.
 */
void Halfedge_Mesh::triangulate() {
	//A2G1: triangulation
	auto Vec3ToVec2 = [](Vec3 pos, TargetCoordinate tagCoordinate)->Vec2
	{
		switch (tagCoordinate)
		{
			case XY:
				return Vec2(pos.x, pos.y);
			case YZ:
				return Vec2(pos.y, pos.z);
			case ZX:
				return Vec2(pos.z, pos.x);
			default:
				return Vec2(pos.x, pos.y);
		}
	};

	auto ConvexFunc = [](Vec2 pre, Vec2 next, Vec2 t, bool isCounterwise)->bool
	{
		// result > 0 -> convex
		float result = (t.x - pre.x)*(next.y - t.y) - (next.x - t.x)*(t.y - pre.y);
		return (isCounterwise ? (result > 0) : (result < 0));
	};

	auto isInTriangle = [](Vec2 p, Vec2 a, Vec2 b, Vec2 c)->bool
	{
		// Barycentric Edge function
		float alpha = (c.x - b.x)*(p.y - b.y) - (p.x - b.x)*(c.y - b.y);
		float beta = (a.x - c.x)*(p.y - c.y) - (p.x - c.x)*(a.y - c.y);
		float gamma =(b.x - a.x)*(p.y - a.y) - (p.x - a.x)*(b.y - a.y);

		return (alpha > 0 && beta > 0 && gamma > 0) || (alpha < 0 && beta < 0 && gamma < 0);
	};
	
	auto DeleteEar = [&](FaceRef oldFace, HalfedgeRef h_Ear, HalfedgeRef h_Ear_pre, HalfedgeRef h_Ear_pre_pre)
	{
		// Counter Clockwise
		FaceRef f_new = emplace_face();
		HalfedgeRef h = emplace_halfedge();
		HalfedgeRef t = emplace_halfedge();
		EdgeRef e = emplace_edge();
		e->halfedge = h;
		h->edge = e;
		t->edge = e;
		f_new->halfedge = h;
		h->twin = t;
		t->twin = h;
		h->face = f_new;
		t->face = oldFace;
		oldFace->halfedge = t;

		// Assign vertex
		h->vertex = h_Ear->next->vertex;
		t->vertex = h_Ear_pre->vertex;
		// Connect
		t->next = h_Ear->next;
		h_Ear->next = h;
		h->next = h_Ear_pre;
		h_Ear_pre_pre->next = t;

		// Assign New Face
		HalfedgeRef startH = h;
		do
		{
			startH->face = f_new;
			startH = startH->next;
		} while (startH != h);
	};

	auto isEar = [&](std::vector<HalfedgeRef>::iterator it, TargetCoordinate tc, std::vector<HalfedgeRef>& halfedgesLoop, bool isCounterwise)->bool
	{
		auto it_pre = (it == halfedgesLoop.begin()) ? std::prev(halfedgesLoop.end()) : it - 1;
		auto it_next = (std::next(it) == halfedgesLoop.end()) ? halfedgesLoop.begin() : std::next(it);
		VertexRef pre = (*it_pre)->vertex;
		VertexRef next = (*it_next)->vertex;
		
		VertexRef t = (*it)->vertex;
		Vec2 pre_Pos = Vec3ToVec2(pre->position, tc);
		Vec2 next_Pos = Vec3ToVec2	(next->position, tc);
		Vec2 t_Pos = Vec3ToVec2(t->position, tc);

		if (ConvexFunc(pre_Pos, next_Pos, t_Pos, isCounterwise))
		{
			for (auto itX = halfedgesLoop.begin(); itX != halfedgesLoop.end(); ++itX)
			{
				VertexRef v_p = (*itX)->vertex;
				if (isInTriangle(Vec3ToVec2(v_p->position, tc), pre_Pos, next_Pos, t_Pos))
				{
					return false;
				}
			}
			return true;
		}
		return false;
	};

	auto EarClipping = [&](FaceRef f)
	{
		HalfedgeRef h = f->halfedge;
		std::vector<HalfedgeRef> halfedgesLoop;

		// 1. get all halfedges
		HalfedgeRef startH = h;
		do
		{
			halfedgesLoop.emplace_back(startH);
			startH = startH->next;
		} while (startH != h);

		if (halfedgesLoop.size() == 3)
			return;

		// 2. decide which plane to project
		Vec3 normal = f->normal();
		float nX = abs(normal.x);
		float nY = abs(normal.y);
		float nZ = abs(normal.z);
		bool isCounterwise = true;
		TargetCoordinate tc = XY;
		if(nX >= nY && nX >= nZ)
		{
			tc = YZ;
			isCounterwise = normal.x > 0;
		}
		else if(nY >= nZ && nY >= nX)
		{
			tc = ZX;
			isCounterwise = normal.y > 0;
		}
		else
			isCounterwise = normal.z > 0;

		// 3. find all ears
		std::vector<std::vector<HalfedgeRef>::iterator> iterators_Ear;
		for (auto it = halfedgesLoop.begin(); it != halfedgesLoop.end(); ++it)
		{
			if (isEar(it, tc, halfedgesLoop, isCounterwise))
				iterators_Ear.emplace_back(it);
		}
		// 4. delete ears
		while (halfedgesLoop.size() > 3)
		{
			auto i_ear = iterators_Ear.begin();
			auto it = *i_ear;
			auto it_pre = (it == halfedgesLoop.begin()) ? halfedgesLoop.end() - 1 : it - 1;
			auto it_next = (std::next(it) == halfedgesLoop.end()) ? halfedgesLoop.begin() : std::next(it);

			HalfedgeRef targetH = *it;
			HalfedgeRef targetH_pre = *it_pre;
			HalfedgeRef targetH_pre_pre = (it_pre == halfedgesLoop.begin()) ? *(halfedgesLoop.end() - 1) : *(it_pre - 1);

			DeleteEar(f, targetH, targetH_pre, targetH_pre_pre);

			halfedgesLoop.erase(it);
			iterators_Ear.erase(i_ear);
			if(halfedgesLoop.size() > 3)
			{
				// 4.2 check nearby two vertices
				if (isEar(it_pre, tc, halfedgesLoop, isCounterwise))
					iterators_Ear.emplace_back(it_pre);
				if (isEar(it_next, tc, halfedgesLoop, isCounterwise))
					iterators_Ear.emplace_back(it_next);
			}
		}
	};

	for (FaceRef f = faces.begin(); f != faces.end(); ++f)
	{
		if(f->boundary)
			continue;
		
		// Caculate the total number of vertex
		HalfedgeRef h = f->halfedge;
		int num = 0;
		HalfedgeRef startH = h;
		do
		{
			startH = startH->next;
			num++;
		}while(startH != h);
		if(num == 3)
			continue;
		
		EarClipping(f);
	}
}

/*
 * linear_subdivide: split faces into quads without moving anything.
 *
 * Works on all valid meshes.
 *
 * (NOTE: uses catmark_subdivide_helper for subdivision)
 */
void Halfedge_Mesh::linear_subdivide() {
	std::unordered_map< VertexCRef, Vec3 > vertex_positions;
	std::unordered_map< EdgeCRef, Vec3 > edge_vertex_positions;
	std::unordered_map< FaceCRef, Vec3 > face_vertex_positions;

	//A2G2: linear subdivision

	// For every vertex, assign its current position to vertex_positions[v]:
	for (VertexCRef v = vertices.begin(); v != vertices.end(); v++)
	{
		vertex_positions.insert({v, v->position});
	}

	//(TODO)

    // For every edge, assign the midpoint of its adjacent vertices to edge_vertex_positions[e]:
	// (you may wish to investigate the helper functions of Halfedge_Mesh::Edge)
	for (EdgeCRef e = edges.begin(); e != edges.end(); e++)
	{
		edge_vertex_positions.insert({e, e->center()}) ;
	}
	

	//(TODO)

    // For every *non-boundary* face, assign the centroid (i.e., arithmetic mean) to face_vertex_positions[f]:
	// (you may wish to investigate the helper functions of Halfedge_Mesh::Face)
	for (FaceCRef f = faces.begin(); f != faces.end(); f++)
	{
		if(!f->boundary)
			face_vertex_positions.insert({f, f->center()});
	}

	//(TODO)
	

	//use the helper function to actually perform the subdivision:
	catmark_subdivide_helper(vertex_positions, edge_vertex_positions, face_vertex_positions);
}

/*
 * catmark_subdivide: split faces into quads with positions calculated by
 *   the Catmull-Clark ruleset.
 *
 * Works on all valid meshes.
 *
 * (NOTE: uses catmark_subdivide_helper for subdivision)
 */
void Halfedge_Mesh::catmark_subdivide() {
	std::unordered_map< VertexCRef, Vec3 > vertex_positions;
	std::unordered_map< EdgeCRef, Vec3 > edge_vertex_positions;
	std::unordered_map< FaceCRef, Vec3 > face_vertex_positions;

	//A2G3: Catmull-Clark Subdivision

	// This routine should end up looking a lot like linear_subdivide
	// above, with the exception that the positions are a bit trickier
	// to compute.

	//Overview of the rules:
	// https://en.wikipedia.org/wiki/Catmull%E2%80%93Clark_subdivision_surface

	// Faces
	for (FaceCRef f = faces.begin(); f != faces.end(); f++)
	{
		if(!f->boundary)
			face_vertex_positions.insert({f, f->center()});
	}
	// Edges
	for (EdgeCRef e = edges.begin(); e != edges.end(); e++)
	{
		Vec3 edgePos;
		edgePos = e->center();
		if(!e->on_boundary())
			edgePos = (edgePos * 2 + e->halfedge->face->center() + e->halfedge->twin->face->center()) / 4; 
		edge_vertex_positions.insert({e, edgePos});
	}
	// Vertices
	for (VertexCRef v = vertices.begin(); v != vertices.end(); v++)
	{
		Vec3 S = v->position;
		Vec3 v_Pos;
		HalfedgeRef h = v->halfedge;
		// n: vertex degree
		float n = static_cast<float>(v->degree());
		if(v->on_boundary())
		{
			Vec3 v_next = h->twin->vertex->position;
			HalfedgeRef toH = h;
			do
			{
				toH = toH->next;
			} while (toH->next != h);
			Vec3 v_pre = toH->vertex->position;
			v_Pos = (v_pre + v_next) / 8.0f + S * 0.75f;
		}
		else
		{
			// Q: average of face coords around vertex
			// R: average of edge coords around vertex
			HalfedgeRef startH = h;
			Vec3 Q;
			Vec3 R;
			int count = 0;
			do
			{
				Q += startH->face->center();
				R += startH->edge->center();
				count ++;
				startH= startH->twin->next;
			} while (startH != h);
			Q /= (float)count;
			R /= (float)count;

			v_Pos = (Q + 2*R + (n-3)* S) / n;

		}
		vertex_positions.insert({v, v_Pos});
	}
	
	
	//Now, use the provided helper function to actually perform the subdivision:
	catmark_subdivide_helper(vertex_positions, edge_vertex_positions, face_vertex_positions);

}

/*
 * loop_subdivide: sub-divide non-boundary faces with the Loop subdivision rule
 * 
 * If all non-boundary faces are triangles:
 *   subdivides mesh using the Loop subdivision rule
 *   returns true
 * Otherwise:
 *   does not change mesh
 *   returns false
 *
 * Do note that this requires a working implementation of edge split and edge flip
 */
bool Halfedge_Mesh::loop_subdivide() {
	//preamble: check for any non-triangular non-boundary faces:
	for (FaceCRef f = faces.begin(); f != faces.end(); ++f) {
		if (f->boundary) continue; //ignore boundary faces for this check
		if (f->halfedge->next->next->next != f->halfedge) 
		{
			//found a non-triangular face!
			return false;
		}
	}

	//if execution reaches this point, all non-boundary faces are triangular, so proceed to subdivide:

	// A2Go1: Loop subdivision.
	// Optional! Only one of {A2Go1, A2Go2, A2Go3} is required!

	// Each vertex and edge of the original mesh can be associated with a
	// vertex in the new (subdivided) mesh. Therefore, our strategy for
	// computing the subdivided vertex locations is to *first* compute the
	// new positions using the connectivity of the original (coarse) mesh.
	// Navigating this mesh will be much easier than navigating the new
	// subdivided (fine) mesh, which has more elements to traverse.  We
	// will then assign vertex positions in the new mesh based on the
	// values we computed for the original mesh.
    
	// Compute new positions for all the vertices in the input mesh using
	// the Loop subdivision rule and store them in vertex_new_pos.
	[[maybe_unused]]
	std::unordered_map< VertexRef, Vec3 > vertex_new_pos;
	for (VertexRef v = vertices.begin(); v != vertices.end(); v++)
	{
		HalfedgeRef h = v->halfedge;
		int num = 0;
		Vec3 v_old = v->position;
		Vec3 v_new;
		Vec3 neighbor;
		bool isBoundaryVertex = false;
		do
		{
			if(h->edge->on_boundary())
				isBoundaryVertex = true;
			neighbor += h->twin->vertex->position;
			h = h->twin->next;
			num++;
		} while (h != v->halfedge);

		if(isBoundaryVertex)
		{
			neighbor = Vec3(0.0f, 0.0f, 0.0f);
			HalfedgeRef h1 = h;
			do
			{
				if(h1->edge->on_boundary())
					neighbor += h1->twin->vertex->position;
				h1 = h1->twin->next;
			} while (h1 != h);
			
			v_new = 0.125f * neighbor + 0.75f * v_old;
		}
		else
		{
			if(num == 3)
				v_new = (1.0f - (float)num * 3.0f / 16.0f) * v_old + 3.0f / 16.0f * neighbor;
			else
				v_new = 5.0f / 8.0f * v_old + 3.0f / (8.0f * (float)num) * neighbor;

		}

		vertex_new_pos.insert({v, v_new});
	}
	    
	// Next, compute the subdivided vertex positions associated with edges, and
	// store them in edge_new_pos:
	[[maybe_unused]]
	std::unordered_map< EdgeRef, Vec3 > edge_new_pos;
    for (EdgeRef e = edges.begin(); e != edges.end(); e++)
	{
		Vec3 newV;
		Vec3 A = e->halfedge->vertex->position;
		Vec3 B = e->halfedge->twin->vertex->position;
		if(e->on_boundary())
		{
			newV = (A + B) / 2.0f;
		}
		else
		{
			Vec3 C = e->halfedge->next->next->vertex->position;
			Vec3 D = e->halfedge->twin->next->next->vertex->position;
			newV = 0.375f * (A + B) + 0.125f * (C + D);
		}
		edge_new_pos.insert({e, newV});
	}
	// Next, we're going to split every edge in the mesh, in any order, placing
	// the split vertex at the recorded edge_new_pos.
	//
	// We'll later need to distinguish edges that align with old edges to new
	// edges added by splitting. So store references to the new edges:
	[[maybe_unused]]
	std::vector< EdgeRef > new_edges;
	std::vector<EdgeRef> originalEdges;
	for (EdgeRef e = edges.begin(); e != edges.end(); ++e) 
	{
		originalEdges.push_back(e);
	}
	
	for (EdgeRef e : originalEdges)
	{
		VertexRef newV = *split_edge(e);
		if (edge_new_pos.count(e)) 
		{
			newV->position = edge_new_pos[e];
		}
		if(!newV->on_boundary())
			new_edges.emplace_back(newV->halfedge->twin->next->edge);

		new_edges.emplace_back(newV->halfedge->next->next->edge);
	}

	// Also note that in this loop, we only want to iterate over edges of the
	// original mesh. Otherwise, we'll end up splitting edges that we just split
	// (and the loop will never end!)

	// Now flip any new edge that connects a new and old vertex.
	// To check if a vertex is new, you can use a simple helper that
	// checks if has an entry in vertex_new_pos:
	[[maybe_unused]]
	auto is_new = [&vertex_new_pos](VertexRef v) -> bool {
		return !vertex_new_pos.count(v);
	};

    // Now flip any new edge that connects an old and new vertex.
	for (size_t i = 0; i < new_edges.size(); i++)
	{
		VertexRef v0 = new_edges[i]->halfedge->vertex;
		VertexRef v1 = new_edges[i]->halfedge->twin->vertex;

		if(!is_new(v0) && is_new(v1))
			flip_edge(new_edges[i]);
	}
    
    // Finally, copy new vertex positions into the Vertex::position.
	for (VertexRef v = vertices.begin(); v != vertices.end(); v++)
	{
		auto it = vertex_new_pos.find(v);
		if(it!= vertex_new_pos.end())
			v->position = it->second;
	}
	return true;
}

//isotropic_remesh: improves mesh quality through local operations.
// Do note that this requires a working implementation of EdgeSplit, EdgeFlip, and EdgeCollapse
void Halfedge_Mesh::isotropic_remesh(Isotropic_Remesh_Parameters const &params) {

	//A2Go2: Isotropic Remeshing
	// Optional! Only one of {A2Go1, A2Go2, A2Go3} is required!

	// Compute the mean edge length. This will be the "target length".

    // Repeat the four main steps for `outer_iterations` iterations:

    // -> Split edges much longer than the target length.
	//     ("much longer" means > target length * params.longer_factor)

    // -> Collapse edges much shorter than the target length.
	//     ("much shorter" means < target length * params.shorter_factor)

    // -> Flip each edge if it improves vertex degree.

    // -> Finally, apply some tangential smoothing to the vertex positions.
	//     This means move every vertex in the plane of its normal,
	//     toward the centroid of its neighbors, by params.smoothing_step of
	//     the total distance (so, smoothing_step of 1 would move all the way,
	//     smoothing_step of 0 would not move).
	// -> Repeat the tangential smoothing part params.smoothing_iterations times.

	//NOTE: many of the steps in this function will be modifying the element
	//      lists they are looping over. Take care to avoid use-after-free
	//      or infinite-loop problems.

}

struct Edge_Record {
	Edge_Record() {
	}
	Edge_Record(std::unordered_map<uint32_t, Mat4>& VQ, Halfedge_Mesh::EdgeRef e) : edge(e) {
		
		// Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
	}
	Halfedge_Mesh::EdgeRef edge;
	Vec3 optimal;
	float score;
};

bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
	if (r1.score != r2.score) {
		return (r1.score < r2.score);
	}
	Halfedge_Mesh::EdgeRef e1 = r1.edge;
	Halfedge_Mesh::EdgeRef e2 = r2.edge;
	return &*e1 < &*e2;
}

template<class T> struct MutablePriorityQueue {
	void insert(const T& item) {
		queue.insert(item);
	}
	void remove(const T& item) {
		if (queue.find(item) != queue.end()) {
			queue.erase(item);
		}
	}
	const T& top() const {
		return *(queue.begin());
	}
	void pop() {
		queue.erase(queue.begin());
	}
	size_t size() {
		return queue.size();
	}

	std::set<T> queue;
};

/*
 * simplify: reduce edge count through collapses
 *  ratio: proportion of original faces to retain
 *
 * you may choose to have your implementation work only on triangle meshes,
 *  in which case it may return 'false' if there are non-triangular
 *  non-boundary faces
 *
 * returns false if it ran out of edges to collapse
 * returns true otherwise
 * 
 * Do note that this requires a working implementation of EdgeCollapse
 */
bool Halfedge_Mesh::simplify(float ratio) {

	//A2Go3: simplification
	// Optional! Only one of {A2Go1, A2Go2, A2Go3} is required!

	std::unordered_map<uint32_t, Mat4> face_quadrics;
	std::unordered_map<uint32_t, Mat4> vertex_quadrics;
	std::unordered_map<uint32_t, Edge_Record> edge_records;
	MutablePriorityQueue<Edge_Record> queue;

	// Compute initial quadrics for each face by writing the plane equation for
    // the face in homogeneous coordinates. These quadrics should be stored in
    // face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above MutablePriorityQueue<Edge_Record> for this.
    // -> Until reaching the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    return false;
}

/*
 * catmark_subdivide_helper: add vertex in every edge and non-boundary face, set positions from parameters
 *
 * Works on all valid meshes.
 */
void Halfedge_Mesh::catmark_subdivide_helper(
	std::unordered_map< VertexCRef, Vec3 > const &vertex_positions, //positions for vertices after subdivision
	std::unordered_map< EdgeCRef, Vec3 > const &edge_vertex_positions, //positions for new vertices added in each edge
	std::unordered_map< FaceCRef, Vec3 > const &face_vertex_positions //positions for new vertices added in each face
	) {

	//check that positions were supplied for every vertex:
	for (VertexCRef v = vertices.begin(); v != vertices.end(); ++v) {
		if (!vertex_positions.count(v)) {
			throw std::runtime_error("No vertex position supplied for vertex with id " + std::to_string(v->id) + ".");
		}
	}

	//check that positions were supplied for every edge:
	for (EdgeCRef e = edges.begin(); e != edges.end(); ++e) {
		if (!edge_vertex_positions.count(e)) {
			throw std::runtime_error("No edge vertex position supplied for edge with id " + std::to_string(e->id) + ".");
		}
	}

	//check that positions were supplied for every (non-boundary) face:
	for (FaceCRef f = faces.begin(); f != faces.end(); ++f) {
		if (f->boundary) {
			if (face_vertex_positions.count(f)) {
				throw std::runtime_error("Extraneous vertex position was supplied for boundary face with id " + std::to_string(f->id) + ".");
			}
		} else {
			if (!face_vertex_positions.count(f)) {
				throw std::runtime_error("No vertex position supplied for face with id " + std::to_string(f->id) + ".");
			}
		}
	}


	{ //check that mesh is in a valid state to start with:
		auto error = validate();
		if (error) {
			throw std::runtime_error("catmark_subdivide_helper called on invalid mesh: " + error.value().second);
		}
	}

	if (vertices.empty() || edges.empty() || faces.empty()) {
		//empty mesh must be empty:
		assert(vertices.empty() && edges.empty() && faces.empty());
		return;
	}

	//store the old last vertex, face, and edge to allow iterating over only the old elements later:
	//(this works because the emplace_* functions add to the end of the element lists)
	VertexRef last_old_vertex = std::prev(vertices.end());
	EdgeRef last_old_edge = std::prev(edges.end());
	FaceRef last_old_face = std::prev(faces.end());
	
	//(can't store .end() iterators because emplace_back puts things "before the end")

	//------------------------
	//split every edge:
	//old halfedges stay connected to their vertices
	//old edge stays connected to e->halfedge->vertex

	//before:
	//     -----h---->
	//  v1 -----e----- v2
	//     <----t-----
	//after:
	//     --h->    --h2->
	//  v1 --e-- vm --e2-- v2
	//     <-t2-    <--t--

	for (EdgeRef e = edges.begin(); e != std::next(last_old_edge); ++e) {
		HalfedgeRef h = e->halfedge;
		HalfedgeRef t = h->twin; assert(t->edge == e);
		VertexRef v1 = h->vertex;
		VertexRef v2 = t->vertex;

		//new elements:
		VertexRef vm = emplace_vertex();
		HalfedgeRef h2 = emplace_halfedge();
		HalfedgeRef t2 = emplace_halfedge();
		EdgeRef e2 = emplace_edge(e->sharp);

		//middle vertex:
		vm->halfedge = h2; //could also use t2
		vm->position = edge_vertex_positions.at(e);
		interpolate_data({v1, v2}, vm);

		//second edge:
		e2->halfedge = h2;

		//second halfedge:
		h2->next = h->next;
		h2->twin = t;
		h2->vertex = vm;
		h2->edge = e2;
		h2->face = h->face;
		interpolate_data({h, h->next}, h2);

		//second twin halfedge:
		t2->next = t->next;
		t2->twin = h;
		t2->vertex = vm;
		t2->edge = e;
		t2->face = t->face;
		interpolate_data({t, t->next}, t2);

		//fix up pointers for existing halfedges:
		h->next = h2;
		h->twin = t2;

		t->next = t2;
		t->twin = h2;
		t->edge = e2;
	}


	//---------------------------
	//split (non-boundary) faces:

	//before:
	//
	//  v0 <-h7- v7 <-h6- v6
	//  |                 ^
	//  h0                h5
	//  v                 |
	//  v1       f        v5
	//  |                 ^
	//  h1                h4
	//  v                 |
	//  v2 -h2-> v3 -h3-> v4
	//
	//after:
	//  v0 <-h7- v7 <-h6- v6
	//  |        |        ^
	//  h0   f   e3   f3  h5
	//  v  --c-> |        |
	//  v1 --e0- vm --e2- v5 
	//  |  <-t-- |        ^
	//  h1   f1  e1  f2   h4
	//  v        |        |
	//  v2 -h2-> v3 -h3-> v4
	//
	// (each new eN has new halfedges as you'd expect,
	//  with eN->halfedge being directed toward the central vertex.)

	for (FaceRef f = faces.begin(); f != std::next(last_old_face); ++f) {
		if (f->boundary) continue; //skip boundary faces

		//get face halfedges:
		std::vector< HalfedgeRef > face_halfedges;
		{
			HalfedgeRef h = f->halfedge;
			do {
				face_halfedges.emplace_back(h);
				h = h->next;
			} while (h != f->halfedge);
			assert(face_halfedges.size() % 2 == 0); //should always be pairs of halfedges along subdivided edges!
		}

		//get face vertices and corners to interpolate data from:
		// (skip the odd vertices/halfedges -- they were just added)
		std::vector< HalfedgeCRef > from_corners;
		std::vector< VertexCRef > from_vertices;
		for (uint32_t i = 0; i < face_halfedges.size(); i += 2) {
			from_corners.emplace_back(face_halfedges[i]);
			from_vertices.emplace_back(face_halfedges[i]->vertex);
		}

		//add central vertex:
		VertexRef vm = emplace_vertex();
		vm->position = face_vertex_positions.at(f);
		interpolate_data(from_vertices, vm);

		//add halfedges and edges around the central vertex:
		std::vector< EdgeRef > inner_edges;
		for (uint32_t i = 0; i + 1 < face_halfedges.size(); i += 2) {
			EdgeRef e = emplace_edge(false);
			HalfedgeRef c = emplace_halfedge();
			HalfedgeRef t = emplace_halfedge();

			e->halfedge = c;

			//halfedge coming from the side:
			c->twin = t;
			//c->next will be set later
			c->vertex = face_halfedges[i+1]->vertex;
			c->edge = e;
			//c->face will be set later
			interpolate_data({face_halfedges[i+1]}, c); //just copy the data

			//halfedge coming from the center:
			t->twin = c;
			//t->next will be set later
			t->vertex = vm;
			t->edge = e;
			//t->face will be set later
			interpolate_data(from_corners, t);

			if (i == 0) vm->halfedge = t;

			//save edge for later connection:
			inner_edges.emplace_back(e);
		}

		//hook up pointers for all the quads:
		for (uint32_t i = 0; i + 1 < face_halfedges.size(); i += 2) {
			HalfedgeRef h0 = face_halfedges[i];
			HalfedgeRef h1 = inner_edges.at(i/2)->halfedge;
			HalfedgeRef h2 = inner_edges.at((i/2 == 0 ? inner_edges.size()-1 : i/2-1))->halfedge->twin;
			HalfedgeRef h3 = face_halfedges[(i == 0 ? face_halfedges.size()-1 : i-1)];

			//connect halfedges around the face:
			h0->next = h1;
			h1->next = h2;
			h2->next = h3;
			assert(h3->next == h0); //already connected and part of the face

			//connect halfedges to the face:
			if (i == 0) {
				//first face re-uses f:
				assert(f->halfedge == h0);
				assert(h0->face == f);
				h1->face = f;
				h2->face = f;
				assert(h3->face == f);
			} else {
				//other faces made fresh:
				FaceRef n = emplace_face(false);
				n->halfedge = h0;
				h0->face = n;
				h1->face = n;
				h2->face = n;
				h3->face = n;
			}
		}

	}

	//--------------------------
	//update positions for vertices
	for (VertexRef v = vertices.begin(); v != std::next(last_old_vertex); ++v) {
		v->position = vertex_positions.at(v);
	}

	{ //PARANOIA: sanity check:
		auto ret = validate();
		if (ret) {
			warn("After subdivide, validate says:\n  %s", ret.value().second.c_str());
		}
		assert(!ret && "subdivide helper should never break topology");
	}
}

/*
 * flip_orientation: flip direction of all halfedges
 *
 * works on all valid meshes.
 */
void Halfedge_Mesh::flip_orientation() {

	//store new h->vertex and v->halfedge pointers:
	std::unordered_map<Halfedge const *, VertexRef> he_to_v;
	std::unordered_map<Vertex const *, HalfedgeRef> v_to_he;
	for (auto &he : halfedges) {
		he_to_v[&he] = he.twin->vertex;
	}
	for (auto &v : vertices) {
		v_to_he[&v] = v.halfedge->twin;
	}

	//reverse all face loops:
	for (auto &face : faces) {
		//read off halfedges around face:
		std::vector<HalfedgeRef> hs;
		std::vector<Vec2> uvs;
		std::vector<Vec3> normals;

		HalfedgeRef h = face.halfedge;
		do {
			hs.emplace_back(h);
			uvs.emplace_back(h->corner_uv);
			normals.emplace_back(h->corner_normal);
			h = h->next;
		} while (h != face.halfedge);

		//reverse face ordering:
		for (uint32_t i = 0; i < hs.size(); ++i) {
			hs[(i+1)%hs.size()]->next = hs[i];
			hs[i]->corner_uv = uvs[(i+1)%hs.size()];
			hs[i]->corner_normal = normals[(i+1)%hs.size()];
		}
	}

	//update h->vertex and v->halfedge pointers:
	for (auto &he : halfedges) {
		he.vertex = he_to_v.at(&he);
	}
	for (auto &v : vertices) {
		v.halfedge = v_to_he.at(&v);
	}
}

/*
 * set_corner_normals: compute face-corner normals based on `sharp` flag and smoothing threshold
 *
 * works on all valid meshes.
 */
void Halfedge_Mesh::set_corner_normals(float threshold) {
	//first, figure out which edges to consider sharp for this operation:
	std::unordered_set< Edge const * > sharp_edges;
	sharp_edges.reserve(edges.size());

	//all edges between boundary and non-boundary get marked sharp regardless of mode:
	for (auto const &edge : edges) {
		if (edge.halfedge->face->boundary != edge.halfedge->twin->face->boundary) {
			sharp_edges.emplace(&edge);
		}
	}

	if (threshold >= 180.0f) {
		//"smooth mode" -- all other edges are considered smooth
	} else {
		//"flat mode" / "auto mode" -- any edges which are marked sharp or have face angle <= threshold get marked sharp:
		float cos_threshold = std::cos( Radians( std::clamp(threshold, 0.0f, 180.0f) ) );
		if (threshold <= 0.0f) cos_threshold = 2.0f; //make sure everything is sharp
		for (auto const &edge : edges) {
			//get adjacent halfedges:
			HalfedgeRef h1 = edge.halfedge;
			HalfedgeRef h2 = h1->twin;
			if (h1->face->boundary || h2->face->boundary) {
				//don't care about edges boundary-boundary, and inside-boundary already marked.
				//thus: nothing to do here
			} else if (edge.sharp) {
				//flagged as sharp, so mark it sharp:
				sharp_edges.emplace(&edge);
			} else {
				//inside-inside edge, non-marked, check angle:
				Vec3 n1 = h1->face->normal();
				Vec3 n2 = h2->face->normal();
				float cos = dot(n1,n2);
				if (cos <= cos_threshold) {
					//treat as sharp:
					sharp_edges.emplace(&edge);
				}
			}
		}
	}

	//clear current corner normals:
	for (auto h = halfedges.begin(); h != halfedges.end(); ++h) {
		h->corner_normal = Vec3{0.0f, 0.0f, 0.0f};
	}

	//now circulate all vertices to set normals:
	for (auto const &v : vertices) {
		//get halfedge leaving this vertex:
		HalfedgeRef begin = v.halfedge;
		assert(&*begin->vertex == &v);

		//circulate begin until it is at a sharp edge (thus, the next corner starts a smoothing group):
		do {
			if (sharp_edges.count(&*begin->edge)) break;
			begin = begin->twin->next;
		} while (begin != v.halfedge); //could be all one big happy smoothing group

		//store all corners around the vertex:
		struct Corner {
			HalfedgeRef in; //halfedge pointing to v
			HalfedgeRef out; //halfedge pointing away from v
			Vec3 weighted_normal; //face normal at corner, weighted... somehow (see below)
		};

		std::vector< std::vector< Corner > > groups;
		HalfedgeRef h = begin;
		do {
			//start a new smoothing group on sharp edges (or at the very first edge):
			if (h == begin || sharp_edges.count(&*h->edge)) {
				groups.emplace_back();
			}
			//add corner after h to current smoothing group:
			Corner corner;
			corner.in = h->twin;
			corner.out = h->twin->next;
			assert(corner.in->face == corner.out->face); //PARANOIA
			{ //compute some sort of weighted normal:
				assert(&*corner.in->vertex != &v);
				assert(&*corner.in->twin->vertex == &v);
				Vec3 from = corner.in->vertex->position - v.position;

				assert(&*corner.out->vertex == &v);
				assert(&*corner.out->twin->vertex != &v);
				Vec3 to = corner.out->twin->vertex->position - v.position;
				/*
				//basic area weighting (weird with non-flat faces and reflex vertices):
				corner.weighted_normal = cross(to - v.position, from - v.position);
				*/
				/*//sort sort of angle weighting thing -- this never works as well as one would hope:
				//...still needs work for reflex angles also
				float angle = std::atan2(cross(from,to).norm(), dot(from, to));
				corner.weighted_normal = angle * corner.in->face->normal();
				*/
				//some other sort of slightly fancy area weighting:
				corner.weighted_normal = cross(to - v.position, from - v.position).norm() * corner.in->face->normal();
			}
			groups.back().emplace_back(corner);

			//advance h:
			h = h->twin->next;
		} while (h != begin);

		//compute weighted normals per-corner:
		for (auto const &group : groups) {
			assert(!group.empty());
			if (group[0].in->face->boundary) {
				//boundary group.
				//PARANOIA:
				for (auto const &corner : group) {
					assert(corner.in->face->boundary);
				}
				//no need for normals on boundary corners
				continue;
			}
			//compute weighted normal:
			Vec3 sum = Vec3{0.0f, 0.0f, 0.0f};
			for (auto const &corner : group) {
				sum += corner.weighted_normal;
			}
			//normalize:
			sum = sum.unit();
			//assign to all corners in group:
			for (auto const &corner : group) {
				assert(&*corner.out->vertex == &v);
				corner.out->corner_normal = sum;
			}
		}
	}

	//normals computed!
}

/*
 * set_corner_uvs_per_face: set uv coordinates to map texture per-face
 */
void Halfedge_Mesh::set_corner_uvs_per_face() {
	//clear existing UVs:
	for (auto &halfedge : halfedges) {
		halfedge.corner_uv = Vec2(0.0f, 0.0f);
	}
	
	//set UVs per-face:
	for (auto const &face : faces) {
		if (face.boundary) continue;

		//come up with a plane perpendicular-ish to the face:
		Vec3 n = face.normal();
		Vec3 p1;
		if (std::abs(n.x) < std::abs(n.y) && std::abs(n.x) < std::abs(n.z)) {
			p1 = Vec3(1.0f, 0.0f, 0.0f);
		} else if (std::abs(n.y) < std::abs(n.z)) {
			p1 = Vec3(0.0f, 1.0f, 0.0f);
		} else {
			p1 = Vec3(0.0f, 0.0f, 1.0f);
		}
		p1 = (p1 - dot(p1, n) * n).unit();
		Vec3 p2 = cross(n, p1);

		//find bounds of face on plane:
		Vec2 min = Vec2(std::numeric_limits< float >::infinity(), std::numeric_limits< float >::infinity());
		Vec2 max = Vec2(-std::numeric_limits< float >::infinity(), -std::numeric_limits< float >::infinity());
		HalfedgeRef v = face.halfedge;
		do {
			Vec2 pt = Vec2(dot(p1, v->vertex->position), dot(p2, v->vertex->position));
			min = hmin(min, pt);
			max = hmax(max, pt);
			v = v->next;
		} while (v != face.halfedge);

		//set corner uvs based on position within bounds:
		do {
			Vec2 pt = Vec2(dot(p1, v->vertex->position), dot(p2, v->vertex->position));
			v->corner_uv = Vec2(
				(pt.x - min.x) / (max.x - min.x),
				(pt.y - min.y) / (max.y - min.y)
			);
			v = v->next;
		} while (v != face.halfedge);
	}
}

/*
 * set_corner_uvs_project: set uv coordinates to map texture by projection to a plane
 */
void Halfedge_Mesh::set_corner_uvs_project(Vec3 origin, Vec3 u_axis, Vec3 v_axis) {

	u_axis /= u_axis.norm_squared();
	v_axis /= v_axis.norm_squared();

	for (auto &halfedge : halfedges) {
		if (halfedge.face->boundary) {
			halfedge.corner_uv = Vec2(0.0f, 0.0f);
		} else {
			halfedge.corner_uv = Vec2(
				dot(halfedge.vertex->position - origin, u_axis),
				dot(halfedge.vertex->position - origin, v_axis)
			);
		}
	}
}
