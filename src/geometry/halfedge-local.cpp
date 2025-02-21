
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	// Collect data
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->next->vertex;
	VertexRef v2 = t->next->vertex;
	VertexRef v3 = h->next->next->vertex;
	VertexRef v4 = t->next->next->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	HalfedgeRef toH_Old = h;
	f1->halfedge = h;
	f2->halfedge = t;
	do
	{
		toH_Old = toH_Old->next;
	} while (toH_Old-> next != h);
	HalfedgeRef toT_Old = t;
	do
	{
		toT_Old = toT_Old->next;
	} while (toT_Old-> next != t);

	// Step 1: new Vertex
	// Creat new vertex
	VertexRef newV = emplace_vertex();
	// Caculate new position
	newV->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, newV);

	// Step 2 : Create two halfedges and one edge for broken h & t
	EdgeRef t_h_half_edge = emplace_edge();
	HalfedgeRef t_half = emplace_halfedge();
	HalfedgeRef h_half = emplace_halfedge();
	t_half->twin = h_half;
	h_half->twin = t_half;
	t_h_half_edge->halfedge = h_half;
	t_half->edge = t_h_half_edge;
	h_half->edge = t_h_half_edge;
	// Step 2.1 : Update h 、 h_half、t_half vertex
	h->vertex = newV;
	h_half->vertex = v2;
	t_half->vertex = newV;
	v2->halfedge = h_half;
	// Step 2.2 : Update h_half、t_half next*
	h_half->next = h;
	t_half->next = t->next;
	// Step2.3 : Connect other halfedge to new halfedge
	t->next = t_half;
	toH_Old->next = h_half;
	newV->halfedge = h;
	t_half->face = t->face;

	auto ConnectTwoVertex = [&](VertexRef vTarget, HalfedgeRef h1, HalfedgeRef h2, bool isHside, FaceRef oldFace)
	{
		EdgeRef Target_edge_newV = emplace_edge();
		HalfedgeRef target_newV = emplace_halfedge();
		HalfedgeRef newV_target = emplace_halfedge();
		target_newV->twin = newV_target;
		newV_target->twin = target_newV;
		Target_edge_newV->halfedge = target_newV;
		target_newV->edge = Target_edge_newV;
		newV_target->edge = Target_edge_newV;
		// Step 3.2 Update v3_newV 、 newV_v3 vertex
		target_newV->vertex = vTarget;
		newV_target->vertex = newV;
		// Step 3.3 : Update v3_newV、newV_v3、h_half h->next next*
		target_newV->next = h1;
		newV_target->next = h1->next->next;
		// Step 3.4 : Connect other halfedge 
		h2->next = newV_target;
		h1->next->next = target_newV;
		// Step 3.5 : Assign new face
		FaceRef newFace = emplace_face();
		newFace->halfedge = isHside ? newV_target : target_newV;
		HalfedgeRef startH = isHside ? newV_target : target_newV;
		do
		{
			startH->face = newFace;
			startH = startH->next;
		} while (startH != (isHside ? newV_target : target_newV));
		startH = isHside ? h1 : h2;
		do
		{
			startH->face = oldFace;
			startH = startH->next;
		} while (startH != (isHside ? h1 : h2));
	};
	// Step 3: Create two pairs halfedges for v3
	if(!f1->boundary)
	{
		ConnectTwoVertex(v3, h, h_half, true, f1);
	}

	// Step 4: Create two pairs halfedges for v4
	if(!f2->boundary)
	{
		ConnectTwoVertex(v4, t_half, t, false, f2);
	}
    return newV;
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Special case : boundary face is invalid.
	if(f->boundary)
		return std::nullopt;

	std::vector<HalfedgeRef> faceHalfedge_old;
	std::vector<VertexRef> faceVertices_old;
	std::vector<VertexRef> faceVertices_new;

	auto AssignFaceToHalfedge = [&](HalfedgeRef h, FaceRef f)
	{
		HalfedgeRef startH = h;
		f->halfedge = h;
		do
		{
			h->face = f;
			h = h->next;
		} while (startH != h);
	};

	// collect data
	HalfedgeRef h1 = f->halfedge;
	do
	{
		faceHalfedge_old.emplace_back(h1);
		faceVertices_old.emplace_back(h1->vertex);
		h1 = h1->next;
	} while (h1 != f->halfedge);

	// Create new vertices
	for (int i = 0; i < static_cast<int>(faceVertices_old.size()); i++)
	{
		VertexRef newVertex = emplace_vertex();
		newVertex->position = faceVertices_old[i]->position;
		interpolate_data({faceVertices_old[i]}, newVertex);
		faceVertices_new.emplace_back(newVertex);
	}

	// Create halfedges of new vertices in inner face
	std::vector<HalfedgeRef> twinHalfedges;
	std::vector<HalfedgeRef> halfHalfedges;
	for (int i = 0; i < static_cast<int>(faceVertices_new.size()); i++)
	{
		HalfedgeRef newTwin = emplace_halfedge();
		HalfedgeRef newHalf = emplace_halfedge();
		newTwin->vertex = faceVertices_new[i];
		newHalf->vertex = faceVertices_new[i];
		halfHalfedges.emplace_back(newHalf);
		twinHalfedges.emplace_back(newTwin);
		faceVertices_new[i]->halfedge = newHalf;
	}
	// Connect new halfedges with each other & create edges
	for (int i = 0; i <  static_cast<int>(twinHalfedges.size()); i++)
	{
		EdgeRef newEdge = emplace_edge();
		halfHalfedges[i]->next = halfHalfedges[(i + 1) > (static_cast<int>(halfHalfedges.size()) - 1) ? 0 : (i + 1)];
		halfHalfedges[i]->twin = twinHalfedges[(i + 1) > (static_cast<int>(twinHalfedges.size()) - 1) ? 0 : (i + 1)];
		halfHalfedges[i]->edge = newEdge;
		halfHalfedges[i]->twin->edge = newEdge;
		newEdge->halfedge = halfHalfedges[i];

		twinHalfedges[i]->next = twinHalfedges[(i - 1) < 0 ? (static_cast<int>(twinHalfedges.size()) - 1) : (i - 1)];
		twinHalfedges[i]->twin = halfHalfedges[(i - 1) < 0 ? (static_cast<int>(halfHalfedges.size()) - 1) : (i - 1)];
	}
	// Connect old halfedges to new halfedges(also have to create new edges and halfedges)
	for (int i = 0; i <  static_cast<int>(twinHalfedges.size()); i++)
	{
		EdgeRef newEdge = emplace_edge();
		HalfedgeRef h = emplace_halfedge();
		HalfedgeRef t = emplace_halfedge();
		h->edge = newEdge;
		t->edge = newEdge;
		h->twin = t;
		t->twin = h;
		newEdge->halfedge = h;

		faceHalfedge_old[i]->next = h;
		h->next = twinHalfedges[(i + 1) > (static_cast<int>(twinHalfedges.size()) - 1) ? 0 : (i + 1)];
		h->vertex = faceHalfedge_old[(i + 1) > (static_cast<int>(twinHalfedges.size()) - 1) ? 0 : (i + 1)]->vertex;
		twinHalfedges[(i + 2)%(twinHalfedges.size())]->next = t;
		t->next = faceHalfedge_old[(i + 1) > (static_cast<int>(twinHalfedges.size()) - 1) ? 0 : (i + 1)];
		t->vertex = faceVertices_new[(i + 1) > ( static_cast<int>(twinHalfedges.size() - 1)) ? 0 : (i + 1)];
	}
	// Assign Face
	AssignFaceToHalfedge(halfHalfedges[0], f);
	for (int i = 0; i <  static_cast<int>(faceHalfedge_old.size()); i++)
	{
		FaceRef newFace = emplace_face();
		AssignFaceToHalfedge(faceHalfedge_old[i], newFace);
	}

	// Remember to also fill in extrude_helper (A2L4h)

    return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	// Boundry edge
	if(e->on_boundary())
    	return std::nullopt;
	else
	{
		// Base on slide
		// Collect data
		HalfedgeRef h = e->halfedge;
		HalfedgeRef t = h->twin;
		VertexRef v1 = h->next->vertex;
		VertexRef v2 = t->next->vertex;
		VertexRef v3 = h->next->next->vertex;
		VertexRef v4 = t->next->next->vertex;
		FaceRef f1 = h->face;
		FaceRef f2 = t->face;

		// special case 1: v3 connect to v4
		if(v3->halfedge->next->vertex == v4)
			return std::nullopt;
		HalfedgeRef toH = h;
		HalfedgeRef toT = t;
		do
		{
			toH = toH->next;
		}while(toH->next != h);do
		{
			toT = toT->next;
		}while(toT->next != t);
		// special case 2: v1/v2 only connect to two edges
		if(toH->twin == t->next || toT->twin == h->next)
			return std::nullopt;
		// special case 3: h and t are not on the same plane
		if(f1->normal() != f2->normal())
			return std::nullopt;
		
		// halfedge->next == h || t
		toH->next = t->next;		// instead of point to h, replace it with t->next
		toT->next = h->next;		// instead of point to t, replace it with h->next

		// disconnect
		v1->halfedge = h->next;
		v2->halfedge = t->next;
		f1->halfedge = h;
		f2->halfedge = t;

		// connect
		t->vertex = v3;
		h->vertex = v4;
		h->next = v3->halfedge;
		t->next = v4->halfedge;

		// update face
		v1->halfedge->face = t->face;
		v2->halfedge->face = h->face;
		// update loop
		v1->halfedge->next = t;
		v2->halfedge->next = h;
		return e;
	}
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge
	// Collect data

	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	HalfedgeRef toH = h;
	HalfedgeRef toT = t;
	VertexRef v0 = h->vertex;
	VertexRef v1 = t->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	do
	{
		toH = toH->next;
	} while (toH-> next != h);
	do
	{
		toT = toT->next;
	} while (toT-> next != t);

	// Special case : single triangle
	if(h->next->twin == toT && t->next->twin == toH)
		return std::nullopt;
	
	// Special case : non manifold
	if(h->next->twin->face->boundary && t->next->twin->face->boundary && toT->twin->face->boundary && toH->twin->face->boundary)
		return std::nullopt;

	
	// method combine h->vertex which mean we have to reconnect t->vertex side data
	// Step 1: Move vertex h
	v0->position = (v0->position + v1->position) / 2.0f;
	interpolate_data({v0, v1}, v0);

	//Step 2 : Disconnect other halfedge
	toT->next = t->next;
	toH->next = h->next;
	h->next->vertex = v0;
	h->next->twin->next->vertex = v0;
	toT->twin->vertex = v0;
	f1->halfedge = h->next;
	f2->halfedge = t->next;
	v0->halfedge = h->next;

	auto CheckEraseFace = [&](FaceRef fDelete, VertexRef vDelete)
	{
		HalfedgeRef targetH = fDelete->halfedge;
		if(fDelete->degree() < 3)
		{
			// which mean omly two edges to construct a triangle
			HalfedgeRef targetTwin = targetH->twin;
	 		HalfedgeRef targetNextTwin = targetH->next->twin;
			VertexRef v3 = targetH->next->vertex;
			EdgeRef eDelete = targetH->edge;
			// make two pairs into one pair
			targetNextTwin->twin = targetTwin;
			targetTwin->twin = targetNextTwin;
			// assign another edge
			targetTwin->edge = targetNextTwin->edge;
			targetNextTwin->edge->halfedge = targetTwin;
			// assign vertex with new halfedge
			vDelete->halfedge = vDelete->halfedge->next->twin;
			v3->halfedge = targetTwin;

			// erase 
			erase_halfedge(targetH->next);
			erase_halfedge(targetH);
			erase_edge(eDelete);
			erase_face(fDelete);
		}
	};
	
	CheckEraseFace(f1, v0);
	CheckEraseFace(f2, v1);

	// Last Step erase h\t\e
	erase_edge(e);
	erase_halfedge(h);
	erase_halfedge(t);
	erase_vertex(v1);


	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
    return v0;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper
	// clamp
	shrink = shrink > 1.0f ? 1.0f : shrink;
	// Collect all vertices
	std::vector<VertexRef> faceVertices;
	HalfedgeRef startHalfedge = face->halfedge;
	do
	{
		faceVertices.emplace_back(startHalfedge->vertex);
		startHalfedge = startHalfedge->next;
	}while(startHalfedge != face->halfedge);

	face->center() += move;
	Vec3 center = face->center();
	for (int i = 0; i <  static_cast<int>(faceVertices.size()); i++)
	{
		// Step 1 Move
		faceVertices[i]->position += move;
		// Step 2 Shrink
		faceVertices[i]->position = faceVertices[i]->position * (1.0f - shrink) + shrink * center;
	}
	
	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	
}

