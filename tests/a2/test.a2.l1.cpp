#include "test.h"
#include "geometry/halfedge.h"

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.flip_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_basic_simple("a2.l1.flip_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 2}, 
		{0, 2, 1}
	});

	expect_flip(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_edge_boundary("a2.l1.flip_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

// Kim Test case
/*
  2
 / \
0---1
 \ /
  3

Flip edge 0-1

After mesh:
  2
 /|\
0 | 1
 \|/
  3
 */

 Test test_a2_l1_flip_doubletriangle("a2.l1.flip_edge.edge.doubletriangle", []() {
    Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, -1.0f, 0.0f)
    }, {
        {0, 1, 2},
        {0, 3, 1}
    });

    Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

    Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, -1.0f, 0.0f)
    }, {
        {0, 3, 2},
        {3, 1, 2}
    });

    expect_flip(mesh, edge, after);

});


/*
  - 2
 / / \
4-0---1
 \ \ /
  - 3

Flip edge 0-1

After mesh:
  - 2
 / /|\
4-0 | 1
 \ \|/
  - 3
 */

Test test_a2_l1_flip_embedded_doubletriangle("a2.l1.flip_edge.edge.embedded_doubletriangle", []() {
    Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, -1.0f, 0.0f), Vec3(-2.0f, 0.0f, 0.0f)
    }, {
        {0, 1, 2},
        {0, 3, 1},
        {2, 4, 3, 0}
    });

    Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

    Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, -1.0f, 0.0f), Vec3(-2.0f, 0.0f, 0.0f)
    }, {
        {0, 3, 2},
        {3, 1, 2},
        {2, 4, 3, 0}
    });

    expect_flip(mesh, edge, after);

});

// Nellie Test case
/*
EDGE CASE

Initial mesh:
0--1--2
|  |  |
|  3  |
|  |  |
4--5--6

Flip Edge on Edge: 1-3

After mesh:
0--1--2
|  |  |
|  3  |
|  |  |
4--5--6
*/
Test test_a2_l1_flip_edge_orphaned_vertex_1("a2.l1.flip_edge.orphaned_vertex.1", []() {
    Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 1.1f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
                        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(-1.3f,-0.7f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
    }, {
        {0, 4, 5, 3, 1}, 
        {1, 3, 5, 6, 2}
    });
    Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.flip_edge(edge)) {
        throw Test::error("flip_edge should not work at the boundary.");
    }
});

/*
EDGE CASE

Initial Mesh:
  /0\
 / | \
1  2  3
 \ | /
  \4/

Flip Edge on Edge: 0-2

After Mesh:
  /0\
 / | \
1  2  3
 \ | /
  \4/
*/
Test test_a2_l1_flip_edge_orphaned_vertex_2("a2.l1.flip_edge.orphaned_vertex.2", []() {
    Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
                        Vec3(0.0f, 1.0f, 0.0f), 
        Vec3(-1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f),
                        Vec3(0.0f, -1.0f, 0.0f)
    }, {
        {0, 1, 4, 2}, 
        {0, 2, 4, 3}
    });
    Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.flip_edge(edge)) {
        throw Test::error("flip_edge should not work at the boundary.");
    }
});

/*
EDGE CASE

Initial Mesh:
0--1--2
|  |  |
|  3  |
|  |  |
|  4  |
|  |  |
5--6--7

Flip Edge on Edge: 3-4

After Mesh:
0--1--2
|  |  |
|  3  |
|  |  |
|  4  |
|  |  |
5--6--7
*/
Test test_a2_l1_flip_edge_orphaned_vertex_3("a2.l1.flip_edge.orphaned_vertex.3", []() {
    Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
        Vec3(-1.0f, 1.1f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
                        Vec3(0.0f, 0.0f, 0.0f),
                        Vec3(0.35f, -0.4f, 0.0f),
        Vec3(-1.3f,-1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
    }, {
        {0, 5, 6, 4, 3, 1}, 
        {1, 3, 4, 6, 7, 2}
    });
    Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.flip_edge(edge)) {
        throw Test::error("flip_edge should not work at the boundary.");
    }
});

// /*
// EDGE CASE

// NOTICE:two faces are not at the same plane

// Initial mesh:
// 0--1--2
// |  |  |
// |  |  |
// |  |  |
// 3--4--5

// Flip Edge on Edge: 1-4

// After mesh:
// 0--1--2
// |  |  |
// |  |  |
// |  |  |
// 3--4--5
// */
// Test test_a2_l1_flip_edge_surfaces_on_the_same_plane("a2.l1.flip_edge.surfaces_on_the_same_plane", []() {
//     Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
//         Vec3(-1.0f, 1.0f, -0.5f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, -0.5f),
                        
//         Vec3(-1.0f,-1.0f, -0.5f), Vec3(0.0f, -1.0f, 0.0f), Vec3(1.0f, -1.0f, -0.5f)
//     }, {
//         {0, 3, 4, 1}, 
//         {1, 4, 5, 2}
//     });
//     Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

// if (mesh.flip_edge(edge)) {
//         throw Test::error("flip_edge should not work when faces where h and t are located are not on the same plane.");
//     }
// });

