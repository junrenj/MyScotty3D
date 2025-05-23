#include "test.h"
#include "geometry/halfedge.h"

static void expect_split(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.split_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("split_edge rejected operation!");
	}
}

/*
BASIC CASE:

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split Edge on Edge: 1-4

After mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/
*/
Test test_a2_l2_split_edge_basic_simple("a2.l2.split_edge.basic.simple", []() {
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
		                         Vec3(1.25f, 0.0f, 0.0f),  Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 4, 5, 2}, 
		{0, 2, 1}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_split(mesh, edge, after);
});

/*
EDGE CASE: 

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split Edge on Edge: 0-1

After mesh:
0--1--2\
|  /  | \
| /   |  3
|/    | /
4-----5/
*/
Test test_a2_l2_split_edge_edge_boundary("a2.l2.split_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f),  Vec3(0.05f, 1.05f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            						Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), 							Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 4, 1}, 
		{1, 4, 5, 2}, 
		{2, 5, 3}
	});

	expect_split(mesh, edge, after);
});


// test case from Anonymous Atom 

/*

Initial Mesh:

0---1---2
|   |   |
|   |   | 
|   |   |    
3---4---5   

split 1-4

After Mesh:

0---1---2
| \ |   |
|   6   | 
|   | \ |    
3---4---5   

*/




Test test_a2_l2_split_center("a2.l2.collapse_edge.split.center", []() {

    Halfedge_Mesh before = Halfedge_Mesh::from_indexed_faces({
        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f),
Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f)
    }, {
        {3,4,1,0},
        {1,4,5,2},
    }); 

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f),
Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f),
Vec3(0.5f, 0.5f, 0.0f)
    }, {
    {1,0,6},
    {0,3,4,6},
    {6,4,5},
    {6,5,2,1}
    });

    Halfedge_Mesh::EdgeRef edge = before.halfedges.begin()->next->edge;
expect_split(before, edge, after);
});

/*

0---1
|   |
|   |
2---3
0---1
| \ 4
|   |
2---3

split 1-3

*/

Test test_a2_l2_split_edge("a2.l2.collapse_edge.split.edge", []() {

    Halfedge_Mesh before = Halfedge_Mesh::from_indexed_faces({

        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f),
Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f)

    }, {
        {0,2,3,1},
    }); 

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f),Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f), 
Vec3(0.5f, 0.5f, 0.0f)
   }, {
        {0,4,1},
	{0,2,3,4}
    });

    Halfedge_Mesh::EdgeRef edge = before.halfedges.begin()->next->next->edge;
expect_split(before, edge, after);
});

/*

0---1
|   |
|   |
2---3

0---1
|   |
4 \ |
2---3

split 0-2

*/

Test test_a2_l2_split_edge_rotated("a2.l2.collapse_edge.split.edge.rotated", []() {
   Halfedge_Mesh before = Halfedge_Mesh::from_indexed_faces({
        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f),Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f)
    }, {
        {0,2,3,1},
    }); 

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
        Vec3(0.0f, 1.0f, 0.0f), Vec3(0.5f, 1.0f, 0.0f),Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f), 
Vec3(0.0f, 0.5f, 0.0f)
    }, {
        {0,4,3,1},
	{4,2,3}
    });

    Halfedge_Mesh::EdgeRef edge = before.halfedges.begin()->next->next->next->next->edge;
expect_split(before, edge, after);
});