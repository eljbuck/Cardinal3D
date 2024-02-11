
#include <queue>
#include <set>
#include <unordered_map>
#include <iostream>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    Check if a triangle is degenerate based on vertex positions around a face
*/
bool Halfedge_Mesh::is_degenerate(Halfedge_Mesh::FaceRef f) {
    
    HalfedgeRef h0 = f->halfedge();
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h0->next()->vertex();
    VertexRef v2 = h0->next()->next()->vertex();

    std::cout << "v0 pos: " <<  v0->pos << std::endl;
    std::cout << "v1 pos: " <<  v1->pos << std::endl;
    std::cout << "v2 pos: " <<  v2->pos << std::endl;

    return v0->pos == v1->pos || v1->pos == v2->pos || v2->pos == v0->pos;
}

bool Halfedge_Mesh::is_collapsible(EdgeRef e) {
    VertexRef v0 = e->halfedge()->vertex();
    VertexRef v1 = e->halfedge()->twin()->vertex();

    std::set<VertexRef> seen_vertices =  std::set<VertexRef>();

    // check which triangles are degenerate based on vertex positions
    HalfedgeRef h_temp = v1->halfedge();
    int count = 0;
    
    do {
        seen_vertices.insert(h_temp->twin()->vertex());

        // if (is_degenerate(h_temp->face())) {
        //     std::cout << "tri was degen, count: " << count << std::endl;
        // }


        h_temp = h_temp->twin()->next();  
    } while(h_temp != v1->halfedge());

    h_temp = v0->halfedge();
    
    do {
        if (seen_vertices.find(h_temp->twin()->vertex()) != seen_vertices.end()) {
            count++;
        }

        h_temp = h_temp->twin()->next();  
    } while(h_temp != v0->halfedge());

    return count <= 2;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    // boundary check:
    if (e->halfedge()->is_boundary() || e->halfedge()->twin()->is_boundary() || !is_collapsible(e)) {
        return std::nullopt;
    }

    // references
    EdgeRef e0 = e;
    Vec3 midpoint = e0->center();

    HalfedgeRef h0 = e0->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();
    HalfedgeRef h3 = h0->twin();
    HalfedgeRef h4 = h1->twin();
    HalfedgeRef h5 = h2->twin();
    HalfedgeRef h6 = h3->next();
    HalfedgeRef h7 = h6->next();
    HalfedgeRef h8 = h6->twin();
    HalfedgeRef h9 = h7->twin();

    EdgeRef e1 = h1->edge();
    EdgeRef e2 = h2->edge();
    EdgeRef e3 = h6->edge();
    EdgeRef e4 = h7->edge();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();
    VertexRef v2 = h2->vertex();
    VertexRef v3 = h7->vertex();

    FaceRef f0 = h0->face();
    FaceRef f1 = h3->face();

    // reassign
    h5->twin() = h4;
    h4->twin() = h5;
    h5->edge() = e2;
    h4->edge() = e2;
    h5->vertex() = v0;
    h4->vertex() = v2;

    e2->halfedge() = h5;
    v0->halfedge() = h5;
    v2->halfedge() = h4;

    h8->twin() = h9;
    h9->twin() = h8;
    h8->edge() = e3;
    h9->edge() = e3;
    h8->vertex() = v3;
    h9->vertex() = v0;

    e3->halfedge() = h8;
    v3->halfedge() = h8;

    // accumulate list of halfedges whose vertex is v0
    std::vector<HalfedgeRef> compromised_halfedges;
    HalfedgeRef cur = h1;
    // NOTE: currently only works on triangles
    do {
        cur = cur->twin()->next();
        cur->face()->halfedge() = cur;
        compromised_halfedges.push_back(cur);
    } while (cur != h9);

    // reassign compromised halfedges to vertex v1
    for (HalfedgeRef halfedge : compromised_halfedges) {
        halfedge->vertex() = v0;
    }

    v1->pos = v0->pos;
    v0->pos = midpoint;

    erase(h0);
    erase(h1);
    erase(h2);
    erase(h3);
    erase(h6);
    erase(h7);
    erase(e0);
    erase(e1);
    erase(e4);
    erase(f0);
    erase(f1);
    erase(v1);

    return v0;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

// void Halfedge_Mesh::set_halfedge(HalfedgeRef halfedge, EdgeRef edge, VertexRef vertex, HalfedgeRef next, HalfedgeRef twin, FaceRef face) {
//     halfedge->edge() = edge;
//     halfedge->vertex() = vertex;
//     halfedge->next() = next;
//     halfedge->twin() = twin;
//     halfedge->face() = face;
// }

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    // boundary check:
    if (e->halfedge()->is_boundary() || e->halfedge()->twin()->is_boundary()) {
        return std::nullopt;
    }
    HalfedgeRef h0 = e->halfedge();
    VertexRef v_twin = h0->vertex();
    VertexRef v = h0->twin()->vertex();
    VertexRef v_next = h0->next()->next()->vertex();
    VertexRef v_twin_next = h0->twin()->next()->next()->vertex();
    HalfedgeRef cur = h0;
    while (cur->next() != h0) {
        cur = cur->next();
    }
    HalfedgeRef prev = cur;
    cur = h0->twin();
    while (cur->next() != h0->twin()) {
        cur = cur->next();
    }
    HalfedgeRef prev_twin = cur;
    HalfedgeRef next = h0->next();
    HalfedgeRef next_twin = h0->twin()->next();
    // wire faces
    h0->face()->halfedge() = h0;
    h0->twin()->face()->halfedge() = h0->twin();
    next->face() = h0->twin()->face();
    next_twin->face() = h0->face();
    // wire vertices
    h0->vertex() = v_twin_next;
    h0->twin()->vertex() = v_next;
    v->halfedge() = next;
    v_twin->halfedge() = next_twin;
    // wire halfedges
    h0->next() = h0->next()->next();
    h0->twin()->next() = h0->twin()->next()->next();
    next->next() = h0->twin();
    next_twin->next() = h0;
    prev->next() = next_twin;
    prev_twin->next() = next;

    return e;
}

// wire inside of triangle (DOES NOT DO TWINS)
void Halfedge_Mesh::wire_triangle(FaceRef f, 
        EdgeRef e0, EdgeRef e1, EdgeRef e2, 
        VertexRef v0, VertexRef v1, VertexRef v2,
        HalfedgeRef h0, HalfedgeRef h1, HalfedgeRef h2) {

    //halfedges
    h0->next() = h1;
    h1->next() = h2;
    h2->next() = h0;

    wire_edge(h0, e0);
    wire_edge(h1, e1);
    wire_edge(h2, e2);

    wire_vertex(h0, v0);
    wire_vertex(h1, v1);
    wire_vertex(h2, v2);

    wire_face(h0, f);
    wire_face(h1, f);
    wire_face(h2, f);
}

void Halfedge_Mesh::wire_twins(HalfedgeRef a, HalfedgeRef b) {
    a->twin() = b;
    b->twin() = a;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    //references
    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();

    HalfedgeRef h3 = h0->twin();
    HalfedgeRef h4 = h3->next();
    HalfedgeRef h5 = h4->next();

    EdgeRef e0 = e;
    EdgeRef e1 = h1->edge();
    EdgeRef e2 = h2->edge();
    EdgeRef e3 = h4->edge();
    EdgeRef e4 = h5->edge();

    FaceRef f0 = h0->face();
    FaceRef f1 = h3->face();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();
    VertexRef v2 = h2->vertex();
    VertexRef v3 = h5->vertex();

    //allocation
    VertexRef m = new_vertex();
    
    EdgeRef e5 = new_edge();
    EdgeRef e6 = new_edge();
    EdgeRef e7 = new_edge();

    FaceRef f2 = new_face();
    FaceRef f3 = new_face();

    HalfedgeRef h6 = new_halfedge();
    HalfedgeRef h7 = new_halfedge();
    HalfedgeRef h8 = new_halfedge();
    HalfedgeRef h9 = new_halfedge();
    HalfedgeRef h10 = new_halfedge();
    HalfedgeRef h11 = new_halfedge();

    // position midpoint
    Vec3 midpoint = e->center();
    m->pos = midpoint;

    //wire triangles
    wire_triangle(f0, e0, e1, e5, m, v1, v2, h0, h1, h6); // top left in diagram
    wire_triangle(f1, e4, e0, e7, v3, v1, m, h5, h3, h11); // top right
    wire_triangle(f2, e6, e5, e2, v0, m, v2, h8, h7, h2); // bot left
    wire_triangle(f3, e3, e7, e6, v0, v3, m, h4, h10, h9); // bot right

    wire_twins(h6, h7);
    wire_twins(h11, h10);
    wire_twins(h9, h8);

    // guarantee that m's associated halfedge is on original mesh
    m->halfedge() = h0;

    return m;
}

/*
    The following three util function add some syntactic assistance to ensure we only need to worry about wiring halfedges
*/
void Halfedge_Mesh::wire_vertex(HalfedgeRef h, VertexRef v) {
    h->vertex() = v;
    v->halfedge() = h;
}

void Halfedge_Mesh::wire_edge(HalfedgeRef h, EdgeRef e) {
    h->edge() = e;
    e->halfedge() = h;
}

void Halfedge_Mesh::wire_face(HalfedgeRef h, FaceRef f) {
    h->face() = f;
    f->halfedge() = h;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // vectors of newly allocated elements
    std::vector<VertexRef> new_verts = std::vector<VertexRef>();
    std::vector<EdgeRef> new_edges = std::vector<EdgeRef>();
    std::vector<HalfedgeRef> new_outside_halfedges = std::vector<HalfedgeRef>();
    std::vector<HalfedgeRef> new_inside_halfedges = std::vector<HalfedgeRef>();

    //allocate new elements
    FaceRef new_f = new_face();
    for (int i = 0; i < (int) v->degree(); i++) {
        new_verts.push_back(new_vertex());
        new_edges.push_back(new_edge());
        new_outside_halfedges.push_back(new_halfedge());
        new_inside_halfedges.push_back(new_halfedge());
    }

    // wire stuff
    HalfedgeRef h = v->halfedge()->twin();

    int initial_degree = (int) v->degree();

    for (int i = 0; i < initial_degree; i++) {
        int next_i = (i + 1) % initial_degree;

        // place vert temporarily
        new_verts[i]->pos = h->edge()->center();

        // get ref to next halfedge
        HalfedgeRef h_next = h->next();

        // wire nexts for halfedges
        h->next() = new_outside_halfedges[i];
        new_outside_halfedges[i]->next() = h_next;
        new_inside_halfedges[i]->next() = new_inside_halfedges[next_i];

        // wire twins for new halfedges
        wire_twins(new_outside_halfedges[i], new_inside_halfedges[i]);

        // wire new edges
        wire_edge(new_outside_halfedges[i], new_edges[i]);
        wire_edge(new_inside_halfedges[i], new_edges[i]);

        // wire new faces
        wire_face(new_outside_halfedges[i], h->face());
        wire_face(new_inside_halfedges[i], new_f);

        // wire new verts
        wire_vertex(new_outside_halfedges[i], new_verts[i]);
        wire_vertex(new_inside_halfedges[i], new_verts[next_i]);
        wire_vertex(h_next, new_verts[next_i]);

        h = h_next->twin(); // advance to next face
    }

    for (int i = 0; i < (int) new_verts.size(); i++) {
        std::cout << "new vert: " << new_verts[i]->id() << std::endl;
        std::cout << "new vert halfedge vertex: " << new_verts[i]->halfedge()->vertex()->id() << std::endl;
    }

    // get rid of v
    erase(v);

    return new_f;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    std::vector<HalfedgeRef> original_halfedges = std::vector<HalfedgeRef>();
    std::vector<HalfedgeRef> top_face_halfedges = std::vector<HalfedgeRef>();
    std::vector<HalfedgeRef> new_left_halfedges = std::vector<HalfedgeRef>();
    std::vector<HalfedgeRef> new_top_halfedges = std::vector<HalfedgeRef>();
    std::vector<HalfedgeRef> new_right_halfedges = std::vector<HalfedgeRef>();

    std::vector<FaceRef> new_faces = std::vector<FaceRef>();
    std::vector<EdgeRef> top_face_edges = std::vector<EdgeRef>();
    std::vector<EdgeRef> outside_face_left_edges = std::vector<EdgeRef>();

    std::vector<VertexRef> original_verts = std::vector<VertexRef>();
    std::vector<VertexRef> new_verts = std::vector<VertexRef>();
    

    // get refs to original halfedges, original verts, add new verts at copied positions, add halfedges for top face
    auto h = f->halfedge();
    do {
        original_halfedges.push_back(h);
        original_verts.push_back(h->vertex());

        top_face_halfedges.push_back(new_halfedge());

        new_left_halfedges.push_back(new_halfedge());
        new_top_halfedges.push_back(new_halfedge());
        new_right_halfedges.push_back(new_halfedge());

        new_faces.push_back(new_face());
        top_face_edges.push_back(new_edge());
        outside_face_left_edges.push_back(new_edge());

        VertexRef new_vert = new_vertex();
        new_vert->pos = h->vertex()->pos;
        new_verts.push_back(new_vert);

        h = h->next();
    } while(h != f->halfedge());

    int n = original_verts.size(); // number of verticies in the original polygon

    
    for (int i = 0; i < n; i++) {
        int next_idx = (i+1) % n;

        // wire top face halfedges to each other and correct verts/face/edge
        top_face_halfedges[i]->next() = top_face_halfedges[next_idx];
        wire_vertex(top_face_halfedges[i], new_verts[i]);
        wire_face(top_face_halfedges[i], f);
        wire_edge(top_face_halfedges[i], top_face_edges[i]);

        // wire twins for top face halfedges
        wire_twins(top_face_halfedges[i], new_top_halfedges[i]);

        // wire left/right twins for outside faces
        wire_twins(new_right_halfedges[i], new_left_halfedges[next_idx]);

        // wire nexts for outside faces
        original_halfedges[i]->next() = new_right_halfedges[i];
        new_right_halfedges[i]->next() = new_top_halfedges[i];
        new_top_halfedges[i]->next() = new_left_halfedges[i];
        new_left_halfedges[i]->next() = original_halfedges[i];

        // wire faces for outside faces
        wire_face(original_halfedges[i], new_faces[i]);
        wire_face(new_right_halfedges[i], new_faces[i]);
        wire_face(new_top_halfedges[i], new_faces[i]);
        wire_face(new_left_halfedges[i], new_faces[i]);

        // wire edges for outside faces
        wire_edge(new_right_halfedges[i], outside_face_left_edges[next_idx]);
        wire_edge(new_top_halfedges[i], top_face_edges[i]);
        wire_edge(new_left_halfedges[i], outside_face_left_edges[i]);

        // wire vertices for outside faces
        wire_vertex(new_right_halfedges[i], original_verts[next_idx]);
        wire_vertex(new_top_halfedges[i], new_verts[next_idx]);
        wire_vertex(new_left_halfedges[i], new_verts[i]);
    }

    // Gave up on using a function (sadge)
    // remember we still need to allocated 8 extra edges, 4 more faces
    // then we need to wire the next()s for the outside faces, 
    // and also wire the edges, faces, and vertices up for those (RIPBOZO)

    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;

    
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;


}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

                                           

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;

    // set initial vertex positions
    for (int i = 0; i < (int) start_positions.size(); i++) {
        new_halfedges[i]->vertex()->pos = start_positions[i];
    }

    Vec3 normal = face->normal();
    Vec3 center = face->center();

    for (int i = 0; i < (int) start_positions.size(); i++) {
        // horizontal offset relative to face
        Vec3 to_center = center - start_positions[i];

        float to_center_mag = sqrt(to_center.x * to_center.x + to_center.y * to_center.y + to_center.z * to_center.z);

        new_halfedges[i]->vertex()->pos += to_center.unit() * clamp(tangent_offset, -std::numeric_limits<float>::max(), to_center_mag);
        // vertical offset relative to face
        new_halfedges[i]->vertex()->pos += normal * normal_offset;
    }
}

void Halfedge_Mesh::triangulate_face(FaceRef f0) {

    std::vector<HalfedgeRef> original_halfedges = std::vector<HalfedgeRef>();
    std::vector<EdgeRef> original_edges = std::vector<EdgeRef>();
    std::vector<VertexRef> original_verts = std::vector<VertexRef>();

    // get original halfedges, edges, and verts
    auto h = f0->halfedge();
    do {
        original_halfedges.push_back(h);
        original_edges.push_back(h->edge());
        original_verts.push_back(h->vertex());

        h = h->next();
    } while(h != f0->halfedge());

    int degree = (int) f0->degree();

    // get faces
    std::vector<FaceRef> faces = std::vector<FaceRef>();

    faces.push_back(f0);
    for (int i = 0; i < degree - 3; i++) {
        faces.push_back(new_face());
    }

    // get edges
    std::vector<EdgeRef> right_edges = std::vector<EdgeRef>();    

    right_edges.push_back(original_edges[0]);
    for (int i = 0; i < degree - 3; i++) {
        right_edges.push_back(new_edge());
    }

    std::vector<EdgeRef> left_edges = std::vector<EdgeRef>();  

    for (int i = 0; i < degree - 3; i++) {
        left_edges.push_back(right_edges[i + 1]);
    }
    left_edges.push_back(original_edges[(int) original_edges.size() - 1]);

    std::vector<EdgeRef> top_edges = std::vector<EdgeRef>();  

    for (int i = 0; i < degree - 2; i++) {
        top_edges.push_back(original_edges[i + 1]);
    }

    //get vertices
    std::vector<VertexRef> right_verts = std::vector<VertexRef>();

    for (int i = 0; i < degree - 2; i++) {
        right_verts.push_back(original_verts[i + 1]);
    }

    std::vector<VertexRef> left_verts = std::vector<VertexRef>();

    for (int i = 0; i < degree - 2; i++) {
        left_verts.push_back(original_verts[i + 2]);
    }

    //get halfedges
    std::vector<HalfedgeRef> right_halfedges = std::vector<HalfedgeRef>();    

    right_halfedges.push_back(original_halfedges[0]);
    for (int i = 0; i < degree - 3; i++) {
        right_halfedges.push_back(new_halfedge());
    }

    std::vector<HalfedgeRef> left_halfedges = std::vector<HalfedgeRef>();  

    for (int i = 0; i < degree - 3; i++) {
        left_halfedges.push_back(new_halfedge());
    }
    left_halfedges.push_back(original_halfedges[original_halfedges.size() - 1]);

    std::vector<HalfedgeRef> top_halfedges = std::vector<HalfedgeRef>();  

    for (int i = 0; i < degree - 2; i++) {
        top_halfedges.push_back(original_halfedges[i + 1]);
    }

    //wire triangles
    for (int i = 0; i < degree - 2; i++) {
        FaceRef f = faces[i];

        EdgeRef e0 = right_edges[i];
        EdgeRef e1 = top_edges[i];
        EdgeRef e2 = left_edges[i];

        VertexRef v0 = original_verts[0];
        VertexRef v1 = right_verts[i];
        VertexRef v2 = left_verts[i];

        HalfedgeRef h0 = right_halfedges[i];
        HalfedgeRef h1 = top_halfedges[i];
        HalfedgeRef h2 = left_halfedges[i];

        wire_triangle(f, e0, e1, e2, v0, v1, v2, h0, h1, h2);
    }

    //wire twins
    for (int i = 0; i < degree - 3; i++) {
        wire_twins(left_halfedges[i], right_halfedges[i+1]);
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // loop over faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        if (f->degree() > 3) {
            triangulate_face(f);
        }
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {
    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }
    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->new_pos = e->center();
    }
    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        e->new_pos = ((h0->face()->center() + h0->twin()->face()->center()) / 2 + e->center()) / 2;
    }
    // Vertices
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        Vec3 Q; 
        Vec3 R;
        Vec3 S = v->pos;
        float n = 0.;
        HalfedgeRef h = v->halfedge();
        do {
            HalfedgeRef h_twin = h->twin();
            Q += h_twin->face()->center();
            R += h_twin->edge()->center();
            n++;

            h = h_twin->next();
        } while(h != v->halfedge());

        Q /= n;
        R /= n;

        v->new_pos = (Q + 2 * R + (n - 3) * S) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {
    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        // flag as old vertex
        v->is_new = false;

        // compute n and u
        float n = v->degree();
        float u;
        if (n == 3) {
            u = 3/16;
        } else { 
            u = 3 / (8 * n);
        }

        // compute sum of neighbor positions
        Vec3 sum;
        HalfedgeRef h = v->halfedge();
        do {
            sum += h->twin()->vertex()->pos;
            h = h->twin()->next();
        } while (h != v->halfedge());

        // compute new position
        v->new_pos = (1 - n * u) * v->pos + u * sum;
    }
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        VertexRef A = h0->vertex();
        VertexRef B = h0->twin()->vertex();
        VertexRef C = h0->next()->next()->vertex();
        VertexRef D = h0->twin()->next()->next()->vertex();
        e->new_pos = (3.0 /8) * (A->pos + B->pos) + (1.0 /8) * (C->pos + D->pos);
        e->is_new = false;
    }
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    int n = n_edges();
    EdgeRef e = edges_begin();
    for (int i = 0; i < n; i++) {

        Vec3 new_pos = e->new_pos;
        // get the next edge NOW!
        EdgeRef nextEdge = e;
        nextEdge++;

        if (!e->is_new) {
            // split edge
            VertexRef new_vertex = split_edge(e).value();
            // set new vertex properties
            new_vertex->is_new = true;
            new_vertex->new_pos = new_pos;
            // set new edge properties
            HalfedgeRef h0 = new_vertex->halfedge();
            h0->next()->next()->edge()->is_new = true;
            h0->twin()->next()->edge()->is_new = true;
            h0->edge()->is_new = false;
            h0->twin()->next()->twin()->next()->edge()->is_new = false;

        }

        e = nextEdge;
    }
    // -> Now flip any new edge that connects an old and new vertex.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if (e->is_new) {
            HalfedgeRef h0 = e->halfedge();
            VertexRef v0 = h0->vertex();
            VertexRef v1 = h0->twin()->vertex();
            if (v0->is_new != v1->is_new) {
                flip_edge(e);
            }
        }
    }
    // -> Finally, copy the new vertex positions into final Vertex::pos.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->pos = v->new_pos;
    }
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e) : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.

        Halfedge_Mesh::HalfedgeRef h = e->halfedge();
        Halfedge_Mesh::VertexRef v0 = h->vertex();
        Halfedge_Mesh::VertexRef v1 = h->twin()->vertex();

        Mat4 K_v0 = vertex_quadrics[v0];
        Mat4 K_v1 = vertex_quadrics[v1];

        Mat4 K = K_v0 + K_v1;

        Vec3 B = -K.cols[3].xyz();

        Vec4 column_0 = Vec4(K.cols[0].xyz(), 0);
        Vec4 column_1 = Vec4(K.cols[1].xyz(), 0);
        Vec4 column_2 = Vec4(K.cols[2].xyz(), 0);
        Vec4 column_3 = Vec4(0, 0, 0, 1);

        Mat4 A = Mat4(column_0, column_1, column_2, column_3);

        Vec3 X;
        Mat4 A_inv = A.inverse();
        if (std::isnan(A_inv.cols[0][0])) {
            std::cout << "an edge had an uninvertible matrix A" << std::endl;
            X = v0->pos;
        } else {
            X = (A_inv * Vec4(B, 1)).xyz();
        }

        optimal = X;
        cost = dot(Vec4(X,1), K * Vec4(X,1));
        edge = e;
        
        // edge = e;
        // optimal = edge->center();
        // cost = 0.0;
    }

    Halfedge_Mesh::EdgeRef edge;

    Vec3 optimal;

    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    T operator[] (int idx) {
        return *std::next(queue.begin(), idx);
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Return true if mesh simplification was successful down to 1/4 original num of triangles, false otherwise.
   */
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // get face quadrics
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        
        Vec3 P = f->center();
        Vec3 N = f->normal();

        Vec4 V = Vec4(N, -dot(N, P));
        Mat4 quadric = outer(V, V);

        face_quadrics[f] = quadric;
    }

    // get vertex quadrics
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {

        HalfedgeRef h = v->halfedge();
        vertex_quadrics[v] = Mat4().Zero;

        do {
            vertex_quadrics[v] += face_quadrics[h->face()];
            h = h->twin()->next();
        } while (h != v->halfedge());
    }

    // get edge records
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {

        edge_records[e] = Edge_Record(vertex_quadrics, e);
        edge_queue.insert(edge_records[e]);
    }

    // must have at least 16 faces to simplify, otherwise the resulting mesh will have less than 4 faces, meaning it's degenerate
    if (faces.size() < 16) {
        return false;
    }

    int target = faces.size() / 4;

    // loop while not enough faces have been simplified
    while ((int) faces.size() > target) {

        // cancel op if no edges are collapsible, but simplify is not done yet
        if (edge_queue.size() == 0) {
            return false;
        }

        Edge_Record cheap;
        cheap = edge_queue.top();

        // NOTE: the commented out code reevaluates the collapsibility of edges from the queue
        // a previous version of our code left non-collapsible edges in the queue, checking their collapsibility every iteration
        // this made more sense theoretically, but visually we couldn't tell the difference, and the runtime
        // was significantly longer (2-3x on dragon and teapot)

        // find the highest priority collapsible edge from the queue
        // for (int i = 0; i < (int) edge_queue.size(); i++) {
        //     if (is_collapsible(edge_queue[i].edge)) {
        //         cheap = edge_queue[i];
        //         break;
        //     }
        // }

        edge_queue.remove(cheap);

        // if edge would lead to degenerate mesh on collapse, ignore it for the rest of this simplify op
        if (!is_collapsible(cheap.edge)) {
            continue;
        }
        
        // get the quadric for the edge that is to be collapsed
        HalfedgeRef h = cheap.edge->halfedge();
        VertexRef v0 = h->vertex();
        VertexRef v1 = h->twin()->vertex();

        Mat4 K_v0 = vertex_quadrics[v0];
        Mat4 K_v1 = vertex_quadrics[v1];
        Mat4 K = K_v0 + K_v1;

        // remove edge records for all edges attached to the edge to be collapsed
        HalfedgeRef h0 = v0->halfedge();
        do {
            edge_queue.remove(edge_records[h0->edge()]);
            h0 = h0->twin()->next();
        } while (h0 != v0->halfedge());

        HalfedgeRef h1 = v1->halfedge();
        do {
            edge_queue.remove(edge_records[h1->edge()]);
            h1 = h1->twin()->next();
        } while (h1 != v1->halfedge());

        //delete old endpoint quadrics
        vertex_quadrics.erase(v0);
        vertex_quadrics.erase(v1);

        // collapse edge, set vertex pos
        VertexRef new_vert = collapse_edge_erase(cheap.edge).value(); // breaks if on boundary
        new_vert->pos = cheap.optimal;

        // add quadric for the new vert
        vertex_quadrics[new_vert] = K;

        // create new edge record for every edge attached to new vert, insert these into the map and queue
        HalfedgeRef h_new = new_vert->halfedge();
        do {
            Edge_Record record(vertex_quadrics, h_new->edge());
            edge_records[h_new->edge()] = record;

            edge_queue.insert(edge_records[h_new->edge()]);
            h_new = h_new->twin()->next();
        } while (h_new != new_vert->halfedge());
    }

    return true;
}
