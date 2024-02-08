
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

bool Halfedge_Mesh::is_degenerate(Halfedge_Mesh::FaceRef f) {
    HalfedgeRef h0 = f->halfedge();
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h0->next()->vertex();
    VertexRef v2 = h0->next()->next()->vertex();
    return v0->pos == v1->pos || v1->pos == v2->pos || v2->pos == v0->pos;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    // boundary check:
    if (e->halfedge()->is_boundary() || e->halfedge()->twin()->is_boundary()) {
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

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
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

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    
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

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
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

    // Edges

    // Vertices
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
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
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
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

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

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
