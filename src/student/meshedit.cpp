
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"
#include <iostream>

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
    HalfedgeRef h = v->halfedge();
    std::vector<HalfedgeRef> halfedges;
    do {
        HalfedgeRef h1 = h;
        HalfedgeRef h2 = h1->twin();
        HalfedgeRef h3 = h2->next()->next();
        while (h3 != h2) {
            halfedges.push_back(h3);
            h3->vertex()->halfedge() = h3;
            h3 = h3->next();
        }

        erase(h1);
        erase(h2);
        erase(h1->edge());
        erase(h2->face());
        
        h = h->twin()->next();
    } while(h != v->halfedge());
    
    erase(v);
    
    FaceRef f = new_face();
    f->halfedge() = halfedges[0];
    for (size_t i = 0; i < halfedges.size(); i++) {
        halfedges[i]->face() = f;
        halfedges[i]->next() = halfedges[i]->twin()->vertex()->halfedge();
    }
    return f;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    HalfedgeRef edges_to_remove[2] = {e->halfedge(), e->halfedge()->twin()};
    std::vector<HalfedgeRef> halfedges;
    for(auto i : edges_to_remove) {
        HalfedgeRef h = i->next();
        do {
            halfedges.push_back(h);
            h->vertex()->halfedge() = h;
            h = h->next();
        } while(h != i);
        erase(h->face());
    }
    FaceRef f = new_face();
    f->halfedge() = halfedges[0];

    for(auto i: halfedges) {
        i->face() = f;
        i->next() = i->twin()->vertex()->halfedge();
    }
    erase(e);
    erase(edges_to_remove[0]);
    erase(edges_to_remove[1]);
    return f;
}

bool Halfedge_Mesh::is_collapsible(Halfedge_Mesh::EdgeRef e) {
    HalfedgeRef h1 = e->halfedge();
    HalfedgeRef h2 = e->halfedge()->twin();
    VertexRef v1 = h1->vertex();
    VertexRef v2 = h2->vertex();

    if (v1->on_boundary() && v2->on_boundary() && !e->on_boundary()) {
        return false;
    }

    int cnt = 2;
    if(h1->face()->degree() > 3) {
        cnt--;
    }
    if(h2->face()->degree() > 3) {
        cnt--;
    }

    HalfedgeRef t1 = h1;
    do {    
        HalfedgeRef t2 = h2;
        do {
            if(t2->twin()->vertex()->id() == t1->twin()->vertex()->id()) {
                if(cnt == 0) {
                    return false;
                }
                cnt--;
            }
            t2 = t2->twin()->next();
        } while(t2 != h2);
        t1 = t1->twin()->next();
    }while(t1 != h1);

    return true;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    /*
       A - H - I              A-H-I 
      / \ / \ / \            / \|/ \ 
     B---F-- K--J     -->   B---N---J
      \ / \ / \ /            \ /|\ / 
       D - L - E              D-L-E
    */

    if (!is_collapsible(e)) {
        return std::nullopt;
    }

    HalfedgeRef start_edges[2] = {e->halfedge(), e->halfedge()->twin()};
    VertexRef start_vertices[2] = {start_edges[0]->vertex(), start_edges[1]->vertex()};
    
    VertexRef v0 = new_vertex();
    v0->pos = (start_vertices[0]->pos + start_vertices[1]->pos) / 2;

    for(auto i: start_vertices) {
        HalfedgeRef h = i->halfedge();
        do {
            h->vertex() = v0;
            h = h->twin()->next();
        } while(h != i->halfedge());
        erase(i);
    }
    
    for(auto i : start_edges) {
        FaceRef face = i->face();
        if(face->degree() == 3) {
            HalfedgeRef h0 = i;
            HalfedgeRef h1 = h0->next();
            HalfedgeRef h2 = h1->next();
            HalfedgeRef h1_t = h1->twin();
            HalfedgeRef h2_t = h2->twin();
            
            EdgeRef e1 = h1->edge();
            EdgeRef e2 = h2->edge();

            FaceRef f = h0->face();

            e1->halfedge() = h1_t;
            h1_t->twin() = h2_t;
            h2_t->twin() = h1_t;
            v0->halfedge() = h2_t;
            h2_t->edge() = e1;
            h1->vertex()->halfedge() = h2_t;
            h2->vertex()->halfedge() = h1_t;

            erase(f);
            erase(h1);
            erase(h2);
            erase(e2);
        } else {
            HalfedgeRef h0 = i;
            HalfedgeRef h1 = h0->next();
            HalfedgeRef pre = i;
            while (pre->next() != i) {
                pre = pre->next();
            }
            pre->next() = h1;
            h1->vertex()->halfedge() = h1;
            h1->face()->halfedge() = h1;
        }
    }
    
    erase(e);
    erase(start_vertices[0]);
    erase(start_vertices[1]);
    erase(start_edges[0]);
    erase(start_edges[1]);
    
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

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    if (e->on_boundary()) {
        return std::nullopt;
    }

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();
    HalfedgeRef h3 = h0->twin();
    HalfedgeRef h4 = h3->next();
    HalfedgeRef h5 = h4->next();

    HalfedgeRef h0_pre = h0;
    while (h0_pre->next() != h0) {
        h0_pre = h0_pre->next();
    }
    HalfedgeRef h3_pre = h3;
    while (h3_pre->next() != h3) {
        h3_pre = h3_pre->next();
    }
    
    FaceRef f0 = h0->face();
    FaceRef f1 = h3->face();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h3->vertex();
    VertexRef v2 = h2->vertex();
    VertexRef v3 = h5->vertex();
    
    h0->next() = h2;
    h1->next() = h3;
    h3->next() = h5;
    h0_pre->next() = h4;
    h4->next() = h0;
    h3_pre->next() = h1;

    h0->vertex() = v3;
    h3->vertex() = v2;
    h1->face() = h3->face();
    h4->face() = h0->face();
   

    if (v0->halfedge() == h0) {
        v0->halfedge() = h4;
    }
    if (v1->halfedge() == h3) {
       v1->halfedge() = h1;
    }
    if (f0->halfedge() == h1) {
        f0->halfedge() = h2;
    }
    if (f1->halfedge() == h4) {
        f1->halfedge() = h5;
    }

    return e;

}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    HalfedgeRef h = e->halfedge();
    HalfedgeRef ht = h->twin();
    EdgeRef e0 = h->edge();
    HalfedgeRef h1 = new_halfedge();
    HalfedgeRef h1t = new_halfedge();
    EdgeRef e1 = new_edge();
    VertexRef v0 = ht->vertex();
    VertexRef v1 = h->vertex();
    VertexRef v = new_vertex();
    v->pos = (h->vertex()->pos + ht->vertex()->pos) / 2;
    v->is_new = true;
   
    h1->set_neighbors(h->next(), ht, v, e0, h->face());
    h1t->set_neighbors(ht->next(), h, v, e1, ht->face());
    h->set_neighbors(h1, h1t, v1, e1, h->face());
    ht->set_neighbors(h1t, h1, v0, e0, ht->face());
    v->halfedge() = h1;
    e0->halfedge() = h1;
    e1->halfedge() = h1t;

    HalfedgeRef start_edges[2] = {h, ht};
    for (auto i : start_edges) {
        if(i->is_boundary()) {
            continue;
        }
        HalfedgeRef he0 = i;
        HalfedgeRef he1 = he0->next();
        HalfedgeRef he2 = he1->next();
        HalfedgeRef he3 = he2->next();

        HalfedgeRef hn = new_halfedge();
        HalfedgeRef hnt = new_halfedge();
        EdgeRef e = new_edge();
        e->halfedge() = hn;
        e->is_new = true;

        FaceRef f0 = he0->face();
        FaceRef f1 = new_face();
        FaceRef f2 = new_face();

        he1->face() = f1;
        he3->face() = f2;
        hn->set_neighbors(he1, hnt, he3->vertex(), e, f1);
        hnt->set_neighbors(he3, hn, he1->vertex(), e, f2);
        he0->set_neighbors(hnt, he0->twin(), he0->vertex(), he0->edge(), f2);
        he2->set_neighbors(hn, he2->twin(), he2->vertex(), he2->edge(), f1);

        f1->halfedge()= he1;
        f2->halfedge() = he3;
        erase(f0);
    }

    return v;
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
    size_t degree = f->degree();
    
    std::vector<HalfedgeRef> top_half_edges;
    std::vector<HalfedgeRef> side_half_edges;
    std::vector<HalfedgeRef> bottom_half_edges;
    std::vector<EdgeRef> top_edges;
    std::vector<EdgeRef> side_edges;
    std::vector<VertexRef> top_vertices;
    std::vector<VertexRef> bottom_vertices;
    std::vector<FaceRef> side_faces;
   

    HalfedgeRef h = f->halfedge();

    do {
        top_half_edges.push_back(new_halfedge());
        top_half_edges.push_back(new_halfedge());
        side_half_edges.push_back(new_halfedge());
        side_half_edges.push_back(new_halfedge());
        bottom_half_edges.push_back(h);
        top_edges.push_back(new_edge());
        side_edges.push_back(new_edge());
        top_vertices.push_back(new_vertex());
        bottom_vertices.push_back(h->vertex());
        side_faces.push_back(new_face());
        
        h = h->next();
    } while( h != f->halfedge());

    for (size_t i = 0; i < degree; i++) {
        VertexRef top_v = top_vertices[i];
        VertexRef bottom_v = bottom_vertices[i];
        top_v->pos = bottom_v->pos;
        VertexRef bottom_next_v = bottom_vertices[(i+1)%degree];
        VertexRef top_next_v = top_vertices[(i+1)%degree];
    
        EdgeRef side_edge = side_edges[i];
        EdgeRef next_side_edge = side_edges[(i+1) % degree];
        EdgeRef top_edge = top_edges[i];
        HalfedgeRef side_half_edge = side_half_edges[i*2];
        HalfedgeRef side_half_edge_twin = side_half_edges[(i*2 + 1) % (degree * 2)];
        HalfedgeRef top_half_edge = top_half_edges[i*2];
        HalfedgeRef top_half_edge_twin = top_half_edges[(i*2 + 1) % (degree * 2)];
        HalfedgeRef top_next_halfEdge_twin = top_half_edges[ ((i+1) * 2 + 1 + degree *2 ) % (degree *2)];
        HalfedgeRef next_side_half_edge = side_half_edges[(i+1)*2 % (degree * 2)];
        HalfedgeRef next_side_half_edge_twin = side_half_edges[((i + 1)*2 + 1) % (degree * 2)];
        FaceRef current_face = side_faces[i];

        HalfedgeRef bottom_half_edge = bottom_half_edges[i];

        side_half_edge_twin->set_neighbors(bottom_half_edge, side_half_edge,top_v, side_edge, current_face);
        next_side_half_edge->set_neighbors(top_half_edge, next_side_half_edge_twin, bottom_next_v, next_side_edge, current_face);
        top_half_edge->set_neighbors(side_half_edge_twin, top_half_edge_twin, top_next_v, top_edge, current_face);
        top_half_edge_twin->set_neighbors(top_next_halfEdge_twin, top_half_edge, top_v, top_edge, f);
        bottom_half_edge->next() = next_side_half_edge;
        bottom_half_edge->face() = current_face;
        
        top_edge->halfedge() = top_half_edge;
        side_edge->halfedge() = side_half_edge;
        top_v->halfedge() = side_half_edge_twin;
        current_face->halfedge() = bottom_half_edge;
    }

    f->halfedge() = top_half_edges[1];
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

    Vec3 norm = face->normal();
    Vec3 c = face->center();
    size_t size = new_halfedges.size();
    
    for(size_t i = 0; i < size; i++) {
        Vec3 pi = start_positions[i];
        Vec3 pos = pi - c;
        pos = -pos * tangent_offset + pi;
        pos = -normal_offset * norm + pos;
        new_halfedges[i]->vertex()->pos = pos;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        if(f->degree() == 3) continue;
        HalfedgeRef h = f->halfedge();
        VertexRef v0 = h->vertex();
        HalfedgeRef h1 = h;
        HalfedgeRef h2 = h1->next();

        while(h2->next()->next()!= h) {
            HalfedgeRef half_edge = new_halfedge();
            HalfedgeRef half_edge_twin = new_halfedge();
            HalfedgeRef halfe_edge_next = h2->next(); 
            
            EdgeRef e = new_edge();

            FaceRef nf = new_face();
            VertexRef v2 = h2->next()->vertex();

            half_edge->set_neighbors(h1, half_edge_twin, v2, e, nf);
            
            half_edge_twin->twin() = half_edge;
            half_edge_twin->edge() = e;
            half_edge_twin->vertex() = v0;
            half_edge_twin->next() = halfe_edge_next;
            
            
            
            h2->face() = nf;
            h1->face() = nf;
            h2->next() = half_edge;
            nf->halfedge() = h1;
            e->halfedge() = half_edge;

            h1 = half_edge_twin;
            h2 = halfe_edge_next;
        }

        FaceRef nf = new_face();
        h1->face() = nf;
        h2->face() = nf;
        h2->next()->next() = h1;
        h2->next()->face() = nf;
        nf->halfedge() = h1;
        
        erase(f);
        
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
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }
    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->new_pos = (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2;
    }
    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        HalfedgeRef h = f->halfedge();
        Vec3 new_pos = Vec3();
        int cnt = 0;
        
        do{
            new_pos += h->vertex()->pos;
            cnt++;
            h = h->next();
        }while(h!= f->halfedge());
        f->new_pos = new_pos / cnt;
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
        HalfedgeRef h = f->halfedge();
        Vec3 new_pos = Vec3();
        int cnt = 0;
        do{
            new_pos += h->vertex()->pos;
            cnt++;
            h = h->next();
        } while(h != f->halfedge());
        f->new_pos = new_pos / cnt;
    }
    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        e->new_pos = (h0->vertex()->pos 
                    + h1->vertex()->pos 
                    + h0->face()->new_pos 
                    + h1->face()->new_pos) / 4;
    }

    // Vertices
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        HalfedgeRef h = v->halfedge();
        Vec3 Q = Vec3();
        Vec3 R = Vec3();
        float n = 0;
        do {
            Q += h->face()->new_pos;
            R += h->edge()->new_pos;
            n++;
            h = h->twin()->next();
        } while(h != v->halfedge());
        Q /= n;
        R /= n;
        v->new_pos = (Q + 2 * R + (n - 3) * v->pos) / n;
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
    
    for (auto face = faces_begin(); face != faces_end(); face++) {
        if (face->degree() != 3) {
            std::cerr << "Loop subdivision is only for triangle mesh" << std::endl;
            return;
        }
    }
    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        HalfedgeRef h = v->halfedge();
        float n = v->degree();
        float u = n == 3 ? 3.0 / 16 : 3.0 / (8 * n);
        Vec3 sum_p = Vec3();
        do {
            sum_p += h->twin()->vertex()->pos;
            h = h->twin()->next();
        } while(h != v->halfedge());
        v->new_pos = (1 - n * u) * v->pos + u * sum_p;
        v->is_new = false;
    }

    // Next, compute the updated vertex positions associated with edges.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        VertexRef v0 = h0->vertex();
        VertexRef v1 = h1->vertex();
        VertexRef v2 = h0->next()->twin()->vertex();
        VertexRef v3 = h1->next()->twin()->vertex();
        e->new_pos = 0.375 * (v0->pos + v1->pos) + 0.125 * (v2->pos + v3->pos);
        e->is_new = false;
    }

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    int n = edges.size();
    EdgeRef edge = edges_begin();
    for(int i = 0; i < n; i++) {
        auto vc = split_edge(edge);
        if(vc) {
            VertexRef v = vc.value();
            v->new_pos = edge->new_pos;
        }
        edge++;
    }

    // Finally, flip any new edge that connects an old and new vertex.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if(!e->is_new) continue;
        VertexRef v0 = e->halfedge()->vertex();
        VertexRef v1 = e->halfedge()->twin()->vertex();
        if(v0->is_new != v1->is_new) {
            flip_edge(e);
        }
    }
    // Copy the updated vertex positions to the subdivided mesh.
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
    for(auto face = faces_begin(); face != faces_end(); face++) {
        if(face->degree() != 3) {
            return false;
        }
    }
    std::cout << n_faces() << std::endl;

    float n = edges.size();
    float mean_length = 0;
    for (auto edge = edges_begin(); edge != edges_end(); edge++) {
       mean_length += edge->length();
    }
    mean_length /= n;
    std::cout << "mean_length: " << mean_length << std::endl;
    EdgeRef edge = edges_begin();
    for(int i = 0; i < n; i++) {
        if(edge->length() > 4.f * mean_length / 3.f) {
            split_edge(edge);
        }
        edge++;
    }
    do_erase();
    edge = edges_begin();
    n = edges.size();
    for(int i = 0; i < n; i++) {
        
        if(edge->length() < 4.f * mean_length / 5.f) {
            collapse_edge(edge);
        }
        while(eerased.find(edge) != eerased.end()) {
            edge++;
            i++;
        }
    }
    do_erase();
    for(auto e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        VertexRef v0 = h0->vertex();
        VertexRef v1 = h1->vertex();
        VertexRef v2 = h0->next()->twin()->vertex();
        VertexRef v3 = h1->next()->twin()->vertex();
        int d0 = v0->degree();
        int d1 = v1->degree();
        int d2 = v2->degree();
        int d3 = v3->degree();
        int cur_dev = abs(d0 - 6) + abs(d1 - 6) + abs(d2 - 6) + abs(d3 - 6);
        int new_dev = abs(d0 - 7) + abs(d1 - 7) + abs(d2 - 5) + abs(d3 - 5);
        if(new_dev < cur_dev) {
            flip_edge(e);
        }
    }
    for(int i = 0; i < 15; i++)  {
        for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
            Vec3 c = v->neighborhood_center();
            Vec3 p = v->pos;
            Vec3 dv = c - p;
            Vec3 n = v->normal();
            dv = dv - dot(dv, n) * n;

            v->new_pos = p + dv;
        }

        for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
            v->pos = v->new_pos;
        }
    }
    return true;
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

        Halfedge_Mesh::VertexRef v0 = e->halfedge()->vertex();
        Halfedge_Mesh::VertexRef v1 = e->halfedge()->twin()->vertex();
        Mat4 K = vertex_quadrics[v0] + vertex_quadrics[v1];

        optimal = K.inverse() *  Vec3(-K[0][3], -K[1][3], -K[2][3]);
        cost = dot(Vec4(optimal, 1) , K * Vec4(optimal, 1));
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

    size_t face_count = faces.size();
    std::cout<<face_count<<std::endl;
    if(face_count < 128) {
        return false;
    }

    auto update_face = [&face_quadrics](const FaceRef& f) {
        Vec3 n = f->normal();
        Vec3 p = f->halfedge()->vertex()->pos;
        float d = -dot(n, p);
        Vec4 v = Vec4(n.x, n.y, n.z, d);
        Mat4 q = outer(v, v);
        face_quadrics[f] = q;
    };

    auto update_vertex = [&](const VertexRef& v) {
        Mat4 q = Mat4::Zero;
        HalfedgeRef h = v->halfedge();
        do {
            update_face(h->face());
            q += face_quadrics[h->face()];
            h = h->twin()->next();
        } while(h != v->halfedge());
        vertex_quadrics[v] = q;
    };

    auto update_edge = [&](const EdgeRef& e) {
        edge_records[e] = Edge_Record(vertex_quadrics, e);
        edge_queue.insert(edge_records[e]);
    };

    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        update_face(f);
    }

    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        update_vertex(v);
    }

    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
       update_edge(e);
    }

    while(faces.size() > face_count / 4) {
        Edge_Record er = edge_queue.top();
        edge_queue.pop();
        EdgeRef e = er.edge;
        if (!is_collapsible(e)) continue;

        VertexRef v0[2] = { e->halfedge()->vertex(),  e->halfedge()->twin()->vertex()};
        for (VertexRef v : v0) {
            HalfedgeRef h = v->halfedge();
            do {
                edge_queue.remove(edge_records[h->edge()]);
                h = h->twin()->next();
            } while(h != v->halfedge());
        }
        edge_records.erase(e);
        vertex_quadrics.erase(v0[0]);
        vertex_quadrics.erase(v0[1]);

        
        auto v_container = collapse_edge_erase(e);
        if(!v_container.has_value()) {
            return false;
        }

        VertexRef v = v_container.value();
        v->pos = er.optimal;

        HalfedgeRef hi = v->halfedge();
        do {
            update_face(hi->face());
            hi = hi->twin()->next();
        } while(hi != v->halfedge());

        update_vertex(v);
        hi = v->halfedge();
        do {
            update_vertex(hi->twin()->vertex());
            hi = hi->twin()->next();
        } while(hi != v->halfedge());

        hi = v->halfedge();
        do {
            update_edge(hi->edge());
            hi = hi->twin()->next();
        } while (hi != v->halfedge());
    
    }
    vertex_quadrics.clear();
    face_quadrics.clear();
    edge_records.clear();
    edge_queue.queue.clear();
    return true;
}
