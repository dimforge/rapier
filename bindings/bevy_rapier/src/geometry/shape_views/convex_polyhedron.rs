use crate::math::Vect;
use rapier::parry::shape::ConvexPolyhedron;

/// Read-only access to the properties of a convex polyhedron.
#[derive(Copy, Clone)]
pub struct ConvexPolyhedronView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a ConvexPolyhedron,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The vertices of this convex polyhedron.
            pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
                self.raw.points().iter().copied()
            }

            /// The number of vertices of this convex polyhedron.
            pub fn num_vertices(&self) -> usize {
                self.raw.vertices().len()
            }

            /// The indices of the edges of this convex polyhedron.
            ///
            /// The internal storage also contains edges between coplanar triangles merged into
            /// the same polygonal face; those are skipped here.
            pub fn edges(&self) -> impl Iterator<Item = u32> + '_ {
                self.raw
                    .edges()
                    .iter()
                    .enumerate()
                    .filter(|(_, edge)| edge.faces[0] != edge.faces[1])
                    .map(|(i, _)| i as u32)
            }

            /// The number of (polygonal) faces of this convex polyhedron.
            pub fn num_faces(&self) -> usize {
                self.raw.faces().len()
            }

            /// The indices of the faces adjacent to the `i`-th vertex.
            pub fn vertex_adjacent_faces(&self, i: usize) -> &[u32] {
                let vertex = &self.raw.vertices()[i];
                let first = vertex.first_adj_face_or_edge as usize;
                let num = vertex.num_adj_faces_or_edge as usize;
                &self.raw.faces_adj_to_vertex()[first..first + num]
            }

            /// The indices of the two endpoints of the `i`-th edge (indexing [`Self::points`]).
            pub fn edge_vertices(&self, i: usize) -> [u32; 2] {
                self.raw.edges()[i].vertices
            }

            /// The indices of the two faces adjacent to the `i`-th edge.
            pub fn edge_faces(&self, i: usize) -> [u32; 2] {
                self.raw.edges()[i].faces
            }

            /// The unit direction of the `i`-th edge, from its first to its second vertex.
            pub fn edge_direction(&self, i: usize) -> Vect {
                self.raw.edges()[i].dir
            }

            /// The outward unit normal of the `i`-th face.
            pub fn face_normal(&self, i: usize) -> Vect {
                self.raw.faces()[i].normal
            }

            /// The indices of the vertices of the `i`-th face (indexing [`Self::points`]).
            pub fn face_vertices(&self, i: usize) -> &[u32] {
                let face = &self.raw.faces()[i];
                let first = face.first_vertex_or_edge as usize;
                let num = face.num_vertices_or_edges as usize;
                &self.raw.vertices_adj_to_face()[first..first + num]
            }

            /// The indices of the edges of the `i`-th face.
            pub fn face_edges(&self, i: usize) -> &[u32] {
                let face = &self.raw.faces()[i];
                let first = face.first_vertex_or_edge as usize;
                let num = face.num_vertices_or_edges as usize;
                &self.raw.edges_adj_to_face()[first..first + num]
            }

            /// Triangulates the boundary of this convex polyhedron, returning its vertex and
            /// index buffers.
            pub fn to_trimesh(&self) -> (Vec<Vect>, Vec<[u32; 3]>) {
                self.raw.to_trimesh()
            }
        }
    }
);

impl_ref_methods!(ConvexPolyhedronView);

/// Read-write access to the properties of a convex polyhedron.
///
/// Parry does not offer any in-place modification of convex polyhedra, so this view is
/// read-only in practice.
pub struct ConvexPolyhedronViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut ConvexPolyhedron,
}

impl_ref_methods!(ConvexPolyhedronViewMut);
