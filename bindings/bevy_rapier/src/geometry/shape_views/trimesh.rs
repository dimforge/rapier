use crate::geometry::Collider;
use crate::math::{Rot, Vect};
#[cfg(feature = "dim3")]
use rapier::parry::shape::TriMeshPseudoNormals;
use rapier::parry::shape::{
    FeatureId, TopologyError, TriMesh, TriMeshBuilderError, TriMeshConnectedComponents,
    TriMeshFlags, TriMeshTopology,
};

/// Read-only access to the properties of a triangle mesh.
#[derive(Copy, Clone)]
pub struct TriMeshView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a TriMesh,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The number of triangles forming this mesh.
            pub fn num_triangles(&self) -> usize {
                self.raw.num_triangles()
            }

            /// An iterator through all the triangles of this mesh.
            pub fn triangles(&self) -> impl ExactSizeIterator<Item = (Vect, Vect, Vect)> + '_ {
                self.raw
                    .triangles()
                    .map(|tri| (tri.a.into(), tri.b.into(), tri.c.into()))
            }

            /// Get the `i`-th triangle of this mesh.
            pub fn triangle(&self, i: u32) -> (Vect, Vect, Vect) {
                let tri = self.raw.triangle(i);
                (tri.a.into(), tri.b.into(), tri.c.into())
            }

            /// The vertex buffer of this mesh.
            pub fn vertices(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
                self.raw.vertices().iter().map(|pt| (*pt).into())
            }

            /// The index buffer of this mesh.
            pub fn indices(&self) -> &[[u32; 3]] {
                self.raw.indices()
            }

            /// A flat view of the index buffer of this mesh.
            pub fn flat_indices(&self) -> &[u32] {
                self.raw.flat_indices()
            }

            /// The flags controlling the optional data associated to this mesh.
            pub fn flags(&self) -> TriMeshFlags {
                self.raw.flags()
            }

            /// The outward normal of the `i`-th triangle, or `None` if it is degenerate.
            #[cfg(feature = "dim3")]
            pub fn triangle_normal(&self, i: u32) -> Option<Vect> {
                self.raw.triangle_normal(i)
            }

            /// Does the given feature (as returned by a ray-cast or point projection on one of
            /// the triangles) identify the back face of that triangle?
            pub fn is_backface(&self, feature: FeatureId) -> bool {
                self.raw.is_backface(feature)
            }

            /// The half-edge topology of this mesh, if it was computed.
            ///
            /// This requires the [`TriMeshFlags::HALF_EDGE_TOPOLOGY`] flag.
            pub fn topology(&self) -> Option<&TriMeshTopology> {
                self.raw.topology()
            }

            /// The connected components of this mesh, if they were computed.
            ///
            /// This requires the [`TriMeshFlags::CONNECTED_COMPONENTS`] flag.
            pub fn connected_components(&self) -> Option<&TriMeshConnectedComponents> {
                self.raw.connected_components()
            }

            /// The number of connected components of this mesh, if they were computed.
            pub fn num_connected_components(&self) -> Option<usize> {
                self.raw
                    .connected_components()
                    .map(|cc| cc.num_connected_components())
            }

            /// Builds one triangle mesh collider per connected component of this mesh, each
            /// built with the given `flags`.
            ///
            /// Returns `None` if the connected components were not computed.
            pub fn connected_component_colliders(
                &self,
                flags: TriMeshFlags,
            ) -> Option<Vec<Result<Collider, TriMeshBuilderError>>> {
                self.raw.connected_component_meshes(flags).map(|meshes| {
                    meshes
                        .into_iter()
                        .map(|mesh| mesh.map(|mesh| rapier::prelude::SharedShape::new(mesh).into()))
                        .collect()
                })
            }

            /// The vertex and edge pseudo-normals of this mesh, if they were computed.
            ///
            /// They are computed with the [`TriMeshFlags::ORIENTED`] or
            /// [`TriMeshFlags::FIX_INTERNAL_EDGES`] flags.
            #[cfg(feature = "dim3")]
            pub fn pseudo_normals(&self) -> Option<&TriMeshPseudoNormals> {
                self.raw.pseudo_normals()
            }
        }
    }
);

impl_ref_methods!(TriMeshView);

/// Read-write access to the properties of a triangle mesh.
pub struct TriMeshViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut TriMesh,
}

impl_ref_methods!(TriMeshViewMut);

impl TriMeshViewMut<'_> {
    /// Sets the flags of this triangle mesh, controlling its optional associated data.
    pub fn set_flags(&mut self, flags: TriMeshFlags) -> Result<(), TopologyError> {
        self.raw.set_flags(flags)
    }

    /// Replaces the vertex positions, keeping the index buffer and topology.
    ///
    /// The acceleration structure is refitted and the pseudo-normals (if any) are recomputed.
    /// This is intended for meshes built with [`TriMeshFlags::DEFORMABLE`].
    ///
    /// # Panics
    /// Panics if `vertices.len()` differs from the current number of vertices.
    pub fn set_vertices(&mut self, vertices: &[Vect]) {
        self.raw.set_vertices(vertices)
    }

    /// Modifies the vertex positions in place through `f`, then refits the acceleration
    /// structure and recomputes the pseudo-normals (if any).
    ///
    /// This is intended for meshes built with [`TriMeshFlags::DEFORMABLE`].
    pub fn update_vertices(&mut self, f: impl FnOnce(&mut [Vect])) {
        self.raw.update_vertices(f)
    }

    /// Applies a rigid transformation to every vertex of this mesh.
    pub fn transform_vertices(&mut self, translation: Vect, rotation: Rot) {
        self.raw
            .transform_vertices(&crate::utils::pose_from(translation, rotation))
    }

    /// Reverses the orientation of every triangle of this mesh.
    pub fn reverse(&mut self) {
        self.raw.reverse()
    }
}
