//! The collision-mesh types: the mesh itself, its vertex mapping, its id and the remap tables.

use crate::alloc_prelude::*;
use crate::geometry::ColliderHandle;
use crate::math::{DIM, Real, Vector};

use super::super::{SoftEdgeContact, SoftVertexContact};
use super::super::{SoftOverlapState, SoftOverlapWarm, SoftVolumeContact};

/// Where a skinned collision-mesh vertex rides: the cell holding it, and its barycentric
/// coordinates in that cell. The coordinates are not clamped, so a vertex the cells do not cover
/// is bound to the closest one through an extrapolation and follows it.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftMeshCellBinding {
    /// The cell holding the vertex.
    pub cell: u32,
    /// Barycentric coordinates of the vertex in that cell, summing to one.
    pub weights: [Real; DIM + 1],
}

/// Where a collision mesh lives in its soft body: the cluster owning it, and its slot in that
/// cluster. Stable for the mesh's whole life (removed meshes leave a dead slot).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftMeshId {
    /// The cluster owning the mesh.
    pub cluster: u32,
    /// The mesh's slot in that cluster.
    pub mesh: u32,
}

/// A collision mesh of a given soft body: what a deformable collider points back to.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftMeshRef {
    /// The soft body owning the mesh.
    pub body: crate::dynamics::SoftBodyHandle,
    /// Where the mesh sits in it.
    pub id: SoftMeshId,
}

/// How a collision mesh's vertices are mapped onto a soft body's computational mesh, as
/// resolved at insertion (see [`crate::dynamics::SoftMeshBinding`] for the request).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftMeshMapping {
    /// Vertex `i` of the mesh is the particle `particles[i]`: the mesh moves with the particles
    /// one for one.
    Direct {
        /// The particle backing each vertex.
        particles: Vec<u32>,
    },
    /// Vertex `i` of the mesh rides a cell of the computational mesh (cage simulation): the
    /// cells only have to contain the mesh, not resolve it.
    Skinned {
        /// Where each vertex rides.
        bindings: Vec<SoftMeshCellBinding>,
    },
}

/// A mesh a soft body collides through: a polyline (2D) or triangle mesh (3D) whose vertices
/// follow the body's particles, and the collider holding it. A skinned mesh carries thin features
/// the coarse cells cannot: its vertices are barycentric combinations of the particles.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftCollisionMesh {
    /// How the vertices follow the computational mesh.
    pub(crate) binding: SoftMeshMapping,
    /// The mesh's elements: segments in 2D, triangles in 3D.
    pub(crate) indices: Vec<[u32; DIM]>,
    /// World-space vertices of a skinned mesh, updated from the particles after every step
    /// (empty for a direct mesh, which reads its particles).
    pub(crate) vertices: Vec<Vector>,
    /// Surface 1-ring of every vertex (CSR over `ring`): the vertices sharing an element with
    /// it. Self contacts against those elements are excluded.
    pub(crate) ring_offsets: Vec<u32>,
    pub(crate) ring: Vec<u32>,
    /// Elements incident to every vertex (CSR over `vertex_elements`).
    pub(crate) vertex_elements_offsets: Vec<u32>,
    pub(crate) vertex_elements: Vec<u32>,
    /// The mesh's unique edges (`[a, b]` with `a < b`) and the edge ids of every element (3D:
    /// the triangle edges; the segments are their own edges in 2D).
    #[cfg(feature = "dim3")]
    pub(crate) edges: Vec<[u32; 2]>,
    #[cfg(feature = "dim3")]
    pub(crate) element_edges: Vec<[u32; DIM]>,
    /// The element owning each edge (the first one containing it): edge-vs-edge tests of an
    /// element pair only involve the edges each element owns, so every edge pair is tested once.
    #[cfg(feature = "dim3")]
    pub(crate) edge_owners: Vec<u32>,
    /// Whether the mesh is closed (every segment vertex / triangle edge shared by exactly two
    /// elements): its contacts with other bodies are then oriented outward.
    pub(crate) closed: bool,
    /// Signed area/volume enclosed by the mesh at rest: with `inverted`, it says which way the
    /// winding points.
    pub(crate) rest_signed_volume: Real,
    /// Whether the closed mesh is currently inside out (its signed volume, updated with the
    /// particles, has the opposite sign to the rest one): the winding is mirrored as a whole, so
    /// the outward normals flip with it.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) inverted: bool,
    /// Whether the closed mesh's signed volume is currently so far below its rest magnitude
    /// (inside the orientation hysteresis band) that the loop is likely self-crossed: part of
    /// its winding is then mirrored and no global flag can say where the outside is.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) orientation_unreliable: bool,
    /// The cell owning each element (`u32::MAX`: none), for a direct mesh: the winding of an
    /// element whose cell is inverted no longer tells where the outside is. Empty for a skinned
    /// mesh, which has no cell of its own to check.
    pub(crate) element_cells: Vec<u32>,
    /// Where this mesh lives in its soft body (its cluster and slot).
    pub(crate) id: SoftMeshId,
    /// Whether this mesh is the computational mesh's boundary: it is re-derived from it after
    /// every topology change, instead of being remapped.
    pub(crate) follows_boundary: bool,
    /// Whether this mesh is a collision mesh (`false`: it is only drawn, and has no
    /// collider).
    pub(crate) collision_enabled: bool,
    /// The collider holding this mesh's shape (invalid until the soft body is inserted in a
    /// set, like [`SoftBody::root_body`], and for a mesh that is only drawn).
    pub(crate) collider: ColliderHandle,
    /// Whether this mesh collides with itself.
    pub(crate) self_contacts: bool,
    /// Edge-vs-edge contacts of the last step (self contacts and contacts with other soft
    /// bodies' meshes, owned by the body with the smaller handle): warm-start state.
    pub(crate) edge_contacts: Vec<SoftEdgeContact>,
    /// Vertex-vs-surface contacts of the last step against this mesh (self contacts and the
    /// other soft bodies' vertices): warm-start state.
    pub(crate) vertex_contacts: Vec<SoftVertexContact>,
    /// Bumped whenever this mesh's elements change (a tear): renderers keying their mesh on it
    /// rebuild it on change.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) topology_version: u32,
    /// The world-space vertices as of the last [`Self::refresh_vertex_cache`] (before every
    /// narrow-phase update): the contact passes read this contiguous copy instead of walking
    /// the particle array.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) vertex_cache: Vec<Vector>,
}

/// How a topology change renumbered a soft body's cells and particles (old index to new,
/// `u32::MAX` removed, empty when unchanged), the split particles as `(copy, source)`, and the
/// segments a cut inserted into as `[a, b, p, q]` (`(a, b)` became `(a, p)` and `(q, b)`).
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct SoftTopologyRemap<'a> {
    pub cells: &'a [u32],
    pub particles: &'a [u32],
}
