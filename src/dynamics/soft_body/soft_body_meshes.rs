//! The collision meshes of a soft body: lookup, binding to clusters, insertion and removal, and update after a step or a tear.
use crate::alloc_prelude::*;
use super::{SoftBody, SoftBodyCell, SoftCollisionMesh, SoftMeshId};
use crate::math::{DIM, Vector};

impl SoftBody {
    /// The meshes this soft body meets the world through, over all its clusters (and the ones
    /// it is only drawn as: a mesh whose collision is disabled has no collider).
    pub fn meshes(&self) -> impl Iterator<Item = &SoftCollisionMesh> {
        self.clusters
            .iter()
            .filter(|cluster| cluster.is_live())
            .flat_map(|cluster| cluster.meshes())
    }

    /// The mesh with the given id, if it is live.
    pub fn mesh(&self, id: SoftMeshId) -> Option<&SoftCollisionMesh> {
        self.clusters
            .get(id.cluster as usize)
            .filter(|cluster| cluster.is_live())
            .and_then(|cluster| cluster.mesh(id.mesh))
    }

    /// The mesh held by `collider`, if this body owns it.
    pub fn mesh_of(&self, collider: crate::geometry::ColliderHandle) -> Option<&SoftCollisionMesh> {
        self.meshes().find(|mesh| mesh.collider() == collider)
    }

    /// The first mesh this soft body collides through, if any.
    pub fn collision_mesh(&self) -> Option<&SoftCollisionMesh> {
        self.meshes().find(|mesh| mesh.collision_enabled())
    }

    pub(crate) fn for_each_mesh_mut(&mut self, mut f: impl FnMut(&SoftBody, &mut SoftCollisionMesh)) {
        let mut clusters = core::mem::take(&mut self.clusters);
        for cluster in &mut clusters {
            for mesh in cluster.meshes.iter_mut().flatten() {
                f(self, mesh);
            }
        }
    /// Updates the meshes' cached vertices from the current particle positions.
    pub(crate) fn update_meshes(&mut self) {
        self.for_each_mesh_mut(|body, mesh| mesh.update(&body.cells, &body.particles));
    }

    /// Rebuilds the meshes' derived tables after a topology change (a tear).
    pub(crate) fn rebuild_mesh_tables(&mut self) {
        self.for_each_mesh_mut(|body, mesh| mesh.rebuild_tables(body));
    }
}
