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

    /// The mesh with the given id, mutably.
    /// Iterates over all collision meshes mutably.
    pub(crate) fn meshes_mut(&mut self) -> impl Iterator<Item = &mut SoftCollisionMesh> {
        self.clusters.iter_mut().flat_map(|c| c.meshes_mut())
    }

    /// Refreshes every mesh's vertex cache from the particles (see
    /// `SoftCollisionMesh::cached_vertex`).
    pub(crate) fn refresh_vertex_caches(&mut self) {
        let particles = &self.particles;
        for cluster in &mut self.clusters {
            for mesh in cluster.meshes_mut() {
                mesh.refresh_vertex_cache(particles);
            }
        }
    }

    pub(crate) fn mesh_mut(&mut self, id: SoftMeshId) -> Option<&mut SoftCollisionMesh> {
        self.clusters
            .get_mut(id.cluster as usize)?
            .meshes
            .get_mut(id.mesh as usize)?
            .as_mut()
    }

    /// The mesh held by `collider`, mutably.
    pub(crate) fn mesh_of_mut(
        &mut self,
        collider: crate::geometry::ColliderHandle,
    ) -> Option<&mut SoftCollisionMesh> {
        self.clusters
            .iter_mut()
            .flat_map(|cluster| cluster.meshes.iter_mut().flatten())
            .find(|mesh| mesh.collider() == collider)
    }

    /// Binds a mesh to the cluster `cluster`, without inserting it: the geometry is given in
    /// world space, as it sits when the binding is taken.
    pub(crate) fn bind_mesh(
        &self,
        cluster: u32,
        binding: &super::SoftMeshBinding,
        vertices: Vec<Vector>,
        indices: Vec<[u32; DIM]>,
    ) -> Result<SoftCollisionMesh, super::SoftBindingError> {
        use super::{SoftBindingError, SoftMeshBindingMode, SoftMeshMapping};
        let cluster = self
            .cluster(cluster)
            .ok_or(SoftBindingError::NotAClusterProxy)?;
        let in_cluster = |particle: u32| cluster.particles().binary_search(&particle).is_ok();

        let mapping = match &binding.mode {
            SoftMeshBindingMode::Direct { particles } => {
                if particles.len() != vertices.len() {
                    return Err(SoftBindingError::DegenerateMesh);
                }
                for (vertex, &particle) in particles.iter().enumerate() {
                    if particle as usize >= self.particles.len() || !in_cluster(particle) {
                        return Err(SoftBindingError::VertexOutsideCluster {
                            vertex: vertex as u32,
                            particle,
                        });
                    }
                }
                SoftMeshMapping::Direct {
                    particles: particles.clone(),
                }
            }
            SoftMeshBindingMode::DirectByPosition { eps } => {
                let mut particles = Vec::with_capacity(vertices.len());
                for (vertex, position) in vertices.iter().enumerate() {
                    let closest = cluster
                        .particles()
                        .iter()
                        .map(|&p| (p, (self.particles[p as usize].position - *position).length()))
                        .filter(|(_, d)| *d <= *eps)
                        .min_by(|a, b| a.1.total_cmp(&b.1));
                    let (particle, _) = closest.ok_or(SoftBindingError::UnmatchedVertex {
                        vertex: vertex as u32,
                    })?;
                    particles.push(particle);
                }
                SoftMeshMapping::Direct { particles }
            }
            SoftMeshBindingMode::Skinned => {
                // Only the cells the cluster fully owns may hold the mesh: a vertex must not
                // move particles its cluster does not have.
                let cells: Vec<usize> = (0..self.cells.len())
                    .filter(|&c| self.cells[c].vertices.iter().all(|v| in_cluster(*v)))
                    .collect();
                let owned: Vec<SoftBodyCell> = cells.iter().map(|&c| self.cells[c]).collect();
                // The mesh is given in world space, so it binds against where the particles
                // are now (`rest_position` is relative to the rest center of mass).
                let positions: Vec<Vector> =
                    self.particles.iter().map(|p| p.position).collect();
                let bindings = super::collision_mesh::bind_to_cells(
                    &vertices, &indices, &owned, &positions,
                )
                .ok_or(SoftBindingError::UnboundVertex { vertex: 0 })?;
                // `bind_to_cells` indexes the cells it was given: back to the body's own ids.
                SoftMeshMapping::Skinned {
                    bindings: bindings
                        .into_iter()
                        .map(|mut binding| {
                            binding.cell = cells[binding.cell as usize] as u32;
                            binding
                        })
                        .collect(),
                }
            }
        };

        Ok(SoftCollisionMesh::new(
            mapping,
            indices,
            vertices,
            self,
            binding.self_contacts,
        ))
    }

    /// Puts a bound mesh in its cluster, reusing a dead slot when there is one.
    pub(crate) fn push_mesh(&mut self, cluster: u32, mut mesh: SoftCollisionMesh) -> SoftMeshId {
        let meshes = &mut self.clusters[cluster as usize].meshes;
        let slot = meshes
            .iter()
            .position(|slot| slot.is_none())
            .unwrap_or(meshes.len());
        let id = SoftMeshId {
            cluster,
            mesh: slot as u32,
        };
        mesh.set_id(id);
        if slot == meshes.len() {
            meshes.push(Some(mesh));
        } else {
            meshes[slot] = Some(mesh);
        }
        self.modified = true;
        id
    }

    /// Records the collider a freshly inserted mesh got.
    pub(crate) fn set_mesh_collider(
        &mut self,
        id: SoftMeshId,
        collider: crate::geometry::ColliderHandle,
    ) {
        if let Some(mesh) = self.mesh_mut(id) {
            mesh.collider = collider;
        }
    }

    /// Removes the mesh `id` from its cluster, leaving a dead slot.
    pub(crate) fn remove_mesh(&mut self, id: SoftMeshId) -> Option<SoftCollisionMesh> {
        self.clusters
            .get_mut(id.cluster as usize)?
            .meshes
            .get_mut(id.mesh as usize)?
            .take()
    }

    /// Runs `f` on every mesh of this body, with the body itself in hand.
    pub(crate) fn for_each_mesh_mut(&mut self, mut f: impl FnMut(&SoftBody, &mut SoftCollisionMesh)) {
        let mut clusters = core::mem::take(&mut self.clusters);
        for cluster in &mut clusters {
            for mesh in cluster.meshes.iter_mut().flatten() {
                f(self, mesh);
            }
        }
        self.clusters = clusters;
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
