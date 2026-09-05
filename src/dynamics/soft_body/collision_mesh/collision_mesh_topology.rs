//! Construction of a collision mesh, its topology tables, and the per-step update of its
//! vertices, orientation and collider shape.

use crate::alloc_prelude::*;
use crate::geometry::ColliderHandle;
use crate::math::{DIM, Pose, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::collision_mesh_binding::{bind_to_cells, closest_cell_binding};
use super::{
    SoftBody, SoftBodyCell, SoftCollisionMesh, SoftMeshId, SoftMeshMapping, SoftTopologyRemap,
    surface_element_cells, surface_is_closed, surface_rings, surface_vertex_elements,
};

impl SoftCollisionMesh {
    /// The mesh a body collides through when it collides through its own boundary: one vertex
    /// per particle (interior particles included, so vertex indices are particle indices), the
    /// cells' boundary as elements.
    pub(crate) fn boundary(
        body: &SoftBody,
        self_contacts: bool,
        collision_enabled: bool,
    ) -> Self {
        let num_vertices = body.particles.len();
        let indices = body.boundary.clone();
        let (ring_offsets, ring) = surface_rings(num_vertices, &indices);
        let (vertex_elements_offsets, vertex_elements) =
            surface_vertex_elements(num_vertices, &indices);
        #[cfg(feature = "dim3")]
        let (edges, element_edges, edge_owners) =
            super::soft_body_builder::surface_edge_table(&indices);

        Self {
            binding: SoftMeshMapping::Direct {
                particles: (0..num_vertices as u32).collect(),
            },
            id: SoftMeshId { cluster: 0, mesh: 0 },
            follows_boundary: true,
            closed: surface_is_closed(&indices),
            rest_signed_volume: SoftBody::boundary_volume(&indices, |i| {
                body.particles[i as usize].rest_position
            }),
            element_cells: Vec::new(),
            indices,
            vertices: Vec::new(),
            ring_offsets,
            ring,
            vertex_elements_offsets,
            vertex_elements,
            #[cfg(feature = "dim3")]
            edges,
            #[cfg(feature = "dim3")]
            element_edges,
            #[cfg(feature = "dim3")]
            edge_owners,
            inverted: false,
            orientation_unreliable: false,
            collision_enabled,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            crossing_sweep_travel: Real::MAX,
            crossed_partners: Vec::new(),
            vertex_cache: Vec::new(),
            topology_version: 0,
        }
    }

    /// A mesh bound to a cluster through `mapping`, whose vertices are at `vertices` when the
    /// binding is taken.
    pub(crate) fn new(
        mapping: SoftMeshMapping,
        indices: Vec<[u32; DIM]>,
        vertices: Vec<Vector>,
        body: &SoftBody,
        self_contacts: bool,
    ) -> Self {
        let num_vertices = vertices.len();
        let (ring_offsets, ring) = surface_rings(num_vertices, &indices);
        let (vertex_elements_offsets, vertex_elements) =
            surface_vertex_elements(num_vertices, &indices);
        #[cfg(feature = "dim3")]
        let (edges, element_edges, edge_owners) =
            super::soft_body_builder::surface_edge_table(&indices);
        let skinned = matches!(mapping, SoftMeshMapping::Skinned { .. });
        // A wire has no facet, so no owning cell to read a winding from.
        let wire = indices
            .first()
            .is_some_and(|e| super::soft_body_builder::element_vertices(e).len() < DIM);

        Self {
            closed: surface_is_closed(&indices),
            // A wire encloses nothing: its "volume" is not a quantity.
            rest_signed_volume: if wire {
                0.0
            } else {
                SoftBody::boundary_volume(&indices, |i| vertices[i as usize])
            },
            element_cells: if skinned || wire {
                Vec::new()
            } else {
                surface_element_cells(&indices, &body.cells)
            },
            binding: mapping,
            id: SoftMeshId::default(),
            follows_boundary: false,
            indices,
            // A direct mesh reads its particles; only a skinned one caches its vertices.
            vertices: if skinned { vertices } else { Vec::new() },
            ring_offsets,
            ring,
            vertex_elements_offsets,
            vertex_elements,
            #[cfg(feature = "dim3")]
            edges,
            #[cfg(feature = "dim3")]
            element_edges,
            #[cfg(feature = "dim3")]
            edge_owners,
            inverted: false,
            orientation_unreliable: false,
            collision_enabled: true,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            crossing_sweep_travel: Real::MAX,
            crossed_partners: Vec::new(),
            vertex_cache: Vec::new(),
            topology_version: 0,
        }
    }

    /// A wire a body collides through: its segments over the particles, one vertex per particle
    /// (vertex indices are particle indices, like the boundary mesh). A wire is a curve that
    /// encloses nothing, so it is never closed and its contacts are always two-sided.
    #[cfg(feature = "dim3")]
    pub(crate) fn wire(
        body: &SoftBody,
        segments: &[[u32; 2]],
        self_contacts: bool,
        collision_enabled: bool,
    ) -> Self {
        let indices: Vec<[u32; DIM]> = segments
            .iter()
            .map(|s| core::array::from_fn(|k| if k < 2 { s[k] } else { u32::MAX }))
            .collect();
        let mut mesh = Self::new(
            SoftMeshMapping::Direct {
                particles: (0..body.particles.len() as u32).collect(),
            },
            indices,
            body.particles.iter().map(|p| p.position).collect(),
            body,
            self_contacts,
        );
        mesh.collision_enabled = collision_enabled;
        mesh
    }

    /// A mesh bound to the body's cells: for every vertex, the closest cell and the vertex's
    /// barycentric coordinates in it. `positions` are the particle positions the cells are in
    /// when the binding is taken, which the mesh's vertices are expected to match.
    pub(crate) fn skinned(
        vertices: &[Vector],
        indices: &[[u32; DIM]],
        cells: &[SoftBodyCell],
        positions: &[Vector],
        self_contacts: bool,
        collision_enabled: bool,
    ) -> Option<Self> {
        let bindings = bind_to_cells(vertices, indices, cells, positions)?;
        let (ring_offsets, ring) = surface_rings(vertices.len(), indices);
        let (vertex_elements_offsets, vertex_elements) =
            surface_vertex_elements(vertices.len(), indices);
        #[cfg(feature = "dim3")]
        let (edges, element_edges, edge_owners) = super::soft_body_builder::surface_edge_table(indices);

        Some(Self {
            binding: SoftMeshMapping::Skinned { bindings },
            id: SoftMeshId { cluster: 0, mesh: 0 },
            follows_boundary: false,
            closed: surface_is_closed(indices),
            rest_signed_volume: SoftBody::boundary_volume(indices, |i| vertices[i as usize]),
            element_cells: Vec::new(),
            indices: indices.to_vec(),
            vertices: vertices.to_vec(),
            ring_offsets,
            ring,
            vertex_elements_offsets,
            vertex_elements,
            #[cfg(feature = "dim3")]
            edges,
            #[cfg(feature = "dim3")]
            element_edges,
            #[cfg(feature = "dim3")]
            edge_owners,
            inverted: false,
            orientation_unreliable: false,
            collision_enabled,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            crossing_sweep_travel: Real::MAX,
            crossed_partners: Vec::new(),
            vertex_cache: Vec::new(),
            topology_version: 0,
        })
    }

    /// The vertices of this mesh's collider shape, expressed in the collider's frame (`pose` is
    /// its world pose: the cluster proxy's pose composed with the collider's pose relative to
    /// it). A rigid motion of the body moves the frame, not the vertices.
    pub(crate) fn local_vertices(&self, body: &SoftBody, pose: &Pose) -> Vec<Vector> {
        (0..self.vertex_count())
            .map(|i| pose.inverse_transform_point(self.vertex(body, i)))
            .collect()
    }

    /// Moves the vertices of this mesh's collider shape to the current positions, in place:
    /// parry refits the shape's BVH.
    pub(crate) fn deform_shape(
        &self,
        body: &SoftBody,
        pose: &Pose,
        shape: &mut dyn parry::shape::Shape,
    ) {
        let write = |vertices: &mut [Vector]| {
            debug_assert_eq!(vertices.len(), self.vertex_count());
            for (i, v) in vertices.iter_mut().enumerate() {
                *v = pose.inverse_transform_point(self.vertex(body, i));
            }
        };
        // A polyline in either dimension (a 2D surface, or a wire in 3D), a trimesh in 3D.
        if let Some(polyline) = shape.as_polyline_mut() {
            polyline.update_vertices(write);
            return;
        }
        #[cfg(feature = "dim3")]
        if let Some(trimesh) = shape.as_trimesh_mut() {
            trimesh.update_vertices(write);
        }
    }

    /// Moves a skinned mesh to the current particle positions (a direct mesh reads them
    /// directly and has nothing to update).
    pub(crate) fn update(&mut self, cells: &[SoftBodyCell], particles: &[super::SoftBodyParticle]) {
        let SoftMeshMapping::Skinned { bindings } = &self.binding else {
            return;
        };
        for (vertex, binding) in self.vertices.iter_mut().zip(bindings) {
            let Some(cell) = cells.get(binding.cell as usize) else {
                continue;
            };
            let mut skinned = Vector::ZERO;
            for (vid, weight) in cell.vertices.iter().zip(&binding.weights) {
                skinned += particles[*vid as usize].position * *weight;
            }
            *vertex = skinned;
        }
    }

    /// Updates the inside-out state of a closed mesh from the current vertex positions.
    pub(crate) fn update_orientation(&mut self, body: &SoftBody) {
        if !self.closed || self.rest_signed_volume == 0.0 {
            return;
        }
        // With hysteresis: a crumpled body hovering around a zero volume must not flip its
        // orientation (contact normals) every step.
        let volume = SoftBody::boundary_volume(&self.indices, |i| self.vertex(body, i as usize));
        let ratio = volume / self.rest_signed_volume;
        if ratio < -0.25 {
            self.inverted = true;
        } else if ratio > 0.25 {
            self.inverted = false;
        }
        // Inside the band the loop is likely self-crossed (an O squeezed into an 8): part of the
        // winding is mirrored, so no global orientation is trustworthy.
        self.orientation_unreliable = ratio.abs() <= 0.25;
    }

    /// A counter bumped whenever this mesh's topology changes: a renderer keying its mesh on it
    /// rebuilds only what changed.
    pub fn topology_version(&self) -> u32 {
        self.topology_version
    }

    /// Rebuilds the tables derived from the elements after a topology change (a tear).
    pub(crate) fn rebuild_tables(&mut self, body: &SoftBody) {
        self.topology_version = self.topology_version.wrapping_add(1);
        let num_vertices = self.vertex_count();
        let (offsets, ring) = surface_rings(num_vertices, &self.indices);
        self.ring_offsets = offsets;
        self.ring = ring;
        let (offsets, elements) = surface_vertex_elements(num_vertices, &self.indices);
        self.vertex_elements_offsets = offsets;
        self.vertex_elements = elements;
        self.closed = surface_is_closed(&self.indices);
        if !self.is_skinned() && !self.follows_boundary {
            self.element_cells = surface_element_cells(&self.indices, &body.cells);
        }
        #[cfg(feature = "dim3")]
        {
            let (edges, element_edges, owners) =
                super::soft_body_builder::surface_edge_table(&self.indices);
            self.edges = edges;
            self.element_edges = element_edges;
            self.edge_owners = owners;
        }
    }

    /// Follows the body through a topology change (tear, cut, cluster removal): a boundary mesh is
    /// re-derived, a direct mesh follows its particles ([`follow_insertions`], [`follow_splits`]),
    /// a skinned mesh follows its cells and comes apart along the cracks ([`follow_pieces`]).
    pub(crate) fn remap_topology(&mut self, body: &SoftBody, remap: &SoftTopologyRemap) {
        match &mut self.binding {
            SoftMeshMapping::Skinned { bindings } => {
                if !remap.cells.is_empty() {
                    for binding in bindings.iter_mut() {
                        binding.cell = remap
                            .cells
                            .get(binding.cell as usize)
                            .copied()
                            .unwrap_or(u32::MAX);
                    }
                }
                let dangling: Vec<usize> = bindings
                    .iter()
                    .enumerate()
                    .filter(|(_, binding)| body.cells.get(binding.cell as usize).is_none())
                    .map(|(i, _)| i)
                    .collect();
                for vertex in dangling {
                    let position = self.vertices[vertex];
                    if let Some(binding) = closest_cell_binding(body, position) {
                        bindings[vertex] = binding;
                    }
                }
            }
            SoftMeshMapping::Direct { particles } if self.follows_boundary => {
                self.indices = body.boundary.clone();
                *particles = (0..body.particles.len() as u32).collect();
                self.rest_signed_volume = SoftBody::boundary_volume(&self.indices, |i| {
                    body.particles[i as usize].rest_position
                });
            }
            SoftMeshMapping::Direct { particles } => {
                if remap.particles.is_empty() {
                    return;
                }
                let mut vertex_remap = vec![u32::MAX; particles.len()];
                let mut kept = Vec::with_capacity(particles.len());
                for (vertex, particle) in particles.iter().enumerate() {
                    let new = remap.particles.get(*particle as usize).copied();
                    if let Some(new) = new.filter(|p| *p != u32::MAX) {
                        vertex_remap[vertex] = kept.len() as u32;
                        kept.push(new);
                    }
                }
                *particles = kept;
                self.indices.retain(|element| {
                    element
                        .iter()
                        .all(|v| vertex_remap[*v as usize] != u32::MAX)
                });
                for element in &mut self.indices {
                    for v in element.iter_mut() {
                        *v = vertex_remap[*v as usize];
                    }
                }
            }
        }
    }

    /// Clears the contact state kept across steps (a topology change invalidated its ids).
    pub(crate) fn clear_contacts(&mut self) {
        self.edge_contacts.clear();
        self.vertex_contacts.clear();
        self.crossing_sweep_travel = Real::MAX;
        self.crossed_partners.clear();
    }
}
