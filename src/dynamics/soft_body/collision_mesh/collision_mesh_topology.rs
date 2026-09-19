//! Construction of a collision mesh, its topology tables, and the per-step update of its
//! vertices, orientation and collider shape.

use crate::alloc_prelude::*;
use crate::geometry::ColliderHandle;
use crate::math::{DIM, Pose, Real, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::collision_mesh_binding::{bind_to_cells, closest_cell_binding};
use super::soft_body_builder::{element_vertices, element_vertices_mut};
use super::{
    SoftBody, SoftBodyCell, SoftCollisionMesh, SoftMeshCellBinding, SoftMeshId, SoftMeshMapping,
    SoftTopologyRemap, surface_element_cells, surface_is_closed, surface_rings,
    surface_vertex_elements,
};

impl SoftCollisionMesh {
    /// The mesh a body collides through when it collides through its own boundary: one vertex
    /// per particle (interior particles included, so vertex indices are particle indices), the
    /// cells' boundary as elements.
    pub(crate) fn boundary(body: &SoftBody, self_contacts: bool, collision_enabled: bool) -> Self {
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
            id: SoftMeshId {
                cluster: 0,
                mesh: 0,
            },
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
            oriented: false,
            collision_enabled,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            overlap_states: Vec::new(),
            overlap_warm: Vec::new(),
            volume_contacts: Vec::new(),
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
            oriented: false,
            collision_enabled: true,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            overlap_states: Vec::new(),
            overlap_warm: Vec::new(),
            volume_contacts: Vec::new(),
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
        let (edges, element_edges, edge_owners) =
            super::soft_body_builder::surface_edge_table(indices);

        Some(Self {
            binding: SoftMeshMapping::Skinned { bindings },
            id: SoftMeshId {
                cluster: 0,
                mesh: 0,
            },
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
            oriented: false,
            collision_enabled,
            collider: ColliderHandle::invalid(),
            self_contacts,
            edge_contacts: Vec::new(),
            vertex_contacts: Vec::new(),
            overlap_states: Vec::new(),
            overlap_warm: Vec::new(),
            volume_contacts: Vec::new(),
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
        #[cfg(feature = "dim3")]
        if let Some(trimesh) = shape.as_trimesh_mut() {
            trimesh.update_vertices(write);
            return;
        }
        // A polyline in either dimension: a 2D surface, or a wire in 3D.
        if let Some(polyline) = shape.as_polyline_mut() {
            polyline.update_vertices(write);
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
                    if let Some(binding) = closest_cell_binding(body, position, |_| true) {
                        bindings[vertex] = binding;
                    }
                }
                if !remap.split.is_empty() {
                    follow_pieces(bindings, &mut self.vertices, &mut self.indices, body);
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
                if !remap.particles.is_empty() {
                    // The vertices whose particle died go with it, and so do the elements they
                    // were in; the survivors are compacted.
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
                    // A wire element pads its unused slot with `u32::MAX`: only the used
                    // vertices are looked up.
                    self.indices.retain(|element| {
                        element_vertices(element)
                            .iter()
                            .all(|v| vertex_remap[*v as usize] != u32::MAX)
                    });
                    for element in &mut self.indices {
                        for v in element_vertices_mut(element) {
                            *v = vertex_remap[*v as usize];
                        }
                    }
                }
                if !remap.inserted.is_empty() {
                    follow_insertions(particles, &mut self.indices, remap.inserted);
                }
                if !remap.split.is_empty() {
                    follow_splits(particles, &mut self.indices, body, remap.split);
                }
            }
        }
    }

    /// A copy of this mesh over the vertices `keep` accepts: other vertices and their elements
    /// dropped, tables rebuilt, contact state cleared, no collider or id. `None` when no element
    /// remains; how a mesh follows a cluster split.
    pub(crate) fn restricted_to(
        &self,
        body: &SoftBody,
        keep: impl Fn(u32) -> bool,
    ) -> Option<Self> {
        let n = self.vertex_count();
        let mut vertex_remap = vec![u32::MAX; n];
        let mut kept = 0u32;
        for v in 0..n {
            if keep(v as u32) {
                vertex_remap[v] = kept;
                kept += 1;
            }
        }
        let indices: Vec<[u32; DIM]> = self
            .indices
            .iter()
            .filter(|element| {
                element_vertices(element)
                    .iter()
                    .all(|v| vertex_remap[*v as usize] != u32::MAX)
            })
            .map(|element| {
                let mut element = *element;
                for v in element_vertices_mut(&mut element) {
                    *v = vertex_remap[*v as usize];
                }
                element
            })
            .collect();
        if indices.is_empty() {
            return None;
        }
        let mut mesh = self.clone();
        mesh.indices = indices;
        let mut keep = (0..n).map(|v| vertex_remap[v] != u32::MAX);
        match &mut mesh.binding {
            SoftMeshMapping::Direct { particles } => particles.retain(|_| keep.next().unwrap()),
            SoftMeshMapping::Skinned { bindings } => {
                let mut keep_vertices = keep.clone();
                bindings.retain(|_| keep.next().unwrap());
                mesh.vertices.retain(|_| keep_vertices.next().unwrap());
            }
        }
        mesh.collider = ColliderHandle::invalid();
        mesh.id = SoftMeshId::default();
        mesh.clear_contacts();
        mesh.overlap_states.clear();
        mesh.overlap_warm.clear();
        mesh.volume_contacts.clear();
        mesh.rebuild_tables(body);
        mesh.rest_signed_volume = if mesh.is_wire() {
            0.0
        } else {
            SoftBody::boundary_volume(&mesh.indices, |i| mesh.rest_vertex(body, i as usize))
        };
        mesh.update_orientation(body);
        Some(mesh)
    }

    /// Clears the contact state kept across steps (a topology change invalidated its ids).
    pub(crate) fn clear_contacts(&mut self) {
        self.edge_contacts.clear();
        self.vertex_contacts.clear();
        self.crossing_sweep_travel = Real::MAX;
        self.crossed_partners.clear();
    }
}

/// Follows the particles a cut inserted into segments (`inserted`: `[a, b, p, q]`, segment `(a, b)`
/// became `(a, p)` and `(q, b)`) in a direct mesh (`particles`: the particle of each vertex): a
/// mesh segment over `a` and `b` is split the same way, with a new vertex per inserted particle.
fn follow_insertions(
    particles: &mut Vec<u32>,
    indices: &mut Vec<[u32; DIM]>,
    inserted: &[[u32; 4]],
) {
    for &[a, b, p, q] in inserted {
        for i in 0..indices.len() {
            let element = indices[i];
            if element_vertices(&element).len() != 2 {
                continue;
            }
            let ends = [
                particles[element[0] as usize],
                particles[element[1] as usize],
            ];
            let (first, second) = if ends == [a, b] {
                (p, q)
            } else if ends == [b, a] {
                (q, p)
            } else {
                continue;
            };
            particles.push(first);
            particles.push(second);
            let n = particles.len() as u32;
            indices[i][1] = n - 2;
            let mut rest = element;
            rest[0] = n - 1;
            indices.push(rest);
        }
    }
}

/// Separates a skinned mesh along the cracks between the body's pieces: in an element whose
/// vertices ride cells of different pieces, each vertex outside the majority piece is replaced by
/// a copy bound to the closest cell of that piece (one shared copy per vertex and piece).
fn follow_pieces(
    bindings: &mut Vec<SoftMeshCellBinding>,
    vertices: &mut Vec<Vector>,
    indices: &mut [[u32; DIM]],
    body: &SoftBody,
) {
    let pieces = body.connected_pieces();
    if pieces.len() < 2 {
        return;
    }
    let mut particle_piece = vec![u32::MAX; body.particles.len()];
    for (k, piece) in pieces.iter().enumerate() {
        for &p in piece {
            particle_piece[p as usize] = k as u32;
        }
    }
    let cell_piece = |cell: u32| {
        body.cells
            .get(cell as usize)
            .map_or(u32::MAX, |c| particle_piece[c.vertices[0] as usize])
    };
    let mut copies: HashMap<(u32, u32), u32> = HashMap::default();
    for element in indices.iter_mut() {
        let used = element_vertices(element).len();
        let piece_of: Vec<u32> = element[..used]
            .iter()
            .map(|&w| {
                bindings
                    .get(w as usize)
                    .map_or(u32::MAX, |b| cell_piece(b.cell))
            })
            .collect();
        // The most frequent piece, ties going to the one met first.
        let mut majority = (u32::MAX, 0);
        for &piece in piece_of.iter().filter(|&&piece| piece != u32::MAX) {
            let count = piece_of.iter().filter(|&&other| other == piece).count();
            if count > majority.1 {
                majority = (piece, count);
            }
        }
        let majority = majority.0;
        if piece_of
            .iter()
            .all(|&piece| piece == majority || piece == u32::MAX)
        {
            continue;
        }
        for k in 0..used {
            if piece_of[k] == majority || piece_of[k] == u32::MAX {
                continue;
            }
            let w = element[k];
            let copy = match copies.get(&(w, majority)) {
                Some(&copy) => copy,
                None => {
                    let position = vertices[w as usize];
                    let Some(binding) =
                        closest_cell_binding(body, position, |cell| cell_piece(cell) == majority)
                    else {
                        continue;
                    };
                    bindings.push(binding);
                    vertices.push(position);
                    let copy = vertices.len() as u32 - 1;
                    let _ = copies.insert((w, majority), copy);
                    copy
                }
            };
            element[k] = copy;
        }
    }
}

/// The largest number of particle combinations [`follow_splits`] tries for one element.
const MAX_SPLIT_COMBINATIONS: usize = 64;

/// Follows particle splits (`split`: `(copy, source)` in creation order) in a direct mesh
/// (`particles`: the particle of each vertex): an element whose particles no longer share a
/// measure element of `body` switches to the combination of copies that does, adding vertices.
fn follow_splits(
    particles: &mut Vec<u32>,
    indices: &mut [[u32; DIM]],
    body: &SoftBody,
    split: &[(u32, u32)],
) {
    // Every split particle's family: the particle it was first split from, then its copies.
    let mut root: HashMap<u32, u32> = HashMap::default();
    let mut families: HashMap<u32, Vec<u32>> = HashMap::default();
    for &(copy, source) in split {
        let first = root.get(&source).copied().unwrap_or(source);
        root.insert(copy, first);
        families
            .entry(first)
            .or_insert_with(|| vec![first])
            .push(copy);
    }
    let mut vertex_of: HashMap<u32, u32> = HashMap::default();
    for (vertex, &particle) in particles.iter().enumerate() {
        vertex_of.entry(particle).or_insert(vertex as u32);
    }
    for element in indices.iter_mut() {
        let used = element_vertices(element).len();
        let current: Vec<u32> = element[..used]
            .iter()
            .map(|&w| particles[w as usize])
            .collect();
        let candidates: Vec<&[u32]> = current
            .iter()
            .map(|p| {
                let first = root.get(p).copied().unwrap_or(*p);
                families
                    .get(&first)
                    .map_or(core::slice::from_ref(p), Vec::as_slice)
            })
            .collect();
        let combinations: usize = candidates.iter().map(|c| c.len()).product();
        if combinations == 1
            || combinations > MAX_SPLIT_COMBINATIONS
            || body.measure_element_holds(&current)
        {
            continue;
        }
        for index in 0..combinations {
            let mut digits = index;
            let combination: Vec<u32> = candidates
                .iter()
                .map(|c| {
                    let p = c[digits % c.len()];
                    digits /= c.len();
                    p
                })
                .collect();
            if body.measure_element_holds(&combination) {
                for (k, &p) in combination.iter().enumerate() {
                    if p != current[k] {
                        element[k] = *vertex_of.entry(p).or_insert_with(|| {
                            particles.push(p);
                            particles.len() as u32 - 1
                        });
                    }
                }
                break;
            }
        }
    }
}
