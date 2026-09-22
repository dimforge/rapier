//! Read accessors of a collision mesh: identity, elements and edges, vertices and the particles
//! a contact at a vertex acts through.

use crate::geometry::ColliderHandle;
use crate::math::{DIM, Real, Vector};

use super::{SoftBody, SoftCollisionMesh, SoftMeshCellBinding, SoftMeshId, SoftMeshMapping};

impl SoftCollisionMesh {
    /// Where this mesh lives in its soft body.
    pub fn id(&self) -> SoftMeshId {
        self.id
    }

    /// Records where this mesh was put (called when it joins a cluster).
    pub(crate) fn set_id(&mut self, id: SoftMeshId) {
        self.id = id;
    }

    /// The collider holding this mesh's shape (invalid for a mesh that is only drawn).
    pub fn collider(&self) -> ColliderHandle {
        self.collider
    }

    /// Whether this mesh collides (`false`: it is only drawn).
    pub fn collision_enabled(&self) -> bool {
        self.collision_enabled
    }

    /// The mesh's elements: segments in 2D, triangles in 3D, or segments in 3D for a wire
    /// (`u32::MAX` fills the unused slot then; see [`Self::element`]).
    pub fn indices(&self) -> &[[u32; DIM]] {
        &self.indices
    }

    /// The vertices of the `i`-th element: two for a segment, `DIM` for a facet.
    #[inline]
    pub fn element(&self, i: usize) -> &[u32] {
        super::soft_body_builder::element_vertices(&self.indices[i])
    }

    /// How many vertices this mesh's elements have: `2` for a wire (a rope in 3D, and every
    /// mesh in 2D), `DIM` for a surface.
    #[inline]
    pub fn arity(&self) -> usize {
        self.indices
            .first()
            .map_or(DIM, |e| super::soft_body_builder::element_vertices(e).len())
    }

    /// Whether this mesh is a wire (segments in 3D): a curve, with no inside and no facets.
    #[inline]
    pub fn is_wire(&self) -> bool {
        self.arity() < DIM
    }

    /// The edges the `i`-th element owns: its three edges for a triangle, itself for a segment.
    pub(crate) fn element_edge_ids(&self, i: usize) -> impl Iterator<Item = u32> + '_ {
        #[cfg(feature = "dim3")]
        let ids: &[u32] = if self.is_wire() {
            &[]
        } else {
            super::soft_body_builder::element_vertices(&self.element_edges[i])
        };
        #[cfg(feature = "dim2")]
        let ids: &[u32] = &[];
        let own = ids.is_empty().then_some(i as u32);
        own.into_iter().chain(ids.iter().copied())
    }

    /// The two vertices of an edge id (see [`Self::element_edge_ids`]).
    pub(crate) fn edge_vertices(&self, edge: u32) -> [u32; 2] {
        #[cfg(feature = "dim3")]
        if !self.is_wire() {
            return self.edges[edge as usize];
        }
        let element = self.element(edge as usize);
        [element[0], element[1]]
    }

    /// The element owning an edge: a segment owns itself.
    pub(crate) fn edge_owner(&self, edge: u32) -> u32 {
        #[cfg(feature = "dim3")]
        if !self.is_wire() {
            return self.edge_owners[edge as usize];
        }
        edge
    }

    /// How this mesh's vertices follow the computational mesh.
    pub fn binding(&self) -> &SoftMeshMapping {
        &self.binding
    }

    /// Whether this mesh rides the body's cells rather than its particles.
    pub fn is_skinned(&self) -> bool {
        matches!(self.binding, SoftMeshMapping::Skinned { .. })
    }

    /// Whether this mesh is closed (every segment vertex / triangle edge shared by exactly two
    /// elements).
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Whether this mesh's collider shape carries parry's `ORIENTED` flag, as of the last step.
    pub fn is_oriented(&self) -> bool {
        self.oriented
    }

    /// Whether this mesh encloses solid matter: closed and oriented. Its contacts with other
    /// bodies are then oriented outward and nothing is held inside it; otherwise it is two-sided.
    pub fn is_solid(&self) -> bool {
        self.closed && self.oriented
    }

    /// Reads the orientation back from the collider's shape, which is the authority on it.
    pub(crate) fn read_orientation(&mut self, colliders: &crate::geometry::ColliderSet) {
        let Some(co) = colliders.get(self.collider) else {
            return;
        };
        #[cfg(feature = "dim2")]
        let oriented = co.shape().as_polyline().is_some_and(|polyline| {
            polyline
                .flags()
                .contains(parry::shape::PolylineFlags::ORIENTED)
        });
        #[cfg(feature = "dim3")]
        let oriented = co.shape().as_trimesh().is_some_and(|trimesh| {
            trimesh
                .flags()
                .contains(parry::shape::TriMeshFlags::ORIENTED)
        });
        self.oriented = oriented;
    }

    /// Whether this mesh collides with itself.
    pub fn self_contacts_enabled(&self) -> bool {
        self.self_contacts
    }

    /// The number of vertices of this mesh.
    pub fn vertex_count(&self) -> usize {
        match &self.binding {
            SoftMeshMapping::Direct { particles } => particles.len(),
            SoftMeshMapping::Skinned { .. } => self.vertices.len(),
        }
    }

    /// The cached world-space vertices of a skinned mesh (empty for a direct mesh, whose
    /// vertices are its particles: see [`Self::vertex_positions`]).
    pub fn vertices(&self) -> &[Vector] {
        &self.vertices
    }

    /// The world-space position of every vertex of this mesh.
    pub fn vertex_positions<'a>(
        &'a self,
        body: &'a SoftBody,
    ) -> impl ExactSizeIterator<Item = Vector> + 'a {
        (0..self.vertex_count()).map(move |i| self.vertex(body, i))
    }

    /// The world position of the `i`-th vertex.
    pub fn vertex(&self, body: &SoftBody, i: usize) -> Vector {
        match &self.binding {
            SoftMeshMapping::Direct { particles } => body.particles[particles[i] as usize].position,
            SoftMeshMapping::Skinned { .. } => self.vertices[i],
        }
    }

    /// The `i`-th vertex as cached by [`Self::refresh_vertex_cache`] (the narrow-phase passes
    /// read the cache, refreshed right before every narrow-phase update).
    #[inline]
    pub(crate) fn cached_vertex(&self, i: usize) -> Vector {
        self.vertex_cache[i]
    }

    /// Refreshes the vertex cache from the particles (see [`Self::cached_vertex`]).
    pub(crate) fn refresh_vertex_cache(&mut self, particles: &[super::super::SoftBodyParticle]) {
        match &self.binding {
            SoftMeshMapping::Direct { particles: map } => {
                self.vertex_cache.clear();
                self.vertex_cache
                    .extend(map.iter().map(|&p| particles[p as usize].position));
            }
            SoftMeshMapping::Skinned { .. } => {
                self.vertex_cache.clear();
                self.vertex_cache.extend_from_slice(&self.vertices);
            }
        }
    }

    /// The world velocity of the `i`-th vertex: a skinned vertex moves with the cell holding it.
    pub fn vertex_velocity(&self, body: &SoftBody, i: usize) -> Vector {
        match &self.binding {
            SoftMeshMapping::Direct { particles } => body.particles[particles[i] as usize].velocity,
            SoftMeshMapping::Skinned { bindings } => {
                let binding = &bindings[i];
                let Some(cell) = body.cells.get(binding.cell as usize) else {
                    return Vector::ZERO;
                };
                let mut velocity = Vector::ZERO;
                for (vid, weight) in cell.vertices.iter().zip(&binding.weights) {
                    velocity += body.particles[*vid as usize].velocity * *weight;
                }
                velocity
            }
        }
    }

    /// Whether the `i`-th vertex belongs to any element of this mesh.
    pub(crate) fn vertex_on_surface(&self, i: usize) -> bool {
        match (
            self.vertex_elements_offsets.get(i),
            self.vertex_elements_offsets.get(i + 1),
        ) {
            (Some(start), Some(end)) => end > start,
            _ => false,
        }
    }

    /// The particles a contact at the `i`-th vertex acts through, and their weights: the
    /// vertex's own particle for a direct mesh, the particles of the cell holding it for a
    /// skinned one.
    pub fn vertex_anchors(&self, body: &SoftBody, i: usize) -> ([u32; DIM + 1], [Real; DIM + 1]) {
        match &self.binding {
            SoftMeshMapping::Direct { particles } => {
                let mut ids = [u32::MAX; DIM + 1];
                let mut weights = [0.0; DIM + 1];
                ids[0] = particles[i];
                weights[0] = 1.0;
                (ids, weights)
            }
            SoftMeshMapping::Skinned { bindings } => {
                match body.cells.get(bindings[i].cell as usize) {
                    Some(cell) => (cell.vertices, bindings[i].weights),
                    None => ([u32::MAX; DIM + 1], [0.0; DIM + 1]),
                }
            }
        }
    }

    /// The particles a contact acts through and their weights (the element's own vertices for a
    /// direct mesh, the bound cell's particles for a skinned one), plus the point the constraint
    /// tracks; `weights` are the contact point's barycentric coordinates in `element`.
    pub(crate) fn contact_anchors(
        &self,
        body: &SoftBody,
        vertices: &[u32],
        weights: &[Real],
        point: Vector,
    ) -> ([u32; DIM + 1], [Real; DIM + 1], Vector) {
        let pad = |vertices: &[u32]| {
            let mut ids = [u32::MAX; DIM + 1];
            let mut ws = [0.0; DIM + 1];
            let len = vertices.len().min(DIM + 1);
            ids[..len].copy_from_slice(&vertices[..len]);
            ws[..len].copy_from_slice(&weights[..len]);
            (ids, ws, point)
        };

        match &self.binding {
            SoftMeshMapping::Direct { particles } => {
                let mut ids = [u32::MAX; DIM + 1];
                let mut ws = [0.0; DIM + 1];
                for k in 0..vertices.len().min(DIM + 1) {
                    ids[k] = particles[vertices[k] as usize];
                    ws[k] = weights[k];
                }
                (ids, ws, point)
            }
            SoftMeshMapping::Skinned { bindings } => self
                .skinned_contact_anchors(body, bindings, vertices, weights, point)
                .unwrap_or_else(|| pad(vertices)),
        }
    }

    fn skinned_contact_anchors(
        &self,
        body: &SoftBody,
        bindings: &[SoftMeshCellBinding],
        vertices: &[u32],
        weights: &[Real],
        point: Vector,
    ) -> Option<([u32; DIM + 1], [Real; DIM + 1], Vector)> {
        let first = bindings.get(*vertices.first()? as usize)?;
        let shared = vertices
            .iter()
            .all(|v| bindings[*v as usize].cell == first.cell);

        if shared {
            let cell = body.cells.get(first.cell as usize)?;
            let mut blended = [0.0; DIM + 1];
            for (v, weight) in vertices.iter().zip(weights) {
                for (w, bw) in blended.iter_mut().zip(&bindings[*v as usize].weights) {
                    *w += bw * weight;
                }
            }
            return Some((cell.vertices, blended, point));
        }

        let heaviest = (0..vertices.len()).max_by(|a, b| weights[*a].total_cmp(&weights[*b]))?;
        let vertex = vertices[heaviest] as usize;
        let binding = &bindings[vertex];
        let cell = body.cells.get(binding.cell as usize)?;
        Some((cell.vertices, binding.weights, self.vertices[vertex]))
    }

    /// The rest position of the `i`-th vertex (relative to the body's rest center of mass,
    /// like `SoftBodyParticle::rest_position`).
    pub(crate) fn rest_vertex(&self, body: &SoftBody, i: usize) -> Vector {
        match &self.binding {
            SoftMeshMapping::Direct { particles } => {
                body.particles[particles[i] as usize].rest_position
            }
            SoftMeshMapping::Skinned { bindings } => {
                let binding = &bindings[i];
                let Some(cell) = body.cells.get(binding.cell as usize) else {
                    return Vector::ZERO;
                };
                let mut position = Vector::ZERO;
                for (vid, weight) in cell.vertices.iter().zip(&binding.weights) {
                    position += body.particles[*vid as usize].rest_position * *weight;
                }
                position
            }
        }
    }

    /// The cell id backing each element, `u32::MAX` for none (empty when the elements are not
    /// cell-backed at all: wires, skinned meshes).
    pub(crate) fn element_cell_ids<'a>(&'a self, body: &'a SoftBody) -> &'a [u32] {
        if self.follows_boundary {
            &body.boundary_element_cells
        } else {
            &self.element_cells
        }
    }

    /// The surface elements incident to `vertex`.
    pub(crate) fn vertex_element_ids(&self, vertex: u32) -> &[u32] {
        match (
            self.vertex_elements_offsets.get(vertex as usize),
            self.vertex_elements_offsets.get(vertex as usize + 1),
        ) {
            (Some(&start), Some(&end)) => &self.vertex_elements[start as usize..end as usize],
            _ => &[],
        }
    }
}
