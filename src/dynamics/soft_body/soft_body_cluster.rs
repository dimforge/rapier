//! Soft-body clusters: weighted particle sets, each backed by a proxy rigid body
//! ([`crate::dynamics::RigidBodyType::SoftFrame`]) standing for it in the islands and joints.
//! Index 0 is the whole-body cluster; removing a cluster deletes the particles it alone covers.

use crate::alloc_prelude::*;
use crate::dynamics::RigidBodyHandle;
use crate::math::{AngVector, AngularInertia, DIM, Pose, Real, Rotation, Vector};
use crate::utils::{AngularInertiaOps, CrossProduct};

use super::SoftBody;
use super::soft_body_shape_matching::extract_rotation;

/// A cluster of a soft body: a set of its particles backed by a proxy rigid body
/// ([`crate::dynamics::RigidBodyType::SoftFrame`]) carrying its frame, gathered velocity and
/// reduced mass. A rank-deficient one (one particle, or collinear in 3D) has no angular response.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyCluster {
    /// The particles of the cluster (sorted, unique). Empty for a removed cluster slot.
    pub(crate) particles: Vec<u32>,
    /// The cluster's proxy rigid body (invalid for a removed cluster slot).
    pub(crate) proxy: RigidBodyHandle,
    /// Warm-started rotation of the cluster's frame fit.
    // NOTE: read once the frames go live (the joint phase); stored from the start so the
    // serialization format is stable.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) rotation: Rotation,
    /// The cell whose vertices are exactly this cluster's particles, if any: the frame rotation
    /// is then the cell's corotational rotation (exact and free) instead of a shape-match fit.
    #[cfg_attr(feature = "serde-serialize", serde(default = "invalid_cell"))]
    pub(crate) cell: u32,
    /// Whether this cluster's particles are shape-matched toward the cluster's frame.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) shape_matching: bool,
    pub(crate) shape_matching_target: Option<Pose>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) prev_shape_matching_target: Option<Pose>,
    /// The rigid velocity gathered from the particles when the proxy was last updated: the
    /// proxy's `rb.vels` was set to exactly this, so `rb.vels - last_gather` is the external
    /// impulse the user applied to the proxy since (scattered by the solver's first pass).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) last_gather: (Vector, AngVector),
    /// Warm-start impulses of this cluster's shape-matching constraints, parallel to `particles`
    /// (empty until the cluster is shape-matched).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) shape_impulses: Vec<Vector>,
    /// The collision meshes this cluster owns (see [`super::SoftCollisionMesh`]). Removed
    /// meshes leave a `None` slot so ids stay stable.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) meshes: Vec<Option<super::SoftCollisionMesh>>,
}

#[cfg(feature = "serde-serialize")]
fn invalid_cell() -> u32 {
    u32::MAX
}

impl SoftBodyCluster {
    pub fn new(particles: Vec<u32>, proxy: RigidBodyHandle, shape_matching: bool) -> Self {
        SoftBodyCluster {
            particles,
            proxy,
            rotation: Rotation::IDENTITY,
            cell: u32::MAX,
            shape_matching,
            shape_matching_target: None,
            prev_shape_matching_target: None,
            last_gather: Default::default(),
            shape_impulses: vec![],
            meshes: Vec::new(),
        }
    }

    /// Whether this cluster slot is live (not removed).
    #[inline]
    pub fn is_live(&self) -> bool {
        !self.proxy.is_invalid()
    }

    /// The particles of this cluster (sorted, unique).
    pub fn particles(&self) -> &[u32] {
        &self.particles
    }

    /// The cluster's proxy rigid body.
    pub fn proxy(&self) -> RigidBodyHandle {
        self.proxy
    }

    /// The collision meshes this cluster owns mutably (dead slots skipped).
    pub(crate) fn meshes_mut(&mut self) -> impl Iterator<Item = &mut super::SoftCollisionMesh> {
        self.meshes.iter_mut().flatten()
    }

    /// The collision meshes this cluster owns (dead slots skipped).
    pub fn meshes(&self) -> impl Iterator<Item = &super::SoftCollisionMesh> {
        self.meshes.iter().flatten()
    }

    /// The `i`-th mesh of this cluster, if it is live.
    pub fn mesh(&self, i: u32) -> Option<&super::SoftCollisionMesh> {
        self.meshes.get(i as usize).and_then(|mesh| mesh.as_ref())
    }

    /// Whether this cluster's particles are shape-matched toward its frame.
    pub fn shape_matching_enabled(&self) -> bool {
        self.shape_matching
    }

    pub fn set_shape_matching_target(&mut self, target: Option<Pose>) {
        self.shape_matching_target = target;
    }
}

/// What [`crate::dynamics::SoftBodySet::remove_cluster`] (or removing a proxy rigid body)
/// deleted: the particles only the removed cluster covered, and everything attached to them.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SoftClusterRemoval {
    /// Particles removed (they belonged to no other cluster).
    pub particles: usize,
    /// Distance constraints removed with them.
    pub edges: usize,
    /// Cells removed with them.
    pub cells: usize,
    /// Dihedral bending constraints removed with them (3D).
    pub dihedrals: usize,
    /// Surface elements removed with them.
    pub surface_elements: usize,
    /// Particle attachments removed with them.
    pub attachments: usize,
    /// Whether the soft body itself was removed (its last cluster is gone).
    pub soft_body_removed: bool,
}

impl SoftBody {
    /// The clusters of this soft body (index 0 is the whole-body cluster created at insertion).
    /// Removed cluster slots stay as dead entries (`SoftBodyCluster::is_live`); indices are
    /// stable.
    pub fn clusters(&self) -> &[SoftBodyCluster] {
        &self.clusters
    }

    /// The `i`-th cluster, if it exists and is live.
    pub fn cluster(&self, i: u32) -> Option<&SoftBodyCluster> {
        self.clusters.get(i as usize).filter(|c| c.is_live())
    }

    /// The `i`-th cluster, if it exists and is live.
    pub fn cluster_mut(&mut self, i: u32) -> Option<&mut SoftBodyCluster> {
        let result = self.clusters.get_mut(i as usize).filter(|c| c.is_live());
        self.modified |= result.is_some();
        result
    }

    /// The proxy rigid body of the `i`-th cluster, if it is live.
    pub fn cluster_proxy(&self, i: u32) -> Option<RigidBodyHandle> {
        self.cluster(i).map(|c| c.proxy)
    }

    /// Iterates over the live clusters as `(index, cluster)`.
    pub fn live_clusters(&self) -> impl Iterator<Item = (u32, &SoftBodyCluster)> {
        self.clusters
            .iter()
            .enumerate()
            .filter(|(_, c)| c.is_live())
            .map(|(i, c)| (i as u32, c))
    }

    /// Number of live clusters.
    pub fn num_live_clusters(&self) -> usize {
        self.clusters.iter().filter(|c| c.is_live()).count()
    }

    /// Whether two of this body's clusters share no particle (a merge walk over their sorted
    /// particle lists; a dead or missing cluster shares nothing).
    pub(crate) fn clusters_are_disjoint(&self, a: u32, b: u32) -> bool {
        let (Some(a), Some(b)) = (self.cluster(a), self.cluster(b)) else {
            return true;
        };
        let (mut i, mut j) = (0, 0);
        let (a, b) = (a.particles(), b.particles());
        while i < a.len() && j < b.len() {
            match a[i].cmp(&b[j]) {
                core::cmp::Ordering::Less => i += 1,
                core::cmp::Ordering::Greater => j += 1,
                core::cmp::Ordering::Equal => return false,
            }
        }
        true
    }

    /// Enables or disables shape matching of the `i`-th cluster's particles toward the cluster's
    /// frame (independent from the whole-body [`Self::enable_shape_matching`]).
    pub fn enable_cluster_shape_matching(&mut self, i: u32, enabled: bool) {
        if let Some(c) = self.clusters.get_mut(i as usize) {
            if c.is_live() && c.shape_matching != enabled {
                c.shape_matching = enabled;
                self.modified = true;
            }
        }
    }

    /// Scales the material stiffness (Young modulus) of every cell fully contained in the
    /// `i`-th cluster: regional materials without a separate body. Cells straddling the
    /// cluster's boundary are left as they are.
    pub fn set_cluster_stiffness_scale(&mut self, i: u32, scale: Real) {
        let Some(cluster) = self.clusters.get(i as usize).filter(|c| c.is_live()) else {
            return;
        };
        let contained: Vec<bool> = self
            .cells
            .iter()
            .map(|c| {
                c.vertices
                    .iter()
                    .all(|v| cluster.particles.binary_search(v).is_ok())
            })
            .collect();
        for (c, inside) in self.cells.iter_mut().zip(contained) {
            if inside {
                c.stiffness_scale = scale.max(0.0);
            }
        }
        self.modified = true;
    }

    /// Overrides the softness of every structural or bending edge fully contained in the `i`-th
    /// cluster (`None` restores the body material's): regional stiffness for cloth and ropes.
    pub fn set_cluster_edge_softness(
        &mut self,
        i: u32,
        softness: Option<crate::dynamics::SpringCoefficients<Real>>,
    ) {
        let Some(cluster) = self.clusters.get(i as usize).filter(|c| c.is_live()) else {
            return;
        };
        let contained: Vec<bool> = self
            .edges
            .iter()
            .map(|e| {
                e.vertices
                    .iter()
                    .all(|v| cluster.particles.binary_search(v).is_ok())
            })
            .collect();
        for (e, inside) in self.edges.iter_mut().zip(contained) {
            if inside {
                e.softness = softness;
            }
        }
        self.modified = true;
    }

    /// Pins (or releases) every particle of the `i`-th cluster at once: the cluster becomes a
    /// fixed (kinematic) region dragging the rest of the body, driven along a path with
    /// [`Self::set_cluster_kinematic_target`]; particles keep their individual pinned semantics.
    pub fn set_cluster_pinned(&mut self, i: u32, pinned: bool) {
        let Some(cluster) = self.clusters.get(i as usize).filter(|c| c.is_live()) else {
            return;
        };
        for &v in cluster.particles.clone().iter() {
            self.set_particle_pinned(v as usize, pinned);
        }
    }

    /// Moves the pinned particles of the `i`-th cluster rigidly toward `pose` over the next step:
    /// each pinned particle's target is the cluster's rest shape placed at `pose`, with matching
    /// velocities; free particles are unaffected (pin them with [`Self::set_cluster_pinned`]).
    pub fn set_cluster_kinematic_target(&mut self, i: u32, pose: Pose) {
        let Some(cluster) = self.clusters.get(i as usize).filter(|c| c.is_live()) else {
            return;
        };
        // Rest centroid of the cluster (nominal masses): the target pose places the rest shape.
        let mut nominal = 0.0;
        let mut rest_com = Vector::ZERO;
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            nominal += p.mass;
            rest_com += p.rest_position * p.mass;
        }
        if nominal > 0.0 {
            rest_com /= nominal;
        }
        for &v in cluster.particles.clone().iter() {
            let offset = self.particles[v as usize].rest_position - rest_com;
            self.set_particle_kinematic_target(v as usize, pose * offset);
        }
    }

    /// Recomputes the per-particle cluster reference counts from the cluster lists.
    pub(crate) fn rebuild_cluster_refs(&mut self) {
        self.cluster_refs.clear();
        self.cluster_refs.resize(self.particles.len(), 0);
        for c in &self.clusters {
            if c.is_live() {
                for &v in &c.particles {
                    if let Some(r) = self.cluster_refs.get_mut(v as usize) {
                        *r += 1;
                    }
                }
            }
        }
    }

    /// The cell whose vertices are exactly `particles` (sorted), if any: such a cluster's frame
    /// rotation is the cell's own corotational rotation.
    pub(crate) fn matching_cell(&self, particles: &[u32]) -> u32 {
        if particles.len() != DIM + 1 {
            return u32::MAX;
        }
        for (i, cell) in self.cells.iter().enumerate() {
            let mut vs = cell.vertices;
            vs.sort_unstable();
            if vs == particles[..DIM + 1] {
                return i as u32;
            }
        }
        u32::MAX
    }

    /// Registers freshly duplicated particles (a tear split) into the clusters of their source
    /// particle, and extends the reference counts. `duplicated` is `(copy, source)` pairs.
    pub(crate) fn inherit_cluster_membership(&mut self, duplicated: &[(u32, u32)]) {
        self.cluster_refs.resize(self.particles.len(), 0);
        for &(copy, source) in duplicated {
            for c in &mut self.clusters {
                if c.is_live() && c.particles.binary_search(&source).is_ok() {
                    if let Err(pos) = c.particles.binary_search(&copy) {
                        c.particles.insert(pos, copy);
                        if !c.shape_impulses.is_empty() {
                            // Keep the warm shape impulses aligned with the particle list.
                            c.shape_impulses.insert(pos.min(c.shape_impulses.len()), Vector::ZERO);
                        }
                        if let Some(r) = self.cluster_refs.get_mut(copy as usize) {
                            *r += 1;
                        }
                    }
                }
            }
        }
        // Duplication changes the cell list; the element frames must follow their cells.
        self.update_cluster_cells();
    }

    /// Re-derives every cluster's `cell` match (cells were removed, added or renumbered).
    pub(crate) fn update_cluster_cells(&mut self) {
        let mut matches = Vec::new();
        for c in &self.clusters {
            matches.push(if c.is_live() {
                self.matching_cell(&c.particles)
            } else {
                u32::MAX
            });
        }
        for (c, cell) in self.clusters.iter_mut().zip(matches) {
            c.cell = cell;
        }
    }

    /// Removes every particle whose `dead` flag is set (they must belong to no live cluster), the
    /// elements touching them and the attachments on them, compacting the remaining indices.
    /// Returns the removal counts and the old-to-new remap (`u32::MAX` if removed; empty if none).
    pub(crate) fn remove_dead_particles(&mut self, dead: &[bool]) -> (SoftClusterRemoval, Vec<u32>) {
        let mut counts = SoftClusterRemoval::default();
        if !dead.contains(&true) {
            return (counts, Vec::new());
        }
        debug_assert_eq!(dead.len(), self.particles.len());

        // Old -> new particle index (u32::MAX for the removed ones).
        let mut remap = vec![u32::MAX; self.particles.len()];
        let mut next = 0u32;
        for (i, &d) in dead.iter().enumerate() {
            if !d {
                remap[i] = next;
                next += 1;
            }
        }
        counts.particles = self.particles.len() - next as usize;

        let alive = |vs: &[u32]| vs.iter().all(|&v| !dead[v as usize]);

        // Elements: drop the ones touching a dead particle, remap the rest.
        let before = self.edges.len();
        self.edges.retain(|e| alive(&e.vertices));
        counts.edges = before - self.edges.len();
        for e in &mut self.edges {
            for v in &mut e.vertices {
                *v = remap[*v as usize];
            }
        }

        // Cells need their own remap for the skin bindings.
        let mut cell_remap = vec![u32::MAX; self.cells.len()];
        let mut next_cell = 0u32;
        for (i, c) in self.cells.iter().enumerate() {
            if alive(&c.vertices) {
                cell_remap[i] = next_cell;
                next_cell += 1;
            }
        }
        let before = self.cells.len();
        self.cells.retain(|c| alive(&c.vertices));
        counts.cells = before - self.cells.len();
        for c in &mut self.cells {
            for v in &mut c.vertices {
                *v = remap[*v as usize];
            }
        }

        #[cfg(feature = "dim3")]
        {
            let before = self.dihedrals.len();
            self.dihedrals.retain(|d| alive(&d.vertices));
            counts.dihedrals = before - self.dihedrals.len();
            for d in &mut self.dihedrals {
                for v in &mut d.vertices {
                    *v = remap[*v as usize];
                }
            }
        }

        let before = self.boundary.len();
        self.boundary.retain(|s| alive(s));
        counts.surface_elements = before - self.boundary.len();
        for s in &mut self.boundary {
            for v in s.iter_mut() {
                *v = remap[*v as usize];
            }
        }

        // Attachments anchored to a dead particle die with it.
        let before = self.attachments.len();
        self.attachments.retain(|a| !dead[a.particle as usize]);
        counts.attachments = before - self.attachments.len();
        for a in &mut self.attachments {
            a.particle = remap[a.particle as usize];
        }
        if counts.attachments > 0 {
            self.attachments_modified = true;
        }
        self.attachments_modified |= counts.particles > 0;

        // Live clusters: drop dead entries (there should be none) and remap; the warm shape
        // impulses follow their particles.
        for c in &mut self.clusters {
            if c.is_live() {
                if c.shape_impulses.len() == c.particles.len() {
                    let mut keep = c.particles.iter().map(|&v| !dead[v as usize]);
                    c.shape_impulses.retain(|_| keep.next().unwrap());
                } else {
                    c.shape_impulses.clear();
                }
                c.particles.retain(|&v| !dead[v as usize]);
                for v in &mut c.particles {
                    *v = remap[*v as usize];
                }
            }
        }

        // The particles themselves, and the per-particle tables.
        let mut keep = dead.iter().map(|d| !d);
        self.particles.retain(|_| keep.next().unwrap());
        self.rebuild_cluster_refs();

        // The meshes follow the renumbered particles and cells; their contact caches hold stale
        // ids.
        let remap_tables = super::collision_mesh::SoftTopologyRemap {
            cells: &cell_remap,
            particles: &remap,
        };
        self.for_each_mesh_mut(|body, mesh| {
            mesh.remap_topology(body, &remap_tables);
            mesh.clear_contacts();
        });

        self.boundary_closed = super::soft_body_builder::surface_is_closed(&self.boundary);
        self.rebuild_mesh_tables();
        self.update_cluster_cells();
        self.recolor();
        self.modified = true;
        self.topology_version = self.topology_version.wrapping_add(1);
        (counts, remap)
    }

    /// Weighted centroid of the cluster's particles (current positions) and their total mass.
    pub(crate) fn cluster_com_and_mass(&self, cluster: &SoftBodyCluster) -> (Vector, Real) {
        let mut com = Vector::ZERO;
        let mut mass = 0.0;
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            com += p.position * p.mass;
            mass += p.mass;
        }
        if mass > 0.0 {
            com /= mass;
        }
        (com, mass)
    }

    /// The current frame and reduced mass matrix of the `ci`-th cluster: origin at the free
    /// particles' mass-weighted centroid (every particle's when all are pinned), rotation from the
    /// matching cell (else the Kabsch fit onto the rest positions), mass from the free particles.
    pub(crate) fn cluster_frame(&self, ci: usize) -> Option<ClusterFrame> {
        let cluster = self.clusters.get(ci)?;
        if !cluster.is_live() {
            return None;
        }

        // Dynamics: the free particles' centroid, momentum and reduced mass matrix.
        let mut mass = 0.0;
        let mut com = Vector::ZERO;
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            if p.inv_mass > 0.0 {
                mass += p.mass;
                com += p.position * p.mass;
            }
        }
        let inv_mass = crate::utils::inv(mass);
        let origin = if mass > 0.0 {
            com * inv_mass
        } else {
            // Fully pinned: the frame still needs an origin.
            let (com_all, _) = self.cluster_com_and_mass(cluster);
            com_all
        };

        let mut linvel = Vector::ZERO;
        let mut angmom = AngVector::default();
        #[cfg(feature = "dim2")]
        let mut inertia: AngularInertia = 0.0;
        #[cfg(feature = "dim3")]
        let mut inertia = parry::utils::SdpMatrix3::zero();
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            if p.inv_mass > 0.0 {
                let r = p.position - origin;
                linvel += p.velocity * p.mass;
                angmom += r.gcross(p.velocity * p.mass);
                #[cfg(feature = "dim2")]
                {
                    inertia += p.mass * r.length_squared();
                }
                #[cfg(feature = "dim3")]
                {
                    let d = p.mass * r.length_squared();
                    inertia = parry::utils::SdpMatrix3 {
                        m11: inertia.m11 + d - p.mass * r.x * r.x,
                        m12: inertia.m12 - p.mass * r.x * r.y,
                        m13: inertia.m13 - p.mass * r.x * r.z,
                        m22: inertia.m22 + d - p.mass * r.y * r.y,
                        m23: inertia.m23 - p.mass * r.y * r.z,
                        m33: inertia.m33 + d - p.mass * r.z * r.z,
                    };
                }
            }
        }
        // Pseudo-inverse with an eigenvalue threshold: a rank-deficient cluster has a singular
        // reduced inertia whose naive inverse explodes; those directions lose their angular
        // response instead (a fully degenerate cluster gets its angular joint axes stripped).
        let inv_inertia = pseudo_inverse_inertia(inertia, inertia_noise_floor(mass, origin));
        let linvel = linvel * inv_mass;
        let angvel = inv_inertia.transform_vector(angmom);

        // Rotation: exact (the matching cell's corotational rotation), or the Kabsch fit of the
        // particles onto their rest positions (see the plan's 2.6: for a one-cell cluster the
        // two differ by O(strain x rest anisotropy), so the exact one is preferred).
        let rotation = if let Some(cell) = self.cells.get(cluster.cell as usize) {
            cell.rotation
        } else {
            let mut rest_com = Vector::ZERO;
            let mut nominal_mass = 0.0;
            for &v in &cluster.particles {
                let p = &self.particles[v as usize];
                rest_com += p.rest_position * p.mass;
                nominal_mass += p.mass;
            }
            if nominal_mass > 0.0 {
                rest_com /= nominal_mass;
            }
            let mut fit_com = Vector::ZERO;
            for &v in &cluster.particles {
                let p = &self.particles[v as usize];
                fit_com += p.position * p.mass;
            }
            if nominal_mass > 0.0 {
                fit_com /= nominal_mass;
            }
            let mut a = crate::math::Matrix::ZERO;
            for &v in &cluster.particles {
                let p = &self.particles[v as usize];
                let x = (p.position - fit_com) * p.mass;
                let q = p.rest_position - rest_com;
                #[cfg(feature = "dim2")]
                {
                    a += crate::math::Matrix::from_cols(x * q.x, x * q.y);
                }
                #[cfg(feature = "dim3")]
                {
                    a += crate::math::Matrix::from_cols(x * q.x, x * q.y, x * q.z);
                }
            }
            extract_rotation(a, cluster.rotation)
        };

        Some(ClusterFrame {
            pose: Pose::from_parts(origin, rotation),
            inv_mass,
            inv_inertia,
            linvel,
            angvel,
        })
    }
}

impl SoftBody {
    /// Solver-side gather of a cluster's rigid state from its particles' solver bodies: `(com,
    /// linvel, angvel)` of the free particles, with the frozen per-step `inv_inertia` (world axes
    /// about the fresh com). `pos`/`vel` map a particle index to its solver state.
    pub(crate) fn gather_cluster_velocity(
        &self,
        ci: usize,
        inv_inertia: &AngularInertia,
        pos: impl Fn(u32) -> Vector,
        vel: impl Fn(u32) -> Vector,
    ) -> (Vector, Vector, AngVector) {
        let cluster = &self.clusters[ci];
        let mut mass = 0.0;
        let mut com = Vector::ZERO;
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            if p.inv_mass > 0.0 {
                mass += p.mass;
                com += pos(v) * p.mass;
            }
        }
        if mass > 0.0 {
            com /= mass;
        }
        let inv_mass = crate::utils::inv(mass);
        let mut linvel = Vector::ZERO;
        let mut angmom = AngVector::default();
        for &v in &cluster.particles {
            let p = &self.particles[v as usize];
            if p.inv_mass > 0.0 {
                let r = pos(v) - com;
                linvel += vel(v) * p.mass;
                angmom += r.gcross(vel(v) * p.mass);
            }
        }
        (
            com,
            linvel * inv_mass,
            inv_inertia.transform_vector(angmom),
        )
    }
}

/// The per-step frame and reduced mass matrix of a cluster (see `SoftBody::cluster_frame`).
#[derive(Copy, Clone, Debug)]
pub(crate) struct ClusterFrame {
    /// Frame pose: origin at the free particles' weighted centroid, rotation from the cell or
    /// the Kabsch fit.
    pub pose: Pose,
    /// Inverse of the free particles' total mass (`0`: immovable).
    pub inv_mass: Real,
    /// Inverse of the reduced angular inertia about the origin (world axes; zero when
    /// rank-deficient).
    pub inv_inertia: AngularInertia,
    /// Mass-weighted mean velocity of the free particles.
    pub linvel: Vector,
    /// Angular velocity of the rigid fit: `I⁻¹ Σ rᵢ × mᵢvᵢ`.
    pub angvel: AngVector,
}

/// Absolute noise floor of a cluster's reduced inertia: a zero-extent cluster (one free particle,
/// a point-collapsed one) has a roundoff inertia whose inverse fits huge angular velocities, so
/// an eigenvalue below the mass put at `sqrt(machine epsilon)` of the coordinate scale is noise.
pub(crate) fn inertia_noise_floor(mass: Real, com: Vector) -> Real {
    let noise_extent = Real::EPSILON.sqrt() * (1.0 + com.length());
    mass * noise_extent * noise_extent
}

/// Pseudo-inverse of a reduced angular inertia: directions whose eigenvalue is negligible next
/// to the largest one, or below the absolute floor `abs_floor` (see [`inertia_noise_floor`]),
/// are zeroed instead of inverted (a singular inertia's naive inverse explodes).
#[cfg(feature = "dim2")]
pub(crate) fn pseudo_inverse_inertia(inertia: AngularInertia, abs_floor: Real) -> AngularInertia {
    if inertia <= abs_floor {
        0.0
    } else {
        crate::utils::inv(inertia)
    }
}

/// See the 2D overload.
#[cfg(feature = "dim3")]
pub(crate) fn pseudo_inverse_inertia(inertia: AngularInertia, abs_floor: Real) -> AngularInertia {
    const RELATIVE_EPS: Real = 1.0e-5;
    let m = na::Matrix3::new(
        inertia.m11, inertia.m12, inertia.m13, inertia.m12, inertia.m22, inertia.m23, inertia.m13,
        inertia.m23, inertia.m33,
    );
    let eig = m.symmetric_eigen();
    let max = eig.eigenvalues.amax();
    if max <= abs_floor {
        return parry::utils::SdpMatrix3::zero();
    }
    let mut inv = na::Matrix3::zeros();
    for k in 0..3 {
        let lambda = eig.eigenvalues[k];
        if lambda > (max * RELATIVE_EPS).max(abs_floor) {
            let u = eig.eigenvectors.column(k);
            inv += u * u.transpose() / lambda;
        }
    }
    parry::utils::SdpMatrix3 {
        m11: inv[(0, 0)],
        m12: inv[(0, 1)],
        m13: inv[(0, 2)],
        m22: inv[(1, 1)],
        m23: inv[(1, 2)],
        m33: inv[(2, 2)],
    }
}
