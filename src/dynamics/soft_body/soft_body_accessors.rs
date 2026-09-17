//! Accessors and simple setters of a soft body: particles, elements, material, volume preservation, sleeping and enabling.
use super::{
    SoftBody, SoftBodyCell, SoftBodyCellModel, SoftBodyEdge, SoftBodyMaterial, SoftBodyParticle,
    SoftVolumePiece,
};
#[cfg(feature = "dim3")]
use super::SoftBodyDihedral;
#[cfg(feature = "fem")]
use super::SoftBodySolver;
use crate::dynamics::RigidBodyHandle;
use crate::math::{DIM, Real, Vector};

impl SoftBody {
    /// A counter bumped whenever the topology changes (a tear removed elements or duplicated
    /// particles): a renderer keying its mesh on the particles and elements rebuilds it when
    /// this value changes.
    pub fn topology_version(&self) -> u32 {
        self.topology_version
    }

    /// The particles of this soft body.
    pub fn particles(&self) -> &[SoftBodyParticle] {
        &self.particles
    }

    /// The number of particles of this soft body.
    pub fn num_particles(&self) -> usize {
        self.particles.len()
    }

    /// The world-space position of the `i`-th particle.
    pub fn particle_position(&self, i: usize) -> Vector {
        self.particles[i].position
    }

    /// The world-space positions of every particle.
    pub fn particle_positions(&self) -> impl ExactSizeIterator<Item = Vector> + '_ {
        self.particles.iter().map(|p| p.position)
    }

    /// The world-space velocity of the `i`-th particle.
    pub fn particle_velocity(&self, i: usize) -> Vector {
        self.particles[i].velocity
    }

    /// The world-space velocities of every particle.
    pub fn particle_velocities(&self) -> impl ExactSizeIterator<Item = Vector> + '_ {
        self.particles.iter().map(|p| p.velocity)
    }

    /// Teleports the `i`-th particle to `position` (no velocity change). For a pinned particle
    /// moving along a path, prefer [`Self::set_particle_kinematic_target`], which gives it the
    /// matching velocity so friction and contacts see the motion.
    pub fn set_particle_position(&mut self, i: usize, position: Vector) {
        self.particles[i].position = position;
        self.particles[i].next_position = None;
        self.positions_modified = true;
        self.modified = true;
    }

    /// Sets the velocity of the `i`-th particle. A pinned particle keeps moving at that velocity
    /// (a velocity-driven kinematic particle) until it is set again or given a kinematic target.
    pub fn set_particle_velocity(&mut self, i: usize, velocity: Vector) {
        self.particles[i].velocity = velocity;
        self.particles[i].next_position = None;
        self.modified = true;
    }

    /// Moves the pinned `i`-th particle to `position` over the next step (with the matching
    /// velocity, so dragged cloth and friction see the motion), then holds it there until the next
    /// target or velocity is set. Ignored for a free particle.
    pub fn set_particle_kinematic_target(&mut self, i: usize, position: Vector) {
        let particle = &mut self.particles[i];
        if particle.inv_mass == 0.0 {
            particle.next_position = Some(position);
            // A target away from the particle is a motion: the body must be awake to follow it.
            if position != particle.position {
                self.modified = true;
            }
        }
    }

    /// The distance constraints of this soft body.
    pub fn edges(&self) -> &[SoftBodyEdge] {
        &self.edges
    }

    /// The dihedral bending constraints of this soft body.
    #[cfg(feature = "dim3")]
    pub fn dihedrals(&self) -> &[SoftBodyDihedral] {
        &self.dihedrals
    }

    /// The simplex cells (triangles in 2D, tetrahedra in 3D) of this soft body.
    pub fn cells(&self) -> &[SoftBodyCell] {
        &self.cells
    }

    /// The boundary of this soft body's computational mesh: segments in 2D, triangles in 3D
    /// (particle indices), oriented outward. This is what the volume constraint integrates over; the
    /// geometry the body collides through is its [`crate::dynamics::SoftCollisionMesh`].
    pub fn boundary(&self) -> &[[u32; DIM]] {
        &self.boundary
    }

    /// The material (per-family stiffness and damping) of this soft body.
    pub fn material(&self) -> &SoftBodyMaterial {
        &self.material
    }

    /// Sets the material of this soft body.
    pub fn set_material(&mut self, material: SoftBodyMaterial) {
        self.material = material;
        self.modified = true;
    }

    /// Mutable access to the material of this soft body.
    pub fn material_mut(&mut self) -> &mut SoftBodyMaterial {
        self.modified = true;
        &mut self.material
    }

    /// The constitutive model of this soft body's cells.
    pub fn cell_model(&self) -> SoftBodyCellModel {
        self.cell_model
    }

    /// Which solver simulates this soft body's elasticity.
    #[cfg(feature = "fem")]
    pub fn solver(&self) -> SoftBodySolver {
        self.solver
    }

    /// Selects the solver simulating this soft body's elasticity (see [`SoftBodySolver`]).
    #[cfg(feature = "fem")]
    pub fn set_solver(&mut self, solver: SoftBodySolver) {
        self.solver = solver;
        self.modified = true;
    }

    /// Whether this soft body's elasticity is solved by the FEM path.
    #[inline]
    pub(crate) fn uses_fem(&self) -> bool {
        #[cfg(feature = "fem")]
        {
            self.solver == SoftBodySolver::Fem
        }
        #[cfg(not(feature = "fem"))]
        {
            false
        }
    }

    /// Whether the area/volume preservation constraints are enabled (see [`Self::volume_pieces`]).
    pub fn volume_preservation_enabled(&self) -> bool {
        self.volume_preservation
    }

    /// Enables or disables area/volume preservation: one constraint per volume piece (see
    /// [`Self::volume_pieces`]); it stays disabled for a body without any.
    pub fn enable_volume_preservation(&mut self, enabled: bool) {
        self.volume_preservation = enabled && !self.volume_pieces.is_empty();
        self.modified = true;
    }

    /// The pieces of material enclosed by a closed boundary, each with its own volume constraint
    /// so squeezing one does not inflate another. Pieces joined by no element (appended bodies,
    /// torn-apart halves) are separate; the holes or cavities of a piece belong to it.
    pub fn volume_pieces(&self) -> &[SoftVolumePiece] {
        &self.volume_pieces
    }

    /// The signed area (2D) or volume (3D) enclosed by the volume pieces at rest (`0.0` for a
    /// body without a closed boundary).
    pub fn rest_volume(&self) -> Real {
        self.volume_pieces.iter().map(|piece| piece.rest_volume).sum()
    }

    /// The signed area (2D) or volume (3D) currently enclosed by the volume pieces.
    pub fn volume(&self) -> Real {
        self.volume_pieces.iter().map(|piece| piece.volume(self)).sum()
    }

    /// The multiplier applied to each volume piece's rest volume to obtain its
    /// volume-preservation target (`> 1` inflates the body).
    pub fn volume_factor(&self) -> Real {
        self.volume_factor
    }

    /// Sets the multiplier applied to each volume piece's rest volume to obtain its
    /// volume-preservation target (`> 1` inflates the body).
    pub fn set_volume_factor(&mut self, factor: Real) {
        self.volume_factor = factor;
        self.modified = true;
    }

    /// The particle spacing of this soft body: what the impact-adaptive substeps bound the
    /// travel per substep by, and the default contact skin of its collision meshes' colliders.
    pub fn particle_radius(&self) -> Real {
        self.particle_radius
    }

    /// The hidden rigid body standing for this soft body in the islands and holding its colliders
    /// (invalid until inserted in a set). Never move, remove or attach joints to it; it only
    /// serves to recognize or exclude the soft body's colliders in queries and events.
    pub fn root_body(&self) -> RigidBodyHandle {
        self.root_body
    }

    /// The soft body this one was split off from by a tear or a cut (`None` for a user-inserted
    /// body): the piece that kept the handle (see [`crate::dynamics::SoftBodyTearEvent::pieces`]).
    /// Informational only; the handle dangles once that body is removed.
    pub fn origin(&self) -> Option<crate::dynamics::SoftBodyHandle> {
        self.origin
    }

    /// The soft bodies split off from this one by tears and cuts, in creation order (a piece
    /// torn off a piece is listed by that piece, not here). Informational: the engine never
    /// reads it, and a handle dangles once that body is removed.
    pub fn pieces(&self) -> &[crate::dynamics::SoftBodyHandle] {
        &self.pieces
    }

    /// The dynamics settings of the particles (damping, gravity scale, sleeping...).
    pub fn particle_settings(&self) -> &super::SoftBodyParticleSettings {
        &self.particle_settings
    }

    /// The number of parallel solver colors used by this soft body's elements (informational:
    /// elements of a same color share no particle and are solved concurrently).
    pub fn num_solver_colors(&self) -> usize {
        self.num_colors as usize + self.has_overflow_color as usize
    }

    /// The mass-weighted center of the particles (from the cached positions).
    pub fn center_of_mass(&self) -> Vector {
        let mut com = Vector::ZERO;
        let mut mass = 0.0;
        for p in &self.particles {
            com += p.position * p.mass;
            mass += p.mass;
        }
        if mass > 0.0 { com / mass } else { com }
    }

    /// The total nominal mass of the particles.
    pub fn mass(&self) -> Real {
        self.particles.iter().map(|p| p.mass).sum()
    }

    /// Whether this soft body is currently sleeping (a soft body sleeps and wakes as a unit,
    /// with the island of the rigid bodies it touches). Updated at the end of every step, and
    /// by [`Self::wake_up`].
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    /// Wakes this soft body up (effective at the start of the next step, together with the
    /// island of the bodies it touches).
    pub fn wake_up(&mut self) {
        self.sleeping = false;
        self.modified = true;
    }

    /// Whether this soft body takes part in the simulation. A disabled soft body (by
    /// [`Self::set_enabled`], or by the NaN quarantine: `PhysicsPipeline::quarantine`) is
    /// left where it is, without colliders or constraints, until it is enabled again.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enables or disables this soft body (effective at the start of the next step). Re-enabling
    /// a quarantined body: set its non-finite particles right first
    /// ([`Self::set_particle_position`]), they are not repaired.
    pub fn set_enabled(&mut self, enabled: bool) {
        if self.enabled != enabled {
            self.enabled = enabled;
            self.modified = true;
        }
    }

    /// Whether every particle's state is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.particles.iter().all(|p| {
            p.position.is_finite()
                && p.velocity.is_finite()
                && p.force.is_finite()
                && p.next_position.is_none_or(|t| t.is_finite())
        })
    }

    /// Neutralizes the dynamics of a non-finite body (velocities, forces, targets); the
    /// positions are left as they are.
    pub(crate) fn sanitize_dynamics(&mut self) {
        for p in &mut self.particles {
            p.velocity = Vector::ZERO;
            p.force = Vector::ZERO;
            p.next_position = None;
        }
    }
}
