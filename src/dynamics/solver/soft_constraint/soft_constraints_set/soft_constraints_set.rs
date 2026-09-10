//! The awake bodies and clusters, the per-color and per-group layout, the cluster records and the constraints set itself.

use crate::alloc_prelude::*;
use core::ops::Range;
use core::sync::atomic::AtomicBool;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::SolverVel;
use crate::dynamics::{SoftBody, SoftBodyHandle};
use crate::math::{AngVector, AngularInertia, Real, Rotation, Vector};

use super::super::soft_contact::SoftContact;
use super::super::soft_element_constraint::{SoftElasticConstraint, SoftScalarConstraint};
use super::*;

/// One awake soft body, as seen by the solver for the current step.
pub(crate) struct AwakeSoftBody {
    /// Raw access to the soft body for the solve scope (the set is not mutated meanwhile);
    /// constraints write their impulses back through it, to disjoint elements.
    pub ptr: *mut SoftBody,
    pub handle: SoftBodyHandle,
    /// Substep solve-group of the body's particles.
    pub group: u16,
    /// Length of the group's substeps (the edge impulses are per substep: this turns them into
    /// forces for the tear criterion).
    pub substep_dt: Real,
    /// No particle is a solver DOF this step (every one pinned, or the free ones asleep): the
    /// body only lends its surface to the contact constraints holding the awake bodies touching it.
    pub frozen: bool,
    /// Solver-body slot of each particle: `slots[slot_start..slot_start + num_particles]` (the
    /// particles' slots are contiguous, laid out group-major after the rigid bodies').
    pub slot_start: usize,
    pub num_particles: usize,
    pub shape_com: Vector,
    /// Range of this body's constraints in `shape_constraints`.
    pub shape_constraints: Range<usize>,
    /// Range of this body's volume constraints in `volume_constraints` (one per volume piece).
    pub volume_constraint: Option<usize>,
    /// Fraction of the non-rigid velocity removed per substep (`deformation_damping`).
    pub damping_factor: Real,
    /// Set by the writeback stage when a cell flowed plastically by a significant amount
    /// (the body is kept awake so its creep goes on).
    pub plastic_flow: AtomicBool,
    /// Set by the writeback stage when an element was strained past the material's tear
    /// strain (the tearing pass at the end of the step removes it).
    pub torn: AtomicBool,
    /// Largest normal approach speed of the rigid bodies met by the surface this step (`None`:
    /// no contact constraint at all), set by the contact assembly.
    pub contact_approach_speed: Option<Real>,
    /// The body's shape-matched clusters this step, with their warm-started fit rotation:
    /// `(cluster index, rotation)`, updated by the per-pass prepare.
    pub awake_clusters: Vec<AwakeCluster>,
    /// The body's FEM system when it is on the FEM path (set by `SoftFemSet::assemble`): the
    /// per-substep constraints (the volume constraints) update their responses through it.
    #[cfg(feature = "fem")]
    pub fem: Option<crate::dynamics::solver::soft_fem::AwakeFem>,
}

// SAFETY: the raw pointer is only dereferenced under the staged solver's stage discipline.
unsafe impl Send for AwakeSoftBody {}
unsafe impl Sync for AwakeSoftBody {}

pub(crate) struct AwakeCluster {
    /// Index of the cluster.
    pub cluster: u32,
    /// Warm-start rotation of the fit, or the kinematic target's rotation.
    pub rotation: Rotation,
    /// Linear velocity of the kinematic target if any. Zero otherwise.
    pub target_linvel: Vector,
    /// Angular velocity of the kinematic target if any. Zero otherwise.
    pub target_angvel: AngVector,
    /// The fit's centroids and inverse inertia of the last substep update: the current and rest
    /// centers of mass of all the cluster's particles, the current center of mass of its free ones,
    /// and the pseudo-inverse of their inertia about it.
    pub com: Vector,
    pub rest_com: Vector,
    pub dyn_com: Vector,
    pub inv_inertia: AngularInertia,
}

/// The constraints of one parallel color: a range of scalar constraints and a range of
/// elastic-cell block constraints, addressed by one virtual index range
/// `0..scalar_constraints.len() + elastic_constraints.len()`.
#[derive(Clone, Debug, Default)]
pub(crate) struct SoftColorRange {
    pub scalar_constraints: Range<usize>,
    pub elastic_constraints: Range<usize>,
}

impl SoftColorRange {
    #[inline]
    pub fn len(&self) -> usize {
        self.scalar_constraints.len() + self.elastic_constraints.len()
    }
}

/// Per-group slices of the soft layout.
#[derive(Clone, Debug, Default)]
pub(crate) struct SoftGroupLayout {
    /// Range of awake soft bodies (into `awake`) belonging to the group.
    pub awake: Range<usize>,
    /// Solver-body slot range of the group's particles.
    pub slots: Range<usize>,
    /// Range into `attachments`.
    pub attachments: Range<usize>,
    /// Range into `clusters`.
    pub clusters: Range<usize>,
    /// Index range into `color_ranges`.
    pub colors: Range<usize>,
    /// Constraints solved serially by worker 0 (overflow color).
    pub serial: SoftColorRange,
    /// Range into `shape_constraints`.
    pub shape_constraints: Range<usize>,
    /// Range into `volume_constraints`.
    pub volume_constraints: Range<usize>,
    /// Range into `overlap_constraints`.
    pub overlap_constraints: Range<usize>,
    /// Range into `contacts`.
    pub contacts: Range<usize>,
    /// Range into `contact_colors`: the parallel contact stages.
    pub contact_colors: Range<usize>,
    /// Range into `contact_chunks`: the contact chunks solved serially by worker 0 after the
    /// colors.
    pub contact_serial: Range<usize>,
    /// Whether an awake body of the group damps its deformation.
    pub damping: bool,
    /// Whether the group's shape constraints must be solved serially: per-cluster shape matching can
    /// give one particle several constraints (whole body + clusters), which must not be claimed by
    /// different workers.
    pub shape_serial: bool,
}

/// One active soft-body cluster for the step: its proxy's solver slot is a virtual rigid body whose
/// velocity is gathered from the cluster's particles before the joints solve and whose velocity
/// change is scattered back after. A cluster is active when a joint, impulse or force acts on it.
#[derive(Copy, Clone)]
pub(crate) struct SoftClusterRecord {
    /// Index of the cluster's soft body in `awake`.
    pub awake: u32,
    /// Index of the cluster in its soft body's cluster list.
    pub cluster: u32,
    /// The proxy's solver-body slot.
    pub slot: u32,
    /// Weighted centroid of the free particles at the last gather (the scatter's torque
    /// reference).
    pub com: Vector,
    /// The proxy-slot velocity already accounted in the particles: the scatter distributes
    /// `slot velocity - ref_vel`, the gather re-derives it.
    pub ref_vel: SolverVel<Real>,
}

/// The soft-body constraints of one step.
#[derive(Default)]
pub(crate) struct SoftConstraintsSet {
    pub awake: Vec<AwakeSoftBody>,
    pub slots: Vec<u32>,
    pub scalar_constraints: Vec<SoftScalarConstraint>,
    pub elastic_constraints: Vec<SoftElasticConstraint>,
    /// Whether each element constraint (the scalar ones, then the elastic ones) was strained
    /// past the re-sweep threshold at its last update: the re-sweep reads these flags instead
    /// of every constraint.
    pub strained: Vec<bool>,
    /// Constraint ranges of the parallel colors, group-major then color ascending.
    pub color_ranges: Vec<SoftColorRange>,
    pub shape_constraints: Vec<SoftShapeConstraint>,
    pub volume_constraints: Vec<SoftVolumeConstraint>,
    pub volume_grads: Vec<Vector>,
    /// The intersection-volume constraints (see `SoftOverlapConstraint`), group-major, and their gradients.
    pub overlap_constraints: Vec<SoftOverlapConstraint>,
    pub overlap_grads: Vec<(u32, Real, Vector, Vector)>,
    /// The warm impulse of every overlap gradient entry (see `SoftOverlapConstraint::warm`).
    pub overlap_warm_impulses: Vec<Vector>,
    /// The `(side, particle)` of every overlap gradient entry (`0`: the owner body, `1`:
    /// the other soft body), for the write-back of the warm impulses.
    pub overlap_particles: Vec<(u8, u32)>,
    /// Contacts against the awake soft bodies' surface colliders, group-major.
    pub contacts: Vec<SoftContact>,
    /// The responses `A⁻¹Jᵀ` of the constraints acting on FEM soft bodies (see
    /// `soft_fem::SoftFemSet::assemble_responses`): a vector per particle, per constraint side and
    /// direction, addressed by the constraints' `fem` records. Empty without FEM bodies.
    pub fem_responses: Vec<Vector>,
    /// The FEM sides of the intersection-volume constraints (see [`FemOverlapSide`]).
    pub overlap_fem_sides: Vec<FemOverlapSide>,
    /// The particle attachments of the awake soft bodies, group-major.
    pub attachments: Vec<super::soft_attachment::SoftAttachmentConstraint>,
    /// The active clusters (a joint or an external impulse acts on their proxy), group-major.
    pub clusters: Vec<SoftClusterRecord>,
    /// The contact constraints chunked by body pair (see `soft_contact_chunks`): constraint indices into
    /// `contacts` chunk by chunk, the chunks' ranges into them (group-major, parallel colors
    /// then the serial tail), and the chunk ranges of the parallel colors.
    pub contact_chunk_constraints: Vec<u32>,
    pub contact_chunks: Vec<Range<usize>>,
    pub contact_colors: Vec<Range<usize>>,
    pub groups: Vec<SoftGroupLayout>,
    /// Workspace of the element-constraint assembly (see `soft_element_constraint_assembly`).
    pub(crate) element_constraint_workspace:
        super::soft_element_constraint_assembly::ElementConstraintAssemblyWorkspace,
    /// Workspace of the edge-vs-edge contact assembly (see `soft_contact_assembly`).
    pub(crate) edge_workspace: super::soft_contact_assembly::EdgeContactWorkspace,
    /// Workspace of the contact chunking (see `soft_contact_chunks`).
    pub(crate) contact_workspace: super::soft_contact_chunks::ContactChunkWorkspace,
}

impl SoftConstraintsSet {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the step has any soft-body work at all.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.awake.is_empty()
    }

    /// Whether the given group has soft-body work needing the per-pass prepare stage
    /// (shape matching, volume pieces, deformation damping).
    #[inline]
    pub fn group_needs_prepare(&self, group: usize, update: bool) -> bool {
        let g = &self.groups[group];
        // The volume gradients and the damping only move with the poses (once per substep);
        // the shape constraints re-fit the velocities every pass.
        !g.shape_constraints.is_empty()
            || (update && (!g.volume_constraints.is_empty() || g.damping))
    }
}
