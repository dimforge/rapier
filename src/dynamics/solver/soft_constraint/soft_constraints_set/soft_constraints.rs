//! The constraint types of the set: the shape-matching, volume-piece and intersection-volume constraints with their FEM sides.

use core::ops::Range;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::soft_body::SoftMeshId;
use crate::geometry::ColliderHandle;
use crate::math::{AngVector, DIM, Matrix, Real, Rotation, Vector};
use crate::utils::ComponentMul;

use super::super::soft_attachment::FemAttachment;

/// The `DIM` axis constraints pulling one particle toward its shape-matching goal.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftShapeConstraint {
    pub particle: u32,
    pub cluster: u32,
    pub solver_id: u32,
    pub im: Vector,
    pub goal: Vector,
    /// Velocity of the goal (the body's best-fit rigid motion, or the target's): the constraint damps
    /// the particle's velocity relative to it, so the rigid modes (a free fall, a spin) are left
    /// alone; damping the absolute velocity summed to a brake on the whole body.
    pub goal_vel: Vector,
    pub erp_inv_dt: Real,
    pub cfm_coeff: Real,
    pub inv_lhs: Vector,
    pub cfm_gain: Vector,
    pub rhs: Vector,
    pub impulse: Vector,
}
impl SoftShapeConstraint {
    #[inline]
    pub fn update(&mut self, bodies: &SolverBodies) {
        let pos = bodies.get_pose(self.solver_id).translation;
        // Uncapped: the bias is the spring force (see `SoftScalarConstraint::update`).
        self.rhs = (pos - self.goal) * self.erp_inv_dt;
    }

    #[inline]
    #[inline]
        if (self.solver_id as usize) >= bodies.len() {
            return;
        }
        let delta = total - self.impulse;
        self.impulse = total;
}
/// The global area/volume preservation row of a soft body (touches every particle: solved
/// serially).
#[derive(Clone, Debug)]
pub(crate) struct SoftVolumeConstraint {
    pub soft_body: u32,
    pub target: Real,
    /// Range of this row's particle gradients in `volume_grads`.
    pub grads: Range<usize>,
    pub erp_inv_dt: Real,
    pub cfm_coeff: Real,
    /// Cap on the velocity the bias may give any particle in one solve.
    pub max_bias_velocity: Real,
    pub inv_lhs: Real,
    pub cfm_gain: Real,
    pub rhs: Real,
    pub impulse: Real,
}
/// An intersection-volume constraint (see `SoftRecoverySettings::overlap_constraints`): a scalar
/// unilateral constraint retracting the part of a closed boundary inside another (or its own
/// mirrored lobe) along the gradient of the enclosed area/volume. Solved serially, no warm start.
#[derive(Clone, Debug)]
pub(crate) struct SoftOverlapConstraint {
    /// Range of this constraint's `(solver slot, inverse mass, gradient)` triples in `overlap_grads`.
    pub grads: Range<usize>,
    /// The overlap measure the constraint corrects (positive while overlapping).
    pub rhs: Real,
    pub erp_inv_dt: Real,
    pub cfm_coeff: Real,
    /// Cap on the velocity the bias may give any particle in one solve.
    pub max_bias_velocity: Real,
    pub impulse: Real,
    /// A negative `rhs` is slack, allowed to close at this rate (the skin band of a hard
    /// row may be consumed within the substep, never past it); `0.0` for the rows whose
    /// `rhs` is an error.
    pub speculative_inv_dt: Real,
    /// The rigid side of an overlap row, when the other body is a simulated rigid body:
    /// its solver slot, and the volume gradient with respect to its translation and its
    /// rotation about its center of mass (the reaction to the soft patch, applied at the
    /// patch's vertices).
    pub rigid: Option<(u32, Vector, AngVector)>,
    /// A hard row (the skin-volume rows, contacts in their own right): its impulse is not
    /// bounded by the pace, and its right-hand side is refreshed every substep from the
    /// current poses (the volume linearized with the frozen gradients from `rhs0`), so
    /// its slack is consumed exactly, never several times over.
    pub hard: bool,
    /// The assembled right-hand side (see `hard`).
    pub rhs0: Real,
    /// The rigid side's center of mass and rotation at assembly (see `hard`).
    pub rigid_pose0: (Vector, Rotation),
    /// Where the row's impulses are carried to for the next step's warm start (see
    /// `SoftOverlapWarm`): the awake body, its mesh and the other collider. The carried
    /// impulse of every entry lives in `overlap_warm_impulses`, aligned with `grads`, and
    /// the entries' particles in `overlap_particles`.
    pub warm: Option<(u32, SoftMeshId, ColliderHandle)>,
    /// The carried per-particle impulses are still to be applied (the first substep).
    pub warm_pending: bool,
    /// The rigid side's carried linear and angular impulse.
    pub warm_rigid: (Vector, AngVector),
}
