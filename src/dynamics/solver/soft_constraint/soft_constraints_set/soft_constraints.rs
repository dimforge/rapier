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
    /// The particle's body is on the FEM path: the constraint acts through the body's augmented
    /// mass (the block `(A⁻¹)_pp` in place of `im`, the responses spread over the body).
    pub fem: Option<FemAttachment>,
    /// The FEM path's effective-mass inverse and softness gain (blocks).
    pub inv_lhs_block: Matrix,
    pub cfm_gain_block: Matrix,
}

impl SoftShapeConstraint {
    #[inline]
    pub fn update(&mut self, bodies: &SolverBodies) {
        let pos = bodies.get_pose(self.solver_id).translation;
        if let Some(fem) = &self.fem {
            let cfm_gain = fem.inv_mass * self.cfm_coeff;
            self.cfm_gain_block = cfm_gain;
            let lhs = fem.inv_mass + cfm_gain;
            let inv = lhs.inverse();
            self.inv_lhs_block = if inv.is_finite() { inv } else { Matrix::ZERO };
        } else {
            let cfm_gain = self.im * self.cfm_coeff;
            self.cfm_gain = cfm_gain;
            let lhs = self.im + cfm_gain;
            let mut inv_lhs = Vector::ZERO;
            for k in 0..DIM {
                inv_lhs[k] = crate::utils::inv(lhs[k]);
            }
            self.inv_lhs = inv_lhs;
        }
        // Uncapped: the bias is the spring force (see `SoftScalarConstraint::update`).
        self.rhs = (pos - self.goal) * self.erp_inv_dt;
    }

    /// Applies `impulse` (positive along the constraint: the particle is pulled back).
    #[inline]
    fn apply(&self, bodies: &mut SolverBodies, pool: &[Vector], impulse: Vector) {
        if let Some(fem) = &self.fem {
            let n = fem.num_particles as usize;
            let first = fem.first_slot as usize;
            for k in 0..DIM {
                let lambda = impulse[k];
                if lambda == 0.0 {
                    continue;
                }
                let u = &pool[fem.start as usize + k * n..fem.start as usize + (k + 1) * n];
                for (i, ui) in u.iter().enumerate() {
                    bodies.vels[first + i].linear -= *ui * lambda;
                }
            }
        } else if (self.solver_id as usize) < bodies.len() {
            bodies.vels[self.solver_id as usize].linear -= self.im.component_mul(&impulse);
        }
    }

    #[inline]
    pub fn warmstart(&self, bodies: &mut SolverBodies, pool: &[Vector]) {
        self.apply(bodies, pool, self.impulse);
    }

    #[inline]
    pub fn solve(&mut self, bodies: &mut SolverBodies, pool: &[Vector]) {
        if (self.solver_id as usize) >= bodies.len() {
            return;
        }
        let v = bodies.vels[self.solver_id as usize].linear;
        let dc = v - self.goal_vel;
        let total = if self.fem.is_some() {
            self.impulse + self.inv_lhs_block * (dc + self.rhs - self.cfm_gain_block * self.impulse)
        } else {
            self.impulse
                + self
                    .inv_lhs
                    .component_mul(&(dc + self.rhs - self.cfm_gain.component_mul(&self.impulse)))
        };
        let delta = total - self.impulse;
        self.impulse = total;
        self.apply(bodies, pool, delta);
    }
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
    /// The body is on the FEM path: the constraint's response `A⁻¹Jᵀ` (refreshed at each substep's
    /// prepare) and augmented gain, see [`FemVolumeSide`].
    pub fem: Option<FemVolumeSide>,
}

/// The FEM side of a volume constraint: its response in `SoftConstraintsSet::fem_responses` from
/// `start` (one vector per particle), its gain `J A⁻¹Jᵀ`, and the largest particle speed per
/// unit impulse (the bias cap's `g_max`).
#[derive(Copy, Clone, Debug)]
pub(crate) struct FemVolumeSide {
    pub start: u32,
    pub gain: Real,
    pub u_max: Real,
}

/// The FEM-body side of an intersection-volume constraint: the gradient response from `start` in
/// `SoftConstraintsSet::fem_responses`, the warm-impulse response (`warm`, `u32::MAX`: none), the
/// augmented gain and speed cap; its `overlap_grads` inverse masses are zeroed (lumped loops skip).
#[derive(Copy, Clone, Debug)]
pub(crate) struct FemOverlapSide {
    pub first_slot: u32,
    pub num_particles: u32,
    pub start: u32,
    pub warm: u32,
    pub gain: Real,
    pub u_max: Real,
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
    /// constraint may be consumed within the substep, never past it); `0.0` for the constraints whose
    /// `rhs` is an error.
    pub speculative_inv_dt: Real,
    /// The rigid side, when the other body is a simulated rigid body: its solver slot and the
    /// volume gradient with respect to its translation and rotation about its center of mass.
    pub rigid: Option<(u32, Vector, AngVector)>,
    /// A hard constraint (skin-volume constraints, contacts in their own right): its impulse is not
    /// bounded by the pace, and its right-hand side is refreshed every substep from the current
    /// poses (the volume linearized with frozen gradients from `rhs0`), consuming its slack once.
    pub hard: bool,
    /// The assembled right-hand side (see `hard`).
    pub rhs0: Real,
    /// The rigid side's center of mass and rotation at assembly (see `hard`).
    pub rigid_pose0: (Vector, Rotation),
    /// Where the impulses are stored for the next step's warm start (see `SoftOverlapWarm`): the
    /// awake body, its mesh and the other collider. The warm impulses live in
    /// `overlap_warm_impulses` (aligned with `grads`), their particles in `overlap_particles`.
    pub warm: Option<(u32, SoftMeshId, ColliderHandle)>,
    /// The warm per-particle impulses are still to be applied (the first substep).
    pub warm_pending: bool,
    /// The rigid side's warm linear and angular impulse.
    pub warm_rigid: (Vector, AngVector),
    /// The constraint's FEM sides in `overlap_fem_sides` (see [`FemOverlapSide`]).
    pub fem_sides: Range<u32>,
    /// The narrow-phase pair's contact slot of the constraint's bin (the two colliders and the slot,
    /// `u32::MAX`: none), which reports its impulse.
    pub report: (ColliderHandle, ColliderHandle, u32),
}
