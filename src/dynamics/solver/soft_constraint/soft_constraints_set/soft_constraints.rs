//! The constraint types of the set: the shape-matching, volume-piece and intersection-volume constraints with their FEM sides.

use core::ops::Range;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{AngVector, DIM, Matrix, Real, Rotation, Vector};
use crate::utils::ComponentMul;

use super::super::soft_attachment::FemAttachment;

/// The `DIM` axis constraints pulling one particle toward its shape-matching goal.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftShapeConstraint {
    pub particle: u32,
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
