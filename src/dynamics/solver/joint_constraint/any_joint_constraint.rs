use crate::dynamics::JointGraphEdge;
use crate::dynamics::solver::joint_constraint::generic_joint_constraint::GenericJointConstraint;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::JointConstraint;
use crate::math::Real;
use na::DVector;

#[cfg(feature = "simd-is-enabled")]
use crate::math::{SIMD_WIDTH, SimdReal};

use crate::dynamics::solver::solver_body::SolverBodies;

#[derive(Debug)]
pub enum AnyJointConstraintMut<'a> {
    GenericTwoBodies(&'a mut GenericJointConstraint),
    TwoBodies(&'a mut JointConstraint<Real, 1>),
    #[cfg(feature = "simd-is-enabled")]
    SimdTwoBodies(&'a mut JointConstraint<SimdReal, SIMD_WIDTH>),
}

impl AnyJointConstraintMut<'_> {
    pub fn remove_bias(&mut self) {
        match self {
            Self::TwoBodies(c) => c.remove_bias_from_rhs(),
            Self::GenericTwoBodies(c) => c.remove_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.remove_bias_from_rhs(),
        }
    }

    pub fn solve(
        &mut self,
        generic_jacobians: &DVector<Real>,
        solver_vels: &mut SolverBodies,
        generic_solver_vels: &mut DVector<Real>,
    ) {
        match self {
            Self::TwoBodies(c) => c.solve(solver_vels),
            Self::GenericTwoBodies(c) => {
                c.solve(generic_jacobians, solver_vels, generic_solver_vels)
            }
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.solve(solver_vels),
        }
    }

    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        match self {
            Self::TwoBodies(c) => c.writeback_impulses(joints_all),
            Self::GenericTwoBodies(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.writeback_impulses(joints_all),
        }
    }
}
