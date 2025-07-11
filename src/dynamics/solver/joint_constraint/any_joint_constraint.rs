use crate::dynamics::JointGraphEdge;
use crate::dynamics::solver::joint_constraint::joint_generic_constraint::{
    JointGenericOneBodyConstraint, JointGenericTwoBodyConstraint,
};
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointOneBodyConstraint, JointTwoBodyConstraint,
};
use crate::dynamics::solver::{AnyConstraintMut, ConstraintTypes};
use crate::math::Real;
use na::DVector;

use crate::dynamics::solver::joint_constraint::joint_generic_constraint_builder::{
    JointGenericOneBodyConstraintBuilder, JointGenericTwoBodyConstraintBuilder,
};
use crate::dynamics::solver::solver_vel::SolverVel;
#[cfg(feature = "simd-is-enabled")]
use crate::{
    dynamics::solver::joint_constraint::joint_constraint_builder::{
        JointOneBodyConstraintBuilderSimd, JointTwoBodyConstraintBuilderSimd,
    },
    math::{SIMD_WIDTH, SimdReal},
};

use crate::dynamics::solver::joint_constraint::joint_constraint_builder::{
    JointOneBodyConstraintBuilder, JointTwoBodyConstraintBuilder,
};

pub struct JointConstraintTypes;

impl AnyConstraintMut<'_, JointConstraintTypes> {
    pub fn remove_bias(&mut self) {
        match self {
            Self::OneBody(c) => c.remove_bias_from_rhs(),
            Self::TwoBodies(c) => c.remove_bias_from_rhs(),
            Self::GenericOneBody(c) => c.remove_bias_from_rhs(),
            Self::GenericTwoBodies(c) => c.remove_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdOneBody(c) => c.remove_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.remove_bias_from_rhs(),
        }
    }

    pub fn solve(
        &mut self,
        generic_jacobians: &DVector<Real>,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        match self {
            Self::OneBody(c) => c.solve(solver_vels),
            Self::TwoBodies(c) => c.solve(solver_vels),
            Self::GenericOneBody(c) => c.solve(generic_jacobians, solver_vels, generic_solver_vels),
            Self::GenericTwoBodies(c) => {
                c.solve(generic_jacobians, solver_vels, generic_solver_vels)
            }
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdOneBody(c) => c.solve(solver_vels),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.solve(solver_vels),
        }
    }

    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        match self {
            Self::OneBody(c) => c.writeback_impulses(joints_all),
            Self::TwoBodies(c) => c.writeback_impulses(joints_all),
            Self::GenericOneBody(c) => c.writeback_impulses(joints_all),
            Self::GenericTwoBodies(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdOneBody(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            Self::SimdTwoBodies(c) => c.writeback_impulses(joints_all),
        }
    }
}

impl ConstraintTypes for JointConstraintTypes {
    type OneBody = JointOneBodyConstraint<Real, 1>;
    type TwoBodies = JointTwoBodyConstraint<Real, 1>;
    type GenericOneBody = JointGenericOneBodyConstraint;
    type GenericTwoBodies = JointGenericTwoBodyConstraint;

    #[cfg(feature = "simd-is-enabled")]
    type SimdOneBody = JointOneBodyConstraint<SimdReal, SIMD_WIDTH>;
    #[cfg(feature = "simd-is-enabled")]
    type SimdTwoBodies = JointTwoBodyConstraint<SimdReal, SIMD_WIDTH>;

    type BuilderOneBody = JointOneBodyConstraintBuilder;
    type BuilderTwoBodies = JointTwoBodyConstraintBuilder;
    type GenericBuilderOneBody = JointGenericOneBodyConstraintBuilder;
    type GenericBuilderTwoBodies = JointGenericTwoBodyConstraintBuilder;
    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderOneBody = JointOneBodyConstraintBuilderSimd;
    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderTwoBodies = JointTwoBodyConstraintBuilderSimd;
}
