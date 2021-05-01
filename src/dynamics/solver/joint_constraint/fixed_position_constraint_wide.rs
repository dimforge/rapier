use super::{FixedPositionConstraint, FixedPositionGroundConstraint};
use crate::dynamics::{
    FixedJoint, IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition,
};
use crate::math::{Isometry, Real, SIMD_WIDTH};

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WFixedPositionConstraint {
    constraints: [FixedPositionConstraint; SIMD_WIDTH],
}

impl WFixedPositionConstraint {
    pub fn from_params(
        rbs1: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&FixedJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| FixedPositionConstraint::from_params(
                (rbs1.0[ii], rbs1.1[ii]),
                (rbs2.0[ii], rbs2.1[ii]),
                cparams[ii]
            )],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}

#[derive(Debug)]
pub(crate) struct WFixedPositionGroundConstraint {
    constraints: [FixedPositionGroundConstraint; SIMD_WIDTH],
}

impl WFixedPositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBodyPosition; SIMD_WIDTH],
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&FixedJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| FixedPositionGroundConstraint::from_params(
                rbs1[ii],
                (rbs2.0[ii], rbs2.1[ii]),
                cparams[ii],
                flipped[ii]
            )],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}
