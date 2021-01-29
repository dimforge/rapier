use super::{FixedPositionConstraint, FixedPositionGroundConstraint};
use crate::dynamics::{FixedJoint, IntegrationParameters, RigidBody};
use crate::math::{Isometry, Real, SIMD_WIDTH};

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WFixedPositionConstraint {
    constraints: [FixedPositionConstraint; SIMD_WIDTH],
}

impl WFixedPositionConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&FixedJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| FixedPositionConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii]); SIMD_WIDTH],
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
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&FixedJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| FixedPositionGroundConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii], flipped[ii]); SIMD_WIDTH],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}
