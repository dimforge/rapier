use super::{RevolutePositionConstraint, RevolutePositionGroundConstraint};
use crate::dynamics::{
    IntegrationParameters, RevoluteJoint, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition,
};
use crate::math::{Isometry, Real, SIMD_WIDTH};

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WRevolutePositionConstraint {
    constraints: [RevolutePositionConstraint; SIMD_WIDTH],
}

impl WRevolutePositionConstraint {
    pub fn from_params(
        rbs1: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&RevoluteJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| RevolutePositionConstraint::from_params(
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
pub(crate) struct WRevolutePositionGroundConstraint {
    constraints: [RevolutePositionGroundConstraint; SIMD_WIDTH],
}

impl WRevolutePositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBodyPosition; SIMD_WIDTH],
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&RevoluteJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| RevolutePositionGroundConstraint::from_params(
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
