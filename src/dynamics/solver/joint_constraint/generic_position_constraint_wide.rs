use super::{GenericPositionConstraint, GenericPositionGroundConstraint};
use crate::dynamics::{GenericJoint, IntegrationParameters, RigidBody};
use crate::math::{Isometry, Real, SIMD_WIDTH};

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WGenericPositionConstraint {
    constraints: [GenericPositionConstraint; SIMD_WIDTH],
}

impl WGenericPositionConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&GenericJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| GenericPositionConstraint::from_params(
                rbs1[ii],
                rbs2[ii],
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
pub(crate) struct WGenericPositionGroundConstraint {
    constraints: [GenericPositionGroundConstraint; SIMD_WIDTH],
}

impl WGenericPositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&GenericJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: gather![|ii| GenericPositionGroundConstraint::from_params(
                rbs1[ii],
                rbs2[ii],
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
