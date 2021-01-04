use super::{RevolutePositionConstraint, RevolutePositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, JointIndex, RevoluteJoint, RigidBody};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, Vector, SIMD_WIDTH};
use crate::utils::WAngularInertia;
use na::Unit;

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WRevolutePositionConstraint {
    constraints: [RevolutePositionConstraint; SIMD_WIDTH],
}

impl WRevolutePositionConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&RevoluteJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| RevolutePositionConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii]); SIMD_WIDTH],
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
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&RevoluteJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| RevolutePositionGroundConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii], flipped[ii]); SIMD_WIDTH],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}
