use super::{PrismaticPositionConstraint, PrismaticPositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, PrismaticJoint, RigidBody};
use crate::math::{Isometry, Real, SIMD_WIDTH};

// TODO: this does not uses SIMD optimizations yet.
#[derive(Debug)]
pub(crate) struct WPrismaticPositionConstraint {
    constraints: [PrismaticPositionConstraint; SIMD_WIDTH],
}

impl WPrismaticPositionConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&PrismaticJoint; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| PrismaticPositionConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii]); SIMD_WIDTH],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}

#[derive(Debug)]
pub(crate) struct WPrismaticPositionGroundConstraint {
    constraints: [PrismaticPositionGroundConstraint; SIMD_WIDTH],
}

impl WPrismaticPositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&PrismaticJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {
            constraints: array![|ii| PrismaticPositionGroundConstraint::from_params(rbs1[ii], rbs2[ii], cparams[ii], flipped[ii]); SIMD_WIDTH],
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        for constraint in &self.constraints {
            constraint.solve(params, positions);
        }
    }
}
