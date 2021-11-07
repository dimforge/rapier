use crate::dynamics::{
    IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, SpringJoint,
};
use crate::math::{Isometry, Real, SIMD_WIDTH};

#[derive(Debug)]
pub(crate) struct WSpringPositionConstraint {}

impl WSpringPositionConstraint {
    pub fn from_params(
        _rbs1: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        _rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        _cparams: [&SpringJoint; SIMD_WIDTH],
    ) -> Self {
        Self {}
    }

    pub fn solve(&self, _params: &IntegrationParameters, _positions: &mut [Isometry<Real>]) {
        // Do nothing
    }
}

#[derive(Debug)]
pub(crate) struct WSpringPositionGroundConstraint {}

impl WSpringPositionGroundConstraint {
    pub fn from_params(
        _rbs1: [&RigidBodyPosition; SIMD_WIDTH],
        _rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        _cparams: [&SpringJoint; SIMD_WIDTH],
        _flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        Self {}
    }

    pub fn solve(&self, _params: &IntegrationParameters, _positions: &mut [Isometry<Real>]) {
        // Do nothing
    }
}
