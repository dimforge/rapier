use crate::dynamics::{
    IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, SpringJoint,
};
use crate::math::{Isometry, Real};

#[derive(Debug)]
pub(crate) struct SpringPositionConstraint;

impl SpringPositionConstraint {
    pub fn from_params(
        _rb1: (&RigidBodyMassProps, &RigidBodyIds),
        _rb2: (&RigidBodyMassProps, &RigidBodyIds),
        _cparams: &SpringJoint,
    ) -> Self {
        Self
    }

    pub fn solve(&self, _params: &IntegrationParameters, _positions: &mut [Isometry<Real>]) {
        //Do nothing
    }
}

pub(crate) struct SpringPositionGroundConstraint;

impl SpringPositionGroundConstraint {
    pub fn from_params(
        _rb1: &RigidBodyPosition,
        _rb2: (&RigidBodyMassProps, &RigidBodyIds),
        _cparams: &SpringJoint,
        _flipped: bool,
    ) -> Self {
        Self
    }

    pub fn solve(&self, _params: &IntegrationParameters, _positions: &mut [Isometry<Real>]) {
        //Do nothing
    }
}
