use super::{GenericVelocityConstraint, GenericVelocityGroundConstraint};
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{GenericJoint, IntegrationParameters, RigidBody};
use crate::math::{
    AngDim, AngVector, AngularInertia, Dim, Isometry, Point, Real, Rotation, SpatialVector, Vector,
    DIM,
};
use crate::utils::{WAngularInertia, WCross};
use na::{Vector3, Vector6};

// FIXME: review this code for the case where the center of masses are not the origin.
#[derive(Debug)]
pub(crate) struct GenericPositionConstraint {
    position1: usize,
    position2: usize,
    local_anchor1: Isometry<Real>,
    local_anchor2: Isometry<Real>,
    local_com1: Point<Real>,
    local_com2: Point<Real>,
    im1: Real,
    im2: Real,
    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    joint: GenericJoint,
}

impl GenericPositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, joint: &GenericJoint) -> Self {
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;

        Self {
            local_anchor1: joint.local_anchor1,
            local_anchor2: joint.local_anchor2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
            im1,
            im2,
            ii1,
            ii2,
            local_com1: rb1.mass_properties.local_com,
            local_com2: rb2.mass_properties.local_com,
            joint: *joint,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        return;
    }

    pub fn solve2(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<Real>],
        dpos: &mut [DeltaVel<Real>],
    ) {
        return;
    }
}

#[derive(Debug)]
pub(crate) struct GenericPositionGroundConstraint {
    position2: usize,
    anchor1: Isometry<Real>,
    local_anchor2: Isometry<Real>,
    local_com2: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    joint: GenericJoint,
}

impl GenericPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &GenericJoint,
        flipped: bool,
    ) -> Self {
        let anchor1;
        let local_anchor2;

        if flipped {
            anchor1 = rb1.predicted_position * joint.local_anchor2;
            local_anchor2 = joint.local_anchor1;
        } else {
            anchor1 = rb1.predicted_position * joint.local_anchor1;
            local_anchor2 = joint.local_anchor2;
        };

        Self {
            anchor1,
            local_anchor2,
            position2: rb2.active_set_offset,
            im2: rb2.effective_inv_mass,
            ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
            local_com2: rb2.mass_properties.local_com,
            joint: *joint,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        return;
    }

    pub fn solve2(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<Real>],
        dpos: &mut [DeltaVel<Real>],
    ) {
        return;
    }
}
