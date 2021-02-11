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
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;
        let r1 = Point::from(anchor1.translation.vector) - position1 * self.local_com1;
        let r2 = Point::from(anchor2.translation.vector) - position2 * self.local_com2;

        let delta_pos = Isometry::from_parts(
            anchor2.translation * anchor1.translation.inverse(),
            anchor2.rotation * anchor1.rotation.inverse(),
        );

        let mass_matrix = GenericVelocityConstraint::compute_mass_matrix(
            &self.joint,
            self.im1,
            self.im2,
            self.ii1,
            self.ii2,
            r1,
            r2,
            false,
        );

        let lin_dpos = delta_pos.translation.vector;
        let ang_dpos = delta_pos.rotation.scaled_axis();
        let dpos = Vector6::new(
            lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
        );
        let err = dpos
            - dpos
                .sup(&self.joint.min_position)
                .inf(&self.joint.max_position);
        let impulse = mass_matrix * err;
        let lin_impulse = impulse.xyz();
        let ang_impulse = Vector3::new(impulse[3], impulse[4], impulse[5]);

        position1.rotation = Rotation::new(
            self.ii1
                .transform_vector(ang_impulse + r1.gcross(lin_impulse)),
        ) * position1.rotation;
        position2.rotation = Rotation::new(
            self.ii2
                .transform_vector(-ang_impulse - r2.gcross(lin_impulse)),
        ) * position2.rotation;

        position1.translation.vector += self.im1 * lin_impulse;
        position2.translation.vector -= self.im2 * lin_impulse;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
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
        let mut position2 = positions[self.position2 as usize];

        let anchor2 = position2 * self.local_anchor2;
        let r2 = Point::from(anchor2.translation.vector) - position2 * self.local_com2;

        let delta_pos = Isometry::from_parts(
            anchor2.translation * self.anchor1.translation.inverse(),
            anchor2.rotation * self.anchor1.rotation.inverse(),
        );
        let mass_matrix = GenericVelocityGroundConstraint::compute_mass_matrix(
            &self.joint,
            self.im2,
            self.ii2,
            r2,
            false,
        );

        let lin_dpos = delta_pos.translation.vector;
        let ang_dpos = delta_pos.rotation.scaled_axis();
        let dpos = Vector6::new(
            lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
        );
        let err = dpos
            - dpos
                .sup(&self.joint.min_position)
                .inf(&self.joint.max_position);
        let impulse = mass_matrix * err;
        let lin_impulse = impulse.xyz();
        let ang_impulse = Vector3::new(impulse[3], impulse[4], impulse[5]);

        position2.rotation = Rotation::new(
            self.ii2
                .transform_vector(-ang_impulse - r2.gcross(lin_impulse)),
        ) * position2.rotation;
        position2.translation.vector -= self.im2 * lin_impulse;

        positions[self.position2 as usize] = position2;
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
