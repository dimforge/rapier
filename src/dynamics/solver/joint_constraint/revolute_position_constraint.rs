use crate::dynamics::{IntegrationParameters, RevoluteJoint, RigidBody};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, Vector};
use crate::utils::WAngularInertia;
use na::Unit;

#[derive(Debug)]
pub(crate) struct RevolutePositionConstraint {
    position1: usize,
    position2: usize,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    lin_inv_lhs: Real,
    ang_inv_lhs: AngularInertia<Real>,

    local_anchor1: Point<Real>,
    local_anchor2: Point<Real>,

    local_axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,
}

impl RevolutePositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, cparams: &RevoluteJoint) -> Self {
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;
        let lin_inv_lhs = 1.0 / (im1 + im2);
        let ang_inv_lhs = (ii1 + ii2).inverse();

        Self {
            im1,
            im2,
            ii1,
            ii2,
            lin_inv_lhs,
            ang_inv_lhs,
            local_anchor1: cparams.local_anchor1,
            local_anchor2: cparams.local_anchor2,
            local_axis1: cparams.local_axis1,
            local_axis2: cparams.local_axis2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        let axis1 = position1 * self.local_axis1;
        let axis2 = position2 * self.local_axis2;
        let delta_rot =
            Rotation::rotation_between_axis(&axis1, &axis2).unwrap_or(Rotation::identity());
        let ang_error = delta_rot.scaled_axis() * params.joint_erp;
        let ang_impulse = self.ang_inv_lhs.transform_vector(ang_error);

        position1.rotation =
            Rotation::new(self.ii1.transform_vector(ang_impulse)) * position1.rotation;
        position2.rotation =
            Rotation::new(self.ii2.transform_vector(-ang_impulse)) * position2.rotation;

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;

        let delta_tra = anchor2 - anchor1;
        let lin_error = delta_tra * params.joint_erp;
        let lin_impulse = self.lin_inv_lhs * lin_error;

        position1.translation.vector += self.im1 * lin_impulse;
        position2.translation.vector -= self.im2 * lin_impulse;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct RevolutePositionGroundConstraint {
    position2: usize,
    anchor1: Point<Real>,
    local_anchor2: Point<Real>,
    axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,
}

impl RevolutePositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &RevoluteJoint,
        flipped: bool,
    ) -> Self {
        let anchor1;
        let local_anchor2;
        let axis1;
        let local_axis2;

        if flipped {
            anchor1 = rb1.predicted_position * cparams.local_anchor2;
            local_anchor2 = cparams.local_anchor1;
            axis1 = rb1.predicted_position * cparams.local_axis2;
            local_axis2 = cparams.local_axis1;
        } else {
            anchor1 = rb1.predicted_position * cparams.local_anchor1;
            local_anchor2 = cparams.local_anchor2;
            axis1 = rb1.predicted_position * cparams.local_axis1;
            local_axis2 = cparams.local_axis2;
        };

        Self {
            anchor1,
            local_anchor2,
            axis1,
            local_axis2,
            position2: rb2.active_set_offset,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2 as usize];

        let axis2 = position2 * self.local_axis2;

        let delta_rot =
            Rotation::scaled_rotation_between_axis(&axis2, &self.axis1, params.joint_erp)
                .unwrap_or(Rotation::identity());
        position2.rotation = delta_rot * position2.rotation;

        let anchor2 = position2 * self.local_anchor2;
        let delta_tra = anchor2 - self.anchor1;
        let lin_error = delta_tra * params.joint_erp;
        position2.translation.vector -= lin_error;

        positions[self.position2 as usize] = position2;
    }
}
