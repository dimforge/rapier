use crate::dynamics::{IntegrationParameters, RevoluteJoint, RigidBody};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::Unit;

#[derive(Debug)]
pub(crate) struct RevolutePositionConstraint {
    position1: usize,
    position2: usize,

    local_com1: Point<Real>,
    local_com2: Point<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    ang_inv_lhs: AngularInertia<Real>,

    local_anchor1: Point<Real>,
    local_anchor2: Point<Real>,

    local_axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,
    local_basis1: [Vector<Real>; 2],
    local_basis2: [Vector<Real>; 2],
}

impl RevolutePositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, cparams: &RevoluteJoint) -> Self {
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;
        let ang_inv_lhs = (ii1 + ii2).inverse();

        Self {
            im1,
            im2,
            ii1,
            ii2,
            ang_inv_lhs,
            local_com1: rb1.mass_properties.local_com,
            local_com2: rb2.mass_properties.local_com,
            local_anchor1: cparams.local_anchor1,
            local_anchor2: cparams.local_anchor2,
            local_axis1: cparams.local_axis1,
            local_axis2: cparams.local_axis2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
            local_basis1: cparams.basis1,
            local_basis2: cparams.basis2,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        /*
         * Linear part.
         */
        {
            let anchor1 = position1 * self.local_anchor1;
            let anchor2 = position2 * self.local_anchor2;

            let r1 = anchor1 - position1 * self.local_com1;
            let r2 = anchor2 - position2 * self.local_com2;

            // TODO: don't do the "to_matrix".
            let lhs = (self
                .ii2
                .quadform(&r2.gcross_matrix())
                .add_diagonal(self.im2)
                + self
                    .ii1
                    .quadform(&r1.gcross_matrix())
                    .add_diagonal(self.im1))
            .into_matrix();
            let inv_lhs = lhs.try_inverse().unwrap();

            let delta_tra = anchor2 - anchor1;
            let lin_error = delta_tra * params.joint_erp;
            let lin_impulse = inv_lhs * lin_error;

            let rot1 = self.ii1 * r1.gcross(lin_impulse);
            let rot2 = self.ii2 * r2.gcross(lin_impulse);
            position1.rotation = Rotation::new(rot1) * position1.rotation;
            position2.rotation = Rotation::new(-rot2) * position2.rotation;
            position1.translation.vector += self.im1 * lin_impulse;
            position2.translation.vector -= self.im2 * lin_impulse;
        }

        /*
         * Angular part.
         */
        {
            let axis1 = position1 * self.local_axis1;
            let axis2 = position2 * self.local_axis2;
            let delta_rot =
                Rotation::rotation_between_axis(&axis1, &axis2).unwrap_or_else(Rotation::identity);
            let ang_error = delta_rot.scaled_axis() * params.joint_erp;
            let ang_impulse = self.ang_inv_lhs.transform_vector(ang_error);

            position1.rotation =
                Rotation::new(self.ii1.transform_vector(ang_impulse)) * position1.rotation;
            position2.rotation =
                Rotation::new(self.ii2.transform_vector(-ang_impulse)) * position2.rotation;
        }

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct RevolutePositionGroundConstraint {
    position2: usize,
    local_com2: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    anchor1: Point<Real>,
    local_anchor2: Point<Real>,
    axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,

    basis1: [Vector<Real>; 2],
    local_basis2: [Vector<Real>; 2],
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
        let basis1;
        let local_basis2;

        if flipped {
            anchor1 = rb1.predicted_position * cparams.local_anchor2;
            local_anchor2 = cparams.local_anchor1;
            axis1 = rb1.predicted_position * cparams.local_axis2;
            local_axis2 = cparams.local_axis1;
            basis1 = [
                rb1.predicted_position * cparams.basis2[0],
                rb1.predicted_position * cparams.basis2[1],
            ];
            local_basis2 = cparams.basis1;
        } else {
            anchor1 = rb1.predicted_position * cparams.local_anchor1;
            local_anchor2 = cparams.local_anchor2;
            axis1 = rb1.predicted_position * cparams.local_axis1;
            local_axis2 = cparams.local_axis2;
            basis1 = [
                rb1.predicted_position * cparams.basis1[0],
                rb1.predicted_position * cparams.basis1[1],
            ];
            local_basis2 = cparams.basis2;
        };

        Self {
            anchor1,
            local_anchor2,
            im2: rb2.effective_inv_mass,
            ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
            local_com2: rb2.mass_properties.local_com,
            axis1,
            local_axis2,
            position2: rb2.active_set_offset,
            basis1,
            local_basis2,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2 as usize];

        /*
         * Linear part.
         */
        {
            let anchor2 = position2 * self.local_anchor2;

            let r2 = anchor2 - position2 * self.local_com2;
            // TODO: don't the the "to_matrix".
            let lhs = self
                .ii2
                .quadform(&r2.gcross_matrix())
                .add_diagonal(self.im2)
                .into_matrix();
            let inv_lhs = lhs.try_inverse().unwrap();

            let delta_tra = anchor2 - self.anchor1;
            let lin_error = delta_tra * params.joint_erp;
            let lin_impulse = inv_lhs * lin_error;

            let rot2 = self.ii2 * r2.gcross(lin_impulse);
            position2.rotation = Rotation::new(-rot2) * position2.rotation;
            position2.translation.vector -= self.im2 * lin_impulse;
        }

        /*
         * Angular part.
         */
        {
            let axis2 = position2 * self.local_axis2;
            let delta_rot = Rotation::rotation_between_axis(&self.axis1, &axis2)
                .unwrap_or_else(Rotation::identity);
            let ang_error = delta_rot.scaled_axis() * params.joint_erp;
            position2.rotation = Rotation::new(-ang_error) * position2.rotation;
        }

        positions[self.position2 as usize] = position2;
    }
}
