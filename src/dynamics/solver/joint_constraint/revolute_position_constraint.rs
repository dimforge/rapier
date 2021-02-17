use crate::dynamics::{IntegrationParameters, RevoluteJoint, RigidBody};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::{Matrix3x2, Matrix5, Unit};

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

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;
        let axis1 = position1 * self.local_axis1;
        let axis2 = position2 * self.local_axis2;

        let basis1 = Matrix3x2::from_columns(&[
            position1 * self.local_basis1[0],
            position1 * self.local_basis1[1],
        ]);
        let basis2 = Matrix3x2::from_columns(&[
            position2 * self.local_basis2[0],
            position2 * self.local_basis2[1],
        ]);

        let basis_filter1 = basis1 * basis1.transpose();
        let basis_filter2 = basis2 * basis2.transpose();
        let basis2 = basis_filter2 * basis1;

        let r1 = anchor1 - position1 * self.local_com1;
        let r2 = anchor2 - position2 * self.local_com2;
        let r1_mat = basis_filter1 * r1.gcross_matrix();
        let r2_mat = basis_filter2 * r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = self.ii2.quadform(&r2_mat).add_diagonal(self.im2)
            + self.ii1.quadform(&r1_mat).add_diagonal(self.im1);
        let lhs10 = basis2.tr_mul(&(self.ii2 * r2_mat)) + basis1.tr_mul(&(self.ii1 * r1_mat));
        let lhs11 = (self.ii1.quadform3x2(&basis1) + self.ii2.quadform3x2(&basis2)).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<na::U3, na::U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<na::U2, na::U3>(3, 0)
            .copy_from(&lhs10);
        lhs.fixed_slice_mut::<na::U2, na::U2>(3, 3)
            .copy_from(&lhs11);

        let inv_lhs = na::Cholesky::new_unchecked(lhs).inverse();

        let delta_tra = anchor2 - anchor1;
        let lin_error = delta_tra * params.joint_erp;
        let delta_rot =
            Rotation::rotation_between_axis(&axis1, &axis2).unwrap_or_else(Rotation::identity);

        let ang_error = basis1.tr_mul(&delta_rot.scaled_axis()) * params.joint_erp;
        let error = na::Vector5::new(
            lin_error.x,
            lin_error.y,
            lin_error.z,
            ang_error.x,
            ang_error.y,
        );
        let impulse = inv_lhs * error;
        let lin_impulse = impulse.fixed_rows::<na::U3>(0).into_owned();
        let ang_impulse1 = basis1 * impulse.fixed_rows::<na::U2>(3).into_owned();
        let ang_impulse2 = basis2 * impulse.fixed_rows::<na::U2>(3).into_owned();

        let rot1 = self.ii1 * (r1_mat * lin_impulse + ang_impulse1);
        let rot2 = self.ii2 * (r2_mat * lin_impulse + ang_impulse2);
        position1.rotation = Rotation::new(rot1) * position1.rotation;
        position2.rotation = Rotation::new(-rot2) * position2.rotation;
        position1.translation.vector += self.im1 * lin_impulse;
        position2.translation.vector -= self.im2 * lin_impulse;

        /*
        /*
         * Linear part.
         */
        {
            let anchor1 = position1 * self.local_anchor1;
            let anchor2 = position2 * self.local_anchor2;

            let r1 = anchor1 - position1 * self.local_com1;
            let r2 = anchor2 - position2 * self.local_com2;
            // TODO: don't the the "to_matrix".
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
         */

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

        let anchor1 = self.anchor1;
        let anchor2 = position2 * self.local_anchor2;
        let axis1 = self.axis1;
        let axis2 = position2 * self.local_axis2;

        let basis1 = Matrix3x2::from_columns(&self.basis1[..]);
        let basis2 = Matrix3x2::from_columns(&[
            position2 * self.local_basis2[0],
            position2 * self.local_basis2[1],
        ]);

        let basis_filter2 = basis2 * basis2.transpose();
        let basis2 = basis_filter2 * basis1;

        let r2 = anchor2 - position2 * self.local_com2;
        let r2_mat = basis_filter2 * r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = self.ii2.quadform(&r2_mat).add_diagonal(self.im2);
        let lhs10 = basis2.tr_mul(&(self.ii2 * r2_mat));
        let lhs11 = self.ii2.quadform3x2(&basis2).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<na::U3, na::U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<na::U2, na::U3>(3, 0)
            .copy_from(&lhs10);
        lhs.fixed_slice_mut::<na::U2, na::U2>(3, 3)
            .copy_from(&lhs11);

        let inv_lhs = na::Cholesky::new_unchecked(lhs).inverse();

        let delta_tra = anchor2 - anchor1;
        let lin_error = delta_tra * params.joint_erp;
        let delta_rot =
            Rotation::rotation_between_axis(&axis1, &axis2).unwrap_or_else(Rotation::identity);

        let ang_error = basis1.tr_mul(&delta_rot.scaled_axis()) * params.joint_erp;
        let error = na::Vector5::new(
            lin_error.x,
            lin_error.y,
            lin_error.z,
            ang_error.x,
            ang_error.y,
        );
        let impulse = inv_lhs * error;
        let lin_impulse = impulse.fixed_rows::<na::U3>(0).into_owned();
        let ang_impulse2 = basis2 * impulse.fixed_rows::<na::U2>(3).into_owned();

        let rot2 = self.ii2 * (r2_mat * lin_impulse + ang_impulse2);
        position2.rotation = Rotation::new(-rot2) * position2.rotation;
        position2.translation.vector -= self.im2 * lin_impulse;

        /*
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
        */

        positions[self.position2 as usize] = position2;
    }
}
