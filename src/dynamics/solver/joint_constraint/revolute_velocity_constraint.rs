use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RevoluteJoint, RigidBody,
};
use crate::math::{AngularInertia, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::{Cholesky, Matrix3x2, Matrix5, Vector5, U2, U3};

#[derive(Debug)]
pub(crate) struct RevoluteVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1: Vector<f32>,
    r2: Vector<f32>,

    inv_lhs: Matrix5<f32>,
    rhs: Vector5<f32>,
    impulse: Vector5<f32>,

    basis1: Matrix3x2<f32>,

    im1: f32,
    im2: f32,

    ii1_sqrt: AngularInertia<f32>,
    ii2_sqrt: AngularInertia<f32>,
}

impl RevoluteVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &RevoluteJoint,
    ) -> Self {
        // Linear part.
        let anchor1 = rb1.position * cparams.local_anchor1;
        let anchor2 = rb2.position * cparams.local_anchor2;
        let basis1 = Matrix3x2::from_columns(&[
            rb1.position * cparams.basis1[0],
            rb1.position * cparams.basis1[1],
        ]);

        //        let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //            .unwrap_or(Rotation::identity())
        //            .to_rotation_matrix()
        //            .into_inner();
        //        let basis2 = r21 * basis1;
        // NOTE: to simplify, we use basis2 = basis1.
        // Though we may want to test if that does not introduce any instability.
        let im1 = rb1.mass_properties.inv_mass;
        let im2 = rb2.mass_properties.inv_mass;

        let ii1 = rb1.world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r1_mat = r1.gcross_matrix();

        let ii2 = rb2.world_inv_inertia_sqrt.squared();
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 =
            ii2.quadform(&r2_mat).add_diagonal(im2) + ii1.quadform(&r1_mat).add_diagonal(im1);
        let lhs10 = basis1.tr_mul(&(ii2 * r2_mat + ii1 * r1_mat));
        let lhs11 = (ii1 + ii2).quadform3x2(&basis1).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = rb2.linvel + rb2.angvel.gcross(r2) - rb1.linvel - rb1.angvel.gcross(r1);
        let ang_rhs = basis1.tr_mul(&(rb2.angvel - rb1.angvel));
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        RevoluteVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            ii1_sqrt: rb1.world_inv_inertia_sqrt,
            basis1,
            im2,
            ii2_sqrt: rb2.world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            inv_lhs,
            rhs,
            r1,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * self.impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2)
            - mj_lambda1.linear
            - ang_vel1.gcross(self.r1);
        let ang_dvel = self.basis1.tr_mul(&(ang_vel2 - ang_vel1));
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::RevoluteJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct RevoluteVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    r2: Vector<f32>,

    inv_lhs: Matrix5<f32>,
    rhs: Vector5<f32>,
    impulse: Vector5<f32>,

    basis1: Matrix3x2<f32>,

    im2: f32,

    ii2_sqrt: AngularInertia<f32>,
}

impl RevoluteVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &RevoluteJoint,
        flipped: bool,
    ) -> Self {
        let anchor2;
        let anchor1;
        let basis1;

        if flipped {
            anchor1 = rb1.position * cparams.local_anchor2;
            anchor2 = rb2.position * cparams.local_anchor1;
            basis1 = Matrix3x2::from_columns(&[
                rb1.position * cparams.basis2[0],
                rb1.position * cparams.basis2[1],
            ]);
        } else {
            anchor1 = rb1.position * cparams.local_anchor1;
            anchor2 = rb2.position * cparams.local_anchor2;
            basis1 = Matrix3x2::from_columns(&[
                rb1.position * cparams.basis1[0],
                rb1.position * cparams.basis1[1],
            ]);
        };

        //        let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //            .unwrap_or(Rotation::identity())
        //            .to_rotation_matrix()
        //            .into_inner();
        //        let basis2 = /*r21 * */ basis1;
        let im2 = rb2.mass_properties.inv_mass;
        let ii2 = rb2.world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = ii2.quadform(&r2_mat).add_diagonal(im2);
        let lhs10 = basis1.tr_mul(&(ii2 * r2_mat));
        let lhs11 = ii2.quadform3x2(&basis1).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = rb2.linvel + rb2.angvel.gcross(r2) - rb1.linvel - rb1.angvel.gcross(r1);
        let ang_rhs = basis1.tr_mul(&(rb2.angvel - rb1.angvel));
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        RevoluteVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2_sqrt: rb2.world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            basis1,
            inv_lhs,
            rhs,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * self.impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let ang_dvel = self.basis1.tr_mul(&ang_vel2);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::RevoluteJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
        }
    }
}
