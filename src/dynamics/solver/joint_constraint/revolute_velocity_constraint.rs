use crate::dynamics::solver::{AnyJointVelocityConstraint, AnyVelocityConstraint, DeltaVel};
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RevoluteJoint, RigidBody,
};
use crate::math::{AngularInertia, Real, Rotation, Vector};
use crate::na::UnitQuaternion;
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use downcast_rs::Downcast;
use na::{Cholesky, Matrix3, Matrix3x2, Matrix5, RealField, Vector5, U2, U3};

#[derive(Debug)]
pub(crate) struct RevoluteVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1_mat: Matrix3<Real>,
    r2_mat: Matrix3<Real>,

    inv_lhs: Matrix5<Real>,
    rhs: Vector5<Real>,
    impulse: Vector5<Real>,

    motor_inv_lhs: Real,
    motor_rhs: Real,
    motor_impulse: Real,
    motor_max_impulse: Real,
    motor_angle: Real, // Exists only to write it back into the joint.

    motor_axis1: Vector<Real>,
    motor_axis2: Vector<Real>,

    basis1: Matrix3x2<Real>,
    basis2: Matrix3x2<Real>,

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl RevoluteVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &RevoluteJoint,
    ) -> Self {
        // Linear part.
        let anchor1 = rb1.position * joint.local_anchor1;
        let anchor2 = rb2.position * joint.local_anchor2;
        let basis1 = Matrix3x2::from_columns(&[
            rb1.position * joint.basis1[0],
            rb1.position * joint.basis1[1],
        ]);
        let basis_projection1 = basis1 * basis1.transpose();

        let basis_projection_half2 = Matrix3x2::from_columns(&[
            rb2.position * joint.basis2[0],
            rb2.position * joint.basis2[1],
        ]);
        let basis_projection2 = basis_projection_half2 * basis_projection_half2.transpose();
        let basis2 = basis_projection2 * basis1;

        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;

        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r1_mat = basis_projection1 * r1.gcross_matrix();

        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = basis_projection2 * r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 =
            ii2.quadform(&r2_mat).add_diagonal(im2) + ii1.quadform(&r1_mat).add_diagonal(im1);
        let lhs10 = basis2.tr_mul(&(ii2 * r2_mat)) + basis1.tr_mul(&(ii1 * r1_mat));
        let lhs11 = (ii1.quadform3x2(&basis1) + ii2.quadform3x2(&basis2)).into_matrix();

        // Note that Cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = (rb2.linvel - r2_mat * rb2.angvel) - (rb1.linvel - r1_mat * rb1.angvel);
        let ang_rhs = basis2.tr_mul(&rb2.angvel) - basis1.tr_mul(&rb1.angvel);
        let mut rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        /*
         * Motor.
         */
        let motor_axis1 = rb1.position * *joint.local_axis1;
        let motor_axis2 = rb2.position * *joint.local_axis2;
        let mut motor_rhs = 0.0;
        let mut motor_inv_lhs = 0.0;
        let mut motor_max_impulse = joint.motor_max_impulse;
        let mut motor_angle = 0.0;

        let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
            params.dt,
            joint.motor_stiffness,
            joint.motor_damping,
        );

        if stiffness != 0.0 {
            motor_angle = joint.estimate_motor_angle(&rb1.position, &rb2.position);
            motor_rhs += (motor_angle - joint.motor_target_pos) * stiffness;
        }

        if damping != 0.0 {
            let curr_vel = rb2.angvel.dot(&motor_axis2) - rb1.angvel.dot(&motor_axis1);
            motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
        }

        if stiffness != 0.0 || damping != 0.0 {
            motor_inv_lhs = if keep_lhs {
                crate::utils::inv(
                    motor_axis2.dot(&ii2.transform_vector(motor_axis2))
                        + motor_axis1.dot(&ii1.transform_vector(motor_axis1)),
                ) * gamma
            } else {
                gamma
            };
            motor_rhs /= gamma;
        }

        /*
         * Adjust the warmstart impulse.
         * If the velocity along the free axis is somewhat high,
         * we need to adjust the angular warmstart impulse because it
         * may have a direction that is too different than last frame,
         * making it counter-productive.
         */
        let mut impulse = joint.impulse * params.warmstart_coeff;
        let axis_rot = Rotation::rotation_between(&joint.prev_axis1, &motor_axis1)
            .unwrap_or_else(UnitQuaternion::identity);
        let rotated_impulse = basis1.tr_mul(&(axis_rot * joint.world_ang_impulse));
        impulse[3] = rotated_impulse.x * params.warmstart_coeff;
        impulse[4] = rotated_impulse.y * params.warmstart_coeff;
        let motor_impulse = na::clamp(joint.motor_impulse, -motor_max_impulse, motor_max_impulse)
            * params.warmstart_coeff;

        RevoluteVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            ii1_sqrt: rb1.effective_world_inv_inertia_sqrt,
            basis1,
            basis2,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse,
            inv_lhs,
            rhs,
            r1_mat,
            r2_mat,
            motor_rhs,
            motor_inv_lhs,
            motor_max_impulse,
            motor_axis1,
            motor_axis2,
            motor_impulse,
            motor_angle,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse1 = self.impulse.fixed_rows::<U3>(0).into_owned();
        let lin_impulse2 = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse1 = self.basis1 * self.impulse.fixed_rows::<U2>(3).into_owned();
        let ang_impulse2 = self.basis2 * self.impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse1 + self.r1_mat * lin_impulse1);

        mj_lambda2.linear -= self.im2 * lin_impulse2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse2 + self.r2_mat * lin_impulse2);

        /*
         * Motor
         */
        if self.motor_inv_lhs != 0.0 {
            mj_lambda1.angular += self
                .ii1_sqrt
                .transform_vector(self.motor_axis1 * self.motor_impulse);
            mj_lambda2.angular -= self
                .ii2_sqrt
                .transform_vector(self.motor_axis2 * self.motor_impulse);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let lin_dvel = (mj_lambda2.linear - self.r2_mat * ang_vel2)
            - (mj_lambda1.linear - self.r1_mat * ang_vel1);
        let ang_dvel = self.basis2.tr_mul(&ang_vel2) - self.basis1.tr_mul(&ang_vel1);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse1 = impulse.fixed_rows::<U3>(0).into_owned();
        let lin_impulse2 = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse1 = self.basis1 * impulse.fixed_rows::<U2>(3).into_owned();
        let ang_impulse2 = self.basis2 * impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse1 + self.r1_mat * lin_impulse1);

        mj_lambda2.linear -= self.im2 * lin_impulse2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse2 + self.r2_mat * lin_impulse2);

        /*
         * Motor.
         */
        if self.motor_inv_lhs != 0.0 {
            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
            let ang_dvel = ang_vel2.dot(&self.motor_axis2) - ang_vel1.dot(&self.motor_axis1);
            let rhs = ang_dvel + self.motor_rhs;

            let new_motor_impulse = na::clamp(
                self.motor_impulse + self.motor_inv_lhs * rhs,
                -self.motor_max_impulse,
                self.motor_max_impulse,
            );
            let impulse = new_motor_impulse - self.motor_impulse;
            self.motor_impulse = new_motor_impulse;

            mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.motor_axis1 * impulse);
            mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.motor_axis2 * impulse);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::RevoluteJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            let rot_part = self.impulse.fixed_rows::<U2>(3).into_owned();
            revolute.world_ang_impulse = self.basis1 * rot_part;
            revolute.prev_axis1 = self.motor_axis1;
            revolute.motor_last_angle = self.motor_angle;
            revolute.motor_impulse = self.motor_impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct RevoluteVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    r2: Vector<Real>,

    inv_lhs: Matrix5<Real>,
    rhs: Vector5<Real>,
    impulse: Vector5<Real>,

    motor_axis2: Vector<Real>,
    motor_inv_lhs: Real,
    motor_rhs: Real,
    motor_impulse: Real,
    motor_max_impulse: Real,
    motor_angle: Real, // Exists just for writing it into the joint.

    basis2: Matrix3x2<Real>,

    im2: Real,

    ii2_sqrt: AngularInertia<Real>,
}

impl RevoluteVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &RevoluteJoint,
        flipped: bool,
    ) -> AnyJointVelocityConstraint {
        let anchor2;
        let anchor1;
        let basis2;

        if flipped {
            anchor1 = rb1.position * joint.local_anchor2;
            anchor2 = rb2.position * joint.local_anchor1;
            basis2 = Matrix3x2::from_columns(&[
                rb2.position * joint.basis1[0],
                rb2.position * joint.basis1[1],
            ]);
        } else {
            anchor1 = rb1.position * joint.local_anchor1;
            anchor2 = rb2.position * joint.local_anchor2;
            basis2 = Matrix3x2::from_columns(&[
                rb2.position * joint.basis2[0],
                rb2.position * joint.basis2[1],
            ]);
        };

        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = ii2.quadform(&r2_mat).add_diagonal(im2);
        let lhs10 = basis2.tr_mul(&(ii2 * r2_mat));
        let lhs11 = ii2.quadform3x2(&basis2).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = rb2.linvel + rb2.angvel.gcross(r2) - rb1.linvel - rb1.angvel.gcross(r1);
        let ang_rhs = basis2.tr_mul(&(rb2.angvel - rb1.angvel));
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        /*
         * Motor part.
         */
        let motor_axis1 = rb1.position * *joint.local_axis1;
        let motor_axis2 = rb2.position * *joint.local_axis2;
        let mut motor_rhs = 0.0;
        let mut motor_inv_lhs = 0.0;
        let mut motor_max_impulse = joint.motor_max_impulse;
        let mut motor_angle = 0.0;

        let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
            params.dt,
            joint.motor_stiffness,
            joint.motor_damping,
        );

        if stiffness != 0.0 {
            motor_angle = joint.estimate_motor_angle(&rb1.position, &rb2.position);
            motor_rhs += (motor_angle - joint.motor_target_pos) * stiffness;
        }

        if damping != 0.0 {
            let curr_vel = rb2.angvel.dot(&motor_axis2) - rb1.angvel.dot(&motor_axis1);
            motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
        }

        if stiffness != 0.0 || damping != 0.0 {
            motor_inv_lhs = if keep_lhs {
                crate::utils::inv(motor_axis2.dot(&ii2.transform_vector(motor_axis2))) * gamma
            } else {
                gamma
            };
            motor_rhs /= gamma;
        }

        let motor_impulse = na::clamp(joint.motor_impulse, -motor_max_impulse, motor_max_impulse)
            * params.warmstart_coeff;

        let result = RevoluteVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse: joint.impulse * params.warmstart_coeff,
            basis2,
            inv_lhs,
            rhs,
            r2,
            motor_inv_lhs,
            motor_impulse,
            motor_axis2,
            motor_max_impulse,
            motor_rhs,
            motor_angle,
        };

        AnyJointVelocityConstraint::RevoluteGroundConstraint(result)
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis2 * self.impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Motor
         */
        if self.motor_inv_lhs != 0.0 {
            mj_lambda2.angular -= self
                .ii2_sqrt
                .transform_vector(self.motor_axis2 * self.motor_impulse);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let ang_vel2 = ang_vel2 - self.motor_axis2 * ang_vel2.dot(&self.motor_axis2);

        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let ang_dvel = self.basis2.tr_mul(&ang_vel2);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis2 * impulse.fixed_rows::<U2>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Motor.
         */
        if self.motor_inv_lhs != 0.0 {
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
            let ang_dvel = ang_vel2.dot(&self.motor_axis2);
            let rhs = ang_dvel + self.motor_rhs;

            let new_motor_impulse = na::clamp(
                self.motor_impulse + self.motor_inv_lhs * rhs,
                -self.motor_max_impulse,
                self.motor_max_impulse,
            );
            let impulse = new_motor_impulse - self.motor_impulse;
            self.motor_impulse = new_motor_impulse;

            mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.motor_axis2 * impulse);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::RevoluteJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            revolute.motor_impulse = self.motor_impulse;
            revolute.motor_last_angle = self.motor_angle;
        }
    }
}
