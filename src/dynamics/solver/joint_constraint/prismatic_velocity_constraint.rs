use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, PrismaticJoint, RigidBody,
};
use crate::math::{AngularInertia, Real, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix, WDot};
#[cfg(feature = "dim3")]
use na::{Cholesky, Matrix3x2, Matrix5, Vector5, U2, U3};
#[cfg(feature = "dim2")]
use {
    na::{Matrix2, Vector2},
    parry::utils::SdpMatrix2,
};

#[cfg(feature = "dim2")]
type LinImpulseDim = na::U1;
#[cfg(feature = "dim3")]
type LinImpulseDim = na::U2;

#[derive(Debug)]
pub(crate) struct PrismaticVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1: Vector<Real>,
    r2: Vector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<Real>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<Real>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<Real>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<Real>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<Real>,

    motor_axis1: Vector<Real>,
    motor_axis2: Vector<Real>,
    motor_impulse: Real,
    motor_rhs: Real,
    motor_inv_lhs: Real,
    motor_max_impulse: Real,

    limits_active: bool,
    limits_impulse: Real,
    /// World-coordinate direction of the limit force on rb2.
    /// The force direction on rb1 is opposite (Newton's third law)..
    limits_forcedir2: Vector<Real>,
    limits_rhs: Real,
    limits_inv_lhs: Real,
    /// min/max applied impulse due to limits
    limits_impulse_limits: (Real, Real),

    #[cfg(feature = "dim2")]
    basis1: Vector2<Real>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<Real>,

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl PrismaticVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &PrismaticJoint,
    ) -> Self {
        // Linear part.
        let anchor1 = rb1.position * joint.local_anchor1;
        let anchor2 = rb2.position * joint.local_anchor2;
        let axis1 = rb1.position * joint.local_axis1;
        let axis2 = rb2.position * joint.local_axis2;
        #[cfg(feature = "dim2")]
        let basis1 = rb1.position * joint.basis1[0];
        #[cfg(feature = "dim3")]
        let basis1 = Matrix3x2::from_columns(&[
            rb1.position * joint.basis1[0],
            rb1.position * joint.basis1[1],
        ]);

        let im1 = rb1.effective_inv_mass;
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r1_mat = r1.gcross_matrix();

        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let r1_mat_b1 = r1_mat * basis1;
            let r2_mat_b1 = r2_mat * basis1;

            lhs = Matrix5::zeros();
            let lhs00 = ii1.quadform3x2(&r1_mat_b1).add_diagonal(im1)
                + ii2.quadform3x2(&r2_mat_b1).add_diagonal(im2);
            let lhs10 = ii1 * r1_mat_b1 + ii2 * r2_mat_b1;
            let lhs11 = (ii1 + ii2).into_matrix();
            lhs.fixed_slice_mut::<U2, U2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(2, 2).copy_from(&lhs11);
        }

        #[cfg(feature = "dim2")]
        {
            let b1r1 = basis1.dot(&r1_mat);
            let b2r2 = basis1.dot(&r2_mat);
            let m11 = im1 + im2 + b1r1 * ii1 * b1r1 + b2r2 * ii2 * b2r2;
            let m12 = basis1.dot(&r1_mat) * ii1 + basis1.dot(&r2_mat) * ii2;
            let m22 = ii1 + ii2;
            lhs = SdpMatrix2::new(m11, m12, m22);
        }

        let anchor_linvel1 = rb1.linvel + rb1.angvel.gcross(r1);
        let anchor_linvel2 = rb2.linvel + rb2.angvel.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let linvel_err = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let angvel_err = rb2.angvel - rb1.angvel;

        #[cfg(feature = "dim2")]
        let mut rhs = Vector2::new(linvel_err.x, angvel_err) * params.velocity_solve_fraction;
        #[cfg(feature = "dim3")]
        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            angvel_err.x,
            angvel_err.y,
            angvel_err.z,
        ) * params.velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let linear_err = basis1.tr_mul(&(anchor2 - anchor1));

            let frame1 = rb1.position * joint.local_frame1();
            let frame2 = rb2.position * joint.local_frame2();
            let ang_err = frame2.rotation * frame1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                rhs += Vector2::new(linear_err.x, ang_err.angle()) * velocity_based_erp_inv_dt;
            }
            #[cfg(feature = "dim3")]
            {
                let ang_err = ang_err.scaled_axis();
                rhs += Vector5::new(linear_err.x, linear_err.y, ang_err.x, ang_err.y, ang_err.z)
                    * velocity_based_erp_inv_dt;
            }
        }

        /*
         * Setup motor.
         */
        let mut motor_rhs = 0.0;
        let mut motor_inv_lhs = 0.0;

        let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
            params.dt,
            joint.motor_stiffness,
            joint.motor_damping,
        );

        if stiffness != 0.0 {
            let dist = anchor2.coords.dot(&axis2) - anchor1.coords.dot(&axis1);
            motor_rhs += (dist - joint.motor_target_pos) * stiffness;
        }

        if damping != 0.0 {
            let curr_vel = rb2.linvel.dot(&axis2) - rb1.linvel.dot(&axis1);
            motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
        }

        if stiffness != 0.0 || damping != 0.0 {
            motor_inv_lhs = if keep_lhs { gamma / (im1 + im2) } else { gamma };
            motor_rhs /= gamma;
        }

        let motor_impulse = na::clamp(
            joint.motor_impulse,
            -joint.motor_max_impulse,
            joint.motor_max_impulse,
        );

        // Setup limit constraint.
        let mut limits_active = false;
        let limits_forcedir2 = axis2.into_inner(); // hopefully axis1 is colinear with axis2
        let mut limits_rhs = 0.0;
        let mut limits_impulse = 0.0;
        let mut limits_inv_lhs = 0.0;
        let mut limits_impulse_limits = (0.0, 0.0);

        if joint.limits_enabled {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // TODO: we should allow predictive constraint activation.

            let (min_limit, max_limit) = (joint.limits[0], joint.limits[1]);
            let min_enabled = dist < min_limit;
            let max_enabled = max_limit < dist;

            limits_impulse_limits.0 = if max_enabled { -Real::INFINITY } else { 0.0 };
            limits_impulse_limits.1 = if min_enabled { Real::INFINITY } else { 0.0 };

            limits_active = min_enabled || max_enabled;
            if limits_active {
                limits_rhs = (anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1))
                    * params.velocity_solve_fraction;

                limits_rhs += ((dist - max_limit).max(0.0) - (min_limit - dist).max(0.0))
                    * velocity_based_erp_inv_dt;

                let gcross1 = r1.gcross(*axis1);
                let gcross2 = r2.gcross(*axis2);
                limits_inv_lhs = crate::utils::inv(
                    im1 + im2
                        + gcross1.gdot(ii1.transform_vector(gcross1))
                        + gcross2.gdot(ii2.transform_vector(gcross2)),
                );

                limits_impulse = joint
                    .limits_impulse
                    .max(limits_impulse_limits.0)
                    .min(limits_impulse_limits.1);
            }
        }

        PrismaticVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            ii1_sqrt: rb1.effective_world_inv_inertia_sqrt,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse: joint.impulse * params.warmstart_coeff,
            limits_active,
            limits_impulse: limits_impulse * params.warmstart_coeff,
            limits_forcedir2,
            limits_rhs,
            limits_inv_lhs,
            limits_impulse_limits,
            motor_rhs,
            motor_inv_lhs,
            motor_impulse,
            motor_axis1: *axis1,
            motor_axis2: *axis2,
            motor_max_impulse: joint.motor_max_impulse,
            basis1,
            inv_lhs,
            rhs,
            r1,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        // Warmstart motors.
        mj_lambda1.linear += self.motor_axis1 * (self.im1 * self.motor_impulse);
        mj_lambda2.linear -= self.motor_axis2 * (self.im2 * self.motor_impulse);

        // Warmstart limits.
        if self.limits_active {
            let limit_impulse1 = -self.limits_forcedir2 * self.limits_impulse;
            let limit_impulse2 = self.limits_forcedir2 * self.limits_impulse;
            mj_lambda1.linear += self.im1 * limit_impulse1;
            mj_lambda1.angular += self
                .ii1_sqrt
                .transform_vector(self.r1.gcross(limit_impulse1));
            mj_lambda2.linear += self.im2 * limit_impulse2;
            mj_lambda2.angular += self
                .ii2_sqrt
                .transform_vector(self.r2.gcross(limit_impulse2));
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_dofs(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_vel1 = mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let lin_vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let lin_dvel = self.basis1.tr_mul(&(lin_vel2 - lin_vel1));
        let ang_dvel = ang_vel2 - ang_vel1;
        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_dvel.x, ang_dvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, ang_dvel.x, ang_dvel.y, ang_dvel.z) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));
    }

    fn solve_limits(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            let limits_forcedir1 = -self.limits_forcedir2;
            let limits_forcedir2 = self.limits_forcedir2;

            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = limits_forcedir2.dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + limits_forcedir1.dot(&(mj_lambda1.linear + ang_vel1.gcross(self.r1)))
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - lin_dvel * self.limits_inv_lhs)
                .max(self.limits_impulse_limits.0)
                .min(self.limits_impulse_limits.1);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            let lin_impulse1 = limits_forcedir1 * dimpulse;
            let lin_impulse2 = limits_forcedir2 * dimpulse;

            mj_lambda1.linear += self.im1 * lin_impulse1;
            mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.r1.gcross(lin_impulse1));
            mj_lambda2.linear += self.im2 * lin_impulse2;
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(lin_impulse2));
        }
    }

    fn solve_motors(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if self.motor_inv_lhs != 0.0 {
            let lin_dvel = self.motor_axis2.dot(&mj_lambda2.linear)
                - self.motor_axis1.dot(&mj_lambda1.linear)
                + self.motor_rhs;
            let new_impulse = na::clamp(
                self.motor_impulse + lin_dvel * self.motor_inv_lhs,
                -self.motor_max_impulse,
                self.motor_max_impulse,
            );
            let dimpulse = new_impulse - self.motor_impulse;
            self.motor_impulse = new_impulse;

            mj_lambda1.linear += self.motor_axis1 * (self.im1 * dimpulse);
            mj_lambda2.linear -= self.motor_axis2 * (self.im2 * dimpulse);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        self.solve_limits(&mut mj_lambda1, &mut mj_lambda2);
        self.solve_motors(&mut mj_lambda1, &mut mj_lambda2);
        self.solve_dofs(&mut mj_lambda1, &mut mj_lambda2);

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::PrismaticJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            revolute.motor_impulse = self.motor_impulse;
            revolute.limits_impulse = self.limits_impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct PrismaticVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    r2: Vector<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<Real>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<Real>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<Real>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<Real>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<Real>,

    limits_active: bool,
    limits_forcedir2: Vector<Real>,
    limits_impulse: Real,
    limits_rhs: Real,
    /// min/max applied impulse due to limits
    limits_impulse_limits: (Real, Real),

    axis2: Vector<Real>,
    motor_impulse: Real,
    motor_rhs: Real,
    motor_inv_lhs: Real,
    motor_max_impulse: Real,

    #[cfg(feature = "dim2")]
    basis1: Vector2<Real>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<Real>,

    im2: Real,
    ii2_sqrt: AngularInertia<Real>,
}

impl PrismaticVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &PrismaticJoint,
        flipped: bool,
    ) -> Self {
        let anchor2;
        let anchor1;
        let axis2;
        let axis1;
        let basis1;

        if flipped {
            anchor2 = rb2.position * joint.local_anchor1;
            anchor1 = rb1.position * joint.local_anchor2;
            axis2 = rb2.position * joint.local_axis1;
            axis1 = rb1.position * joint.local_axis2;
            #[cfg(feature = "dim2")]
            {
                basis1 = rb1.position * joint.basis2[0];
            }
            #[cfg(feature = "dim3")]
            {
                basis1 = Matrix3x2::from_columns(&[
                    rb1.position * joint.basis2[0],
                    rb1.position * joint.basis2[1],
                ]);
            }
        } else {
            anchor2 = rb2.position * joint.local_anchor2;
            anchor1 = rb1.position * joint.local_anchor1;
            axis2 = rb2.position * joint.local_axis2;
            axis1 = rb1.position * joint.local_axis1;
            #[cfg(feature = "dim2")]
            {
                basis1 = rb1.position * joint.basis1[0];
            }
            #[cfg(feature = "dim3")]
            {
                basis1 = Matrix3x2::from_columns(&[
                    rb1.position * joint.basis1[0],
                    rb1.position * joint.basis1[1],
                ]);
            }
        };

        // #[cfg(feature = "dim2")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .to_rotation_matrix()
        //     .into_inner();
        // #[cfg(feature = "dim3")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .unwrap_or_else(Rotation::identity)
        //     .to_rotation_matrix()
        //     .into_inner();
        // let basis2 = r21 * basis1;
        // NOTE: we use basis2 := basis1 for now is that allows
        // simplifications of the computation without introducing
        // much instabilities.

        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r2 = anchor2 - rb2.world_com;
        let r2_mat = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let r2_mat_b1 = r2_mat * basis1;

            lhs = Matrix5::zeros();
            let lhs00 = ii2.quadform3x2(&r2_mat_b1).add_diagonal(im2);
            let lhs10 = ii2 * r2_mat_b1;
            let lhs11 = ii2.into_matrix();
            lhs.fixed_slice_mut::<U2, U2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(2, 2).copy_from(&lhs11);
        }

        #[cfg(feature = "dim2")]
        {
            let b2r2 = basis1.dot(&r2_mat);
            let m11 = im2 + b2r2 * ii2 * b2r2;
            let m12 = basis1.dot(&r2_mat) * ii2;
            let m22 = ii2;
            lhs = SdpMatrix2::new(m11, m12, m22);
        }

        let anchor_linvel1 = rb1.linvel + rb1.angvel.gcross(r1);
        let anchor_linvel2 = rb2.linvel + rb2.angvel.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let linvel_err = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let angvel_err = rb2.angvel - rb1.angvel;

        #[cfg(feature = "dim2")]
        let mut rhs = Vector2::new(linvel_err.x, angvel_err) * params.velocity_solve_fraction;
        #[cfg(feature = "dim3")]
        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            angvel_err.x,
            angvel_err.y,
            angvel_err.z,
        ) * params.velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let linear_err = basis1.tr_mul(&(anchor2 - anchor1));

            let (frame1, frame2);
            if flipped {
                frame1 = rb1.position * joint.local_frame2();
                frame2 = rb2.position * joint.local_frame1();
            } else {
                frame1 = rb1.position * joint.local_frame1();
                frame2 = rb2.position * joint.local_frame2();
            }

            let ang_err = frame2.rotation * frame1.rotation.inverse();
            #[cfg(feature = "dim2")]
            {
                rhs += Vector2::new(linear_err.x, ang_err.angle()) * velocity_based_erp_inv_dt;
            }
            #[cfg(feature = "dim3")]
            {
                let ang_err = ang_err.scaled_axis();
                rhs += Vector5::new(linear_err.x, linear_err.y, ang_err.x, ang_err.y, ang_err.z)
                    * velocity_based_erp_inv_dt;
            }
        }

        /*
         * Setup motor.
         */
        let mut motor_rhs = 0.0;
        let mut motor_inv_lhs = 0.0;

        let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
            params.dt,
            joint.motor_stiffness,
            joint.motor_damping,
        );

        if stiffness != 0.0 {
            let dist = anchor2.coords.dot(&axis2) - anchor1.coords.dot(&axis1);
            motor_rhs += (dist - joint.motor_target_pos) * stiffness;
        }

        if damping != 0.0 {
            let curr_vel = rb2.linvel.dot(&axis2) - rb1.linvel.dot(&axis1);
            motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
        }

        if stiffness != 0.0 || damping != 0.0 {
            motor_inv_lhs = if keep_lhs { gamma / im2 } else { gamma };
            motor_rhs /= gamma;
        }

        let motor_impulse = na::clamp(
            joint.motor_impulse,
            -joint.motor_max_impulse,
            joint.motor_max_impulse,
        );

        /*
         * Setup limit constraint.
         */
        let mut limits_active = false;
        let limits_forcedir2 = axis2.into_inner();
        let mut limits_rhs = 0.0;
        let mut limits_impulse = 0.0;
        let mut limits_impulse_limits = (0.0, 0.0);

        if joint.limits_enabled {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // TODO: we should allow predictive constraint activation.

            let (min_limit, max_limit) = (joint.limits[0], joint.limits[1]);
            let min_enabled = dist < min_limit;
            let max_enabled = max_limit < dist;

            limits_impulse_limits.0 = if max_enabled { -Real::INFINITY } else { 0.0 };
            limits_impulse_limits.1 = if min_enabled { Real::INFINITY } else { 0.0 };

            limits_active = min_enabled || max_enabled;
            if limits_active {
                limits_rhs = (anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1))
                    * params.velocity_solve_fraction;

                limits_rhs += ((dist - max_limit).max(0.0) - (min_limit - dist).max(0.0))
                    * velocity_based_erp_inv_dt;

                limits_impulse = joint
                    .limits_impulse
                    .max(limits_impulse_limits.0)
                    .min(limits_impulse_limits.1);
            }
        }

        PrismaticVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse: joint.impulse * params.warmstart_coeff,
            limits_active,
            limits_forcedir2,
            limits_impulse: limits_impulse * params.warmstart_coeff,
            limits_rhs,
            limits_impulse_limits,
            motor_rhs,
            motor_inv_lhs,
            motor_impulse,
            motor_max_impulse: joint.motor_max_impulse,
            basis1,
            inv_lhs,
            rhs,
            r2,
            axis2: axis2.into_inner(),
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        // Warmstart motors.
        mj_lambda2.linear -= self.axis2 * (self.im2 * self.motor_impulse);

        // Warmstart limits.
        mj_lambda2.linear += self.limits_forcedir2 * (self.im2 * self.limits_impulse);

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_dofs(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let lin_dvel = self.basis1.tr_mul(&lin_vel2);
        let ang_dvel = ang_vel2;
        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_dvel.x, ang_dvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, ang_dvel.x, ang_dvel.y, ang_dvel.z) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));
    }

    fn solve_limits(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = self
                .limits_forcedir2
                .dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - lin_dvel / self.im2)
                .max(self.limits_impulse_limits.0)
                .min(self.limits_impulse_limits.1);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda2.linear += self.limits_forcedir2 * (self.im2 * dimpulse);
        }
    }

    fn solve_motors(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if self.motor_inv_lhs != 0.0 {
            let lin_dvel = self.axis2.dot(&mj_lambda2.linear) + self.motor_rhs;
            let new_impulse = na::clamp(
                self.motor_impulse + lin_dvel * self.motor_inv_lhs,
                -self.motor_max_impulse,
                self.motor_max_impulse,
            );
            let dimpulse = new_impulse - self.motor_impulse;
            self.motor_impulse = new_impulse;

            mj_lambda2.linear -= self.axis2 * (self.im2 * dimpulse);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        self.solve_limits(&mut mj_lambda2);
        self.solve_motors(&mut mj_lambda2);
        self.solve_dofs(&mut mj_lambda2);

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // TODO: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::PrismaticJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            revolute.motor_impulse = self.motor_impulse;
            revolute.limits_impulse = self.limits_impulse;
        }
    }
}
