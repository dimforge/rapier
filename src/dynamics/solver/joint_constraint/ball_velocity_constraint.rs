use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    BallJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{AngVector, AngularInertia, Real, Rotation, SdpMatrix, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix, WDot};

#[derive(Debug)]
pub(crate) struct BallVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    rhs: Vector<Real>,
    impulse: Vector<Real>,

    r1: Vector<Real>,
    r2: Vector<Real>,

    inv_lhs: SdpMatrix<Real>,

    motor_rhs: AngVector<Real>,
    motor_impulse: AngVector<Real>,
    motor_inv_lhs: Option<AngularInertia<Real>>,
    motor_max_impulse: Real,

    limits_active: bool,
    limits_rhs: Real,
    limits_inv_lhs: Real,
    limits_impulse: Real,
    limits_axis: AngVector<Real>,

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl BallVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        rb2: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        joint: &BallJoint,
    ) -> Self {
        let (rb_pos1, rb_vels1, rb_mprops1, rb_ids1) = rb1;
        let (rb_pos2, rb_vels2, rb_mprops2, rb_ids2) = rb2;

        let anchor_world1 = rb_pos1.position * joint.local_anchor1;
        let anchor_world2 = rb_pos2.position * joint.local_anchor2;
        let anchor1 = anchor_world1 - rb_mprops1.world_com;
        let anchor2 = anchor_world2 - rb_mprops2.world_com;

        let vel1 = rb_vels1.linvel + rb_vels1.angvel.gcross(anchor1);
        let vel2 = rb_vels2.linvel + rb_vels2.angvel.gcross(anchor2);
        let im1 = rb_mprops1.effective_inv_mass;
        let im2 = rb_mprops2.effective_inv_mass;

        let rhs = (vel2 - vel1) * params.velocity_solve_fraction
            + (anchor_world2 - anchor_world1) * params.velocity_based_erp_inv_dt();

        let lhs;
        let cmat1 = anchor1.gcross_matrix();
        let cmat2 = anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        {
            lhs = rb_mprops2
                .effective_world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2)
                + rb_mprops1
                    .effective_world_inv_inertia_sqrt
                    .squared()
                    .quadform(&cmat1)
                    .add_diagonal(im1);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let ii1 = rb_mprops1.effective_world_inv_inertia_sqrt.squared();
            let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
            let m11 = im1 + im2 + cmat1.x * cmat1.x * ii1 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat1.x * cmat1.y * ii1 + cmat2.x * cmat2.y * ii2;
            let m22 = im1 + im2 + cmat1.y * cmat1.y * ii1 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let inv_lhs = lhs.inverse_unchecked();

        /*
         * Motor part.
         */
        let mut motor_rhs = na::zero();
        let mut motor_inv_lhs = None;
        let motor_max_impulse = joint.motor_max_impulse;

        if motor_max_impulse > 0.0 {
            let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
                params.dt,
                joint.motor_stiffness,
                joint.motor_damping,
            );

            if stiffness != 0.0 {
                let dpos = rb_pos2.position.rotation
                    * (rb_pos1.position.rotation * joint.motor_target_pos).inverse();
                #[cfg(feature = "dim2")]
                {
                    motor_rhs += dpos.angle() * stiffness;
                }
                #[cfg(feature = "dim3")]
                {
                    motor_rhs += dpos.scaled_axis() * stiffness;
                }
            }

            if damping != 0.0 {
                let curr_vel = rb_vels2.angvel - rb_vels1.angvel;
                motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
            }

            #[cfg(feature = "dim2")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii1 = rb_mprops1.effective_world_inv_inertia_sqrt.squared();
                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    Some(gamma / (ii1 + ii2))
                } else {
                    Some(gamma)
                };
                motor_rhs /= gamma;
            }

            #[cfg(feature = "dim3")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii1 = rb_mprops1.effective_world_inv_inertia_sqrt.squared();
                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    Some((ii1 + ii2).inverse_unchecked() * gamma)
                } else {
                    Some(SdpMatrix::diagonal(gamma))
                };
                motor_rhs /= gamma;
            }
        }

        #[cfg(feature = "dim2")]
        let motor_impulse = na::clamp(joint.motor_impulse, -motor_max_impulse, motor_max_impulse)
            * params.warmstart_coeff;
        #[cfg(feature = "dim3")]
        let motor_impulse =
            joint.motor_impulse.cap_magnitude(motor_max_impulse) * params.warmstart_coeff;

        /*
         * Setup the limits constraint.
         */
        let mut limits_active = false;
        let mut limits_rhs = 0.0;
        let mut limits_inv_lhs = 0.0;
        let mut limits_impulse = 0.0;
        let mut limits_axis = na::zero();

        if joint.limits_enabled {
            let axis1 = rb_pos1.position * joint.limits_local_axis1;
            let axis2 = rb_pos2.position * joint.limits_local_axis2;

            #[cfg(feature = "dim2")]
            let axis_angle = Rotation::rotation_between_axis(&axis2, &axis1).axis_angle();
            #[cfg(feature = "dim3")]
            let axis_angle =
                Rotation::rotation_between_axis(&axis2, &axis1).and_then(|r| r.axis_angle());

            // TODO: handle the case where dot(axis1, axis2) = -1.0
            if let Some((axis, angle)) = axis_angle {
                if angle >= joint.limits_angle {
                    #[cfg(feature = "dim2")]
                    let axis = axis[0];
                    #[cfg(feature = "dim3")]
                    let axis = axis.into_inner();

                    limits_active = true;
                    limits_rhs = (rb_vels2.angvel.gdot(axis) - rb_vels1.angvel.gdot(axis))
                        * params.velocity_solve_fraction;

                    limits_rhs += (angle - joint.limits_angle) * params.velocity_based_erp_inv_dt();

                    let ii1 = rb_mprops1.effective_world_inv_inertia_sqrt.squared();
                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    limits_inv_lhs = crate::utils::inv(
                        axis.gdot(ii2.transform_vector(axis))
                            + axis.gdot(ii1.transform_vector(axis)),
                    );

                    limits_impulse = joint.limits_impulse * params.warmstart_coeff;
                    limits_axis = axis;
                }
            }
        }

        BallVelocityConstraint {
            joint_id,
            mj_lambda1: rb_ids1.active_set_offset,
            mj_lambda2: rb_ids2.active_set_offset,
            im1,
            im2,
            impulse: joint.impulse * params.warmstart_coeff,
            r1: anchor1,
            r2: anchor2,
            rhs,
            inv_lhs,
            motor_rhs,
            motor_impulse,
            motor_inv_lhs,
            motor_max_impulse: joint.motor_max_impulse,
            ii1_sqrt: rb_mprops1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: rb_mprops2.effective_world_inv_inertia_sqrt,
            limits_active,
            limits_axis,
            limits_rhs,
            limits_inv_lhs,
            limits_impulse,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        mj_lambda1.linear += self.im1 * self.impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(self.r1.gcross(self.impulse) + self.motor_impulse);
        mj_lambda2.linear -= self.im2 * self.impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(self.r2.gcross(self.impulse) + self.motor_impulse);

        /*
         * Warmstart limits.
         */
        if self.limits_active {
            let limit_impulse1 = -self.limits_axis * self.limits_impulse;
            let limit_impulse2 = self.limits_axis * self.limits_impulse;
            mj_lambda1.angular += self.ii1_sqrt.transform_vector(limit_impulse1);
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(limit_impulse2);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_dofs(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let dvel = -vel1 + vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda1.linear += self.im1 * impulse;
        mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear -= self.im2 * impulse;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            let limits_torquedir1 = -self.limits_axis;
            let limits_torquedir2 = self.limits_axis;

            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let ang_dvel = limits_torquedir1.gdot(ang_vel1)
                + limits_torquedir2.gdot(ang_vel2)
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - ang_dvel * self.limits_inv_lhs).max(0.0);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            let ang_impulse1 = limits_torquedir1 * dimpulse;
            let ang_impulse2 = limits_torquedir2 * dimpulse;

            mj_lambda1.angular += self.ii1_sqrt.transform_vector(ang_impulse1);
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(ang_impulse2);
        }
    }

    fn solve_motors(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if let Some(motor_inv_lhs) = &self.motor_inv_lhs {
            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dangvel = (ang_vel2 - ang_vel1) + self.motor_rhs;

            let new_impulse = self.motor_impulse + motor_inv_lhs.transform_vector(dangvel);

            #[cfg(feature = "dim2")]
            let clamped_impulse =
                na::clamp(new_impulse, -self.motor_max_impulse, self.motor_max_impulse);
            #[cfg(feature = "dim3")]
            let clamped_impulse = new_impulse.cap_magnitude(self.motor_max_impulse);

            let effective_impulse = clamped_impulse - self.motor_impulse;
            self.motor_impulse = clamped_impulse;

            mj_lambda1.angular += self.ii1_sqrt.transform_vector(effective_impulse);
            mj_lambda2.angular -= self.ii2_sqrt.transform_vector(effective_impulse);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        self.solve_limits(&mut mj_lambda1, &mut mj_lambda2);
        self.solve_dofs(&mut mj_lambda1, &mut mj_lambda2);
        self.solve_motors(&mut mj_lambda1, &mut mj_lambda2);

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::BallJoint(ball) = &mut joint.params {
            ball.impulse = self.impulse;
            ball.motor_impulse = self.motor_impulse;
            ball.limits_impulse = self.limits_impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct BallVelocityGroundConstraint {
    mj_lambda2: usize,
    joint_id: JointIndex,
    r2: Vector<Real>,

    rhs: Vector<Real>,
    impulse: Vector<Real>,
    inv_lhs: SdpMatrix<Real>,

    motor_rhs: AngVector<Real>,
    motor_impulse: AngVector<Real>,
    motor_inv_lhs: Option<AngularInertia<Real>>,
    motor_max_impulse: Real,

    limits_active: bool,
    limits_rhs: Real,
    limits_inv_lhs: Real,
    limits_impulse: Real,
    limits_axis: AngVector<Real>,

    im2: Real,
    ii2_sqrt: AngularInertia<Real>,
}

impl BallVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: (&RigidBodyPosition, &RigidBodyVelocity, &RigidBodyMassProps),
        rb2: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        joint: &BallJoint,
        flipped: bool,
    ) -> Self {
        let (rb_pos1, rb_vels1, rb_mprops1) = rb1;
        let (rb_pos2, rb_vels2, rb_mprops2, rb_ids2) = rb2;

        let (anchor_world1, anchor_world2) = if flipped {
            (
                rb_pos1.position * joint.local_anchor2,
                rb_pos2.position * joint.local_anchor1,
            )
        } else {
            (
                rb_pos1.position * joint.local_anchor1,
                rb_pos2.position * joint.local_anchor2,
            )
        };

        let anchor1 = anchor_world1 - rb_mprops1.world_com;
        let anchor2 = anchor_world2 - rb_mprops2.world_com;

        let im2 = rb_mprops2.effective_inv_mass;
        let vel1 = rb_vels1.linvel + rb_vels1.angvel.gcross(anchor1);
        let vel2 = rb_vels2.linvel + rb_vels2.angvel.gcross(anchor2);

        let rhs = (vel2 - vel1) * params.velocity_solve_fraction
            + (anchor_world2 - anchor_world1) * params.velocity_based_erp_inv_dt();

        let cmat2 = anchor2.gcross_matrix();

        let lhs;

        #[cfg(feature = "dim3")]
        {
            lhs = rb_mprops2
                .effective_world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2);
        }

        #[cfg(feature = "dim2")]
        {
            let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
            let m11 = im2 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat2.x * cmat2.y * ii2;
            let m22 = im2 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let inv_lhs = lhs.inverse_unchecked();

        /*
         * Motor part.
         */
        let mut motor_rhs = na::zero();
        let mut motor_inv_lhs = None;
        let motor_max_impulse = joint.motor_max_impulse;

        if motor_max_impulse > 0.0 {
            let (stiffness, damping, gamma, keep_lhs) = joint.motor_model.combine_coefficients(
                params.dt,
                joint.motor_stiffness,
                joint.motor_damping,
            );

            if stiffness != 0.0 {
                let dpos = rb_pos2.position.rotation
                    * (rb_pos1.position.rotation * joint.motor_target_pos).inverse();
                #[cfg(feature = "dim2")]
                {
                    motor_rhs += dpos.angle() * stiffness;
                }
                #[cfg(feature = "dim3")]
                {
                    motor_rhs += dpos.scaled_axis() * stiffness;
                }
            }

            if damping != 0.0 {
                let curr_vel = rb_vels2.angvel - rb_vels1.angvel;
                motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
            }

            #[cfg(feature = "dim2")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    Some(gamma / ii2)
                } else {
                    Some(gamma)
                };
                motor_rhs /= gamma;
            }

            #[cfg(feature = "dim3")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    Some(ii2.inverse_unchecked() * gamma)
                } else {
                    Some(SdpMatrix::diagonal(gamma))
                };
                motor_rhs /= gamma;
            }
        }

        #[cfg(feature = "dim2")]
        let motor_impulse = na::clamp(joint.motor_impulse, -motor_max_impulse, motor_max_impulse)
            * params.warmstart_coeff;
        #[cfg(feature = "dim3")]
        let motor_impulse =
            joint.motor_impulse.cap_magnitude(motor_max_impulse) * params.warmstart_coeff;

        /*
         * Setup the limits constraint.
         */
        let mut limits_active = false;
        let mut limits_rhs = 0.0;
        let mut limits_inv_lhs = 0.0;
        let mut limits_impulse = 0.0;
        let mut limits_axis = na::zero();

        if joint.limits_enabled {
            let (axis1, axis2) = if flipped {
                (
                    rb_pos1.position * joint.limits_local_axis2,
                    rb_pos2.position * joint.limits_local_axis1,
                )
            } else {
                (
                    rb_pos1.position * joint.limits_local_axis1,
                    rb_pos2.position * joint.limits_local_axis2,
                )
            };

            #[cfg(feature = "dim2")]
            let axis_angle = Rotation::rotation_between_axis(&axis2, &axis1).axis_angle();
            #[cfg(feature = "dim3")]
            let axis_angle =
                Rotation::rotation_between_axis(&axis2, &axis1).and_then(|r| r.axis_angle());

            // TODO: handle the case where dot(axis1, axis2) = -1.0
            if let Some((axis, angle)) = axis_angle {
                if angle >= joint.limits_angle {
                    #[cfg(feature = "dim2")]
                    let axis = axis[0];
                    #[cfg(feature = "dim3")]
                    let axis = axis.into_inner();

                    limits_active = true;
                    limits_rhs = (rb_vels2.angvel.gdot(axis) - rb_vels1.angvel.gdot(axis))
                        * params.velocity_solve_fraction;

                    limits_rhs += (angle - joint.limits_angle) * params.velocity_based_erp_inv_dt();

                    let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();
                    limits_inv_lhs = crate::utils::inv(axis.gdot(ii2.transform_vector(axis)));
                    limits_impulse = joint.limits_impulse * params.warmstart_coeff;
                    limits_axis = axis;
                }
            }
        }

        BallVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb_ids2.active_set_offset,
            im2,
            impulse: joint.impulse * params.warmstart_coeff,
            r2: anchor2,
            rhs,
            inv_lhs,
            motor_rhs,
            motor_impulse,
            motor_inv_lhs,
            motor_max_impulse: joint.motor_max_impulse,
            ii2_sqrt: rb_mprops2.effective_world_inv_inertia_sqrt,
            limits_active,
            limits_axis,
            limits_rhs,
            limits_inv_lhs,
            limits_impulse,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        mj_lambda2.linear -= self.im2 * self.impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(self.r2.gcross(self.impulse) + self.motor_impulse);

        /*
         * Warmstart limits.
         */
        if self.limits_active {
            let limit_impulse2 = self.limits_axis * self.limits_impulse;
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(limit_impulse2);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_dofs(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        let angvel = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel2 = mj_lambda2.linear + angvel.gcross(self.r2);
        let dvel = vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda2.linear -= self.im2 * impulse;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            let limits_torquedir2 = self.limits_axis;
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let ang_dvel = limits_torquedir2.gdot(ang_vel2) + self.limits_rhs;
            let new_impulse = (self.limits_impulse - ang_dvel * self.limits_inv_lhs).max(0.0);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            let ang_impulse2 = limits_torquedir2 * dimpulse;
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(ang_impulse2);
        }
    }

    fn solve_motors(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if let Some(motor_inv_lhs) = &self.motor_inv_lhs {
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dangvel = ang_vel2 + self.motor_rhs;
            let new_impulse = self.motor_impulse + motor_inv_lhs.transform_vector(dangvel);

            #[cfg(feature = "dim2")]
            let clamped_impulse =
                na::clamp(new_impulse, -self.motor_max_impulse, self.motor_max_impulse);
            #[cfg(feature = "dim3")]
            let clamped_impulse = new_impulse.cap_magnitude(self.motor_max_impulse);

            let effective_impulse = clamped_impulse - self.motor_impulse;
            self.motor_impulse = clamped_impulse;

            mj_lambda2.angular -= self.ii2_sqrt.transform_vector(effective_impulse);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        self.solve_limits(&mut mj_lambda2);
        self.solve_dofs(&mut mj_lambda2);
        self.solve_motors(&mut mj_lambda2);

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::BallJoint(ball) = &mut joint.params {
            ball.impulse = self.impulse;
            ball.motor_impulse = self.motor_impulse;
            ball.limits_impulse = self.limits_impulse;
        }
    }
}
