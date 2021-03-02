use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    BallJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
};
use crate::math::{AngVector, AngularInertia, Real, SdpMatrix, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};

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

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl BallVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &BallJoint,
    ) -> Self {
        let anchor_world1 = rb1.position * joint.local_anchor1;
        let anchor_world2 = rb2.position * joint.local_anchor2;
        let anchor1 = anchor_world1 - rb1.world_com;
        let anchor2 = anchor_world2 - rb2.world_com;

        let vel1 = rb1.linvel + rb1.angvel.gcross(anchor1);
        let vel2 = rb2.linvel + rb2.angvel.gcross(anchor2);
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;

        let rhs = (vel2 - vel1) * params.velocity_solve_fraction
            + (anchor_world2 - anchor_world1) * params.velocity_based_erp_inv_dt();

        let lhs;
        let cmat1 = anchor1.gcross_matrix();
        let cmat2 = anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        {
            lhs = rb2
                .effective_world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2)
                + rb1
                    .effective_world_inv_inertia_sqrt
                    .squared()
                    .quadform(&cmat1)
                    .add_diagonal(im1);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
            let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
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
                let dpos = rb2.position.rotation
                    * (rb1.position.rotation * joint.motor_target_pos).inverse();
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
                let curr_vel = rb2.angvel - rb1.angvel;
                motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
            }

            #[cfg(feature = "dim2")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
                    let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
                    Some(gamma / (ii1 + ii2))
                } else {
                    Some(gamma)
                };
                motor_rhs /= gamma;
            }

            #[cfg(feature = "dim3")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
                    let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
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

        BallVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
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
            ii1_sqrt: rb1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
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

    im2: Real,
    ii2_sqrt: AngularInertia<Real>,
}

impl BallVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &BallJoint,
        flipped: bool,
    ) -> Self {
        let (anchor_world1, anchor_world2) = if flipped {
            (
                rb1.position * joint.local_anchor2,
                rb2.position * joint.local_anchor1,
            )
        } else {
            (
                rb1.position * joint.local_anchor1,
                rb2.position * joint.local_anchor2,
            )
        };

        let anchor1 = anchor_world1 - rb1.world_com;
        let anchor2 = anchor_world2 - rb2.world_com;

        let im2 = rb2.effective_inv_mass;
        let vel1 = rb1.linvel + rb1.angvel.gcross(anchor1);
        let vel2 = rb2.linvel + rb2.angvel.gcross(anchor2);

        let rhs = (vel2 - vel1) * params.velocity_solve_fraction
            + (anchor_world2 - anchor_world1) * params.velocity_based_erp_inv_dt();

        let cmat2 = anchor2.gcross_matrix();

        let lhs;

        #[cfg(feature = "dim3")]
        {
            lhs = rb2
                .effective_world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2);
        }

        #[cfg(feature = "dim2")]
        {
            let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
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
                let dpos = rb2.position.rotation
                    * (rb1.position.rotation * joint.motor_target_pos).inverse();
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
                let curr_vel = rb2.angvel - rb1.angvel;
                motor_rhs += (curr_vel - joint.motor_target_vel) * damping;
            }

            #[cfg(feature = "dim2")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
                    Some(gamma / ii2)
                } else {
                    Some(gamma)
                };
                motor_rhs /= gamma;
            }

            #[cfg(feature = "dim3")]
            if stiffness != 0.0 || damping != 0.0 {
                motor_inv_lhs = if keep_lhs {
                    let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
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

        BallVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            impulse: joint.impulse * params.warmstart_coeff,
            r2: anchor2,
            rhs,
            inv_lhs,
            motor_rhs,
            motor_impulse,
            motor_inv_lhs,
            motor_max_impulse: joint.motor_max_impulse,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        mj_lambda2.linear -= self.im2 * self.impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(self.r2.gcross(self.impulse) + self.motor_impulse);
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
        }
    }
}
