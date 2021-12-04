use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity, SpringJoint,
};
use crate::math::{AngularInertia, Real, Vector};
use crate::utils::{WAngularInertia, WCross};

#[derive(Debug)]
pub(crate) struct SpringVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1: Vector<Real>,
    r2: Vector<Real>,
    vel1: Vector<Real>,
    vel2: Vector<Real>,
    u: Vector<Real>,
    dx: Real,

    impulse: Real,

    gamma: Real,
    bias: Real,
    inv_lhs: Real,

    limits_active: bool,
    limits_min_length: Real,
    limits_max_length: Real,
    limits_inv_lhs: Real,
    limits_lower_rhs: Real,
    limits_upper_rhs: Real,
    limits_lower_impulse: Real,
    limits_upper_impulse: Real,

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl SpringVelocityConstraint {
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
        joint: &SpringJoint,
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
        let ii1 = rb_mprops1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let cr1u = anchor1.gcross(u);
        let cr2u = anchor2.gcross(u);

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im1 + im2 + ii1 * cr1u * cr1u + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i1: Real = (cr1u.transpose() * ii1.into_matrix() * cr1u)[0];
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im1 + im2 + inv_i1 + inv_i2;
        }

        let dx = current_length - joint.rest_length;

        let gamma = crate::utils::inv(params.dt * (joint.damping + params.dt * joint.stiffness));
        let bias = dx * params.dt * joint.stiffness * gamma;
        let inv_lhs = crate::utils::inv(lhs + gamma);

        let limits_active = joint.limits_enabled;
        let limits_min_length = joint.limits_min_length;
        let limits_max_length = joint.limits_max_length;
        let mut limits_inv_lhs = 0.0;
        let mut limits_lower_rhs = 0.0;
        let mut limits_upper_rhs = 0.0;
        let mut limits_lower_impulse = 0.0;
        let mut limits_upper_impulse = 0.0;

        if joint.limits_enabled {
            limits_inv_lhs = crate::utils::inv(lhs);

            if limits_min_length < limits_max_length {
                let inv_dt = crate::utils::inv(params.dt);
                limits_lower_rhs = (current_length - limits_min_length).max(0.0) * inv_dt;
                limits_upper_rhs = (limits_max_length - current_length).max(0.0) * inv_dt;
            }

            limits_lower_impulse = joint.limits_lower_impulse * params.warmstart_coeff;
            limits_upper_impulse = joint.limits_upper_impulse * params.warmstart_coeff;
        }

        Self {
            joint_id,
            mj_lambda1: rb_ids1.active_set_offset,
            mj_lambda2: rb_ids2.active_set_offset,
            im1,
            im2,
            impulse: joint.impulse * params.warmstart_coeff,
            u,
            dx,
            vel1,
            vel2,
            r1: anchor1,
            r2: anchor2,
            gamma,
            bias,
            inv_lhs,
            limits_active,
            limits_min_length,
            limits_max_length,
            limits_inv_lhs,
            limits_lower_rhs,
            limits_upper_rhs,
            limits_lower_impulse,
            limits_upper_impulse,
            ii1_sqrt: rb_mprops1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: rb_mprops2.effective_world_inv_inertia_sqrt,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let impulse = self.impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));
        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_spring(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active && self.limits_min_length >= self.limits_max_length {
            return;
        }
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let delta_impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);

        self.impulse += delta_impulse;
        let impulse = delta_impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_lower_limit(
        &mut self,
        mj_lambda1: &mut DeltaVel<Real>,
        mj_lambda2: &mut DeltaVel<Real>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let lower_impulse = -self.limits_inv_lhs * (dvel + self.limits_lower_rhs);
        let old_impulse = self.limits_lower_impulse;
        self.limits_lower_impulse = (old_impulse + lower_impulse).max(0.0);
        let lower_impulse = self.limits_lower_impulse - old_impulse;
        let impulse = lower_impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_upper_limit(
        &mut self,
        mj_lambda1: &mut DeltaVel<Real>,
        mj_lambda2: &mut DeltaVel<Real>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel1 - vel2).dot(&self.u);

        let upper_impulse = -self.limits_inv_lhs * (dvel + self.limits_upper_rhs);
        let old_impulse = self.limits_upper_impulse;
        self.limits_upper_impulse = (old_impulse + upper_impulse).max(0.0);
        let upper_impulse = self.limits_upper_impulse - old_impulse;
        let impulse = -upper_impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_equal_limits(
        &mut self,
        mj_lambda1: &mut DeltaVel<Real>,
        mj_lambda2: &mut DeltaVel<Real>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let delta_impulse = -self.limits_inv_lhs * dvel;
        self.impulse += delta_impulse;

        let impulse = delta_impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(&mut self, mj_lambda1: &mut DeltaVel<Real>, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            if self.limits_min_length < self.limits_max_length {
                self.solve_lower_limit(mj_lambda1, mj_lambda2);
                self.solve_upper_limit(mj_lambda1, mj_lambda2);
            } else {
                self.solve_equal_limits(mj_lambda1, mj_lambda2);
            }
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        if !self.limits_active || self.limits_min_length < self.limits_max_length {
            self.solve_spring(&mut mj_lambda1, &mut mj_lambda2);
        }
        if self.limits_active {
            self.solve_limits(&mut mj_lambda1, &mut mj_lambda2);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::SpringJoint(dist) = &mut joint.params {
            dist.impulse = self.impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct SpringVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1: Vector<Real>,
    r2: Vector<Real>,
    vel1: Vector<Real>,
    vel2: Vector<Real>,
    u: Vector<Real>,
    dx: Real,

    impulse: Real,

    gamma: Real,
    bias: Real,
    inv_lhs: Real,

    limits_active: bool,
    limits_min_length: Real,
    limits_max_length: Real,
    limits_inv_lhs: Real,
    limits_lower_rhs: Real,
    limits_upper_rhs: Real,
    limits_lower_impulse: Real,
    limits_upper_impulse: Real,

    im2: Real,

    ii2_sqrt: AngularInertia<Real>,
}

impl SpringVelocityGroundConstraint {
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
        joint: &SpringJoint,
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

        let vel1 = rb_vels1.linvel + rb_vels1.angvel.gcross(anchor1);
        let vel2 = rb_vels2.linvel + rb_vels2.angvel.gcross(anchor2);

        let im2 = rb_mprops2.effective_inv_mass;
        let ii2 = rb_mprops2.effective_world_inv_inertia_sqrt.squared();

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let cr2u = anchor2.gcross(u);

        let dx = current_length - joint.rest_length;

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im2 + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im2 + inv_i2;
        }

        let gamma = crate::utils::inv(params.dt * (joint.damping + params.dt * joint.stiffness));
        let bias = dx * params.dt * joint.stiffness * gamma;
        let inv_lhs = crate::utils::inv(lhs + gamma);

        let limits_active = joint.limits_enabled;
        let limits_min_length = joint.limits_min_length;
        let limits_max_length = joint.limits_max_length;
        let mut limits_inv_lhs = 0.0;
        let mut limits_lower_rhs = 0.0;
        let mut limits_upper_rhs = 0.0;
        let mut limits_lower_impulse = 0.0;
        let mut limits_upper_impulse = 0.0;

        if joint.limits_enabled {
            limits_inv_lhs = crate::utils::inv(lhs);

            if limits_min_length < limits_max_length {
                let inv_dt = crate::utils::inv(params.dt);
                limits_lower_rhs = (current_length - limits_min_length).max(0.0) * inv_dt;
                limits_upper_rhs = (limits_max_length - current_length).max(0.0) * inv_dt;
            }

            limits_lower_impulse = joint.limits_lower_impulse * params.warmstart_coeff;
            limits_upper_impulse = joint.limits_upper_impulse * params.warmstart_coeff;
        }

        Self {
            joint_id,
            mj_lambda2: rb_ids2.active_set_offset,
            im2,
            impulse: joint.impulse * params.warmstart_coeff,
            u,
            dx,
            vel1,
            vel2,
            r1: anchor1,
            r2: anchor2,
            gamma,
            bias,
            inv_lhs,
            limits_active,
            limits_min_length,
            limits_max_length,
            limits_inv_lhs,
            limits_lower_rhs,
            limits_upper_rhs,
            limits_lower_impulse,
            limits_upper_impulse,
            ii2_sqrt: rb_mprops2.effective_world_inv_inertia_sqrt,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let impulse = self.impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    fn solve_spring(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active && self.limits_min_length >= self.limits_max_length {
            return;
        }
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let delta_impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);

        self.impulse += delta_impulse;
        let impulse = delta_impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_lower_limit(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let lower_impulse = -self.limits_inv_lhs * (dvel + self.limits_lower_rhs);
        let old_impulse = self.limits_lower_impulse;
        self.limits_lower_impulse = (old_impulse + lower_impulse).max(0.0);
        let lower_impulse = self.limits_lower_impulse - old_impulse;
        let impulse = lower_impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_upper_limit(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel1 - vel2).dot(&self.u);

        let upper_impulse = -self.limits_inv_lhs * (dvel + self.limits_upper_rhs);
        let old_impulse = self.limits_upper_impulse;
        self.limits_upper_impulse = (old_impulse + upper_impulse).max(0.0);
        let upper_impulse = self.limits_upper_impulse - old_impulse;
        let impulse = -upper_impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_equal_limits(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let impulse = -self.limits_inv_lhs * dvel;
        self.impulse += impulse;

        let impulse = impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(&mut self, mj_lambda2: &mut DeltaVel<Real>) {
        if self.limits_active {
            if self.limits_min_length < self.limits_max_length {
                self.solve_lower_limit(mj_lambda2);
                self.solve_upper_limit(mj_lambda2);
            } else {
                self.solve_equal_limits(mj_lambda2);
            }
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        if !self.limits_active || self.limits_min_length < self.limits_max_length {
            self.solve_spring(&mut mj_lambda2);
        }
        if self.limits_active {
            self.solve_limits(&mut mj_lambda2);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::SpringJoint(dist) = &mut joint.params {
            dist.impulse = self.impulse;
        }
    }
}
