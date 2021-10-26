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
    inv_lhs: Real,
    gamma: Real,
    bias: Real,

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

        let dx = current_length - joint.rest_length;

        let cr1u = anchor1.gcross(u);
        let cr2u = anchor2.gcross(u);

        let gamma = crate::utils::inv(params.dt * (joint.damping + params.dt * joint.stiffness));
        let bias = dx * params.dt * joint.stiffness * gamma;

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im1 + im2 + ii1 * cr1u * cr1u + ii2 * cr2u * cr2u + gamma;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i1: Real = (cr1u.transpose() * ii1.into_matrix() * cr1u)[0];
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im1 + im2 + inv_i1 + inv_i2 + gamma;
        }

        let inv_lhs = crate::utils::inv(lhs);

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
            ii1_sqrt: rb_mprops1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: rb_mprops1.effective_world_inv_inertia_sqrt,
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

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);
        self.impulse += impulse;
        let impulse = impulse * self.u;

        mj_lambda1.linear -= self.im1 * impulse;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
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
    inv_lhs: Real,
    gamma: Real,
    bias: Real,

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

        let dx = current_length - joint.rest_length;

        let cr2u = anchor2.gcross(u);

        let gamma = crate::utils::inv(params.dt * (joint.damping + params.dt * joint.stiffness));
        let bias = dx * params.dt * joint.stiffness * gamma;

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im2 + ii2 * cr2u * cr2u + gamma;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im2 + inv_i2 + gamma;
        }

        let inv_lhs = crate::utils::inv(lhs);

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

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);
        self.impulse += impulse;
        let impulse = impulse * self.u;

        mj_lambda2.linear += self.im2 * impulse;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::SpringJoint(dist) = &mut joint.params {
            dist.impulse = self.impulse;
        }
    }
}
