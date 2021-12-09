use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity, SpringJoint,
};
use crate::math::{
    AngVector, AngularInertia, Isometry, Point, Real, SimdBool, SimdReal, Vector, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross};
use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

#[derive(Debug)]
pub(crate) struct WSpringVelocityConstraint {
    mj_lambda1: [usize; SIMD_WIDTH],
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,
    vel1: Vector<SimdReal>,
    vel2: Vector<SimdReal>,
    u: Vector<SimdReal>,
    dx: SimdReal,

    impulse: SimdReal,

    gamma: SimdReal,
    bias: SimdReal,
    inv_lhs: SimdReal,

    limits_active: bool,
    limits_min_length: SimdReal,
    limits_max_length: SimdReal,
    limits_inv_lhs: SimdReal,
    limits_lower_rhs: SimdReal,
    limits_upper_rhs: SimdReal,
    limits_lower_impulse: SimdReal,
    limits_upper_impulse: SimdReal,

    im1: SimdReal,
    im2: SimdReal,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WSpringVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&SpringJoint; SIMD_WIDTH],
    ) -> Self {
        let (poss1, vels1, mprops1, ids1) = rbs1;
        let (poss2, vels2, mprops2, ids2) = rbs2;

        let position1 = Isometry::from(gather![|ii| poss1[ii].position]);
        let linvel1 = Vector::from(gather![|ii| vels1[ii].linvel]);
        let angvel1 = AngVector::<SimdReal>::from(gather![|ii| vels1[ii].angvel]);
        let world_com1 = Point::from(gather![|ii| mprops1[ii].world_com]);
        let im1 = SimdReal::from(gather![|ii| mprops1[ii].effective_inv_mass]);
        let ii1_sqrt = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops1[ii].effective_world_inv_inertia_sqrt
        ]);
        let ii1 = ii1_sqrt.squared();
        let mj_lambda1 = gather![|ii| ids1[ii].active_set_offset];

        let position2 = Isometry::from(gather![|ii| poss2[ii].position]);
        let linvel2 = Vector::from(gather![|ii| vels2[ii].linvel]);
        let angvel2 = AngVector::<SimdReal>::from(gather![|ii| vels2[ii].angvel]);
        let world_com2 = Point::from(gather![|ii| mprops2[ii].world_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ]);
        let ii2 = ii2_sqrt.squared();
        let mj_lambda2 = gather![|ii| ids2[ii].active_set_offset];

        let rest_length = SimdReal::from(gather![|ii| cparams[ii].rest_length]);
        let stiffness = SimdReal::from(gather![|ii| cparams[ii].stiffness]);
        let damping = SimdReal::from(gather![|ii| cparams[ii].damping]);

        let local_anchor1 = Point::from(gather![|ii| cparams[ii].local_anchor1]);
        let local_anchor2 = Point::from(gather![|ii| cparams[ii].local_anchor2]);
        let impulse = SimdReal::from(gather![|ii| cparams[ii].impulse]);
        let dt = SimdReal::splat(params.dt);
        let warmstart_coeff = SimdReal::splat(params.warmstart_coeff);

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let cr1u = anchor1.gcross(u);
        let cr2u = anchor2.gcross(u);

        let lhs: SimdReal;
        #[cfg(feature = "dim2")]
        {
            lhs = im1 + im2 + ii1 * cr1u * cr1u + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i1: SimdReal = (cr1u.transpose() * ii1.into_matrix() * cr1u)[0];
            let inv_i2: SimdReal = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im1 + im2 + inv_i1 + inv_i2;
        }

        let dx = current_length - rest_length;

        let gamma = crate::utils::simd_inv(dt * (damping + dt * stiffness));
        let bias = dx * dt * stiffness * gamma;
        let inv_lhs = crate::utils::simd_inv(lhs + gamma);

        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        let mut limits_active = false;
        let zero: SimdReal = na::zero();
        let mut limits_inv_lhs = zero;
        let mut limits_lower_rhs = zero;
        let mut limits_upper_rhs = zero;
        let mut limits_lower_impulse = zero;
        let mut limits_upper_impulse = zero;
        let limits_min_length = SimdReal::from(gather![|ii| cparams[ii].limits_min_length]);
        let limits_max_length = SimdReal::from(gather![|ii| cparams[ii].limits_max_length]);

        if limits_enabled.any() {
            limits_inv_lhs = crate::utils::simd_inv(lhs);

            limits_active = limits_min_length.simd_lt(limits_max_length).any();
            if limits_active {
                let inv_dt = crate::utils::simd_inv(dt);
                limits_lower_rhs = (current_length - limits_min_length).simd_max(zero) * inv_dt;
                limits_upper_rhs = (limits_max_length - current_length).simd_max(zero) * inv_dt;
            }

            let prev_lower_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_lower_impulse]);
            let prev_upper_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_upper_impulse]);

            limits_lower_impulse = prev_lower_impulse * warmstart_coeff;
            limits_upper_impulse = prev_upper_impulse * warmstart_coeff;
        }

        WSpringVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            im2,
            impulse: impulse * warmstart_coeff,
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
            dx,
            u,
            vel1,
            vel2,
            ii1_sqrt,
            ii2_sqrt,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular
            ]),
        };
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let impulse: Vector<SimdReal> =
            self.u * (self.impulse + self.limits_lower_impulse + self.limits_upper_impulse);

        mj_lambda1.linear -= impulse * self.im1;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));
        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda1[ii] as usize].linear = mj_lambda1.linear.extract(ii);
            mj_lambdas[self.mj_lambda1[ii] as usize].angular = mj_lambda1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    fn solve_spring(
        &mut self, 
        mj_lambda1: &mut DeltaVel<SimdReal>, 
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);
        let delta_impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);

        self.impulse += delta_impulse;
        let impulse = self.u * delta_impulse;

        mj_lambda1.linear -= impulse * self.im1;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_lower_limit(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let lower_impulse = -self.limits_inv_lhs * (dvel + self.limits_lower_rhs);
        let old_impulse = self.limits_lower_impulse;
        self.limits_lower_impulse = (old_impulse + lower_impulse).simd_max(SimdReal::splat(0.0));
        let lower_impulse = self.limits_lower_impulse - old_impulse;
        let impulse = self.u * lower_impulse;

        mj_lambda1.linear -= impulse * self.im1;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_upper_limit(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel1 - vel2).dot(&self.u);

        let upper_impulse = -self.limits_inv_lhs * (dvel + self.limits_upper_rhs);
        let old_impulse = self.limits_upper_impulse;
        self.limits_upper_impulse = (old_impulse + upper_impulse).simd_max(SimdReal::splat(0.0));
        let upper_impulse = self.limits_upper_impulse - old_impulse;
        let impulse = self.u * -upper_impulse;

        mj_lambda1.linear -= impulse * self.im1;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_equal_limits(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let delta_impulse = -self.limits_inv_lhs * dvel;
        self.impulse += delta_impulse;

        let impulse = self.u * delta_impulse;

        mj_lambda1.linear -= impulse * self.im1;
        mj_lambda1.angular -= self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        if self.limits_min_length.simd_lt(self.limits_max_length).any() {
            self.solve_lower_limit(mj_lambda1, mj_lambda2);
            self.solve_upper_limit(mj_lambda1, mj_lambda2);
        } else {
            self.solve_equal_limits(mj_lambda1, mj_lambda2);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1: DeltaVel<SimdReal> = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular
            ]),
        };
        let mut mj_lambda2: DeltaVel<SimdReal> = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        self.solve_spring(&mut mj_lambda1, &mut mj_lambda2);
        if self.limits_active {
            self.solve_limits(&mut mj_lambda1, &mut mj_lambda2);
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda1[ii] as usize].linear = mj_lambda1.linear.extract(ii);
            mj_lambdas[self.mj_lambda1[ii] as usize].angular = mj_lambda1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::SpringJoint(spring) = &mut joint.params {
                spring.impulse = self.impulse.extract(ii)
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct WSpringVelocityGroundConstraint {
    mj_lambda2: [usize; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,
    vel1: Vector<SimdReal>,
    vel2: Vector<SimdReal>,
    u: Vector<SimdReal>,
    dx: SimdReal,

    impulse: SimdReal,

    gamma: SimdReal,
    bias: SimdReal,
    inv_lhs: SimdReal,

    limits_active: bool,
    limits_min_length: SimdReal,
    limits_max_length: SimdReal,
    limits_inv_lhs: SimdReal,
    limits_lower_rhs: SimdReal,
    limits_upper_rhs: SimdReal,
    limits_lower_impulse: SimdReal,
    limits_upper_impulse: SimdReal,

    im2: SimdReal,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WSpringVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&SpringJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let (poss1, vels1, mprops1) = rbs1;
        let (poss2, vels2, mprops2, ids2) = rbs2;

        let position1 = Isometry::from(gather![|ii| poss1[ii].position]);
        let linvel1 = Vector::from(gather![|ii| vels1[ii].linvel]);
        let angvel1 = AngVector::<SimdReal>::from(gather![|ii| vels1[ii].angvel]);
        let world_com1 = Point::from(gather![|ii| mprops1[ii].world_com]);
        let local_anchor1 = Point::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_anchor2
        } else {
            cparams[ii].local_anchor1
        }]);

        let position2 = Isometry::from(gather![|ii| poss2[ii].position]);
        let linvel2 = Vector::from(gather![|ii| vels2[ii].linvel]);
        let angvel2 = AngVector::<SimdReal>::from(gather![|ii| vels2[ii].angvel]);
        let world_com2 = Point::from(gather![|ii| mprops2[ii].world_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ]);
        let ii2 = ii2_sqrt.squared();
        let mj_lambda2 = gather![|ii| ids2[ii].active_set_offset];

        let local_anchor2 = Point::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_anchor1
        } else {
            cparams[ii].local_anchor2
        }]);
        let impulse = SimdReal::from(gather![|ii| cparams[ii].impulse]);

        let rest_length = SimdReal::from(gather![|ii| cparams[ii].rest_length]);
        let stiffness = SimdReal::from(gather![|ii| cparams[ii].stiffness]);
        let damping = SimdReal::from(gather![|ii| cparams[ii].damping]);
        let dt = SimdReal::splat(params.dt);
        let warmstart_coeff = SimdReal::splat(params.warmstart_coeff);

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let cr2u = anchor2.gcross(u);

        let lhs: SimdReal;
        #[cfg(feature = "dim2")]
        {
            lhs = im2 + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i2: SimdReal = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im2 + inv_i2;
        }

        let dx = current_length - rest_length;

        let gamma = crate::utils::simd_inv(dt * (damping + dt * stiffness));
        let bias = dx * dt * stiffness * gamma;
        let inv_lhs = crate::utils::simd_inv(lhs + gamma);

        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        let mut limits_active = false;
        let zero: SimdReal = na::zero();
        let mut limits_inv_lhs = zero;
        let mut limits_lower_rhs = zero;
        let mut limits_upper_rhs = zero;
        let mut limits_lower_impulse = zero;
        let mut limits_upper_impulse = zero;
        let limits_min_length = SimdReal::from(gather![|ii| cparams[ii].limits_min_length]);
        let limits_max_length = SimdReal::from(gather![|ii| cparams[ii].limits_max_length]);

        if limits_enabled.any() {
            limits_inv_lhs = crate::utils::simd_inv(lhs);

            limits_active = limits_min_length.simd_lt(limits_max_length).any();
            if limits_active {
                let inv_dt = crate::utils::simd_inv(dt);
                limits_lower_rhs = (current_length - limits_min_length).simd_max(zero) * inv_dt;
                limits_upper_rhs = (limits_max_length - current_length).simd_max(zero) * inv_dt;
            }

            let prev_lower_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_lower_impulse]);
            let prev_upper_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_upper_impulse]);

            limits_lower_impulse = prev_lower_impulse * warmstart_coeff;
            limits_upper_impulse = prev_upper_impulse * warmstart_coeff;
        }

        Self {
            joint_id,
            mj_lambda2,
            im2,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
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
            u,
            dx,
            vel1,
            vel2,
            r1: anchor1,
            r2: anchor2,
            ii2_sqrt,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let impulse: Vector<SimdReal> = self.u * self.impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    fn solve_spring(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>){
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let delta_impulse = -self.inv_lhs * (dvel + self.bias + self.gamma * self.impulse);

        self.impulse += delta_impulse;
        let impulse = self.u * delta_impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_lower_limit(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let lower_impulse = -self.limits_inv_lhs * (dvel + self.limits_lower_rhs);
        let old_impulse = self.limits_lower_impulse;
        self.limits_lower_impulse = (old_impulse + lower_impulse).simd_max(SimdReal::splat(0.0));
        let lower_impulse = self.limits_lower_impulse - old_impulse;
        let impulse = self.u * lower_impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_upper_limit(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel1 - vel2).dot(&self.u);

        let upper_impulse = -self.limits_inv_lhs * (dvel + self.limits_upper_rhs);
        let old_impulse = self.limits_upper_impulse;
        self.limits_upper_impulse = (old_impulse + upper_impulse).simd_max(SimdReal::splat(0.0));
        let upper_impulse = self.limits_upper_impulse - old_impulse;
        let impulse = self.u * -upper_impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_equal_limits(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);

        let impulse = -self.limits_inv_lhs * dvel;
        self.impulse += impulse;

        let impulse = self.u * impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));
    }

    fn solve_limits(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
        if self.limits_min_length.simd_lt(self.limits_max_length).any() {
            self.solve_lower_limit(mj_lambda2);
            self.solve_upper_limit(mj_lambda2);
        } else {
            self.solve_equal_limits(mj_lambda2);
        }
    }


    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2: DeltaVel<SimdReal> = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        self.solve_spring(&mut mj_lambda2);
        if self.limits_active {
            self.solve_limits(&mut mj_lambda2);
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::SpringJoint(spring) = &mut joint.params {
                spring.impulse = self.impulse.extract(ii)
            }
        }
    }
}
