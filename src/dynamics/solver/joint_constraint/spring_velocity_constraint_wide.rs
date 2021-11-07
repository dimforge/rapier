use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity, SpringJoint,
};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Real, SimdReal, Vector, SIMD_WIDTH};
use crate::utils::{WAngularInertia, WCross};
use simba::simd::SimdValue;

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
    stiffness_impulse: SimdReal,
    damping_factor: SimdReal,

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
        let mj_lambda1 = gather![|ii| ids1[ii].active_set_offset];

        let position2 = Isometry::from(gather![|ii| poss2[ii].position]);
        let linvel2 = Vector::from(gather![|ii| vels2[ii].linvel]);
        let angvel2 = AngVector::<SimdReal>::from(gather![|ii| vels2[ii].angvel]);
        let world_com2 = Point::from(gather![|ii| mprops2[ii].world_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ]);
        let mj_lambda2 = gather![|ii| ids2[ii].active_set_offset];

        let rest_length = SimdReal::from(gather![|ii| cparams[ii].rest_length]);
        let stiffness = SimdReal::from(gather![|ii| cparams[ii].stiffness]);
        let damping = SimdReal::from(gather![|ii| cparams[ii].damping]);

        let local_anchor1 = Point::from(gather![|ii| cparams[ii].local_anchor1]);
        let local_anchor2 = Point::from(gather![|ii| cparams[ii].local_anchor2]);
        let impulse = SimdReal::from(gather![|ii| cparams[ii].impulse]);
        let dt = SimdReal::splat(params.dt);

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let dx = current_length - rest_length;

        let stiffness_impulse = stiffness * dx * dt;
        let damping_factor = damping * dt;

        WSpringVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            im2,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            r1: anchor1,
            r2: anchor2,
            stiffness_impulse,
            damping_factor,
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

        let impulse: Vector<SimdReal> = self.u * self.impulse;

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

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1 + mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);
        let impulse = -(self.stiffness_impulse + self.damping_factor * dvel);

        let impulse = impulse - self.impulse;

        self.impulse += impulse;
        let impulse = self.u * impulse;

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
    stiffness_impulse: SimdReal,
    damping_factor: SimdReal,

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

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);

        let u = anchor_world2 - anchor_world1;
        let current_length = u.magnitude();
        let u = u.normalize();

        let dx = current_length - rest_length;

        let stiffness_impulse = stiffness * dx * dt;
        let damping_factor = damping * dt;

        Self {
            joint_id,
            mj_lambda2,
            im2,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            u,
            dx,
            vel1,
            vel2,
            r1: anchor1,
            r2: anchor2,
            stiffness_impulse,
            damping_factor,
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

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2: DeltaVel<SimdReal> = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel1 = self.vel1;
        let vel2 = self.vel2 + mj_lambda2.linear + ang_vel2.gcross(self.r2);

        let dvel = (vel2 - vel1).dot(&self.u);
        let impulse = -(self.stiffness_impulse + self.damping_factor * dvel);

        let impulse = impulse - self.impulse;

        self.impulse += impulse;
        let impulse = self.u * impulse;

        mj_lambda2.linear += impulse * self.im2;
        mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

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
