use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    BallJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{
    AngVector, AngularInertia, Isometry, Point, Real, SdpMatrix, SimdReal, Vector, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use simba::simd::SimdValue;

#[derive(Debug)]
pub(crate) struct WBallVelocityConstraint {
    mj_lambda1: [usize; SIMD_WIDTH],
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    rhs: Vector<SimdReal>,
    pub(crate) impulse: Vector<SimdReal>,

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,

    inv_lhs: SdpMatrix<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WBallVelocityConstraint {
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
        cparams: [&BallJoint; SIMD_WIDTH],
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

        let local_anchor1 = Point::from(gather![|ii| cparams[ii].local_anchor1]);
        let local_anchor2 = Point::from(gather![|ii| cparams[ii].local_anchor2]);
        let impulse = Vector::from(gather![|ii| cparams[ii].impulse]);

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);
        let rhs = (vel2 - vel1) * SimdReal::splat(params.velocity_solve_fraction)
            + (anchor_world2 - anchor_world1) * SimdReal::splat(params.velocity_based_erp_inv_dt());
        let lhs;

        let cmat1 = anchor1.gcross_matrix();
        let cmat2 = anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        {
            lhs = ii2_sqrt.squared().quadform(&cmat2).add_diagonal(im2)
                + ii1_sqrt.squared().quadform(&cmat1).add_diagonal(im1);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let ii1 = ii1_sqrt.squared();
            let ii2 = ii2_sqrt.squared();
            let m11 = im1 + im2 + cmat1.x * cmat1.x * ii1 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat1.x * cmat1.y * ii1 + cmat2.x * cmat2.y * ii2;
            let m22 = im1 + im2 + cmat1.y * cmat1.y * ii1 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let inv_lhs = lhs.inverse_unchecked();

        WBallVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            im2,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            r1: anchor1,
            r2: anchor2,
            rhs,
            inv_lhs,
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

        mj_lambda1.linear += self.impulse * self.im1;
        mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.r1.gcross(self.impulse));
        mj_lambda2.linear -= self.impulse * self.im2;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(self.impulse));

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
        let vel1 = mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let dvel = -vel1 + vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda1.linear += impulse * self.im1;
        mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.r1.gcross(impulse));

        mj_lambda2.linear -= impulse * self.im2;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

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
            if let JointParams::BallJoint(ball) = &mut joint.params {
                ball.impulse = self.impulse.extract(ii)
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct WBallVelocityGroundConstraint {
    mj_lambda2: [usize; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],
    rhs: Vector<SimdReal>,
    pub(crate) impulse: Vector<SimdReal>,
    r2: Vector<SimdReal>,
    inv_lhs: SdpMatrix<SimdReal>,
    im2: SimdReal,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WBallVelocityGroundConstraint {
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
        cparams: [&BallJoint; SIMD_WIDTH],
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
        let impulse = Vector::from(gather![|ii| cparams[ii].impulse]);

        let anchor_world1 = position1 * local_anchor1;
        let anchor_world2 = position2 * local_anchor2;
        let anchor1 = anchor_world1 - world_com1;
        let anchor2 = anchor_world2 - world_com2;

        let vel1: Vector<SimdReal> = linvel1 + angvel1.gcross(anchor1);
        let vel2: Vector<SimdReal> = linvel2 + angvel2.gcross(anchor2);
        let rhs = (vel2 - vel1) * SimdReal::splat(params.velocity_solve_fraction)
            + (anchor_world2 - anchor_world1) * SimdReal::splat(params.velocity_based_erp_inv_dt());
        let lhs;

        let cmat2 = anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        {
            lhs = ii2_sqrt.squared().quadform(&cmat2).add_diagonal(im2);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let ii2 = ii2_sqrt.squared();
            let m11 = im2 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat2.x * cmat2.y * ii2;
            let m22 = im2 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let inv_lhs = lhs.inverse_unchecked();

        WBallVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            r2: anchor2,
            rhs,
            inv_lhs,
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

        mj_lambda2.linear -= self.impulse * self.im2;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(self.impulse));

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

        let angvel = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let vel2 = mj_lambda2.linear + angvel.gcross(self.r2);
        let dvel = vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda2.linear -= impulse * self.im2;
        mj_lambda2.angular -= self.ii2_sqrt.transform_vector(self.r2.gcross(impulse));

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::BallJoint(ball) = &mut joint.params {
                ball.impulse = self.impulse.extract(ii)
            }
        }
    }
}
