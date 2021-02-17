use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    BallJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
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
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&BallJoint; SIMD_WIDTH],
    ) -> Self {
        let position1 = Isometry::from(array![|ii| rbs1[ii].position; SIMD_WIDTH]);
        let linvel1 = Vector::from(array![|ii| rbs1[ii].linvel; SIMD_WIDTH]);
        let angvel1 = AngVector::<SimdReal>::from(array![|ii| rbs1[ii].angvel; SIMD_WIDTH]);
        let world_com1 = Point::from(array![|ii| rbs1[ii].world_com; SIMD_WIDTH]);
        let im1 = SimdReal::from(array![|ii| rbs1[ii].effective_inv_mass; SIMD_WIDTH]);
        let ii1_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs1[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda1 = array![|ii| rbs1[ii].active_set_offset; SIMD_WIDTH];

        let position2 = Isometry::from(array![|ii| rbs2[ii].position; SIMD_WIDTH]);
        let linvel2 = Vector::from(array![|ii| rbs2[ii].linvel; SIMD_WIDTH]);
        let angvel2 = AngVector::<SimdReal>::from(array![|ii| rbs2[ii].angvel; SIMD_WIDTH]);
        let world_com2 = Point::from(array![|ii| rbs2[ii].world_com; SIMD_WIDTH]);
        let im2 = SimdReal::from(array![|ii| rbs2[ii].effective_inv_mass; SIMD_WIDTH]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs2[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let local_anchor1 = Point::from(array![|ii| cparams[ii].local_anchor1; SIMD_WIDTH]);
        let local_anchor2 = Point::from(array![|ii| cparams[ii].local_anchor2; SIMD_WIDTH]);
        let impulse = Vector::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

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
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular; SIMD_WIDTH],
            ),
        };
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
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
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular; SIMD_WIDTH],
            ),
        };
        let mut mj_lambda2: DeltaVel<SimdReal> = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
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
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&BallJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let position1 = Isometry::from(array![|ii| rbs1[ii].position; SIMD_WIDTH]);
        let linvel1 = Vector::from(array![|ii| rbs1[ii].linvel; SIMD_WIDTH]);
        let angvel1 = AngVector::<SimdReal>::from(array![|ii| rbs1[ii].angvel; SIMD_WIDTH]);
        let world_com1 = Point::from(array![|ii| rbs1[ii].world_com; SIMD_WIDTH]);
        let local_anchor1 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor2 } else { cparams[ii].local_anchor1 }; SIMD_WIDTH],
        );

        let position2 = Isometry::from(array![|ii| rbs2[ii].position; SIMD_WIDTH]);
        let linvel2 = Vector::from(array![|ii| rbs2[ii].linvel; SIMD_WIDTH]);
        let angvel2 = AngVector::<SimdReal>::from(array![|ii| rbs2[ii].angvel; SIMD_WIDTH]);
        let world_com2 = Point::from(array![|ii| rbs2[ii].world_com; SIMD_WIDTH]);
        let im2 = SimdReal::from(array![|ii| rbs2[ii].effective_inv_mass; SIMD_WIDTH]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs2[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let local_anchor2 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor1 } else { cparams[ii].local_anchor2 }; SIMD_WIDTH],
        );
        let impulse = Vector::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

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
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
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
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
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
