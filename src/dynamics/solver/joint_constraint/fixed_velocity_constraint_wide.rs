use simba::simd::SimdValue;

use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    FixedJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{
    AngVector, AngularInertia, CrossMatrix, Isometry, Point, Real, SimdReal, SpacialVector, Vector,
    DIM, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim3")]
use na::{Cholesky, Matrix6, Vector3, Vector6};
#[cfg(feature = "dim2")]
use {
    na::{Matrix3, Vector3},
    parry::utils::SdpMatrix3,
};

#[derive(Debug)]
pub(crate) struct WFixedVelocityConstraint {
    mj_lambda1: [usize; SIMD_WIDTH],
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    impulse: SpacialVector<SimdReal>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<SimdReal>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<SimdReal>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<SimdReal>,
    #[cfg(feature = "dim2")]
    rhs: Vector3<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1: AngularInertia<SimdReal>,
    ii2: AngularInertia<SimdReal>,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,
}

impl WFixedVelocityConstraint {
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
        cparams: [&FixedJoint; SIMD_WIDTH],
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

        let local_frame1 = Isometry::from(gather![|ii| cparams[ii].local_frame1]);
        let local_frame2 = Isometry::from(gather![|ii| cparams[ii].local_frame2]);
        let impulse = SpacialVector::from(gather![|ii| cparams[ii].impulse]);

        let anchor1 = position1 * local_frame1;
        let anchor2 = position2 * local_frame2;
        let ii1 = ii1_sqrt.squared();
        let ii2 = ii2_sqrt.squared();
        let r1 = anchor1.translation.vector - world_com1.coords;
        let r2 = anchor2.translation.vector - world_com2.coords;
        let rmat1: CrossMatrix<_> = r1.gcross_matrix();
        let rmat2: CrossMatrix<_> = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let lhs00 =
                ii1.quadform(&rmat1).add_diagonal(im1) + ii2.quadform(&rmat2).add_diagonal(im2);
            let lhs10 = ii1.transform_matrix(&rmat1) + ii2.transform_matrix(&rmat2);
            let lhs11 = (ii1 + ii2).into_matrix();

            // Note that Cholesky only reads the lower-triangular part of the matrix
            // so we don't need to fill lhs01.
            lhs = Matrix6::zeros();
            lhs.fixed_slice_mut::<3, 3>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<3, 3>(3, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<3, 3>(3, 3).copy_from(&lhs11);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let m11 = im1 + im2 + rmat1.x * rmat1.x * ii1 + rmat2.x * rmat2.x * ii2;
            let m12 = rmat1.x * rmat1.y * ii1 + rmat2.x * rmat2.y * ii2;
            let m22 = im1 + im2 + rmat1.y * rmat1.y * ii1 + rmat2.y * rmat2.y * ii2;
            let m13 = rmat1.x * ii1 + rmat2.x * ii2;
            let m23 = rmat1.y * ii1 + rmat2.y * ii2;
            let m33 = ii1 + ii2;
            lhs = SdpMatrix3::new(m11, m12, m13, m22, m23, m33)
        }

        // NOTE: we don't use cholesky in 2D because we only have a 3x3 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix(); // FIXME: don't extract the matrix?
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_dvel = -linvel1 - angvel1.gcross(r1) + linvel2 + angvel2.gcross(r2);
        let ang_dvel = -angvel1 + angvel2;

        let velocity_solve_fraction = SimdReal::splat(params.velocity_solve_fraction);

        #[cfg(feature = "dim2")]
        let mut rhs = Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel) * velocity_solve_fraction;

        #[cfg(feature = "dim3")]
        let mut rhs = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        ) * velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let lin_err = anchor2.translation.vector - anchor1.translation.vector;
            let ang_err = anchor2.rotation * anchor1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                let ang_err = ang_err.angle();
                rhs += Vector3::new(lin_err.x, lin_err.y, ang_err) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = Vector3::from(gather![|ii| ang_err.extract(ii).scaled_axis()]);
                rhs += Vector6::new(
                    lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y, ang_err.z,
                ) * velocity_based_erp_inv_dt;
            }
        }

        WFixedVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            im2,
            ii1,
            ii2,
            ii1_sqrt,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            inv_lhs,
            r1,
            r2,
            rhs,
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

        let lin_impulse = self.impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

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

        let dlinvel = -mj_lambda1.linear - ang_vel1.gcross(self.r1)
            + mj_lambda2.linear
            + ang_vel2.gcross(self.r2);
        let dangvel = -ang_vel1 + ang_vel2;

        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

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
            if let JointParams::FixedJoint(fixed) = &mut joint.params {
                fixed.impulse = self.impulse.extract(ii)
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct WFixedVelocityGroundConstraint {
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    impulse: SpacialVector<SimdReal>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<SimdReal>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<SimdReal>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<SimdReal>,
    #[cfg(feature = "dim2")]
    rhs: Vector3<SimdReal>,

    im2: SimdReal,
    ii2: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
    r2: Vector<SimdReal>,
}

impl WFixedVelocityGroundConstraint {
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
        cparams: [&FixedJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let (poss1, vels1, mprops1) = rbs1;
        let (poss2, vels2, mprops2, ids2) = rbs2;

        let position1 = Isometry::from(gather![|ii| poss1[ii].position]);
        let linvel1 = Vector::from(gather![|ii| vels1[ii].linvel]);
        let angvel1 = AngVector::<SimdReal>::from(gather![|ii| vels1[ii].angvel]);
        let world_com1 = Point::from(gather![|ii| mprops1[ii].world_com]);

        let position2 = Isometry::from(gather![|ii| poss2[ii].position]);
        let linvel2 = Vector::from(gather![|ii| vels2[ii].linvel]);
        let angvel2 = AngVector::<SimdReal>::from(gather![|ii| vels2[ii].angvel]);
        let world_com2 = Point::from(gather![|ii| mprops2[ii].world_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ]);
        let mj_lambda2 = gather![|ii| ids2[ii].active_set_offset];

        let local_frame1 = Isometry::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_frame2
        } else {
            cparams[ii].local_frame1
        }]);
        let local_frame2 = Isometry::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_frame1
        } else {
            cparams[ii].local_frame2
        }]);
        let impulse = SpacialVector::from(gather![|ii| cparams[ii].impulse]);

        let anchor1 = position1 * local_frame1;
        let anchor2 = position2 * local_frame2;
        let ii2 = ii2_sqrt.squared();
        let r1 = anchor1.translation.vector - world_com1.coords;
        let r2 = anchor2.translation.vector - world_com2.coords;
        let rmat2: CrossMatrix<_> = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let lhs00 = ii2.quadform(&rmat2).add_diagonal(im2);
            let lhs10 = ii2.transform_matrix(&rmat2);
            let lhs11 = ii2.into_matrix();

            lhs = Matrix6::zeros();
            lhs.fixed_slice_mut::<3, 3>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<3, 3>(3, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<3, 3>(3, 3).copy_from(&lhs11);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let m11 = im2 + rmat2.x * rmat2.x * ii2;
            let m12 = rmat2.x * rmat2.y * ii2;
            let m22 = im2 + rmat2.y * rmat2.y * ii2;
            let m13 = rmat2.x * ii2;
            let m23 = rmat2.y * ii2;
            let m33 = ii2;
            lhs = SdpMatrix3::new(m11, m12, m13, m22, m23, m33)
        }

        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix(); // FIXME: don't do into_matrix?
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_dvel = linvel2 + angvel2.gcross(r2) - linvel1 - angvel1.gcross(r1);
        let ang_dvel = angvel2 - angvel1;

        let velocity_solve_fraction = SimdReal::splat(params.velocity_solve_fraction);

        #[cfg(feature = "dim2")]
        let mut rhs = Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel) * velocity_solve_fraction;

        #[cfg(feature = "dim3")]
        let mut rhs = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        ) * velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let lin_err = anchor2.translation.vector - anchor1.translation.vector;
            let ang_err = anchor2.rotation * anchor1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                let ang_err = ang_err.angle();
                rhs += Vector3::new(lin_err.x, lin_err.y, ang_err) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = Vector3::from(gather![|ii| ang_err.extract(ii).scaled_axis()]);
                rhs += Vector6::new(
                    lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y, ang_err.z,
                ) * velocity_based_erp_inv_dt;
            }
        }

        WFixedVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            ii2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            inv_lhs,
            r2,
            rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let lin_impulse = self.impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

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
        let dlinvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let dangvel = ang_vel2;
        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let impulse = self.inv_lhs * rhs;

        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::FixedJoint(fixed) = &mut joint.params {
                fixed.impulse = self.impulse.extract(ii)
            }
        }
    }
}
