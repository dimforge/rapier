use simba::simd::SimdValue;

use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RevoluteJoint, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{
    AngVector, AngularInertia, Isometry, Point, Real, Rotation, SimdReal, Vector, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::{Cholesky, Matrix3x2, Matrix5, Unit, Vector5};

#[derive(Debug)]
pub(crate) struct WRevoluteVelocityConstraint {
    mj_lambda1: [usize; SIMD_WIDTH],
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,

    inv_lhs: Matrix5<SimdReal>,
    rhs: Vector5<SimdReal>,
    impulse: Vector5<SimdReal>,

    axis1: [Vector<Real>; SIMD_WIDTH],
    basis1: Matrix3x2<SimdReal>,
    basis2: Matrix3x2<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WRevoluteVelocityConstraint {
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
        joints: [&RevoluteJoint; SIMD_WIDTH],
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

        let local_anchor1 = Point::from(gather![|ii| joints[ii].local_anchor1]);
        let local_anchor2 = Point::from(gather![|ii| joints[ii].local_anchor2]);
        let local_basis1 = [
            Vector::from(gather![|ii| joints[ii].basis1[0]]),
            Vector::from(gather![|ii| joints[ii].basis1[1]]),
        ];
        let local_basis2 = [
            Vector::from(gather![|ii| joints[ii].basis2[0]]),
            Vector::from(gather![|ii| joints[ii].basis2[1]]),
        ];
        let impulse = Vector5::from(gather![|ii| joints[ii].impulse]);

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;
        let basis1 =
            Matrix3x2::from_columns(&[position1 * local_basis1[0], position1 * local_basis1[1]]);
        let basis2 =
            Matrix3x2::from_columns(&[position2 * local_basis2[0], position2 * local_basis2[1]]);
        let basis_projection2 = basis2 * basis2.transpose();
        let basis2 = basis_projection2 * basis1;

        let ii1 = ii1_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r1_mat = r1.gcross_matrix();

        let ii2 = ii2_sqrt.squared();
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 =
            ii2.quadform(&r2_mat).add_diagonal(im2) + ii1.quadform(&r1_mat).add_diagonal(im1);
        let lhs10 = basis1.tr_mul(&(ii2 * r2_mat)) + basis2.tr_mul(&(ii1 * r1_mat));
        let lhs11 = (ii1.quadform3x2(&basis1) + ii2.quadform3x2(&basis2)).into_matrix();

        // Note that Cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<3, 3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<2, 3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<2, 2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let linvel_err = linvel2 + angvel2.gcross(r2) - linvel1 - angvel1.gcross(r1);
        let angvel_err = basis2.tr_mul(&angvel2) - basis1.tr_mul(&angvel1);

        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            linvel_err.z,
            angvel_err.x,
            angvel_err.y,
        ) * SimdReal::splat(params.velocity_solve_fraction);

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let lin_err = anchor2 - anchor1;

            let local_axis1 = Unit::<Vector<_>>::from(gather![|ii| joints[ii].local_axis1]);
            let local_axis2 = Unit::<Vector<_>>::from(gather![|ii| joints[ii].local_axis2]);

            let axis1 = position1 * local_axis1;
            let axis2 = position2 * local_axis2;

            let axis_error = axis1.cross(&axis2);
            let ang_err =
                (basis2.tr_mul(&axis_error) + basis1.tr_mul(&axis_error)) * SimdReal::splat(0.5);

            rhs += Vector5::new(lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y)
                * velocity_based_erp_inv_dt;
        }

        /*
         * Adjust the warmstart impulse.
         * If the velocity along the free axis is somewhat high,
         * we need to adjust the angular warmstart impulse because it
         * may have a direction that is too different than last frame,
         * making it counter-productive.
         */
        let warmstart_coeff = SimdReal::splat(params.warmstart_coeff);
        let mut impulse = impulse * warmstart_coeff;

        let axis1 = gather![|ii| poss1[ii].position * *joints[ii].local_axis1];
        let rotated_impulse = Vector::from(gather![|ii| {
            let axis_rot = Rotation::rotation_between(&joints[ii].prev_axis1, &axis1[ii])
                .unwrap_or_else(Rotation::identity);
            axis_rot * joints[ii].world_ang_impulse
        }]);

        let rotated_basis_impulse = basis1.tr_mul(&rotated_impulse);
        impulse[3] = rotated_basis_impulse.x * warmstart_coeff;
        impulse[4] = rotated_basis_impulse.y * warmstart_coeff;

        WRevoluteVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            ii1_sqrt,
            axis1,
            basis1,
            basis2,
            im2,
            ii2_sqrt,
            impulse,
            inv_lhs,
            rhs,
            r1,
            r2,
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

        let lin_impulse1 = self.impulse.fixed_rows::<3>(0).into_owned();
        let lin_impulse2 = self.impulse.fixed_rows::<3>(0).into_owned();
        let ang_impulse1 = self.basis1 * self.impulse.fixed_rows::<2>(3).into_owned();
        let ang_impulse2 = self.basis2 * self.impulse.fixed_rows::<2>(3).into_owned();

        mj_lambda1.linear += lin_impulse1 * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse1 + self.r1.gcross(lin_impulse1));

        mj_lambda2.linear -= lin_impulse2 * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse2 + self.r2.gcross(lin_impulse2));

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

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let lin_dvel = (mj_lambda2.linear + ang_vel2.gcross(self.r2))
            - (mj_lambda1.linear + ang_vel1.gcross(self.r1));
        let ang_dvel = self.basis2.tr_mul(&ang_vel2) - self.basis1.tr_mul(&ang_vel1);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse1 = impulse.fixed_rows::<3>(0).into_owned();
        let lin_impulse2 = impulse.fixed_rows::<3>(0).into_owned();
        let ang_impulse1 = self.basis1 * impulse.fixed_rows::<2>(3).into_owned();
        let ang_impulse2 = self.basis2 * impulse.fixed_rows::<2>(3).into_owned();

        mj_lambda1.linear += lin_impulse1 * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse1 + self.r1.gcross(lin_impulse1));

        mj_lambda2.linear -= lin_impulse2 * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse2 + self.r2.gcross(lin_impulse2));

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
        let rot_part = self.impulse.fixed_rows::<2>(3).into_owned();
        let world_ang_impulse = self.basis1 * rot_part;

        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::RevoluteJoint(rev) = &mut joint.params {
                rev.impulse = self.impulse.extract(ii);
                rev.world_ang_impulse = world_ang_impulse.extract(ii);
                rev.prev_axis1 = self.axis1[ii];
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct WRevoluteVelocityGroundConstraint {
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    r2: Vector<SimdReal>,

    inv_lhs: Matrix5<SimdReal>,
    rhs: Vector5<SimdReal>,
    impulse: Vector5<SimdReal>,

    basis2: Matrix3x2<SimdReal>,

    im2: SimdReal,

    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WRevoluteVelocityGroundConstraint {
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
        joints: [&RevoluteJoint; SIMD_WIDTH],
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
        let impulse = Vector5::from(gather![|ii| joints[ii].impulse]);

        let local_anchor1 = Point::from(gather![|ii| if flipped[ii] {
            joints[ii].local_anchor2
        } else {
            joints[ii].local_anchor1
        }]);
        let local_anchor2 = Point::from(gather![|ii| if flipped[ii] {
            joints[ii].local_anchor1
        } else {
            joints[ii].local_anchor2
        }]);
        let basis1 = Matrix3x2::from_columns(&[
            position1
                * Vector::from(gather![|ii| if flipped[ii] {
                    joints[ii].basis2[0]
                } else {
                    joints[ii].basis1[0]
                }]),
            position1
                * Vector::from(gather![|ii| if flipped[ii] {
                    joints[ii].basis2[1]
                } else {
                    joints[ii].basis1[1]
                }]),
        ]);
        let basis2 = Matrix3x2::from_columns(&[
            position2
                * Vector::from(gather![|ii| if flipped[ii] {
                    joints[ii].basis1[0]
                } else {
                    joints[ii].basis2[0]
                }]),
            position2
                * Vector::from(gather![|ii| if flipped[ii] {
                    joints[ii].basis1[1]
                } else {
                    joints[ii].basis2[1]
                }]),
        ]);
        let basis_projection2 = basis2 * basis2.transpose();
        let basis2 = basis_projection2 * basis1;

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;

        let ii2 = ii2_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = ii2.quadform(&r2_mat).add_diagonal(im2);
        let lhs10 = basis2.tr_mul(&(ii2 * r2_mat));
        let lhs11 = ii2.quadform3x2(&basis2).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<3, 3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<2, 3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<2, 2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let linvel_err = (linvel2 + angvel2.gcross(r2)) - (linvel1 + angvel1.gcross(r1));
        let angvel_err = basis2.tr_mul(&angvel2) - basis1.tr_mul(&angvel1);

        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            linvel_err.z,
            angvel_err.x,
            angvel_err.y,
        ) * SimdReal::splat(params.velocity_solve_fraction);

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let lin_err = anchor2 - anchor1;

            let local_axis1 = Unit::<Vector<_>>::from(gather![|ii| if flipped[ii] {
                joints[ii].local_axis2
            } else {
                joints[ii].local_axis1
            }]);
            let local_axis2 = Unit::<Vector<_>>::from(gather![|ii| if flipped[ii] {
                joints[ii].local_axis1
            } else {
                joints[ii].local_axis2
            }]);
            let axis1 = position1 * local_axis1;
            let axis2 = position2 * local_axis2;

            let axis_error = axis1.cross(&axis2);
            let ang_err = basis2.tr_mul(&axis_error) - basis1.tr_mul(&axis_error);

            rhs += Vector5::new(lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y)
                * velocity_based_erp_inv_dt;
        }

        WRevoluteVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            basis2,
            inv_lhs,
            rhs,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let lin_impulse = self.impulse.fixed_rows::<3>(0).into_owned();
        let ang_impulse = self.basis2 * self.impulse.fixed_rows::<2>(3).into_owned();

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
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let ang_dvel = self.basis2.tr_mul(&ang_vel2);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<3>(0).into_owned();
        let ang_impulse = self.basis2 * impulse.fixed_rows::<2>(3).into_owned();

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
            if let JointParams::RevoluteJoint(rev) = &mut joint.params {
                rev.impulse = self.impulse.extract(ii)
            }
        }
    }
}
