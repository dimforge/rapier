use simba::simd::SimdValue;

use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RevoluteJoint, RigidBody,
};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Real, SimdReal, Vector, SIMD_WIDTH};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::{Cholesky, Matrix3x2, Matrix5, Vector5, U2, U3};

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

    basis1: Matrix3x2<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WRevoluteVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&RevoluteJoint; SIMD_WIDTH],
    ) -> Self {
        let position1 = Isometry::from(array![|ii| rbs1[ii].position; SIMD_WIDTH]);
        let linvel1 = Vector::from(array![|ii| rbs1[ii].linvel; SIMD_WIDTH]);
        let angvel1 = AngVector::<SimdReal>::from(array![|ii| rbs1[ii].angvel; SIMD_WIDTH]);
        let world_com1 = Point::from(array![|ii| rbs1[ii].world_com; SIMD_WIDTH]);
        let im1 = SimdReal::from(array![|ii| rbs1[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let ii1_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs1[ii].world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda1 = array![|ii| rbs1[ii].active_set_offset; SIMD_WIDTH];

        let position2 = Isometry::from(array![|ii| rbs2[ii].position; SIMD_WIDTH]);
        let linvel2 = Vector::from(array![|ii| rbs2[ii].linvel; SIMD_WIDTH]);
        let angvel2 = AngVector::<SimdReal>::from(array![|ii| rbs2[ii].angvel; SIMD_WIDTH]);
        let world_com2 = Point::from(array![|ii| rbs2[ii].world_com; SIMD_WIDTH]);
        let im2 = SimdReal::from(array![|ii| rbs2[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs2[ii].world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let local_anchor1 = Point::from(array![|ii| cparams[ii].local_anchor1; SIMD_WIDTH]);
        let local_anchor2 = Point::from(array![|ii| cparams[ii].local_anchor2; SIMD_WIDTH]);
        let local_basis1 = [
            Vector::from(array![|ii| cparams[ii].basis1[0]; SIMD_WIDTH]),
            Vector::from(array![|ii| cparams[ii].basis1[1]; SIMD_WIDTH]),
        ];
        let impulse = Vector5::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;
        let basis1 =
            Matrix3x2::from_columns(&[position1 * local_basis1[0], position1 * local_basis1[1]]);

        //        let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //            .unwrap_or(Rotation::identity())
        //            .to_rotation_matrix()
        //            .into_inner();
        //        let basis2 = r21 * basis1;
        // NOTE: to simplify, we use basis2 = basis1.
        // Though we may want to test if that does not introduce any instability.
        let ii1 = ii1_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r1_mat = r1.gcross_matrix();

        let ii2 = ii2_sqrt.squared();
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 =
            ii2.quadform(&r2_mat).add_diagonal(im2) + ii1.quadform(&r1_mat).add_diagonal(im1);
        let lhs10 = basis1.tr_mul(&(ii2 * r2_mat + ii1 * r1_mat));
        let lhs11 = (ii1 + ii2).quadform3x2(&basis1).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = linvel2 + angvel2.gcross(r2) - linvel1 - angvel1.gcross(r1);
        let ang_rhs = basis1.tr_mul(&(angvel2 - angvel1));
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        WRevoluteVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            ii1_sqrt,
            basis1,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            inv_lhs,
            rhs,
            r1,
            r2,
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

        let lin_impulse = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * self.impulse.fixed_rows::<U2>(3).into_owned();

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

        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2)
            - mj_lambda1.linear
            - ang_vel1.gcross(self.r1);
        let ang_dvel = self.basis1.tr_mul(&(ang_vel2 - ang_vel1));
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * impulse.fixed_rows::<U2>(3).into_owned();

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
            if let JointParams::RevoluteJoint(rev) = &mut joint.params {
                rev.impulse = self.impulse.extract(ii)
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

    basis1: Matrix3x2<SimdReal>,

    im2: SimdReal,

    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WRevoluteVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&RevoluteJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let position1 = Isometry::from(array![|ii| rbs1[ii].position; SIMD_WIDTH]);
        let linvel1 = Vector::from(array![|ii| rbs1[ii].linvel; SIMD_WIDTH]);
        let angvel1 = AngVector::<SimdReal>::from(array![|ii| rbs1[ii].angvel; SIMD_WIDTH]);
        let world_com1 = Point::from(array![|ii| rbs1[ii].world_com; SIMD_WIDTH]);

        let position2 = Isometry::from(array![|ii| rbs2[ii].position; SIMD_WIDTH]);
        let linvel2 = Vector::from(array![|ii| rbs2[ii].linvel; SIMD_WIDTH]);
        let angvel2 = AngVector::<SimdReal>::from(array![|ii| rbs2[ii].angvel; SIMD_WIDTH]);
        let world_com2 = Point::from(array![|ii| rbs2[ii].world_com; SIMD_WIDTH]);
        let im2 = SimdReal::from(array![|ii| rbs2[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let ii2_sqrt = AngularInertia::<SimdReal>::from(
            array![|ii| rbs2[ii].world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let mj_lambda2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];
        let impulse = Vector5::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

        let local_anchor1 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor2 } else { cparams[ii].local_anchor1 }; SIMD_WIDTH],
        );
        let local_anchor2 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor1 } else { cparams[ii].local_anchor2 }; SIMD_WIDTH],
        );
        let basis1 = Matrix3x2::from_columns(&[
            position1
                * Vector::from(
                    array![|ii| if flipped[ii] { cparams[ii].basis2[0] } else { cparams[ii].basis1[0] }; SIMD_WIDTH],
                ),
            position1
                * Vector::from(
                    array![|ii| if flipped[ii] { cparams[ii].basis2[1] } else { cparams[ii].basis1[1] }; SIMD_WIDTH],
                ),
        ]);

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;

        //        let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //            .unwrap_or(Rotation::identity())
        //            .to_rotation_matrix()
        //            .into_inner();
        //        let basis2 = /*r21 * */ basis1;
        let ii2 = ii2_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        let mut lhs = Matrix5::zeros();
        let lhs00 = ii2.quadform(&r2_mat).add_diagonal(im2);
        let lhs10 = basis1.tr_mul(&(ii2 * r2_mat));
        let lhs11 = ii2.quadform3x2(&basis1).into_matrix();

        // Note that cholesky won't read the upper-right part
        // of lhs so we don't have to fill it.
        lhs.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&lhs00.into_matrix());
        lhs.fixed_slice_mut::<U2, U3>(3, 0).copy_from(&lhs10);
        lhs.fixed_slice_mut::<U2, U2>(3, 3).copy_from(&lhs11);

        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = linvel2 + angvel2.gcross(r2) - linvel1 - angvel1.gcross(r1);
        let ang_rhs = basis1.tr_mul(&(angvel2 - angvel1));
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y);

        WRevoluteVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            basis1,
            inv_lhs,
            rhs,
            r2,
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

        let lin_impulse = self.impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * self.impulse.fixed_rows::<U2>(3).into_owned();

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
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_dvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let ang_dvel = self.basis1.tr_mul(&ang_vel2);
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = impulse.fixed_rows::<U3>(0).into_owned();
        let ang_impulse = self.basis1 * impulse.fixed_rows::<U2>(3).into_owned();

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
