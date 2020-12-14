use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, PrismaticJoint, RigidBody,
};
use crate::math::{
    AngVector, AngularInertia, Isometry, Point, SimdBool, SimdReal, Vector, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim3")]
use na::{Cholesky, Matrix3x2, Matrix5, Vector5, U2, U3};
#[cfg(feature = "dim2")]
use {
    crate::utils::SdpMatrix2,
    na::{Matrix2, Vector2},
};

#[cfg(feature = "dim2")]
type LinImpulseDim = na::U1;
#[cfg(feature = "dim3")]
type LinImpulseDim = na::U2;

#[derive(Debug)]
pub(crate) struct WPrismaticVelocityConstraint {
    mj_lambda1: [usize; SIMD_WIDTH],
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    r1: Vector<SimdReal>,
    r2: Vector<SimdReal>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<SimdReal>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<SimdReal>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<SimdReal>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<SimdReal>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<SimdReal>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<SimdReal>,

    limits_impulse: SimdReal,
    limits_forcedirs: Option<(Vector<SimdReal>, Vector<SimdReal>)>,
    limits_rhs: SimdReal,

    #[cfg(feature = "dim2")]
    basis1: Vector2<SimdReal>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1_sqrt: AngularInertia<SimdReal>,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WPrismaticVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&PrismaticJoint; SIMD_WIDTH],
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
        let local_axis1 = Vector::from(array![|ii| *cparams[ii].local_axis1; SIMD_WIDTH]);
        let local_axis2 = Vector::from(array![|ii| *cparams[ii].local_axis2; SIMD_WIDTH]);

        #[cfg(feature = "dim2")]
        let local_basis1 = [Vector::from(array![|ii| cparams[ii].basis1[0]; SIMD_WIDTH])];
        #[cfg(feature = "dim3")]
        let local_basis1 = [
            Vector::from(array![|ii| cparams[ii].basis1[0]; SIMD_WIDTH]),
            Vector::from(array![|ii| cparams[ii].basis1[1]; SIMD_WIDTH]),
        ];

        #[cfg(feature = "dim2")]
        let impulse = Vector2::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);
        #[cfg(feature = "dim3")]
        let impulse = Vector5::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;
        let axis1 = position1 * local_axis1;
        let axis2 = position2 * local_axis2;

        #[cfg(feature = "dim2")]
        let basis1 = position1 * local_basis1[0];
        #[cfg(feature = "dim3")]
        let basis1 =
            Matrix3x2::from_columns(&[position1 * local_basis1[0], position1 * local_basis1[1]]);

        // #[cfg(feature = "dim2")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .to_rotation_matrix()
        //     .into_inner();
        // #[cfg(feature = "dim3")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .unwrap_or(Rotation::identity())
        //     .to_rotation_matrix()
        //     .into_inner();
        // let basis2 = r21 * basis1;
        // NOTE: we use basis2 := basis1 for now is that allows
        // simplifications of the computation without introducing
        // much instabilities.

        let ii1 = ii1_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r1_mat = r1.gcross_matrix();

        let ii2 = ii2_sqrt.squared();
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let r1_mat_b1 = r1_mat * basis1;
            let r2_mat_b1 = r2_mat * basis1;

            lhs = Matrix5::zeros();
            let lhs00 = ii1.quadform3x2(&r1_mat_b1).add_diagonal(im1)
                + ii2.quadform3x2(&r2_mat_b1).add_diagonal(im2);
            let lhs10 = ii1 * r1_mat_b1 + ii2 * r2_mat_b1;
            let lhs11 = (ii1 + ii2).into_matrix();
            lhs.fixed_slice_mut::<U2, U2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(2, 2).copy_from(&lhs11);
        }

        #[cfg(feature = "dim2")]
        {
            let b1r1 = basis1.dot(&r1_mat);
            let b2r2 = basis1.dot(&r2_mat);
            let m11 = im1 + im2 + b1r1 * ii1 * b1r1 + b2r2 * ii2 * b2r2;
            let m12 = basis1.dot(&r1_mat) * ii1 + basis1.dot(&r2_mat) * ii2;
            let m22 = ii1 + ii2;
            lhs = SdpMatrix2::new(m11, m12, m22);
        }

        let anchor_linvel1 = linvel1 + angvel1.gcross(r1);
        let anchor_linvel2 = linvel2 + angvel2.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let ang_rhs = angvel2 - angvel1;

        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_rhs.x, ang_rhs);
        #[cfg(feature = "dim3")]
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, ang_rhs.x, ang_rhs.y, ang_rhs.z);

        // Setup limit constraint.
        let mut limits_forcedirs = None;
        let mut limits_rhs = na::zero();
        let mut limits_impulse = na::zero();
        let limits_enabled = SimdBool::from(array![|ii| cparams[ii].limits_enabled; SIMD_WIDTH]);

        if limits_enabled.any() {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // FIXME: we should allow both limits to be active at
            // the same time + allow predictive constraint activation.
            let min_limit = SimdReal::from(array![|ii| cparams[ii].limits[0]; SIMD_WIDTH]);
            let max_limit = SimdReal::from(array![|ii| cparams[ii].limits[1]; SIMD_WIDTH]);
            let lim_impulse = SimdReal::from(array![|ii| cparams[ii].limits_impulse; SIMD_WIDTH]);

            let min_enabled = dist.simd_lt(min_limit);
            let max_enabled = dist.simd_gt(max_limit);
            let _0: SimdReal = na::zero();
            let _1: SimdReal = na::one();
            let sign = _1.select(min_enabled, (-_1).select(max_enabled, _0));

            if sign != _0 {
                limits_forcedirs = Some((axis1 * -sign, axis2 * sign));
                limits_rhs = (anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1)) * sign;
                limits_impulse = lim_impulse.select(min_enabled | max_enabled, _0);
            }
        }

        WPrismaticVelocityConstraint {
            joint_id,
            mj_lambda1,
            mj_lambda2,
            im1,
            ii1_sqrt,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            limits_impulse: limits_impulse * SimdReal::splat(params.warmstart_coeff),
            limits_forcedirs,
            limits_rhs,
            basis1,
            inv_lhs,
            rhs,
            r1,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
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

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        if let Some((limits_forcedir1, limits_forcedir2)) = self.limits_forcedirs {
            mj_lambda1.linear += limits_forcedir1 * (self.im1 * self.limits_impulse);
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * self.limits_impulse);
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

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
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

        /*
         * Joint consraint.
         */
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_vel1 = mj_lambda1.linear + ang_vel1.gcross(self.r1);
        let lin_vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let lin_dvel = self.basis1.tr_mul(&(lin_vel2 - lin_vel1));
        let ang_dvel = ang_vel2 - ang_vel1;
        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_dvel.x, ang_dvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, ang_dvel.x, ang_dvel.y, ang_dvel.z) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Joint limits.
         */
        if let Some((limits_forcedir1, limits_forcedir2)) = self.limits_forcedirs {
            // FIXME: the transformation by ii2_sqrt could be avoided by
            // reusing some computations above.
            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = limits_forcedir2.dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + limits_forcedir1.dot(&(mj_lambda1.linear + ang_vel1.gcross(self.r1)))
                + self.limits_rhs;
            let new_impulse =
                (self.limits_impulse - lin_dvel / (self.im1 + self.im2)).simd_max(na::zero());
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda1.linear += limits_forcedir1 * (self.im1 * dimpulse);
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * dimpulse);
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
            if let JointParams::PrismaticJoint(rev) = &mut joint.params {
                rev.impulse = self.impulse.extract(ii);
                rev.limits_impulse = self.limits_impulse.extract(ii);
            }
        }
    }
}

#[derive(Debug)]
pub(crate) struct WPrismaticVelocityGroundConstraint {
    mj_lambda2: [usize; SIMD_WIDTH],

    joint_id: [JointIndex; SIMD_WIDTH],

    r2: Vector<SimdReal>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<SimdReal>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<SimdReal>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<SimdReal>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<SimdReal>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<SimdReal>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<SimdReal>,

    limits_impulse: SimdReal,
    limits_rhs: SimdReal,

    axis2: Vector<SimdReal>,
    #[cfg(feature = "dim2")]
    basis1: Vector2<SimdReal>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<SimdReal>,
    limits_forcedir2: Option<Vector<SimdReal>>,

    im2: SimdReal,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WPrismaticVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        rbs1: [&RigidBody; SIMD_WIDTH],
        rbs2: [&RigidBody; SIMD_WIDTH],
        cparams: [&PrismaticJoint; SIMD_WIDTH],
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

        #[cfg(feature = "dim2")]
        let impulse = Vector2::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);
        #[cfg(feature = "dim3")]
        let impulse = Vector5::from(array![|ii| cparams[ii].impulse; SIMD_WIDTH]);

        let local_anchor1 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor2 } else { cparams[ii].local_anchor1 }; SIMD_WIDTH],
        );
        let local_anchor2 = Point::from(
            array![|ii| if flipped[ii] { cparams[ii].local_anchor1 } else { cparams[ii].local_anchor2 }; SIMD_WIDTH],
        );
        let local_axis1 = Vector::from(
            array![|ii| if flipped[ii] { *cparams[ii].local_axis2 } else { *cparams[ii].local_axis1 }; SIMD_WIDTH],
        );
        let local_axis2 = Vector::from(
            array![|ii| if flipped[ii] { *cparams[ii].local_axis1 } else { *cparams[ii].local_axis2 }; SIMD_WIDTH],
        );

        #[cfg(feature = "dim2")]
        let basis1 = position1
            * Vector::from(
                array![|ii| if flipped[ii] { cparams[ii].basis2[0] } else { cparams[ii].basis1[0] }; SIMD_WIDTH],
            );
        #[cfg(feature = "dim3")]
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
        let axis1 = position1 * local_axis1;
        let axis2 = position2 * local_axis2;

        // #[cfg(feature = "dim2")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .to_rotation_matrix()
        //     .into_inner();
        // #[cfg(feature = "dim3")]
        // let r21 = Rotation::rotation_between_axis(&axis1, &axis2)
        //     .unwrap_or(Rotation::identity())
        //     .to_rotation_matrix()
        //     .into_inner();
        // let basis2 = r21 * basis1;
        // NOTE: we use basis2 := basis1 for now is that allows
        // simplifications of the computation without introducing
        // much instabilities.
        let ii2 = ii2_sqrt.squared();
        let r1 = anchor1 - world_com1;
        let r2 = anchor2 - world_com2;
        let r2_mat = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let r2_mat_b1 = r2_mat * basis1;

            lhs = Matrix5::zeros();
            let lhs00 = ii2.quadform3x2(&r2_mat_b1).add_diagonal(im2);
            let lhs10 = ii2 * r2_mat_b1;
            let lhs11 = ii2.into_matrix();
            lhs.fixed_slice_mut::<U2, U2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(2, 2).copy_from(&lhs11);
        }

        #[cfg(feature = "dim2")]
        {
            let b2r2 = basis1.dot(&r2_mat);
            let m11 = im2 + b2r2 * ii2 * b2r2;
            let m12 = basis1.dot(&r2_mat) * ii2;
            let m22 = ii2;
            lhs = SdpMatrix2::new(m11, m12, m22);
        }

        let anchor_linvel1 = linvel1 + angvel1.gcross(r1);
        let anchor_linvel2 = linvel2 + angvel2.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let ang_rhs = angvel2 - angvel1;

        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_rhs.x, ang_rhs);
        #[cfg(feature = "dim3")]
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, ang_rhs.x, ang_rhs.y, ang_rhs.z);

        // Setup limit constraint.
        let mut limits_forcedir2 = None;
        let mut limits_rhs = na::zero();
        let mut limits_impulse = na::zero();
        let limits_enabled = SimdBool::from(array![|ii| cparams[ii].limits_enabled; SIMD_WIDTH]);

        if limits_enabled.any() {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // FIXME: we should allow both limits to be active at
            // the same time + allow predictive constraint activation.
            let min_limit = SimdReal::from(array![|ii| cparams[ii].limits[0]; SIMD_WIDTH]);
            let max_limit = SimdReal::from(array![|ii| cparams[ii].limits[1]; SIMD_WIDTH]);
            let lim_impulse = SimdReal::from(array![|ii| cparams[ii].limits_impulse; SIMD_WIDTH]);

            let use_min = dist.simd_lt(min_limit);
            let use_max = dist.simd_gt(max_limit);
            let _0: SimdReal = na::zero();
            let _1: SimdReal = na::one();
            let sign = _1.select(use_min, (-_1).select(use_max, _0));

            if sign != _0 {
                limits_forcedir2 = Some(axis2 * sign);
                limits_rhs = anchor_linvel2.dot(&axis2) * sign - anchor_linvel1.dot(&axis1) * sign;
                limits_impulse = lim_impulse.select(use_min | use_max, _0);
            }
        }

        WPrismaticVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            limits_impulse: limits_impulse * SimdReal::splat(params.warmstart_coeff),
            basis1,
            inv_lhs,
            rhs,
            r2,
            axis2,
            limits_forcedir2,
            limits_rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        if let Some(limits_forcedir2) = self.limits_forcedir2 {
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * self.limits_impulse);
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        /*
         * Joint consraint.
         */
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);
        let lin_vel2 = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let lin_dvel = self.basis1.tr_mul(&lin_vel2);
        let ang_dvel = ang_vel2;
        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_dvel.x, ang_dvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let rhs =
            Vector5::new(lin_dvel.x, lin_dvel.y, ang_dvel.x, ang_dvel.y, ang_dvel.z) + self.rhs;
        let impulse = self.inv_lhs * rhs;
        self.impulse += impulse;
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Joint limits.
         */
        if let Some(limits_forcedir2) = self.limits_forcedir2 {
            // FIXME: the transformation by ii2_sqrt could be avoided by
            // reusing some computations above.
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = limits_forcedir2.dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - lin_dvel / self.im2).simd_max(na::zero());
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda2.linear += limits_forcedir2 * (self.im2 * dimpulse);
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            if let JointParams::PrismaticJoint(rev) = &mut joint.params {
                rev.impulse = self.impulse.extract(ii);
                rev.limits_impulse = self.limits_impulse.extract(ii);
            }
        }
    }
}
