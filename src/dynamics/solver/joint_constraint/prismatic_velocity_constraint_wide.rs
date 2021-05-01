use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, PrismaticJoint, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{
    AngVector, AngularInertia, Isometry, Point, Real, SimdBool, SimdReal, Vector, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix, WDot};

#[cfg(feature = "dim3")]
use na::{Cholesky, Matrix3x2, Matrix5, Vector3, Vector5};

#[cfg(feature = "dim2")]
use {
    na::{Matrix2, Vector2},
    parry::utils::SdpMatrix2,
};

#[cfg(feature = "dim2")]
const LIN_IMPULSE_DIM: usize = 1;

#[cfg(feature = "dim3")]
const LIN_IMPULSE_DIM: usize = 2;

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

    limits_active: bool,
    limits_impulse: SimdReal,
    limits_forcedir2: Vector<SimdReal>,
    limits_rhs: SimdReal,
    limits_inv_lhs: SimdReal,
    limits_impulse_limits: (SimdReal, SimdReal),

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
        cparams: [&PrismaticJoint; SIMD_WIDTH],
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
        let local_axis1 = Vector::from(gather![|ii| *cparams[ii].local_axis1]);
        let local_axis2 = Vector::from(gather![|ii| *cparams[ii].local_axis2]);

        #[cfg(feature = "dim2")]
        let local_basis1 = [Vector::from(gather![|ii| cparams[ii].basis1[0]])];
        #[cfg(feature = "dim3")]
        let local_basis1 = [
            Vector::from(gather![|ii| cparams[ii].basis1[0]]),
            Vector::from(gather![|ii| cparams[ii].basis1[1]]),
        ];

        #[cfg(feature = "dim2")]
        let impulse = Vector2::from(gather![|ii| cparams[ii].impulse]);
        #[cfg(feature = "dim3")]
        let impulse = Vector5::from(gather![|ii| cparams[ii].impulse]);

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
        //     .unwrap_or_else(Rotation::identity)
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
            lhs.fixed_slice_mut::<2, 2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<3, 2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<3, 3>(2, 2).copy_from(&lhs11);
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

        let linvel_err = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let angvel_err = angvel2 - angvel1;

        let velocity_solve_fraction = SimdReal::splat(params.velocity_solve_fraction);

        #[cfg(feature = "dim2")]
        let mut rhs = Vector2::new(linvel_err.x, angvel_err) * velocity_solve_fraction;
        #[cfg(feature = "dim3")]
        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            angvel_err.x,
            angvel_err.y,
            angvel_err.z,
        ) * velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let linear_err = basis1.tr_mul(&(anchor2 - anchor1));

            let local_frame1 = Isometry::from(gather![|ii| cparams[ii].local_frame1()]);
            let local_frame2 = Isometry::from(gather![|ii| cparams[ii].local_frame2()]);

            let frame1 = position1 * local_frame1;
            let frame2 = position2 * local_frame2;
            let ang_err = frame2.rotation * frame1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                rhs += Vector2::new(linear_err.x, ang_err.angle()) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = Vector3::from(gather![|ii| ang_err.extract(ii).scaled_axis()]);
                rhs += Vector5::new(linear_err.x, linear_err.y, ang_err.x, ang_err.y, ang_err.z)
                    * velocity_based_erp_inv_dt;
            }
        }

        // Setup limit constraint.
        let zero: SimdReal = na::zero();
        let limits_forcedir2 = axis2; // hopefully axis1 is colinear with axis2
        let mut limits_active = false;
        let mut limits_rhs = zero;
        let mut limits_impulse = zero;
        let mut limits_inv_lhs = zero;
        let mut limits_impulse_limits = (zero, zero);

        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        if limits_enabled.any() {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // TODO: we should allow predictive constraint activation.

            let min_limit = SimdReal::from(gather![|ii| cparams[ii].limits[0]]);
            let max_limit = SimdReal::from(gather![|ii| cparams[ii].limits[1]]);

            let min_enabled = dist.simd_lt(min_limit);
            let max_enabled = dist.simd_gt(max_limit);

            limits_impulse_limits.0 = SimdReal::splat(-Real::INFINITY).select(max_enabled, zero);
            limits_impulse_limits.1 = SimdReal::splat(Real::INFINITY).select(min_enabled, zero);

            limits_active = (min_enabled | max_enabled).any();
            if limits_active {
                let gcross1 = r1.gcross(axis1);
                let gcross2 = r2.gcross(axis2);

                limits_rhs = (anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1))
                    * velocity_solve_fraction;

                limits_rhs += ((dist - max_limit).simd_max(zero)
                    - (min_limit - dist).simd_max(zero))
                    * SimdReal::splat(velocity_based_erp_inv_dt);

                limits_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_impulse])
                    .simd_max(limits_impulse_limits.0)
                    .simd_min(limits_impulse_limits.1);

                limits_inv_lhs = SimdReal::splat(1.0)
                    / (im1
                        + im2
                        + gcross1.gdot(ii1.transform_vector(gcross1))
                        + gcross2.gdot(ii2.transform_vector(gcross2)));
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
            limits_active,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            limits_impulse: limits_impulse * SimdReal::splat(params.warmstart_coeff),
            limits_forcedir2,
            limits_rhs,
            limits_inv_lhs,
            limits_impulse_limits,
            basis1,
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

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LIN_IMPULSE_DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(2).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        // Warmstart limits.
        if self.limits_active {
            let limit_impulse1 = -self.limits_forcedir2 * self.limits_impulse;
            let limit_impulse2 = self.limits_forcedir2 * self.limits_impulse;

            mj_lambda1.linear += limit_impulse1 * self.im1;
            mj_lambda1.angular += self
                .ii1_sqrt
                .transform_vector(self.r1.gcross(limit_impulse1));
            mj_lambda2.linear += limit_impulse2 * self.im2;
            mj_lambda2.angular += self
                .ii2_sqrt
                .transform_vector(self.r2.gcross(limit_impulse2));
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

    fn solve_dofs(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
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
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LIN_IMPULSE_DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<3>(2).into_owned();

        mj_lambda1.linear += lin_impulse * self.im1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));
    }

    fn solve_limits(
        &mut self,
        mj_lambda1: &mut DeltaVel<SimdReal>,
        mj_lambda2: &mut DeltaVel<SimdReal>,
    ) {
        if self.limits_active {
            let limits_forcedir1 = -self.limits_forcedir2;
            let limits_forcedir2 = self.limits_forcedir2;

            let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = limits_forcedir2.dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + limits_forcedir1.dot(&(mj_lambda1.linear + ang_vel1.gcross(self.r1)))
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - lin_dvel * self.limits_inv_lhs)
                .simd_max(self.limits_impulse_limits.0)
                .simd_min(self.limits_impulse_limits.1);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            let lin_impulse1 = limits_forcedir1 * dimpulse;
            let lin_impulse2 = limits_forcedir2 * dimpulse;

            mj_lambda1.linear += lin_impulse1 * self.im1;
            mj_lambda1.angular += self.ii1_sqrt.transform_vector(self.r1.gcross(lin_impulse1));
            mj_lambda2.linear += lin_impulse2 * self.im2;
            mj_lambda2.angular += self.ii2_sqrt.transform_vector(self.r2.gcross(lin_impulse2));
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

        self.solve_dofs(&mut mj_lambda1, &mut mj_lambda2);
        self.solve_limits(&mut mj_lambda1, &mut mj_lambda2);

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

    limits_active: bool,
    limits_forcedir2: Vector<SimdReal>,
    limits_impulse: SimdReal,
    limits_rhs: SimdReal,
    limits_impulse_limits: (SimdReal, SimdReal),

    axis2: Vector<SimdReal>,
    #[cfg(feature = "dim2")]
    basis1: Vector2<SimdReal>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<SimdReal>,

    im2: SimdReal,
    ii2_sqrt: AngularInertia<SimdReal>,
}

impl WPrismaticVelocityGroundConstraint {
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
        cparams: [&PrismaticJoint; SIMD_WIDTH],
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

        #[cfg(feature = "dim2")]
        let impulse = Vector2::from(gather![|ii| cparams[ii].impulse]);
        #[cfg(feature = "dim3")]
        let impulse = Vector5::from(gather![|ii| cparams[ii].impulse]);

        let local_anchor1 = Point::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_anchor2
        } else {
            cparams[ii].local_anchor1
        }]);
        let local_anchor2 = Point::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_anchor1
        } else {
            cparams[ii].local_anchor2
        }]);
        let local_axis1 = Vector::from(gather![|ii| if flipped[ii] {
            *cparams[ii].local_axis2
        } else {
            *cparams[ii].local_axis1
        }]);
        let local_axis2 = Vector::from(gather![|ii| if flipped[ii] {
            *cparams[ii].local_axis1
        } else {
            *cparams[ii].local_axis2
        }]);

        #[cfg(feature = "dim2")]
        let basis1 = position1
            * Vector::from(gather![|ii| if flipped[ii] {
                cparams[ii].basis2[0]
            } else {
                cparams[ii].basis1[0]
            }]);
        #[cfg(feature = "dim3")]
        let basis1 = Matrix3x2::from_columns(&[
            position1
                * Vector::from(gather![|ii| if flipped[ii] {
                    cparams[ii].basis2[0]
                } else {
                    cparams[ii].basis1[0]
                }]),
            position1
                * Vector::from(gather![|ii| if flipped[ii] {
                    cparams[ii].basis2[1]
                } else {
                    cparams[ii].basis1[1]
                }]),
        ]);

        let anchor1 = position1 * local_anchor1;
        let anchor2 = position2 * local_anchor2;
        let axis1 = position1 * local_axis1;
        let axis2 = position2 * local_axis2;

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
            lhs.fixed_slice_mut::<2, 2>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<3, 2>(2, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<3, 3>(2, 2).copy_from(&lhs11);
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

        let linvel_err = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let angvel_err = angvel2 - angvel1;

        let velocity_solve_fraction = SimdReal::splat(params.velocity_solve_fraction);

        #[cfg(feature = "dim2")]
        let mut rhs = Vector2::new(linvel_err.x, angvel_err) * velocity_solve_fraction;
        #[cfg(feature = "dim3")]
        let mut rhs = Vector5::new(
            linvel_err.x,
            linvel_err.y,
            angvel_err.x,
            angvel_err.y,
            angvel_err.z,
        ) * velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let velocity_based_erp_inv_dt = SimdReal::splat(velocity_based_erp_inv_dt);

            let linear_err = basis1.tr_mul(&(anchor2 - anchor1));

            let frame1 = position1
                * Isometry::from(gather![|ii| if flipped[ii] {
                    cparams[ii].local_frame2()
                } else {
                    cparams[ii].local_frame1()
                }]);
            let frame2 = position2
                * Isometry::from(gather![|ii| if flipped[ii] {
                    cparams[ii].local_frame1()
                } else {
                    cparams[ii].local_frame2()
                }]);

            let ang_err = frame2.rotation * frame1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                rhs += Vector2::new(linear_err.x, ang_err.angle()) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = Vector3::from(gather![|ii| ang_err.extract(ii).scaled_axis()]);
                rhs += Vector5::new(linear_err.x, linear_err.y, ang_err.x, ang_err.y, ang_err.z)
                    * velocity_based_erp_inv_dt;
            }
        }

        // Setup limit constraint.
        let zero: SimdReal = na::zero();
        let limits_forcedir2 = axis2; // hopefully axis1 is colinear with axis2
        let mut limits_active = false;
        let mut limits_rhs = zero;
        let mut limits_impulse = zero;
        let mut limits_impulse_limits = (zero, zero);

        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        if limits_enabled.any() {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // TODO: we should allow predictive constraint activation.
            let min_limit = SimdReal::from(gather![|ii| cparams[ii].limits[0]]);
            let max_limit = SimdReal::from(gather![|ii| cparams[ii].limits[1]]);

            let min_enabled = dist.simd_lt(min_limit);
            let max_enabled = dist.simd_gt(max_limit);

            limits_impulse_limits.0 = SimdReal::splat(-Real::INFINITY).select(max_enabled, zero);
            limits_impulse_limits.1 = SimdReal::splat(Real::INFINITY).select(min_enabled, zero);

            limits_active = (min_enabled | max_enabled).any();
            if limits_active {
                limits_rhs = (anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1))
                    * velocity_solve_fraction;

                limits_rhs += ((dist - max_limit).simd_max(zero)
                    - (min_limit - dist).simd_max(zero))
                    * SimdReal::splat(velocity_based_erp_inv_dt);

                limits_impulse = SimdReal::from(gather![|ii| cparams[ii].limits_impulse])
                    .simd_max(limits_impulse_limits.0)
                    .simd_min(limits_impulse_limits.1);
            }
        }

        WPrismaticVelocityGroundConstraint {
            joint_id,
            mj_lambda2,
            im2,
            ii2_sqrt,
            impulse: impulse * SimdReal::splat(params.warmstart_coeff),
            limits_active,
            limits_forcedir2,
            limits_rhs,
            limits_impulse: limits_impulse * SimdReal::splat(params.warmstart_coeff),
            limits_impulse_limits,
            basis1,
            inv_lhs,
            rhs,
            r2,
            axis2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LIN_IMPULSE_DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(2).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambda2.linear += self.limits_forcedir2 * (self.im2 * self.limits_impulse);

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    fn solve_dofs(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
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
        let lin_impulse = self.basis1 * impulse.fixed_rows::<LIN_IMPULSE_DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = impulse.fixed_rows::<3>(2).into_owned();

        mj_lambda2.linear -= lin_impulse * self.im2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));
    }

    fn solve_limits(&mut self, mj_lambda2: &mut DeltaVel<SimdReal>) {
        if self.limits_active {
            // FIXME: the transformation by ii2_sqrt could be avoided by
            // reusing some computations above.
            let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let lin_dvel = self
                .limits_forcedir2
                .dot(&(mj_lambda2.linear + ang_vel2.gcross(self.r2)))
                + self.limits_rhs;
            let new_impulse = (self.limits_impulse - lin_dvel / self.im2)
                .simd_max(self.limits_impulse_limits.0)
                .simd_min(self.limits_impulse_limits.1);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda2.linear += self.limits_forcedir2 * (self.im2 * dimpulse);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        self.solve_dofs(&mut mj_lambda2);
        self.solve_limits(&mut mj_lambda2);

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
