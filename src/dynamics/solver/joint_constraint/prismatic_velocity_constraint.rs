use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, JointParams, PrismaticJoint, RigidBody,
};
use crate::math::{AngularInertia, Real, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim3")]
use na::{Cholesky, Matrix3x2, Matrix5, Vector5, U2, U3};
#[cfg(feature = "dim2")]
use {
    na::{Matrix2, Vector2},
    parry::utils::SdpMatrix2,
};

#[cfg(feature = "dim2")]
type LinImpulseDim = na::U1;
#[cfg(feature = "dim3")]
type LinImpulseDim = na::U2;

#[derive(Debug)]
pub(crate) struct PrismaticVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    r1: Vector<Real>,
    r2: Vector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<Real>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<Real>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<Real>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<Real>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<Real>,

    limits_impulse: Real,
    limits_forcedirs: Option<(Vector<Real>, Vector<Real>)>,
    limits_rhs: Real,

    #[cfg(feature = "dim2")]
    basis1: Vector2<Real>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<Real>,

    im1: Real,
    im2: Real,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
}

impl PrismaticVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &PrismaticJoint,
    ) -> Self {
        // Linear part.
        let anchor1 = rb1.position * cparams.local_anchor1;
        let anchor2 = rb2.position * cparams.local_anchor2;
        let axis1 = rb1.position * cparams.local_axis1;
        let axis2 = rb2.position * cparams.local_axis2;
        #[cfg(feature = "dim2")]
        let basis1 = rb1.position * cparams.basis1[0];
        #[cfg(feature = "dim3")]
        let basis1 = Matrix3x2::from_columns(&[
            rb1.position * cparams.basis1[0],
            rb1.position * cparams.basis1[1],
        ]);

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

        let im1 = rb1.effective_inv_mass;
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r1_mat = r1.gcross_matrix();

        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r2 = anchor2 - rb2.world_com;
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

        let anchor_linvel1 = rb1.linvel + rb1.angvel.gcross(r1);
        let anchor_linvel2 = rb2.linvel + rb2.angvel.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let ang_rhs = rb2.angvel - rb1.angvel;

        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_rhs.x, ang_rhs);
        #[cfg(feature = "dim3")]
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, ang_rhs.x, ang_rhs.y, ang_rhs.z);

        // Setup limit constraint.
        let mut limits_forcedirs = None;
        let mut limits_rhs = 0.0;
        let mut limits_impulse = 0.0;

        if cparams.limits_enabled {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // FIXME: we should allow both limits to be active at
            // the same time, and allow predictive constraint activation.
            if dist < cparams.limits[0] {
                limits_forcedirs = Some((-axis1.into_inner(), axis2.into_inner()));
                limits_rhs = anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1);
                limits_impulse = cparams.limits_impulse;
            } else if dist > cparams.limits[1] {
                limits_forcedirs = Some((axis1.into_inner(), -axis2.into_inner()));
                limits_rhs = -anchor_linvel2.dot(&axis2) + anchor_linvel1.dot(&axis1);
                limits_impulse = cparams.limits_impulse;
            }
        }

        PrismaticVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            ii1_sqrt: rb1.effective_world_inv_inertia_sqrt,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            limits_impulse: limits_impulse * params.warmstart_coeff,
            limits_forcedirs,
            limits_rhs,
            basis1,
            inv_lhs,
            rhs,
            r1,
            r2,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        if let Some((limits_forcedir1, limits_forcedir2)) = self.limits_forcedirs {
            mj_lambda1.linear += limits_forcedir1 * (self.im1 * self.limits_impulse);
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * self.limits_impulse);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

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

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
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
            let new_impulse = (self.limits_impulse - lin_dvel / (self.im1 + self.im2)).max(0.0);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda1.linear += limits_forcedir1 * (self.im1 * dimpulse);
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * dimpulse);
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::PrismaticJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            revolute.limits_impulse = self.limits_impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct PrismaticVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    r2: Vector<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix2<Real>,
    #[cfg(feature = "dim2")]
    rhs: Vector2<Real>,
    #[cfg(feature = "dim2")]
    impulse: Vector2<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix5<Real>,
    #[cfg(feature = "dim3")]
    rhs: Vector5<Real>,
    #[cfg(feature = "dim3")]
    impulse: Vector5<Real>,

    limits_impulse: Real,
    limits_rhs: Real,

    axis2: Vector<Real>,
    #[cfg(feature = "dim2")]
    basis1: Vector2<Real>,
    #[cfg(feature = "dim3")]
    basis1: Matrix3x2<Real>,
    limits_forcedir2: Option<Vector<Real>>,

    im2: Real,
    ii2_sqrt: AngularInertia<Real>,
}

impl PrismaticVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &PrismaticJoint,
        flipped: bool,
    ) -> Self {
        let anchor2;
        let anchor1;
        let axis2;
        let axis1;
        let basis1;

        if flipped {
            anchor2 = rb2.position * cparams.local_anchor1;
            anchor1 = rb1.position * cparams.local_anchor2;
            axis2 = rb2.position * cparams.local_axis1;
            axis1 = rb1.position * cparams.local_axis2;
            #[cfg(feature = "dim2")]
            {
                basis1 = rb1.position * cparams.basis2[0];
            }
            #[cfg(feature = "dim3")]
            {
                basis1 = Matrix3x2::from_columns(&[
                    rb1.position * cparams.basis2[0],
                    rb1.position * cparams.basis2[1],
                ]);
            }
        } else {
            anchor2 = rb2.position * cparams.local_anchor2;
            anchor1 = rb1.position * cparams.local_anchor1;
            axis2 = rb2.position * cparams.local_axis2;
            axis1 = rb1.position * cparams.local_axis1;
            #[cfg(feature = "dim2")]
            {
                basis1 = rb1.position * cparams.basis1[0];
            }
            #[cfg(feature = "dim3")]
            {
                basis1 = Matrix3x2::from_columns(&[
                    rb1.position * cparams.basis1[0],
                    rb1.position * cparams.basis1[1],
                ]);
            }
        };

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

        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1 - rb1.world_com;
        let r2 = anchor2 - rb2.world_com;
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

        let anchor_linvel1 = rb1.linvel + rb1.angvel.gcross(r1);
        let anchor_linvel2 = rb2.linvel + rb2.angvel.gcross(r2);

        // NOTE: we don't use Cholesky in 2D because we only have a 2x2 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.inverse_unchecked().into_matrix();
        #[cfg(feature = "dim3")]
        let inv_lhs = Cholesky::new_unchecked(lhs).inverse();

        let lin_rhs = basis1.tr_mul(&(anchor_linvel2 - anchor_linvel1));
        let ang_rhs = rb2.angvel - rb1.angvel;

        #[cfg(feature = "dim2")]
        let rhs = Vector2::new(lin_rhs.x, ang_rhs);
        #[cfg(feature = "dim3")]
        let rhs = Vector5::new(lin_rhs.x, lin_rhs.y, ang_rhs.x, ang_rhs.y, ang_rhs.z);

        // Setup limit constraint.
        let mut limits_forcedir2 = None;
        let mut limits_rhs = 0.0;
        let mut limits_impulse = 0.0;

        if cparams.limits_enabled {
            let danchor = anchor2 - anchor1;
            let dist = danchor.dot(&axis1);

            // FIXME: we should allow both limits to be active at
            // the same time.
            // FIXME: allow predictive constraint activation.
            if dist < cparams.limits[0] {
                limits_forcedir2 = Some(axis2.into_inner());
                limits_rhs = anchor_linvel2.dot(&axis2) - anchor_linvel1.dot(&axis1);
                limits_impulse = cparams.limits_impulse;
            } else if dist > cparams.limits[1] {
                limits_forcedir2 = Some(-axis2.into_inner());
                limits_rhs = -anchor_linvel2.dot(&axis2) + anchor_linvel1.dot(&axis1);
                limits_impulse = cparams.limits_impulse;
            }
        }

        PrismaticVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            limits_impulse: limits_impulse * params.warmstart_coeff,
            basis1,
            inv_lhs,
            rhs,
            r2,
            axis2: axis2.into_inner(),
            limits_forcedir2,
            limits_rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis1 * self.impulse.fixed_rows::<LinImpulseDim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse.y;
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(2).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        if let Some(limits_forcedir2) = self.limits_forcedir2 {
            mj_lambda2.linear += limits_forcedir2 * (self.im2 * self.limits_impulse);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

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

        mj_lambda2.linear -= self.im2 * lin_impulse;
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
            let new_impulse = (self.limits_impulse - lin_dvel / self.im2).max(0.0);
            let dimpulse = new_impulse - self.limits_impulse;
            self.limits_impulse = new_impulse;

            mj_lambda2.linear += limits_forcedir2 * (self.im2 * dimpulse);
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::PrismaticJoint(revolute) = &mut joint.params {
            revolute.impulse = self.impulse;
            revolute.limits_impulse = self.limits_impulse;
        }
    }
}
