use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    FixedJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::math::{AngularInertia, Real, SpacialVector, Vector, DIM};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim2")]
use na::{Matrix3, Vector3};
#[cfg(feature = "dim3")]
use na::{Matrix6, Vector6};

#[derive(Debug)]
pub(crate) struct FixedVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    impulse: SpacialVector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim2")]
    rhs: Vector3<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,

    r1: Vector<Real>,
    r2: Vector<Real>,
}

impl FixedVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        rb2: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        cparams: &FixedJoint,
    ) -> Self {
        let (poss1, vels1, mprops1, ids1) = rb1;
        let (poss2, vels2, mprops2, ids2) = rb2;

        let anchor1 = poss1.position * cparams.local_frame1;
        let anchor2 = poss2.position * cparams.local_frame2;
        let im1 = mprops1.effective_inv_mass;
        let im2 = mprops2.effective_inv_mass;
        let ii1 = mprops1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = mprops2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1.translation.vector - mprops1.world_com.coords;
        let r2 = anchor2.translation.vector - mprops2.world_com.coords;
        let rmat1 = r1.gcross_matrix();
        let rmat2 = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D
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
            lhs = Matrix3::new(m11, m12, m13, m12, m22, m23, m13, m23, m33)
        }

        // NOTE: we don't use cholesky in 2D because we only have a 3x3 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.try_inverse().expect("Singular system.");
        #[cfg(feature = "dim3")]
        let inv_lhs = lhs.cholesky().expect("Singular system.").inverse();

        let lin_dvel =
            -vels1.linvel - vels1.angvel.gcross(r1) + vels2.linvel + vels2.angvel.gcross(r2);
        let ang_dvel = -vels1.angvel + vels2.angvel;

        #[cfg(feature = "dim2")]
        let mut rhs =
            Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel) * params.velocity_solve_fraction;

        #[cfg(feature = "dim3")]
        let mut rhs = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        ) * params.velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let lin_err = anchor2.translation.vector - anchor1.translation.vector;
            let ang_err = anchor2.rotation * anchor1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                let ang_err = ang_err.angle();
                rhs += Vector3::new(lin_err.x, lin_err.y, ang_err) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = ang_err.scaled_axis();
                rhs += Vector6::new(
                    lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y, ang_err.z,
                ) * velocity_based_erp_inv_dt;
            }
        }

        FixedVelocityConstraint {
            joint_id,
            mj_lambda1: ids1.active_set_offset,
            mj_lambda2: ids2.active_set_offset,
            im1,
            im2,
            ii1,
            ii2,
            ii1_sqrt: mprops1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: mprops2.effective_world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            inv_lhs,
            r1,
            r2,
            rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

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

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::FixedJoint(fixed) = &mut joint.params {
            fixed.impulse = self.impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct FixedVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    impulse: SpacialVector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim2")]
    rhs: Vector3<Real>,

    im2: Real,
    ii2: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
    r2: Vector<Real>,
}

impl FixedVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: (&RigidBodyPosition, &RigidBodyVelocity, &RigidBodyMassProps),
        rb2: (
            &RigidBodyPosition,
            &RigidBodyVelocity,
            &RigidBodyMassProps,
            &RigidBodyIds,
        ),
        cparams: &FixedJoint,
        flipped: bool,
    ) -> Self {
        let (poss1, vels1, mprops1) = rb1;
        let (poss2, vels2, mprops2, ids2) = rb2;

        let (anchor1, anchor2) = if flipped {
            (
                poss1.position * cparams.local_frame2,
                poss2.position * cparams.local_frame1,
            )
        } else {
            (
                poss1.position * cparams.local_frame1,
                poss2.position * cparams.local_frame2,
            )
        };

        let r1 = anchor1.translation.vector - mprops1.world_com.coords;

        let im2 = mprops2.effective_inv_mass;
        let ii2 = mprops2.effective_world_inv_inertia_sqrt.squared();
        let r2 = anchor2.translation.vector - mprops2.world_com.coords;
        let rmat2 = r2.gcross_matrix();

        #[allow(unused_mut)] // For 2D.
        let mut lhs;

        #[cfg(feature = "dim3")]
        {
            let lhs00 = ii2.quadform(&rmat2).add_diagonal(im2);
            let lhs10 = ii2.transform_matrix(&rmat2);
            let lhs11 = ii2.into_matrix();

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
            let m11 = im2 + rmat2.x * rmat2.x * ii2;
            let m12 = rmat2.x * rmat2.y * ii2;
            let m22 = im2 + rmat2.y * rmat2.y * ii2;
            let m13 = rmat2.x * ii2;
            let m23 = rmat2.y * ii2;
            let m33 = ii2;
            lhs = Matrix3::new(m11, m12, m13, m12, m22, m23, m13, m23, m33)
        }

        #[cfg(feature = "dim2")]
        let inv_lhs = lhs.try_inverse().expect("Singular system.");
        #[cfg(feature = "dim3")]
        let inv_lhs = lhs.cholesky().expect("Singular system.").inverse();

        let lin_dvel =
            vels2.linvel + vels2.angvel.gcross(r2) - vels1.linvel - vels1.angvel.gcross(r1);
        let ang_dvel = vels2.angvel - vels1.angvel;

        #[cfg(feature = "dim2")]
        let mut rhs =
            Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel) * params.velocity_solve_fraction;
        #[cfg(feature = "dim3")]
        let mut rhs = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        ) * params.velocity_solve_fraction;

        let velocity_based_erp_inv_dt = params.velocity_based_erp_inv_dt();
        if velocity_based_erp_inv_dt != 0.0 {
            let lin_err = anchor2.translation.vector - anchor1.translation.vector;
            let ang_err = anchor2.rotation * anchor1.rotation.inverse();

            #[cfg(feature = "dim2")]
            {
                let ang_err = ang_err.angle();
                rhs += Vector3::new(lin_err.x, lin_err.y, ang_err) * velocity_based_erp_inv_dt;
            }

            #[cfg(feature = "dim3")]
            {
                let ang_err = ang_err.scaled_axis();
                rhs += Vector6::new(
                    lin_err.x, lin_err.y, lin_err.z, ang_err.x, ang_err.y, ang_err.z,
                ) * velocity_based_erp_inv_dt;
            }
        }

        FixedVelocityGroundConstraint {
            joint_id,
            mj_lambda2: ids2.active_set_offset,
            im2,
            ii2,
            ii2_sqrt: mprops2.effective_world_inv_inertia_sqrt,
            impulse: cparams.impulse * params.warmstart_coeff,
            inv_lhs,
            r2,
            rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<DIM>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<3>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

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

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::FixedJoint(fixed) = &mut joint.params {
            fixed.impulse = self.impulse;
        }
    }
}
