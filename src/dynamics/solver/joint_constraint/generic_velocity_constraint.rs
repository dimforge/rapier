use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    GenericJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
};
use crate::math::{AngularInertia, Dim, Isometry, Real, SpacialVector, Vector, DIM};
use crate::parry::math::{AngDim, SpatialVector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim2")]
use na::{Matrix3, Vector3};
#[cfg(feature = "dim3")]
use na::{Matrix6, Vector6, U3};

#[derive(Debug)]
pub(crate) struct GenericVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    impulse: SpacialVector<Real>,
    pos_impulse: SpacialVector<Real>,

    max_positive_impulse: SpatialVector<Real>,
    max_negative_impulse: SpatialVector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim2")]
    rhs: Vector3<Real>,

    pos_rhs: Vector6<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,

    r1: Vector<Real>,
    r2: Vector<Real>,
}

impl GenericVelocityConstraint {
    #[inline(always)]
    pub fn compute_mass_matrix(
        joint: &GenericJoint,
        im1: Real,
        im2: Real,
        ii1: AngularInertia<Real>,
        ii2: AngularInertia<Real>,
        r1: Vector<Real>,
        r2: Vector<Real>,
        velocity_solver: bool,
    ) -> Matrix6<Real> {
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
            lhs.fixed_slice_mut::<U3, U3>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&lhs11);

            // Adjust the mass matrix to take force limits into account.
            // If a DoF has a force limit, then we need to make its
            // constraint independent from the others because otherwise
            // the force clamping will cause errors to propagate in the
            // other constraints.
            if velocity_solver {
                for i in 0..6 {
                    if joint.max_negative_impulse[i] > -Real::MAX
                        || joint.max_positive_impulse[i] < Real::MAX
                    {
                        let diag = lhs[(i, i)];
                        lhs.column_mut(i).fill(0.0);
                        lhs.row_mut(i).fill(0.0);
                        lhs[(i, i)] = diag;
                    }
                }
            }
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

        // NOTE: we don't use Cholesky in 2D because we only have a 3x3 matrix
        // for which a textbook inverse is still efficient.
        #[cfg(feature = "dim2")]
        return lhs.try_inverse().expect("Singular system.");
        #[cfg(feature = "dim3")]
        return lhs.cholesky().expect("Singular system.").inverse();
    }

    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &GenericJoint,
    ) -> Self {
        let anchor1 = rb1.position * joint.local_anchor1;
        let anchor2 = rb2.position * joint.local_anchor2;
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1.translation.vector - rb1.world_com.coords;
        let r2 = anchor2.translation.vector - rb2.world_com.coords;

        let lin_dvel = -rb1.linvel - rb1.angvel.gcross(r1) + rb2.linvel + rb2.angvel.gcross(r2);
        let ang_dvel = -rb1.angvel + rb2.angvel;

        let inv_lhs = Self::compute_mass_matrix(joint, im1, im2, ii1, ii2, r1, r2, true);

        #[cfg(feature = "dim2")]
        let dvel = Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel);

        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        );

        let target_linvel = anchor2 * joint.target_velocity.xyz();
        let target_angvel = anchor2 * joint.target_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let target_vel = Vector6::new(
            target_linvel.x,
            target_linvel.y,
            target_linvel.z,
            target_angvel.x,
            target_angvel.y,
            target_angvel.z,
        );

        let rhs = dvel - dvel.sup(&target_vel).inf(&target_vel);

        let delta_pos = Isometry::from_parts(
            anchor2.translation * anchor1.translation.inverse(),
            anchor2.rotation * anchor1.rotation.inverse(),
        );
        let lin_dpos = delta_pos.translation.vector;
        let ang_dpos = delta_pos.rotation.scaled_axis();
        let dpos = Vector6::new(
            lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
        );
        let err = dpos - dpos.sup(&joint.min_position).inf(&joint.max_position);
        let pos_rhs = err * params.inv_dt() * params.joint_erp;

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&joint.max_positive_impulse)
            .sup(&joint.max_negative_impulse);

        GenericVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            im2,
            ii1,
            ii2,
            ii1_sqrt: rb1.effective_world_inv_inertia_sqrt,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse,
            pos_impulse: na::zero(),
            max_positive_impulse: joint.max_positive_impulse,
            max_negative_impulse: joint.max_negative_impulse,
            inv_lhs,
            r1,
            r2,
            rhs,
            pos_rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(3).into_owned();

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
        return;
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
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let new_impulse = (self.impulse + self.inv_lhs * dvel)
            .sup(&self.max_negative_impulse)
            .inf(&self.max_positive_impulse);
        let effective_impulse = new_impulse - self.impulse;
        self.impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

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

    pub fn solve2(
        &mut self,
        mj_lambdas: &mut [DeltaVel<Real>],
        mj_lambdas_pos: &mut [DeltaVel<Real>],
    ) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        let mut mj_lambda_pos1 = mj_lambdas_pos[self.mj_lambda1 as usize];
        let mut mj_lambda_pos2 = mj_lambdas_pos[self.mj_lambda2 as usize];

        /*
         * Solve velocity.
         */
        let ang_vel1 = self.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let dlinvel = -mj_lambda1.linear - ang_vel1.gcross(self.r1)
            + mj_lambda2.linear
            + ang_vel2.gcross(self.r2);
        let dangvel = -ang_vel1 + ang_vel2;

        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let new_impulse = (self.impulse + self.inv_lhs * dvel)
            .sup(&self.max_negative_impulse)
            .inf(&self.max_positive_impulse);
        let effective_impulse = new_impulse - self.impulse;
        self.impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda1.linear += self.im1 * lin_impulse;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Solve positions.
         */

        let ang_pos1 = self.ii1_sqrt.transform_vector(mj_lambda_pos1.angular);
        let ang_pos2 = self.ii2_sqrt.transform_vector(mj_lambda_pos2.angular);

        let dlinpos = -mj_lambda_pos1.linear - ang_pos1.gcross(self.r1)
            + mj_lambda_pos2.linear
            + ang_pos2.gcross(self.r2);
        let dangpos = -ang_pos1 + ang_pos2;

        #[cfg(feature = "dim3")]
        let dpos = Vector6::new(
            dlinpos.x, dlinpos.y, dlinpos.z, dangpos.x, dangpos.y, dangpos.z,
        ) + self.pos_rhs;

        let new_impulse = self.pos_impulse + self.inv_lhs * dpos;
        let effective_impulse = new_impulse - self.pos_impulse;
        self.pos_impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda_pos1.linear += self.im1 * lin_impulse;
        mj_lambda_pos1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse + self.r1.gcross(lin_impulse));

        mj_lambda_pos2.linear -= self.im2 * lin_impulse;
        mj_lambda_pos2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
        mj_lambdas_pos[self.mj_lambda1 as usize] = mj_lambda_pos1;
        mj_lambdas_pos[self.mj_lambda2 as usize] = mj_lambda_pos2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::GenericJoint(fixed) = &mut joint.params {
            fixed.impulse = self.impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct GenericVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    impulse: SpacialVector<Real>,
    pos_impulse: SpacialVector<Real>,

    max_positive_impulse: SpatialVector<Real>,
    max_negative_impulse: SpatialVector<Real>,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim3")]
    rhs: Vector6<Real>,

    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>, // FIXME: replace by Cholesky.
    #[cfg(feature = "dim2")]
    rhs: Vector3<Real>,

    pos_rhs: Vector6<Real>,

    im2: Real,
    ii2: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
    r2: Vector<Real>,
}

impl GenericVelocityGroundConstraint {
    #[inline(always)]
    pub fn compute_mass_matrix(
        joint: &GenericJoint,
        im2: Real,
        ii2: AngularInertia<Real>,
        r2: Vector<Real>,
        velocity_solver: bool,
    ) -> Matrix6<Real> {
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
            lhs.fixed_slice_mut::<U3, U3>(0, 0)
                .copy_from(&lhs00.into_matrix());
            lhs.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&lhs10);
            lhs.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&lhs11);

            // Adjust the mass matrix to take force limits into account.
            // If a DoF has a force limit, then we need to make its
            // constraint independent from the others because otherwise
            // the force clamping will cause errors to propagate in the
            // other constraints.
            if velocity_solver {
                for i in 0..6 {
                    if joint.max_negative_impulse[i] > -Real::MAX
                        || joint.max_positive_impulse[i] < Real::MAX
                    {
                        let diag = lhs[(i, i)];
                        lhs.column_mut(i).fill(0.0);
                        lhs.row_mut(i).fill(0.0);
                        lhs[(i, i)] = diag;
                    }
                }
            }
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
        return lhs.try_inverse().expect("Singular system.");
        #[cfg(feature = "dim3")]
        return lhs.cholesky().expect("Singular system.").inverse();
    }

    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &GenericJoint,
        flipped: bool,
    ) -> Self {
        let (anchor1, anchor2) = if flipped {
            (
                rb1.position * joint.local_anchor2,
                rb2.position * joint.local_anchor1,
            )
        } else {
            (
                rb1.position * joint.local_anchor1,
                rb2.position * joint.local_anchor2,
            )
        };

        let r1 = anchor1.translation.vector - rb1.world_com.coords;
        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r2 = anchor2.translation.vector - rb2.world_com.coords;

        let inv_lhs = Self::compute_mass_matrix(joint, im2, ii2, r2, true);

        let lin_dvel = rb2.linvel + rb2.angvel.gcross(r2) - rb1.linvel - rb1.angvel.gcross(r1);
        let ang_dvel = rb2.angvel - rb1.angvel;

        #[cfg(feature = "dim2")]
        let dvel = Vector3::new(lin_dvel.x, lin_dvel.y, ang_dvel);
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            lin_dvel.x, lin_dvel.y, lin_dvel.z, ang_dvel.x, ang_dvel.y, ang_dvel.z,
        );
        let target_linvel = anchor2 * joint.target_velocity.xyz();
        let target_angvel = anchor2 * joint.target_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let target_vel = Vector6::new(
            target_linvel.x,
            target_linvel.y,
            target_linvel.z,
            target_angvel.x,
            target_angvel.y,
            target_angvel.z,
        );

        let mut rhs = dvel - dvel.sup(&target_vel).inf(&target_vel);

        let delta_pos = Isometry::from_parts(
            anchor2.translation * anchor1.translation.inverse(),
            anchor2.rotation * anchor1.rotation.inverse(),
        );
        let lin_dpos = delta_pos.translation.vector;
        let ang_dpos = delta_pos.rotation.scaled_axis();
        let dpos = Vector6::new(
            lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
        );
        let err = dpos - dpos.sup(&joint.min_position).inf(&joint.max_position);
        let pos_rhs = err * params.inv_dt() * params.joint_erp;

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&joint.max_positive_impulse)
            .sup(&joint.max_negative_impulse);

        GenericVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            impulse,
            pos_impulse: na::zero(),
            max_positive_impulse: joint.max_positive_impulse,
            max_negative_impulse: joint.max_negative_impulse,
            inv_lhs,
            r2,
            rhs,
            pos_rhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        return;
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let dlinvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let dangvel = ang_vel2;
        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let new_impulse = (self.impulse + self.inv_lhs * dvel)
            .sup(&self.max_negative_impulse)
            .inf(&self.max_positive_impulse);
        let effective_impulse = new_impulse - self.impulse;
        self.impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve2(
        &mut self,
        mj_lambdas: &mut [DeltaVel<Real>],
        mj_lambdas_pos: &mut [DeltaVel<Real>],
    ) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        let mut mj_lambda_pos2 = mj_lambdas_pos[self.mj_lambda2 as usize];

        /*
         * Solve velocities.
         */
        let ang_vel2 = self.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let dlinvel = mj_lambda2.linear + ang_vel2.gcross(self.r2);
        let dangvel = ang_vel2;
        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        let new_impulse = (self.impulse + self.inv_lhs * dvel)
            .sup(&self.max_negative_impulse)
            .inf(&self.max_positive_impulse);
        let effective_impulse = new_impulse - self.impulse;
        self.impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        /*
         * Solve positions.
         */
        let ang_pos2 = self.ii2_sqrt.transform_vector(mj_lambda_pos2.angular);

        let dlinpos = mj_lambda_pos2.linear + ang_pos2.gcross(self.r2);
        let dangpos = ang_pos2;
        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinpos.x, dlinpos.y, dangpos) + self.rhs;
        #[cfg(feature = "dim3")]
        let dpos = Vector6::new(
            dlinpos.x, dlinpos.y, dlinpos.z, dangpos.x, dangpos.y, dangpos.z,
        ) + self.pos_rhs;

        let new_impulse = self.pos_impulse + self.inv_lhs * dpos;
        let effective_impulse = new_impulse - self.pos_impulse;
        self.pos_impulse = new_impulse;

        let lin_impulse = effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda_pos2.linear -= self.im2 * lin_impulse;
        mj_lambda_pos2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
        mj_lambdas_pos[self.mj_lambda2 as usize] = mj_lambda_pos2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::GenericJoint(fixed) = &mut joint.params {
            fixed.impulse = self.impulse;
        }
    }
}
