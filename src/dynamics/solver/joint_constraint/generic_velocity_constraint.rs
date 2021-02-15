use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    GenericJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
};
use crate::math::{AngularInertia, Dim, Isometry, Real, Rotation, SpacialVector, Vector, DIM};
use crate::na::UnitQuaternion;
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

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // TODO: replace by Cholesky?
    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,

    r1: Vector<Real>,
    r2: Vector<Real>,
    basis: Rotation<Real>,
    dependant_set_mask: SpacialVector<Real>,

    vel: GenericConstraintPart,
}

impl GenericVelocityConstraint {
    #[inline(always)]
    pub fn compute_delassus_matrix(
        im1: Real,
        im2: Real,
        ii1: AngularInertia<Real>,
        ii2: AngularInertia<Real>,
        r1: Vector<Real>,
        r2: Vector<Real>,
        basis: Rotation<Real>,
    ) -> Matrix6<Real> {
        let rotmat = basis.to_rotation_matrix().into_inner();
        let rmat1 = r1.gcross_matrix() * rotmat;
        let rmat2 = r2.gcross_matrix() * rotmat;

        #[cfg(feature = "dim3")]
        {
            let del00 =
                ii1.quadform(&rmat1).add_diagonal(im1) + ii2.quadform(&rmat2).add_diagonal(im2);
            let del10 =
                rotmat.transpose() * (ii1.transform_matrix(&rmat1) + ii2.transform_matrix(&rmat2));
            let del11 = (ii1 + ii2).quadform(&rotmat).into_matrix();

            // Note that Cholesky only reads the lower-triangular part of the matrix
            // so we don't need to fill del01.
            let mut del = Matrix6::zeros();
            del.fixed_slice_mut::<U3, U3>(0, 0)
                .copy_from(&del00.into_matrix());
            del.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&del10);
            del.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&del11);
            del
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            panic!("Take the rotmat into account.");
            let m11 = im1 + im2 + rmat1.x * rmat1.x * ii1 + rmat2.x * rmat2.x * ii2;
            let m12 = rmat1.x * rmat1.y * ii1 + rmat2.x * rmat2.y * ii2;
            let m22 = im1 + im2 + rmat1.y * rmat1.y * ii1 + rmat2.y * rmat2.y * ii2;
            let m13 = rmat1.x * ii1 + rmat2.x * ii2;
            let m23 = rmat1.y * ii1 + rmat2.y * ii2;
            let m33 = ii1 + ii2;
            Matrix3::new(m11, m12, m13, m12, m22, m23, m13, m23, m33)
        }
    }

    pub fn compute_velocity_error(
        min_velocity: &SpatialVector<Real>,
        max_velocity: &SpatialVector<Real>,
        r1: &Vector<Real>,
        r2: &Vector<Real>,
        basis: &Rotation<Real>,
        rb1: &RigidBody,
        rb2: &RigidBody,
    ) -> SpatialVector<Real> {
        let lin_dvel = -rb1.linvel - rb1.angvel.gcross(*r1) + rb2.linvel + rb2.angvel.gcross(*r2);
        let ang_dvel = -rb1.angvel + rb2.angvel;

        let lin_dvel2 = basis.inverse_transform_vector(&lin_dvel);
        let ang_dvel2 = basis.inverse_transform_vector(&ang_dvel);

        let min_linvel = min_velocity.xyz();
        let min_angvel = min_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let max_linvel = max_velocity.xyz();
        let max_angvel = max_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let lin_rhs = lin_dvel2 - lin_dvel2.sup(&min_linvel).inf(&max_linvel);
        let ang_rhs = ang_dvel2 - ang_dvel2.sup(&min_angvel).inf(&max_angvel);

        #[cfg(feature = "dim2")]
        return Vector3::new(lin_rhs.x, lin_rhs.y, ang_rhs);

        #[cfg(feature = "dim3")]
        return Vector6::new(
            lin_rhs.x, lin_rhs.y, lin_rhs.z, ang_rhs.x, ang_rhs.y, ang_rhs.z,
        );
    }

    pub fn compute_lin_position_error(
        anchor1: &Isometry<Real>,
        anchor2: &Isometry<Real>,
        basis: &Rotation<Real>,
        min_position: &Vector<Real>,
        max_position: &Vector<Real>,
    ) -> Vector<Real> {
        let dpos = anchor2.translation.vector - anchor1.translation.vector;
        let local_dpos = basis.inverse_transform_vector(&dpos);
        local_dpos - local_dpos.sup(min_position).inf(max_position)
    }

    pub fn compute_ang_position_error(
        anchor1: &Isometry<Real>,
        anchor2: &Isometry<Real>,
        basis: &Rotation<Real>,
        min_position: &Vector<Real>,
        max_position: &Vector<Real>,
    ) -> Vector<Real> {
        let drot = anchor2.rotation * anchor1.rotation.inverse();
        let local_drot_diff = basis.inverse_transform_vector(&drot.scaled_axis());

        let error = local_drot_diff - local_drot_diff.sup(min_position).inf(max_position);
        let error_code =
            (error[0] == 0.0) as usize + (error[1] == 0.0) as usize + (error[2] == 0.0) as usize;

        if error_code == 1 {
            // Align two axes.
            let constrained_axis = error.iamin();
            let axis1 = anchor1
                .rotation
                .to_rotation_matrix()
                .into_inner()
                .column(constrained_axis)
                .into_owned();
            let axis2 = anchor2
                .rotation
                .to_rotation_matrix()
                .into_inner()
                .column(constrained_axis)
                .into_owned();
            let rot_cross = UnitQuaternion::rotation_between(&axis1, &axis2)
                .unwrap_or(UnitQuaternion::identity());
            let local_drot_diff = basis.inverse_transform_vector(&rot_cross.scaled_axis());
            local_drot_diff - local_drot_diff.sup(min_position).inf(max_position)
        } else {
            error
        }
    }

    pub fn compute_position_error(
        joint: &GenericJoint,
        anchor1: &Isometry<Real>,
        anchor2: &Isometry<Real>,
        basis: &Rotation<Real>,
    ) -> SpatialVector<Real> {
        let delta_pos = Isometry::from_parts(
            anchor2.translation * anchor1.translation.inverse(),
            anchor2.rotation * anchor1.rotation.inverse(),
        );
        let lin_dpos = basis.inverse_transform_vector(&delta_pos.translation.vector);
        let ang_dpos = basis.inverse_transform_vector(&delta_pos.rotation.scaled_axis());

        let dpos = Vector6::new(
            lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
        );

        let error = dpos - dpos.sup(&joint.min_position).inf(&joint.max_position);
        let error_code =
            (error[3] == 0.0) as usize + (error[4] == 0.0) as usize + (error[5] == 0.0) as usize;

        match error_code {
            1 => {
                let constrained_axis = error.rows(3, 3).iamin();
                let axis1 = anchor1
                    .rotation
                    .to_rotation_matrix()
                    .into_inner()
                    .column(constrained_axis)
                    .into_owned();
                let axis2 = anchor2
                    .rotation
                    .to_rotation_matrix()
                    .into_inner()
                    .column(constrained_axis)
                    .into_owned();
                let rot_cross = UnitQuaternion::rotation_between(&axis1, &axis2)
                    .unwrap_or(UnitQuaternion::identity());
                let ang_dpos = basis.inverse_transform_vector(&rot_cross.scaled_axis());
                let dpos = Vector6::new(
                    lin_dpos.x, lin_dpos.y, lin_dpos.z, ang_dpos.x, ang_dpos.y, ang_dpos.z,
                );

                dpos - dpos.sup(&joint.min_position).inf(&joint.max_position)
            }
            _ => error,
        }
    }

    pub fn invert_partial_delassus_matrix(
        min_impulse: &Vector<Real>,
        max_impulse: &Vector<Real>,
        dependant_set_mask: &mut Vector<Real>,
        mut delassus: na::Matrix3<Real>,
    ) -> na::Matrix3<Real> {
        // Adjust the Delassus matrix to take force limits into account.
        // If a DoF has a force limit, then we need to make its
        // constraint independent from the others because otherwise
        // the force clamping will cause errors to propagate in the
        // other constraints.
        for i in 0..3 {
            if min_impulse[i] > -Real::MAX || max_impulse[i] < Real::MAX {
                let diag = delassus[(i, i)];
                delassus.column_mut(i).fill(0.0);
                delassus.row_mut(i).fill(0.0);
                delassus[(i, i)] = diag;
                dependant_set_mask[i] = 0.0;
            } else {
                dependant_set_mask[i] = 1.0;
            }
        }

        delassus.try_inverse().unwrap()
    }

    pub fn invert_delassus_matrix(
        min_impulse: &Vector6<Real>,
        max_impulse: &Vector6<Real>,
        dependant_set_mask: &mut Vector6<Real>,
        mut delassus: Matrix6<Real>,
    ) -> Matrix6<Real> {
        // Adjust the Delassus matrix to take force limits into account.
        // If a DoF has a force limit, then we need to make its
        // constraint independent from the others because otherwise
        // the force clamping will cause errors to propagate in the
        // other constraints.
        dependant_set_mask.fill(1.0);

        for i in 0..6 {
            if min_impulse[i] > -Real::MAX || max_impulse[i] < Real::MAX {
                let diag = delassus[(i, i)];
                delassus.column_mut(i).fill(0.0);
                delassus.row_mut(i).fill(0.0);
                delassus[(i, i)] = diag;
                dependant_set_mask[i] = 0.0;
            }
        }

        // NOTE: we don't use Cholesky in 2D because we only have a 3x3 matrix.
        #[cfg(feature = "dim2")]
        return delassus.try_inverse().expect("Singular system.");
        #[cfg(feature = "dim3")]
        return delassus.cholesky().expect("Singular system.").inverse();
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
        let basis = anchor1.rotation;
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1.translation.vector - rb1.world_com.coords;
        let r2 = anchor2.translation.vector - rb2.world_com.coords;
        let mut min_impulse = joint.min_impulse;
        let mut max_impulse = joint.max_impulse;
        let mut min_pos_impulse = joint.min_pos_impulse;
        let mut max_pos_impulse = joint.max_pos_impulse;
        let mut min_velocity = joint.min_velocity;
        let mut max_velocity = joint.max_velocity;
        let mut dependant_set_mask = SpacialVector::repeat(1.0);

        let pos_rhs = Self::compute_position_error(joint, &anchor1, &anchor2, &basis)
            * params.inv_dt()
            * params.joint_erp;

        for i in 0..6 {
            if pos_rhs[i] < 0.0 {
                min_impulse[i] = -Real::MAX;
                min_pos_impulse[i] = -Real::MAX;
                min_velocity[i] = 0.0;
            }
            if pos_rhs[i] > 0.0 {
                max_impulse[i] = Real::MAX;
                max_pos_impulse[i] = Real::MAX;
                max_velocity[i] = 0.0;
            }
        }

        let rhs =
            Self::compute_velocity_error(&min_velocity, &max_velocity, &r1, &r2, &basis, rb1, rb2);

        let mut delassus = Self::compute_delassus_matrix(im1, im2, ii1, ii2, r1, r2, basis);
        let inv_lhs = Self::invert_delassus_matrix(
            &min_impulse,
            &max_impulse,
            &mut dependant_set_mask,
            delassus,
        );

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&max_impulse)
            .sup(&min_impulse);

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
            inv_lhs,
            r1,
            r2,
            basis,
            dependant_set_mask,
            vel: GenericConstraintPart {
                impulse,
                min_impulse,
                max_impulse,
                rhs,
            },
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis * self.vel.impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.basis * self.vel.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.basis * self.vel.impulse.fixed_rows::<U3>(3).into_owned();

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

        self.vel.impulse = self.vel.solve(self, &mut mj_lambda1, &mut mj_lambda2);

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::GenericJoint(fixed) = &mut joint.params {
            fixed.impulse = self.vel.impulse;
        }
    }
}

#[derive(Debug)]
pub(crate) struct GenericVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    #[cfg(feature = "dim3")]
    inv_lhs: Matrix6<Real>, // TODO: replace by Cholesky?
    #[cfg(feature = "dim2")]
    inv_lhs: Matrix3<Real>,

    im2: Real,
    ii2: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
    r2: Vector<Real>,
    basis: Rotation<Real>,

    dependant_set_mask: SpacialVector<Real>,

    vel: GenericConstraintPart,
}

impl GenericVelocityGroundConstraint {
    #[inline(always)]
    pub fn compute_delassus_matrix(
        im2: Real,
        ii2: AngularInertia<Real>,
        r2: Vector<Real>,
        basis: Rotation<Real>,
    ) -> Matrix6<Real> {
        let rotmat2 = basis.to_rotation_matrix().into_inner();
        let rmat2 = r2.gcross_matrix() * rotmat2;

        #[cfg(feature = "dim3")]
        {
            let del00 = ii2.quadform(&rmat2).add_diagonal(im2);
            let del10 = rotmat2.transpose() * ii2.transform_matrix(&rmat2);
            let del11 = ii2.quadform(&rotmat2).into_matrix();

            // Note that Cholesky only reads the lower-triangular part of the matrix
            // so we don't need to fill lhs01.
            let mut del = Matrix6::zeros();
            del.fixed_slice_mut::<U3, U3>(0, 0)
                .copy_from(&del00.into_matrix());
            del.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&del10);
            del.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&del11);
            del
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            panic!("Properly handle the rotmat2");
            let m11 = im2 + rmat2.x * rmat2.x * ii2;
            let m12 = rmat2.x * rmat2.y * ii2;
            let m22 = im2 + rmat2.y * rmat2.y * ii2;
            let m13 = rmat2.x * ii2;
            let m23 = rmat2.y * ii2;
            let m33 = ii2;
            Matrix3::new(m11, m12, m13, m12, m22, m23, m13, m23, m33)
        }
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

        let basis = anchor2.rotation;
        let im2 = rb2.effective_inv_mass;
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let r1 = anchor1.translation.vector - rb1.world_com.coords;
        let r2 = anchor2.translation.vector - rb2.world_com.coords;
        let mut min_impulse = joint.min_impulse;
        let mut max_impulse = joint.max_impulse;
        let mut min_pos_impulse = joint.min_pos_impulse;
        let mut max_pos_impulse = joint.max_pos_impulse;
        let mut min_velocity = joint.min_velocity;
        let mut max_velocity = joint.max_velocity;
        let mut dependant_set_mask = SpacialVector::repeat(1.0);

        let pos_rhs =
            GenericVelocityConstraint::compute_position_error(joint, &anchor1, &anchor2, &basis)
                * params.inv_dt()
                * params.joint_erp;

        for i in 0..6 {
            if pos_rhs[i] < 0.0 {
                min_impulse[i] = -Real::MAX;
                min_pos_impulse[i] = -Real::MAX;
                min_velocity[i] = 0.0;
            }
            if pos_rhs[i] > 0.0 {
                max_impulse[i] = Real::MAX;
                max_pos_impulse[i] = Real::MAX;
                max_velocity[i] = 0.0;
            }
        }

        let rhs = GenericVelocityConstraint::compute_velocity_error(
            &min_velocity,
            &max_velocity,
            &r1,
            &r2,
            &basis,
            rb1,
            rb2,
        );

        let delassus = Self::compute_delassus_matrix(im2, ii2, r2, basis);
        let inv_lhs = GenericVelocityConstraint::invert_delassus_matrix(
            &min_impulse,
            &max_impulse,
            &mut dependant_set_mask,
            delassus,
        );

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&max_impulse)
            .sup(&min_impulse);

        GenericVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            inv_lhs,
            r2,
            basis,
            vel: GenericConstraintPart {
                impulse,
                min_impulse,
                max_impulse,
                rhs,
            },
            dependant_set_mask,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis * self.vel.impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = self.basis * self.vel.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.basis * self.vel.impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        self.vel.impulse = self.vel.solve_ground(self, &mut mj_lambda2);
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // TODO: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::GenericJoint(fixed) = &mut joint.params {
            fixed.impulse = self.vel.impulse;
        }
    }
}

#[derive(Debug)]
struct GenericConstraintPart {
    impulse: SpacialVector<Real>,
    max_impulse: SpatialVector<Real>,
    min_impulse: SpatialVector<Real>,

    #[cfg(feature = "dim3")]
    rhs: Vector6<Real>,
    #[cfg(feature = "dim2")]
    rhs: Vector3<Real>,
}

impl GenericConstraintPart {
    fn solve_ground(
        &self,
        parent: &GenericVelocityGroundConstraint,
        mj_lambda2: &mut DeltaVel<Real>,
    ) -> SpatialVector<Real> {
        let mut new_impulse = SpacialVector::zeros();

        for i in 0..3 {
            // Solve the independent 1D constraints.
            if parent.dependant_set_mask[i] == 0.0
                && (self.min_impulse[i] != 0.0 || self.max_impulse[i] != 0.0)
            {
                let constraint_axis = parent.basis * Vector::ith(i % 3, 1.0);
                let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

                let dvel = if i < 3 {
                    (mj_lambda2.linear + ang_vel2.gcross(parent.r2)).dot(&constraint_axis)
                        + self.rhs[i]
                } else {
                    ang_vel2.dot(&constraint_axis) + self.rhs[i]
                };

                new_impulse[i] = na::clamp(
                    self.impulse[i] + parent.inv_lhs[(i, i)] * dvel,
                    self.min_impulse[i],
                    self.max_impulse[i],
                );

                let effective_impulse = new_impulse[i] - self.impulse[i];
                let impulse = constraint_axis * effective_impulse;

                if i < 3 {
                    mj_lambda2.linear -= parent.im2 * impulse;
                    mj_lambda2.angular -=
                        parent.ii2_sqrt.transform_vector(parent.r2.gcross(impulse));
                } else {
                    mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(impulse);
                }
            }
        }

        let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let dlinvel = parent
            .basis
            .inverse_transform_vector(&(mj_lambda2.linear + ang_vel2.gcross(parent.r2)));
        let dangvel = parent.basis.inverse_transform_vector(&ang_vel2);

        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        new_impulse +=
            (self.impulse + parent.inv_lhs * dvel).component_mul(&parent.dependant_set_mask);
        let effective_impulse =
            (new_impulse - self.impulse).component_mul(&parent.dependant_set_mask);

        let lin_impulse = parent.basis * effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = parent.basis * effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = parent.basis * effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda2.linear -= parent.im2 * lin_impulse;
        mj_lambda2.angular -= parent
            .ii2_sqrt
            .transform_vector(ang_impulse + parent.r2.gcross(lin_impulse));

        for i in 3..6 {
            // Solve the independent 1D constraints.
            if parent.dependant_set_mask[i] == 0.0
                && (self.min_impulse[i] != 0.0 || self.max_impulse[i] != 0.0)
            {
                let constraint_axis = parent.basis * Vector::ith(i % 3, 1.0);
                let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

                let dvel = if i < 3 {
                    (mj_lambda2.linear + ang_vel2.gcross(parent.r2)).dot(&constraint_axis)
                        + self.rhs[i]
                } else {
                    ang_vel2.dot(&constraint_axis) + self.rhs[i]
                };

                let inv_lhs = parent.inv_lhs[(i, i)];

                new_impulse[i] = na::clamp(
                    self.impulse[i] + inv_lhs * dvel,
                    self.min_impulse[i],
                    self.max_impulse[i],
                );

                let effective_impulse = new_impulse[i] - self.impulse[i];
                let impulse = constraint_axis * effective_impulse;

                if i < 3 {
                    mj_lambda2.linear -= parent.im2 * impulse;
                    mj_lambda2.angular -=
                        parent.ii2_sqrt.transform_vector(parent.r2.gcross(impulse));
                } else {
                    mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(impulse);
                }
            }
        }

        new_impulse
    }

    fn solve(
        &self,
        parent: &GenericVelocityConstraint,
        mj_lambda1: &mut DeltaVel<Real>,
        mj_lambda2: &mut DeltaVel<Real>,
    ) -> SpatialVector<Real> {
        let mut new_impulse = SpacialVector::zeros();

        for i in 0..3 {
            // Solve the independent 1D constraints.
            if parent.dependant_set_mask[i] == 0.0
                && (self.min_impulse[i] != 0.0 || self.max_impulse[i] != 0.0)
            {
                let constraint_axis = parent.basis * Vector::ith(i % 3, 1.0);
                let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);
                let ang_vel1 = parent.ii1_sqrt.transform_vector(mj_lambda1.angular);

                let dvel = if i < 3 {
                    (mj_lambda2.linear + ang_vel2.gcross(parent.r2)
                        - mj_lambda1.linear
                        - ang_vel1.gcross(parent.r1))
                    .dot(&constraint_axis)
                        + self.rhs[i]
                } else {
                    (ang_vel2 - ang_vel1).dot(&constraint_axis) + self.rhs[i]
                };

                new_impulse[i] = na::clamp(
                    self.impulse[i] + parent.inv_lhs[(i, i)] * dvel,
                    self.min_impulse[i],
                    self.max_impulse[i],
                );

                let effective_impulse = new_impulse[i] - self.impulse[i];
                let impulse = constraint_axis * effective_impulse;

                if i < 3 {
                    mj_lambda1.linear += parent.im1 * impulse;
                    mj_lambda1.angular +=
                        parent.ii1_sqrt.transform_vector(parent.r1.gcross(impulse));
                    mj_lambda2.linear -= parent.im2 * impulse;
                    mj_lambda2.angular -=
                        parent.ii2_sqrt.transform_vector(parent.r2.gcross(impulse));
                } else {
                    mj_lambda1.angular += parent.ii1_sqrt.transform_vector(impulse);
                    mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(impulse);
                }
            }
        }

        let ang_vel1 = parent.ii1_sqrt.transform_vector(mj_lambda1.angular);
        let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

        let dlinvel = parent.basis.inverse_transform_vector(
            &(-mj_lambda1.linear - ang_vel1.gcross(parent.r1)
                + mj_lambda2.linear
                + ang_vel2.gcross(parent.r2)),
        );
        let dangvel = parent
            .basis
            .inverse_transform_vector(&(-ang_vel1 + ang_vel2));

        #[cfg(feature = "dim2")]
        let rhs = Vector3::new(dlinvel.x, dlinvel.y, dangvel) + self.rhs;
        #[cfg(feature = "dim3")]
        let dvel = Vector6::new(
            dlinvel.x, dlinvel.y, dlinvel.z, dangvel.x, dangvel.y, dangvel.z,
        ) + self.rhs;

        new_impulse +=
            (self.impulse + parent.inv_lhs * dvel).component_mul(&parent.dependant_set_mask);
        let effective_impulse =
            (new_impulse - self.impulse).component_mul(&parent.dependant_set_mask);

        let lin_impulse = parent.basis * effective_impulse.fixed_rows::<Dim>(0).into_owned();
        #[cfg(feature = "dim2")]
        let ang_impulse = parent.basis * effective_impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = parent.basis * effective_impulse.fixed_rows::<U3>(3).into_owned();

        mj_lambda1.linear += parent.im1 * lin_impulse;
        mj_lambda1.angular += parent
            .ii1_sqrt
            .transform_vector(ang_impulse + parent.r1.gcross(lin_impulse));

        mj_lambda2.linear -= parent.im2 * lin_impulse;
        mj_lambda2.angular -= parent
            .ii2_sqrt
            .transform_vector(ang_impulse + parent.r2.gcross(lin_impulse));

        for i in 3..6 {
            // Solve the independent 1D constraints.
            if parent.dependant_set_mask[i] == 0.0
                && (self.min_impulse[i] != 0.0 || self.max_impulse[i] != 0.0)
            {
                let constraint_axis = parent.basis * Vector::ith(i % 3, 1.0);
                let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);
                let ang_vel1 = parent.ii1_sqrt.transform_vector(mj_lambda1.angular);

                let dvel = if i < 3 {
                    (mj_lambda2.linear + ang_vel2.gcross(parent.r2)
                        - mj_lambda1.linear
                        - ang_vel1.gcross(parent.r1))
                    .dot(&constraint_axis)
                        + self.rhs[i]
                } else {
                    (ang_vel2 - ang_vel1).dot(&constraint_axis) + self.rhs[i]
                };

                let inv_lhs = parent.inv_lhs[(i, i)];

                new_impulse[i] = na::clamp(
                    self.impulse[i] + inv_lhs * dvel,
                    self.min_impulse[i],
                    self.max_impulse[i],
                );

                let effective_impulse = new_impulse[i] - self.impulse[i];
                let impulse = constraint_axis * effective_impulse;

                if i < 3 {
                    mj_lambda1.linear += parent.im1 * impulse;
                    mj_lambda1.angular +=
                        parent.ii1_sqrt.transform_vector(parent.r1.gcross(impulse));
                    mj_lambda2.linear -= parent.im2 * impulse;
                    mj_lambda2.angular -=
                        parent.ii2_sqrt.transform_vector(parent.r2.gcross(impulse));
                } else {
                    mj_lambda1.angular += parent.ii1_sqrt.transform_vector(impulse);
                    mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(impulse);
                }
            }
        }

        new_impulse
    }
}
