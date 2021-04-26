use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    GenericJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
};
use crate::math::{AngularInertia, Dim, Isometry, Real, Rotation, SpacialVector, Vector, DIM};
use crate::na::UnitQuaternion;
use crate::parry::math::{AngDim, SpatialVector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
#[cfg(feature = "dim3")]
use na::{Matrix3, Matrix6, Vector3, Vector6, U3};
#[cfg(feature = "dim2")]
use na::{Matrix3, Vector3};

#[derive(Debug)]
pub(crate) struct GenericVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    inv_lhs_lin: Matrix3<Real>,
    inv_lhs_ang: Matrix3<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    ii1_sqrt: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,

    r1: Vector<Real>,
    r2: Vector<Real>,
    basis1: Rotation<Real>,
    basis2: Rotation<Real>,
    dependant_set_mask: SpacialVector<Real>,

    vel: GenericConstraintPart,
}

impl GenericVelocityConstraint {
    pub fn compute_velocity_error(
        min_velocity: &SpatialVector<Real>,
        max_velocity: &SpatialVector<Real>,
        r1: &Vector<Real>,
        r2: &Vector<Real>,
        basis1: &Rotation<Real>,
        basis2: &Rotation<Real>,
        rb1: &RigidBody,
        rb2: &RigidBody,
    ) -> SpatialVector<Real> {
        let lin_dvel = basis1.inverse_transform_vector(&(-rb1.linvel() - rb1.angvel().gcross(*r1)))
            + basis2.inverse_transform_vector(&(rb2.linvel() + rb2.angvel().gcross(*r2)));
        let ang_dvel = basis1.inverse_transform_vector(&-rb1.angvel)
            + basis2.inverse_transform_vector(&rb2.angvel);

        let min_linvel = min_velocity.xyz();
        let min_angvel = min_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let max_linvel = max_velocity.xyz();
        let max_angvel = max_velocity.fixed_rows::<AngDim>(DIM).into_owned();
        let lin_rhs = lin_dvel - lin_dvel.sup(&min_linvel).inf(&max_linvel);
        let ang_rhs = ang_dvel - ang_dvel.sup(&min_angvel).inf(&max_angvel);

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

    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &GenericJoint,
    ) -> Self {
        let anchor1 = rb1.position() * joint.local_anchor1;
        let anchor2 = rb2.position() * joint.local_anchor2;
        let basis1 = anchor1.rotation;
        let basis2 = anchor2.rotation;
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

        let pos_rhs = Self::compute_position_error(joint, &anchor1, &anchor2, &basis1)
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

        let rhs = Self::compute_velocity_error(
            &min_velocity,
            &max_velocity,
            &r1,
            &r2,
            &basis1,
            &basis2,
            rb1,
            rb2,
        );
        let rhs_lin = rhs.xyz();
        let rhs_ang = rhs.fixed_rows::<Dim>(DIM).into();

        // TODO: we should keep the SdpMatrix3 type.
        let rotmat1 = basis1.to_rotation_matrix().into_inner();
        let rotmat2 = basis2.to_rotation_matrix().into_inner();
        let rmat1 = r1.gcross_matrix() * rotmat1;
        let rmat2 = r2.gcross_matrix() * rotmat2;
        let delassus00 = (ii1.quadform(&rmat1).add_diagonal(im1)
            + ii2.quadform(&rmat2).add_diagonal(im2))
        .into_matrix();
        let delassus11 = (ii1.quadform(&rotmat1) + ii2.quadform(&rotmat2)).into_matrix();

        let inv_lhs_lin = GenericVelocityConstraint::invert_partial_delassus_matrix(
            &min_pos_impulse.xyz(),
            &max_pos_impulse.xyz(),
            &mut Vector3::zeros(),
            delassus00,
        );
        let inv_lhs_ang = GenericVelocityConstraint::invert_partial_delassus_matrix(
            &min_pos_impulse.fixed_rows::<Dim>(DIM).into_owned(),
            &max_pos_impulse.fixed_rows::<Dim>(DIM).into_owned(),
            &mut Vector3::zeros(),
            delassus11,
        );

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&max_impulse)
            .sup(&min_impulse);

        let lin_impulse = impulse.xyz();
        let ang_impulse = impulse.fixed_rows::<Dim>(DIM).into_owned();
        let min_lin_impulse = min_impulse.xyz();
        let min_ang_impulse = min_impulse.fixed_rows::<Dim>(DIM).into_owned();
        let max_lin_impulse = max_impulse.xyz();
        let max_ang_impulse = max_impulse.fixed_rows::<Dim>(DIM).into_owned();

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
            inv_lhs_lin,
            inv_lhs_ang,
            r1,
            r2,
            basis1,
            basis2,
            dependant_set_mask,
            vel: GenericConstraintPart {
                lin_impulse,
                ang_impulse,
                min_lin_impulse,
                min_ang_impulse,
                max_lin_impulse,
                max_ang_impulse,
                rhs_lin,
                rhs_ang,
            },
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse1 = self.basis1 * self.vel.lin_impulse;
        let ang_impulse1 = self.basis1 * self.vel.ang_impulse;
        let lin_impulse2 = self.basis2 * self.vel.lin_impulse;
        let ang_impulse2 = self.basis2 * self.vel.ang_impulse;

        mj_lambda1.linear += self.im1 * lin_impulse1;
        mj_lambda1.angular += self
            .ii1_sqrt
            .transform_vector(ang_impulse1 + self.r1.gcross(lin_impulse1));

        mj_lambda2.linear -= self.im2 * lin_impulse2;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse2 + self.r2.gcross(lin_impulse2));

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let (lin_imp, ang_imp) = self.vel.solve(self, &mut mj_lambda1, &mut mj_lambda2);
        self.vel.lin_impulse = lin_imp;
        self.vel.ang_impulse = ang_imp;

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        match &mut joint.params {
            JointParams::GenericJoint(out) => {
                out.impulse[0] = self.vel.lin_impulse.x;
                out.impulse[1] = self.vel.lin_impulse.y;
                out.impulse[2] = self.vel.lin_impulse.z;
                out.impulse[3] = self.vel.ang_impulse.x;
                out.impulse[4] = self.vel.ang_impulse.y;
                out.impulse[5] = self.vel.ang_impulse.z;
            }
            JointParams::RevoluteJoint(out) => {
                out.impulse[0] = self.vel.lin_impulse.x;
                out.impulse[1] = self.vel.lin_impulse.y;
                out.impulse[2] = self.vel.lin_impulse.z;
                out.motor_impulse = self.vel.ang_impulse.x;
                out.impulse[3] = self.vel.ang_impulse.y;
                out.impulse[4] = self.vel.ang_impulse.z;
            }
            _ => unimplemented!(),
        }
    }
}

#[derive(Debug)]
pub(crate) struct GenericVelocityGroundConstraint {
    mj_lambda2: usize,

    joint_id: JointIndex,

    inv_lhs_lin: Matrix3<Real>,
    inv_lhs_ang: Matrix3<Real>,

    im2: Real,
    ii2: AngularInertia<Real>,
    ii2_sqrt: AngularInertia<Real>,
    r2: Vector<Real>,
    basis: Rotation<Real>,

    dependant_set_mask: SpacialVector<Real>,

    vel: GenericConstraintPart,
}

impl GenericVelocityGroundConstraint {
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
                rb1.position() * joint.local_anchor2,
                rb2.position() * joint.local_anchor1,
            )
        } else {
            (
                rb1.position() * joint.local_anchor1,
                rb2.position() * joint.local_anchor2,
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
            &basis,
            rb1,
            rb2,
        );
        let rhs_lin = rhs.xyz();
        let rhs_ang = rhs.fixed_rows::<Dim>(DIM).into_owned();

        // TODO: we should keep the SdpMatrix3 type.
        let rotmat = basis.to_rotation_matrix().into_inner();
        let rmat2 = r2.gcross_matrix() * rotmat;
        let delassus00 = ii2.quadform(&rmat2).add_diagonal(im2).into_matrix();
        let delassus11 = ii2.quadform(&rotmat).into_matrix();

        let inv_lhs_lin = GenericVelocityConstraint::invert_partial_delassus_matrix(
            &min_pos_impulse.xyz(),
            &max_pos_impulse.xyz(),
            &mut Vector3::zeros(),
            delassus00,
        );
        let inv_lhs_ang = GenericVelocityConstraint::invert_partial_delassus_matrix(
            &min_pos_impulse.fixed_rows::<Dim>(DIM).into_owned(),
            &max_pos_impulse.fixed_rows::<Dim>(DIM).into_owned(),
            &mut Vector3::zeros(),
            delassus11,
        );

        let impulse = (joint.impulse * params.warmstart_coeff)
            .inf(&max_impulse)
            .sup(&min_impulse);

        let lin_impulse = impulse.xyz();
        let ang_impulse = impulse.fixed_rows::<Dim>(DIM).into_owned();
        let min_lin_impulse = min_impulse.xyz();
        let min_ang_impulse = min_impulse.fixed_rows::<Dim>(DIM).into_owned();
        let max_lin_impulse = max_impulse.xyz();
        let max_ang_impulse = max_impulse.fixed_rows::<Dim>(DIM).into_owned();

        GenericVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            ii2,
            ii2_sqrt: rb2.effective_world_inv_inertia_sqrt,
            inv_lhs_lin,
            inv_lhs_ang,
            r2,
            basis,
            vel: GenericConstraintPart {
                lin_impulse,
                ang_impulse,
                min_lin_impulse,
                min_ang_impulse,
                max_lin_impulse,
                max_ang_impulse,
                rhs_lin,
                rhs_ang,
            },
            dependant_set_mask,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let lin_impulse = self.basis * self.vel.lin_impulse;
        #[cfg(feature = "dim2")]
        let ang_impulse = self.basis * self.vel.impulse[2];
        #[cfg(feature = "dim3")]
        let ang_impulse = self.basis * self.vel.ang_impulse;

        mj_lambda2.linear -= self.im2 * lin_impulse;
        mj_lambda2.angular -= self
            .ii2_sqrt
            .transform_vector(ang_impulse + self.r2.gcross(lin_impulse));

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let (lin_imp, ang_imp) = self.vel.solve_ground(self, &mut mj_lambda2);
        self.vel.lin_impulse = lin_imp;
        self.vel.ang_impulse = ang_imp;

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // TODO: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        match &mut joint.params {
            JointParams::GenericJoint(out) => {
                out.impulse[0] = self.vel.lin_impulse.x;
                out.impulse[1] = self.vel.lin_impulse.y;
                out.impulse[2] = self.vel.lin_impulse.z;
                out.impulse[3] = self.vel.ang_impulse.x;
                out.impulse[4] = self.vel.ang_impulse.y;
                out.impulse[5] = self.vel.ang_impulse.z;
            }
            JointParams::RevoluteJoint(out) => {
                out.impulse[0] = self.vel.lin_impulse.x;
                out.impulse[1] = self.vel.lin_impulse.y;
                out.impulse[2] = self.vel.lin_impulse.z;
                out.motor_impulse = self.vel.ang_impulse.x;
                out.impulse[3] = self.vel.ang_impulse.y;
                out.impulse[4] = self.vel.ang_impulse.z;
            }
            _ => unimplemented!(),
        }
    }
}

#[derive(Debug)]
struct GenericConstraintPart {
    lin_impulse: Vector3<Real>,
    max_lin_impulse: Vector3<Real>,
    min_lin_impulse: Vector3<Real>,
    rhs_lin: Vector3<Real>,

    ang_impulse: Vector3<Real>,
    max_ang_impulse: Vector3<Real>,
    min_ang_impulse: Vector3<Real>,
    rhs_ang: Vector3<Real>,
}

impl GenericConstraintPart {
    fn solve(
        &self,
        parent: &GenericVelocityConstraint,
        mj_lambda1: &mut DeltaVel<Real>,
        mj_lambda2: &mut DeltaVel<Real>,
    ) -> (Vector3<Real>, Vector3<Real>) {
        let new_lin_impulse;
        let new_ang_impulse;

        /*
         *
         * Solve translations.
         *
         */
        {
            let ang_vel1 = parent.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dvel = parent
                .basis1
                .inverse_transform_vector(&(-mj_lambda1.linear - ang_vel1.gcross(parent.r1)))
                + parent
                    .basis2
                    .inverse_transform_vector(&(mj_lambda2.linear + ang_vel2.gcross(parent.r2)));

            let err = dvel + self.rhs_lin;

            new_lin_impulse = (self.lin_impulse + parent.inv_lhs_lin * err)
                .sup(&self.min_lin_impulse)
                .inf(&self.max_lin_impulse);
            let effective_impulse1 = parent.basis1 * (new_lin_impulse - self.lin_impulse);
            let effective_impulse2 = parent.basis2 * (new_lin_impulse - self.lin_impulse);

            mj_lambda1.linear += parent.im1 * effective_impulse1;
            mj_lambda1.angular += parent
                .ii1_sqrt
                .transform_vector(parent.r1.gcross(effective_impulse1));

            mj_lambda2.linear -= parent.im2 * effective_impulse2;
            mj_lambda2.angular -= parent
                .ii2_sqrt
                .transform_vector(parent.r2.gcross(effective_impulse2));
        }

        /*
         *
         * Solve rotations.
         *
         */
        {
            let ang_vel1 = parent.ii1_sqrt.transform_vector(mj_lambda1.angular);
            let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dvel = parent.basis2.inverse_transform_vector(&ang_vel2)
                - parent.basis1.inverse_transform_vector(&ang_vel1);
            let err = dvel + self.rhs_ang;

            new_ang_impulse = (self.ang_impulse + parent.inv_lhs_ang * err)
                .sup(&self.min_ang_impulse)
                .inf(&self.max_ang_impulse);
            let effective_impulse1 = parent.basis1 * (new_ang_impulse - self.ang_impulse);
            let effective_impulse2 = parent.basis2 * (new_ang_impulse - self.ang_impulse);

            mj_lambda1.angular += parent.ii1_sqrt.transform_vector(effective_impulse1);
            mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(effective_impulse2);
        }

        (new_lin_impulse, new_ang_impulse)
    }

    fn solve_ground(
        &self,
        parent: &GenericVelocityGroundConstraint,
        mj_lambda2: &mut DeltaVel<Real>,
    ) -> (Vector3<Real>, Vector3<Real>) {
        let new_lin_impulse;
        let new_ang_impulse;

        /*
         *
         * Solve translations.
         *
         */
        {
            let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dvel = parent
                .basis
                .inverse_transform_vector(&(mj_lambda2.linear + ang_vel2.gcross(parent.r2)));

            let err = dvel + self.rhs_lin;

            new_lin_impulse = (self.lin_impulse + parent.inv_lhs_lin * err)
                .sup(&self.min_lin_impulse)
                .inf(&self.max_lin_impulse);
            let effective_impulse = parent.basis * (new_lin_impulse - self.lin_impulse);

            mj_lambda2.linear -= parent.im2 * effective_impulse;
            mj_lambda2.angular -= parent
                .ii2_sqrt
                .transform_vector(parent.r2.gcross(effective_impulse));
        }

        /*
         *
         * Solve rotations.
         *
         */
        {
            let ang_vel2 = parent.ii2_sqrt.transform_vector(mj_lambda2.angular);

            let dvel = parent.basis.inverse_transform_vector(&ang_vel2);
            let err = dvel + self.rhs_ang;

            new_ang_impulse = (self.ang_impulse + parent.inv_lhs_ang * err)
                .sup(&self.min_ang_impulse)
                .inf(&self.max_ang_impulse);
            let effective_impulse = parent.basis * (new_ang_impulse - self.ang_impulse);

            mj_lambda2.angular -= parent.ii2_sqrt.transform_vector(effective_impulse);
        }

        (new_lin_impulse, new_ang_impulse)
    }
}
