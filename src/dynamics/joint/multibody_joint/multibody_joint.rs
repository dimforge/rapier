use crate::dynamics::solver::AnyJointVelocityConstraint;
use crate::dynamics::{
    joint, FixedJoint, IntegrationParameters, JointAxesMask, JointData, Multibody, MultibodyLink,
    RigidBodyVelocity,
};
use crate::math::{
    Isometry, JacobianSliceMut, Matrix, Real, Rotation, SpacialVector, Translation, Vector,
    ANG_DIM, DIM, SPATIAL_DIM,
};
use crate::utils::WCross;
use na::{DVector, DVectorSliceMut};
#[cfg(feature = "dim3")]
use {
    crate::utils::WCrossMatrix,
    na::{UnitQuaternion, Vector3, VectorSlice3},
};

#[derive(Copy, Clone, Debug)]
pub struct MultibodyJoint {
    pub data: JointData,
    pub(crate) coords: SpacialVector<Real>,
    pub(crate) joint_rot: Rotation<Real>,
    jacobian_v: Matrix<Real>,
    jacobian_dot_v: Matrix<Real>,
    jacobian_dot_veldiff_v: Matrix<Real>,
}

#[cfg(feature = "dim2")]
fn revolute_locked_axes() -> JointAxesMask {
    JointAxesMask::X | JointAxesMask::Y
}

#[cfg(feature = "dim3")]
fn revolute_locked_axes() -> JointAxesMask {
    JointAxesMask::X
        | JointAxesMask::Y
        | JointAxesMask::Z
        | JointAxesMask::ANG_Y
        | JointAxesMask::ANG_Z
}

impl MultibodyJoint {
    pub fn new(data: JointData) -> Self {
        Self {
            data,
            coords: na::zero(),
            joint_rot: Rotation::identity(),
            jacobian_v: na::zero(),
            jacobian_dot_v: na::zero(),
            jacobian_dot_veldiff_v: na::zero(),
        }
    }

    pub(crate) fn free(pos: Isometry<Real>) -> Self {
        let mut result = Self::new(JointData::default());
        result.set_free_pos(pos);
        result
    }

    pub(crate) fn fixed(pos: Isometry<Real>) -> Self {
        Self::new(FixedJoint::new().local_frame1(pos).into())
    }

    pub(crate) fn set_free_pos(&mut self, pos: Isometry<Real>) {
        self.coords
            .fixed_rows_mut::<DIM>(0)
            .copy_from(&pos.translation.vector);
        self.joint_rot = pos.rotation;
    }

    pub fn local_joint_rot(&self) -> &Rotation<Real> {
        &self.joint_rot
    }

    fn num_free_lin_dofs(&self) -> usize {
        let locked_bits = self.data.locked_axes.bits();
        DIM - (locked_bits & ((1 << DIM) - 1)).count_ones() as usize
    }

    /// The number of degrees of freedom allowed by the multibody_joint.
    pub fn ndofs(&self) -> usize {
        SPATIAL_DIM - self.data.locked_axes.bits().count_ones() as usize
    }

    /// The position of the multibody link containing this multibody_joint relative to its parent.
    pub fn body_to_parent(&self) -> Isometry<Real> {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            self.data.local_frame1.translation
                * self.joint_rot
                * self.data.local_frame2.translation.inverse()
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let mut transform = self.joint_rot * self.data.local_frame2.inverse();

            for i in 0..DIM {
                if (locked_bits & (1 << i)) == 0 {
                    transform = Translation::from(Vector::ith(i, self.coords[i])) * transform;
                }
            }

            self.data.local_frame1 * transform
        }
    }

    /// Integrate the position of this multibody_joint.
    pub fn integrate(&mut self, dt: Real, vels: &[Real]) {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            #[cfg(feature = "dim3")]
            let axis = self.data.local_frame1 * Vector::x_axis();
            self.coords[DIM] += vels[0] * dt;

            #[cfg(feature = "dim2")]
            {
                self.joint_rot = Rotation::from_angle(self.coords[DIM]);
            }
            #[cfg(feature = "dim3")]
            {
                self.joint_rot = Rotation::from_axis_angle(&axis, self.coords[DIM]);
            }
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let mut curr_free_dof = 0;

            for i in 0..DIM {
                if (locked_bits & (1 << i)) == 0 {
                    self.coords[i] += vels[curr_free_dof] * dt;
                    curr_free_dof += 1;
                }
            }

            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let angvel = Vector3::from_row_slice(&vels[curr_free_dof..curr_free_dof + 3]);
                    let disp = UnitQuaternion::new_eps(angvel * dt, 0.0);
                    self.joint_rot = disp * self.joint_rot;
                }
                _ => unreachable!(),
            }
        }
    }

    /// Apply a displacement to the multibody_joint.
    pub fn apply_displacement(&mut self, disp: &[Real]) {
        self.integrate(1.0, disp);
    }

    /// Update the jacobians of this multibody_joint.
    pub fn update_jacobians(&mut self, vels: &[Real]) {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            #[cfg(feature = "dim2")]
            let axis = 1.0;
            #[cfg(feature = "dim3")]
            let axis = self.data.local_frame1 * Vector::x_axis();
            let body_shift = self.data.local_frame2.translation.vector;
            let shift = self.joint_rot * -body_shift;
            let shift_dot_veldiff = axis.gcross(shift);

            #[cfg(feature = "dim2")]
            {
                self.jacobian_v.column_mut(0).copy_from(&axis.gcross(shift));
            }
            #[cfg(feature = "dim3")]
            {
                self.jacobian_v.column_mut(0).copy_from(&axis.gcross(shift));
            }
            self.jacobian_dot_veldiff_v
                .column_mut(0)
                .copy_from(&axis.gcross(shift_dot_veldiff));
            self.jacobian_dot_v
                .column_mut(0)
                .copy_from(&(axis.gcross(shift_dot_veldiff) * vels[0]));
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let num_free_lin_dofs = self.num_free_lin_dofs();
                    let inv_frame2 = self.data.local_frame2.inverse();
                    let shift = self.joint_rot * inv_frame2.translation.vector;
                    let angvel =
                        VectorSlice3::from_slice(&vels[num_free_lin_dofs..num_free_lin_dofs + 3]);
                    let inv_rotmat2 = inv_frame2.rotation.to_rotation_matrix().into_inner();

                    self.jacobian_v = inv_rotmat2 * shift.gcross_matrix().transpose();
                    self.jacobian_dot_v =
                        inv_rotmat2 * angvel.cross(&shift).gcross_matrix().transpose();
                }
                _ => unreachable!(),
            }
        }
    }

    /// Sets in `out` the non-zero entries of the multibody_joint jacobian transformed by `transform`.
    pub fn jacobian(&self, transform: &Isometry<Real>, out: &mut JacobianSliceMut<Real>) {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            #[cfg(feature = "dim2")]
            let axis = 1.0;
            #[cfg(feature = "dim3")]
            let axis = self.data.local_frame1 * Vector::x();
            let jacobian = RigidBodyVelocity::new(self.jacobian_v.column(0).into_owned(), axis);
            out.copy_from(jacobian.transformed(transform).as_vector())
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let mut curr_free_dof = 0;

            for i in 0..DIM {
                if (locked_bits & (1 << i)) == 0 {
                    let transformed_axis = transform * self.data.local_frame1 * Vector::ith(i, 1.0);
                    out.fixed_slice_mut::<DIM, 1>(0, curr_free_dof)
                        .copy_from(&transformed_axis);
                    curr_free_dof += 1;
                }
            }

            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let rotmat = transform.rotation.to_rotation_matrix();
                    out.fixed_slice_mut::<3, 3>(0, curr_free_dof)
                        .copy_from(&(rotmat * self.jacobian_v));
                    out.fixed_slice_mut::<3, 3>(3, curr_free_dof)
                        .copy_from(rotmat.matrix());
                }
                _ => unreachable!(),
            }
        }
    }

    /// Sets in `out` the non-zero entries of the time-derivative of the multibody_joint jacobian transformed by `transform`.
    pub fn jacobian_dot(&self, transform: &Isometry<Real>, out: &mut JacobianSliceMut<Real>) {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            let jacobian = RigidBodyVelocity::from_vectors(
                self.jacobian_dot_v.column(0).into_owned(),
                na::zero(),
            );
            out.copy_from(jacobian.transformed(transform).as_vector())
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let num_free_lin_dofs = self.num_free_lin_dofs();
                    let rotmat = transform.rotation.to_rotation_matrix();
                    out.fixed_slice_mut::<3, 3>(0, num_free_lin_dofs)
                        .copy_from(&(rotmat * self.jacobian_dot_v));
                }
                _ => unreachable!(),
            }
        }
    }

    /// Sets in `out` the non-zero entries of the velocity-derivative of the time-derivative of the multibody_joint jacobian transformed by `transform`.
    pub fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry<Real>,
        acc: &[Real],
        out: &mut JacobianSliceMut<Real>,
    ) {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            let jacobian = RigidBodyVelocity::from_vectors(
                self.jacobian_dot_veldiff_v.column(0).into_owned(),
                na::zero(),
            );
            out.copy_from((jacobian.transformed(transform) * acc[0]).as_vector())
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let num_free_lin_dofs = self.num_free_lin_dofs();
                    let angvel =
                        Vector3::from_row_slice(&acc[num_free_lin_dofs..num_free_lin_dofs + 3]);
                    let rotmat = transform.rotation.to_rotation_matrix();
                    let res = rotmat * angvel.gcross_matrix() * self.jacobian_v;
                    out.fixed_slice_mut::<3, 3>(0, num_free_lin_dofs)
                        .copy_from(&res);
                }
                _ => unreachable!(),
            }
        }
    }

    /// Multiply the multibody_joint jacobian by generalized velocities to obtain the
    /// relative velocity of the multibody link containing this multibody_joint.
    pub fn jacobian_mul_coordinates(&self, acc: &[Real]) -> RigidBodyVelocity {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            #[cfg(feature = "dim2")]
            let axis = 1.0;
            #[cfg(feature = "dim3")]
            let axis = self.data.local_frame1 * Vector::x();
            RigidBodyVelocity::new(self.jacobian_v.column(0).into_owned(), axis) * acc[0]
        } else {
            let locked_bits = self.data.locked_axes.bits();
            let mut result = RigidBodyVelocity::zero();
            let mut curr_free_dof = 0;

            for i in 0..DIM {
                if (locked_bits & (1 << i)) == 0 {
                    result.linvel += self.data.local_frame1 * Vector::ith(i, acc[curr_free_dof]);
                    curr_free_dof += 1;
                }
            }

            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => { /* No free dofs. */ }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let angvel = Vector3::from_row_slice(&acc[curr_free_dof..curr_free_dof + 3]);
                    let linvel = self.jacobian_v * angvel;
                    result += RigidBodyVelocity::new(linvel, angvel);
                }
                _ => unreachable!(),
            }
            result
        }
    }

    /// Multiply the multibody_joint jacobian by generalized accelerations to obtain the
    /// relative acceleration of the multibody link containing this multibody_joint.
    pub fn jacobian_dot_mul_coordinates(&self, acc: &[Real]) -> RigidBodyVelocity {
        if self.data.locked_axes == revolute_locked_axes() {
            // FIXME: this is a special case for the revolute joint.
            // We have the mathematical formulation ready that works in the general case, but its
            // implementation will take some time. So let’s make a special case for the alpha
            // release and fix is soon after.
            RigidBodyVelocity::from_vectors(self.jacobian_dot_v.column(0).into_owned(), na::zero())
                * acc[0]
        } else {
            let locked_bits = self.data.locked_axes.bits();

            let locked_ang_bits = locked_bits >> DIM;
            let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
            match num_free_ang_dofs {
                0 => {
                    /* No free dofs. */
                    RigidBodyVelocity::zero()
                }
                1 => {
                    todo!()
                }
                2 => {
                    todo!()
                }
                #[cfg(feature = "dim3")]
                3 => {
                    let num_free_lin_dofs = self.num_free_lin_dofs();
                    let angvel =
                        Vector3::from_row_slice(&acc[num_free_lin_dofs..num_free_lin_dofs + 3]);
                    let linvel = self.jacobian_dot_v * angvel;
                    RigidBodyVelocity::new(linvel, na::zero())
                }
                _ => unreachable!(),
            }
        }
    }

    /// Fill `out` with the non-zero entries of a damping that can be applied by default to ensure a good stability of the multibody_joint.
    pub fn default_damping(&self, out: &mut DVectorSliceMut<Real>) {
        let locked_bits = self.data.locked_axes.bits();
        let mut curr_free_dof = self.num_free_lin_dofs();

        // A default damping only for the angular dofs
        for i in DIM..SPATIAL_DIM {
            if locked_bits & (1 << i) == 0 {
                // This is a free angular DOF.
                out[curr_free_dof] = 0.2;
                curr_free_dof += 1;
            }
        }
    }

    /// Maximum number of velocity constrains that can be generated by this multibody_joint.
    pub fn num_velocity_constraints(&self) -> usize {
        let locked_bits = self.data.locked_axes.bits();
        let limit_bits = self.data.limit_axes.bits();
        let motor_bits = self.data.motor_axes.bits();
        let mut num_constraints = 0;

        for i in 0..SPATIAL_DIM {
            if (locked_bits & (1 << i)) == 0 {
                if (limit_bits & (1 << i)) != 0 {
                    num_constraints += 1;
                }
                if (motor_bits & (1 << i)) != 0 {
                    num_constraints += 1;
                }
            }
        }

        num_constraints
    }

    /// Initialize and generate velocity constraints to enforce, e.g., multibody_joint limits and motors.
    pub fn velocity_constraints(
        &self,
        params: &IntegrationParameters,
        multibody: &Multibody,
        link: &MultibodyLink,
        dof_id: usize,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        constraints: &mut Vec<AnyJointVelocityConstraint>,
    ) {
        let locked_bits = self.data.locked_axes.bits();
        let limit_bits = self.data.limit_axes.bits();
        let motor_bits = self.data.motor_axes.bits();
        let mut curr_free_dof = 0;

        for i in 0..DIM {
            if (locked_bits & (1 << i)) == 0 {
                if (limit_bits & (1 << i)) != 0 {
                    joint::unit_joint_limit_constraint(
                        params,
                        multibody,
                        link,
                        [self.data.limits[i].min, self.data.limits[i].max],
                        self.coords[i],
                        dof_id + curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                    );
                }

                if (motor_bits & (1 << i)) != 0 {
                    joint::unit_joint_motor_constraint(
                        params,
                        multibody,
                        link,
                        &self.data.motors[i],
                        self.coords[i],
                        dof_id + curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                    );
                }
                curr_free_dof += 1;
            }
        }

        /*
        let locked_ang_bits = locked_bits >> DIM;
        let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
        match num_free_ang_dofs {
            0 => { /* No free dofs. */ }
            1 => {}
            2 => {
                todo!()
            }
            3 => {}
            _ => unreachable!(),
        }
         */
        // TODO: we should make special cases for multi-angular-dofs limits/motors
        for i in DIM..SPATIAL_DIM {
            if (locked_bits & (1 << i)) == 0 {
                if (limit_bits & (1 << i)) != 0 {
                    joint::unit_joint_limit_constraint(
                        params,
                        multibody,
                        link,
                        [self.data.limits[i].min, self.data.limits[i].max],
                        self.coords[i],
                        dof_id + curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                    );
                }

                if (motor_bits & (1 << i)) != 0 {
                    joint::unit_joint_motor_constraint(
                        params,
                        multibody,
                        link,
                        &self.data.motors[i],
                        self.coords[i],
                        dof_id + curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                    );
                }
                curr_free_dof += 1;
            }
        }
    }
}
