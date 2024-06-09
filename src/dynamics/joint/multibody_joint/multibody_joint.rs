use crate::dynamics::solver::JointGenericOneBodyConstraint;
use crate::dynamics::{
    joint, FixedJointBuilder, GenericJoint, IntegrationParameters, Multibody, MultibodyLink,
    RigidBodyVelocity,
};
use crate::math::{
    Isometry, JacobianViewMut, Real, Rotation, SpacialVector, Translation, Vector, ANG_DIM, DIM,
    SPATIAL_DIM,
};
use na::{DVector, DVectorViewMut};
#[cfg(feature = "dim3")]
use na::{UnitQuaternion, Vector3};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
/// An joint attached to two bodies based on the reduced coordinates formalism.
pub struct MultibodyJoint {
    /// The joint’s description.
    pub data: GenericJoint,
    /// Is the joint a kinematic joint?
    ///
    /// Kinematic joint velocities are never changed by the physics engine. This gives the user
    /// total control over the values of their degrees of freedoms.
    pub kinematic: bool,
    pub(crate) coords: SpacialVector<Real>,
    pub(crate) joint_rot: Rotation<Real>,
}

impl MultibodyJoint {
    /// Creates a new multibody joint from its description.
    pub fn new(data: GenericJoint, kinematic: bool) -> Self {
        Self {
            data,
            kinematic,
            coords: na::zero(),
            joint_rot: Rotation::identity(),
        }
    }

    pub(crate) fn free(pos: Isometry<Real>) -> Self {
        let mut result = Self::new(GenericJoint::default(), false);
        result.set_free_pos(pos);
        result
    }

    pub(crate) fn fixed(pos: Isometry<Real>) -> Self {
        Self::new(
            FixedJointBuilder::new().local_frame1(pos).build().into(),
            false,
        )
    }

    pub(crate) fn set_free_pos(&mut self, pos: Isometry<Real>) {
        self.coords
            .fixed_rows_mut::<DIM>(0)
            .copy_from(&pos.translation.vector);
        self.joint_rot = pos.rotation;
    }

    // pub(crate) fn local_joint_rot(&self) -> &Rotation<Real> {
    //     &self.joint_rot
    // }

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
        let locked_bits = self.data.locked_axes.bits();
        let mut transform = self.joint_rot * self.data.local_frame2.inverse();

        for i in 0..DIM {
            if (locked_bits & (1 << i)) == 0 {
                transform = Translation::from(Vector::ith(i, self.coords[i])) * transform;
            }
        }

        self.data.local_frame1 * transform
    }

    /// Integrate the position of this multibody_joint.
    pub fn integrate(&mut self, dt: Real, vels: &[Real]) {
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
                let dof_id = (!locked_ang_bits).trailing_zeros() as usize;
                self.coords[DIM + dof_id] += vels[curr_free_dof] * dt;
                #[cfg(feature = "dim2")]
                {
                    self.joint_rot = Rotation::new(self.coords[DIM + dof_id]);
                }
                #[cfg(feature = "dim3")]
                {
                    self.joint_rot = Rotation::from_axis_angle(
                        &Vector::ith_axis(dof_id),
                        self.coords[DIM + dof_id],
                    );
                }
            }
            2 => {
                todo!()
            }
            #[cfg(feature = "dim3")]
            3 => {
                let angvel = Vector3::from_row_slice(&vels[curr_free_dof..curr_free_dof + 3]);
                let disp = UnitQuaternion::new_eps(angvel * dt, 0.0);
                self.joint_rot = disp * self.joint_rot;
                self.coords[3] += angvel[0] * dt;
                self.coords[4] += angvel[1] * dt;
                self.coords[5] += angvel[2] * dt;
            }
            _ => unreachable!(),
        }
    }

    /// Apply a displacement to the multibody_joint.
    pub fn apply_displacement(&mut self, disp: &[Real]) {
        self.integrate(1.0, disp);
    }

    /// Sets in `out` the non-zero entries of the multibody_joint jacobian transformed by `transform`.
    pub fn jacobian(&self, transform: &Rotation<Real>, out: &mut JacobianViewMut<Real>) {
        let locked_bits = self.data.locked_axes.bits();
        let mut curr_free_dof = 0;

        for i in 0..DIM {
            if (locked_bits & (1 << i)) == 0 {
                let transformed_axis = transform * Vector::ith(i, 1.0);
                out.fixed_view_mut::<DIM, 1>(0, curr_free_dof)
                    .copy_from(&transformed_axis);
                curr_free_dof += 1;
            }
        }

        let locked_ang_bits = locked_bits >> DIM;
        let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
        match num_free_ang_dofs {
            0 => { /* No free dofs. */ }
            1 => {
                #[cfg(feature = "dim2")]
                {
                    out[(DIM, curr_free_dof)] = 1.0;
                }

                #[cfg(feature = "dim3")]
                {
                    let dof_id = (!locked_ang_bits).trailing_zeros() as usize;
                    let rotmat = transform.to_rotation_matrix().into_inner();
                    out.fixed_view_mut::<ANG_DIM, 1>(DIM, curr_free_dof)
                        .copy_from(&rotmat.column(dof_id));
                }
            }
            2 => {
                todo!()
            }
            #[cfg(feature = "dim3")]
            3 => {
                let rotmat = transform.to_rotation_matrix();
                out.fixed_view_mut::<3, 3>(3, curr_free_dof)
                    .copy_from(rotmat.matrix());
            }
            _ => unreachable!(),
        }
    }

    /// Multiply the multibody_joint jacobian by generalized velocities to obtain the
    /// relative velocity of the multibody link containing this multibody_joint.
    pub fn jacobian_mul_coordinates(&self, acc: &[Real]) -> RigidBodyVelocity {
        let locked_bits = self.data.locked_axes.bits();
        let mut result = RigidBodyVelocity::zero();
        let mut curr_free_dof = 0;

        for i in 0..DIM {
            if (locked_bits & (1 << i)) == 0 {
                result.linvel += Vector::ith(i, acc[curr_free_dof]);
                curr_free_dof += 1;
            }
        }

        let locked_ang_bits = locked_bits >> DIM;
        let num_free_ang_dofs = ANG_DIM - locked_ang_bits.count_ones() as usize;
        match num_free_ang_dofs {
            0 => { /* No free dofs. */ }
            1 => {
                #[cfg(feature = "dim2")]
                {
                    result.angvel += acc[curr_free_dof];
                }
                #[cfg(feature = "dim3")]
                {
                    let dof_id = (!locked_ang_bits).trailing_zeros() as usize;
                    result.angvel[dof_id] += acc[curr_free_dof];
                }
            }
            2 => {
                todo!()
            }
            #[cfg(feature = "dim3")]
            3 => {
                let angvel = Vector3::from_row_slice(&acc[curr_free_dof..curr_free_dof + 3]);
                result.angvel += angvel;
            }
            _ => unreachable!(),
        }
        result
    }

    /// Fill `out` with the non-zero entries of a damping that can be applied by default to ensure a good stability of the multibody_joint.
    pub fn default_damping(&self, out: &mut DVectorViewMut<Real>) {
        let locked_bits = self.data.locked_axes.bits();
        let mut curr_free_dof = self.num_free_lin_dofs();

        // A default damping only for the angular dofs
        for i in DIM..SPATIAL_DIM {
            if locked_bits & (1 << i) == 0 {
                // This is a free angular DOF.
                out[curr_free_dof] = 0.1;
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
        mut j_id: usize,
        jacobians: &mut DVector<Real>,
        constraints: &mut [JointGenericOneBodyConstraint],
    ) -> usize {
        let j_id = &mut j_id;
        let locked_bits = self.data.locked_axes.bits();
        let limit_bits = self.data.limit_axes.bits();
        let motor_bits = self.data.motor_axes.bits();
        let mut num_constraints = 0;
        let mut curr_free_dof = 0;

        for i in 0..DIM {
            if (locked_bits & (1 << i)) == 0 {
                let limits = if (limit_bits & (1 << i)) != 0 {
                    Some([self.data.limits[i].min, self.data.limits[i].max])
                } else {
                    None
                };

                if (motor_bits & (1 << i)) != 0 {
                    joint::unit_joint_motor_constraint(
                        params,
                        multibody,
                        link,
                        &self.data.motors[i],
                        self.coords[i],
                        limits,
                        curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                        &mut num_constraints,
                    );
                }

                if (limit_bits & (1 << i)) != 0 {
                    joint::unit_joint_limit_constraint(
                        params,
                        multibody,
                        link,
                        [self.data.limits[i].min, self.data.limits[i].max],
                        self.coords[i],
                        curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                        &mut num_constraints,
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
                let limits = if (limit_bits & (1 << i)) != 0 {
                    let limits = [self.data.limits[i].min, self.data.limits[i].max];
                    joint::unit_joint_limit_constraint(
                        params,
                        multibody,
                        link,
                        limits,
                        self.coords[i],
                        curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                        &mut num_constraints,
                    );
                    Some(limits)
                } else {
                    None
                };

                if (motor_bits & (1 << i)) != 0 {
                    joint::unit_joint_motor_constraint(
                        params,
                        multibody,
                        link,
                        &self.data.motors[i],
                        self.coords[i],
                        limits,
                        curr_free_dof,
                        j_id,
                        jacobians,
                        constraints,
                        &mut num_constraints,
                    );
                }
                curr_free_dof += 1;
            }
        }

        num_constraints
    }
}
