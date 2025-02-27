use crate::dynamics::{AxisMask, RigidBody, RigidBodyPosition, RigidBodyVelocity};
use crate::math::{Isometry, Point, Real, Rotation, Vector};
use parry::math::AngVector;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PidController {
    pub lin_integral: Vector<Real>,
    pub ang_integral: AngVector<Real>,
    pub lin_kp: Vector<Real>,
    pub lin_ki: Vector<Real>,
    pub lin_kd: Vector<Real>,
    pub ang_kp: AngVector<Real>,
    pub ang_ki: AngVector<Real>,
    pub ang_kd: AngVector<Real>,
    pub axes: AxisMask,
}

impl Default for PidController {
    fn default() -> Self {
        Self::new(60.0, 1.0, 0.8, AxisMask::all())
    }
}

pub struct PidErrors {
    pub linear: Vector<Real>,
    pub angular: AngVector<Real>,
}

impl From<RigidBodyVelocity> for PidErrors {
    fn from(vels: RigidBodyVelocity) -> Self {
        Self {
            linear: vels.linvel,
            angular: vels.angvel,
        }
    }
}

impl PidController {
    pub fn new(kp: Real, ki: Real, kd: Real, axes: AxisMask) -> PidController {
        #[cfg(feature = "dim2")]
        return Self {
            lin_integral: na::zero(),
            ang_integral: na::zero(),
            lin_kp: Vector::repeat(kp),
            lin_ki: Vector::repeat(ki),
            lin_kd: Vector::repeat(kd),
            ang_kp: kp,
            ang_ki: ki,
            ang_kd: kd,
            axes,
        };

        #[cfg(feature = "dim3")]
        return Self {
            lin_integral: na::zero(),
            ang_integral: na::zero(),
            lin_kp: Vector::repeat(kp),
            lin_ki: Vector::repeat(ki),
            lin_kd: Vector::repeat(kd),
            ang_kp: AngVector::repeat(kp),
            ang_ki: AngVector::repeat(ki),
            ang_kd: AngVector::repeat(kd),
            axes,
        };
    }

    pub fn reset(&mut self) {
        self.lin_integral = na::zero();
        self.ang_integral = na::zero();
    }

    pub fn update_with_rigid_body_linear(
        &mut self,
        dt: Real,
        rb: &RigidBody,
        target_pos: Point<Real>,
        target_linvel: Vector<Real>,
    ) -> Vector<Real> {
        self.update_with_rigid_body(
            dt,
            rb,
            Isometry::from(target_pos),
            RigidBodyVelocity {
                linvel: target_linvel,
                angvel: *rb.angvel(),
            },
        )
        .linvel
    }

    pub fn update_with_rigid_body_angular(
        &mut self,
        dt: Real,
        rb: &RigidBody,
        target_rot: Rotation<Real>,
        target_angvel: Vector<Real>,
    ) -> AngVector<Real> {
        self.update_with_rigid_body(
            dt,
            rb,
            Isometry::from_parts(na::one(), target_rot),
            RigidBodyVelocity {
                linvel: *rb.linvel(),
                angvel: target_angvel,
            },
        )
        .angvel
    }

    pub fn update_with_rigid_body(
        &mut self,
        dt: Real,
        rb: &RigidBody,
        target_pose: Isometry<Real>,
        target_vels: RigidBodyVelocity,
    ) -> RigidBodyVelocity {
        let pose_errors = RigidBodyPosition {
            position: rb.pos.position,
            next_position: target_pose,
        }
        .pose_errors(rb.local_center_of_mass());
        let vels_errors = target_vels - rb.vels;
        self.update(dt, &pose_errors, &vels_errors.into())
    }

    /// Mask where each component is 1.0 or 0.0 depending on whether
    /// the corresponding linear axis is enabled.
    fn lin_mask(&self) -> Vector<Real> {
        #[cfg(feature = "dim2")]
        return Vector::new(
            self.axes.contains(AxisMask::LIN_X) as u32 as Real,
            self.axes.contains(AxisMask::LIN_Y) as u32 as Real,
        );
        #[cfg(feature = "dim3")]
        return Vector::new(
            self.axes.contains(AxisMask::LIN_X) as u32 as Real,
            self.axes.contains(AxisMask::LIN_Y) as u32 as Real,
            self.axes.contains(AxisMask::LIN_Z) as u32 as Real,
        );
    }

    /// Mask where each component is 1.0 or 0.0 depending on whether
    /// the corresponding angular axis is enabled.
    fn ang_mask(&self) -> AngVector<Real> {
        #[cfg(feature = "dim2")]
        return self.axes.contains(AxisMask::ANG_Z) as u32 as Real;
        #[cfg(feature = "dim3")]
        return Vector::new(
            self.axes.contains(AxisMask::ANG_X) as u32 as Real,
            self.axes.contains(AxisMask::ANG_Y) as u32 as Real,
            self.axes.contains(AxisMask::ANG_Z) as u32 as Real,
        );
    }

    pub fn update(
        &mut self,
        dt: Real,
        pose_errors: &PidErrors,
        vel_errors: &PidErrors,
    ) -> RigidBodyVelocity {
        self.lin_integral += pose_errors.linear * dt;
        self.ang_integral += pose_errors.angular * dt;

        let lin_mask = self.lin_mask();
        let ang_mask = self.ang_mask();

        RigidBodyVelocity {
            linvel: (pose_errors.linear.component_mul(&self.lin_kp)
                + vel_errors.linear.component_mul(&self.lin_kd)
                + self.lin_integral.component_mul(&self.lin_ki))
            .component_mul(&lin_mask),
            #[cfg(feature = "dim2")]
            angvel: (pose_errors.angular * self.ang_kp
                + vel_errors.angular * self.ang_kd
                + self.ang_integral * self.ang_ki)
                * ang_mask,
            #[cfg(feature = "dim3")]
            angvel: (pose_errors.angular.component_mul(&self.ang_kp)
                + vel_errors.angular.component_mul(&self.ang_kd)
                + self.ang_integral.component_mul(&self.ang_ki))
            .component_mul(&ang_mask),
        }
    }
}
