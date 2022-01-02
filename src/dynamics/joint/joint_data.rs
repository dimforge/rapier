use crate::dynamics::solver::MotorParameters;
use crate::dynamics::MotorModel;
use crate::math::{Isometry, Point, Real, Rotation, UnitVector, SPATIAL_DIM};
use crate::utils::WBasis;

#[cfg(feature = "dim3")]
bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        const FREE = 0;
        const X = 1 << 0;
        const Y = 1 << 1;
        const Z = 1 << 2;
        const ANG_X = 1 << 3;
        const ANG_Y = 1 << 4;
        const ANG_Z = 1 << 5;
    }
}

#[cfg(feature = "dim2")]
bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        const FREE = 0;
        const X = 1 << 0;
        const Y = 1 << 1;
        const ANG_X = 1 << 2;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum JointAxis {
    X = 0,
    Y,
    #[cfg(feature = "dim3")]
    Z,
    AngX,
    #[cfg(feature = "dim3")]
    AngY,
    #[cfg(feature = "dim3")]
    AngZ,
}

impl From<JointAxis> for JointAxesMask {
    fn from(axis: JointAxis) -> Self {
        JointAxesMask::from_bits(1 << axis as usize).unwrap()
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct JointLimits {
    pub min: Real,
    pub max: Real,
    pub impulse: Real,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            min: -Real::MAX,
            max: Real::MAX,
            impulse: 0.0,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct JointMotor {
    pub target_vel: Real,
    pub target_pos: Real,
    pub stiffness: Real,
    pub damping: Real,
    pub max_impulse: Real,
    pub impulse: Real,
    pub model: MotorModel,
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            target_pos: 0.0,
            target_vel: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_impulse: Real::MAX,
            impulse: 0.0,
            model: MotorModel::VelocityBased,
        }
    }
}

impl JointMotor {
    pub(crate) fn motor_params(&self, dt: Real) -> MotorParameters<Real> {
        let (stiffness, damping, gamma, _keep_lhs) =
            self.model
                .combine_coefficients(dt, self.stiffness, self.damping);
        MotorParameters {
            stiffness,
            damping,
            gamma,
            // keep_lhs,
            target_pos: self.target_pos,
            target_vel: self.target_vel,
            max_impulse: self.max_impulse,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct JointData {
    pub local_frame1: Isometry<Real>,
    pub local_frame2: Isometry<Real>,
    pub locked_axes: JointAxesMask,
    pub limit_axes: JointAxesMask,
    pub motor_axes: JointAxesMask,
    pub limits: [JointLimits; SPATIAL_DIM],
    pub motors: [JointMotor; SPATIAL_DIM],
}

impl Default for JointData {
    fn default() -> Self {
        Self {
            local_frame1: Isometry::identity(),
            local_frame2: Isometry::identity(),
            locked_axes: JointAxesMask::FREE,
            limit_axes: JointAxesMask::FREE,
            motor_axes: JointAxesMask::FREE,
            limits: [JointLimits::default(); SPATIAL_DIM],
            motors: [JointMotor::default(); SPATIAL_DIM],
        }
    }
}

impl JointData {
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self::default().lock_axes(locked_axes)
    }

    /// Can this joint use SIMD-accelerated constraint formulations?
    pub fn supports_simd_constraints(&self) -> bool {
        self.limit_axes.is_empty() && self.motor_axes.is_empty()
    }

    #[must_use]
    pub fn lock_axes(mut self, axes: JointAxesMask) -> Self {
        self.locked_axes |= axes;
        self
    }

    fn complete_ang_frame(axis: UnitVector<Real>) -> Rotation<Real> {
        let basis = axis.orthonormal_basis();

        #[cfg(feature = "dim2")]
        {
            use na::{Matrix2, Rotation2, UnitComplex};
            let mat = Matrix2::from_columns(&[axis.into_inner(), basis[0]]);
            let rotmat = Rotation2::from_matrix_unchecked(mat);
            UnitComplex::from_rotation_matrix(&rotmat)
        }

        #[cfg(feature = "dim3")]
        {
            use na::{Matrix3, Rotation3, UnitQuaternion};
            let mat = Matrix3::from_columns(&[axis.into_inner(), basis[0], basis[1]]);
            let rotmat = Rotation3::from_matrix_unchecked(mat);
            UnitQuaternion::from_rotation_matrix(&rotmat)
        }
    }

    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.local_frame1 = local_frame;
        self
    }

    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.local_frame2 = local_frame;
        self
    }

    #[must_use]
    pub fn local_axis1(mut self, local_axis: UnitVector<Real>) -> Self {
        self.local_frame1.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    #[must_use]
    pub fn local_axis2(mut self, local_axis: UnitVector<Real>) -> Self {
        self.local_frame2.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.local_frame1.translation.vector = anchor1.coords;
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.local_frame2.translation.vector = anchor2.coords;
        self
    }

    #[must_use]
    pub fn limit_axis(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        let i = axis as usize;
        self.limit_axes |= axis.into();
        self.limits[i].min = limits[0];
        self.limits[i].max = limits[1];
        self
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn motor_model(mut self, axis: JointAxis, model: MotorModel) -> Self {
        self.motors[axis as usize].model = model;
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn motor_velocity(self, axis: JointAxis, target_vel: Real, factor: Real) -> Self {
        self.motor_axis(
            axis,
            self.motors[axis as usize].target_pos,
            target_vel,
            0.0,
            factor,
        )
    }

    /// Sets the target angle this motor needs to reach.
    pub fn motor_position(
        self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.motor_axis(axis, target_pos, 0.0, stiffness, damping)
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn motor_axis(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.motor_axes |= axis.into();
        let i = axis as usize;
        self.motors[i].target_vel = target_vel;
        self.motors[i].target_pos = target_pos;
        self.motors[i].stiffness = stiffness;
        self.motors[i].damping = damping;
        self
    }

    pub fn motor_max_impulse(mut self, axis: JointAxis, max_impulse: Real) -> Self {
        self.motors[axis as usize].max_impulse = max_impulse;
        self
    }
}
