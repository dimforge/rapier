use na::SimdRealField;

use crate::dynamics::solver::MotorParameters;
use crate::dynamics::{FixedJoint, MotorModel, PrismaticJoint, RevoluteJoint};
use crate::math::{Isometry, Point, Real, Rotation, UnitVector, Vector, SPATIAL_DIM};
use crate::utils::WBasis;

#[cfg(feature = "dim3")]
use crate::dynamics::SphericalJoint;

#[cfg(feature = "dim3")]
bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        const X = 1 << 0;
        const Y = 1 << 1;
        const Z = 1 << 2;
        const ANG_X = 1 << 3;
        const ANG_Y = 1 << 4;
        const ANG_Z = 1 << 5;
        const LOCKED_REVOLUTE_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        const LOCKED_PRISMATIC_AXES = Self::Y.bits | Self::Z.bits | Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        const LOCKED_FIXED_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits | Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        const LOCKED_SPHERICAL_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits;
        const FREE_REVOLUTE_AXES = Self::ANG_X.bits;
        const FREE_PRISMATIC_AXES = Self::X.bits;
        const FREE_FIXED_AXES = 0;
        const FREE_SPHERICAL_AXES = Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
    }
}

#[cfg(feature = "dim2")]
bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        const X = 1 << 0;
        const Y = 1 << 1;
        const ANG_X = 1 << 2;
        const LOCKED_REVOLUTE_AXES = Self::X.bits | Self::Y.bits;
        const LOCKED_PRISMATIC_AXES = Self::Y.bits | Self::ANG_X.bits;
        const LOCKED_FIXED_AXES = Self::X.bits | Self::Y.bits | Self::ANG_X.bits;
        const FREE_REVOLUTE_AXES = Self::ANG_X.bits;
        const FREE_PRISMATIC_AXES = Self::X.bits;
        const FREE_FIXED_AXES = 0;
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
pub struct JointLimits<N> {
    pub min: N,
    pub max: N,
    pub impulse: N,
}

impl<N: SimdRealField<Element = Real>> Default for JointLimits<N> {
    fn default() -> Self {
        Self {
            min: -N::splat(Real::MAX),
            max: N::splat(Real::MAX),
            impulse: N::splat(0.0),
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
    pub max_force: Real,
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
            max_force: Real::MAX,
            impulse: 0.0,
            model: MotorModel::AccelerationBased, // VelocityBased,
        }
    }
}

impl JointMotor {
    pub(crate) fn motor_params(&self, dt: Real) -> MotorParameters<Real> {
        let (erp_inv_dt, cfm_coeff, cfm_gain) =
            self.model
                .combine_coefficients(dt, self.stiffness, self.damping);
        MotorParameters {
            erp_inv_dt,
            cfm_coeff,
            cfm_gain,
            // keep_lhs,
            target_pos: self.target_pos,
            target_vel: self.target_vel,
            max_impulse: self.max_force * dt,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GenericJoint {
    pub local_frame1: Isometry<Real>,
    pub local_frame2: Isometry<Real>,
    pub locked_axes: JointAxesMask,
    pub limit_axes: JointAxesMask,
    pub motor_axes: JointAxesMask,
    pub limits: [JointLimits<Real>; SPATIAL_DIM],
    pub motors: [JointMotor; SPATIAL_DIM],
}

impl Default for GenericJoint {
    fn default() -> Self {
        Self {
            local_frame1: Isometry::identity(),
            local_frame2: Isometry::identity(),
            locked_axes: JointAxesMask::empty(),
            limit_axes: JointAxesMask::empty(),
            motor_axes: JointAxesMask::empty(),
            limits: [JointLimits::default(); SPATIAL_DIM],
            motors: [JointMotor::default(); SPATIAL_DIM],
        }
    }
}

impl GenericJoint {
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        *Self::default().lock_axes(locked_axes)
    }

    /// Can this joint use SIMD-accelerated constraint formulations?
    pub(crate) fn supports_simd_constraints(&self) -> bool {
        self.limit_axes.is_empty() && self.motor_axes.is_empty()
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

    pub fn lock_axes(&mut self, axes: JointAxesMask) -> &mut Self {
        self.locked_axes |= axes;
        self
    }

    pub fn set_local_frame1(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.local_frame1 = local_frame;
        self
    }

    pub fn set_local_frame2(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.local_frame2 = local_frame;
        self
    }

    #[must_use]
    pub fn local_axis1(&self) -> UnitVector<Real> {
        self.local_frame1 * Vector::x_axis()
    }

    pub fn set_local_axis1(&mut self, local_axis: UnitVector<Real>) -> &mut Self {
        self.local_frame1.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    #[must_use]
    pub fn local_axis2(&self) -> UnitVector<Real> {
        self.local_frame2 * Vector::x_axis()
    }

    pub fn set_local_axis2(&mut self, local_axis: UnitVector<Real>) -> &mut Self {
        self.local_frame2.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.local_frame1.translation.vector.into()
    }

    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.local_frame1.translation.vector = anchor1.coords;
        self
    }

    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.local_frame2.translation.vector.into()
    }

    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.local_frame2.translation.vector = anchor2.coords;
        self
    }

    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        let i = axis as usize;
        if self.limit_axes.contains(axis.into()) {
            Some(&self.limits[i])
        } else {
            None
        }
    }

    pub fn set_limits(&mut self, axis: JointAxis, limits: [Real; 2]) -> &mut Self {
        let i = axis as usize;
        self.limit_axes |= axis.into();
        self.limits[i].min = limits[0];
        self.limits[i].max = limits[1];
        self
    }

    #[must_use]
    pub fn motor_model(&self, axis: JointAxis) -> Option<MotorModel> {
        let i = axis as usize;
        if self.motor_axes.contains(axis.into()) {
            Some(self.motors[i].model)
        } else {
            None
        }
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, axis: JointAxis, model: MotorModel) -> &mut Self {
        self.motors[axis as usize].model = model;
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(
        &mut self,
        axis: JointAxis,
        target_vel: Real,
        factor: Real,
    ) -> &mut Self {
        self.set_motor(
            axis,
            self.motors[axis as usize].target_pos,
            target_vel,
            0.0,
            factor,
        )
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.set_motor(axis, target_pos, 0.0, stiffness, damping)
    }

    pub fn set_motor_max_force(&mut self, axis: JointAxis, max_force: Real) -> &mut Self {
        self.motors[axis as usize].max_force = max_force;
        self
    }

    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        let i = axis as usize;
        if self.motor_axes.contains(axis.into()) {
            Some(&self.motors[i])
        } else {
            None
        }
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        axis: JointAxis,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.motor_axes |= axis.into();
        let i = axis as usize;
        self.motors[i].target_vel = target_vel;
        self.motors[i].target_pos = target_pos;
        self.motors[i].stiffness = stiffness;
        self.motors[i].damping = damping;
        self
    }
}

macro_rules! joint_conversion_methods(
    ($as_joint: ident, $as_joint_mut: ident, $Joint: ty, $axes: expr) => {
        #[must_use]
        pub fn $as_joint(&self) -> Option<&$Joint> {
            if self.locked_axes == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute(self) })
            } else {
                None
            }
        }

        #[must_use]
        pub fn $as_joint_mut(&mut self) -> Option<&mut $Joint> {
            if self.locked_axes == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute(self) })
            } else {
                None
            }
        }
    }
);

impl GenericJoint {
    joint_conversion_methods!(
        as_revolute,
        as_revolute_mut,
        RevoluteJoint,
        JointAxesMask::LOCKED_REVOLUTE_AXES
    );
    joint_conversion_methods!(
        as_fixed,
        as_fixed_mut,
        FixedJoint,
        JointAxesMask::LOCKED_FIXED_AXES
    );
    joint_conversion_methods!(
        as_prismatic,
        as_prismatic_mut,
        PrismaticJoint,
        JointAxesMask::LOCKED_PRISMATIC_AXES
    );

    #[cfg(feature = "dim3")]
    joint_conversion_methods!(
        as_spherical,
        as_spherical_mut,
        SphericalJoint,
        JointAxesMask::LOCKED_SPHERICAL_AXES
    );
}

#[derive(Copy, Clone, Debug)]
pub struct GenericJointBuilder(GenericJoint);

impl GenericJointBuilder {
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self(GenericJoint::new(locked_axes))
    }

    #[must_use]
    pub fn lock_axes(mut self, axes: JointAxesMask) -> Self {
        self.0.lock_axes(axes);
        self
    }

    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame1(local_frame);
        self
    }

    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame2(local_frame);
        self
    }

    #[must_use]
    pub fn local_axis1(mut self, local_axis: UnitVector<Real>) -> Self {
        self.0.set_local_axis1(local_axis);
        self
    }

    #[must_use]
    pub fn local_axis2(mut self, local_axis: UnitVector<Real>) -> Self {
        self.0.set_local_axis2(local_axis);
        self
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    #[must_use]
    pub fn limits(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.0.set_limits(axis, limits);
        self
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, axis: JointAxis, model: MotorModel) -> Self {
        self.0.set_motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, axis: JointAxis, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    #[must_use]
    pub fn motor_position(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0
            .set_motor_position(axis, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    #[must_use]
    pub fn set_motor(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0
            .set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }

    #[must_use]
    pub fn motor_max_force(mut self, axis: JointAxis, max_force: Real) -> Self {
        self.0.set_motor_max_force(axis, max_force);
        self
    }

    #[must_use]
    pub fn build(self) -> GenericJoint {
        self.0
    }
}
