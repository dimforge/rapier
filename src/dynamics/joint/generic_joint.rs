use crate::dynamics::solver::MotorParameters;
use crate::dynamics::{FixedJoint, MotorModel, PrismaticJoint, RevoluteJoint, RopeJoint};
use crate::math::{Isometry, Point, Real, Rotation, UnitVector, Vector, SPATIAL_DIM};
use crate::utils::{WBasis, WReal};

#[cfg(feature = "dim3")]
use crate::dynamics::SphericalJoint;

#[cfg(feature = "dim3")]
bitflags::bitflags! {
    /// A bit mask identifying multiple degrees of freedom of a joint.
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        /// The translational degree of freedom along the local X axis of a joint.
        const X = 1 << 0;
        /// The translational degree of freedom along the local Y axis of a joint.
        const Y = 1 << 1;
        /// The translational degree of freedom along the local Z axis of a joint.
        const Z = 1 << 2;
        /// The angular degree of freedom along the local X axis of a joint.
        const ANG_X = 1 << 3;
        /// The angular degree of freedom along the local Y axis of a joint.
        const ANG_Y = 1 << 4;
        /// The angular degree of freedom along the local Z axis of a joint.
        const ANG_Z = 1 << 5;
        /// The set of degrees of freedom locked by a revolute joint.
        const LOCKED_REVOLUTE_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        /// The set of degrees of freedom locked by a prismatic joint.
        const LOCKED_PRISMATIC_AXES = Self::Y.bits | Self::Z.bits | Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        /// The set of degrees of freedom locked by a fixed joint.
        const LOCKED_FIXED_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits | Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        /// The set of degrees of freedom locked by a spherical joint.
        const LOCKED_SPHERICAL_AXES = Self::X.bits | Self::Y.bits | Self::Z.bits;
        /// The set of degrees of freedom left free by a revolute joint.
        const FREE_REVOLUTE_AXES = Self::ANG_X.bits;
        /// The set of degrees of freedom left free by a prismatic joint.
        const FREE_PRISMATIC_AXES = Self::X.bits;
        /// The set of degrees of freedom left free by a fixed joint.
        const FREE_FIXED_AXES = 0;
        /// The set of degrees of freedom left free by a spherical joint.
        const FREE_SPHERICAL_AXES = Self::ANG_X.bits | Self::ANG_Y.bits | Self::ANG_Z.bits;
        /// The set of all translational degrees of freedom.
        const LIN_AXES = Self::X.bits() | Self::Y.bits() | Self::Z.bits();
        /// The set of all angular degrees of freedom.
        const ANG_AXES = Self::ANG_X.bits() | Self::ANG_Y.bits() | Self::ANG_Z.bits();
    }
}

#[cfg(feature = "dim2")]
bitflags::bitflags! {
    /// A bit mask identifying multiple degrees of freedom of a joint.
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    pub struct JointAxesMask: u8 {
        /// The translational degree of freedom along the local X axis of a joint.
        const X = 1 << 0;
        /// The translational degree of freedom along the local Y axis of a joint.
        const Y = 1 << 1;
        /// The angular degree of freedom of a joint.
        const ANG_X = 1 << 2;
        /// The set of degrees of freedom locked by a revolute joint.
        const LOCKED_REVOLUTE_AXES = Self::X.bits | Self::Y.bits;
        /// The set of degrees of freedom locked by a prismatic joint.
        const LOCKED_PRISMATIC_AXES = Self::Y.bits | Self::ANG_X.bits;
        /// The set of degrees of freedom locked by a fixed joint.
        const LOCKED_FIXED_AXES = Self::X.bits | Self::Y.bits | Self::ANG_X.bits;
        /// The set of degrees of freedom left free by a revolute joint.
        const FREE_REVOLUTE_AXES = Self::ANG_X.bits;
        /// The set of degrees of freedom left free by a prismatic joint.
        const FREE_PRISMATIC_AXES = Self::X.bits;
        /// The set of degrees of freedom left free by a fixed joint.
        const FREE_FIXED_AXES = 0;
        /// The set of all translational degrees of freedom.
        const LIN_AXES = Self::X.bits() | Self::Y.bits();
        /// The set of all angular degrees of freedom.
        const ANG_AXES = Self::ANG_X.bits();
    }
}

impl Default for JointAxesMask {
    fn default() -> Self {
        Self::empty()
    }
}

/// Identifiers of degrees of freedoms of a joint.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum JointAxis {
    /// The translational degree of freedom along the joint’s local X axis.
    X = 0,
    /// The translational degree of freedom along the joint’s local Y axis.
    Y,
    /// The translational degree of freedom along the joint’s local Z axis.
    #[cfg(feature = "dim3")]
    Z,
    /// The rotational degree of freedom along the joint’s local X axis.
    AngX,
    /// The rotational degree of freedom along the joint’s local Y axis.
    #[cfg(feature = "dim3")]
    AngY,
    /// The rotational degree of freedom along the joint’s local Z axis.
    #[cfg(feature = "dim3")]
    AngZ,
}

impl From<JointAxis> for JointAxesMask {
    fn from(axis: JointAxis) -> Self {
        JointAxesMask::from_bits(1 << axis as usize).unwrap()
    }
}

/// The limits of a joint along one of its degrees of freedom.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct JointLimits<N> {
    /// The minimum bound of the joint limit.
    pub min: N,
    /// The maximum bound of the joint limit.
    pub max: N,
    /// The impulse applied to enforce the joint’s limit.
    pub impulse: N,
}

impl<N: WReal> Default for JointLimits<N> {
    fn default() -> Self {
        Self {
            min: -N::splat(Real::MAX),
            max: N::splat(Real::MAX),
            impulse: N::splat(0.0),
        }
    }
}

/// A joint’s motor along one of its degrees of freedom.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct JointMotor {
    /// The target velocity of the motor.
    pub target_vel: Real,
    /// The target position of the motor.
    pub target_pos: Real,
    /// The stiffness coefficient of the motor’s spring-like equation.
    pub stiffness: Real,
    /// The damping coefficient of the motor’s spring-like equation.
    pub damping: Real,
    /// The maximum force this motor can deliver.
    pub max_force: Real,
    /// The impulse applied by this motor.
    pub impulse: Real,
    /// The spring-like model used for simulating this motor.
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
            model: MotorModel::AccelerationBased,
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

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Enum indicating whether or not a joint is enabled.
pub enum JointEnabled {
    /// The joint is enabled.
    Enabled,
    /// The joint wasn’t disabled by the user explicitly but it is attached to
    /// a disabled rigid-body.
    DisabledByAttachedBody,
    /// The joint is disabled by the user explicitly.
    Disabled,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
/// A generic joint.
pub struct GenericJoint {
    /// The joint’s frame, expressed in the first rigid-body’s local-space.
    pub local_frame1: Isometry<Real>,
    /// The joint’s frame, expressed in the second rigid-body’s local-space.
    pub local_frame2: Isometry<Real>,
    /// The degrees-of-freedoms locked by this joint.
    pub locked_axes: JointAxesMask,
    /// The degrees-of-freedoms limited by this joint.
    pub limit_axes: JointAxesMask,
    /// The degrees-of-freedoms motorised by this joint.
    pub motor_axes: JointAxesMask,
    /// The coupled degrees of freedom of this joint.
    pub coupled_axes: JointAxesMask,
    /// The limits, along each degrees of freedoms of this joint.
    ///
    /// Note that the limit must also be explicitly enabled by the `limit_axes` bitmask.
    pub limits: [JointLimits<Real>; SPATIAL_DIM],
    /// The motors, along each degrees of freedoms of this joint.
    ///
    /// Note that the mostor must also be explicitly enabled by the `motors` bitmask.
    pub motors: [JointMotor; SPATIAL_DIM],
    /// Are contacts between the attached rigid-bodies enabled?
    pub contacts_enabled: bool,
    /// Whether or not the joint is enabled.
    pub enabled: JointEnabled,
    /// User-defined data associated to this joint.
    pub user_data: u128,
}

impl Default for GenericJoint {
    fn default() -> Self {
        Self {
            local_frame1: Isometry::identity(),
            local_frame2: Isometry::identity(),
            locked_axes: JointAxesMask::empty(),
            limit_axes: JointAxesMask::empty(),
            motor_axes: JointAxesMask::empty(),
            coupled_axes: JointAxesMask::empty(),
            limits: [JointLimits::default(); SPATIAL_DIM],
            motors: [JointMotor::default(); SPATIAL_DIM],
            contacts_enabled: true,
            enabled: JointEnabled::Enabled,
            user_data: 0,
        }
    }
}

impl GenericJoint {
    /// Creates a new generic joint that locks the specified degrees of freedom.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        *Self::default().lock_axes(locked_axes)
    }

    #[cfg(feature = "simd-is-enabled")]
    /// Can this joint use SIMD-accelerated constraint formulations?
    pub(crate) fn supports_simd_constraints(&self) -> bool {
        self.limit_axes.is_empty() && self.motor_axes.is_empty()
    }

    #[doc(hidden)]
    pub fn complete_ang_frame(axis: UnitVector<Real>) -> Rotation<Real> {
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

    /// Is this joint enabled?
    pub fn is_enabled(&self) -> bool {
        self.enabled == JointEnabled::Enabled
    }

    /// Set whether this joint is enabled or not.
    pub fn set_enabled(&mut self, enabled: bool) {
        match self.enabled {
            JointEnabled::Enabled | JointEnabled::DisabledByAttachedBody => {
                if !enabled {
                    self.enabled = JointEnabled::Disabled;
                }
            }
            JointEnabled::Disabled => {
                if enabled {
                    self.enabled = JointEnabled::Enabled;
                }
            }
        }
    }

    /// Add the specified axes to the set of axes locked by this joint.
    pub fn lock_axes(&mut self, axes: JointAxesMask) -> &mut Self {
        self.locked_axes |= axes;
        self
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    pub fn set_local_frame1(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.local_frame1 = local_frame;
        self
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    pub fn set_local_frame2(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.local_frame2 = local_frame;
        self
    }

    /// The principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(&self) -> UnitVector<Real> {
        self.local_frame1 * Vector::x_axis()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_axis1(&mut self, local_axis: UnitVector<Real>) -> &mut Self {
        self.local_frame1.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    /// The principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(&self) -> UnitVector<Real> {
        self.local_frame2 * Vector::x_axis()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_axis2(&mut self, local_axis: UnitVector<Real>) -> &mut Self {
        self.local_frame2.rotation = Self::complete_ang_frame(local_axis);
        self
    }

    /// The anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.local_frame1.translation.vector.into()
    }

    /// Sets anchor of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.local_frame1.translation.vector = anchor1.coords;
        self
    }

    /// The anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.local_frame2.translation.vector.into()
    }

    /// Sets anchor of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.local_frame2.translation.vector = anchor2.coords;
        self
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.contacts_enabled = enabled;
        self
    }

    /// The joint limits along the specified axis.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        let i = axis as usize;
        if self.limit_axes.contains(axis.into()) {
            Some(&self.limits[i])
        } else {
            None
        }
    }

    /// Sets the joint limits along the specified axis.
    pub fn set_limits(&mut self, axis: JointAxis, limits: [Real; 2]) -> &mut Self {
        let i = axis as usize;
        self.limit_axes |= axis.into();
        self.limits[i].min = limits[0];
        self.limits[i].max = limits[1];
        self
    }

    /// The spring-like motor model along the specified axis of this joint.
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

    /// Sets the maximum force the motor can deliver along the specified axis.
    pub fn set_motor_max_force(&mut self, axis: JointAxis, max_force: Real) -> &mut Self {
        self.motors[axis as usize].max_force = max_force;
        self
    }

    /// The motor affecting the joint’s degree of freedom along the specified axis.
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
        /// Converts the joint to its specific variant, if it is one.
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

        /// Converts the joint to its specific mutable variant, if it is one.
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
    joint_conversion_methods!(
        as_rope,
        as_rope_mut,
        RopeJoint,
        JointAxesMask::FREE_FIXED_AXES
    );

    #[cfg(feature = "dim3")]
    joint_conversion_methods!(
        as_spherical,
        as_spherical_mut,
        SphericalJoint,
        JointAxesMask::LOCKED_SPHERICAL_AXES
    );
}

/// Create generic joints using the builder pattern.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GenericJointBuilder(pub GenericJoint);

impl GenericJointBuilder {
    /// Creates a new generic joint builder.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self(GenericJoint::new(locked_axes))
    }

    /// Sets the degrees of freedom locked by the joint.
    #[must_use]
    pub fn locked_axes(mut self, axes: JointAxesMask) -> Self {
        self.0.locked_axes = axes;
        self
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.contacts_enabled = enabled;
        self
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame1(local_frame);
        self
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame2(local_frame);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(mut self, local_axis: UnitVector<Real>) -> Self {
        self.0.set_local_axis1(local_axis);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(mut self, local_axis: UnitVector<Real>) -> Self {
        self.0.set_local_axis2(local_axis);
        self
    }

    /// Sets the anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets the joint limits along the specified axis.
    #[must_use]
    pub fn limits(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.0.set_limits(axis, limits);
        self
    }

    /// Sets the coupled degrees of freedom for this joint’s limits and motor.
    #[must_use]
    pub fn coupled_axes(mut self, axes: JointAxesMask) -> Self {
        self.0.coupled_axes = axes;
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

    /// Sets the maximum force the motor can deliver along the specified axis.
    #[must_use]
    pub fn motor_max_force(mut self, axis: JointAxis, max_force: Real) -> Self {
        self.0.set_motor_max_force(axis, max_force);
        self
    }

    /// An arbitrary user-defined 128-bit integer associated to the joints built by this builder.
    pub fn user_data(mut self, data: u128) -> Self {
        self.0.user_data = data;
        self
    }

    /// Builds the generic joint.
    #[must_use]
    pub fn build(self) -> GenericJoint {
        self.0
    }
}

impl Into<GenericJoint> for GenericJointBuilder {
    fn into(self) -> GenericJoint {
        self.0
    }
}
