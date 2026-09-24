use crate::dynamics::{FixedJoint, PrismaticJoint, RevoluteJoint, RopeJoint, TypedJoint};
use crate::math::{Real, Rot, Vect};
use rapier::dynamics::{
    GenericJoint as RapierGenericJoint, JointAxesMask, JointAxis, JointLimits, JointMotor,
    MotorModel, SpringCoefficients,
};
use rapier::math::Pose;

#[cfg(feature = "dim2")]
use crate::dynamics::PinSlotJoint;

#[cfg(feature = "dim3")]
use crate::dynamics::SphericalJoint;

/// The description of any joint.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[repr(transparent)]
pub struct GenericJoint {
    /// The raw Rapier description of the joint.
    pub raw: RapierGenericJoint,
}

impl GenericJoint {
    /// Converts this joint into a Rapier joint.
    pub fn into_rapier(self) -> RapierGenericJoint {
        self.raw
    }
}

/*
 * NOTE: the following are copy-pasted from Rapier’s GenericJoint, to match its
 *       construction methods, but using glam types.
 */

impl GenericJoint {
    /// Creates a new generic joint that locks the specified degrees of freedom.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self {
            raw: RapierGenericJoint::new(locked_axes),
        }
    }

    /// The set of axes locked by this joint.
    pub fn locked_axes(&self) -> JointAxesMask {
        self.raw.locked_axes
    }

    /// Add the specified axes to the set of axes locked by this joint.
    pub fn lock_axes(&mut self, axes: JointAxesMask) -> &mut Self {
        self.raw.lock_axes(axes);
        self
    }

    /// The basis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(&self) -> Rot {
        #[cfg(feature = "dim2")]
        return self.raw.local_frame1.rotation.angle();
        #[cfg(feature = "dim3")]
        return self.raw.local_frame1.rotation;
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    pub fn set_local_basis1(&mut self, local_basis: Rot) -> &mut Self {
        #[cfg(feature = "dim2")]
        {
            self.raw.local_frame1.rotation = rapier::math::Rot2::new(local_basis);
        }
        #[cfg(feature = "dim3")]
        {
            self.raw.local_frame1.rotation = local_basis;
        }
        self
    }

    /// The basis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(&self) -> Rot {
        #[cfg(feature = "dim2")]
        return self.raw.local_frame2.rotation.angle();
        #[cfg(feature = "dim3")]
        return self.raw.local_frame2.rotation;
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    pub fn set_local_basis2(&mut self, local_basis: Rot) -> &mut Self {
        #[cfg(feature = "dim2")]
        {
            self.raw.local_frame2.rotation = rapier::math::Rot2::new(local_basis);
        }
        #[cfg(feature = "dim3")]
        {
            self.raw.local_frame2.rotation = local_basis;
        }
        self
    }

    /// The principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(&self) -> Vect {
        self.raw.local_axis1()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_axis1(&mut self, local_axis: Vect) -> &mut Self {
        self.raw.set_local_axis1(local_axis.normalize());
        self
    }

    /// The principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(&self) -> Vect {
        self.raw.local_axis2()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_axis2(&mut self, local_axis: Vect) -> &mut Self {
        self.raw.set_local_axis2(local_axis.normalize());
        self
    }

    /// The anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(&self) -> Vect {
        self.raw.local_anchor1()
    }

    /// Sets anchor of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_anchor1(&mut self, anchor1: Vect) -> &mut Self {
        self.raw.set_local_anchor1(anchor1);
        self
    }

    /// The anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(&self) -> Vect {
        self.raw.local_anchor2()
    }

    /// Sets anchor of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_anchor2(&mut self, anchor2: Vect) -> &mut Self {
        self.raw.set_local_anchor2(anchor2);
        self
    }

    /// The joint’s frame (anchor and basis), expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(&self) -> Pose {
        self.raw.local_frame1
    }

    /// Sets the joint’s frame (anchor and basis), expressed in the first rigid-body’s local-space.
    pub fn set_local_frame1(&mut self, local_frame: Pose) -> &mut Self {
        self.raw.set_local_frame1(local_frame);
        self
    }

    /// The joint’s frame (anchor and basis), expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(&self) -> Pose {
        self.raw.local_frame2
    }

    /// Sets the joint’s frame (anchor and basis), expressed in the second rigid-body’s local-space.
    pub fn set_local_frame2(&mut self, local_frame: Pose) -> &mut Self {
        self.raw.set_local_frame2(local_frame);
        self
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.raw.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.raw.set_contacts_enabled(enabled);
        self
    }

    /// Is this joint enabled?
    ///
    /// This only reflects the value set on this joint description. See
    /// [`ImpulseJointDisabled`](crate::dynamics::ImpulseJointDisabled) for a marker component
    /// disabling an impulse joint.
    pub fn is_enabled(&self) -> bool {
        self.raw.is_enabled()
    }

    /// Sets whether this joint is enabled.
    ///
    /// A disabled impulse joint is ignored by the solver but stays attached to its rigid-bodies.
    /// Multibody joints ignore this flag, except for filtering contacts between attached bodies.
    pub fn set_enabled(&mut self, enabled: bool) -> &mut Self {
        self.raw.set_enabled(enabled);
        self
    }

    /// The spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(&self) -> SpringCoefficients<Real> {
        self.raw.softness
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    pub fn set_softness(&mut self, softness: SpringCoefficients<Real>) -> &mut Self {
        self.raw.softness = softness;
        self
    }

    /// Flips the orientation of the joint, swapping its two frames and negating its limits and motor targets.
    ///
    /// This is useful to keep the joint’s behavior unchanged after swapping its two attached rigid-bodies.
    pub fn flip(&mut self) -> &mut Self {
        self.raw.flip();
        self
    }

    /// The joint limits along the specified axis.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        self.raw.limits(axis)
    }

    /// Sets the joint limits along the specified axis.
    pub fn set_limits(&mut self, axis: JointAxis, limits: [Real; 2]) -> &mut Self {
        self.raw.set_limits(axis, limits);
        self
    }

    /// Sets the coupled degrees of freedom for this joint’s limits and motor.
    pub fn set_coupled_axes(&mut self, axes: JointAxesMask) -> &mut Self {
        self.raw.coupled_axes = axes;
        self
    }

    /// The spring-like motor model along the specified axis of this joint.
    #[must_use]
    pub fn motor_model(&self, axis: JointAxis) -> Option<MotorModel> {
        self.raw.motor_model(axis)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, axis: JointAxis, model: MotorModel) -> &mut Self {
        self.raw.set_motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(
        &mut self,
        axis: JointAxis,
        target_vel: Real,
        factor: Real,
    ) -> &mut Self {
        self.raw.set_motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.raw
            .set_motor_position(axis, target_pos, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver along the specified axis.
    pub fn set_motor_max_force(&mut self, axis: JointAxis, max_force: Real) -> &mut Self {
        self.raw.set_motor_max_force(axis, max_force);
        self
    }

    /// The motor affecting the joint’s degree of freedom along the specified axis.
    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        self.raw.motor(axis)
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
        self.raw
            .set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }
}

macro_rules! joint_conversion_methods(
    ($as_joint: ident, $as_joint_mut: ident, $Joint: ty, $axes: expr) => {
        /// Converts the joint to its specific variant, if it is one.
        #[must_use]
        pub fn $as_joint(&self) -> Option<&$Joint> {
            if self.locked_axes() == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute::<&Self, &$Joint>(self) })
            } else {
                None
            }
        }

        /// Converts the joint to its specific mutable variant, if it is one.
        #[must_use]
        pub fn $as_joint_mut(&mut self) -> Option<&mut $Joint> {
            if self.locked_axes() == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute::<&mut Self, &mut $Joint>(self) })
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

    #[cfg(feature = "dim2")]
    joint_conversion_methods!(
        as_pin_slot,
        as_pin_slot_mut,
        PinSlotJoint,
        JointAxesMask::LOCKED_PIN_SLOT_AXES
    );
}

/// Create generic joints using the builder pattern.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub struct GenericJointBuilder(GenericJoint);

impl GenericJointBuilder {
    /// Creates a new generic joint builder.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self(GenericJoint::new(locked_axes))
    }

    /// Sets the degrees of freedom locked by the joint.
    #[must_use]
    pub fn locked_axes(mut self, axes: JointAxesMask) -> Self {
        self.0.lock_axes(axes);
        self
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis1(local_basis);
        self
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis2(local_basis);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(mut self, local_axis: Vect) -> Self {
        self.0.set_local_axis1(local_axis);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(mut self, local_axis: Vect) -> Self {
        self.0.set_local_axis2(local_axis);
        self
    }

    /// Sets the anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Vect) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Vect) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets the joint’s frame (anchor and basis), expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(mut self, local_frame: Pose) -> Self {
        self.0.set_local_frame1(local_frame);
        self
    }

    /// Sets the joint’s frame (anchor and basis), expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(mut self, local_frame: Pose) -> Self {
        self.0.set_local_frame2(local_frame);
        self
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets whether the joint is enabled.
    #[must_use]
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.0.set_enabled(enabled);
        self
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(mut self, softness: SpringCoefficients<Real>) -> Self {
        self.0.set_softness(softness);
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
        self.0.set_coupled_axes(axes);
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

    /// Builds the generic joint.
    #[must_use]
    pub fn build(self) -> GenericJoint {
        self.0
    }
}

impl From<GenericJointBuilder> for GenericJoint {
    fn from(joint: GenericJointBuilder) -> GenericJoint {
        joint.0
    }
}

impl From<GenericJointBuilder> for TypedJoint {
    fn from(joint: GenericJointBuilder) -> TypedJoint {
        joint.0.into()
    }
}

impl From<GenericJoint> for TypedJoint {
    fn from(joint: GenericJoint) -> TypedJoint {
        TypedJoint::GenericJoint(joint)
    }
}
