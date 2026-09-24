use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{Real, Vect};
use crate::plugin::context::RapierRigidBodySet;
use bevy::prelude::Entity;
use rapier::dynamics::{
    JointAxesMask, JointAxis, JointLimits, JointMotor, MotorModel, RigidBodyHandle, RigidBodySet,
};

#[cfg(doc)]
use crate::prelude::RapierContext;

use super::{SpringCoefficients, TypedJoint};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A revolute joint, locks all relative motion except for rotation along the joint’s principal axis.
pub struct RevoluteJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

#[cfg(feature = "dim2")]
impl Default for RevoluteJoint {
    fn default() -> Self {
        Self::new()
    }
}

impl RevoluteJoint {
    /// Creates a new revolute joint allowing only relative rotations.
    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES);
        Self { data: data.build() }
    }

    /// Creates a new revolute joint allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: Vect) -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_axis1(axis)
            .local_axis2(axis)
            .build();
        Self { data }
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled()
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(&self) -> SpringCoefficients<Real> {
        self.data.softness()
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    pub fn set_softness(&mut self, softness: SpringCoefficients<Real>) -> &mut Self {
        self.data.set_softness(softness);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Vect {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Vect) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Vect {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Vect) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }

    /// Sets the joint's rotation axis in the local-space of the first rigid-body
    pub fn set_local_axis1(&mut self, axis1: Vect) -> &mut Self {
        self.data.set_local_axis1(axis1);
        self
    }

    /// Sets the joint's rotation axis in the local-space of the second rigid-body
    pub fn set_local_axis2(&mut self, axis2: Vect) -> &mut Self {
        self.data.set_local_axis2(axis2);
        self
    }

    /// The motor affecting the joint’s rotational degree of freedom.
    #[must_use]
    pub fn motor(&self) -> Option<&JointMotor> {
        self.data.motor(JointAxis::AngX)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::AngX, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::AngX, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor_position(JointAxis::AngX, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor(JointAxis::AngX, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::AngX, max_force);
        self
    }

    /// The limit angle attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(&self) -> Option<&JointLimits<Real>> {
        self.data.limits(JointAxis::AngX)
    }

    /// Sets the `[min,max]` limit angle attached bodies can translate along the joint’s principal axis.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::AngX, limits);
        self
    }

    /// The angle along the free degree of freedom of this revolute joint in `[-π, π]`.
    ///
    /// See also [`Self::angle`] for a version of this method taking entities instead of rigid-body handles.
    /// Similarly [`RapierContext::impulse_revolute_joint_angle`] only takes a single entity as argument to compute that angle.
    ///
    /// # Parameters
    /// - `bodies` : the rigid body set from [`RapierRigidBodySet`]
    /// - `body1`: the first rigid-body attached to this revolute joint, obtained through [`rapier::dynamics::ImpulseJoint`] or [`rapier::dynamics::MultibodyJoint`].
    /// - `body2`: the second rigid-body attached to this revolute joint, obtained through [`rapier::dynamics::ImpulseJoint`] or [`rapier::dynamics::MultibodyJoint`].
    pub fn angle_from_handles(
        &self,
        bodies: &RigidBodySet,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ) -> f32 {
        // NOTE: unwrap will always succeed since `Self` is known to be a revolute joint.
        let joint = self.data.raw.as_revolute().unwrap();

        let rb1 = &bodies[body1];
        let rb2 = &bodies[body2];
        joint.angle(rb1.rotation(), rb2.rotation())
    }

    /// The angle along the free degree of freedom of this revolute joint in `[-π, π]`.
    ///
    /// # Parameters
    /// - `bodies` : the rigid body set from [`RapierRigidBodySet`]
    /// - `body1`: the first rigid-body attached to this revolute joint.
    /// - `body2`: the second rigid-body attached to this revolute joint.
    pub fn angle(&self, rigidbody_set: &RapierRigidBodySet, body1: Entity, body2: Entity) -> f32 {
        let rb1 = rigidbody_set.entity2body().get(&body1).unwrap();
        let rb2 = rigidbody_set.entity2body().get(&body2).unwrap();
        self.angle_from_handles(&rigidbody_set.bodies, *rb1, *rb2)
    }
}

impl From<RevoluteJoint> for GenericJoint {
    fn from(joint: RevoluteJoint) -> GenericJoint {
        joint.data
    }
}

/// Create revolute joints using the builder pattern.
///
/// A revolute joint locks all relative motion except for rotations along the joint’s principal axis.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RevoluteJointBuilder(RevoluteJoint);

#[cfg(feature = "dim2")]
impl Default for RevoluteJointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RevoluteJointBuilder {
    /// Creates a new revolute joint builder.
    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        Self(RevoluteJoint::new())
    }

    /// Creates a new revolute joint builder, allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: Vect) -> Self {
        Self(RevoluteJoint::new(axis))
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Vect) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Vect) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets the joint's axis, expressed in the local-space of the first rigid-body
    pub fn local_axis1(mut self, axis1: Vect) -> Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the joint's axis, expressed in the local-space of the second rigid-body
    pub fn local_axis2(mut self, axis2: Vect) -> Self {
        self.0.set_local_axis2(axis2);
        self
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, model: MotorModel) -> Self {
        self.0.set_motor_model(model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    #[must_use]
    pub fn motor_position(mut self, target_pos: Real, stiffness: Real, damping: Real) -> Self {
        self.0.set_motor_position(target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    #[must_use]
    pub fn motor(
        mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0.set_motor(target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    #[must_use]
    pub fn motor_max_force(mut self, max_force: Real) -> Self {
        self.0.set_motor_max_force(max_force);
        self
    }

    /// Sets the `[min,max]` limit angles attached bodies can rotate along the joint’s principal axis.
    #[must_use]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.set_limits(limits);
        self
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(mut self, softness: SpringCoefficients<Real>) -> Self {
        self.0.set_softness(softness);
        self
    }

    /// Builds the revolute joint.
    #[must_use]
    pub fn build(self) -> RevoluteJoint {
        self.0
    }
}

impl From<RevoluteJointBuilder> for TypedJoint {
    fn from(joint: RevoluteJointBuilder) -> TypedJoint {
        joint.0.into()
    }
}

impl From<RevoluteJoint> for TypedJoint {
    fn from(joint: RevoluteJoint) -> TypedJoint {
        TypedJoint::RevoluteJoint(joint)
    }
}
