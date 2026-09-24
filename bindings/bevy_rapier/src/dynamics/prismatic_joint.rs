use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{Real, Vect};
use rapier::dynamics::{JointAxesMask, JointAxis, JointLimits, JointMotor, MotorModel};

use super::{SpringCoefficients, TypedJoint};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A prismatic joint, locks all relative motion between two bodies except for translation along the joint’s principal axis.
pub struct PrismaticJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

impl PrismaticJoint {
    /// Creates a new prismatic joint allowing only relative translations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    pub fn new(axis: Vect) -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_PRISMATIC_AXES)
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

    /// The principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(&self) -> Vect {
        self.data.local_axis1()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    pub fn set_local_axis1(&mut self, axis1: Vect) -> &mut Self {
        self.data.set_local_axis1(axis1);
        self
    }

    /// The principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(&self) -> Vect {
        self.data.local_axis2()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    pub fn set_local_axis2(&mut self, axis2: Vect) -> &mut Self {
        self.data.set_local_axis2(axis2);
        self
    }

    /// The motor affecting the joint’s translational degree of freedom.
    #[must_use]
    pub fn motor(&self) -> Option<&JointMotor> {
        self.data.motor(JointAxis::LinX)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::LinX, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::LinX, target_vel, factor);
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
            .set_motor_position(JointAxis::LinX, target_pos, stiffness, damping);
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
            .set_motor(JointAxis::LinX, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::LinX, max_force);
        self
    }

    /// The limit distance attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(&self) -> Option<&JointLimits<Real>> {
        self.data.limits(JointAxis::LinX)
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate along the joint’s principal axis.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::LinX, limits);
        self
    }
}

impl From<PrismaticJoint> for GenericJoint {
    fn from(joint: PrismaticJoint) -> GenericJoint {
        joint.data
    }
}

/// Create prismatic joints using the builder pattern.
///
/// A prismatic joint locks all relative motion except for translations along the joint’s principal axis.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PrismaticJointBuilder(PrismaticJoint);

impl PrismaticJointBuilder {
    /// Creates a new builder for prismatic joints.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    pub fn new(axis: Vect) -> Self {
        Self(PrismaticJoint::new(axis))
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

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(mut self, axis1: Vect) -> Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
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
    pub fn set_motor(
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

    /// Sets the `[min,max]` limit distances attached bodies can translate along the joint’s principal axis.
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

    /// Builds the prismatic joint.
    #[must_use]
    pub fn build(self) -> PrismaticJoint {
        self.0
    }
}

impl From<PrismaticJointBuilder> for TypedJoint {
    fn from(joint: PrismaticJointBuilder) -> TypedJoint {
        joint.0.into()
    }
}

impl From<PrismaticJoint> for TypedJoint {
    fn from(joint: PrismaticJoint) -> TypedJoint {
        TypedJoint::PrismaticJoint(joint)
    }
}
