use crate::dynamics::joint::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::dynamics::{JointAxis, JointMotor, MotorModel};
use crate::math::{Isometry, Point, Real};

use super::JointLimits;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A spherical joint, locks all relative translations between two bodies.
pub struct SphericalJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

impl Default for SphericalJoint {
    fn default() -> Self {
        SphericalJoint::new()
    }
}

impl SphericalJoint {
    /// Creates a new spherical joint locking all relative translations between two bodies.
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES).build();
        Self { data }
    }

    /// The underlying generic joint.
    pub fn data(&self) -> &GenericJoint {
        &self.data
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }

    /// Gets both the joint anchor and the joint’s reference orientation relative to the first
    /// rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(&self) -> &Isometry<Real> {
        &self.data.local_frame1
    }

    /// Sets both the joint anchor and the joint’s reference orientation relative to the first
    /// rigid-body’s local-space.
    pub fn set_local_frame1(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame1(local_frame);
        self
    }

    /// Gets both the joint anchor and the joint’s reference orientation relative to the second
    /// rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(&self) -> &Isometry<Real> {
        &self.data.local_frame2
    }

    /// Sets both the joint anchor and the joint’s reference orientation relative to the second
    /// rigid-body’s local-space.
    pub fn set_local_frame2(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame2(local_frame);
        self
    }

    /// The motor affecting the joint’s rotational degree of freedom along the specified axis.
    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        self.data.motor(axis)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, axis: JointAxis, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(
        &mut self,
        axis: JointAxis,
        target_vel: Real,
        factor: Real,
    ) -> &mut Self {
        self.data.set_motor_velocity(axis, target_vel, factor);
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
        self.data
            .set_motor_position(axis, target_pos, stiffness, damping);
        self
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
        self.data
            .set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver along the specified axis.
    pub fn set_motor_max_force(&mut self, axis: JointAxis, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(axis, max_force);
        self
    }

    /// The limit distance attached bodies can translate along the specified axis.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        self.data.limits(axis)
    }

    /// Sets the `[min,max]` limit angles attached bodies can translate along the joint’s principal
    /// axis.
    pub fn set_limits(&mut self, axis: JointAxis, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(axis, limits);
        self
    }
}

impl From<SphericalJoint> for GenericJoint {
    fn from(val: SphericalJoint) -> GenericJoint {
        val.data
    }
}

/// Create spherical joints using the builder pattern.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct SphericalJointBuilder(pub SphericalJoint);

impl SphericalJointBuilder {
    /// Creates a new builder for spherical joints.
    pub fn new() -> Self {
        Self(SphericalJoint::new())
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets both the joint anchor and the joint’s reference orientation relative to the first
    /// rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(mut self, frame1: Isometry<Real>) -> Self {
        self.0.set_local_frame1(frame1);
        self
    }

    /// Sets both the joint anchor and the joint’s reference orientation relative to the second
    /// rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(mut self, frame2: Isometry<Real>) -> Self {
        self.0.set_local_frame2(frame2);
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
    pub fn motor(
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

    /// Sets the `[min,max]` limit distances attached bodies can rotate along the specified axis.
    #[must_use]
    pub fn limits(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.0.set_limits(axis, limits);
        self
    }

    /// Builds the spherical joint.
    #[must_use]
    pub fn build(self) -> SphericalJoint {
        self.0
    }
}

impl From<SphericalJointBuilder> for GenericJoint {
    fn from(val: SphericalJointBuilder) -> GenericJoint {
        val.0.into()
    }
}
