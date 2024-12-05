use crate::dynamics::joint::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::dynamics::{JointAxis, JointLimits, JointMotor, MotorModel};
use crate::math::{Point, Real, Rotation};

#[cfg(feature = "dim3")]
use crate::math::UnitVector;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A revolute joint, locks all relative motion except for rotation along the joint’s principal axis.
pub struct RevoluteJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

impl RevoluteJoint {
    /// Creates a new revolute joint allowing only relative rotations.
    #[cfg(feature = "dim2")]
    #[allow(clippy::new_without_default)] // For symmetry with 3D which can’t have a Default impl.
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES);
        Self { data: data.build() }
    }

    /// Creates a new revolute joint allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: UnitVector<Real>) -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_axis1(axis)
            .local_axis2(axis)
            .build();
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

    /// The angle along the free degree of freedom of this revolute joint in `[-π, π]`.
    ///
    /// # Parameters
    /// - `rb_rot1`: the rotation of the first rigid-body attached to this revolute joint.
    /// - `rb_rot2`: the rotation of the second rigid-body attached to this revolute joint.
    pub fn angle(&self, rb_rot1: &Rotation<Real>, rb_rot2: &Rotation<Real>) -> Real {
        let joint_rot1 = rb_rot1 * self.data.local_frame1.rotation;
        let joint_rot2 = rb_rot2 * self.data.local_frame2.rotation;
        let ang_err = joint_rot1.inverse() * joint_rot2;

        #[cfg(feature = "dim3")]
        if joint_rot1.dot(&joint_rot2) < 0.0 {
            -ang_err.i.clamp(-1.0, 1.0).asin() * 2.0
        } else {
            ang_err.i.clamp(-1.0, 1.0).asin() * 2.0
        }

        #[cfg(feature = "dim2")]
        {
            ang_err.angle()
        }
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
}

impl From<RevoluteJoint> for GenericJoint {
    fn from(val: RevoluteJoint) -> GenericJoint {
        val.data
    }
}

/// Create revolute joints using the builder pattern.
///
/// A revolute joint locks all relative motion except for rotations along the joint’s principal axis.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RevoluteJointBuilder(pub RevoluteJoint);

impl RevoluteJointBuilder {
    /// Creates a new revolute joint builder.
    #[cfg(feature = "dim2")]
    #[allow(clippy::new_without_default)] // For symmetry with 3D which can’t have a Default impl.
    pub fn new() -> Self {
        Self(RevoluteJoint::new())
    }

    /// Creates a new revolute joint builder, allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: UnitVector<Real>) -> Self {
        Self(RevoluteJoint::new(axis))
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

    /// Builds the revolute joint.
    #[must_use]
    pub fn build(self) -> RevoluteJoint {
        self.0
    }
}

impl From<RevoluteJointBuilder> for GenericJoint {
    fn from(val: RevoluteJointBuilder) -> GenericJoint {
        val.0.into()
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn test_revolute_joint_angle() {
        use crate::math::{Real, Rotation};
        use crate::na::RealField;
        #[cfg(feature = "dim3")]
        use crate::{math::Vector, na::vector};

        #[cfg(feature = "dim2")]
        let revolute = super::RevoluteJointBuilder::new().build();
        #[cfg(feature = "dim2")]
        let rot1 = Rotation::new(1.0);
        #[cfg(feature = "dim3")]
        let revolute = super::RevoluteJointBuilder::new(Vector::y_axis()).build();
        #[cfg(feature = "dim3")]
        let rot1 = Rotation::new(vector![0.0, 1.0, 0.0]);

        let steps = 100;

        // The -pi and pi values will be checked later.
        for i in 1..steps {
            let delta = -Real::pi() + i as Real * Real::two_pi() / steps as Real;
            #[cfg(feature = "dim2")]
            let rot2 = Rotation::new(1.0 + delta);
            #[cfg(feature = "dim3")]
            let rot2 = Rotation::new(vector![0.0, 1.0 + delta, 0.0]);
            approx::assert_relative_eq!(revolute.angle(&rot1, &rot2), delta, epsilon = 1.0e-5);
        }

        // Check the special case for -pi and pi that may return an angle with a flipped sign
        // (because they are equivalent).
        for delta in [-Real::pi(), Real::pi()] {
            #[cfg(feature = "dim2")]
            let rot2 = Rotation::new(1.0 + delta);
            #[cfg(feature = "dim3")]
            let rot2 = Rotation::new(vector![0.0, 1.0 + delta, 0.0]);
            approx::assert_relative_eq!(
                revolute.angle(&rot1, &rot2).abs(),
                delta.abs(),
                epsilon = 1.0e-2
            );
        }
    }
}
