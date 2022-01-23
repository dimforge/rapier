use crate::dynamics::joint::{JointAxesMask, JointData};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Point, Real};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SphericalJoint {
    data: JointData,
}

impl Default for SphericalJoint {
    fn default() -> Self {
        SphericalJoint::new()
    }
}

impl SphericalJoint {
    pub fn new() -> Self {
        let data =
            JointData::default().lock_axes(JointAxesMask::X | JointAxesMask::Y | JointAxesMask::Z);
        Self { data }
    }

    pub fn data(&self) -> &JointData {
        &self.data
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.data = self.data.local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.data = self.data.local_anchor2(anchor2);
        self
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn motor_model(mut self, axis: JointAxis, model: MotorModel) -> Self {
        self.data = self.data.motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn motor_velocity(mut self, axis: JointAxis, target_vel: Real, factor: Real) -> Self {
        self.data = self.data.motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn motor_position(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.data = self
            .data
            .motor_position(axis, target_pos, stiffness, damping);
        self
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
        self.data = self
            .data
            .motor_axis(axis, target_pos, target_vel, stiffness, damping);
        self
    }

    pub fn motor_max_impulse(mut self, axis: JointAxis, max_impulse: Real) -> Self {
        self.data = self.data.motor_max_impulse(axis, max_impulse);
        self
    }

    #[must_use]
    pub fn limit_axis(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.data = self.data.limit_axis(axis, limits);
        self
    }
}

impl Into<JointData> for SphericalJoint {
    fn into(self) -> JointData {
        self.data
    }
}
