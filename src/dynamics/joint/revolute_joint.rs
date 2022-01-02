use crate::dynamics::joint::{JointAxesMask, JointData};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Point, Real};

#[cfg(feature = "dim3")]
use crate::math::UnitVector;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RevoluteJoint {
    data: JointData,
}

impl RevoluteJoint {
    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        let mask = JointAxesMask::X | JointAxesMask::Y;

        let data = JointData::default().lock_axes(mask);
        Self { data }
    }

    #[cfg(feature = "dim3")]
    pub fn new(axis: UnitVector<Real>) -> Self {
        let mask = JointAxesMask::X
            | JointAxesMask::Y
            | JointAxesMask::Z
            | JointAxesMask::ANG_Y
            | JointAxesMask::ANG_Z;

        let data = JointData::default()
            .lock_axes(mask)
            .local_axis1(axis)
            .local_axis2(axis);
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
    pub fn motor_model(mut self, model: MotorModel) -> Self {
        self.data = self.data.motor_model(JointAxis::AngX, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn motor_velocity(mut self, target_vel: Real, factor: Real) -> Self {
        self.data = self
            .data
            .motor_velocity(JointAxis::AngX, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn motor_position(mut self, target_pos: Real, stiffness: Real, damping: Real) -> Self {
        self.data = self
            .data
            .motor_position(JointAxis::AngX, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn motor_axis(
        mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.data =
            self.data
                .motor_axis(JointAxis::AngX, target_pos, target_vel, stiffness, damping);
        self
    }

    pub fn motor_max_impulse(mut self, max_impulse: Real) -> Self {
        self.data = self.data.motor_max_impulse(JointAxis::AngX, max_impulse);
        self
    }

    #[must_use]
    pub fn limit_axis(mut self, limits: [Real; 2]) -> Self {
        self.data = self.data.limit_axis(JointAxis::AngX, limits);
        self
    }
}

impl Into<JointData> for RevoluteJoint {
    fn into(self) -> JointData {
        self.data
    }
}
