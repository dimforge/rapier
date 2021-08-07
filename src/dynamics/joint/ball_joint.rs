use crate::dynamics::SpringModel;
use crate::math::{Point, Real, Rotation, UnitVector, Vector};

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that removes all relative linear motion between a pair of points on two bodies.
pub struct BallJoint {
    /// Where the ball joint is attached on the first body, expressed in the first body local frame.
    pub local_anchor1: Point<Real>,
    /// Where the ball joint is attached on the second body, expressed in the second body local frame.
    pub local_anchor2: Point<Real>,
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    pub impulse: Vector<Real>,

    /// The target relative angular velocity the motor will attempt to reach.
    #[cfg(feature = "dim2")]
    pub motor_target_vel: Real,
    /// The target relative angular velocity the motor will attempt to reach.
    #[cfg(feature = "dim3")]
    pub motor_target_vel: Vector<Real>,
    /// The target angular position of this joint, expressed as an axis-angle.
    pub motor_target_pos: Rotation<Real>,
    /// The motor's stiffness.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_stiffness: Real,
    /// The motor's damping.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_damping: Real,
    /// The maximal impulse the motor is able to deliver.
    pub motor_max_impulse: Real,
    /// The angular impulse applied by the motor.
    #[cfg(feature = "dim2")]
    pub motor_impulse: Real,
    /// The angular impulse applied by the motor.
    #[cfg(feature = "dim3")]
    pub motor_impulse: Vector<Real>,
    /// The spring-like model used by the motor to reach the target velocity and .
    pub motor_model: SpringModel,

    /// Are the limits enabled for this joint?
    pub limits_enabled: bool,
    /// The axis of the limit cone for this joint, if the local-space of the first body.
    pub limits_local_axis1: UnitVector<Real>,
    /// The axis of the limit cone for this joint, if the local-space of the first body.
    pub limits_local_axis2: UnitVector<Real>,
    /// The maximum angle allowed between the two limit axes in world-space.
    pub limits_angle: Real,
    /// The impulse applied to enforce joint limits.
    pub limits_impulse: Real,
}

impl BallJoint {
    /// Creates a new Ball joint from two anchors given on the local spaces of the respective bodies.
    pub fn new(local_anchor1: Point<Real>, local_anchor2: Point<Real>) -> Self {
        Self::with_impulse(local_anchor1, local_anchor2, Vector::zeros())
    }

    pub(crate) fn with_impulse(
        local_anchor1: Point<Real>,
        local_anchor2: Point<Real>,
        impulse: Vector<Real>,
    ) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            impulse,
            motor_target_vel: na::zero(),
            motor_target_pos: Rotation::identity(),
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_impulse: na::zero(),
            motor_max_impulse: Real::MAX,
            motor_model: SpringModel::default(),
            limits_enabled: false,
            limits_local_axis1: Vector::x_axis(),
            limits_local_axis2: Vector::x_axis(),
            limits_angle: Real::MAX,
            limits_impulse: 0.0,
        }
    }

    /// Can a SIMD constraint be used for resolving this joint?
    pub fn supports_simd_constraints(&self) -> bool {
        // SIMD ball constraints don't support motors and limits right now.
        !self.limits_enabled
            && (self.motor_max_impulse == 0.0
                || (self.motor_stiffness == 0.0 && self.motor_damping == 0.0))
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn configure_motor_model(&mut self, model: SpringModel) {
        self.motor_model = model;
    }

    /// Sets the target velocity and velocity correction factor this motor.
    #[cfg(feature = "dim2")]
    pub fn configure_motor_velocity(&mut self, target_vel: Real, factor: Real) {
        self.configure_motor(self.motor_target_pos, target_vel, 0.0, factor)
    }

    /// Sets the target velocity and velocity correction factor this motor.
    #[cfg(feature = "dim3")]
    pub fn configure_motor_velocity(&mut self, target_vel: Vector<Real>, factor: Real) {
        self.configure_motor(self.motor_target_pos, target_vel, 0.0, factor)
    }

    /// Sets the target orientation this motor needs to reach.
    pub fn configure_motor_position(
        &mut self,
        target_pos: Rotation<Real>,
        stiffness: Real,
        damping: Real,
    ) {
        self.configure_motor(target_pos, na::zero(), stiffness, damping)
    }

    /// Sets the target orientation this motor needs to reach.
    #[cfg(feature = "dim2")]
    pub fn configure_motor(
        &mut self,
        target_pos: Rotation<Real>,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) {
        self.motor_target_vel = target_vel;
        self.motor_target_pos = target_pos;
        self.motor_stiffness = stiffness;
        self.motor_damping = damping;
    }

    /// Configure both the target orientation and target velocity of the motor.
    #[cfg(feature = "dim3")]
    pub fn configure_motor(
        &mut self,
        target_pos: Rotation<Real>,
        target_vel: Vector<Real>,
        stiffness: Real,
        damping: Real,
    ) {
        self.motor_target_vel = target_vel;
        self.motor_target_pos = target_pos;
        self.motor_stiffness = stiffness;
        self.motor_damping = damping;
    }
}
