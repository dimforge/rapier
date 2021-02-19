use crate::dynamics::SpringModel;
use crate::math::{Isometry, Point, Real, Vector};
use crate::utils::WBasis;
use na::{RealField, Unit, Vector5};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that removes all relative motion between two bodies, except for the rotations along one axis.
pub struct RevoluteJoint {
    /// Where the revolute joint is attached on the first body, expressed in the local space of the first attached body.
    pub local_anchor1: Point<Real>,
    /// Where the revolute joint is attached on the second body, expressed in the local space of the second attached body.
    pub local_anchor2: Point<Real>,
    /// The rotation axis of this revolute joint expressed in the local space of the first attached body.
    pub local_axis1: Unit<Vector<Real>>,
    /// The rotation axis of this revolute joint expressed in the local space of the second attached body.
    pub local_axis2: Unit<Vector<Real>>,
    /// The basis orthonormal to `local_axis1`, expressed in the local space of the first attached body.
    pub basis1: [Vector<Real>; 2],
    /// The basis orthonormal to `local_axis2`, expressed in the local space of the second attached body.
    pub basis2: [Vector<Real>; 2],
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    pub impulse: Vector5<Real>,
    /// The target relative angular velocity the motor will attempt to reach.
    pub motor_target_vel: Real,
    /// The target relative angle along the joint axis the motor will attempt to reach.
    pub motor_target_pos: Real,
    /// The motor's stiffness.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_stiffness: Real,
    /// The motor's damping.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_damping: Real,
    /// The maximal impulse the motor is able to deliver.
    pub motor_max_impulse: Real,
    /// The angular impulse applied by the motor.
    pub motor_impulse: Real,
    /// The spring-like model used by the motor to reach the target velocity and .
    pub motor_model: SpringModel,
    // Used to handle cases where the position target ends up being more than pi radians away.
    pub(crate) motor_last_angle: Real,
    // The angular impulse expressed in world-space.
    pub(crate) world_ang_impulse: Vector<Real>,
    // The world-space orientation of the free axis of the first attached body.
    pub(crate) prev_axis1: Vector<Real>,
}

impl RevoluteJoint {
    /// Creates a new revolute joint with the given point of applications and axis, all expressed
    /// in the local-space of the affected bodies.
    pub fn new(
        local_anchor1: Point<Real>,
        local_axis1: Unit<Vector<Real>>,
        local_anchor2: Point<Real>,
        local_axis2: Unit<Vector<Real>>,
    ) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            local_axis1,
            local_axis2,
            basis1: local_axis1.orthonormal_basis(),
            basis2: local_axis2.orthonormal_basis(),
            impulse: na::zero(),
            world_ang_impulse: na::zero(),
            motor_target_vel: 0.0,
            motor_target_pos: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_max_impulse: Real::MAX,
            motor_impulse: 0.0,
            prev_axis1: *local_axis1,
            motor_model: SpringModel::VelocityBased,
            motor_last_angle: 0.0,
        }
    }

    /// Can a SIMD constraint be used for resolving this joint?
    pub fn supports_simd_constraints(&self) -> bool {
        // SIMD revolute constraints don't support motors right now.
        self.motor_max_impulse == 0.0 || (self.motor_stiffness == 0.0 && self.motor_damping == 0.0)
    }

    pub fn configure_motor_model(&mut self, model: SpringModel) {
        self.motor_model = model;
    }

    pub fn configure_motor_velocity(&mut self, target_vel: Real, factor: Real) {
        self.configure_motor(self.motor_target_pos, target_vel, 0.0, factor)
    }

    pub fn configure_motor_position(&mut self, target_pos: Real, stiffness: Real, damping: Real) {
        self.configure_motor(target_pos, 0.0, stiffness, damping)
    }

    pub fn configure_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) {
        self.motor_target_vel = target_vel;
        self.motor_target_pos = target_pos;
        self.motor_stiffness = stiffness;
        self.motor_damping = damping;
    }

    /// Estimates the current position of the motor angle.
    pub fn estimate_motor_angle(
        &self,
        body_pos1: &Isometry<Real>,
        body_pos2: &Isometry<Real>,
    ) -> Real {
        let motor_axis1 = body_pos1 * self.local_axis1;
        let ref1 = body_pos1 * self.basis1[0];
        let ref2 = body_pos2 * self.basis2[0];

        let last_angle_cycles = (self.motor_last_angle / Real::two_pi()).trunc() * Real::two_pi();

        // Measure the position between 0 and 2-pi
        let new_angle = if ref1.cross(&ref2).dot(&motor_axis1) < 0.0 {
            Real::two_pi() - ref1.angle(&ref2)
        } else {
            ref1.angle(&ref2)
        };

        // The last angle between 0 and 2-pi
        let last_angle_zero_two_pi = self.motor_last_angle - last_angle_cycles;

        // Figure out the smallest angle differance.
        let mut angle_diff = new_angle - last_angle_zero_two_pi;
        if angle_diff > Real::pi() {
            angle_diff -= Real::two_pi()
        } else if angle_diff < -Real::pi() {
            angle_diff += Real::two_pi()
        }

        self.motor_last_angle + angle_diff
    }
}
