//! Post-insertion runtime helpers attached to [`MjcfRobotHandles`]:
//! gravity compensation, contact-hook construction, mocap-keyframe
//! application, actuator control, and sensor reading.

use mjcf_rs::extras::Keyframe;

use rapier3d::dynamics::{
    ImpulseJointHandle, ImpulseJointSet, JointAxis, RigidBody, RigidBodySet, RigidBodyType,
};
use rapier3d::math::{Pose, Real, Rotation, Vector};

use crate::hooks::{MjcfContactHooks, PairOverride};

use super::handles::MjcfRobotHandles;
use super::types::{MjcfRobot, SensorObjectRef};

impl<H> MjcfRobotHandles<H> {
    /// Apply per-body gravity compensation as an explicit per-step force.
    ///
    /// When `body.gravcomp ∈ {0, 1}`, the loader sets `gravity_scale`
    /// directly so the user has nothing to do. For *fractional* values
    /// the loader's `gravity_scale = 1 - gravcomp` is only exact while
    /// the body's mass doesn't change; calling this helper each step
    /// instead applies a force that exactly cancels `gravcomp * mass *
    /// gravity` against the current `mass`.
    ///
    /// The helper takes over from the loader's static reduction by first
    /// resetting `gravity_scale` to 1 (otherwise the two compensations
    /// would stack — for `gravcomp = 1` that means a body that floats
    /// upward at `g` instead of standing still). It also resets the
    /// body's accumulated user force before adding its own contribution,
    /// so successive per-step calls don't pile up across frames. If you
    /// rely on `add_force` for other things on a `gravcomp` body, you'll
    /// need to re-apply those after `apply_gravity_compensation`.
    ///
    /// Call once per frame before `pipeline.step`.
    pub fn apply_gravity_compensation(
        &self,
        bodies: &mut RigidBodySet,
        robot: &MjcfRobot,
        gravity: Vector,
    ) {
        for (i, b) in robot.bodies.iter().enumerate() {
            if b.gravcomp <= 0.0 {
                continue;
            }
            let Some(handle) = self.bodies.get(i).and_then(|h| h.as_ref()) else {
                continue;
            };
            if let Some(rb) = bodies.get_mut(handle.body) {
                if rb.gravity_scale() != 1.0 {
                    rb.set_gravity_scale(1.0, false);
                }
                rb.reset_forces(false);
                // Use the MJCF-declared mass: `rb.mass()` returns 0 until
                // the rapier pipeline runs `update_world_mass_properties`,
                // which means the first call (before any step) would
                // silently fail to compensate.
                let force = -gravity * (b.gravcomp as Real) * b.mass;
                rb.add_force(force, true);
            }
        }
    }

    /// Build a [`MjcfContactHooks`] honoring this robot's `<contact><exclude>`
    /// and `<contact><pair>` entries. The returned hook can be passed to
    /// `pipeline.step(&hooks, ...)`.
    ///
    /// If the robot has no contact entries the returned hook is empty;
    /// installing it has no effect (other than a per-pair function-pointer
    /// dispatch).
    pub fn contact_hooks(&self, robot: &MjcfRobot) -> MjcfContactHooks {
        let mut hooks = MjcfContactHooks::new();

        // <exclude>: forbid contacts between every collider attached to body1
        // and every collider attached to body2.
        for ex in &robot.contact_excludes {
            let Some(b1) = robot.body_name_to_idx.get(&ex.body1).copied() else {
                log::warn!("<contact><exclude>: unknown body1 `{}`", ex.body1);
                continue;
            };
            let Some(b2) = robot.body_name_to_idx.get(&ex.body2).copied() else {
                log::warn!("<contact><exclude>: unknown body2 `{}`", ex.body2);
                continue;
            };
            let Some(h1) = self.bodies.get(b1).and_then(|b| b.as_ref()) else {
                continue;
            };
            let Some(h2) = self.bodies.get(b2).and_then(|b| b.as_ref()) else {
                continue;
            };
            for c1 in &h1.colliders {
                for c2 in &h2.colliders {
                    hooks.exclude(c1.handle, c2.handle);
                }
            }
        }

        // <pair>: per-pair friction / margin overrides between two named geoms.
        for p in &robot.contact_pairs {
            let Some(&(b1, gi1)) = robot.geom_name_to_collider.get(&p.geom1) else {
                log::warn!("<contact><pair>: unknown geom1 `{}`", p.geom1);
                continue;
            };
            let Some(&(b2, gi2)) = robot.geom_name_to_collider.get(&p.geom2) else {
                log::warn!("<contact><pair>: unknown geom2 `{}`", p.geom2);
                continue;
            };
            let Some(h1) = self
                .bodies
                .get(b1)
                .and_then(|b| b.as_ref())
                .and_then(|b| b.colliders.get(gi1))
            else {
                continue;
            };
            let Some(h2) = self
                .bodies
                .get(b2)
                .and_then(|b| b.as_ref())
                .and_then(|b| b.colliders.get(gi2))
            else {
                continue;
            };
            let ov = PairOverride {
                friction: p.friction.map(|f| f[0] as Real),
                margin: p.margin.map(|m| m as Real),
            };
            hooks.add_override(h1.handle, h2.handle, ov);
        }

        hooks
    }

    /// Apply a keyframe's `mpos`/`mquat` to mocap bodies. Joint coordinates
    /// (`qpos`/`qvel`) require knowledge of generalized-coordinate ordering
    /// and are best applied through [`MjcfRobot`] in impulse-joint mode
    /// (where each body has a global pose), or via the multibody set in
    /// multibody-joint mode.
    pub fn apply_mocap_keyframe(
        &self,
        bodies: &mut RigidBodySet,
        robot: &MjcfRobot,
        key: &Keyframe,
    ) {
        // mpos/mquat are concatenated arrays — 3 floats per mocap body.
        let mocap_bodies: Vec<usize> = robot
            .bodies
            .iter()
            .enumerate()
            .skip(1)
            .filter_map(|(i, b)| {
                if !b.is_intermediate
                    && robot.bodies[i].body.body_type() == RigidBodyType::KinematicPositionBased
                {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();
        for (k, idx) in mocap_bodies.iter().enumerate() {
            let pos = if key.mpos.len() >= 3 * (k + 1) {
                Vector::new(
                    key.mpos[3 * k] as Real,
                    key.mpos[3 * k + 1] as Real,
                    key.mpos[3 * k + 2] as Real,
                )
            } else {
                continue;
            };
            let q = if key.mquat.len() >= 4 * (k + 1) {
                Rotation::from_xyzw(
                    key.mquat[4 * k + 1] as Real,
                    key.mquat[4 * k + 2] as Real,
                    key.mquat[4 * k + 3] as Real,
                    key.mquat[4 * k] as Real,
                )
            } else {
                Rotation::IDENTITY
            };
            if let Some(handle) = self.bodies.get(*idx).and_then(|b| b.as_ref()) {
                if let Some(rb) = bodies.get_mut(handle.body) {
                    rb.set_position(Pose::from_parts(pos, q), true);
                }
            }
        }
    }
}

impl MjcfRobotHandles<ImpulseJointHandle> {
    /// Apply per-actuator control values to the impulse joints they drive.
    ///
    /// `ctrl` is a flat array of one scalar per actuator, in the order of
    /// [`MjcfRobot::actuators`] (and [`MjcfRobotHandles::actuators`]). The
    /// per-actuator mapping uses MJCF semantics:
    ///
    /// - `<motor>` — a force/torque input. We emulate the constant-force
    ///   semantics by aiming the motor at a very large velocity and
    ///   capping the applied force at `|ctrl * gear|`.
    /// - `<position>` → `set_motor_position(ctrl, kp, kv)`.
    /// - `<velocity>` → `set_motor_velocity(ctrl, kv)`.
    /// - `<damper>` → zero velocity target with damping `gainprm[0] * |ctrl|`.
    /// - everything else (`general`, `intvelocity`, …) is left to the user
    ///   — the metadata is preserved on `Self::actuators`.
    pub fn apply_controls(&self, joints: &mut ImpulseJointSet, ctrl: &[Real]) {
        use mjcf_rs::extras::ActuatorKind;
        for (i, ah) in self.actuators.iter().enumerate() {
            let Some(handle) = ah.joint else { continue };
            let Some(joint) = joints.get_mut(handle, true) else {
                continue;
            };
            let u = ctrl.get(i).copied().unwrap_or(0.0);
            // Determine the axis the actuator drives — for hinge/ball, the
            // angular X axis is the joint's free axis (after the per-joint
            // basis rotation we applied in build_serial_joint).
            let ax = JointAxis::AngX; // default (hinge / ball)
            let lin_ax = JointAxis::LinX;
            let kp = ah.actuator.kp.unwrap_or_default() as Real;
            let kv = ah.actuator.kv.unwrap_or_default() as Real;
            let gear = ah.actuator.gear[0] as Real;
            let force_max = ah
                .actuator
                .force_range
                .map(|r| r[1].abs() as Real)
                .unwrap_or(Real::INFINITY);
            match ah.actuator.kind {
                ActuatorKind::Motor => {
                    // MJCF `<motor>` is a *force* (or torque) input: it adds
                    // `ctrl * gear` to the joint's generalized force each
                    // step. Rapier's joint motors have no "constant force"
                    // mode, so we emulate it by aiming the motor at a very
                    // large velocity in the direction of the desired force,
                    // then capping the applied force at `|ctrl * gear|`.
                    // With `stiffness = damping = 0` plus a hard cap, the
                    // motor saturates at its `max_force` as long as the
                    // joint hasn't reached the (very far) target velocity —
                    // which it never does — so the effective output is a
                    // constant `ctrl * gear` along the joint axis.
                    //
                    // We don't know whether the joint is rotational or
                    // prismatic, so we configure both axes and let the
                    // locked one's motor be a no-op.
                    let force = u * gear;
                    let max = force.abs().min(force_max);
                    let far_vel = if force >= 0.0 { 1.0e9 } else { -1.0e9 };
                    joint.data.set_motor_velocity(ax, far_vel, 0.0);
                    joint.data.set_motor_velocity(lin_ax, far_vel, 0.0);
                    joint.data.set_motor_max_force(ax, max);
                    joint.data.set_motor_max_force(lin_ax, max);
                }
                ActuatorKind::Position => {
                    joint.data.set_motor_position(ax, u, kp, kv);
                    joint.data.set_motor_position(lin_ax, u, kp, kv);
                }
                ActuatorKind::Velocity => {
                    joint.data.set_motor_velocity(ax, u, kv);
                    joint.data.set_motor_velocity(lin_ax, u, kv);
                }
                ActuatorKind::Damper => {
                    let damp =
                        ah.actuator.gainprm.first().copied().unwrap_or(0.0) as Real * u.abs();
                    joint.data.set_motor_velocity(ax, 0.0, damp);
                    joint.data.set_motor_velocity(lin_ax, 0.0, damp);
                }
                _ => {
                    // intvelocity / general / other — user-driven.
                }
            }
        }
    }
}

/// One scalar / vector / quaternion reading from a sensor.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum MjcfSensorValue {
    /// One scalar.
    Scalar(Real),
    /// 3-vector.
    Vector3(Vector),
    /// Unit quaternion.
    Quat(Rotation),
}

impl MjcfRobot {
    /// Read a sensor's current value from the rapier state. Returns `None`
    /// for sensors the loader doesn't know how to evaluate.
    pub fn read_sensor(
        &self,
        sensor_idx: usize,
        bodies: &RigidBodySet,
        handles: &MjcfRobotHandles<impl Copy>,
        gravity: Vector,
    ) -> Option<MjcfSensorValue> {
        let binding = self.sensors.get(sensor_idx)?;
        let s = &binding.sensor;
        let body_handle_of = |b: usize| -> Option<&RigidBody> {
            handles
                .bodies
                .get(b)
                .and_then(|h| h.as_ref())
                .and_then(|h| bodies.get(h.body))
        };
        match (s.kind.as_str(), &binding.object) {
            ("framepos", SensorObjectRef::Body(b))
            | ("framepos", SensorObjectRef::Site { body: b, .. }) => {
                Some(MjcfSensorValue::Vector3(body_handle_of(*b)?.translation()))
            }
            ("framequat", SensorObjectRef::Body(b)) => {
                Some(MjcfSensorValue::Quat(*body_handle_of(*b)?.rotation()))
            }
            ("framelinvel", SensorObjectRef::Body(b)) => {
                Some(MjcfSensorValue::Vector3(body_handle_of(*b)?.linvel()))
            }
            ("frameangvel", SensorObjectRef::Body(b)) => {
                Some(MjcfSensorValue::Vector3(body_handle_of(*b)?.angvel()))
            }
            ("velocimeter", SensorObjectRef::Body(b))
            | ("velocimeter", SensorObjectRef::Site { body: b, .. }) => {
                Some(MjcfSensorValue::Vector3(body_handle_of(*b)?.linvel()))
            }
            ("gyro", SensorObjectRef::Body(b))
            | ("gyro", SensorObjectRef::Site { body: b, .. }) => {
                let rb = body_handle_of(*b)?;
                // Express angular velocity in the body's local frame.
                let inv_rot = rb.rotation().inverse();
                Some(MjcfSensorValue::Vector3(inv_rot * rb.angvel()))
            }
            ("subtreemass", SensorObjectRef::Body(b)) => {
                let rb = body_handle_of(*b)?;
                Some(MjcfSensorValue::Scalar(rb.mass()))
            }
            ("subtreecom", SensorObjectRef::Body(b)) => {
                let rb = body_handle_of(*b)?;
                let local_com = rb.mass_properties().local_mprops.local_com;
                Some(MjcfSensorValue::Vector3(*rb.position() * local_com))
            }
            ("clock", _) => {
                // The user is expected to keep their own simulation time.
                let _ = gravity;
                Some(MjcfSensorValue::Scalar(0.0))
            }
            _ => None,
        }
    }
}
