//! Post-insertion runtime helpers attached to [`MjcfRobotHandles`]:
//! gravity compensation, contact-hook construction, mocap-keyframe
//! application, actuator control, and sensor reading.

use mjcf_rs::extras::Keyframe;

use rapier3d::dynamics::{
    GenericJoint, ImpulseJointHandle, ImpulseJointSet, JointAxis, MultibodyJointHandle,
    MultibodyJointSet, RigidBody, RigidBodySet, RigidBodyType,
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
            if let Some(handle) = self.bodies.get(*idx).and_then(|b| b.as_ref())
                && let Some(rb) = bodies.get_mut(handle.body)
            {
                rb.set_position(Pose::from_parts(pos, q), true);
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
        for (i, ah) in self.actuators.iter().enumerate() {
            let Some(handle) = ah.joint else { continue };
            let Some(joint) = joints.get_mut(handle, true) else {
                continue;
            };
            let u = ctrl.get(i).copied().unwrap_or(0.0);
            configure_actuator_motor(&mut joint.data, &ah.actuator, u);
        }
    }
}

impl MjcfRobotHandles<Option<MultibodyJointHandle>> {
    /// Apply per-actuator control values to the multibody joints they drive.
    ///
    /// This is the multibody-path counterpart of
    /// [`apply_controls`](MjcfRobotHandles::<ImpulseJointHandle>::apply_controls):
    /// it reproduces MuJoCo's "actuation" by writing each actuator's motor
    /// onto the multibody link it drives. The per-actuator interpretation is
    /// identical (see [`configure_actuator_motor`]); call it once per frame
    /// before `pipeline.step`.
    ///
    /// `ctrl` is a flat array of one scalar per actuator, in the order of
    /// [`MjcfRobot::actuators`] (and [`MjcfRobotHandles::actuators`]).
    /// Actuators whose joint was dropped from the multibody chain (a loop
    /// closure) are skipped.
    ///
    /// Each driven body is woken so a steady control input keeps holding the
    /// pose even after the multibody would otherwise have gone to sleep
    /// (mirroring the impulse path, which wakes through the joint graph).
    pub fn apply_controls_multibody(
        &self,
        bodies: &mut RigidBodySet,
        multibody_joints: &mut MultibodyJointSet,
        ctrl: &[Real],
    ) {
        for (i, ah) in self.actuators.iter().enumerate() {
            // `H` is `Option<MultibodyJointHandle>` here, so `ah.joint` is
            // `Option<Option<_>>`: the outer `None` means "no actuator joint",
            // the inner `None` means "joint dropped as a loop closure".
            let Some(Some(handle)) = ah.joint else { continue };
            let Some((mb, link_id)) = multibody_joints.get_mut(handle) else {
                continue;
            };
            let Some(link) = mb.links_mut().nth(link_id) else {
                continue;
            };
            let u = ctrl.get(i).copied().unwrap_or(0.0);
            configure_actuator_motor(&mut link.joint.data, &ah.actuator, u);
            let rb = link.rigid_body_handle();
            if let Some(body) = bodies.get_mut(rb) {
                body.wake_up(true);
            }
        }
    }
}

/// Write the joint-motor configuration for one MJCF actuator driving one
/// rapier joint, given the control value `u`. Shared by the impulse- and
/// multibody-joint control paths. MJCF semantics:
///
/// - `<motor>` → constant generalized force `u * gear` (emulated with a
///   far-target velocity motor capped at `|u * gear|`).
/// - `<position>` → `set_motor_position(u, kp, kv)`.
/// - `<velocity>` → `set_motor_velocity(u, kv)`.
/// - `<damper>` → zero-velocity target with damping `gainprm[0] * |u|`.
/// - `<general>` with `biastype="affine"` → a position/velocity servo. The
///   affine model `force = gainprm0·u + biasprm0 + biasprm1·q + biasprm2·q̇`
///   matches a spring `stiffness·(target − q) − damping·q̇` with
///   `stiffness = −biasprm1`, `damping = −biasprm2`, and
///   `target = (gainprm0·u + biasprm0) / stiffness`. This is what holds e.g.
///   the flybody legs in their commanded pose.
/// - `<general>` with a non-affine bias → constant force `gainprm0·u * gear`
///   (same emulation as `<motor>`).
/// - other kinds (`intvelocity`, muscle, …) are left untouched.
///
/// The joint's free axis is the angular (hinge/ball) or linear (slide) X axis
/// after the per-joint basis rotation applied in `build_serial_joint`; we
/// configure both and let the locked one's motor be a no-op.
fn configure_actuator_motor(
    data: &mut GenericJoint,
    actuator: &mjcf_rs::extras::Actuator,
    u: Real,
) {
    use mjcf_rs::extras::ActuatorKind;

    let ax = JointAxis::AngX;
    let lin_ax = JointAxis::LinX;
    let kp = actuator.kp.unwrap_or_default() as Real;
    let kv = actuator.kv.unwrap_or_default() as Real;
    let gear = actuator.gear[0] as Real;
    let force_max = actuator
        .force_range
        .map(|r| r[1].abs() as Real)
        .unwrap_or(Real::INFINITY);

    // Emulate a constant generalized force: aim the motor at a far-away
    // velocity in the force's direction and cap the impulse at the force
    // magnitude. With stiffness = damping = 0, the motor saturates at its
    // max_force every step (the target is never reached), so the net effect
    // is a constant force along the joint axis.
    let apply_constant_force = |data: &mut GenericJoint, force: Real| {
        let max = force.abs().min(force_max);
        let far_vel = if force >= 0.0 { 1.0e9 } else { -1.0e9 };
        data.set_motor_velocity(ax, far_vel, 0.0);
        data.set_motor_velocity(lin_ax, far_vel, 0.0);
        data.set_motor_max_force(ax, max);
        data.set_motor_max_force(lin_ax, max);
    };

    match actuator.kind {
        ActuatorKind::Motor => apply_constant_force(data, u * gear),
        ActuatorKind::Position => {
            data.set_motor_position(ax, u, kp, kv);
            data.set_motor_position(lin_ax, u, kp, kv);
            // Apply the actuator's `forcerange` as the motor's max force.
            // `set_motor_position` leaves `max_force` untouched, so without this
            // the servo inherits whatever joint construction left it at (e.g. the
            // `frictionloss` passed to `motor_max_force` when building the joint),
            // which starves position servos — the joint can't deliver enough
            // torque to hold against gravity. `force_max` is `INFINITY` when no
            // `forcerange` is given, which leaves the motor unbounded (a no-op).
            data.set_motor_max_force(ax, force_max);
            data.set_motor_max_force(lin_ax, force_max);
        }
        ActuatorKind::Velocity => {
            data.set_motor_velocity(ax, u, kv);
            data.set_motor_velocity(lin_ax, u, kv);
            data.set_motor_max_force(ax, force_max);
            data.set_motor_max_force(lin_ax, force_max);
        }
        ActuatorKind::Damper => {
            let damp = actuator.gainprm.first().copied().unwrap_or(0.0) as Real * u.abs();
            data.set_motor_velocity(ax, 0.0, damp);
            data.set_motor_velocity(lin_ax, 0.0, damp);
            data.set_motor_max_force(ax, force_max);
            data.set_motor_max_force(lin_ax, force_max);
        }
        ActuatorKind::General => {
            let g0 = actuator.gainprm.first().copied().unwrap_or(0.0) as Real;
            let b0 = actuator.biasprm.first().copied().unwrap_or(0.0) as Real;
            let b1 = actuator.biasprm.get(1).copied().unwrap_or(0.0) as Real;
            let b2 = actuator.biasprm.get(2).copied().unwrap_or(0.0) as Real;
            let affine = actuator.bias_type.as_deref() == Some("affine");
            // A position servo needs a real restoring term (`-biasprm1·q`,
            // i.e. stiffness > 0); without it the affine bias is just a
            // constant + velocity-damping force.
            if affine && -b1 > 0.0 {
                let stiffness = -b1;
                let damping = -b2;
                let target = (g0 * u + b0) / stiffness;
                data.set_motor_position(ax, target, stiffness, damping);
                data.set_motor_position(lin_ax, target, stiffness, damping);
                data.set_motor_max_force(ax, force_max);
                data.set_motor_max_force(lin_ax, force_max);
            } else {
                // `gaintype="fixed"`, non-restoring bias: constant force
                // `gainprm0·u` through the transmission.
                apply_constant_force(data, g0 * u * gear);
            }
        }
        ActuatorKind::IntVelocity | ActuatorKind::Other => {
            // Not driven automatically — user-controlled.
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
