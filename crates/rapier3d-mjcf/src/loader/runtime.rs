//! Post-insertion runtime helpers attached to [`MjcfRobotHandles`]:
//! gravity compensation, contact-hook construction, mocap-keyframe
//! application, actuator control, and sensor reading.

use mjcf_rs::extras::Keyframe;

use rapier3d::dynamics::{
    GenericJoint, ImpulseJointHandle, ImpulseJointSet, JointAxis, MultibodyIndex,
    MultibodyJointHandle, MultibodyJointSet, RigidBody, RigidBodySet, RigidBodyType,
};
use rapier3d::math::{Pose, Real, Rotation, Vector};

use crate::hooks::{MjcfContactHooks, PairOverride};

use super::handles::MjcfRobotHandles;
use super::types::{MjcfDofKind, MjcfQposDof, MjcfRobot, SensorObjectRef};

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

/// The world pose encoded by a free joint's 7 `qpos` (`x y z` + `w x y z`
/// quaternion, MuJoCo's wxyz order), with the loader's length `scale` applied
/// to the translation and the loader's `shift` pre-multiplied so the floating
/// base lands in the same frame as the rest of the model.
fn free_pose(qp: &[f64], scale: Real, shift: Pose) -> Pose {
    let translation = Vector::new(qp[0] as Real, qp[1] as Real, qp[2] as Real) * scale;
    let rotation = Rotation::from_xyzw(qp[4] as Real, qp[5] as Real, qp[6] as Real, qp[3] as Real);
    shift * Pose::from_parts(translation, rotation)
}

/// A hinge/slide DoF's target rapier joint coordinate from its single `qpos`
/// scalar. MuJoCo stores the absolute coordinate, so we subtract `<joint ref>`
/// (and scale lengths) to reach rapier's frame-relative convention — the same
/// transform the serial-joint builder applies to the joint limits.
fn scalar_coord(dof: &MjcfQposDof, qp: &[f64]) -> Real {
    match dof.kind {
        MjcfDofKind::Hinge => qp[0] as Real - dof.reference,
        MjcfDofKind::Slide => (qp[0] as Real - dof.reference) * dof.scale,
        _ => 0.0,
    }
}

/// The target relative rotation of a ball DoF from its 4 `qpos` (wxyz).
fn ball_rotation(qp: &[f64]) -> Rotation {
    Rotation::from_xyzw(qp[1] as Real, qp[2] as Real, qp[3] as Real, qp[0] as Real)
}

impl MjcfRobotHandles<Option<MultibodyJointHandle>> {
    /// Apply a keyframe's full state (`qpos`, `qvel`, and `mpos`/`mquat`) to a
    /// robot inserted through [`insert_using_multibody_joints`](MjcfRobot::insert_using_multibody_joints).
    ///
    /// Joint coordinates are written to the multibody's generalized
    /// coordinates (then propagated through forward-kinematics so the rapier
    /// rigid-bodies match), and a floating base's `qpos` sets its root body's
    /// world pose. Call this once after insertion to start the model in a
    /// declared keyframe (e.g. MuJoCo's `home`).
    ///
    /// `qvel` is written to the generalized velocities of articulated joints
    /// and as the world-frame linear/angular velocity of a floating base.
    /// Mocap bodies are handled exactly as [`apply_mocap_keyframe`](Self::apply_mocap_keyframe).
    pub fn apply_keyframe(
        &self,
        bodies: &mut RigidBodySet,
        multibody_joints: &mut MultibodyJointSet,
        robot: &MjcfRobot,
        key: &Keyframe,
    ) {
        self.apply_mocap_keyframe(bodies, robot, key);

        // Multibodies whose root pose / joint coordinates changed: re-run
        // forward-kinematics on each once at the end.
        let mut touched: Vec<MultibodyIndex> = Vec::new();
        let touch = |touched: &mut Vec<MultibodyIndex>, idx: MultibodyIndex| {
            if !touched.contains(&idx) {
                touched.push(idx);
            }
        };

        let mut qpos_i = 0;
        let mut qvel_i = 0;
        for dof in &robot.qpos_dofs {
            let qpos = key.qpos.get(qpos_i..qpos_i + dof.kind.qpos_width());
            let qvel = key.qvel.get(qvel_i..qvel_i + dof.kind.qvel_width());
            qpos_i += dof.kind.qpos_width();
            qvel_i += dof.kind.qvel_width();

            if dof.kind == MjcfDofKind::Free {
                let Some(handle) = self.bodies.get(dof.body).and_then(|b| b.as_ref()) else {
                    continue;
                };
                if let Some(rb) = bodies.get_mut(handle.body) {
                    if let Some(qp) = qpos {
                        rb.set_position(free_pose(qp, dof.scale, robot.base_shift), true);
                    }
                    if let Some(qv) = qvel {
                        let lin =
                            Vector::new(qv[0] as Real, qv[1] as Real, qv[2] as Real) * dof.scale;
                        let ang = Vector::new(qv[3] as Real, qv[4] as Real, qv[5] as Real);
                        rb.set_linvel(lin, true);
                        rb.set_angvel(ang, true);
                    }
                }
                if let Some(link) = multibody_joints.rigid_body_link(handle.body) {
                    touch(&mut touched, link.multibody);
                }
                continue;
            }

            // Articulated DoF (hinge / slide / ball): write to the multibody's
            // generalized coordinates.
            let Some(jidx) = dof.joint else { continue };
            let Some(jh) = self.joints.get(jidx) else {
                continue;
            };
            // `joint` is `None` for a joint that was dropped as a loop closure.
            let Some(handle) = jh.joint else { continue };
            if let Some(link) = multibody_joints.rigid_body_link(jh.link2) {
                touch(&mut touched, link.multibody);
            }
            let Some((mb, link_id)) = multibody_joints.get_mut(handle) else {
                continue;
            };

            // Compute the displacement (target − current) under an immutable
            // borrow, then apply it under a mutable one.
            let (assembly_id, ndofs, disp) = {
                let Some(link) = mb.link(link_id) else {
                    continue;
                };
                let joint = link.joint();
                let coords = joint.coords();
                let disp = qpos.map(|qp| match dof.kind {
                    MjcfDofKind::Hinge => vec![scalar_coord(dof, qp) - coords[3]],
                    MjcfDofKind::Slide => vec![scalar_coord(dof, qp) - coords[0]],
                    MjcfDofKind::Ball => {
                        // Reach the target relative rotation from the current
                        // one: `from_scaled_axis(disp) * joint_rot == target`.
                        let delta = ball_rotation(qp) * joint.joint_rot().inverse();
                        let sa = delta.to_scaled_axis();
                        vec![sa.x, sa.y, sa.z]
                    }
                    MjcfDofKind::Free => vec![],
                });
                (link.assembly_id(), joint.ndofs(), disp)
            };
            if let Some(disp) = disp
                && let Some(link) = mb.link_mut(link_id)
            {
                link.joint.apply_displacement(&disp);
            }
            if let Some(qv) = qvel {
                let scale = if dof.kind == MjcfDofKind::Slide {
                    dof.scale
                } else {
                    1.0
                };
                let mut vels = mb.generalized_velocity_mut();
                for k in 0..ndofs.min(qv.len()) {
                    vels[assembly_id + k] = qv[k] as Real * scale;
                }
            }
        }

        for idx in touched {
            if let Some(mb) = multibody_joints.get_multibody_mut(idx) {
                mb.forward_kinematics(bodies, true);
                mb.update_rigid_bodies(bodies, false);
            }
        }
    }
}

impl MjcfRobotHandles<ImpulseJointHandle> {
    /// Apply a keyframe's `qpos` (and a floating base's `qvel`) to a robot
    /// inserted through [`insert_using_impulse_joints`](MjcfRobot::insert_using_impulse_joints).
    ///
    /// In the impulse-joint path every body carries a global pose, so the
    /// keyframe is realized by running forward-kinematics over the joint tree
    /// (anchored at the unchanged fixed/welded roots, or at a floating base set
    /// from its `qpos`) and writing each body's resulting world pose. The joint
    /// constraints are already satisfied by those poses.
    ///
    /// Only a floating base's `qvel` is applied (as world-frame body velocity);
    /// per-joint `qvel` has no generalized-coordinate home in this path and is
    /// ignored. Mocap bodies are handled as [`apply_mocap_keyframe`](Self::apply_mocap_keyframe).
    pub fn apply_keyframe(&self, bodies: &mut RigidBodySet, robot: &MjcfRobot, key: &Keyframe) {
        self.apply_mocap_keyframe(bodies, robot, key);

        // Per-joint relative transform from the keyframe; identity where a
        // joint carries no qpos (synthesized welds keep the bodies at rest).
        let mut joint_tf: Vec<Pose> = vec![Pose::default(); robot.joints.len()];
        // Forward-kinematics world poses, seeded with the load-time poses so
        // fixed/welded roots and the world body keep their place.
        let mut world: Vec<Pose> = robot.bodies.iter().map(|b| *b.body.position()).collect();
        // Floating-base velocities, applied after the pose write-back.
        let mut free_vels: Vec<(usize, Vector, Vector)> = Vec::new();

        let mut qpos_i = 0;
        let mut qvel_i = 0;
        for dof in &robot.qpos_dofs {
            let qpos = key.qpos.get(qpos_i..qpos_i + dof.kind.qpos_width());
            let qvel = key.qvel.get(qvel_i..qvel_i + dof.kind.qvel_width());
            qpos_i += dof.kind.qpos_width();
            qvel_i += dof.kind.qvel_width();

            match dof.kind {
                MjcfDofKind::Free => {
                    if let Some(qp) = qpos {
                        world[dof.body] = free_pose(qp, dof.scale, robot.base_shift);
                    }
                    if let Some(qv) = qvel {
                        let lin =
                            Vector::new(qv[0] as Real, qv[1] as Real, qv[2] as Real) * dof.scale;
                        let ang = Vector::new(qv[3] as Real, qv[4] as Real, qv[5] as Real);
                        free_vels.push((dof.body, lin, ang));
                    }
                }
                MjcfDofKind::Hinge | MjcfDofKind::Slide => {
                    if let (Some(jidx), Some(qp)) = (dof.joint, qpos) {
                        let q = scalar_coord(dof, qp);
                        joint_tf[jidx] = if dof.kind == MjcfDofKind::Hinge {
                            Pose::from_rotation(Rotation::from_axis_angle(Vector::X, q))
                        } else {
                            Pose::from_translation(Vector::X * q)
                        };
                    }
                }
                MjcfDofKind::Ball => {
                    if let (Some(jidx), Some(qp)) = (dof.joint, qpos) {
                        joint_tf[jidx] = Pose::from_rotation(ball_rotation(qp));
                    }
                }
            }
        }

        // Forward-kinematics: `robot.joints` is in topological order, so each
        // joint's parent body is already positioned. This mirrors rapier's
        // multibody `body_to_parent`: world₂ = world₁ · frame1 · q · frame2⁻¹.
        for (jidx, j) in robot.joints.iter().enumerate() {
            world[j.link2] = world[j.link1]
                * j.joint.local_frame1
                * joint_tf[jidx]
                * j.joint.local_frame2.inverse();
        }

        for (i, handle) in self.bodies.iter().enumerate() {
            if let Some(handle) = handle
                && let Some(rb) = bodies.get_mut(handle.body)
            {
                rb.set_position(world[i], true);
            }
        }
        for (body, lin, ang) in free_vels {
            if let Some(handle) = self.bodies.get(body).and_then(|b| b.as_ref())
                && let Some(rb) = bodies.get_mut(handle.body)
            {
                rb.set_linvel(lin, true);
                rb.set_angvel(ang, true);
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
        self.apply_controls_scaled(joints, ctrl, 1.0);
    }

    /// Like [`apply_controls`](Self::apply_controls) but uniformly scales every
    /// actuator's strength by `gain_scale` (gains and force limits; see
    /// `configure_actuator_motor`). `gain_scale < 1` softens the actuation —
    /// e.g. to ease servo-driven moves that would otherwise saturate and snap.
    pub fn apply_controls_scaled(
        &self,
        joints: &mut ImpulseJointSet,
        ctrl: &[Real],
        gain_scale: Real,
    ) {
        for (i, ah) in self.actuators.iter().enumerate() {
            let Some(handle) = ah.joint else { continue };
            let Some(joint) = joints.get_mut(handle, true) else {
                continue;
            };
            let u = ctrl.get(i).copied().unwrap_or(0.0);
            configure_actuator_motor(&mut joint.data, &ah.actuator, u, gain_scale);
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
    /// identical (see `configure_actuator_motor`); call it once per frame
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
        self.apply_controls_multibody_scaled(bodies, multibody_joints, ctrl, 1.0);
    }

    /// Like [`apply_controls_multibody`](Self::apply_controls_multibody) but
    /// uniformly scales every actuator's strength by `gain_scale` (gains and
    /// force limits; see `configure_actuator_motor`). `gain_scale < 1` softens
    /// the actuation, so a servo-driven move (e.g. easing between keyframes)
    /// ramps in instead of saturating to its force limit and arriving instantly.
    pub fn apply_controls_multibody_scaled(
        &self,
        bodies: &mut RigidBodySet,
        multibody_joints: &mut MultibodyJointSet,
        ctrl: &[Real],
        gain_scale: Real,
    ) {
        for (i, ah) in self.actuators.iter().enumerate() {
            // `H` is `Option<MultibodyJointHandle>` here, so `ah.joint` is
            // `Option<Option<_>>`: the outer `None` means "no actuator joint",
            // the inner `None` means "joint dropped as a loop closure".
            let Some(Some(handle)) = ah.joint else {
                continue;
            };
            let Some((mb, link_id)) = multibody_joints.get_mut(handle) else {
                continue;
            };
            let Some(link) = mb.links_mut().nth(link_id) else {
                continue;
            };
            let u = ctrl.get(i).copied().unwrap_or(0.0);
            configure_actuator_motor(&mut link.joint.data, &ah.actuator, u, gain_scale);
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
    gain_scale: Real,
) {
    use mjcf_rs::extras::ActuatorKind;

    let ax = JointAxis::AngX;
    let lin_ax = JointAxis::LinX;
    // MuJoCo's `<position>` defaults `kp` to 1 when it isn't set (directly or
    // through a `<default>` class), not 0. Defaulting to 0 here would give a
    // zero-gain — i.e. completely limp — servo, which is what made e.g. the
    // shadow hand's many `kp`-less finger actuators unable to hold/move their
    // joints. `kv` (velocity gain) does default to 0 in MuJoCo.
    //
    // `gain_scale` uniformly scales the actuator's strength: the servo gains
    // (`kp`/`kv`, or an affine `<general>`'s stiffness/damping — target pose
    // unchanged) and the force cap (`forcerange`) and constant forces. Below 1
    // the actuator pushes more softly, so a servo-driven move (e.g. snapping
    // between keyframes) eases in instead of saturating to its force limit and
    // arriving almost instantly. `1.0` reproduces the model as authored.
    let kp = actuator.kp.unwrap_or(1.0) as Real * gain_scale;
    let kv = actuator.kv.unwrap_or_default() as Real * gain_scale;
    let gear = actuator.gear[0] as Real;
    let base_force_max = actuator
        .force_range
        .map(|r| r[1].abs() as Real)
        .unwrap_or(Real::INFINITY);
    // Scale the force cap too (an unbounded `INFINITY` cap stays unbounded —
    // and avoids `INFINITY * 0` = `NaN` for a zero scale).
    let force_max = if base_force_max.is_finite() {
        base_force_max * gain_scale
    } else {
        base_force_max
    };

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
        ActuatorKind::Motor => apply_constant_force(data, u * gear * gain_scale),
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
            let damp =
                actuator.gainprm.first().copied().unwrap_or(0.0) as Real * u.abs() * gain_scale;
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
                // Target uses the unscaled stiffness so the commanded pose is
                // unchanged; only the gains (force) scale.
                let target = (g0 * u + b0) / stiffness;
                data.set_motor_position(ax, target, stiffness * gain_scale, damping * gain_scale);
                data.set_motor_position(
                    lin_ax,
                    target,
                    stiffness * gain_scale,
                    damping * gain_scale,
                );
                data.set_motor_max_force(ax, force_max);
                data.set_motor_max_force(lin_ax, force_max);
            } else {
                // `gaintype="fixed"`, non-restoring bias: constant force
                // `gainprm0·u` through the transmission.
                apply_constant_force(data, g0 * u * gear * gain_scale);
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
    /// Look up a keyframe by its `<key name="…">`. Returns `None` if no
    /// keyframe with that name exists.
    pub fn keyframe_by_name(&self, name: &str) -> Option<&Keyframe> {
        self.keyframes
            .iter()
            .find(|k| k.name.as_deref() == Some(name))
    }

    /// Build a per-actuator control vector that commands the given keyframe's
    /// pose, in the order of [`Self::actuators`].
    ///
    /// This is what lets a keyframe *hold* under actuation instead of being
    /// dragged back to the zero configuration: driving the actuators with a
    /// plain zero vector (the obvious default) makes every position servo pull
    /// its joint to 0, undoing the keyframe. Each entry is resolved as:
    ///
    /// - the keyframe's explicit `ctrl[i]` when it provides one (MuJoCo's own
    ///   commanded input for the pose), otherwise
    /// - for an actuator driving a 1-DoF (hinge/slide) joint, that joint's
    ///   keyframe `qpos` in rapier coordinates (the natural position-servo
    ///   target), otherwise
    /// - `0`.
    ///
    /// Pass the result to [`apply_controls`](MjcfRobotHandles::<rapier3d::dynamics::ImpulseJointHandle>::apply_controls)
    /// / [`apply_controls_multibody`](MjcfRobotHandles::<Option<MultibodyJointHandle>>::apply_controls_multibody).
    pub fn keyframe_controls(&self, key: &Keyframe) -> Vec<Real> {
        // Per-joint position target (rapier coordinates) derived from `qpos`.
        let mut joint_target: Vec<Option<Real>> = vec![None; self.joints.len()];
        let mut qpos_i = 0;
        for dof in &self.qpos_dofs {
            if let Some(jidx) = dof.joint
                && let Some(&q) = key.qpos.get(qpos_i)
            {
                let target = match dof.kind {
                    MjcfDofKind::Hinge => Some(q as Real - dof.reference),
                    MjcfDofKind::Slide => Some((q as Real - dof.reference) * dof.scale),
                    // Ball/free are multi-DoF; no single scalar servo target.
                    _ => None,
                };
                if let Some(t) = target {
                    joint_target[jidx] = Some(t);
                }
            }
            qpos_i += dof.kind.qpos_width();
        }

        self.actuators
            .iter()
            .enumerate()
            .map(|(i, a)| {
                if let Some(&c) = key.ctrl.get(i) {
                    c as Real
                } else {
                    a.joint_index
                        .and_then(|j| joint_target.get(j).copied().flatten())
                        .unwrap_or(0.0)
                }
            })
            .collect()
    }

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
