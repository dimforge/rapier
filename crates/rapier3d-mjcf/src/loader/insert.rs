//! Insertion paths. Materialize the [`MjcfRobot`] into rapier's
//! `RigidBodySet` / `ColliderSet` / `ImpulseJointSet` (or `MultibodyJointSet`)
//! and return per-body / per-joint handles. Also hosts the post-insertion
//! `append_transform` helper that the testbed uses to position multiple
//! instances of the same robot.

use rapier3d::dynamics::{
    ImpulseJointHandle, ImpulseJointSet, MultibodyJointHandle, MultibodyJointSet, RigidBodyHandle,
    RigidBodySet,
};
use rapier3d::geometry::ColliderSet;
use rapier3d::math::Pose;

use super::handles::{
    MjcfActuatorHandle, MjcfBodyHandle, MjcfColliderHandle, MjcfJointHandle, MjcfRobotHandles,
};
use super::mass::{add_armature_to_multibody, move_motor_damping_to_multibody};
use super::options::MjcfMultibodyOptions;
use super::types::MjcfRobot;

impl MjcfRobot {
    /// Insert the robot into the rapier sets, using impulse joints.
    pub fn insert_using_impulse_joints(
        self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
    ) -> MjcfRobotHandles<ImpulseJointHandle> {
        // Decide up front whether to insert the world body. We do so when
        // the world has colliders, or when any joint or equality references
        // it (which is the common case for any non-free root body).
        let world_referenced = self.joints.iter().any(|j| j.link1 == 0 || j.link2 == 0)
            || self
                .equality_joints
                .iter()
                .any(|j| j.link1 == 0 || j.link2 == 0)
            || !self.bodies[0].colliders.is_empty();
        let mut body_handles: Vec<Option<MjcfBodyHandle>> = vec![None; self.bodies.len()];
        for (i, b) in self.bodies.into_iter().enumerate() {
            if i == 0 && !world_referenced {
                continue;
            }
            let handle = bodies.insert(b.body);
            let mut col_handles = Vec::with_capacity(b.colliders.len());
            for c in b.colliders {
                let h = colliders.insert_with_parent(c, handle, bodies);
                col_handles.push(MjcfColliderHandle { handle: h });
            }
            body_handles[i] = Some(MjcfBodyHandle {
                body: handle,
                colliders: col_handles,
            });
        }
        // Insert joints.
        let mut joint_handles = Vec::with_capacity(self.joints.len());
        for j in self.joints {
            let l1 = body_handles[j.link1].as_ref().map(|b| b.body);
            let l2 = body_handles[j.link2].as_ref().map(|b| b.body);
            let (Some(l1), Some(l2)) = (l1, l2) else {
                continue;
            };
            let h = impulse_joints.insert(l1, l2, j.joint, true);
            joint_handles.push(MjcfJointHandle {
                joint: h,
                link1: l1,
                link2: l2,
            });
        }
        // Equality joints.
        let mut eq_handles = Vec::with_capacity(self.equality_joints.len());
        for eq in self.equality_joints {
            let l1 = body_handles[eq.link1].as_ref().map(|b| b.body);
            let l2 = body_handles[eq.link2].as_ref().map(|b| b.body);
            let (Some(l1), Some(l2)) = (l1, l2) else {
                continue;
            };
            let mut joint = eq.joint;
            joint.set_enabled(eq.active);
            let h = impulse_joints.insert(l1, l2, joint, true);
            eq_handles.push(MjcfJointHandle {
                joint: h,
                link1: l1,
                link2: l2,
            });
        }
        let actuators = self
            .actuators
            .iter()
            .map(|a| MjcfActuatorHandle {
                actuator: a.actuator.clone(),
                joint: a
                    .joint_index
                    .and_then(|i| joint_handles.get(i))
                    .map(|jh| jh.joint),
            })
            .collect();
        MjcfRobotHandles {
            bodies: body_handles,
            joints: joint_handles,
            equality_joints: eq_handles,
            actuators,
        }
    }

    /// Insert the robot into rapier using multibody joints. Equality
    /// constraints are still inserted as impulse joints (rapier multibodies
    /// don't support extra loop-closure joints).
    pub fn insert_using_multibody_joints(
        self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        multibody_joints: &mut MultibodyJointSet,
        impulse_joints: &mut ImpulseJointSet,
        options: MjcfMultibodyOptions,
    ) -> MjcfRobotHandles<Option<MultibodyJointHandle>> {
        let skip_loop_closures = options.contains(MjcfMultibodyOptions::SKIP_LOOP_CLOSURES);
        let skip_motors = options.contains(MjcfMultibodyOptions::SKIP_JOINT_MOTORS);
        let skip_limits = options.contains(MjcfMultibodyOptions::SKIP_JOINT_LIMITS);
        let world_referenced = self.joints.iter().any(|j| j.link1 == 0 || j.link2 == 0)
            || (!skip_loop_closures
                && self
                    .equality_joints
                    .iter()
                    .any(|j| j.link1 == 0 || j.link2 == 0))
            || !self.bodies[0].colliders.is_empty();
        let mut body_handles: Vec<Option<MjcfBodyHandle>> = vec![None; self.bodies.len()];
        for (i, b) in self.bodies.into_iter().enumerate() {
            if i == 0 && !world_referenced {
                continue;
            }
            let handle = bodies.insert(b.body);
            let mut col_handles = Vec::with_capacity(b.colliders.len());
            for c in b.colliders {
                let h = colliders.insert_with_parent(c, handle, bodies);
                col_handles.push(MjcfColliderHandle { handle: h });
            }
            body_handles[i] = Some(MjcfBodyHandle {
                body: handle,
                colliders: col_handles,
            });
        }
        let mut joint_handles = Vec::with_capacity(self.joints.len());
        for j in self.joints {
            let l1 = body_handles[j.link1].as_ref().map(|b| b.body);
            let l2 = body_handles[j.link2].as_ref().map(|b| b.body);
            let (Some(l1), Some(l2)) = (l1, l2) else {
                joint_handles.push(MjcfJointHandle {
                    joint: None,
                    link1: l1.unwrap_or_else(RigidBodyHandle::invalid),
                    link2: l2.unwrap_or_else(RigidBodyHandle::invalid),
                });
                continue;
            };
            let damping_per_dof = j.damping_per_dof;
            let armature_per_dof = j.armature_per_dof;
            let joint_name = j.name.clone();
            // `limit_axes` / `motor_axes` are bitmasks over the joint's
            // DoFs; clearing them is equivalent to "joint built without
            // .limits(...) / .motor*(...)". The motor/limit data itself
            // is still in the joint struct but the solver ignores it.
            let mut joint = j.joint;
            if skip_limits {
                joint.limit_axes = rapier3d::dynamics::JointAxesMask::empty();
            }
            if skip_motors {
                joint.motor_axes = rapier3d::dynamics::JointAxesMask::empty();
            }
            let h = if options.contains(MjcfMultibodyOptions::JOINTS_ARE_KINEMATIC) {
                multibody_joints.insert_kinematic(l1, l2, joint, true)
            } else {
                multibody_joints.insert(l1, l2, joint, true)
            };
            if h.is_none() {
                // Rapier returns None when the insertion would form a loop
                // in the multibody graph (multibody chains must be trees).
                // The joint is silently dropped — surface that so the user
                // notices.
                log::warn!(
                    "<joint name={joint_name:?}>: dropped from the multibody chain because it would form a loop (rapier multibodies are tree-structured). Use an `<equality><connect>` or `<equality><weld>` for loop closures.",
                );
            }
            if let Some(h) = h {
                if options.contains(MjcfMultibodyOptions::DISABLE_SELF_CONTACTS)
                    && let Some((mb, _)) = multibody_joints.get_mut(h)
                {
                    mb.set_self_contacts_enabled(false);
                }
                // Route MJCF `<joint damping>` through the multibody's
                // per-DoF damping (more numerically stable than motor
                // damping under stiff loads, and naturally covers every
                // free DoF of a ball joint instead of only the motorised
                // AngX). Also zero the motor's damping component so we
                // don't double-damp.
                if damping_per_dof > 0.0 {
                    move_motor_damping_to_multibody(multibody_joints, h, damping_per_dof);
                }
                // Route MJCF `<joint armature>` into the multibody's per-DoF
                // mass-matrix diagonal (joint-space rotor inertia), instead of
                // baking it into the link's spatial inertia tensor.
                if armature_per_dof > 0.0 {
                    add_armature_to_multibody(multibody_joints, h, armature_per_dof);
                }
            }
            joint_handles.push(MjcfJointHandle {
                joint: h,
                link1: l1,
                link2: l2,
            });
        }
        let mut eq_handles = Vec::with_capacity(self.equality_joints.len());
        if !skip_loop_closures {
            for eq in self.equality_joints {
                let l1 = body_handles[eq.link1].as_ref().map(|b| b.body);
                let l2 = body_handles[eq.link2].as_ref().map(|b| b.body);
                let (Some(l1), Some(l2)) = (l1, l2) else {
                    continue;
                };
                let mut joint = eq.joint;
                joint.set_enabled(eq.active);
                let h = impulse_joints.insert(l1, l2, joint, true);
                eq_handles.push(MjcfJointHandle {
                    joint: h,
                    link1: l1,
                    link2: l2,
                });
            }
        }
        let actuators = self
            .actuators
            .iter()
            .map(|a| MjcfActuatorHandle {
                actuator: a.actuator.clone(),
                joint: a
                    .joint_index
                    .and_then(|i| joint_handles.get(i))
                    .map(|jh| jh.joint),
            })
            .collect();
        MjcfRobotHandles {
            bodies: body_handles,
            joints: joint_handles,
            equality_joints: eq_handles,
            actuators,
        }
    }

    /// Append a transform to every rigid-body in this robot.
    ///
    /// The implicit world body (index 0) is intentionally **not** moved —
    /// "the world" is shared by every robot in the scene. To keep joints
    /// that anchor to the world (e.g. a cartpole's slider) satisfied at
    /// rest after the shift, this also transforms each such joint's
    /// world-side local frame by the same transform.
    pub fn append_transform(&mut self, transform: &Pose) {
        for b in self.bodies.iter_mut().skip(1) {
            let p = transform * b.body.position();
            b.body.set_position(p, true);
        }
        // Re-anchor world-attached joint frames so the constraint stays
        // satisfied: if the body moved by `transform`, the world-side
        // local frame must move with it (since the world body itself is
        // fixed at the origin).
        for j in self.joints.iter_mut() {
            if j.link1 == 0 {
                let f = j.joint.local_frame1;
                j.joint.set_local_frame1(*transform * f);
            } else if j.link2 == 0 {
                let f = j.joint.local_frame2;
                j.joint.set_local_frame2(*transform * f);
            }
        }
        for eq in self.equality_joints.iter_mut() {
            if eq.link1 == 0 {
                let f = eq.joint.local_frame1;
                eq.joint.set_local_frame1(*transform * f);
            } else if eq.link2 == 0 {
                let f = eq.joint.local_frame2;
                eq.joint.set_local_frame2(*transform * f);
            }
        }
    }
}
