//! Serial-joint construction. Given the MJCF joint description and the
//! world poses of the parent / child rapier bodies, this produces the
//! [`GenericJoint`] that captures axis orientation, limits, motors, and
//! per-DoF springs.

use mjcf_rs::Pose as MPose;
use mjcf_rs::body as mb;
use mjcf_rs::glam::{DQuat, DVec3};

use rapier3d::dynamics::{GenericJoint, GenericJointBuilder, JointAxesMask, JointAxis, MotorModel};
use rapier3d::math::{Pose, Real};

use super::conversion::Conversion;

impl<'a> Conversion<'a> {
    /// Build a serial joint connecting `prev` body to `cur` body.
    pub(super) fn build_serial_joint(
        &self,
        joint: &mb::Joint,
        prev_world_pose: MPose,
        cur_world_pose: MPose,
    ) -> GenericJoint {
        let s = self.options.scale as f64;
        // The MJCF joint anchor is in the **child body's local frame**.
        // For multi-joint chains, only the first joint has the body's full
        // pose between the parent and child; subsequent joints sit at the
        // body frame's origin (the intermediate bodies are at the body's
        // world pose). That distinction is captured by the caller through
        // `prev_world_pose` (the parent pose for the first joint, the body
        // pose afterwards), so the world-space anchor is computed the same
        // way regardless of position in the chain.
        let anchor_in_child = MPose::from_translation(DVec3::new(
            joint.pos[0] * s,
            joint.pos[1] * s,
            joint.pos[2] * s,
        ));
        let anchor_world = cur_world_pose * anchor_in_child;
        let local1 = prev_world_pose.inverse() * anchor_world;
        let local2 = anchor_in_child;

        // Build the joint axes mask.
        let locked = match joint.type_ {
            mb::JointType::Hinge => JointAxesMask::LOCKED_REVOLUTE_AXES,
            mb::JointType::Slide => JointAxesMask::LOCKED_PRISMATIC_AXES,
            mb::JointType::Ball => JointAxesMask::LOCKED_SPHERICAL_AXES,
            mb::JointType::Free => JointAxesMask::empty(),
        };

        // Re-orient the joint frames so the rapier "X" axis lines up with
        // MJCF's `axis`.
        let axis = joint.axis;
        let axis_norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let mut frame1 = local1;
        let mut frame2 = local2;
        if matches!(joint.type_, mb::JointType::Hinge | mb::JointType::Slide) && axis_norm > 1e-12 {
            let frame_rot = MPose::from_rotation(axis_basis_quat(axis));
            // Apply the same rotation on both sides so the joint resolves at zero at rest.
            frame1 *= frame_rot;
            frame2 *= frame_rot;
        }

        let mut builder = GenericJointBuilder::new(locked)
            .contacts_enabled(self.options.enable_joint_collisions)
            .local_frame1(Pose::from(frame1))
            .local_frame2(Pose::from(frame2));

        // Limits.
        //
        // MJCF's `ref` shifts the joint's zero: at the body's declared rest
        // pose, the joint reads `ref`. Rapier reads zero at the rest pose
        // of the joint frames, so we subtract `ref` from MJCF's limits to
        // express them in rapier's convention. (For prismatic joints,
        // `ref` is in the joint's length unit and the lengths are scaled
        // by `options.scale` just like `range`.)
        let limited = joint
            .limited
            .resolve(self.model.compiler.autolimits, joint.range.is_some());
        if limited && let Some(r) = joint.range {
            let s_lin = self.options.scale;
            let ref_ = joint.ref_ as Real;
            let ref_lin = ref_ * s_lin;
            match joint.type_ {
                mb::JointType::Hinge => {
                    builder =
                        builder.limits(JointAxis::AngX, [r[0] as Real - ref_, r[1] as Real - ref_]);
                }
                mb::JointType::Slide => {
                    builder = builder.limits(
                        JointAxis::LinX,
                        [
                            r[0] as Real * s_lin - ref_lin,
                            r[1] as Real * s_lin - ref_lin,
                        ],
                    );
                }
                _ => {}
            }
        }

        if !self.options.disable_joint_motors {
            // Spring / damping / armature / etc.
            let stiffness = if let Some(sd) = joint.springdamper {
                let timeconst = sd[0];
                let dampratio = sd[1];
                // MuJoCo uses an under-damped spring with: stiffness = (2π/T)² * m_eff
                // and damping = 2 ξ √(stiffness * m_eff). We don't know m_eff
                // here — fall back to interpreting the pair as (stiffness, damping)
                // when timeconst is zero (the explicit form).
                if timeconst > 0.0 {
                    // Treat (2π/T)² as the stiffness coefficient assuming m_eff=1.
                    let omega = std::f64::consts::TAU / timeconst;
                    Some((omega * omega, 2.0 * dampratio * omega))
                } else {
                    Some((dampratio, 0.0))
                }
            } else if joint.stiffness > 0.0 || joint.damping > 0.0 {
                Some((joint.stiffness, joint.damping))
            } else {
                None
            };
            let target_pos = joint.springref - joint.ref_;
            if let Some((k, d)) = stiffness {
                let axis = match joint.type_ {
                    mb::JointType::Hinge | mb::JointType::Ball => Some(JointAxis::AngX),
                    mb::JointType::Slide => Some(JointAxis::LinX),
                    _ => None,
                };
                if let Some(ax) = axis {
                    // AccelerationBased decouples the motor's effective
                    // stiffness from the link's inertia, which makes the
                    // timestep-stability margin uniform across the chain. For
                    // stiff MJCF springs (e.g. cassie's shin/heel springs at
                    // k=1250–1500) ForceBased was numerically unstable on the
                    // multibody path and produced NaN within a few steps.
                    builder = builder.motor_model(ax, MotorModel::AccelerationBased);
                    builder = builder.motor_position(ax, target_pos as Real, k as Real, d as Real);
                }
            }

            // Friction loss (lossy approximation): use a velocity motor
            // capped at `frictionloss`.
            if joint.frictionloss > 0.0 {
                let axis = match joint.type_ {
                    mb::JointType::Hinge | mb::JointType::Ball => Some(JointAxis::AngX),
                    mb::JointType::Slide => Some(JointAxis::LinX),
                    _ => None,
                };
                if let Some(ax) = axis {
                    builder = builder.motor_velocity(ax, 0.0, 0.0);
                    builder = builder.motor_max_force(ax, joint.frictionloss as Real);
                }
            }
        }

        builder.build()
    }
}

fn axis_basis_quat(axis: [f64; 3]) -> DQuat {
    // Build a rotation that maps the world X axis to `axis`. For prismatic
    // / hinge joints, rapier locks all but the X axis, so the free axis is
    // X — re-orienting both joint frames by this rotation lines up the
    // joint's free axis with the MJCF-specified axis.
    let a = DVec3::from_array(axis);
    let n = a.length();
    if n <= 1e-15 {
        DQuat::IDENTITY
    } else {
        DQuat::from_rotation_arc(DVec3::X, a / n)
    }
}
