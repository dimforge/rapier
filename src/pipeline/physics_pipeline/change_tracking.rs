//! The physics pipeline's bookkeeping for snapshot deltas: the stamps of the writes that
//! bypass the containers' per-slot stamping, and the delta baselines a step invalidates.

use crate::alloc_prelude::*;
use crate::dynamics::{
    ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodySet, SoftBodySet,
};
use crate::geometry::{BroadPhaseBvh, ColliderSet, NarrowPhase};

use super::PhysicsPipeline;

impl PhysicsPipeline {
    /// Runs before a step: stepping diverges every component from the sender state it
    /// mirrored, and the bodies falling asleep this step are awake now (written before they
    /// sleep).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn begin_change_tracking(
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhaseBvh,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        soft_bodies: &mut SoftBodySet,
    ) {
        #[cfg(feature = "serde-serialize")]
        {
            use crate::data::delta::clear_delta_baseline;
            clear_delta_baseline(islands);
            clear_delta_baseline(broad_phase);
            clear_delta_baseline(narrow_phase);
            clear_delta_baseline(bodies);
            clear_delta_baseline(colliders);
            clear_delta_baseline(impulse_joints);
            clear_delta_baseline(multibody_joints);
            clear_delta_baseline(soft_bodies);
        }
        #[cfg(not(feature = "serde-serialize"))]
        let _ = (broad_phase, narrow_phase, impulse_joints, multibody_joints, soft_bodies);
        Self::stamp_active_set(islands, bodies, colliders);
    }

    /// Runs after the solver: it writes the joints and contacts back through raw pointers.
    pub(super) fn stamp_solver_writes(
        impulse_joints: &mut ImpulseJointSet,
        joint_constraint_indices: &[crate::dynamics::JointIndex],
        narrow_phase: &mut NarrowPhase,
    ) {
        impulse_joints.mark_joints_changed(joint_constraint_indices);
        narrow_phase.mark_solver_contacts_written();
    }

    /// Runs at the end of a step: the bodies woken during the step were written too.
    pub(super) fn end_change_tracking(
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        Self::stamp_active_set(islands, bodies, colliders);
    }

    /// The per-step forward kinematics of every multibody, stamping for snapshot deltas only
    /// the multibodies (and link bodies) it can change: awake or user-edited ones. On the others
    /// it recomputes the stored values bit for bit, so stamping them would bloat every delta.
    pub(super) fn forward_kinematics_tracking_changes(
        bodies: &mut RigidBodySet,
        multibody_joints: &mut MultibodyJointSet,
    ) {
        use crate::dynamics::RigidBodyChanges;

        #[cfg(all(feature = "serde-serialize", debug_assertions))]
        let checks = (
            crate::data::delta::UnstampedWriteCheck::new(&bodies.bodies),
            crate::data::delta::UnstampedWriteCheck::new(&multibody_joints.multibodies),
        );
        let (multibodies, mb_stamps) = multibody_joints.multibodies.iter_mut_unstamped();
        let mb_now = mb_stamps.stamp_now();
        let mut saved_stamps = Vec::new();
        for (index, multibody) in multibodies {
            let slot = index.into_raw_parts().0 as usize;
            let edited = mb_now != 0 && mb_stamps.stamp(slot) == mb_now;
            let changed = edited
                || multibody.links().any(|link| {
                    bodies.get(link.rigid_body).is_some_and(|rb| {
                        !rb.is_sleeping() || rb.changes.contains(RigidBodyChanges::IN_MODIFIED_SET)
                    })
                });
            saved_stamps.clear();
            if !changed {
                saved_stamps.extend(
                    multibody
                        .links()
                        .map(|link| (link.rigid_body, bodies.bodies.stamp_of(link.rigid_body.0))),
                );
            }
            multibody.forward_kinematics(bodies, true);
            multibody.update_rigid_bodies_internal(bodies, true, false, false);
            if changed {
                mb_stamps.mark(slot);
            } else {
                for (handle, stamp) in &saved_stamps {
                    bodies
                        .bodies
                        .stamps
                        .set_stamp(handle.0.into_raw_parts().0 as usize, *stamp);
                }
            }
        }
            #[cfg(all(feature = "serde-serialize", debug_assertions))]
        {
            checks.0.check(&bodies.bodies, "multibody forward kinematics");
            checks.1.check(&multibody_joints.multibodies, "multibody forward kinematics");
        }
    }

    /// Stamps the awake bodies and their colliders for snapshot deltas: the solver and the
    /// integration write them through raw pointers, bypassing the per-slot stamping.
    fn stamp_active_set(
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        if !bodies.bodies.stamps.is_enabled() && !colliders.colliders.stamps.is_enabled() {
            return;
        }
        for handle in islands.active_bodies() {
            bodies.bodies.mark_changed(handle.0);
            if let Some(rb) = bodies.get(handle) {
                for co in rb.colliders() {
                    colliders.colliders.mark_changed(co.0);
                }
            }
        }
    }
}
