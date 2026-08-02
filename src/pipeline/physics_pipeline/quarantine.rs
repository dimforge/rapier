//! Containment of non-finite (NaN or infinite) simulation state.
//!
//! Non-finite state is detected at two chokepoints (user modifications at step start, the
//! position advance at step end) before it can corrupt the broad-phase or spread to other
//! bodies; the affected body or collider is disabled, rolled back to its last valid pose when
//! one is known, and reported through [`Quarantine`].

use crate::alloc_prelude::*;
use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet, RigidBodyVelocity};
use crate::geometry::{ColliderHandle, ColliderSet};
use crate::math::{Pose, Vector};
use crate::pipeline::PhysicsPipeline;

/// Non-finite (NaN or infinite) state detected and neutralized during the last step.
///
/// Reported bodies got rolled back to their last valid pose (when known), their velocities and
/// user forces zeroed, and disabled; they can be re-enabled with `set_enabled(true)`. Reports
/// are cleared each step.
#[derive(Default)]
pub struct Quarantine {
    /// Bodies disabled because their pose or velocity went non-finite.
    bodies: Vec<RigidBodyHandle>,
    /// Colliders disabled because their own geometry went non-finite.
    colliders: Vec<ColliderHandle>,
    /// Bodies flagged by the end-of-step advance with their last valid pose;
    /// consumed by `apply_end_step`.
    pub(super) body_scratch: Vec<(RigidBodyHandle, Pose)>,
    /// Colliders flagged by the end-of-step advance with a non-finite AABB despite a finite
    /// body pose; consumed by `apply_end_step`.
    pub(super) collider_scratch: Vec<ColliderHandle>,
}

impl Quarantine {
    /// The rigid-bodies quarantined during the last step.
    pub fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies
    }

    /// The colliders quarantined during the last step.
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders
    }

    /// Was nothing quarantined during the last step?
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty() && self.colliders.is_empty()
    }

    /// Clears the reports at the beginning of a step.
    pub(super) fn clear(&mut self) {
        self.bodies.clear();
        self.colliders.clear();
    }

    /// Neutralizes every non-finite value found in `rb`'s velocities and user forces.
    fn sanitize_body_dynamics(rb: &mut RigidBody) {
        rb.vels = RigidBodyVelocity::zero();
        rb.ccd_vels = RigidBodyVelocity::zero();
        rb.forces.force = Vector::ZERO;
        rb.forces.torque = Default::default();
        rb.forces.user_force = Vector::ZERO;
        rb.forces.user_torque = Default::default();
    }

    /// Step-start chokepoint: quarantines user-modified bodies and colliders whose new state is
    /// non-finite, before it reaches the broad-phase. Quarantined objects are already in the
    /// modified lists, so their disable is processed by the same step's user-changes handling.
    pub(super) fn detect_user_changes(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        for i in 0..bodies.modified_bodies.len() {
            let handle = bodies.modified_bodies[i];
            let Some(rb) = bodies.get_mut_internal(handle) else {
                continue;
            };
            if !rb.is_enabled() {
                continue;
            }

            let position_ok = rb.pos.position.is_finite();
            let next_position_ok = rb.pos.next_position.is_finite();
            if position_ok && next_position_ok && rb.vels.is_finite() {
                continue;
            }

            // Repair the pose from its surviving finite half, if any (e.g. only the kinematic
            // target was invalid).
            if position_ok && !next_position_ok {
                rb.pos.next_position = rb.pos.position;
            } else if !position_ok && next_position_ok {
                rb.pos.position = rb.pos.next_position;
            }
            Self::sanitize_body_dynamics(rb);
            rb.set_enabled(false);
            self.bodies.push(handle);
        }

        for i in 0..colliders.modified_colliders.len() {
            let handle = colliders.modified_colliders[i];
            let Some(co) = colliders.get_mut_internal(handle) else {
                continue;
            };
            if !co.is_enabled() {
                continue;
            }

            let local_aabb = co.shape.compute_local_aabb();
            if co.pos.0.is_finite()
                && local_aabb.mins.is_finite()
                && local_aabb.maxs.is_finite()
                && co
                    .parent
                    .as_ref()
                    .is_none_or(|p| p.pos_wrt_parent.is_finite())
            {
                continue;
            }

            co.set_enabled(false);
            self.colliders.push(handle);
        }
    }

    /// Applies the quarantines detected by the last `advance_to_final_positions` call, which
    /// left the flagged bodies un-advanced (pose, collider positions and mass-properties still
    /// broad-phase-consistent), so rolling back only means discarding `next_position`.
    /// Velocities are zeroed immediately so remaining CCD substeps can't spread them; the
    /// disable is processed next step, like a user `set_enabled(false)` between steps.
    pub(super) fn apply_end_step(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        if self.body_scratch.is_empty() && self.collider_scratch.is_empty() {
            return;
        }

        let mut body_scratch = core::mem::take(&mut self.body_scratch);
        for (handle, prev_pose) in body_scratch.drain(..) {
            let Some(rb) = bodies.get_mut_internal_with_modification_tracking(handle) else {
                continue;
            };

            if prev_pose.is_finite() {
                rb.pos.position = prev_pose;
                rb.pos.next_position = prev_pose;
            } else if rb.pos.position.is_finite() {
                rb.pos.next_position = rb.pos.position;
            }
            Self::sanitize_body_dynamics(rb);

            // Later CCD substeps can re-detect the same body; only the first detection reports.
            if rb.is_enabled() {
                rb.set_enabled(false);
                self.bodies.push(handle);
            }
        }
        self.body_scratch = body_scratch;

        let mut collider_scratch = core::mem::take(&mut self.collider_scratch);
        for handle in collider_scratch.drain(..) {
            let Some(co) = colliders.get_mut_internal_with_modification_tracking(handle) else {
                continue;
            };
            if co.is_enabled() {
                co.set_enabled(false);
                self.colliders.push(handle);
            }
        }
        self.collider_scratch = collider_scratch;
    }
}

impl PhysicsPipeline {
    /// The non-finite state detected and neutralized during the most recent call to
    /// [`Self::step`].
    pub fn quarantine(&self) -> &Quarantine {
        &self.quarantine
    }
}

#[cfg(test)]
mod test {
    use crate::math::{Real, Vector};
    use crate::prelude::{ColliderBuilder, FixedJointBuilder, PhysicsWorld, RigidBodyBuilder};

    fn world_with_ground() -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        // A big fixed ball whose top surface is at y = 0 (shape choice is dimension-agnostic).
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::Y * -10.0),
            ColliderBuilder::ball(10.0),
        );
        world
    }

    fn assert_enabled_bodies_are_finite(world: &PhysicsWorld) {
        for (handle, rb) in world.rigid_bodies() {
            if rb.is_enabled() {
                assert!(
                    rb.position().is_finite() && rb.vels().is_finite(),
                    "enabled body {handle:?} has non-finite state"
                );
            }
        }
    }

    #[test]
    fn healthy_sim_never_quarantines() {
        let mut world = world_with_ground();
        let (handle, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );

        for _ in 0..60 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
            assert!(world.quarantine().colliders().is_empty());
        }

        assert!(world.bodies[handle].position().is_finite());
    }

    #[test]
    fn user_set_nan_position_is_quarantined() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        let (healthy, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::X * 5.0 + Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();

        world.bodies[poisoned].set_translation(Vector::NAN, true);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        assert!(!world.bodies[poisoned].is_enabled());
        assert!(world.bodies[healthy].is_enabled());

        // The report only covers the last step, and the simulation keeps running.
        for _ in 0..10 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
            assert_enabled_bodies_are_finite(&world);
        }
    }

    #[test]
    fn user_set_infinite_position_is_quarantined() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();

        world.bodies[poisoned].set_translation(Vector::Y * Real::INFINITY, true);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        assert!(!world.bodies[poisoned].is_enabled());
        for _ in 0..10 {
            world.step();
            assert_enabled_bodies_are_finite(&world);
        }
    }

    #[test]
    fn user_set_nan_linvel_is_quarantined() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();
        let pose_before = *world.bodies[poisoned].position();

        world.bodies[poisoned].set_linvel(Vector::NAN, true);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        let rb = &world.bodies[poisoned];
        assert!(!rb.is_enabled());
        // The velocity was neutralized before it could corrupt the pose.
        assert_eq!(rb.linvel(), Vector::ZERO);
        assert_eq!(*rb.position(), pose_before);
    }

    #[test]
    fn nan_force_is_quarantined_at_end_of_step() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();
        let pose_before = *world.bodies[poisoned].position();

        // A NaN force is invisible at the start of the step; integration turns it into a NaN
        // velocity and pose mid-step, which the end-of-step chokepoint must catch and roll back.
        world.bodies[poisoned].add_force(Vector::NAN, true);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        let rb = &world.bodies[poisoned];
        assert!(!rb.is_enabled());
        assert_eq!(rb.linvel(), Vector::ZERO);
        // Rolled back to the pose it had at the beginning of the poisoned step.
        assert_eq!(*rb.position(), pose_before);

        for _ in 0..10 {
            world.step();
            assert_enabled_bodies_are_finite(&world);
        }
    }

    #[test]
    fn nan_spread_through_contacts_is_contained() {
        let mut world = world_with_ground();
        let (bottom, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 0.5),
            ColliderBuilder::ball(0.5),
        );
        let (_top, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 1.5),
            ColliderBuilder::ball(0.5),
        );
        // Let the stack settle into persistent contacts.
        for _ in 0..30 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
        }

        world.bodies[bottom].add_force(Vector::NAN, true);
        world.step();

        // The invalid value may legitimately infect the whole island through the contact solver
        // before the end-of-step catch; containment means every infected body is quarantined and
        // nothing else is corrupted.
        assert!(world.quarantine().bodies().contains(&bottom));
        assert_enabled_bodies_are_finite(&world);

        for _ in 0..10 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
            assert_enabled_bodies_are_finite(&world);
        }
    }

    #[test]
    fn joint_partner_survives_user_set_nan_pose() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        let (partner, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 4.0),
            ColliderBuilder::ball(0.5),
        );
        world.insert_impulse_joint(poisoned, partner, FixedJointBuilder::new());
        world.step();

        // Set between steps: the step-start chokepoint catches it before the joint solver can
        // spread it to the partner.
        world.bodies[poisoned].set_translation(Vector::NAN, true);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        assert!(world.bodies[partner].is_enabled());
        for _ in 0..10 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
            assert_enabled_bodies_are_finite(&world);
        }
    }

    #[test]
    fn nan_kinematic_target_is_quarantined() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::kinematic_position_based().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();
        let pose_before = *world.bodies[poisoned].position();

        world.bodies[poisoned].set_next_kinematic_translation(Vector::NAN);
        world.step();

        assert_eq!(world.quarantine().bodies(), &[poisoned]);
        let rb = &world.bodies[poisoned];
        assert!(!rb.is_enabled());
        // Only the kinematic target was invalid; the pose was repaired from its valid half.
        assert_eq!(*rb.position(), pose_before);
    }

    #[test]
    fn nan_shape_standalone_collider_is_quarantined() {
        let mut world = world_with_ground();
        let poisoned = world.insert_collider(ColliderBuilder::ball(Real::NAN), None);
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );

        for _ in 0..10 {
            world.step();
            assert_enabled_bodies_are_finite(&world);
        }
        assert!(!world.colliders[poisoned].is_enabled());
        assert!(world.bodies[body].is_enabled());
    }

    #[test]
    fn nan_shape_attached_collider_is_quarantined_but_body_survives() {
        let mut world = world_with_ground();
        let (body, poisoned) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(Real::NAN),
        );

        for _ in 0..10 {
            world.step();
            assert_enabled_bodies_are_finite(&world);
        }
        // The collider is neutralized; the body itself keeps simulating (in free fall,
        // since it lost its only collider).
        assert!(!world.colliders[poisoned].is_enabled());
        let rb = &world.bodies[body];
        assert!(rb.is_enabled());
        assert!(rb.position().is_finite());
    }

    #[test]
    fn quarantined_body_can_be_resurrected() {
        let mut world = world_with_ground();
        let (poisoned, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
            ColliderBuilder::ball(0.5),
        );
        world.step();
        world.bodies[poisoned].add_force(Vector::NAN, true);
        world.step();
        assert_eq!(world.quarantine().bodies(), &[poisoned]);

        // The quarantine left the body with a finite pose and cleared the poisoned force, so
        // re-enabling it resumes a healthy simulation.
        let rb = &mut world.bodies[poisoned];
        rb.set_translation(Vector::Y * 3.0, true);
        rb.set_enabled(true);

        for _ in 0..30 {
            world.step();
            assert!(world.quarantine().bodies().is_empty());
            assert_enabled_bodies_are_finite(&world);
        }
        assert!(world.bodies[poisoned].is_enabled());
    }
}
