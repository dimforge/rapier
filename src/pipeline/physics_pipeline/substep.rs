//! The pipeline's inner step loop: CCD substepping and motion clamping, plus
//! end-of-step advancement of bodies, colliders and broad-phase AABBs.

use crate::alloc_prelude::*;

use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodyChanges, RigidBodySet, RigidBodyType,
};
#[cfg(feature = "parallel")]
use crate::geometry::ColliderHandle;
use crate::geometry::{
    BroadPhaseBvh, ColliderChanges, ColliderSet, ModifiedColliders, NarrowPhase,
};
use crate::math::{Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks};
use crate::prelude::ModifiedRigidBodies;

use super::PhysicsPipeline;

impl PhysicsPipeline {
    fn clear_modified_colliders(
        &mut self,
        colliders: &mut ColliderSet,
        modified_colliders: &mut ModifiedColliders,
    ) {
        // Every collider with a non-empty change flag is in the modified set (flags are
        // only set by the user-modification paths; internal motion doesn't use flags), so
        // clearing the listed colliders is exhaustive — O(modified), not an O(total) sweep.
        for handle in modified_colliders.iter() {
            if let Some(co) = colliders.get_mut_internal(*handle) {
                co.changes = ColliderChanges::empty();
            }
        }

        modified_colliders.clear();
    }

    fn clear_modified_bodies(
        &mut self,
        bodies: &mut RigidBodySet,
        modified_bodies: &mut ModifiedRigidBodies,
    ) {
        for handle in modified_bodies.iter() {
            if let Some(rb) = bodies.get_mut_internal(*handle) {
                rb.changes = RigidBodyChanges::empty();
            }
        }

        modified_bodies.clear();
    }

    #[allow(clippy::too_many_arguments)]
    fn run_ccd_motion_clamping(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        broad_phase: &mut BroadPhaseBvh,
        narrow_phase: &NarrowPhase,
        ccd_solver: &mut CCDSolver,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
        scene_changed: bool,
    ) {
        self.counters.ccd.toi_computation_time.start();
        // Handle CCD: sweep the fast bodies and clamp their `next_position` to their
        // earliest time of impact (velocities are preserved).
        ccd_solver.solve_continuous(
            integration_parameters,
            islands,
            bodies,
            colliders,
            broad_phase,
            narrow_phase,
            hooks,
            events,
            scene_changed,
        );
        self.counters.ccd.toi_computation_time.pause();
    }

    fn advance_to_final_positions(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        // Set bodies to their final position, propagate to colliders, refresh world mass-properties
        // (user code between steps applies forces w.r.t. the fresh CoM). NOTE: internal motion skips the
        // user-modification tracking — moved colliders are harvested here for the broad-phase refresh and narrow-phase walk (no flags).
        use parry::bounding_volume::BoundingVolume;

        self.end_step_collider_aabbs.clear();
        let prediction = integration_parameters.prediction_distance();
        let dt = integration_parameters.dt;

        // Broad-phase AABB of a just-moved collider, with its parent body in hand
        // (same semantics as `Collider::compute_broad_phase_aabb`, minus its
        // per-collider body-arena lookup for the soft-CCD check).
        let collider_aabb = |co: &crate::geometry::Collider,
                             rb: &crate::dynamics::RigidBody|
         -> crate::geometry::Aabb {
            let mut aabb = co.compute_collision_aabb(prediction / 2.0);
            if rb.soft_ccd_prediction() > 0.0 {
                let next_pose = rb.predict_position_using_velocity_and_forces_with_max_dist(
                    dt,
                    rb.soft_ccd_prediction(),
                ) * co.parent.as_ref().unwrap().pos_wrt_parent;
                let next_aabb = co
                    .shape
                    .compute_aabb(&next_pose)
                    .loosened(co.contact_skin() + prediction / 2.0);
                aabb.merge(&next_aabb);
            }
            aabb
        };

        #[cfg(not(feature = "parallel"))]
        {
            for handle in islands.active_bodies() {
                let rb = bodies.index_mut_internal(handle);
                // Non-finite pose: leave the body at its last valid state for
                // `Quarantine::apply_end_step` to neutralize.
                if !rb.pos.next_position.is_finite() {
                    self.quarantine.body_scratch.push((handle, rb.pos.position));
                    continue;
                }
                rb.pos.position = rb.pos.next_position;
                for co_handle in rb.colliders.0.iter() {
                    let co = colliders.index_mut_internal(*co_handle);
                    let new_pos = rb.pos.position * co.parent.as_ref().unwrap().pos_wrt_parent;
                    co.pos = crate::geometry::ColliderPosition(new_pos);
                    if co.is_enabled() {
                        let aabb = collider_aabb(co, rb);
                        if aabb.mins.is_finite() && aabb.maxs.is_finite() {
                            self.end_step_collider_aabbs.push((*co_handle, aabb));
                        } else {
                            // Finite body pose but non-finite AABB: the collider's own
                            // geometry is invalid.
                            self.quarantine.collider_scratch.push(*co_handle);
                        }
                    }
                }
                rb.mprops
                    .update_world_mass_properties(rb.body_type, &rb.pos.position);
            }
        }

        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;

            self.active_body_handles.clear();
            self.active_body_handles.extend(islands.active_bodies());
            let bodies_ptr = core::sync::atomic::AtomicPtr::new(bodies as *mut RigidBodySet);
            let colliders_ptr = core::sync::atomic::AtomicPtr::new(colliders as *mut ColliderSet);

            type ChunkResult = (
                Vec<(ColliderHandle, crate::geometry::Aabb)>,
                Vec<(crate::dynamics::RigidBodyHandle, crate::math::Pose)>,
                Vec<ColliderHandle>,
            );
            let moved: Vec<ChunkResult> = self
                .active_body_handles
                .par_chunks(256)
                .map(|chunk| {
                    // SAFETY: the body handles are distinct, and each collider has a
                    //         single parent, so all the mutated bodies and colliders
                    //         are disjoint across this loop.
                    let bodies =
                        unsafe { &mut *bodies_ptr.load(core::sync::atomic::Ordering::Relaxed) };
                    let colliders =
                        unsafe { &mut *colliders_ptr.load(core::sync::atomic::Ordering::Relaxed) };
                    let mut moved = Vec::new();
                    let mut quarantined_bodies = Vec::new();
                    let mut quarantined_colliders = Vec::new();

                    for handle in chunk {
                        let rb = bodies.index_mut_internal(*handle);
                        // Non-finite pose containment; see the serial branch.
                        if !rb.pos.next_position.is_finite() {
                            quarantined_bodies.push((*handle, rb.pos.position));
                            continue;
                        }
                        rb.pos.position = rb.pos.next_position;

                        for co_handle in rb.colliders.0.iter() {
                            let co = colliders.index_mut_internal(*co_handle);
                            let new_pos =
                                rb.pos.position * co.parent.as_ref().unwrap().pos_wrt_parent;
                            co.pos = crate::geometry::ColliderPosition(new_pos);
                            if co.is_enabled() {
                                let aabb = collider_aabb(co, rb);
                                if aabb.mins.is_finite() && aabb.maxs.is_finite() {
                                    moved.push((*co_handle, aabb));
                                } else {
                                    quarantined_colliders.push(*co_handle);
                                }
                            }
                        }

                        rb.mprops
                            .update_world_mass_properties(rb.body_type, &rb.pos.position);
                    }

                    (moved, quarantined_bodies, quarantined_colliders)
                })
                .collect();

            // Chunks are collected in order, so the harvested lists are deterministic.
            for (chunk, quarantined_bodies, quarantined_colliders) in &moved {
                self.end_step_collider_aabbs.extend_from_slice(chunk);
                self.quarantine
                    .body_scratch
                    .extend_from_slice(quarantined_bodies);
                self.quarantine
                    .collider_scratch
                    .extend_from_slice(quarantined_colliders);
            }
        }
    }

    /// Feeds the broad-phase the AABBs computed by the last `advance_to_final_positions`
    /// call, through `set_aabb` (whose `pending_set_aabb` protocol makes the next
    /// broad-phase update account for them in change-flag resolution and stale-pair detection).
    fn refresh_moved_collider_aabbs(
        &mut self,
        integration_parameters: &IntegrationParameters,
        broad_phase: &mut BroadPhaseBvh,
    ) {
        // Join the concurrent tree-optimization pass right before the tree writes.
        self.join_deferred_bvh_optimize(broad_phase);

        for (handle, aabb) in &self.end_step_collider_aabbs {
            broad_phase.set_aabb(integration_parameters, *handle, *aabb);
        }
    }

    fn interpolate_kinematic_velocities(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
    ) {
        // Update kinematic bodies velocities.
        // TODO: what is the best place for this? It should at least be
        // located before the island computation because we test the velocity
        // there to determine if this kinematic body should wake-up dynamic
        // bodies it is touching.
        for handle in islands.active_bodies() {
            // TODO PERF: only iterate on kinematic position-based bodies
            let rb = bodies.index_mut_internal(handle);

            if rb.body_type == RigidBodyType::KinematicPositionBased {
                rb.vels = rb.pos.interpolate_velocity(
                    integration_parameters.inv_dt(),
                    rb.mprops.local_mprops.local_com,
                );
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn step_inner(
        &mut self,
        gravity: Vector,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhaseBvh,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        ccd_solver: &mut CCDSolver,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        self.counters.reset();
        self.counters.step_started();
        self.quarantine.clear();

        // Apply some of delayed wake-ups.
        self.counters.stages.user_changes.start();
        #[cfg(feature = "enhanced-determinism")]
        let to_wake_up_iterator = impulse_joints
            .to_wake_up
            .drain(..)
            .chain(multibody_joints.to_wake_up.drain(..));
        #[cfg(not(feature = "enhanced-determinism"))]
        let to_wake_up_iterator = impulse_joints
            .to_wake_up
            .drain()
            .chain(multibody_joints.to_wake_up.drain());
        for handle in to_wake_up_iterator {
            islands.wake_up(bodies, handle, true);
        }

        // Quarantine user-introduced non-finite state before it reaches the broad-phase.
        self.quarantine.detect_user_changes(bodies, colliders);

        // Apply modifications.
        let mut modified_colliders = colliders.take_modified();
        let mut removed_colliders = colliders.take_removed();

        crate::pipeline::user_changes::handle_user_changes_to_colliders(
            bodies,
            colliders,
            &modified_colliders[..],
        );

        let mut modified_bodies = bodies.take_modified();
        crate::pipeline::user_changes::handle_user_changes_to_rigid_bodies(
            Some(islands),
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            &modified_bodies,
            &mut modified_colliders,
        );

        // Disabled colliders are treated as if they were removed.
        // NOTE: this must be called here, after handle_user_changes_to_rigid_bodies to take into
        //       account colliders disabled because of their parent rigid-body.
        removed_colliders.extend(
            modified_colliders
                .iter()
                .copied()
                .filter(|h| colliders.get(*h).map(|c| !c.is_enabled()).unwrap_or(false)),
        );

        // Whether any user change could have added, removed or moved a FIXED
        // collider this step — the CCD fixed-target cache invalidation signal
        // (internal motion never touches fixed colliders nor these lists).
        let ccd_scene_changed = !modified_colliders.is_empty()
            || !removed_colliders.is_empty()
            || !modified_bodies.is_empty();

        // Join islands based on new joints.
        #[cfg(feature = "enhanced-determinism")]
        let to_join_iterator = impulse_joints
            .to_join
            .drain(..)
            .chain(multibody_joints.to_join.drain(..));
        #[cfg(not(feature = "enhanced-determinism"))]
        let to_join_iterator = impulse_joints
            .to_join
            .drain()
            .chain(multibody_joints.to_join.drain());
        for (handle1, handle2) in to_join_iterator {
            islands.interaction_changed(bodies, Some(handle1), Some(handle2), false);
        }

        // Persistent islands: apply the joint connectivity edits (in order).
        let joint_island_events: Vec<_> = core::mem::take(&mut impulse_joints.island_events);
        for event in joint_island_events {
            islands.apply_impulse_joint_island_event(bodies, event);
        }
        let mut mb_chain_events = core::mem::take(&mut multibody_joints.island_chain_events);
        for mb_id in &mb_chain_events {
            islands.refresh_multibody_chain(bodies, multibody_joints, *mb_id);
        }
        mb_chain_events.clear();
        multibody_joints.island_chain_events = mb_chain_events;
        self.counters.stages.user_changes.pause();

        // TODO: do this only on user-change.
        // TODO: do we want some kind of automatic inverse kinematics?
        for multibody in &mut multibody_joints.multibodies {
            multibody.1.forward_kinematics(bodies, true);
            multibody
                .1
                .update_rigid_bodies_internal(bodies, true, false, false);
        }

        self.detect_collisions(
            integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            &modified_colliders,
            &removed_colliders,
            hooks,
            events,
            true,
        );

        self.counters.stages.user_changes.resume();
        self.clear_modified_colliders(colliders, &mut modified_colliders);
        self.clear_modified_bodies(bodies, &mut modified_bodies);
        removed_colliders.clear();
        self.counters.stages.user_changes.pause();

        let mut remaining_time = integration_parameters.dt;
        let mut integration_parameters = *integration_parameters;

        let (ccd_is_enabled, mut remaining_substeps) =
            if integration_parameters.max_ccd_substeps == 0 {
                (false, 1)
            } else {
                (true, integration_parameters.max_ccd_substeps)
            };

        while remaining_substeps > 0 {
            // If there are more than one CCD substep, we need to split
            // the timestep into multiple intervals. First, estimate the
            // size of the time slice we will integrate for this substep.
            //
            // Note that we must do this now, before the constraints resolution
            // because we need to use the correct timestep length for the
            // integration of external forces.
            //
            // If there is only one or zero CCD substep, there is no need
            // to split the timestep interval. So we can just skip this part.
            if ccd_is_enabled && remaining_substeps > 1 {
                // NOTE: Take forces into account when updating the bodies CCD activation flags
                //       these forces have not been integrated to the body's velocity yet.
                let ccd_active =
                    ccd_solver.update_ccd_active_flags(islands, bodies, remaining_time, true);
                self.join_deferred_bvh_optimize(broad_phase);
                let first_impact = if ccd_active {
                    ccd_solver.find_first_impact(
                        remaining_time,
                        &integration_parameters,
                        islands,
                        bodies,
                        colliders,
                        broad_phase,
                        narrow_phase,
                        hooks,
                    )
                } else {
                    None
                };

                if let Some(toi) = first_impact {
                    let original_interval = remaining_time / (remaining_substeps as Real);

                    if toi < original_interval {
                        integration_parameters.dt = original_interval;
                    } else {
                        integration_parameters.dt =
                            toi + (remaining_time - toi) / (remaining_substeps as Real);
                    }

                    remaining_substeps -= 1;
                } else {
                    // No impact, don't do any other substep after this one.
                    integration_parameters.dt = remaining_time;
                    remaining_substeps = 0;
                }

                remaining_time -= integration_parameters.dt;

                // Avoid substep length that are too small.
                if remaining_time <= integration_parameters.min_ccd_dt {
                    integration_parameters.dt += remaining_time;
                    remaining_substeps = 0;
                }
            } else {
                integration_parameters.dt = remaining_time;
                remaining_time = 0.0;
                remaining_substeps = 0;
            }

            self.counters.ccd.num_substeps += 1;

            self.counters.custom.resume();
            self.interpolate_kinematic_velocities(&integration_parameters, islands, bodies);
            self.counters.custom.pause();
            self.build_islands_and_solve_velocity_constraints(
                gravity,
                &integration_parameters,
                islands,
                narrow_phase,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                events,
            );

            // If CCD is enabled, execute the CCD motion clamping.
            if ccd_is_enabled {
                // The staged solver's body writeback already computed the post-solve CCD flags;
                // a serial walk is only needed when that verdict is unavailable (multibodies).
                // NOTE: don't include forces — the solver already integrated them into the velocities.
                let ccd_active = match self.staged_solver.post_solve_ccd_active {
                    Some(any_active) => any_active,
                    None => ccd_solver.update_ccd_active_flags(
                        islands,
                        bodies,
                        integration_parameters.dt,
                        false,
                    ),
                };
                if ccd_active {
                    self.join_deferred_bvh_optimize(broad_phase);
                    self.run_ccd_motion_clamping(
                        &integration_parameters,
                        islands,
                        bodies,
                        colliders,
                        broad_phase,
                        narrow_phase,
                        ccd_solver,
                        hooks,
                        events,
                        ccd_scene_changed,
                    );
                }
            }

            self.counters.stages.update_time.resume();
            self.advance_to_final_positions(&integration_parameters, islands, bodies, colliders);
            // Neutralize bodies whose integrated pose went non-finite before the remaining
            // CCD substeps can spread their velocities.
            self.quarantine.apply_end_step(bodies, colliders);
            self.counters.stages.update_time.pause();

            if remaining_substeps > 0 {
                // Feed the just-moved colliders' AABBs to the broad-phase before
                // re-running collision detection for the next CCD substep.
                self.counters.stages.collision_detection_time.resume();
                self.counters.cd.final_broad_phase_time.resume();
                self.refresh_moved_collider_aabbs(&integration_parameters, broad_phase);
                self.counters.cd.final_broad_phase_time.pause();
                self.counters.stages.collision_detection_time.pause();

                self.detect_collisions(
                    &integration_parameters,
                    islands,
                    broad_phase,
                    narrow_phase,
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    &modified_colliders,
                    &[],
                    hooks,
                    events,
                    false,
                );

                self.clear_modified_colliders(colliders, &mut modified_colliders);
            } else {
                // If we ran the last substep, just update the broad-phase bvh instead
                // of a full collision-detection step. Internal motion doesn't go
                // through the modification tracking anymore: the moved colliders were
                // harvested by `advance_to_final_positions`.
                self.counters.stages.collision_detection_time.resume();
                self.counters.cd.final_broad_phase_time.resume();
                self.refresh_moved_collider_aabbs(&integration_parameters, broad_phase);
                self.counters.cd.final_broad_phase_time.pause();
                self.counters.stages.collision_detection_time.pause();
            }
        }

        // Finally, make sure we update the world mass-properties of the rigid-bodies
        // that moved. Otherwise, users may end up applying forces with respect to an
        // outdated center of mass.
        // TODO: avoid updating the world mass properties twice (here, and
        //       at the beginning of the next timestep) for bodies that were
        //       not modified by the user in the mean time.
        // NOTE: the world mass-properties of the bodies that moved were refreshed by
        //       `advance_to_final_positions`.

        // Re-insert the modified vector we extracted for the borrow-checker.
        colliders.set_modified(modified_colliders);

        self.counters.step_completed();
    }
}
