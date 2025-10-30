use crate::dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBody, RigidBodyActivation, RigidBodyChanges,
    RigidBodyColliders, RigidBodyHandle, RigidBodyIds, RigidBodySet, RigidBodyType,
    RigidBodyVelocity,
};
use crate::geometry::{ColliderSet, ContactPair, NarrowPhase};
use crate::math::Real;
use crate::prelude::{ColliderHandle, ContactManifoldData};
use crate::utils::SimdDot;
use std::collections::VecDeque;
use std::ops::IndexMut;
use vec_map::VecMap;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
struct Island {
    bodies: Vec<RigidBodyHandle>,
    additional_solver_iterations: usize,
}

/// System that manages which bodies are active (awake) vs sleeping to optimize performance.
///
/// ## Sleeping Optimization
///
/// Bodies at rest automatically "sleep" - they're excluded from simulation until something
/// disturbs them (collision, joint connection to moving body, manual wake-up). This can
/// dramatically improve performance in scenes with many static/resting objects.
///
/// ## Islands
///
/// Connected bodies (via contacts or joints) are grouped into "islands" that are solved together.
/// This allows parallel solving and better organization.
///
/// You rarely interact with this directly - it's automatically managed by [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct IslandManager {
    // Island 0 is active, all the others are sleeping.
    pub(crate) islands: VecMap<Island>,
    pub(crate) free_islands: Vec<usize>,
    /// Potential candidate roots for graph traversal to identify a sleeping
    /// connected component.
    traversal_candidates: VecDeque<RigidBodyHandle>,
    /// Same as `traversal_candidates` but these roots **must** all be traversed as the next
    /// timestep to properly handle bodies that are sleeping at creation.
    priority_traversal_candidates: Vec<RigidBodyHandle>,
    timestamp: u32, // TODO: the physics pipeline (or something else) should expose a step_id?
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    stack: Vec<RigidBodyHandle>, // Workspace.
}

impl Default for IslandManager {
    fn default() -> Self {
        let mut islands = VecMap::new();
        islands.insert(0, Island::default());
        Self {
            islands,
            free_islands: Default::default(),
            traversal_candidates: Default::default(),
            priority_traversal_candidates: Default::default(),
            timestamp: Default::default(),
            stack: Default::default(),
        }
    }
}

impl IslandManager {
    /// Creates a new empty island manager.
    pub fn new() -> Self {
        Self::default()
    }

    // TODO: rename this to `num_active_islands`?
    pub(crate) fn num_active_islands(&self) -> usize {
        1
    }

    pub(crate) fn rigid_body_removed(
        &mut self,
        removed_handle: RigidBodyHandle,
        removed_ids: &RigidBodyIds,
        bodies: &mut RigidBodySet,
    ) {
        let Some(island) = self.islands.get_mut(removed_ids.active_island_id) else {
            // The island already doesn’t exist.
            return;
        };

        let swapped_handle = island.bodies.last().copied().unwrap_or(removed_handle);
        island
            .bodies
            .swap_remove(removed_ids.active_set_offset as usize);

        if swapped_handle != removed_handle {
            let swapped_body = bodies
                .get_mut(swapped_handle)
                .expect("Internal error: bodies must be removed from islands on at a times");
            swapped_body.ids.active_set_offset = removed_ids.active_set_offset;
            swapped_body.ids.active_set_id = removed_ids.active_set_id;
        }

        if removed_ids.active_island_id != 0 && island.bodies.is_empty() {
            // We removed the last body from this island. Remove the island (but never remove
            // island 0 since it’s the special active island).
            self.islands.remove(removed_ids.active_island_id);
            self.free_islands.push(removed_ids.active_island_id);
        }
    }

    pub(crate) fn contact_started_or_stopped(
        &mut self,
        bodies: &mut RigidBodySet,
        handle1: Option<RigidBodyHandle>,
        handle2: Option<RigidBodyHandle>,
        started: bool,
    ) {
        if started {
            if let Some(handle1) = handle1 {
                self.wake_up(bodies, handle1, false);
            }
            if let Some(handle2) = handle2 {
                self.wake_up(bodies, handle2, false);
            }
        }
    }

    /// Wakes up a sleeping body, forcing it back into the active simulation.
    ///
    /// Use this when you want to ensure a body is active (useful after manually moving
    /// a sleeping body, or to prevent it from sleeping in the next few frames).
    ///
    /// # Parameters
    /// * `strong` - If `true`, the body is guaranteed to stay awake for multiple frames.
    ///   If `false`, it might sleep again immediately if conditions are met.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut islands = IslandManager::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// islands.wake_up(&mut bodies, body_handle, true);
    /// let body = bodies.get_mut(body_handle).unwrap();
    /// // Wake up a body before applying force to it
    /// body.add_force(vector![100.0, 0.0, 0.0], false);
    /// ```
    ///
    /// Only affects dynamic bodies (kinematic and fixed bodies don't sleep).
    pub fn wake_up(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle, strong: bool) {
        // NOTE: the use an Option here because there are many legitimate cases (like when
        //       deleting a joint attached to an already-removed body) where we could be
        //       attempting to wake-up a rigid-body that has already been deleted.
        if bodies.get(handle).map(|rb| rb.is_dynamic_or_kinematic()) == Some(true) {
            let rb = bodies.index_mut_internal(handle);

            // TODO: not sure if this is still relevant:
            // // Check that the user didn’t change the sleeping state explicitly, in which
            // // case we don’t overwrite it.
            // if rb.changes.contains(RigidBodyChanges::SLEEP) {
            //     return;
            // }

            rb.activation.wake_up(strong);
            if rb.is_enabled() && rb.ids.active_island_id != 0 {
                // Wake up the sleeping island this rigid-body is part of.
                let Some(removed_island) = self.islands.remove(rb.ids.active_island_id) else {
                    // TODO: the island doesn’t exist is that an internal error?
                    return;
                };

                self.free_islands.push(rb.ids.active_island_id);

                // TODO: if we switched to linked list, we could avoid moving around all this memory.
                let active_island = &mut self.islands[0];
                for handle in &removed_island.bodies {
                    let Some(rb) = bodies.get_mut_internal(*handle) else {
                        // This body no longer exists.
                        continue;
                    };
                    rb.wake_up(false);
                    rb.ids.active_island_id = 0;
                    rb.ids.active_set_id = active_island.bodies.len();
                    rb.ids.active_set_offset = (active_island.bodies.len()) as u32;
                    active_island.bodies.push(*handle);
                }

                active_island.additional_solver_iterations = active_island
                    .additional_solver_iterations
                    .max(removed_island.additional_solver_iterations);
            }
        }
    }

    // TODO: remove the island_id argument?
    pub(crate) fn active_island(&self, island_id: usize) -> &[RigidBodyHandle] {
        assert_eq!(island_id, 0);
        &self.islands[0].bodies
    }

    pub(crate) fn active_island_additional_solver_iterations(&self, island_id: usize) -> usize {
        self.islands[0].additional_solver_iterations
    }

    /// Handles of dynamic and kinematic rigid-bodies that are currently active (i.e. not sleeping).
    #[inline]
    pub fn active_bodies(&self) -> &[RigidBodyHandle] {
        &self.islands[0].bodies
    }

    // TODO: what if the body is sleeping when we add it?
    pub(crate) fn update_body(&mut self, handle: RigidBodyHandle, bodies: &mut RigidBodySet) {
        let Some(rb) = bodies.get_mut(handle) else {
            return;
        };

        if rb.is_fixed() {
            return;
        }

        let id = self.islands[0].bodies.len();

        if rb.ids.active_set_id == usize::MAX {
            // This body is new.
            rb.ids.active_island_id = 0;
            rb.ids.active_set_id = id;
            rb.ids.active_set_offset = id as u32;
            self.islands[0].bodies.push(handle);

            if rb.is_sleeping() {
                // The rigid-body is sleeping. We need to move it into a sleeping island during
                // the next step.
                self.priority_traversal_candidates.push(handle);
            }
        }

        // Push the body to the active set if it is not inside the active set yet, and
        // is not longer sleeping or became dynamic.
        if (rb.changes.contains(RigidBodyChanges::SLEEP)
            || rb.changes.contains(RigidBodyChanges::TYPE))
            && rb.is_enabled()
            // Don’t wake up if the user put it to sleep manually.
            && !rb.activation.sleeping
        {
            self.wake_up(bodies, handle, false);
        }
    }

    pub(crate) fn update_active_set_with_contacts(
        &mut self,
        dt: Real,
        length_unit: Real,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        min_island_size: usize,
    ) {
        self.timestamp += 1;
        // 1. Update active rigid-bodies energy.
        // let t0 = std::time::Instant::now();
        for handle in &self.islands[0].bodies {
            let Some(rb) = bodies.get_mut_internal(*handle) else {
                // This branch happens if the rigid-body no longer exists.
                continue;
            };
            let sq_linvel = rb.vels.linvel.norm_squared();
            let sq_angvel = rb.vels.angvel.gdot(rb.vels.angvel);
            let can_sleep_before = rb.activation.is_eligible_for_sleep();

            rb.activation
                .update_energy(rb.body_type, length_unit, sq_linvel, sq_angvel, dt);

            let can_sleep_now = rb.activation.is_eligible_for_sleep();

            // 2. Identify active rigid-bodies that transition from "awake" to "can_sleep"
            //    and push the sleep root candidate if applicable.
            if !can_sleep_before && can_sleep_now {
                // This is a new candidate for island extraction.
                if !rb.activation.is_sleep_root_candidate {
                    self.traversal_candidates.push_back(*handle);
                }
            }
        }
        // println!("Update energy: {}", t0.elapsed().as_secs_f32() * 1000.0);

        // 3. Perform sleeping islands extraction on **all** the prioritized sleep candidates.
        // let t0 = std::time::Instant::now();
        let frame_base_timestamp = self.timestamp;
        let mut niters = 0;
        for sleep_root in std::mem::take(&mut self.priority_traversal_candidates) {
            niters += self.extract_sleeping_island(
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                narrow_phase,
                sleep_root,
                frame_base_timestamp,
            );
            self.timestamp += 1;
        }

        // 3. Perform one, or multiple, sleeping islands extraction (graph traversal).
        //    Limit the traversal cost by not traversing all the known sleeping roots if
        //    there are too many.
        while let Some(sleep_root) = self.traversal_candidates.pop_front() {
            if niters > 1000 {
                break; // Limit cost per frame.
            }

            niters += self.extract_sleeping_island(
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                narrow_phase,
                sleep_root,
                frame_base_timestamp,
            );

            // TODO PERF: early-break if we consider we have done enough island extraction work.
            self.timestamp += 1;
        }
        // println!("Island extraction: {}", t0.elapsed().as_secs_f32() * 1000.0);
    }

    /// Returns the number of iterations run by the graph traversal so we can balance load across
    /// frames.
    fn extract_sleeping_island(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        narrow_phase: &NarrowPhase,
        sleep_root: RigidBodyHandle,
        frame_base_timestamp: u32,
    ) -> usize {
        let Some(rb) = bodies.get_mut_internal(sleep_root) else {
            // This branch happens if the rigid-body no longer exists.
            return 0;
        };

        rb.activation.is_sleep_root_candidate = false;
        if rb.ids.active_island_id != 0 || rb.ids.active_set_timestamp == self.timestamp {
            return 0; // This rigid-body is already in a sleeping island or was already traversed.
        }

        // TODO: implement recycling islands to avoid repeated allocations?
        let mut new_island = Island::default();
        self.stack.push(sleep_root);

        let mut can_sleep = true;
        let mut niter = 0;
        while let Some(handle) = self.stack.pop() {
            let rb = bodies.index_mut_internal(handle);

            if rb.is_fixed() {
                // Don’t propagate islands through fixed bodies.
                continue;
            }

            if rb.ids.active_set_timestamp == self.timestamp {
                // We already visited this body and its neighbors.
                continue;
            }

            if rb.ids.active_set_timestamp >= frame_base_timestamp {
                // We already visited this body and its neighbors during this frame.
                // So we already know this islands cannot sleep (otherwise the bodies
                // currently being traversed would already have been marked as sleeping).
                return niter;
            }

            niter += 1;
            rb.ids.active_set_timestamp = self.timestamp;

            assert!(!rb.activation.sleeping);

            // TODO PERF: early-exit as soon as we reach a body not eligible to sleep.
            can_sleep = can_sleep && rb.activation().is_eligible_for_sleep();
            if !rb.activation.is_eligible_for_sleep() {
                // If this body cannot sleep, abort the traversal, we are not traversing
                // yet an island that can sleep.
                self.stack.clear();
                return niter;
            }

            // Traverse bodies that are interacting with the current one either through
            // contacts or a joint.
            push_contacting_bodies(&rb.colliders, colliders, narrow_phase, &mut self.stack);
            push_linked_bodies(impulse_joints, multibody_joints, handle, &mut self.stack);
            new_island.bodies.push(handle);
        }

        assert!(can_sleep);

        // If we reached this line, we completed a sleeping island traversal.
        // - Put its bodies to sleep.
        // - Remove them from the active set.
        // - Push the sleeping island.
        let new_island_id = self.free_islands.pop().unwrap_or(self.islands.len());
        let active_island = &mut self.islands[0];
        for (id, handle) in new_island.bodies.iter().enumerate() {
            let rb = bodies.index_mut_internal(*handle);
            rb.sleep();

            let id_to_remove = rb.ids.active_set_id;
            rb.ids.active_island_id = new_island_id;
            rb.ids.active_set_id = id;
            rb.ids.active_set_offset = id as u32;

            active_island.bodies.swap_remove(id_to_remove);
            if let Some(moved_handle) = active_island.bodies.get(id_to_remove).copied() {
                let moved_rb = bodies.index_mut_internal(moved_handle);
                moved_rb.ids.active_set_id = id_to_remove;
                moved_rb.ids.active_set_offset = id_to_remove as u32;
            }
            niter += 1; // This contributes to the cost of the operation.
        }

        self.islands.insert(new_island_id, new_island);
        niter
    }
}

// Read all the contacts and push objects touching this rigid-body.
#[inline]
fn push_contacting_bodies(
    rb_colliders: &RigidBodyColliders,
    colliders: &ColliderSet,
    narrow_phase: &NarrowPhase,
    stack: &mut Vec<RigidBodyHandle>,
) {
    for collider_handle in &rb_colliders.0 {
        for inter in narrow_phase.contact_pairs_with(*collider_handle) {
            for manifold in &inter.manifolds {
                if !manifold.data.solver_contacts.is_empty() {
                    let other = crate::utils::select_other(
                        (inter.collider1, inter.collider2),
                        *collider_handle,
                    );
                    if let Some(other_body) = colliders[other].parent {
                        stack.push(other_body.handle);
                    }
                    break;
                }
            }
        }
    }
}

fn push_linked_bodies(
    impulse_joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    handle: RigidBodyHandle,
    stack: &mut Vec<RigidBodyHandle>,
) {
    for inter in impulse_joints.attached_enabled_joints(handle) {
        let other = crate::utils::select_other((inter.0, inter.1), handle);
        stack.push(other);
    }

    for other in multibody_joints.bodies_attached_with_enabled_joint(handle) {
        stack.push(other);
    }
}
