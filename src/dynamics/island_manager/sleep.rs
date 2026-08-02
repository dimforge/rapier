use crate::alloc_prelude::*;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::NarrowPhase;

use super::{Island, IslandManager};

impl IslandManager {
    /// Wakes up a sleeping body, forcing it back into the active simulation.
    ///
    /// Waking any body of a sleeping island wakes the **whole island** (its
    /// entire touching-contact/joint connected component) and resets every
    /// member's sleep timer.
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
    /// body.add_force(Vector::new(100.0, 0.0, 0.0), false);
    /// ```
    ///
    /// Only affects dynamic bodies (kinematic and fixed bodies don't sleep).
    pub fn wake_up(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle, strong: bool) {
        // NOTE: the use an Option here because there are many legitimate cases (like when
        //       deleting a joint attached to an already-removed body) where we could be
        //       attempting to wake-up a rigid-body that has already been deleted.
        if bodies.get(handle).map(|rb| !rb.is_fixed()) == Some(true) {
            let rb = bodies.index_mut_internal(handle);

            rb.activation.wake_up(strong);
            let persistent_id = rb.ids.island_id;
            let island_to_wake_up = rb.ids.active_island_id;

            // Whole-island wake: waking any body wakes the entire persistent island, with
            // a *strong* timer reset for every member — `RigidBody::sleep` leaves timers at the
            // eligibility threshold, so a freshly woken island would otherwise re-sleep next step.
            if persistent_id != crate::dynamics::INVALID_ISLAND {
                let sleeping_island = self
                    .persistent
                    .islands
                    .get(persistent_id as usize)
                    .is_some_and(|island| island.sleeping);
                if sleeping_island {
                    let island = &mut self.persistent.islands[persistent_id as usize];
                    island.sleeping = false;
                    // The island's bodies normally share one sleeping-chunk
                    // container, but joint-merged sleeping islands can span
                    // several: wake each body's chunk.
                    let handles = island.bodies.clone();
                    for h in &handles {
                        if let Some(rb) = bodies.get_mut(*h) {
                            rb.activation.wake_up(true);
                        }
                    }
                    for h in handles {
                        let chunk = match bodies.get(h) {
                            Some(rb) => rb.ids.active_island_id,
                            None => continue,
                        };
                        self.wake_up_island(bodies, chunk as usize);
                    }
                    return;
                }
            }

            self.wake_up_island(bodies, island_to_wake_up as usize);
        }
    }

    /// Puts `chunks` (disjoint subsets of the awake island's bodies, all
    /// sleep-eligible) to sleep: in place if they cover the entire awake
    /// island, otherwise by extracting each chunk into a new sleeping island.
    pub(super) fn commit_sleeping_chunks(
        &mut self,
        bodies: &mut RigidBodySet,
        narrow_phase: &mut NarrowPhase,
        active_island_id: usize,
        active_island_len: usize,
        mut chunks: Vec<Vec<RigidBodyHandle>>,
    ) {
        if chunks.len() == 1 && chunks[0].len() == active_island_len {
            // The whole island is asleep. No need to insert a new one.
            // Put all its bodies to sleep.
            let active_island = &mut self.islands[active_island_id];
            for handle in &active_island.bodies {
                bodies.index_mut_internal(*handle).sleep();
            }

            for handle in &active_island.bodies {
                let rb = &bodies[*handle];
                for co_handle in rb.colliders.0.iter().copied() {
                    narrow_phase.clear_asleep_pair_solver_hint_counts_of(co_handle);
                }
            }

            // Membership changed (the whole island leaves the active set): bump the epoch so
            // epoch-keyed caches can't go stale. The hint count-clears above only cover bodies
            // WITH colliders — collider-less (joint-only) bodies would otherwise sleep without invalidating e.g. the cached body qualification table.
            self.active_set_epoch = self.active_set_epoch.wrapping_add(1);

            // Mark the island as sleeping: no island is awake anymore.
            debug_assert_eq!(self.awake_island, Some(active_island_id));
            self.awake_island = None;
        } else {
            let slept: Vec<RigidBodyHandle> = chunks.iter().flatten().copied().collect();

            for chunk in &mut chunks {
                let new_island = Island {
                    bodies: core::mem::take(chunk),
                };
                self.extract_sleeping_sub_island(bodies, active_island_id, new_island);
            }

            // Clear hints after the extractions (which flag the bodies as
            // sleeping).
            for handle in &slept {
                let rb = &bodies[*handle];
                for co_handle in rb.colliders.0.iter().copied() {
                    narrow_phase.clear_asleep_pair_solver_hint_counts_of(co_handle);
                }
            }
        }
    }

    pub(super) fn wake_up_island(&mut self, bodies: &mut RigidBodySet, island_id: usize) {
        if self.awake_island == Some(island_id) {
            // Already awake.
            return;
        }

        let Some(island) = self.islands.get_mut(island_id) else {
            return;
        };

        match self.awake_island {
            None => {
                // Nothing is awake: this chunk becomes the awake island. No renumbering (bodies
                // keep their `active_set_id`s), but the active-set *membership* changes, so
                // epoch-keyed caches (body qualification table, persistent solver graph, solver constraint caches) must not survive — bump the epoch like the merge branch. (Direct field bump: `island` still borrows `self.islands`.)
                self.active_set_epoch = self.active_set_epoch.wrapping_add(1);
                self.awake_island = Some(island_id);

                for handle in &island.bodies {
                    if let Some(rb) = bodies.get_mut(*handle) {
                        rb.wake_up(false);
                    }
                }
            }
            Some(awake_id) => {
                // Merge the chunk's bodies into the single awake island.
                self.bump_active_set_epoch();
                let Some(removed) = self.islands.remove(island_id) else {
                    unreachable!()
                };
                self.free_islands.push(island_id);

                let target = &mut self.islands[awake_id];
                for handle in &removed.bodies {
                    let Some(rb) = bodies.get_mut(*handle) else {
                        // This body no longer exists.
                        continue;
                    };
                    rb.wake_up(false);
                    rb.ids.active_island_id = awake_id as u32;
                    rb.ids.active_set_id = (target.bodies.len()) as u32;
                    target.bodies.push(*handle);
                }
            }
        }
    }
}
