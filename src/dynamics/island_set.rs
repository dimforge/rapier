use crate::data::Coarena;
use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::utils;
use std::collections::VecDeque;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct Island {
    bodies: Vec<RigidBodyHandle>,
}

impl Island {
    pub fn new() -> Self {
        Self { bodies: vec![] }
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct IslandSet {
    // We can't store this in the rigid-bodies because that would
    // cause borrowing issues during the traversal.
    traversal_body_timestamps: Coarena<u64>,
    islands: Vec<Island>,
    active_island: usize,
    islands_to_extract: VecDeque<RigidBodyHandle>,
    islands_extraction_queue: Vec<RigidBodyHandle>,
    // NOTE: this value must never be 0, otherwise some
    // invariants used during the traversal can break.
    traversal_timestamp: u64,
}

impl IslandSet {
    pub fn new() -> Self {
        IslandSet {
            traversal_body_timestamps: Coarena::new(),
            islands: vec![Island::new()],
            active_island: 0,
            islands_to_extract: VecDeque::new(),
            islands_extraction_queue: vec![],
            traversal_timestamp: 1,
        }
    }

    pub fn num_active_islands(&self) -> usize {
        1
    }

    pub fn is_island_sleeping(&self, island_id: usize) -> bool {
        island_id != self.active_island
    }

    pub fn active_bodies(&self) -> &[RigidBodyHandle] {
        &self.islands[self.active_island].bodies
    }

    pub fn incremental_update(&mut self, narrow_phase: &NarrowPhase) {}

    pub fn add_rigid_body(&mut self, handle: RigidBodyHandle, body: &mut RigidBody) {
        assert_eq!(body.island_id, crate::INVALID_USIZE);

        if !body.is_dynamic() {
            return;
        }

        self.traversal_body_timestamps
            .ensure_element_exists(handle.0, 0);

        if body.can_sleep() {
            body.island_id = self.islands.len();
            body.island_offset = 0;
            self.islands.push(Island {
                bodies: vec![handle],
            })
        } else {
            let mut active_island = &mut self.islands[self.active_island];
            body.island_id = self.active_island;
            body.island_offset = active_island.bodies.len();
            active_island.bodies.push(handle);
        }
    }

    pub fn contact_started(
        &mut self,
        bodies: &mut RigidBodySet,
        mut h1: RigidBodyHandle,
        mut h2: RigidBodyHandle,
    ) {
        let island1 = bodies[h1].island_id;
        let island2 = bodies[h2].island_id;

        self.wake_up_island(bodies, island1);
        self.wake_up_island(bodies, island2);
    }

    pub fn body_sleep_state_changed(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle) {
        let body = &mut bodies[handle];
        let island_id = body.island_id;

        if island_id == crate::INVALID_USIZE {
            self.add_rigid_body(handle, body);
        } else if island_id == self.active_island && body.can_sleep() {
            // The body is sleeping now.
            self.islands_to_extract.push_back(handle);
        } else if island_id != self.active_island && !body.can_sleep() {
            // The body is awake now.
            self.wake_up_island(bodies, island_id);
        }
    }

    #[inline(always)]
    fn push_contacting_bodies(
        handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        queue: &mut Vec<RigidBodyHandle>,
        traversal_body_timestamps: &mut Coarena<u64>,
        first_traversal_timestamp: u64,
        traversal_timestamp: u64,
    ) -> bool {
        for collider_handle in &bodies[handle].colliders {
            if let Some(contacts) = narrow_phase.contacts_with(*collider_handle) {
                for inter in contacts {
                    let other = crate::utils::select_other((inter.0, inter.1), *collider_handle);
                    let other_body_handle = colliders[other].parent;
                    let other_body = &bodies[other_body_handle];

                    if other_body.is_dynamic() {
                        if !other_body.can_sleep() {
                            return false;
                        }

                        let other_body_timestamp =
                            &mut traversal_body_timestamps[other_body_handle.0];

                        if *other_body_timestamp >= first_traversal_timestamp
                            && *other_body_timestamp < traversal_timestamp
                        {
                            // We already saw this body during another traversal this frame.
                            // So we don't need to continue any further.
                            return false;
                        }

                        if *other_body_timestamp != traversal_timestamp {
                            *other_body_timestamp = traversal_timestamp;
                            queue.push(other_body_handle);
                        }
                    }
                }
            }
        }

        true
    }

    pub fn update_sleeping_islands(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
    ) {
        let first_traversal_timestamp = self.traversal_timestamp + 1;
        let mut island_found = false;

        while let Some(handle) = self.islands_to_extract.pop_front() {
            self.traversal_timestamp += 1;
            island_found = self.extract_sleeping_island(
                bodies,
                colliders,
                narrow_phase,
                handle,
                first_traversal_timestamp,
            ) || island_found;

            // Make sure our `traversal_timestamp` remains correct
            // even if it overflows (can happen in very long-running
            // simulations).
            if self.traversal_timestamp == u64::MAX - 1 {
                // Stop the work now because the overflowing timestamp
                // will cause issues with the traversal. Forcibly reset all
                // the timestamps and continue next frame.
                for body in bodies.iter_mut_internal() {
                    body.1.traversal_timestamp = 0;
                }
                self.traversal_timestamp = 1;
                break;
            }
        }

        if island_found {
            // Now we need to remove from the active set all the bodies
            // we moved to a sleeping island.
            let mut i = 0;
            let mut new_len = 0;
            let active_island = &mut self.islands[self.active_island];

            while i < active_island.bodies.len() {
                let handle = active_island.bodies[i];
                let body = &mut bodies[handle];

                if body.island_id == self.active_island {
                    // This body still belongs to this island.
                    // Update its offset.
                    body.island_offset = new_len;
                    active_island.bodies[new_len] = handle;
                    new_len += 1;
                }

                i += 1;
            }

            active_island.bodies.truncate(new_len);
        }
    }

    fn extract_sleeping_island(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
        handle: RigidBodyHandle,
        first_traversal_timestamp: u64,
    ) -> bool {
        // Attempt to extract a sleeping island by traversing the
        // contact graph starting with the given rigid-body.
        if let Some(body) = bodies.get(handle) {
            if body.is_dynamic() && body.can_sleep() {
                // Perform a breadth-first search of the contact graph starting
                // with this body. We stop the search when an active body is
                // found (meaning that the island cannot sleep), or if we
                // extracted the whole island.
                // We do a breadth-first search in order to increase our chances
                // to find a non-sleeping body quickly: if this body just started
                // sleeping, chances are that a non-sleeping body is close.
                self.islands_extraction_queue.clear();
                self.islands_extraction_queue.push(handle);
                self.traversal_body_timestamps[handle.0] = self.traversal_timestamp;
                let mut i = 0;

                while i < self.islands_extraction_queue.len() {
                    let handle = self.islands_extraction_queue[i];

                    let continue_traversal = Self::push_contacting_bodies(
                        handle,
                        bodies,
                        colliders,
                        narrow_phase,
                        &mut self.islands_extraction_queue,
                        &mut self.traversal_body_timestamps,
                        first_traversal_timestamp,
                        self.traversal_timestamp,
                    );

                    if !continue_traversal {
                        // This island cannot sleep yet.
                        return false;
                    }

                    i += 1;
                }

                // If we reached this point, then we successfully found a sleeping island.
                for (k, handle) in self.islands_extraction_queue.iter().enumerate() {
                    let body = &mut bodies[*handle];
                    body.island_id = self.islands.len();
                    body.island_offset = k;
                }

                // FIXME: recycle old islands.
                self.islands.push(Island {
                    bodies: std::mem::replace(&mut self.islands_extraction_queue, Vec::new()),
                });

                return true;
            }
        }

        false
    }

    pub fn wake_up_island(&mut self, bodies: &mut RigidBodySet, island_id: usize) {
        if island_id == crate::INVALID_USIZE {
            return;
        }

        let mut island_id1 = self.active_island;
        let mut island_id2 = island_id;

        if island_id1 != island_id2 {
            let (mut island1, mut island2) =
                utils::get2_mut(&mut self.islands, island_id1, island_id2);

            // Make sure island1 points to the biggest island.
            // The typical scenario is that only a few object are awaking
            // one big island. So in this typical scenario, we actually want
            // to merge the active island into the sleeping island, and then
            // mark the sleeping island as active.
            if island2.len() > island1.len() {
                std::mem::swap(&mut island1, &mut island2);
                std::mem::swap(&mut island_id1, &mut island_id2);
            }

            // Now merge island2 (the smallest island) into island1 (the bigger one).
            island1.bodies.reserve(island2.len());

            for handle in island2.bodies.drain(..) {
                let body = &mut bodies[handle];
                body.island_id = island_id1;
                body.island_offset = island1.len();
                island1.bodies.push(handle);
            }

            // Islands can get big so let's save some memory.
            island2.bodies = vec![];
            self.active_island = island_id1;
        }
    }
}
