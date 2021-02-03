use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::NarrowPhase;
use crate::utils;
use downcast_rs::__std::collections::VecDeque;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct Island {
    bodies: Vec<RigidBodyHandle>,

    // Number of bodies that are awake on this island.
    // If this is equal to zero, ten the whole island is asleep.
    wake_up_count: usize,
    // Index in the island_set.active_islands vector.
    active_island_id: usize,
    dirty: bool,
}

impl Island {
    pub fn new() -> Self {
        Self {
            bodies: vec![],
            wake_up_count: 0,
            active_island_id: crate::INVALID_USIZE,
            dirty: false,
        }
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn is_sleeping(&self) -> bool {
        self.wake_up_count == 0
    }

    pub fn active_island_id(&self) -> usize {
        self.active_island_id
    }

    pub fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies[..]
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct IslandSet {
    active_islands: Vec<usize>,
    islands: Vec<Island>,
    to_update: VecDeque<usize>,
}

impl IslandSet {
    pub fn new() -> Self {
        IslandSet {
            active_islands: vec![],
            islands: vec![],
            to_update: VecDeque::new(),
        }
    }

    pub fn islands(&self) -> &[Island] {
        &self.islands[..]
    }

    pub fn num_active_islands(&self) -> usize {
        self.active_islands.len()
    }

    pub fn active_island(&self, i: usize) -> &Island {
        &self.islands[self.active_islands[i]]
    }

    pub fn active_bodies<'a>(&'a self) -> impl Iterator<Item = RigidBodyHandle> + 'a {
        let islands = &self.islands;
        self.active_islands
            .iter()
            .copied()
            .flat_map(move |i| islands[i].bodies.iter())
            .copied()
    }

    pub fn contact_stopped(
        &mut self,
        bodies: &RigidBodySet,
        h1: RigidBodyHandle,
        _h2: RigidBodyHandle,
    ) {
        // // NOTE: we don't actually need h2 because they are both on the same island anyway.
        // // Yet we keep the `h2` argument to show that this function properly take into account
        // // both bodies.
        // if let Some(island_id) = bodies.get(h1).map(|b| b.island_id) {
        //     if let Some(island) = self.islands.get_mut(island_id) {
        //         if !island.dirty {
        //             island.dirty = true;
        //             self.to_update.push_back(island_id);
        //         }
        //     }
        // }
    }

    pub fn incremental_update(&mut self, narrow_phase: &NarrowPhase) {
        // if let Some(island) = self.to_update.pop_front() {
        //     // Next island to update.
        // }
    }

    pub fn is_island_sleeping(&self, island_id: usize) -> bool {
        self.islands[island_id].is_sleeping()
    }

    pub fn add_rigid_body(&mut self, handle: RigidBodyHandle, body: &mut RigidBody) {
        assert_eq!(body.island_id, crate::INVALID_USIZE);

        if !body.is_dynamic() {
            return;
        }

        let new_island_id = handle.into_raw_parts().0;

        if self.islands.len() <= new_island_id {
            self.islands.resize(new_island_id + 1, Island::new());
        }

        body.island_id = new_island_id;
        body.island_offset = self.islands[new_island_id].bodies.len();
        self.islands[new_island_id].bodies.push(handle);

        // NOTE: `body_sleep_state_changed` must be called afterwards.
    }

    pub fn body_sleep_state_changed(&mut self, body: &RigidBody) {
        // Non-dynamic bodies never take part in the island computation
        // since they don't transmit any forces.
        if !body.is_dynamic() {
            return;
        }

        let island = &mut self.islands[body.island_id];

        if body.can_sleep() {
            island.wake_up_count -= 1;

            if island.wake_up_count == 0 {
                // The island is sleeping now.
                // Remove it from the active set.
                let active_island_id_to_remove = island.active_island_id;
                island.active_island_id = crate::INVALID_USIZE;
                self.active_islands.swap_remove(active_island_id_to_remove);

                if let Some(moved_island) = self.active_islands.get(active_island_id_to_remove) {
                    self.islands[*moved_island].active_island_id = active_island_id_to_remove;
                }
            }
        } else {
            if island.wake_up_count == 0 {
                // The islands is waking up.
                // Add it to the active set.
                assert_eq!(island.active_island_id, crate::INVALID_USIZE);
                island.active_island_id = self.active_islands.len();
                self.active_islands.push(body.island_id);
            }

            island.wake_up_count += 1;
        }

        if self.active_islands.len() == 0 {
            dbg!("Hurray!");
        }
    }

    pub fn contact_started(
        &mut self,
        bodies: &mut RigidBodySet,
        mut h1: RigidBodyHandle,
        mut h2: RigidBodyHandle,
    ) {
        let body1 = &bodies[h1];
        let body2 = &bodies[h2];

        if !body1.is_dynamic() || !body2.is_dynamic() {
            // Non-dynamic bodies don't transmit forces.
            // So we can ignore their contact for island computation.
            return;
        }

        assert_ne!(body1.island_id, crate::INVALID_USIZE);
        assert_ne!(body2.island_id, crate::INVALID_USIZE);

        let mut island_id1 = body1.island_id;
        let mut island_id2 = body2.island_id;

        if island_id1 != island_id2 {
            let (mut island1, mut island2) =
                utils::get2_mut(&mut self.islands, island_id1, island_id2);

            // Make sure island1 points to the biggest island.
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

            // Update the wake-up count and determine if the island is awake.
            // Note that if both `wake_up_count` are zero, then the island
            // won't end up in the active island set (which is what we want).
            let island2_was_sleeping = island2.is_sleeping();

            if island1.is_sleeping() && !island2_was_sleeping {
                // Add the first island to the active island set.
                assert_eq!(island1.active_island_id, crate::INVALID_USIZE);
                island1.active_island_id = self.active_islands.len();
                self.active_islands.push(island_id1);
            }

            island1.wake_up_count += island2.wake_up_count;
            island2.wake_up_count = 0;

            assert!(island1.wake_up_count != 0 || island1.active_island_id == crate::INVALID_USIZE);

            if !island2_was_sleeping {
                // The second island will be emptied, so we
                // can remove it from the set of active islands.
                let active_island_id_to_remove = island2.active_island_id;
                island2.active_island_id = crate::INVALID_USIZE;
                self.active_islands.swap_remove(active_island_id_to_remove);

                if let Some(moved_island) = self.active_islands.get(active_island_id_to_remove) {
                    self.islands[*moved_island].active_island_id = active_island_id_to_remove;
                }
            } else {
                assert_eq!(island2.active_island_id, crate::INVALID_USIZE);
            }
        }
    }
}
