use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::NarrowPhase;
use crate::utils;
use downcast_rs::__std::collections::VecDeque;

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

    pub fn is_sleeping(&self) -> bool {
        self.wake_up_count == 0
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct IslandSet2 {
    inactive_islands: Vec<Island>,
    active_island: Island,
}

impl IslandSet2 {
    pub fn new() -> Self {
        IslandSet2 {
            inactive_islands: vec![],
            active_island: Island::new(),
        }
    }

    pub fn num_active_islands(&self) -> usize {
        1
    }

    pub fn active_island(&self, i: usize) -> &Island {
        &self.active_island
    }

    pub fn active_bodies<'a>(&'a self) -> impl Iterator<Item = RigidBodyHandle> + 'a {
        self.active_island.bodies.iter()
    }

    pub fn contact_stopped(
        &mut self,
        bodies: &RigidBodySet,
        h1: RigidBodyHandle,
        _h2: RigidBodyHandle,
    ) {
    }

    pub fn incremental_update(&mut self, narrow_phase: &NarrowPhase) {}

    pub fn is_island_sleeping(&self, island_id: usize) -> bool {
        island_id != crate::INVALID_USIZE
    }

    pub fn add_rigid_body(&mut self, handle: RigidBodyHandle, body: &mut RigidBody) {
        assert_eq!(body.island_id, crate::INVALID_USIZE);

        if !body.is_dynamic() {
            return;
        }

        if body.can_sleep() {
            let new_island_id = handle.into_raw_parts().0;

            if self.islands.len() <= new_island_id {
                self.islands.resize(new_island_id + 1, Island::new());
            }

            body.island_id = new_island_id;
            body.island_offset = self.islands[new_island_id].bodies.len();
            self.islands[new_island_id].bodies.push(handle);
        } else {
            body.island_offset = self.active_island.len();
            self.active_island.push(handle);
        }
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
    }

    pub fn contact_started(
        &mut self,
        bodies: &mut RigidBodySet,
        mut h1: RigidBodyHandle,
        mut h2: RigidBodyHandle,
    ) {
    }
}
