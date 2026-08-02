use crate::alloc_prelude::*;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};

use super::IslandManager;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
pub(crate) struct Island {
    /// The rigid-bodies part of this island.
    ///
    /// At most one island is awake — the one `IslandManager::awake_island` points
    /// to (all awake bodies share it); every other island is a sleeping chunk.
    pub(super) bodies: Vec<RigidBodyHandle>,
}

impl Island {
    pub fn singleton(handle: RigidBodyHandle) -> Self {
        Self {
            bodies: vec![handle],
        }
    }

    pub fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }
}

impl IslandManager {
    /// Remove from the island at `source_id` all the rigid-bodies that are in
    /// `new_island`, put them to sleep, and insert `new_island` into the islands
    /// set as a sleeping chunk.
    ///
    /// **All** rigid-bodies from `new_island` must currently be part of the island at `source_id`.
    pub(super) fn extract_sleeping_sub_island(
        &mut self,
        bodies: &mut RigidBodySet,
        source_id: usize,
        new_island: Island,
    ) {
        self.bump_active_set_epoch();
        let new_island_id = self.free_islands.pop().unwrap_or(self.islands.len());
        let source_island = &mut self.islands[source_id];

        for (id, handle) in new_island.bodies.iter().enumerate() {
            let rb = bodies.index_mut_internal(*handle);
            rb.sleep();

            let id_to_remove = rb.ids.active_set_id as usize;

            assert_eq!(
                rb.ids.active_island_id as usize, source_id,
                "note, new id: {}",
                new_island_id
            );
            rb.ids.active_island_id = new_island_id as u32;
            rb.ids.active_set_id = id as u32;

            source_island.bodies.swap_remove(id_to_remove);

            if let Some(moved_handle) = source_island.bodies.get(id_to_remove).copied() {
                let moved_rb = bodies.index_mut_internal(moved_handle);
                moved_rb.ids.active_set_id = id_to_remove as u32;
            }
        }

        self.islands.insert(new_island_id, new_island);
    }
}
