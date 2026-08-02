//! Tier-2 global island split: the deferred O(island) union-find fallback that
//! re-derives a [`PersistentIslands`] island's connected components when the
//! bounded local search ([`super::local_split`]) could not settle a removal.

use crate::alloc_prelude::*;
use crate::data::union_find::UnionFind;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};

use super::persistent::{INVALID_ISLAND, INVALID_LOC, PersistentIslands, SPLIT_RETRY_COOLDOWN};

#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(super) struct SplitScratch {
    uf: UnionFind,
    contact_counts: Vec<u32>,
    joint_counts: Vec<u32>,
    root_island: Vec<u32>,
    /// Per-link node id of the endpoint that belongs to the island being split
    /// (`INVALID_ISLAND` if neither does), memoized by the union pass so the
    /// move pass doesn't have to touch the body arena again.
    contact_nodes: Vec<u32>,
    joint_nodes: Vec<u32>,
    /// `(stamp, generation, union-find node)` per body-arena index: lets the union pass resolve
    /// link endpoints from this ~12 B/body table instead of chasing `RigidBody`s at random through
    /// the arena — that random walk *was* the whole cost of the split. `stamp` (bumped per split) invalidates old entries in O(1); `generation` rejects stale links whose dead body's slot was recycled.
    node_map: Vec<(u32, u32, u32)>,
    node_map_stamp: u32,
}

impl PersistentIslands {
    /// Schedules `island_id` (a body of that island wants to sleep but the island
    /// has pending removals) for next step's (single) pending split.
    #[inline]
    pub fn schedule_split(&mut self, island_id: u32) {
        self.split_island = Some(island_id);
    }

    /// Runs the pending split, if any (at most one island per step). Sleeping
    /// islands are skipped: splits only run on awake islands (their old-style
    /// sleeping-chunk container wouldn't follow the split).
    pub fn run_pending_split(&mut self, bodies: &mut RigidBodySet) {
        if let Some(id) = self.split_island.take() {
            if self
                .islands
                .get(id as usize)
                .is_some_and(|island| !island.sleeping)
            {
                self.split_island_now(bodies, id);
            }
        }
    }

    /// Drops the pending split if it targets `island_id` (called when that
    /// island falls asleep).
    #[inline]
    pub fn clear_pending_split_of(&mut self, island_id: u32) {
        if self.split_island == Some(island_id) {
            self.split_island = None;
        }
    }

    /// Splits `island_id` into connected components (union-find over cached links);
    /// if still one component, only resets `constraint_remove_count`. The split is *in place*:
    /// the largest component keeps `island_id`, so the move pass is proportional to what
    /// leaves instead of re-creating every component.
    pub fn split_island_now(&mut self, bodies: &mut RigidBodySet, island_id: u32) {
        let island = &self.islands[island_id as usize];
        let body_count = island.bodies.len();
        if body_count <= 1 {
            let island = &mut self.islands[island_id as usize];
            island.constraint_remove_count = 0;
            island.split_denied_until = self.sleep_scan_stamp + SPLIT_RETRY_COOLDOWN;
            return;
        }

        let mut scratch = core::mem::take(&mut self.split_scratch);
        scratch.uf.reset(body_count);
        scratch.contact_counts.clear();
        scratch.contact_counts.resize(body_count, 0);
        scratch.joint_counts.clear();
        scratch.joint_counts.resize(body_count, 0);

        // A body's union-find node id is its `island_index`. Rather than read it back from the
        // body arena (random, cache-missing access per link endpoint), index the island's members
        // by arena index up front — handles carry everything needed, `island.bodies` is all live.
        scratch.node_map_stamp = scratch.node_map_stamp.wrapping_add(1);
        if scratch.node_map_stamp == 0 {
            // Wrapped: a zeroed (never-written) entry would alias stamp 0.
            scratch.node_map.clear();
            scratch.node_map_stamp = 1;
        }
        let stamp = scratch.node_map_stamp;
        for (node, handle) in self.islands[island_id as usize].bodies.iter().enumerate() {
            let (index, generation) = handle.into_raw_parts();
            if scratch.node_map.len() <= index as usize {
                scratch.node_map.resize(index as usize + 1, (0, 0, 0));
            }
            scratch.node_map[index as usize] = (stamp, generation, node as u32);
        }

        // An endpoint that is not a member of this island (fixed body, removed
        // body, or a body already moved out) doesn't connect.
        let node_of = |node_map: &[(u32, u32, u32)], h: RigidBodyHandle| -> Option<u32> {
            let (index, generation) = h.into_raw_parts();
            let entry = node_map.get(index as usize)?;
            (entry.0 == stamp && entry.1 == generation).then_some(entry.2)
        };

        // The union pass memoizes each link's member-side node id so the move pass decides where
        // a link goes from island-owned memory alone; `INVALID_ISLAND` marks a link whose two
        // endpoints both left the island (dropped below).
        let island = &self.islands[island_id as usize];
        let SplitScratch {
            uf,
            contact_counts,
            joint_counts,
            contact_nodes,
            joint_nodes,
            node_map,
            ..
        } = &mut scratch;
        contact_nodes.clear();
        contact_nodes.reserve(island.contact_links.len());
        joint_nodes.clear();
        joint_nodes.reserve(island.joint_links.len());

        for link in &island.contact_links {
            let n1 = node_of(node_map, link.body1);
            let n2 = node_of(node_map, link.body2);
            match (n1, n2) {
                (Some(a), Some(b)) => {
                    uf.union(a, b);
                    contact_counts[uf.find(a) as usize] += 1;
                }
                (Some(a), None) => contact_counts[uf.find(a) as usize] += 1,
                (None, Some(b)) => contact_counts[uf.find(b) as usize] += 1,
                (None, None) => {}
            }
            contact_nodes.push(n1.or(n2).unwrap_or(INVALID_ISLAND));
        }
        for link in &island.joint_links {
            let n1 = node_of(node_map, link.body1);
            let n2 = node_of(node_map, link.body2);
            match (n1, n2) {
                (Some(a), Some(b)) => {
                    uf.union(a, b);
                    joint_counts[uf.find(a) as usize] += 1;
                }
                (Some(a), None) => joint_counts[uf.find(a) as usize] += 1,
                (None, Some(b)) => joint_counts[uf.find(b) as usize] += 1,
                (None, None) => {}
            }
            joint_nodes.push(n1.or(n2).unwrap_or(INVALID_ISLAND));
        }

        // Flatten so the move passes below can resolve roots with immutable
        // single reads, then pick the biggest component (by the union-find's
        // set sizes, i.e. body counts): it keeps the base island.
        scratch.uf.flatten();
        let mut component_count = 0usize;
        let mut keep_root = 0u32;
        let mut keep_size = 0u32;
        for i in 0..body_count as u32 {
            if scratch.uf.root(i) == i {
                component_count += 1;
                if scratch.uf.size(i) > keep_size {
                    keep_size = scratch.uf.size(i);
                    keep_root = i;
                }
            }
        }
        if component_count <= 1 {
            let island = &mut self.islands[island_id as usize];
            island.constraint_remove_count = 0;
            island.split_denied_until = self.sleep_scan_stamp + SPLIT_RETRY_COOLDOWN;
            self.split_scratch = scratch;
            return;
        }

        // NOTE: the per-root contact/joint counts of the union pass were
        // accumulated on the root at the time of the union, which may not be
        // the final root. Fold them up onto final roots instead of trusting
        // them as-is.
        for i in 0..body_count as u32 {
            let root = scratch.uf.root(i);
            if root != i {
                scratch.contact_counts[root as usize] += scratch.contact_counts[i as usize];
                scratch.joint_counts[root as usize] += scratch.joint_counts[i as usize];
                scratch.contact_counts[i as usize] = 0;
                scratch.joint_counts[i as usize] = 0;
            }
        }

        // One new island per component, except `keep_root`'s, which stays in
        // the base island (and keeps its sleeping/cooldown state).
        let base_sleeping = self.islands[island_id as usize].sleeping;
        scratch.root_island.clear();
        scratch.root_island.resize(body_count, INVALID_ISLAND);
        scratch.root_island[keep_root as usize] = island_id;

        for i in 0..body_count {
            let root = scratch.uf.root(i as u32) as usize;
            if scratch.root_island[root] == INVALID_ISLAND {
                let new_id = self.alloc_island();
                let island = &mut self.islands[new_id as usize];
                island.sleeping = base_sleeping;
                island.bodies.reserve(scratch.uf.size(root as u32) as usize);
                island
                    .contact_links
                    .reserve(scratch.contact_counts[root] as usize);
                island
                    .joint_links
                    .reserve(scratch.joint_counts[root] as usize);
                scratch.root_island[root] = new_id;
            }
        }

        // Move the links out FIRST, while the bodies' `island_index` (the union-find node ids)
        // still describe the base island. Descending index order, so each `swap_remove` only ever
        // pulls in an element that stays: everything past the current index was already visited.
        let mut links = core::mem::take(&mut self.islands[island_id as usize].contact_links);
        for i in (0..links.len()).rev() {
            let node = scratch.contact_nodes[i];
            let target = if node == INVALID_ISLAND {
                // Both endpoints left (dead/fixed/disabled): drop the link.
                INVALID_ISLAND
            } else {
                scratch.root_island[scratch.uf.root(node) as usize]
            };
            if target == island_id {
                continue;
            }

            let link = links.swap_remove(i);
            if let Some(moved) = links.get(i) {
                self.contact_link_locs[moved.edge_id as usize] = (island_id, i as u32);
            }

            if target == INVALID_ISLAND {
                self.contact_link_locs[link.edge_id as usize] = INVALID_LOC;
            } else {
                let island = &mut self.islands[target as usize];
                self.contact_link_locs[link.edge_id as usize] =
                    (target, island.contact_links.len() as u32);
                island.contact_links.push(link);
            }
        }
        self.islands[island_id as usize].contact_links = links;

        let mut links = core::mem::take(&mut self.islands[island_id as usize].joint_links);
        for i in (0..links.len()).rev() {
            let node = scratch.joint_nodes[i];
            let target = if node == INVALID_ISLAND {
                INVALID_ISLAND
            } else {
                scratch.root_island[scratch.uf.root(node) as usize]
            };
            if target == island_id {
                continue;
            }

            let link = links.swap_remove(i);
            if let Some(moved) = links.get(i) {
                self.joint_link_locs
                    .insert(moved.key, (island_id, i as u32));
            }

            if target == INVALID_ISLAND {
                crate::utils::hashmap_remove(&mut self.joint_link_locs, &link.key);
            } else {
                let island = &mut self.islands[target as usize];
                self.joint_link_locs
                    .insert(link.key, (target, island.joint_links.len() as u32));
                island.joint_links.push(link);
            }
        }
        self.islands[island_id as usize].joint_links = links;

        // Then move the bodies, descending again: `swap_remove(i)` pulls in a body that stays,
        // whose `island_index` becomes `i` — node ids of not-yet-visited bodies (indices < i) are
        // untouched, so the union-find roots stay valid for the rest of the loop.
        let mut island_bodies = core::mem::take(&mut self.islands[island_id as usize].bodies);
        for i in (0..island_bodies.len()).rev() {
            let target = scratch.root_island[scratch.uf.root(i as u32) as usize];
            if target == island_id {
                continue;
            }

            let handle = island_bodies.swap_remove(i);
            if let Some(moved) = island_bodies.get(i) {
                bodies.index_mut_internal(*moved).ids.island_index = i as u32;
            }

            let island = &mut self.islands[target as usize];
            if let Some(rb) = bodies.get_mut_internal(handle) {
                rb.ids.island_id = target;
                rb.ids.island_index = island.bodies.len() as u32;
            }
            island.bodies.push(handle);
        }
        self.islands[island_id as usize].bodies = island_bodies;

        let island = &mut self.islands[island_id as usize];
        island.constraint_remove_count = 0;
        island.split_denied_until = self.sleep_scan_stamp + SPLIT_RETRY_COOLDOWN;
        self.split_scratch = scratch;
    }
}
