//! Persistent islands: eager union-by-size merges, deferred splits over flat per-island link arrays that cache body handles (split never dereferences contact/joint records); fixed bodies are never members, enabled non-fixed bodies are in exactly one island.
//! Splitting is two-tiered: [`super::local_split`] settles ~all removals (99.98% on a 43k pyramid) at O(smaller piece) and *proves* harmless ones, so contact churn doesn't dirty the island or block sleep; leftovers reach the global O(island) union-find ([`PersistentIslands::split_island_now`]), cooldown-throttled ([`PersistentIsland::split_denied_until`]) and capped at one island/step — running that full scan inline every other step spiked alternating multi-ms.
//! Location back-references live here only: bodies carry `RigidBodyIds::island_id/island_index`; contact links via [`PersistentIslands::contact_link_locs`] (dense per contact-graph edge id, mirrors the edges vec's swap-removes like `pair_solver_hints`); joint links via a [`JointLinkKey`] map.

use super::global_split::SplitScratch;
use crate::alloc_prelude::*;
use crate::dynamics::{MultibodyIndex, MultibodyJointSet, RigidBodyHandle, RigidBodySet};
use parry::utils::VecMap;
use parry::utils::hashmap::HashMap;

use crate::dynamics::joint::ImpulseJointHandle;

/// Deferred island-connectivity event emitted by [`crate::dynamics::ImpulseJointSet`] on joint
/// edits, drained at the start of the next step.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) enum ImpulseJointIslandEvent {
    Link {
        handle: ImpulseJointHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    },
    Unlink {
        handle: ImpulseJointHandle,
    },
}

pub(crate) const INVALID_ISLAND: u32 = u32::MAX;

/// Steps to wait before re-splitting an island that was just split-checked.
pub(super) const SPLIT_RETRY_COOLDOWN: u32 = 16;
pub(crate) const INVALID_LOC: (u32, u32) = (u32::MAX, u32::MAX);

/// Identity of a joint-induced island link.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) enum JointLinkKey {
    /// An impulse joint (arena handle, generation included).
    Impulse(ImpulseJointHandle),
    /// The `ordinal`-th link of the internal connectivity chain of the multibody at `multibody`
    /// (packed arena index + generation). Multibodies are atomic for sleep: their non-fixed
    /// bodies are chained in link order, rebuilt whenever the multibody's structure changes.
    MultibodyChain { multibody: u64, ordinal: u32 },
}

/// Packs a multibody arena index (index + generation) into a stable key.
pub(super) fn multibody_index_key(id: MultibodyIndex) -> u64 {
    let (idx, generation) = id.0.into_raw_parts();
    ((idx as u64) << 32) | generation as u64
}

/// A touching contact edge cached inside an island. The body handles are denormalized here
/// so the split's union-find pass iterates island-owned memory only.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct ContactLink {
    pub edge_id: u32,
    pub body1: RigidBodyHandle,
    pub body2: RigidBodyHandle,
}

/// A joint edge cached inside an island.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct JointLink {
    pub key: JointLinkKey,
    pub body1: RigidBodyHandle,
    pub body2: RigidBodyHandle,
}

/// A persistent island: one connected component of the touching-contact/joint
/// graph over enabled non-fixed bodies.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct PersistentIsland {
    pub bodies: Vec<RigidBodyHandle>,
    pub contact_links: Vec<ContactLink>,
    pub joint_links: Vec<JointLink>,
    /// Removals the local search could *not* settle (plus body removals, never tried locally —
    /// a body can be a cut vertex). Non-zero marks a split candidate and blocks sleep (unless
    /// single-body); the local tier proves most removals harmless without bumping this.
    pub constraint_remove_count: u32,
    /// Sleep-scan stamp before which this island may not bid for a global split again; armed after
    /// every split check so a workload that defeats the local search can't re-run the O(links)
    /// union-find every other step. Scheduling-only: delays a real split (and the detached piece's sleep) by ≤ [`SPLIT_RETRY_COOLDOWN`] steps.
    pub split_denied_until: u32,
    /// Whether this island is asleep. Only meaningful once the whole-island
    /// sleep policy is active; connectivity maintenance ignores it.
    pub sleeping: bool,
}

/// An edge that left an island this step, pending [`PersistentIslands::resolve_removals`]. Only
/// the endpoints are recorded: resolution re-derives their islands, since an earlier removal of
/// the same batch may have moved one of them out already.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(super) struct Removal {
    pub body1: RigidBodyHandle,
    pub body2: RigidBodyHandle,
}

/// Serializes the joint-link map by key, so the bytes describe its content rather than the
/// map's insertion history (a hash map's iteration order is history- and target-dependent,
/// which would make two snapshots of the same state differ).
#[cfg(feature = "serde-serialize")]
fn serialize_joint_link_locs<S: serde::Serializer>(
    locs: &HashMap<JointLinkKey, (u32, u32)>,
    s: S,
) -> Result<S::Ok, S::Error> {
    crate::utils::serde::serialize_sorted_to_vec_tuple(
        locs,
        |k| match k {
            // Impulse joints and multibody chains never collide: they are tagged apart.
            JointLinkKey::Impulse(h) => (
                0u8,
                h.into_raw_parts().0 as u64,
                h.into_raw_parts().1 as u64,
            ),
            JointLinkKey::MultibodyChain { multibody, ordinal } => {
                (1u8, *multibody, *ordinal as u64)
            }
        },
        s,
    )
}

/// The persistent-island bookkeeping.
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct PersistentIslands {
    pub(crate) islands: VecMap<PersistentIsland>,
    free_islands: Vec<u32>,
    /// `contact_link_locs[edge_id]` = (island id, index in `contact_links`); `INVALID_LOC` when
    /// the edge isn't linked (not touching). Mirrors the contact graph's edges vec: grown on
    /// link, swap-removed through [`Self::contact_edge_removed`].
    pub(super) contact_link_locs: Vec<(u32, u32)>,
    #[cfg_attr(
        feature = "serde-serialize",
        serde(
            serialize_with = "serialize_joint_link_locs",
            deserialize_with = "crate::utils::serde::deserialize_from_vec_tuple"
        )
    )]
    pub(super) joint_link_locs: HashMap<JointLinkKey, (u32, u32)>,
    /// The edges unlinked since the last [`Self::resolve_removals`].
    pub(super) removal_journal: Vec<Removal>,
    /// Scratch for the local split search.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(super) local_split: super::local_split::LocalSplitScratch,
    /// Split candidate chosen last step (the sleepiest island that lost
    /// constraints), consumed by [`Self::run_pending_split`] this step.
    pub(super) split_island: Option<u32>,
    /// Union-find & counting scratch for the split.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(super) split_scratch: SplitScratch,
    /// Per-step sleep-scan scratch, indexed by island id: `(stamp, all_bodies_eligible_so_far)`.
    /// Stamped so the scan is O(active bodies), never O(total islands) — sleeping islands are
    /// never touched.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    sleep_scan: Vec<(u32, bool)>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    sleep_scan_touched: Vec<u32>,
    pub(super) sleep_scan_stamp: u32,
    /// Whether the structures were initialized from the current world state
    /// (they are not serialized; the first update after construction or
    /// deserialization rebuilds them from scratch).
    pub(crate) bootstrapped: bool,
}

impl PersistentIslands {
    /// The island a body belongs to, if any.
    #[inline]
    pub fn body_island(&self, bodies: &RigidBodySet, handle: RigidBodyHandle) -> Option<u32> {
        let id = bodies.get(handle)?.ids.island_id;
        (id != INVALID_ISLAND).then_some(id)
    }

    /// Whether `island_id` may currently bid for the per-step island split:
    /// it must have pending constraint removals and not be in the retry
    /// cooldown of a recent still-connected split attempt.
    #[inline]
    pub fn split_allowed(&self, island_id: u32) -> bool {
        self.islands.get(island_id as usize).is_some_and(|island| {
            island.constraint_remove_count > 0 && self.sleep_scan_stamp >= island.split_denied_until
        })
    }

    /// The `(island id, link index)` of the contact link for `edge_id`, if
    /// that edge is linked. Only the structural validation reads it.
    #[cfg(debug_assertions)]
    #[inline]
    pub fn contact_link_loc(&self, edge_id: u32) -> Option<(u32, u32)> {
        self.contact_link_locs
            .get(edge_id as usize)
            .copied()
            .filter(|loc| *loc != INVALID_LOC)
    }

    pub(super) fn alloc_island(&mut self) -> u32 {
        let id = self
            .free_islands
            .pop()
            .unwrap_or_else(|| self.islands.len() as u32);
        self.islands
            .insert(id as usize, PersistentIsland::default());
        id
    }

    fn free_island(&mut self, id: u32) {
        let island = self.islands.remove(id as usize);
        debug_assert!(island.is_some_and(|i| i.bodies.is_empty()));
        if self.split_island == Some(id) {
            self.split_island = None;
        }
        self.free_islands.push(id);
    }

    /// Ensures `handle` (enabled, non-fixed) is an island member; creates a
    /// singleton island for it if it has none.
    pub fn ensure_body(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle) {
        let Some(rb) = bodies.get_mut_internal(handle) else {
            return;
        };
        if rb.is_fixed() || !rb.is_enabled() || rb.ids.island_id != INVALID_ISLAND {
            return;
        }
        let id = self.alloc_island();
        let island = &mut self.islands[id as usize];
        rb.ids.island_id = id;
        rb.ids.island_index = 0;
        island.bodies.push(handle);
        island.sleeping = rb.activation.sleeping;
    }

    /// Removes `handle` from its island (body removed, disabled, or turned fixed). Links
    /// referencing the body are *not* eagerly removed: contact links die with their pairs, joint
    /// links when joint edits drain; the split treats their endpoints as non-connecting meanwhile.
    pub fn remove_body(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle) {
        let Some(rb) = bodies.get_mut_internal(handle) else {
            return;
        };
        let island_id = rb.ids.island_id;
        let island_index = rb.ids.island_index;
        rb.ids.island_id = INVALID_ISLAND;
        rb.ids.island_index = u32::MAX;
        self.remove_body_raw(bodies, island_id, island_index);
    }

    /// Like [`Self::remove_body`], for a body that is already gone from the
    /// body set (its ids were captured before removal).
    pub fn remove_body_raw(
        &mut self,
        bodies: &mut RigidBodySet,
        island_id: u32,
        island_index: u32,
    ) {
        if island_id == INVALID_ISLAND || self.islands.get(island_id as usize).is_none() {
            return;
        }
        let index = island_index as usize;

        let island = &mut self.islands[island_id as usize];
        island.bodies.swap_remove(index);
        // Losing a body can split the island exactly like losing a constraint.
        island.constraint_remove_count += 1;
        if let Some(moved) = island.bodies.get(index).copied() {
            bodies.index_mut_internal(moved).ids.island_index = index as u32;
        }

        if island.bodies.is_empty() {
            // Any leftover links reference dead/fixed bodies only: drop them.
            for link in core::mem::take(&mut island.contact_links) {
                self.contact_link_locs[link.edge_id as usize] = INVALID_LOC;
            }
            for link in core::mem::take(&mut self.islands[island_id as usize].joint_links) {
                crate::utils::hashmap_remove(&mut self.joint_link_locs, &link.key);
            }
            self.free_island(island_id);
        }
    }

    /// Links a touching contact (graph edge `edge_id`) between its colliders' parents, merging
    /// their islands if they differ; no-op if already linked. Only island-member sides connect
    /// (fixed or missing parents don't); if neither side is a member, nothing is recorded.
    pub fn link_contact(
        &mut self,
        bodies: &mut RigidBodySet,
        edge_id: u32,
        h1: Option<RigidBodyHandle>,
        h2: Option<RigidBodyHandle>,
    ) {
        if self.contact_link_locs.len() <= edge_id as usize {
            self.contact_link_locs
                .resize(edge_id as usize + 1, INVALID_LOC);
        }
        if self.contact_link_locs[edge_id as usize] != INVALID_LOC {
            return;
        }

        let island_of = |bodies: &RigidBodySet, h: Option<RigidBodyHandle>| {
            h.and_then(|h| bodies.get(h))
                .map(|rb| rb.ids.island_id)
                .filter(|id| *id != INVALID_ISLAND)
        };
        let island1 = island_of(bodies, h1);
        let island2 = island_of(bodies, h2);

        let target = match (island1, island2) {
            (Some(a), Some(b)) => self.merge_islands(bodies, a, b),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (None, None) => return,
        };

        let island = &mut self.islands[target as usize];
        let index = island.contact_links.len() as u32;
        island.contact_links.push(ContactLink {
            edge_id,
            body1: h1.unwrap_or(RigidBodyHandle::invalid()),
            body2: h2.unwrap_or(RigidBodyHandle::invalid()),
        });
        self.contact_link_locs[edge_id as usize] = (target, index);
    }

    /// Unlinks the contact at `edge_id` (stopped touching, or its pair is
    /// being removed). No-op if it wasn't linked.
    pub fn unlink_contact(&mut self, edge_id: u32) {
        let Some(loc) = self.contact_link_locs.get(edge_id as usize).copied() else {
            return;
        };
        if loc == INVALID_LOC {
            return;
        }
        self.contact_link_locs[edge_id as usize] = INVALID_LOC;
        let island = &mut self.islands[loc.0 as usize];
        let link = island.contact_links.swap_remove(loc.1 as usize);
        if let Some(moved) = island.contact_links.get(loc.1 as usize) {
            self.contact_link_locs[moved.edge_id as usize] = loc;
        }
        self.journal_removal(link.body1, link.body2);
    }

    /// Mirrors the contact graph's edges-vec swap-remove: `removed_edge_id` is gone (must already
    /// be unlinked) and the edge at `last_edge_id` moved into its slot. The location table may be
    /// shorter than the edges vec (grows on link, empty after deserialization): untracked slots are implicitly unlinked.
    pub fn contact_edge_removed(&mut self, removed_edge_id: u32, last_edge_id: u32) {
        let n = self.contact_link_locs.len() as u32;
        if removed_edge_id >= n {
            return;
        }
        debug_assert_eq!(
            self.contact_link_locs[removed_edge_id as usize], INVALID_LOC,
            "removed contact edge still linked"
        );
        if n == last_edge_id + 1 {
            // Fully tracked: mirror the swap-remove.
            self.contact_link_locs.swap_remove(removed_edge_id as usize);
            if removed_edge_id != last_edge_id {
                let moved = self.contact_link_locs[removed_edge_id as usize];
                if moved != INVALID_LOC {
                    self.islands[moved.0 as usize].contact_links[moved.1 as usize].edge_id =
                        removed_edge_id;
                }
            }
        }
        // Else: the moved edge was untracked (implicitly unlinked) and the
        // removed slot is already `INVALID_LOC` — nothing to do.
    }

    /// Links a joint edge, merging islands if needed. No-op if already linked.
    pub fn link_joint(
        &mut self,
        bodies: &mut RigidBodySet,
        key: JointLinkKey,
        h1: RigidBodyHandle,
        h2: RigidBodyHandle,
    ) {
        if self.joint_link_locs.contains_key(&key) {
            return;
        }
        let island_of = |bodies: &RigidBodySet, h: RigidBodyHandle| {
            bodies
                .get(h)
                .map(|rb| rb.ids.island_id)
                .filter(|id| *id != INVALID_ISLAND)
        };
        let target = match (island_of(bodies, h1), island_of(bodies, h2)) {
            (Some(a), Some(b)) => self.merge_islands(bodies, a, b),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (None, None) => return,
        };
        let island = &mut self.islands[target as usize];
        let index = island.joint_links.len() as u32;
        island.joint_links.push(JointLink {
            key,
            body1: h1,
            body2: h2,
        });
        self.joint_link_locs.insert(key, (target, index));
    }

    /// Unlinks a joint edge. No-op if it wasn't linked.
    pub fn unlink_joint(&mut self, key: JointLinkKey) {
        let Some(loc) = crate::utils::hashmap_remove(&mut self.joint_link_locs, &key) else {
            return;
        };
        let island = &mut self.islands[loc.0 as usize];
        let link = island.joint_links.swap_remove(loc.1 as usize);
        if let Some(moved) = island.joint_links.get(loc.1 as usize) {
            self.joint_link_locs.insert(moved.key, loc);
        }
        self.journal_removal(link.body1, link.body2);
    }

    /// Records an unlinked edge for [`Self::resolve_removals`] (top of next step). Replaces
    /// an eager `constraint_remove_count += 1`: a removal only dirties its island —
    /// blocking sleep and buying a global split — once the local search fails to settle it cheaply.
    fn journal_removal(&mut self, body1: RigidBodyHandle, body2: RigidBodyHandle) {
        if body1 == body2 {
            // Self-loop (two colliders of the *same* body): never carried connectivity, so losing
            // it can't disconnect anything — and it would fool the local search (both endpoints
            // seed the same body).
            return;
        }
        self.removal_journal.push(Removal { body1, body2 });
    }

    /// Merges two islands (eager, union by size: the island with more bodies
    /// absorbs the other). Returns the surviving island id.
    fn merge_islands(&mut self, bodies: &mut RigidBodySet, a: u32, b: u32) -> u32 {
        if a == b {
            return a;
        }
        let (big, small) =
            if self.islands[a as usize].bodies.len() >= self.islands[b as usize].bodies.len() {
                (a, b)
            } else {
                (b, a)
            };

        let mut small_island =
            core::mem::take(self.islands.get_mut(small as usize).unwrap_or_else(|| {
                unreachable!();
            }));
        let big_island = &mut self.islands[big as usize];

        for handle in &small_island.bodies {
            if let Some(rb) = bodies.get_mut_internal(*handle) {
                rb.ids.island_id = big;
                rb.ids.island_index = big_island.bodies.len() as u32;
            }
            big_island.bodies.push(*handle);
        }
        for link in small_island.contact_links.drain(..) {
            self.contact_link_locs[link.edge_id as usize] =
                (big, big_island.contact_links.len() as u32);
            big_island.contact_links.push(link);
        }
        for link in small_island.joint_links.drain(..) {
            self.joint_link_locs
                .insert(link.key, (big, big_island.joint_links.len() as u32));
            big_island.joint_links.push(link);
        }
        big_island.constraint_remove_count += small_island.constraint_remove_count;
        big_island.sleeping &= small_island.sleeping;
        self.free_island(small);
        big
    }

    /// Starts a new per-step sleep scan (see [`Self::observe_body_for_sleep`]).
    pub fn begin_sleep_scan(&mut self) {
        self.sleep_scan_stamp = self.sleep_scan_stamp.wrapping_add(1);
        // Island ids can exceed `islands.len()` (entry count): the id space is
        // `len + free` (every hole is in the free list).
        let id_space = self.islands.len() + self.free_islands.len();
        if self.sleep_scan.len() < id_space {
            self.sleep_scan.resize(id_space, (0, false));
        }
        self.sleep_scan_touched.clear();
    }

    /// Feeds one awake body's sleep eligibility into the scan.
    #[inline]
    pub fn observe_body_for_sleep(&mut self, island_id: u32, eligible: bool) {
        let slot = &mut self.sleep_scan[island_id as usize];
        if slot.0 != self.sleep_scan_stamp {
            *slot = (self.sleep_scan_stamp, eligible);
            self.sleep_scan_touched.push(island_id);
        } else {
            slot.1 &= eligible;
        }
    }

    /// Ends the scan: returns islands whose every observed body is eligible and that pass the
    /// split guard (an island that lost constraints must split before sleeping unless
    /// single-body). Returned islands are NOT yet marked sleeping — the caller commits them.
    pub fn finish_sleep_scan(&mut self) -> Vec<u32> {
        let mut sleepable = Vec::new();
        for id in self.sleep_scan_touched.drain(..) {
            if !self.sleep_scan[id as usize].1 {
                continue;
            }
            let Some(island) = self.islands.get(id as usize) else {
                continue;
            };
            if island.sleeping {
                continue;
            }
            if island.constraint_remove_count > 0 && island.bodies.len() > 1 {
                continue;
            }
            sleepable.push(id);
        }
        sleepable
    }

    /// Marks an island as asleep (the caller moves its bodies out of the
    /// active set) and cancels any pending split targeting it.
    pub fn mark_island_sleeping(&mut self, island_id: u32) {
        self.islands[island_id as usize].sleeping = true;
        self.clear_pending_split_of(island_id);
    }

    /// Applies a deferred impulse-joint connectivity event.
    pub fn apply_impulse_joint_event(
        &mut self,
        bodies: &mut RigidBodySet,
        event: ImpulseJointIslandEvent,
    ) {
        match event {
            ImpulseJointIslandEvent::Link {
                handle,
                body1,
                body2,
            } => self.link_joint(bodies, JointLinkKey::Impulse(handle), body1, body2),
            ImpulseJointIslandEvent::Unlink { handle } => {
                self.unlink_joint(JointLinkKey::Impulse(handle))
            }
        }
    }

    /// Refreshes the multibody's internal connectivity chain: unlinks the old chain, then (if it
    /// still exists under `mb_id`) re-links one over its non-fixed bodies in link order.
    /// Multibodies are atomic for sleep — branches under a fixed root must share one island, which per-joint edges wouldn't guarantee (an edge to a fixed root doesn't connect).
    pub fn refresh_multibody_chain(
        &mut self,
        bodies: &mut RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        mb_id: MultibodyIndex,
    ) {
        let raw = multibody_index_key(mb_id);
        // Unlink the previous chain (ordinals are dense from 0).
        let mut ordinal = 0;
        loop {
            let key = JointLinkKey::MultibodyChain {
                multibody: raw,
                ordinal,
            };
            if !self.joint_link_locs.contains_key(&key) {
                break;
            }
            self.unlink_joint(key);
            ordinal += 1;
        }

        let Some(mb) = multibody_joints.get_multibody(mb_id) else {
            return;
        };

        let mut prev: Option<RigidBodyHandle> = None;
        let mut ordinal = 0;
        for link in mb.links() {
            let handle = link.rigid_body;
            let is_member = bodies
                .get(handle)
                .is_some_and(|rb| !rb.is_fixed() && rb.is_enabled());
            if !is_member {
                continue;
            }
            if let Some(prev) = prev {
                self.link_joint(
                    bodies,
                    JointLinkKey::MultibodyChain {
                        multibody: raw,
                        ordinal,
                    },
                    prev,
                    handle,
                );
                ordinal += 1;
            }
            prev = Some(handle);
        }
    }

    /// Rebuilds everything from the current world state (first step after construction or
    /// deserialization): singleton islands, then every touching contact and enabled joint linked.
    /// Returns sleeping bodies stranded in a *non*-sleeping island (partial-island-era serialized state); the caller must wake them to restore the whole-island invariant.
    pub fn bootstrap(
        &mut self,
        bodies: &mut RigidBodySet,
        touching_pairs: impl Iterator<Item = (u32, Option<RigidBodyHandle>, Option<RigidBodyHandle>)>,
        impulse_joints: &crate::dynamics::ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
    ) -> Vec<RigidBodyHandle> {
        self.islands = VecMap::default();
        self.free_islands.clear();
        self.contact_link_locs.clear();
        self.joint_link_locs.clear();
        self.split_island = None;
        // The rebuild below re-derives the components from scratch, which
        // subsumes anything the journal was going to answer.
        self.removal_journal.clear();

        let handles: Vec<RigidBodyHandle> = bodies.iter().map(|(h, _)| h).collect();
        for handle in handles {
            let rb = bodies.index_mut_internal(handle);
            rb.ids.island_id = INVALID_ISLAND;
            rb.ids.island_index = u32::MAX;
            self.ensure_body(bodies, handle);
        }

        for (edge_id, h1, h2) in touching_pairs {
            self.link_contact(bodies, edge_id, h1, h2);
        }

        for (handle, joint) in impulse_joints.iter() {
            if joint.data.is_enabled() {
                self.link_joint(
                    bodies,
                    JointLinkKey::Impulse(handle),
                    joint.body1,
                    joint.body2,
                );
            }
        }

        let mb_ids: Vec<MultibodyIndex> = multibody_joints
            .multibodies
            .iter()
            .map(|(id, _)| MultibodyIndex(id))
            .collect();
        for mb_id in mb_ids {
            self.refresh_multibody_chain(bodies, multibody_joints, mb_id);
        }

        // Merging propagated `sleeping &=`, so an island is `sleeping` iff
        // every body of it was; sleeping bodies stranded in a mixed island
        // must be woken by the caller.
        let mut to_wake = Vec::new();
        for (_, island) in self.islands.iter() {
            if !island.sleeping {
                to_wake.extend(
                    island
                        .bodies
                        .iter()
                        .filter(|h| bodies.get(**h).is_some_and(|rb| rb.activation.sleeping))
                        .copied(),
                );
            }
        }

        self.bootstrapped = true;
        to_wake
    }

    /// Structural validation (debug/test only): every membership index,
    /// location table entry, and link endpoint is consistent.
    #[allow(dead_code)]
    pub fn assert_consistent(&self, bodies: &RigidBodySet) {
        let mut seen_bodies = 0;
        for (id, island) in self.islands.iter() {
            assert!(!island.bodies.is_empty(), "empty island {id} kept alive");
            for (index, handle) in island.bodies.iter().enumerate() {
                let rb = &bodies[*handle];
                assert_eq!(rb.ids.island_id as usize, id);
                assert_eq!(rb.ids.island_index as usize, index);
                assert!(!rb.is_fixed());
                // Whole-island sleep: a sleeping island's bodies are all
                // asleep. (An *awake* island may contain manually-slept
                // bodies until the whole island becomes eligible.)
                if island.sleeping {
                    assert!(
                        rb.activation.sleeping,
                        "awake body {handle:?} inside sleeping island {id}"
                    );
                }
                seen_bodies += 1;
            }
            for (index, link) in island.contact_links.iter().enumerate() {
                assert_eq!(
                    self.contact_link_locs[link.edge_id as usize],
                    (id as u32, index as u32)
                );
                // At least one endpoint must be a member of this island.
                let member = |h: RigidBodyHandle| {
                    bodies
                        .get(h)
                        .is_some_and(|rb| rb.ids.island_id as usize == id)
                };
                assert!(member(link.body1) || member(link.body2));
            }
            for (index, link) in island.joint_links.iter().enumerate() {
                assert_eq!(self.joint_link_locs[&link.key], (id as u32, index as u32));
                let member = |h: RigidBodyHandle| {
                    bodies
                        .get(h)
                        .is_some_and(|rb| rb.ids.island_id as usize == id)
                };
                assert!(member(link.body1) || member(link.body2));
            }
        }

        // Every enabled non-fixed body is in exactly one island.
        let mut expected_bodies = 0;
        for (handle, rb) in bodies.iter() {
            if !rb.is_fixed() && rb.is_enabled() {
                expected_bodies += 1;
                assert_ne!(
                    rb.ids.island_id, INVALID_ISLAND,
                    "body {handle:?} has no island"
                );
            } else {
                assert_eq!(rb.ids.island_id, INVALID_ISLAND);
            }
        }
        assert_eq!(seen_bodies, expected_bodies);

        // Location tables point at real links.
        for (edge_id, loc) in self.contact_link_locs.iter().enumerate() {
            if *loc != INVALID_LOC {
                let island = &self.islands[loc.0 as usize];
                assert_eq!(
                    island.contact_links[loc.1 as usize].edge_id as usize,
                    edge_id
                );
            }
        }
        for (key, loc) in self.joint_link_locs.iter() {
            let island = &self.islands[loc.0 as usize];
            assert_eq!(island.joint_links[loc.1 as usize].key, *key);
        }
    }
}
