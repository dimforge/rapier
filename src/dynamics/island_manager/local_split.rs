//! Bounded local island splits (decremental connectivity). Answering every removal *globally* is an O(island) union-find — a 43k-body pyramid shedding one bouncing box would pay a 163k-link scan every other step; here a **lockstep** dual search from the unlinked edge's endpoints answers it at O(smaller piece): meeting the other side ⇒ still connected, island NOT dirtied and still sleep-eligible; one frontier exhausting ⇒ that side is the detached — necessarily smaller — component, moved out in O(its size); [`SEARCH_BUDGET`] exceeded ⇒ dirty the island and defer to the global split ([`PersistentIslands::split_island_now`]), which bounds the worst case at the old behavior.
//! Adjacency reuses existing structures (no new bookkeeping): [`NarrowPhase::touching_edges_with`] (its edge ids and touching predicate are exactly what keys the island's contact links), [`ImpulseJointSet::attached_joints`], and multibody chain links (a multibody is one atomic-for-sleep neighborhood; its chain links travel with it).
//! Ordering: every unlink of a step happens in the narrow phase, *before* islands update, so all searches run against the final post-removal graph — verdicts are batch-order-independent.

#[cfg(not(feature = "std"))]
use simba::scalar::ComplexField as _;

use super::INVALID_ISLAND;
use super::persistent::{JointLinkKey, PersistentIslands, Removal, multibody_index_key};
use crate::alloc_prelude::*;
use crate::dynamics::{ImpulseJointSet, MultibodyJointSet, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::math::Real;
use crate::utils::DotProduct;

/// Total expansions (both sides) a removal may spend before handing off to the global split.
/// Only bounds the pathological case (still connected, but only the long way around a big cycle);
/// deliberately ~10x under the fallback's cost (1024 expansions ≈ 10k edges vs ~200k for a global split of a big pile). Real cases settle in a couple of hops (5,411/5,412 on the 43k pyramid).
const SEARCH_BUDGET: usize = 1024;

/// An island link incident to a body, in the form needed to relocate it.
#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
enum IncidentLink {
    Contact(u32),
    Joint(JointLinkKey),
}

/// Reusable buffers: the search runs every step and must not allocate once warm.
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(super) struct LocalSplitScratch {
    /// `(stamp, side)` per body-arena index. Stamped, so a search never clears the
    /// map — only the bodies it actually touched carry the live stamp.
    visited: Vec<(u32, u8)>,
    stamp: u32,
    /// The two frontiers, and every body each side has reached.
    frontier: [Vec<RigidBodyHandle>; 2],
    reached: [Vec<RigidBodyHandle>; 2],
    /// The detached component's links, collected while `bodies` is still only
    /// immutably borrowed.
    links: Vec<IncidentLink>,
}

/// The read-only graph the search walks.
struct IslandGraph<'a> {
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    narrow_phase: &'a NarrowPhase,
    impulse_joints: &'a ImpulseJointSet,
    multibody_joints: &'a MultibodyJointSet,
}

impl IslandGraph<'_> {
    /// The island of `handle`, if it is a member of one (fixed, disabled and
    /// removed bodies are not).
    fn island_of(&self, handle: RigidBodyHandle) -> Option<u32> {
        let id = self.bodies.get(handle)?.ids.island_id;
        (id != INVALID_ISLAND).then_some(id)
    }

    /// Whether `handle` is an island member (the multibody chain's own rule).
    fn is_member(&self, handle: RigidBodyHandle) -> bool {
        self.bodies
            .get(handle)
            .is_some_and(|rb| !rb.is_fixed() && rb.is_enabled())
    }

    /// Calls `f` for every body of `island_id` linked to `body`. The hot path:
    /// this is what the frontier expansion runs.
    fn for_each_neighbor(
        &self,
        body: RigidBodyHandle,
        island_id: u32,
        mut f: impl FnMut(RigidBodyHandle),
    ) {
        let Some(rb) = self.bodies.get(body) else {
            return;
        };

        let mut visit = |other: RigidBodyHandle| {
            if other != body && self.island_of(other) == Some(island_id) {
                f(other);
            }
        };

        for collider in rb.colliders() {
            for (_, other_co) in self.narrow_phase.touching_edges_with(*collider) {
                if let Some(parent) = self.colliders.get(other_co).and_then(|c| c.parent()) {
                    visit(parent);
                }
            }
        }

        for (body1, body2, _, joint) in self.impulse_joints.attached_joints(body) {
            if joint.data.is_enabled() {
                visit(if body1 == body { body2 } else { body1 });
            }
        }

        if let Some(link) = self.multibody_joints.rigid_body_link(body) {
            if let Some(mb) = self.multibody_joints.get_multibody(link.multibody) {
                for mb_link in mb.links() {
                    visit(mb_link.rigid_body);
                }
            }
        }
    }

    /// Calls `f` for every island link incident to `body` — including links whose far side is
    /// *not* an island member (contact against fixed geometry): those carry no connectivity, but
    /// they belong to the island and must follow the body when it moves out.
    fn for_each_incident_link(&self, body: RigidBodyHandle, mut f: impl FnMut(IncidentLink)) {
        let Some(rb) = self.bodies.get(body) else {
            return;
        };

        for collider in rb.colliders() {
            for (edge_id, _) in self.narrow_phase.touching_edges_with(*collider) {
                f(IncidentLink::Contact(edge_id));
            }
        }

        for (_, _, handle, joint) in self.impulse_joints.attached_joints(body) {
            if joint.data.is_enabled() {
                f(IncidentLink::Joint(JointLinkKey::Impulse(handle)));
            }
        }

        // The chain links are keyed by ordinal, not by body: one per *consecutive
        // pair of members*, exactly as `refresh_multibody_chain` numbers them. A
        // multibody moves as a whole, so emit all of them.
        if let Some(link) = self.multibody_joints.rigid_body_link(body) {
            let mb_id = link.multibody;
            if let Some(mb) = self.multibody_joints.get_multibody(mb_id) {
                let members = mb.links().filter(|l| self.is_member(l.rigid_body)).count();
                let multibody = multibody_index_key(mb_id);
                for ordinal in 0..members.saturating_sub(1) as u32 {
                    f(IncidentLink::Joint(JointLinkKey::MultibodyChain {
                        multibody,
                        ordinal,
                    }));
                }
            }
        }
    }
}

/// What a single removal turned out to mean.
enum Verdict {
    /// The endpoints are still connected: nothing changed.
    Connected,
    /// They are in different components now; `side` is the smaller one, and its
    /// bodies are in `scratch.reached[side]`.
    Detached(usize),
    /// Not settled within the budget: hand it to the global split.
    OverBudget,
}

impl PersistentIslands {
    /// Resolves the edges unlinked since the last call (see module docs). Runs at the top of the
    /// step — after the narrow phase's touch transitions, before the split-candidate bids — so a
    /// harmless removal never reaches the global machinery at all.
    pub(crate) fn resolve_removals(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        length_unit: Real,
    ) {
        if self.removal_journal.is_empty() {
            return;
        }

        let journal = core::mem::take(&mut self.removal_journal);
        let mut scratch = core::mem::take(&mut self.local_split);

        for removal in &journal {
            let graph = IslandGraph {
                bodies,
                colliders,
                narrow_phase,
                impulse_joints,
                multibody_joints,
            };

            // Re-derive the endpoints' islands: an earlier removal of this batch
            // may already have moved one of them out.
            let (Some(island1), Some(island2)) = (
                graph.island_of(removal.body1),
                graph.island_of(removal.body2),
            ) else {
                // An endpoint is fixed, disabled or gone: the edge carried no connectivity, so
                // losing it can't disconnect anything. (A body *removal* is different — a body can
                // be a cut vertex — and `remove_body_raw` still dirties its island eagerly.)
                continue;
            };

            if island1 != island2 {
                // Already in different islands: an earlier removal of this batch
                // detached one of them, which already accounted for this edge.
                continue;
            }

            // A sleeping island's bodies sit in an active-set sleeping chunk that
            // would not follow a split, so leave it to the global path — which
            // knows to skip sleeping islands, exactly as before.
            if self.islands[island1 as usize].sleeping {
                self.islands[island1 as usize].constraint_remove_count += 1;
                continue;
            }

            // If BOTH endpoints move above the sleep speed threshold, the removal can't be what
            // keeps a sleepy island awake: skip the search, defer to the global split —
            // churny scenes (mixers, tumblers) pay nothing, while a flickering or detaching contact on a resting pile still resolves locally. Gates on CURRENT speed (the sleep-energy farthest-point metric), not the stillness timer: a strong wake zeroes a whole island's timers, yet a still body separating (e.g. its neighbor teleported away) must split out the very step it stops touching.
            let hot = |h: crate::dynamics::RigidBodyHandle| {
                graph.bodies.get(h).is_some_and(|rb| {
                    let lin_threshold = rb.activation.normalized_linear_threshold * length_unit;
                    if lin_threshold < 0.0 {
                        // Never-sleeps body: always "hot" (it keeps its island
                        // awake regardless of any split).
                        return true;
                    }
                    let sq_linvel = rb.vels.linvel.length_squared();
                    let sq_angvel = rb.vels.angvel.gdot(rb.vels.angvel);
                    let max_point_vel =
                        sq_linvel.sqrt() + sq_angvel.sqrt() * rb.mprops.max_extent();
                    max_point_vel > lin_threshold
                })
            };
            if hot(removal.body1) && hot(removal.body2) {
                self.islands[island1 as usize].constraint_remove_count += 1;
                continue;
            }

            match search(&graph, &mut scratch, island1, removal) {
                Verdict::Connected => {}
                Verdict::OverBudget => {
                    self.islands[island1 as usize].constraint_remove_count += 1;
                }
                Verdict::Detached(side) => {
                    // Collect the component's links while `bodies` is still only
                    // immutably borrowed, then relocate bodies and links together.
                    let component = core::mem::take(&mut scratch.reached[side]);
                    scratch.links.clear();
                    for body in &component {
                        graph.for_each_incident_link(*body, |link| scratch.links.push(link));
                    }
                    self.move_component_out(bodies, island1, &component, &scratch.links);
                    scratch.reached[side] = component;
                }
            }
        }

        self.local_split = scratch;
        self.removal_journal = journal;
        self.removal_journal.clear();
    }

    /// Moves `component` — a set of bodies now disconnected from the rest of
    /// `island_id` — and its `links` into a fresh island. O(component).
    fn move_component_out(
        &mut self,
        bodies: &mut RigidBodySet,
        island_id: u32,
        component: &[RigidBodyHandle],
        links: &[IncidentLink],
    ) {
        let new_id = self.alloc_island();
        let sleeping = self.islands[island_id as usize].sleeping;
        {
            let island = &mut self.islands[new_id as usize];
            island.sleeping = sleeping;
            island.bodies.reserve(component.len());
            island.contact_links.reserve(links.len());
        }

        // Links first: they are found through the loc tables, which the body moves
        // below don't disturb. A link between two component bodies is enumerated
        // from both ends — the loc check makes the second visit a no-op.
        for link in links {
            match *link {
                IncidentLink::Contact(edge_id) => {
                    let Some(loc) = self.contact_link_locs.get(edge_id as usize).copied() else {
                        continue;
                    };
                    if loc.0 != island_id {
                        continue;
                    }
                    let base = &mut self.islands[island_id as usize];
                    let moved = base.contact_links.swap_remove(loc.1 as usize);
                    if let Some(swapped) = base.contact_links.get(loc.1 as usize) {
                        self.contact_link_locs[swapped.edge_id as usize] = loc;
                    }
                    let island = &mut self.islands[new_id as usize];
                    self.contact_link_locs[edge_id as usize] =
                        (new_id, island.contact_links.len() as u32);
                    island.contact_links.push(moved);
                }
                IncidentLink::Joint(key) => {
                    let Some(loc) = self.joint_link_locs.get(&key).copied() else {
                        continue;
                    };
                    if loc.0 != island_id {
                        continue;
                    }
                    let base = &mut self.islands[island_id as usize];
                    let moved = base.joint_links.swap_remove(loc.1 as usize);
                    if let Some(swapped) = base.joint_links.get(loc.1 as usize) {
                        self.joint_link_locs.insert(swapped.key, loc);
                    }
                    let island = &mut self.islands[new_id as usize];
                    self.joint_link_locs
                        .insert(key, (new_id, island.joint_links.len() as u32));
                    island.joint_links.push(moved);
                }
            }
        }

        for handle in component {
            let index = bodies[*handle].ids.island_index as usize;
            let base = &mut self.islands[island_id as usize];
            debug_assert_eq!(base.bodies[index], *handle);
            base.bodies.swap_remove(index);
            if let Some(swapped) = base.bodies.get(index).copied() {
                bodies.index_mut_internal(swapped).ids.island_index = index as u32;
            }

            let island = &mut self.islands[new_id as usize];
            let rb = bodies.index_mut_internal(*handle);
            rb.ids.island_id = new_id;
            rb.ids.island_index = island.bodies.len() as u32;
            island.bodies.push(*handle);
        }

        // The component was strictly smaller than the island it left.
        debug_assert!(!self.islands[island_id as usize].bodies.is_empty());
    }
}

/// The lockstep dual search. See the module docs.
fn search(
    graph: &IslandGraph,
    scratch: &mut LocalSplitScratch,
    island_id: u32,
    removal: &Removal,
) -> Verdict {
    scratch.stamp = scratch.stamp.wrapping_add(1);
    if scratch.stamp == 0 {
        // Wrapped: a zeroed (never-visited) entry would alias stamp 0.
        scratch.visited.clear();
        scratch.stamp = 1;
    }
    let stamp = scratch.stamp;

    for side in 0..2 {
        scratch.frontier[side].clear();
        scratch.reached[side].clear();
    }
    for (side, seed) in [removal.body1, removal.body2].into_iter().enumerate() {
        mark(scratch, seed, stamp, side as u8);
        scratch.frontier[side].push(seed);
        scratch.reached[side].push(seed);
    }

    let mut expansions = 0;
    loop {
        for side in 0..2 {
            let Some(body) = scratch.frontier[side].pop() else {
                // This side ran out of frontier without ever reaching the other
                // seed: it is a detached component — and, having advanced in
                // lockstep, the smaller of the two pieces.
                return Verdict::Detached(side);
            };

            let mut met = false;
            graph.for_each_neighbor(body, island_id, |neighbor| {
                let (index, _) = neighbor.into_raw_parts();
                match scratch.visited.get(index as usize) {
                    Some(&(s, seen_side)) if s == stamp => {
                        // Reaching a body the *other* search already owns means the
                        // two endpoints are still connected through it.
                        met |= seen_side != side as u8;
                    }
                    _ => {
                        mark(scratch, neighbor, stamp, side as u8);
                        scratch.frontier[side].push(neighbor);
                        scratch.reached[side].push(neighbor);
                    }
                }
            });
            if met {
                return Verdict::Connected;
            }

            expansions += 1;
            if expansions >= SEARCH_BUDGET {
                return Verdict::OverBudget;
            }
        }
    }
}

#[inline]
fn mark(scratch: &mut LocalSplitScratch, handle: RigidBodyHandle, stamp: u32, side: u8) {
    let (index, _) = handle.into_raw_parts();
    if scratch.visited.len() <= index as usize {
        scratch.visited.resize(index as usize + 1, (0, 0));
    }
    scratch.visited[index as usize] = (stamp, side);
}
