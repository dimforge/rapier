//! Substep solve-groups: partition of the awake set by effective `RigidBody::additional_solver_iterations`. Coupled bodies must share a substep cadence, so the unit of elevation is a connected component of the awake constraint graph (count = max over members); components are then grouped **by count, not by component**, so the awake set splits into `#distinct counts` contiguous ranges (typically 2) instead of re-fragmenting into per-island solves.
//! Runs only when some awake body is elevated (`any_extra`); recomputed from live state every step but *applied* (reorder + `active_set_id` re-stamp + epoch bump) only when the list isn't already grouped — the epoch bump forces a full solver-contact-graph rebuild, so steady-state elevated scenes must re-partition zero times per step.

use super::IslandManager;
use crate::data::union_find::UnionFind;
use crate::dynamics::{ImpulseJointSet, MultibodyJointSet, RigidBodyHandle, RigidBodySet};
use crate::geometry::NarrowPhase;
use alloc::vec::Vec;
use core::ops::Range;

/// A contiguous range of the awake island's bodies sharing one substep count. Ranges are ordered
/// by decreasing `extra_iters`, so kinematic bodies (never merged, assigned to the highest-cadence
/// group they touch) are integrated before any lower-cadence group solves against them.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SolveGroup {
    /// Range into the awake island's `bodies` (== solver-body slot range).
    pub body_range: Range<usize>,
    /// Extra substeps for this group, on top of
    /// `IntegrationParameters::num_solver_iterations`.
    pub extra_iters: u32,
}

/// Scratch state for [`IslandManager::update_substep_groups`], kept to reuse
/// allocations across steps.
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SubstepGroupsWorkspace {
    uf: UnionFind,
    /// Effective extra count per awake body slot (component max).
    keys: Vec<u32>,
    /// Distinct extra counts in use, sorted descending.
    distinct: Vec<u32>,
    /// Scatter cursors/counts per distinct key.
    offsets: Vec<usize>,
    /// Reorder scratch for the awake `bodies` vec.
    scratch: Vec<RigidBodyHandle>,
}

impl IslandManager {
    /// Recomputes the awake set's substep solve-groups, reordering the awake island's body list so
    /// groups are contiguous (descending extra count). Must run after `update_islands` (last
    /// mutator of the awake body list) and before anything consumes body order or stamps solver-body indices. `any_extra` = caller's OR of `additional_solver_iterations > 0`; when `false` this is one branch + `Vec::clear` (empty groups = one implicit whole-set group).
    pub(crate) fn update_substep_groups(
        &mut self,
        any_extra: bool,
        bodies: &mut RigidBodySet,
        narrow_phase: &NarrowPhase,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
    ) {
        self.solve_groups.clear();
        if !any_extra {
            return;
        }
        let Some(awake_id) = self.awake_island else {
            return;
        };
        let num_bodies = self.islands[awake_id].bodies.len();

        let ws = &mut self.substep_groups_workspace;
        ws.uf.reset(num_bodies);

        // An edge merges components only between awake *dynamic* bodies: fixed/kinematic bodies
        // are not solver DOFs and must not glue unrelated components (a kinematic platform would
        // merge an elevated assembly with the default world); sleeping neighbors are read-only walls.
        let slot = |handle: RigidBodyHandle| -> Option<u32> {
            let rb = bodies.get(handle)?;
            (rb.is_dynamic() && !rb.is_sleeping() && rb.ids.active_island_id == awake_id as u32)
                .then_some(rb.ids.active_set_id)
        };

        // Contact edges, from the narrow-phase's current pairs (post `detect_collisions`: no stale
        // refs, no one-step lag). Solver-side filters (COMPUTE_IMPULSES) are deliberately ignored:
        // over-merging only substeps slightly more; under-merging would split coupled bodies' cadences.
        for pair in narrow_phase.contact_pairs() {
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some(manifold) = pair.manifolds.first() else {
                continue;
            };
            if let (Some(h1), Some(h2)) = (manifold.data.rigid_body1, manifold.data.rigid_body2)
                && let (Some(s1), Some(s2)) = (slot(h1), slot(h2))
            {
                ws.uf.union(s1, s2);
            }
        }

        // Impulse-joint edges. All joints are considered (not just this step's
        // active selection, which isn't computed yet at this point): a joint
        // between two awake dynamic bodies couples them regardless.
        for (_, joint) in impulse_joints.iter() {
            if let (Some(s1), Some(s2)) = (slot(joint.body1), slot(joint.body2)) {
                ws.uf.union(s1, s2);
            }
        }

        // Multibody-joint edges: each joint links a body to its parent link's
        // body. Chaining parent unions connects every dynamic link of an
        // articulation even when its root is fixed.
        for (_, link_id, multibody, link) in multibody_joints.iter() {
            if let Some(parent) = multibody.link(link.parent_internal_id)
                && let (Some(s1), Some(s2)) = (
                    slot(parent.rigid_body_handle()),
                    slot(link.rigid_body_handle()),
                )
            {
                ws.uf.union(s1, s2);
            }
            let _ = link_id;
        }

        // Effective extra count per body = max over its component. Non-dynamic
        // awake bodies (kinematic) stay singletons here and get key 0, then are
        // lifted to the highest-cadence group they touch below.
        ws.keys.clear();
        ws.keys.resize(num_bodies, 0);
        for i in 0..num_bodies {
            let handle = self.islands[awake_id].bodies[i];
            let extra = bodies[handle].additional_solver_iterations() as u32;
            if extra > 0 {
                let root = ws.uf.find(i as u32) as usize;
                ws.keys[root] = ws.keys[root].max(extra);
            }
        }
        for i in 0..num_bodies {
            let root = ws.uf.find(i as u32) as usize;
            ws.keys[i] = ws.keys[root];
        }

        // Lift kinematic bodies to the max key among the dynamic bodies they touch. Contacts are
        // the only solver coupling a kinematic body has; joints follow the same rule for consistency.
        let kinematic_slot = |handle: RigidBodyHandle| -> Option<u32> {
            let rb = bodies.get(handle)?;
            (!rb.is_dynamic()
                && rb.is_dynamic_or_kinematic()
                && !rb.is_sleeping()
                && rb.ids.active_island_id == awake_id as u32)
                .then_some(rb.ids.active_set_id)
        };
        let lift = |ws: &mut SubstepGroupsWorkspace, h1, h2| {
            if let (Some(k), Some(d)) = (kinematic_slot(h1), slot(h2)) {
                ws.keys[k as usize] = ws.keys[k as usize].max(ws.keys[d as usize]);
            }
        };
        for pair in narrow_phase.contact_pairs() {
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some(manifold) = pair.manifolds.first() else {
                continue;
            };
            if let (Some(h1), Some(h2)) = (manifold.data.rigid_body1, manifold.data.rigid_body2) {
                lift(ws, h1, h2);
                lift(ws, h2, h1);
            }
        }
        for (_, joint) in impulse_joints.iter() {
            lift(ws, joint.body1, joint.body2);
            lift(ws, joint.body2, joint.body1);
        }

        // Fast path: the list is already grouped (keys non-increasing along the
        // vec). Steady-state elevated scenes take this branch every step; only
        // derive the ranges, no reorder, no epoch bump.
        if ws.keys.is_sorted_by(|a, b| a >= b) {
            push_group_ranges(&mut self.solve_groups, &ws.keys);
            return;
        }

        // Reorder: stable counting sort by key, descending. Distinct keys are
        // few (one per elevation level in use), so the "find ordinal" scans are
        // effectively O(1).
        ws.distinct.clear();
        for &k in &ws.keys {
            if !ws.distinct.contains(&k) {
                ws.distinct.push(k);
            }
        }
        ws.distinct.sort_unstable_by(|a, b| b.cmp(a));

        ws.offsets.clear();
        ws.offsets.resize(ws.distinct.len(), 0);
        for &k in &ws.keys {
            let ord = ws.distinct.iter().position(|&d| d == k).unwrap();
            ws.offsets[ord] += 1;
        }
        let mut start = 0;
        for count in &mut ws.offsets {
            let c = *count;
            *count = start;
            start += c;
        }

        let island_bodies = &mut self.islands[awake_id].bodies;
        ws.scratch.clear();
        ws.scratch.resize(num_bodies, RigidBodyHandle::invalid());
        for (i, &handle) in island_bodies.iter().enumerate() {
            let ord = ws.distinct.iter().position(|&d| d == ws.keys[i]).unwrap();
            ws.scratch[ws.offsets[ord]] = handle;
            ws.offsets[ord] += 1;
        }
        core::mem::swap(island_bodies, &mut ws.scratch);

        // Re-stamp the invariant `active_set_id == index in bodies`, re-derive
        // the (now sorted) keys, and invalidate every ordering-derived cache.
        for (i, handle) in self.islands[awake_id].bodies.iter().enumerate() {
            bodies.index_mut_internal(*handle).ids.active_set_id = i as u32;
        }
        ws.keys.sort_unstable_by(|a, b| b.cmp(a));
        push_group_ranges(&mut self.solve_groups, &ws.keys);
        self.bump_active_set_epoch();
    }
}

/// Derives contiguous group ranges from a descending-sorted key sequence.
fn push_group_ranges(groups: &mut Vec<SolveGroup>, keys: &[u32]) {
    let mut start = 0;
    for i in 1..=keys.len() {
        if i == keys.len() || keys[i] != keys[start] {
            groups.push(SolveGroup {
                body_range: start..i,
                extra_iters: keys[start],
            });
            start = i;
        }
    }
}
