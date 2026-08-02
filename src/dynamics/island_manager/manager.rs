use super::Island;
use crate::alloc_prelude::*;
use crate::dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBody, RigidBodyChanges, RigidBodyHandle, RigidBodyIds,
    RigidBodySet,
};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::math::Real;
use crate::utils::DotProduct;
use parry::utils::VecMap;

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
/// All awake bodies live in a single active set solved together. Sleep is decided per
/// **island** — a connected component of the touching-contact/joint graph, maintained
/// persistently (eager merges, deferred splits): an island falls asleep once
/// *every* body has been sleep-eligible long enough, and wakes as a single unit.
///
/// You rarely interact with this directly - it's automatically managed by [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
pub struct IslandManager {
    /// Bumped whenever any body's `active_set_id`/island assignment changes, so
    /// the solver's persistent constraint cache can cheaply detect that its cached
    /// solver-body indices went stale.
    pub(crate) active_set_epoch: u32,
    pub(crate) islands: VecMap<Island>,
    /// The single awake island's id, if any island is awake (all awake bodies live
    /// in one island; every other `Island` container is a sleeping chunk).
    pub(crate) awake_island: Option<usize>,
    pub(crate) free_islands: Vec<usize>,
    /// The awake set's substep solve-groups, recomputed each step by
    /// [`Self::update_substep_groups`]. Empty = one implicit group spanning the whole
    /// awake set (no body has `additional_solver_iterations > 0`).
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) solve_groups: Vec<super::SolveGroup>,
    /// Scratch buffers for [`Self::update_substep_groups`].
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(super) substep_groups_workspace: super::substep_groups::SubstepGroupsWorkspace,
    /// The persistent islands: connected components of the touching-contact/joint
    /// graph, maintained incrementally.
    pub(crate) persistent: super::PersistentIslands,
}

impl IslandManager {
    /// Creates a new empty island manager.
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub(crate) fn bump_active_set_epoch(&mut self) {
        self.active_set_epoch = self.active_set_epoch.wrapping_add(1);
    }

    pub(crate) fn rigid_body_removed_or_disabled(
        &mut self,
        removed_handle: RigidBodyHandle,
        removed_ids: &RigidBodyIds,
        bodies: &mut RigidBodySet,
    ) {
        self.bump_active_set_epoch();

        // Persistent islands: drop the body (clears the live body's ids too,
        // for the disabled case; uses the captured ids for the removed case).
        if let Some(rb) = bodies.get_mut_internal(removed_handle) {
            rb.ids.island_id = super::INVALID_ISLAND;
            rb.ids.island_index = u32::MAX;
        }
        self.persistent
            .remove_body_raw(bodies, removed_ids.island_id, removed_ids.island_index);

        let Some(island) = self.islands.get_mut(removed_ids.active_island_id as usize) else {
            // The island already doesn’t exist.
            return;
        };

        // If the rigid-body was disabled, it is still in the body set. Invalid its islands ids.
        if let Some(body) = bodies.get_mut_internal(removed_handle) {
            body.ids.active_island_id = u32::MAX;
            body.ids.active_set_id = u32::MAX;
        }

        let swapped_handle = island.bodies.last().copied().unwrap_or(removed_handle);
        island
            .bodies
            .swap_remove(removed_ids.active_set_id as usize);

        // Remap the active_set_id of the body we moved with the `swap_remove`.
        if swapped_handle != removed_handle {
            let swapped_body = bodies
                .get_mut(swapped_handle)
                .expect("Internal error: bodies must be removed from islands on at a times");
            swapped_body.ids.active_set_id = removed_ids.active_set_id;
        }

        // If we deleted the last body from this island, delete the island.
        if island.bodies.is_empty() {
            if self.awake_island == Some(removed_ids.active_island_id as usize) {
                self.awake_island = None;
            }
            self.islands.remove(removed_ids.active_island_id as usize);
            self.free_islands
                .push(removed_ids.active_island_id as usize);
        }
    }

    /// Handles an interaction starting or stopping between the two endpoints:
    /// wakes both when requested.
    pub(crate) fn interaction_changed(
        &mut self,
        bodies: &mut RigidBodySet,
        handle1: Option<RigidBodyHandle>,
        handle2: Option<RigidBodyHandle>,
        wake_up: bool,
    ) {
        // NOTE: no epoch bump here: a contact start/stop within one island doesn't
        // renumber anything; the actual id-changing paths (wakes inserting bodies,
        // sleep extractions) bump the epoch themselves.
        if wake_up {
            for handle in [handle1, handle2].into_iter().flatten() {
                self.wake_up(bodies, handle, false);
            }
        }

        // Non-fixed enabled endpoints must be registered in the active set. (Two awake
        // touching bodies sharing an island is structural: there is at most one awake
        // island.)
        #[cfg(debug_assertions)]
        for handle in [handle1, handle2].into_iter().flatten() {
            if let Some(rb) = bodies.get(handle) {
                debug_assert!(
                    rb.is_fixed() || !rb.is_enabled() || rb.ids.active_island_id != u32::MAX
                );
            }
        }
    }

    pub(crate) fn island(&self, island_id: usize) -> &Island {
        &self.islands[island_id]
    }

    /// Applies a deferred impulse-joint island event, first restoring the invariant for
    /// `Link`: a sleeping island is woken before merging with an awake one. Two sleeping islands
    /// merge *without* waking; a fixed or missing endpoint doesn't disturb a sleeping island.
    pub(crate) fn apply_impulse_joint_island_event(
        &mut self,
        bodies: &mut RigidBodySet,
        event: crate::dynamics::ImpulseJointIslandEvent,
    ) {
        if let crate::dynamics::ImpulseJointIslandEvent::Link { body1, body2, .. } = event {
            self.wake_for_link(bodies, body1, body2);
        }
        self.persistent.apply_impulse_joint_event(bodies, event);
    }

    /// Refreshes a multibody's island-connectivity chain, first waking its
    /// sleeping members if any member is awake (a multibody is atomic: its
    /// bodies must share one sleep state).
    pub(crate) fn refresh_multibody_chain(
        &mut self,
        bodies: &mut RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        mb_id: crate::dynamics::MultibodyIndex,
    ) {
        if let Some(mb) = multibody_joints.get_multibody(mb_id) {
            let mut any_awake = false;
            let mut sleeping = Vec::new();
            for link in mb.links() {
                if let Some(rb) = bodies.get(link.rigid_body) {
                    if !rb.is_fixed() && rb.is_enabled() {
                        if rb.activation.sleeping {
                            sleeping.push(link.rigid_body);
                        } else {
                            any_awake = true;
                        }
                    }
                }
            }
            if any_awake {
                for handle in sleeping {
                    self.wake_up(bodies, handle, true);
                }
            }
        }
        self.persistent
            .refresh_multibody_chain(bodies, multibody_joints, mb_id);
    }

    /// Wakes the sleeping side of a new link when the other side is awake.
    fn wake_for_link(
        &mut self,
        bodies: &mut RigidBodySet,
        h1: RigidBodyHandle,
        h2: RigidBodyHandle,
    ) {
        let state = |bodies: &RigidBodySet, h: RigidBodyHandle| {
            bodies
                .get(h)
                .filter(|rb| !rb.is_fixed() && rb.is_enabled())
                .map(|rb| rb.activation.sleeping)
        };
        match (state(bodies, h1), state(bodies, h2)) {
            (Some(false), Some(true)) => self.wake_up(bodies, h2, true),
            (Some(true), Some(false)) => self.wake_up(bodies, h1, true),
            _ => {}
        }
    }

    /// The persistent island a body belongs to (`None` for fixed, disabled or removed bodies).
    /// Test/debug introspection only — island ids are unstable across steps (merges and splits
    /// recycle them); only *equality* between two bodies' islands is meaningful.
    #[doc(hidden)]
    pub fn persistent_island_of(
        &self,
        bodies: &RigidBodySet,
        handle: RigidBodyHandle,
    ) -> Option<u32> {
        self.persistent.body_island(bodies, handle)
    }

    /// Handles of dynamic and kinematic rigid-bodies that are currently active (i.e. not sleeping).
    #[inline]
    pub fn active_bodies(&self) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        self.awake_island
            .into_iter()
            .flat_map(|i| self.islands[i].bodies.iter().copied())
    }

    /// The awake island's body slice (same content and order as
    /// [`Self::active_bodies`]), for callers that want to chunk the active set in
    /// parallel.
    #[cfg(feature = "parallel")]
    #[inline]
    pub(crate) fn active_body_slices(&self) -> impl Iterator<Item = &[RigidBodyHandle]> {
        self.awake_island
            .into_iter()
            .map(|i| self.islands[i].bodies.as_slice())
    }

    /// Number of currently active (non-sleeping) dynamic and kinematic bodies.
    #[inline]
    pub fn num_active_bodies(&self) -> usize {
        self.awake_island
            .map(|i| self.islands[i].bodies.len())
            .unwrap_or(0)
    }

    pub(crate) fn rigid_body_updated(
        &mut self,
        handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) {
        self.bump_active_set_epoch();
        let Some(rb) = bodies.get_mut(handle) else {
            return;
        };

        if rb.is_fixed() {
            // A body turned fixed leaves the persistent islands (fixed bodies
            // are never island members).
            self.persistent.remove_body(bodies, handle);
            return;
        }

        // Check if this is the first time we see this rigid-body.
        if rb.ids.active_island_id == u32::MAX {
            if !rb.is_sleeping() {
                // Awake bodies all live in the single awake island.
                if let Some(id) = self.awake_island {
                    let target_island = &mut self.islands[id];
                    rb.ids.active_island_id = id as u32;
                    rb.ids.active_set_id = (target_island.bodies.len()) as u32;
                    target_island.bodies.push(handle);
                } else {
                    let new_island = Island::singleton(handle);
                    let id = self.free_islands.pop().unwrap_or(self.islands.len());
                    self.awake_island = Some(id);
                    self.islands.insert(id, new_island);
                    rb.ids.active_island_id = id as u32;
                    rb.ids.active_set_id = 0;
                }
            } else {
                // A body inserted asleep gets its own sleeping chunk.
                let new_island = Island::singleton(handle);
                let id = self.free_islands.pop().unwrap_or(self.islands.len());
                self.islands.insert(id, new_island);
                rb.ids.active_island_id = id as u32;
                rb.ids.active_set_id = 0;
            }
        }

        // Persistent islands: first-seen, re-enabled, or fixed-turned-dynamic
        // bodies get a singleton island (no-op for existing members).
        self.persistent.ensure_body(bodies, handle);
        let rb = bodies.index_mut_internal(handle);

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

    /// Updates a body's sleep-eligibility timer from its current velocities
    /// and last-step displacement.
    pub(crate) fn update_body_energy(rb: &mut RigidBody, dt: Real, length_unit: Real) {
        let sq_linvel = rb.vels.linvel.length_squared();
        let sq_angvel = rb.vels.angvel.gdot(rb.vels.angvel);
        let pose = rb.pos.position;
        rb.activation.update_energy(
            rb.body_type,
            length_unit,
            sq_linvel,
            sq_angvel,
            rb.mprops.max_extent(),
            &pose,
            dt,
        );
    }

    pub(crate) fn update_islands(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        narrow_phase: &mut NarrowPhase,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        sleep_observations: &[(u32, bool)],
    ) {
        // First update after construction or deserialization: rebuild the persistent islands
        // from the current graphs, and wake any sleeping body stranded in a mixed island
        // (deserialized partial-island-era state) to restore the whole-island invariant.
        if !self.persistent.bootstrapped {
            let to_wake = self.persistent.bootstrap(
                bodies,
                narrow_phase.touching_pairs_with_ids(colliders),
                impulse_joints,
                multibody_joints,
            );
            for handle in to_wake {
                self.wake_up(bodies, handle, false);
            }

            // `max_extent` (sleep metric) is only refreshed when colliders
            // change: seed it for deserialized snapshots that predate it.
            let handles: Vec<RigidBodyHandle> = bodies.iter().map(|(h, _)| h).collect();
            for handle in handles {
                let rb = bodies.index_mut_internal(handle);
                if rb.mprops.max_extent() == 0.0 {
                    rb.mprops.recompute_max_extent(colliders, &rb.colliders);
                }
            }
        }

        // Whole-island sleep decision: an island sleeps once *every* body has been
        // sleep-eligible long enough; one that lost constraints must split first (unless
        // single-body). Observations come from the pipeline's fused active-bodies traversal, so this never touches the body arena.
        let mut chunks: Vec<Vec<RigidBodyHandle>> = Vec::new();
        if !sleep_observations.is_empty() {
            self.persistent.begin_sleep_scan();
            for (island_id, eligible) in sleep_observations {
                self.persistent
                    .observe_body_for_sleep(*island_id, *eligible);
            }

            let sleepable = self.persistent.finish_sleep_scan();
            for id in sleepable {
                self.persistent.mark_island_sleeping(id);
                chunks.push(self.persistent.islands[id as usize].bodies.clone());
            }
        }

        if !chunks.is_empty() {
            let awake_id = self
                .awake_island
                .expect("sleep observations imply an awake island");
            let awake_len = self.islands[awake_id].len();
            self.commit_sleeping_chunks(bodies, narrow_phase, awake_id, awake_len, chunks);
        }

        // Persistent-island structural validation (debug builds only): index
        // consistency, plus "every touching pair with an island-member side is
        // linked, into that member's island".
        #[cfg(debug_assertions)]
        {
            self.persistent.assert_consistent(bodies);
            for (edge_id, h1, h2) in narrow_phase.touching_pairs_with_ids(colliders) {
                let member = |h: Option<RigidBodyHandle>| {
                    h.and_then(|h| bodies.get(h))
                        .filter(|rb| !rb.is_fixed() && rb.is_enabled())
                        .map(|rb| rb.ids.island_id)
                };
                let m1 = member(h1);
                let m2 = member(h2);
                if m1.is_some() || m2.is_some() {
                    let loc = self.persistent.contact_link_loc(edge_id);
                    assert!(
                        loc.is_some(),
                        "touching pair (edge {edge_id}) not linked in the persistent islands"
                    );
                    let island = loc.unwrap().0;
                    for m in [m1, m2].into_iter().flatten() {
                        assert_eq!(
                            m, island,
                            "touching pair (edge {edge_id}) linked into the wrong island"
                        );
                    }
                }
            }
        }
    }
}
