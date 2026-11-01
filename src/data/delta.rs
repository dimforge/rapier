//! Snapshot deltas: serializing only what changed since an earlier tick.
//!
//! Every serializable physics component ([`RigidBodySet`](crate::dynamics::RigidBodySet),
//! [`ColliderSet`](crate::geometry::ColliderSet), [`NarrowPhase`](crate::geometry::NarrowPhase),
//! and so on) implements [`SnapshotDelta`]. With change tracking enabled, the component stamps
//! each storage slot it writes with its current tick; [`SnapshotDelta::delta_since`] then
//! gathers only the slots written after a baseline tick, and
//! [`SnapshotDelta::apply_delta`] patches them into a receiver holding that baseline.
//!
//! A receiver that applied every delta from a baseline keeps simulating identically, and is
//! byte-identical to the sender once serialized (see the limitations).
//! [`PhysicsWorld`](crate::pipeline::PhysicsWorld) bundles the deltas of its components.
//!
//! A delta's size follows what changed since the baseline: about the awake bodies, their
//! colliders and contacts, plus a few hundred bytes of per-component headers. Heavy shapes are
//! referenced by content and only sent when the receiver cannot hold them yet.
//!
//! # Baselines
//!
//! An incremental delta applies only to its baseline state: the target of the last delta the
//! receiver applied, or a tick declared with [`SnapshotDelta::set_delta_baseline`] (after
//! restoring a snapshot the sender took right after [`SnapshotDelta::change_tick`]). Stepping
//! the receiver, or cloning the sender, invalidates baselines; a full delta always applies. Other
//! local edits of a receiver are not detected: they also require a full delta.
//!
//! # Limitations
//!
//! - Without the `enhanced-determinism` feature, parry serializes the contact workspaces of
//!   composite shapes (compounds, soft-body surfaces, voxels) in hash-map order, so a receiver
//!   may serialize them in another order than the sender (with identical content).
//! - Applying a broad-phase delta that changes leaves rebuilds the receiver's tree, like a
//!   deserialization (O(n log n)).
//! - Encoding clones the changed slots: about a full serialization when everything is awake.
//! - Ticks wrap after 2^32 issued ticks (every `delta_since` and `change_tick` issues one: four
//!   days at 60 Hz for 100 receivers): a slot not rewritten for 2^31 ticks is then resent by
//!   every delta until it is rewritten.
//!
//! # Example
//!
//! ```
//! # #[cfg(feature = "serde-serialize")] {
//! # use rapier3d::prelude::*;
//! # use rapier3d::data::SnapshotDelta;
//! let mut server = RigidBodySet::new();
//! server.enable_change_tracking();
//! server.insert(RigidBodyBuilder::dynamic());
//!
//! // Bootstrap a client with the full state, as a delta from nothing.
//! let (full, mut acked) = server.delta_since(None).unwrap();
//! let mut client = RigidBodySet::new();
//! client.apply_delta(full).unwrap();
//!
//! // Later: only what changed since the acknowledged tick.
//! server.insert(RigidBodyBuilder::fixed());
//! let (delta, tick) = server.delta_since(Some(acked)).unwrap();
//! client.apply_delta(delta).unwrap();
//! acked = tick;
//! # let _ = acked;
//! assert_eq!(client.len(), 2);
//! # }
//! ```

use crate::alloc_prelude::*;
use crate::data::change_stamps::{ChangeStamps, stamp_is_newer};
use core::fmt;
use core::marker::PhantomData;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// A tick issued by a component's change clock, with the clock's identity.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub(crate) struct RawTick {
    pub(crate) tracker: u32,
    pub(crate) tick: u32,
}

/// A point in one component's change history, as returned by
/// [`SnapshotDelta::delta_since`] and [`SnapshotDelta::apply_delta`].
///
/// It is typed by the component that issued it, so a tick of one component cannot be passed
/// to another. It is serializable: receivers send it back to acknowledge a state.
#[derive(Serialize, Deserialize)]
#[serde(transparent)]
pub struct ChangeTick<C> {
    raw: RawTick,
    #[serde(skip)]
    _component: PhantomData<fn() -> C>,
}

impl<C> ChangeTick<C> {
    pub(crate) fn from_raw(raw: RawTick) -> Self {
        Self {
            raw,
            _component: PhantomData,
        }
    }
}

impl<C> Copy for ChangeTick<C> {}
impl<C> Clone for ChangeTick<C> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<C> PartialEq for ChangeTick<C> {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}
impl<C> Eq for ChangeTick<C> {}
impl<C> core::hash::Hash for ChangeTick<C> {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.raw.hash(state)
    }
}
impl<C> fmt::Debug for ChangeTick<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChangeTick")
            .field("tracker", &self.raw.tracker)
            .field("tick", &self.raw.tick)
            .finish()
    }
}

/// Errors of [`SnapshotDelta`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum DeltaError {
    /// [`SnapshotDelta::delta_since`] was called while change tracking is disabled.
    #[error("change tracking is not enabled on this component")]
    TrackingDisabled,
    /// The baseline tick was not issued by this component's current change clock (the clock
    /// was restarted, or the tick comes from another component): send a full state instead
    /// (`delta_since(None)`).
    #[error("the baseline tick was not issued by this change tracker; send a full state")]
    UnknownBaseline,
    /// The delta was produced against a baseline other than the state it is applied to.
    #[error("the delta's baseline does not match the state it is applied to")]
    BaselineMismatch,
    /// The delta references storage it does not provide, or is otherwise inconsistent.
    #[error("malformed snapshot delta")]
    Malformed,
    /// A collider references a shape the receiver does not hold.
    #[error("the delta references a shape the receiver does not hold")]
    MissingShape,
}

/// A serialized change set of the component `C`, produced by [`SnapshotDelta::delta_since`].
///
/// It is a serde type: serialize it with any format, then apply it on the receiver with
/// [`SnapshotDelta::apply_delta`].
#[derive(Serialize, Deserialize)]
#[serde(bound(
    serialize = "C::Payload: Serialize",
    deserialize = "C::Payload: DeserializeOwned"
))]
pub struct Delta<C: SnapshotDelta> {
    pub(crate) from: Option<RawTick>,
    pub(crate) to: RawTick,
    pub(crate) payload: C::Payload,
}

impl<C: SnapshotDelta> Clone for Delta<C> {
    fn clone(&self) -> Self {
        Self {
            from: self.from,
            to: self.to,
            payload: self.payload.clone(),
        }
    }
}

impl<C: SnapshotDelta> fmt::Debug for Delta<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Delta")
            .field("from", &self.from)
            .field("to", &self.to)
            .finish_non_exhaustive()
    }
}

impl<C: SnapshotDelta> Delta<C> {
    /// Whether this delta carries the component's whole state (produced with no baseline).
    pub fn is_full(&self) -> bool {
        self.from.is_none()
    }

    /// The tick a receiver is at once this delta is applied.
    pub fn target_tick(&self) -> ChangeTick<C> {
        ChangeTick::from_raw(self.to)
    }
}

/// Crate-internal items that public signatures name (declared `pub` in a private module, so
/// they cannot be reached from outside the crate).
mod internal {
    use super::*;

    pub trait Sealed {}

    /// The per-component hooks behind [`SnapshotDelta`].
    pub trait DeltaComponent: Default {
        type Payload: Serialize + DeserializeOwned + Clone;

        fn change_tracker(&self) -> &ChangeTracker;

        fn change_tracker_mut(&mut self) -> &mut ChangeTracker;

        /// Propagates the stamp tick to every [`ChangeStamps`] the component owns (zero disables).
        fn set_stamp_tick(&mut self, now: u32);

        /// The slots written after `since` (all of them for `None`).
        fn collect_delta(&mut self, since: Option<u32>) -> Self::Payload;

        fn apply_delta_payload(&mut self, payload: Self::Payload) -> Result<(), DeltaError>;

        /// Brings up to date the bookkeeping that deltas from an issued tick rely on.
        fn sync_before_tick(&mut self) {}

        /// Drops the receiver-side state only incremental deltas need, once the component
        /// diverged from its baseline (only a full delta, which resets it, applies then).
        fn forget_baseline_state(&mut self) {}

        /// Resets to the state a deserialization gives, before a full delta is applied (the change
        /// tracker is restored afterwards).
        fn reset_for_full_delta(&mut self) {
            *self = Self::default();
        }
    }

    /// A fresh identity for a change clock, so ticks of a restarted clock (another process, or
    /// tracking re-enabled) are rejected instead of misread.
    fn new_tracker_id(seed: Option<u64>) -> u32 {
        use core::hash::{BuildHasher, Hasher};
        use core::sync::atomic::{AtomicU32, Ordering};
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        let count = COUNTER.fetch_add(1, Ordering::Relaxed);
        let id = match seed {
            // SplitMix64 finalizer of the seed and counter.
            Some(seed) => {
                let mut z = seed ^ (count as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                z ^ (z >> 31)
            }
            None => {
                let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
                hasher.write_u32(count);
                hasher.finish()
            }
        };
        ((id >> 32) as u32 ^ id as u32).max(1)
    }

    /// A component's change clock, plus the sender tick its state was last patched to.
    ///
    /// Ticks wrap after 2^32 issued ticks (see the module limitations).
    #[derive(Debug)]
    pub struct ChangeTracker {
        /// `(tracker id, current tick)`; `None` while tracking is disabled.
        clock: Option<(u32, u32)>,
        /// The sender tick this component's state matches: the last delta applied, or the baseline
        /// set explicitly. Cleared when the component is stepped.
        applied: Option<RawTick>,
        /// A process-unique id renewed whenever a delta replaces state behind the pipeline's back,
        /// so the pipeline can drop caches derived from the previous state.
        state_id: u32,
    }

    /// A fresh [`ChangeTracker::state_id`].
    fn new_state_id() -> u32 {
        use core::sync::atomic::{AtomicU32, Ordering};
        static NEXT: AtomicU32 = AtomicU32::new(1);
        NEXT.fetch_add(1, Ordering::Relaxed)
    }

    impl Default for ChangeTracker {
        fn default() -> Self {
            Self {
                clock: None,
                applied: None,
                state_id: new_state_id(),
            }
        }
    }

    impl Clone for ChangeTracker {
        /// A clone gets its own clock identity: two copies issuing the same ticks for diverging
        /// states would accept each other's baselines.
        fn clone(&self) -> Self {
            let mut clone = Self {
                clock: self.clock,
                applied: self.applied,
                state_id: new_state_id(),
            };
            clone.restart();
            clone
        }
    }

    impl ChangeTracker {
        /// See [`Self::state_id`](ChangeTracker#structfield.state_id).
        pub(crate) fn state_id(&self) -> u32 {
            self.state_id
        }

        pub(crate) fn renew_state_id(&mut self) {
            self.state_id = new_state_id();
        }

        pub(crate) fn is_enabled(&self) -> bool {
            self.clock.is_some()
        }

        /// Starts a fresh clock and returns its first tick.
        pub(crate) fn enable(&mut self, seed: Option<u64>) -> u32 {
            self.clock = Some((new_tracker_id(seed), 1));
            1
        }

        pub(crate) fn disable(&mut self) {
            self.clock = None;
        }

        /// Gives a running clock a fresh identity, keeping its tick. Returns the stamp tick (zero
        /// while disabled).
        pub(crate) fn restart(&mut self) -> u32 {
            match &mut self.clock {
                Some((tracker, now)) => {
                    // Derived from the previous identity, so a seeded clock stays seeded.
                    *tracker = new_tracker_id(Some(*tracker as u64));
                    *now
                }
                None => 0,
            }
        }

        /// Checks that tracking is on and `since` (if any) is a tick this clock issued.
        pub(crate) fn check_since(&self, since: Option<RawTick>) -> Result<(), DeltaError> {
            let (tracker, now) = self.clock.ok_or(DeltaError::TrackingDisabled)?;
            match since {
                // The current tick has not been issued yet.
                Some(since)
                    if since.tracker != tracker
                        || since.tick == now
                        || stamp_is_newer(since.tick, now) =>
                {
                    Err(DeltaError::UnknownBaseline)
                }
                _ => Ok(()),
            }
        }

        /// Validates `since`, returns `(since tick, issued tick, next stamp tick)` and advances
        /// the clock past the issued tick.
        pub(crate) fn advance(
            &mut self,
            since: Option<RawTick>,
        ) -> Result<(Option<u32>, RawTick, u32), DeltaError> {
            self.check_since(since)?;
            let (tracker, now) = self.clock.ok_or(DeltaError::TrackingDisabled)?;
            let since = since.map(|since| since.tick);
            let issued = RawTick { tracker, tick: now };
            let next = now.wrapping_add(1).max(1);
            self.clock = Some((tracker, next));
            Ok((since, issued, next))
        }

        /// Checks that a delta with baseline `from` (`None`: full) can be applied.
        pub(crate) fn check_apply(&self, from: Option<RawTick>) -> Result<(), DeltaError> {
            match from {
                Some(from) if self.applied != Some(from) => Err(DeltaError::BaselineMismatch),
                _ => Ok(()),
            }
        }

        pub(crate) fn set_applied(&mut self, to: RawTick) {
            self.applied = Some(to);
        }

        /// Forgets the sender baseline, once the state diverged from it (the component was stepped).
        pub(crate) fn clear_applied(&mut self) {
            self.applied = None;
        }
    }
}

pub(crate) use internal::{ChangeTracker, DeltaComponent};

/// Checks that `component` can produce a delta from `since`, without producing it.
pub(crate) fn check_delta_since<C: DeltaComponent>(
    component: &C,
    since: Option<ChangeTick<C>>,
) -> Result<(), DeltaError> {
    component.change_tracker().check_since(since.map(|t| t.raw))
}

/// Checks that `delta` can be applied to `component`, without applying it.
pub(crate) fn check_delta_baseline<C: DeltaComponent>(
    component: &C,
    delta: &Delta<C>,
) -> Result<(), DeltaError> {
    component.change_tracker().check_apply(delta.from)
}

/// The id of `component`'s current state (see `ChangeTracker`), renewed by every applied delta.
pub(crate) fn delta_state_id(component: &impl DeltaComponent) -> u32 {
    component.change_tracker().state_id()
}

/// Forgets the sender state `component` mirrored, once it diverged from it (it was stepped).
pub(crate) fn clear_delta_baseline(component: &mut impl DeltaComponent) {
    component.change_tracker_mut().clear_applied();
    component.forget_baseline_state();
}

/// A physics component that can serialize the changes made since an earlier state.
///
/// Implemented by rapier's components only. See the [module documentation](self) for the
/// workflow.
pub trait SnapshotDelta: internal::Sealed + Sized {
    /// The component-specific content of a [`Delta`].
    #[doc(hidden)]
    type Payload: Serialize + DeserializeOwned + Clone;

    /// Starts stamping writes with change ticks, under a fresh clock identity. Restarting it
    /// invalidates every tick issued before.
    ///
    /// The identity comes from the standard library's random hasher keys, which repeat across
    /// runs on targets without OS randomness (`wasm32-unknown-unknown`): use
    /// [`Self::enable_change_tracking_with_seed`] there.
    fn enable_change_tracking(&mut self);

    /// Like [`Self::enable_change_tracking`], deriving the clock identity from `seed` (pass a
    /// random value that differs between runs).
    fn enable_change_tracking_with_seed(&mut self, seed: u64);

    /// Stops stamping writes and drops the stamps.
    fn disable_change_tracking(&mut self);

    /// Whether writes are currently stamped.
    fn is_change_tracking_enabled(&self) -> bool;

    /// The changes made since `since` (`None`: the whole state), and the tick a receiver is
    /// at once it applied them. Requires change tracking.
    fn delta_since(
        &mut self,
        since: Option<ChangeTick<Self>>,
    ) -> Result<(Delta<Self>, ChangeTick<Self>), DeltaError>;

    /// Issues a tick naming the current state without building a delta, e.g. to send a full
    /// snapshot instead: the receiver restores it, then calls [`Self::set_delta_baseline`].
    fn change_tick(&mut self) -> Result<ChangeTick<Self>, DeltaError>;

    /// Declares that this component holds the sender's state at `tick`, so deltas from that
    /// tick can be applied (e.g. after restoring a snapshot taken right after
    /// [`Self::change_tick`]).
    fn set_delta_baseline(&mut self, tick: ChangeTick<Self>);

    /// Patches a delta into this component. An incremental delta requires the delta's
    /// baseline state: the target of the last delta applied (or [`Self::set_delta_baseline`]),
    /// not stepped since. A full delta first resets the component, as a deserialization would.
    /// Returns the tick to acknowledge to the sender.
    ///
    /// On error, the component may be left partially patched: apply a full delta to recover.
    fn apply_delta(&mut self, delta: Delta<Self>) -> Result<ChangeTick<Self>, DeltaError>;
}

impl<T: DeltaComponent> internal::Sealed for T {}

impl<T: DeltaComponent> SnapshotDelta for T {
    type Payload = T::Payload;

    fn enable_change_tracking(&mut self) {
        // Drop the bookkeeping of a previous clock first.
        self.set_stamp_tick(0);
        let now = self.change_tracker_mut().enable(None);
        self.set_stamp_tick(now);
    }

    fn enable_change_tracking_with_seed(&mut self, seed: u64) {
        self.set_stamp_tick(0);
        let now = self.change_tracker_mut().enable(Some(seed));
        self.set_stamp_tick(now);
    }

    fn disable_change_tracking(&mut self) {
        self.change_tracker_mut().disable();
        self.set_stamp_tick(0);
    }

    fn is_change_tracking_enabled(&self) -> bool {
        self.change_tracker().is_enabled()
    }

    fn delta_since(
        &mut self,
        since: Option<ChangeTick<Self>>,
    ) -> Result<(Delta<Self>, ChangeTick<Self>), DeltaError> {
        let (since_tick, issued, next) = self.change_tracker_mut().advance(since.map(|t| t.raw))?;
        self.sync_before_tick();
        let payload = self.collect_delta(since_tick);
        self.set_stamp_tick(next);
        Ok((
            Delta {
                from: since.map(|t| t.raw),
                to: issued,
                payload,
            },
            ChangeTick::from_raw(issued),
        ))
    }

    fn change_tick(&mut self) -> Result<ChangeTick<Self>, DeltaError> {
        let (_, issued, next) = self.change_tracker_mut().advance(None)?;
        self.sync_before_tick();
        self.set_stamp_tick(next);
        Ok(ChangeTick::from_raw(issued))
    }

    fn set_delta_baseline(&mut self, tick: ChangeTick<Self>) {
        self.change_tracker_mut().set_applied(tick.raw);
    }

    fn apply_delta(&mut self, delta: Delta<Self>) -> Result<ChangeTick<Self>, DeltaError> {
        self.change_tracker().check_apply(delta.from)?;
        // Until the patch succeeds, the state matches no baseline, and pipeline caches are stale.
        let tracker = self.change_tracker_mut();
        tracker.clear_applied();
        tracker.renew_state_id();
        if delta.from.is_none() {
            let mut tracker = core::mem::take(self.change_tracker_mut());
            self.reset_for_full_delta();
            // Downstream receivers' baselines predate the reset: reject them.
            let now = tracker.restart();
            *self.change_tracker_mut() = tracker;
            self.set_stamp_tick(now);
        }
        self.apply_delta_payload(delta.payload)?;
        self.change_tracker_mut().set_applied(delta.to);
        Ok(ChangeTick::from_raw(delta.to))
    }
}

/// Debug check of the passes writing arena slots without stamping them: with
/// `RAPIER_DBG_VALIDATE` set, it hashes every slot before the pass, and [`Self::check`] panics
/// if a slot left unstamped changed.
#[cfg(debug_assertions)]
pub(crate) struct UnstampedWriteCheck(Option<Vec<Option<crate::utils::content_hash::ContentKey>>>);

#[cfg(debug_assertions)]
impl UnstampedWriteCheck {
    pub(crate) fn new<T: Serialize>(arena: &crate::data::Arena<T>) -> Self {
        let enabled =
            arena.stamps.is_enabled() && std::env::var_os("RAPIER_DBG_VALIDATE").is_some();
        Self(enabled.then(|| {
            let (entries, ..) = arena.raw_entries();
            entries
                .iter()
                .map(crate::utils::content_hash::content_key)
                .collect()
        }))
    }

    pub(crate) fn check<T: Serialize>(self, arena: &crate::data::Arena<T>, pass: &str) {
        let Some(before) = self.0 else {
            return;
        };
        let now = arena.stamps.stamp_now();
        let (entries, ..) = arena.raw_entries();
        for (i, (entry, before)) in entries.iter().zip(&before).enumerate() {
            assert!(
                arena.stamps.stamp(i) == now
                    || crate::utils::content_hash::content_key(entry) == *before,
                "{pass}: slot {i} changed without being stamped for snapshot deltas"
            );
        }
    }
}

/// The storage slots whose derived data a deserialization or a delta left stale, so it is
/// rebuilt only where needed.
#[derive(Clone, Debug, Default)]
pub(crate) enum StaleSlots {
    /// Nothing to rebuild.
    #[default]
    None,
    /// Every slot (after a deserialization).
    All,
    /// These slots (after deltas).
    Some(Vec<u32>),
}

impl StaleSlots {
    pub(crate) fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    /// Slots were swap-removed, so the listed indices may now name other slots.
    pub(crate) fn slots_moved(&mut self) {
        if matches!(self, Self::Some(_)) {
            *self = Self::All;
        }
    }

    /// Adds `slots` to the stale set of a storage of `capacity` slots (beyond which listing
    /// them costs more than rebuilding them all).
    pub(crate) fn extend(&mut self, slots: impl Iterator<Item = u32>, capacity: usize) {
        match self {
            Self::All => {}
            Self::None => *self = Self::Some(slots.collect()),
            Self::Some(list) => list.extend(slots),
        }
        if matches!(self, Self::Some(list) if list.len() > capacity) {
            *self = Self::All;
        }
    }
}

/// The stamped slots of an index-addressed vector: its new length, and the content of every
/// slot written after the baseline (every slot for a full delta).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct VecDelta<T> {
    pub(crate) len: u32,
    pub(crate) slots: Vec<(u32, T)>,
}

impl<T> VecDelta<T> {
    /// The indices of the slots this delta writes.
    pub(crate) fn slot_indices(&self) -> impl Iterator<Item = u32> + '_ {
        self.slots.iter().map(|(i, _)| *i)
    }

    /// Gathers the slots of `items` written after `since`, transformed by `f`.
    pub(crate) fn collect_with<U>(
        items: &[U],
        stamps: &ChangeStamps,
        since: Option<u32>,
        mut f: impl FnMut(usize, &U) -> T,
    ) -> Self {
        let slots = match since {
            None => items
                .iter()
                .enumerate()
                .map(|(i, item)| (i as u32, f(i, item)))
                .collect(),
            Some(since) => stamps
                .changed_since(since)
                .take_while(|i| *i < items.len())
                .map(|i| (i as u32, f(i, &items[i])))
                .collect(),
        };
        Self {
            len: items.len() as u32,
            slots,
        }
    }

    /// Patches this delta into `items` through `f`, stamping the patched slots (slots appended
    /// past the current length must all be provided).
    pub(crate) fn apply_with<U>(
        self,
        items: &mut Vec<U>,
        stamps: &mut ChangeStamps,
        mut placeholder: impl FnMut() -> U,
        mut f: impl FnMut(T) -> Result<U, DeltaError>,
    ) -> Result<(), DeltaError> {
        let new_len = self.len as usize;
        let old_len = items.len();
        if new_len > old_len {
            // Each appended slot must be provided: reject a bogus length before allocating.
            if self.slots.len() < new_len - old_len {
                return Err(DeltaError::Malformed);
            }
            let mut covered = vec![false; new_len - old_len];
            for (i, _) in &self.slots {
                if let Some(c) = (*i as usize)
                    .checked_sub(old_len)
                    .and_then(|i| covered.get_mut(i))
                {
                    *c = true;
                }
            }
            if covered.iter().any(|c| !*c) {
                return Err(DeltaError::Malformed);
            }
            items.resize_with(new_len, &mut placeholder);
        } else {
            items.truncate(new_len);
            stamps.truncate(new_len);
        }
        for (i, value) in self.slots {
            let slot = items.get_mut(i as usize).ok_or(DeltaError::Malformed)?;
            *slot = f(value)?;
            stamps.mark(i as usize);
        }
        Ok(())
    }
}

impl<T: Clone> VecDelta<T> {
    pub(crate) fn collect(items: &[T], stamps: &ChangeStamps, since: Option<u32>) -> Self {
        Self::collect_with(items, stamps, since, |_, item| item.clone())
    }

    pub(crate) fn apply(
        self,
        items: &mut Vec<T>,
        stamps: &mut ChangeStamps,
        placeholder: impl FnMut() -> T,
    ) -> Result<(), DeltaError> {
        self.apply_with(items, stamps, placeholder, Ok)
    }
}

/// Change stamps of a plain vector, derived by comparing it with a copy of its content at the
/// previous delta: for small slots written from many places (or through raw pointers in
/// parallel), where stamping every write would be fragile.
#[derive(Clone, Debug)]
pub(crate) struct ShadowStamps<T> {
    shadow: Vec<T>,
    pub(crate) stamps: ChangeStamps,
}

impl<T> Default for ShadowStamps<T> {
    fn default() -> Self {
        Self {
            shadow: Vec::new(),
            stamps: ChangeStamps::new(),
        }
    }
}

impl<T: Clone + PartialEq> ShadowStamps<T> {
    /// Stamps the slots of `current` that differ from the shadow, then updates the shadow.
    pub(crate) fn sync(&mut self, current: &[T]) {
        let common = self.shadow.len().min(current.len());
        for i in 0..common {
            if self.shadow[i] != current[i] {
                self.stamps.mark(i);
                self.shadow[i] = current[i].clone();
            }
        }
        self.shadow.truncate(current.len());
        self.stamps.truncate(current.len());
        if current.len() > common {
            self.stamps.mark_range(common..current.len());
            self.shadow.extend_from_slice(&current[common..]);
        }
    }

    /// The slots of `current` changed since `since`, once [`Self::sync`] ran for this tick.
    pub(crate) fn collect(&self, current: &[T], since: Option<u32>) -> VecDelta<T> {
        VecDelta::collect(current, &self.stamps, since)
    }

    /// Patches `delta` into `items`, stamping the patched slots.
    pub(crate) fn apply(
        &mut self,
        delta: VecDelta<T>,
        items: &mut Vec<T>,
        placeholder: impl FnMut() -> T,
    ) -> Result<(), DeltaError> {
        delta.apply(items, &mut self.stamps, placeholder)
    }
}
