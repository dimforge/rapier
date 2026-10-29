//! Snapshot deltas: serializing only what changed since an earlier tick.
//!
//! Every serializable physics component ([`RigidBodySet`](crate::dynamics::RigidBodySet),
//! [`ColliderSet`](crate::geometry::ColliderSet), [`NarrowPhase`](crate::geometry::NarrowPhase),
//! and so on) implements [`SnapshotDelta`]. With change tracking enabled, the component stamps
//! each storage slot it writes with its current tick; [`SnapshotDelta::delta_since`] then
//! gathers only the slots written after a baseline tick, and
//! [`SnapshotDelta::apply_delta`] patches them into a receiver holding that baseline.
//!
//! A receiver that applied every delta from a baseline is byte-identical to the sender once
//! serialized, and keeps simulating identically. [`PhysicsWorld`](crate::pipeline::PhysicsWorld)
//! bundles the deltas of its components.
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

    pub(crate) fn raw(self) -> RawTick {
        self.raw
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

/// A physics component that can serialize the changes made since an earlier state.
///
/// See the [module documentation](self) for the workflow.
pub trait SnapshotDelta: Sized {
    /// The component-specific content of a [`Delta`].
    #[doc(hidden)]
    type Payload: Serialize + DeserializeOwned + Clone;

    /// Starts stamping writes with change ticks. Restarting it invalidates every tick
    /// issued before.
    fn enable_change_tracking(&mut self);

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

    /// Patches a delta into this component, which must hold the delta's baseline state (or be
    /// in any state for a full delta). Returns the tick to acknowledge to the sender.
    fn apply_delta(&mut self, delta: Delta<Self>) -> Result<ChangeTick<Self>, DeltaError>;
}

/// A fresh, process-unique-ish identity for a change clock, so ticks of a restarted clock
/// (another process, or tracking re-enabled) are rejected instead of misread.
fn new_tracker_id() -> u32 {
    use core::hash::{BuildHasher, Hasher};
    use core::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);

    let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
    hasher.write_u32(COUNTER.fetch_add(1, Ordering::Relaxed));
    let id = hasher.finish();
    ((id >> 32) as u32 ^ id as u32).max(1)
}

/// A component's change clock, plus the sender tick its state was last patched to.
#[derive(Clone, Debug, Default)]
pub(crate) struct ChangeTracker {
    /// `(tracker id, current tick)`; `None` while tracking is disabled.
    clock: Option<(u32, u32)>,
    /// The tick of the last delta applied to this component.
    applied: Option<RawTick>,
}

impl ChangeTracker {
    /// The tick writes are currently stamped with (zero when disabled).
    pub(crate) fn now(&self) -> u32 {
        self.clock.map_or(0, |(_, now)| now)
    }

    pub(crate) fn is_enabled(&self) -> bool {
        self.clock.is_some()
    }

    /// Starts a fresh clock and returns its first tick.
    pub(crate) fn enable(&mut self) -> u32 {
        self.clock = Some((new_tracker_id(), 1));
        1
    }

    pub(crate) fn disable(&mut self) {
        self.clock = None;
    }

    /// Validates `since`, returns `(since tick, issued tick, next stamp tick)` and advances
    /// the clock past the issued tick.
    pub(crate) fn advance(
        &mut self,
        since: Option<RawTick>,
    ) -> Result<(Option<u32>, RawTick, u32), DeltaError> {
        let (tracker, now) = self.clock.ok_or(DeltaError::TrackingDisabled)?;
        let since = match since {
            None => None,
            Some(since) => {
                if since.tracker != tracker || stamp_is_newer(since.tick, now) {
                    return Err(DeltaError::UnknownBaseline);
                }
                Some(since.tick)
            }
        };
        let issued = RawTick { tracker, tick: now };
        let next = now.wrapping_add(1).max(1);
        self.clock = Some((tracker, next));
        Ok((since, issued, next))
    }

    /// Checks that a delta can be applied, and records its target tick once it is.
    pub(crate) fn check_apply(&self, from: Option<RawTick>) -> Result<(), DeltaError> {
        match (from, self.applied) {
            (Some(from), Some(applied)) if from != applied => Err(DeltaError::BaselineMismatch),
            _ => Ok(()),
        }
    }

    pub(crate) fn set_applied(&mut self, to: RawTick) {
        self.applied = Some(to);
    }
}

/// Implements [`SnapshotDelta`] for a component providing:
/// - `fn change_tracker(&mut self) -> &mut ChangeTracker` and
///   `fn change_tracker_ref(&self) -> &ChangeTracker`;
/// - `fn set_stamp_tick(&mut self, now: u32)`, propagating the stamp tick to every
///   [`ChangeStamps`] it owns (zero disables);
/// - `fn collect_delta(&mut self, since: Option<u32>) -> Payload`;
/// - `fn apply_delta_payload(&mut self, payload: Payload) -> Result<(), DeltaError>`.
macro_rules! impl_snapshot_delta {
    ($component: ty, $payload: ty) => {
        impl $crate::data::delta::SnapshotDelta for $component {
            type Payload = $payload;

            fn enable_change_tracking(&mut self) {
                let now = self.change_tracker().enable();
                self.set_stamp_tick(now);
            }

            fn disable_change_tracking(&mut self) {
                self.change_tracker().disable();
                self.set_stamp_tick(0);
            }

            fn is_change_tracking_enabled(&self) -> bool {
                self.change_tracker_ref().is_enabled()
            }

            fn delta_since(
                &mut self,
                since: Option<$crate::data::delta::ChangeTick<Self>>,
            ) -> Result<
                (
                    $crate::data::delta::Delta<Self>,
                    $crate::data::delta::ChangeTick<Self>,
                ),
                $crate::data::delta::DeltaError,
            > {
                let (since_tick, issued, next) =
                    self.change_tracker().advance(since.map(|t| t.raw()))?;
                let payload = self.collect_delta(since_tick);
                self.set_stamp_tick(next);
                Ok((
                    $crate::data::delta::Delta {
                        from: since.map(|t| t.raw()),
                        to: issued,
                        payload,
                    },
                    $crate::data::delta::ChangeTick::from_raw(issued),
                ))
            }

            fn apply_delta(
                &mut self,
                delta: $crate::data::delta::Delta<Self>,
            ) -> Result<$crate::data::delta::ChangeTick<Self>, $crate::data::delta::DeltaError>
            {
                self.change_tracker_ref().check_apply(delta.from)?;
                self.apply_delta_payload(delta.payload)?;
                self.change_tracker().set_applied(delta.to);
                Ok($crate::data::delta::ChangeTick::from_raw(delta.to))
            }
        }
    };
}
pub(crate) use impl_snapshot_delta;

/// The stamped slots of an index-addressed vector: its new length, and the content of every
/// slot written after the baseline (every slot for a full delta).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct VecDelta<T> {
    pub(crate) len: u32,
    pub(crate) slots: Vec<(u32, T)>,
}

impl<T> VecDelta<T> {
    /// Gathers the slots of `items` written after `since`, transformed by `f`.
    pub(crate) fn collect_with<U>(
        items: &[U],
        stamps: &ChangeStamps,
        since: Option<u32>,
        mut f: impl FnMut(&U) -> T,
    ) -> Self {
        let slots = match since {
            None => items
                .iter()
                .enumerate()
                .map(|(i, item)| (i as u32, f(item)))
                .collect(),
            Some(since) => stamps
                .changed_since(since)
                .take_while(|i| *i < items.len())
                .map(|i| (i as u32, f(&items[i])))
                .collect(),
        };
        Self {
            len: items.len() as u32,
            slots,
        }
    }

    /// Patches this delta into `items`, transforming each slot with `f`. Slots appended past the
    /// current length must all be provided. Patched slots are stamped.
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
            let mut covered = vec![false; new_len - old_len];
            for (i, _) in &self.slots {
                if let Some(c) = (*i as usize).checked_sub(old_len).and_then(|i| covered.get_mut(i))
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
        Self::collect_with(items, stamps, since, T::clone)
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
