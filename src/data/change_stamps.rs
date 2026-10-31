//! Per-slot change stamps: the bookkeeping behind snapshot deltas.

#[cfg(feature = "serde-serialize")]
use crate::alloc_prelude::*;

/// Whether a slot stamped at `stamp` was written after the tick `since` (wrapping-aware; a
/// zero stamp means "never written since tracking started").
#[cfg(feature = "serde-serialize")]
#[inline]
pub(crate) fn stamp_is_newer(stamp: u32, since: u32) -> bool {
    stamp != 0 && (stamp.wrapping_sub(since) as i32) > 0
}

/// The tick at which each slot of an indexed container was last written.
///
/// Inert (one predictable branch per [`Self::mark`]) until change tracking is enabled on the
/// owning component, which sets a nonzero current tick.
#[cfg(feature = "serde-serialize")]
#[derive(Clone, Debug, Default)]
pub(crate) struct ChangeStamps {
    /// The tick written by [`Self::mark`]; zero while tracking is disabled.
    now: u32,
    /// Per-slot stamps; may be shorter than the container (missing slots are unstamped).
    stamps: Vec<u32>,
}

/// Without serde there are no snapshot deltas: the stamps are a zero-sized no-op.
#[cfg(not(feature = "serde-serialize"))]
#[derive(Clone, Debug, Default)]
pub(crate) struct ChangeStamps;

#[cfg(not(feature = "serde-serialize"))]
impl ChangeStamps {
    pub(crate) const fn new() -> Self {
        Self
    }

    #[inline(always)]
    pub(crate) fn stamp_now(&self) -> u32 {
        0
    }

    #[inline(always)]
    pub(crate) fn is_enabled(&self) -> bool {
        false
    }

    #[inline(always)]
    pub(crate) fn mark(&mut self, _: usize) {}

    #[inline(always)]
    pub(crate) fn mark_range(&mut self, _: core::ops::Range<usize>) {}

    #[inline(always)]
    pub(crate) fn swap_remove(&mut self, _: usize, _: usize) {}

    #[inline(always)]
    pub(crate) fn truncate(&mut self, _: usize) {}

    #[inline(always)]
    pub(crate) fn stamp(&self, _: usize) -> u32 {
        0
    }

    #[inline(always)]
    pub(crate) fn set_stamp(&mut self, _: usize, _: u32) {}
}

#[cfg(feature = "serde-serialize")]
impl ChangeStamps {
    pub(crate) const fn new() -> Self {
        Self {
            now: 0,
            stamps: Vec::new(),
        }
    }

    /// The tick written by [`Self::mark`] (zero while disabled).
    #[inline]
    pub(crate) fn stamp_now(&self) -> u32 {
        self.now
    }

    #[inline]
    pub(crate) fn is_enabled(&self) -> bool {
        self.now != 0
    }

    /// Sets the tick subsequent writes are stamped with; zero disables tracking and drops
    /// every stamp.
    pub(crate) fn set_now(&mut self, now: u32) {
        self.now = now;
        if now == 0 {
            self.stamps = Vec::new();
        }
    }

    /// Records that slot `i` was written at the current tick.
    #[inline(always)]
    pub(crate) fn mark(&mut self, i: usize) {
        if self.now != 0 {
            self.mark_enabled(i);
        }
    }

    #[inline]
    fn mark_enabled(&mut self, i: usize) {
        if i >= self.stamps.len() {
            self.stamps.resize(i + 1, 0);
        }
        self.stamps[i] = self.now;
    }

    /// Records that every slot of `range` was written at the current tick.
    pub(crate) fn mark_range(&mut self, range: core::ops::Range<usize>) {
        if self.now != 0 && !range.is_empty() {
            if range.end > self.stamps.len() {
                self.stamps.resize(range.end, 0);
            }
            self.stamps[range].fill(self.now);
        }
    }

    /// Mirrors a `swap_remove(i)` on a container that had `len_before` slots: the last slot
    /// moved into `i`, which is therefore rewritten.
    #[inline]
    pub(crate) fn swap_remove(&mut self, i: usize, len_before: usize) {
        if self.now != 0 {
            self.stamps.truncate(len_before - 1);
            if i < len_before - 1 {
                self.mark_enabled(i);
            }
        }
    }

    /// Mirrors a truncation of the container to `len` slots.
    #[inline]
    pub(crate) fn truncate(&mut self, len: usize) {
        self.stamps.truncate(len);
    }

    /// The tick slot `i` was last written at (zero if never since tracking started).
    #[inline]
    pub(crate) fn stamp(&self, i: usize) -> u32 {
        self.stamps.get(i).copied().unwrap_or(0)
    }

    /// Overwrites slot `i`'s stamp, to undo stamps of writes known to be value-neutral.
    #[inline]
    pub(crate) fn set_stamp(&mut self, i: usize, stamp: u32) {
        if self.now != 0 {
            if i >= self.stamps.len() {
                self.stamps.resize(i + 1, 0);
            }
            self.stamps[i] = stamp;
        }
    }

    /// The slots written after `since`, in increasing order.
    pub(crate) fn changed_since(&self, since: u32) -> impl Iterator<Item = usize> + '_ {
        self.stamps
            .iter()
            .enumerate()
            .filter(move |(_, s)| stamp_is_newer(**s, since))
            .map(|(i, _)| i)
    }
}
