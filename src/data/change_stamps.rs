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

/// Slots per entry of [`ChangeStamps::blocks`], as a shift.
#[cfg(feature = "serde-serialize")]
const BLOCK_SHIFT: usize = 6;

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
    /// Per block of `1 << BLOCK_SHIFT` slots, a stamp no older than any of the block's, so
    /// [`Self::changed_since`] skips unchanged blocks.
    blocks: Vec<u32>,
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
}

#[cfg(feature = "serde-serialize")]
impl ChangeStamps {
    pub(crate) const fn new() -> Self {
        Self {
            now: 0,
            stamps: Vec::new(),
            blocks: Vec::new(),
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
            self.blocks = Vec::new();
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
            self.grow(i + 1);
        }
        self.stamps[i] = self.now;
        // The clock only moves forward, so the latest write is the block's newest stamp.
        self.blocks[i >> BLOCK_SHIFT] = self.now;
    }

    #[cold]
    fn grow(&mut self, len: usize) {
        self.stamps.resize(len, 0);
        self.blocks.resize(len.div_ceil(1 << BLOCK_SHIFT), 0);
    }

    /// Records that every slot of `range` was written at the current tick.
    pub(crate) fn mark_range(&mut self, range: core::ops::Range<usize>) {
        if self.now != 0 && !range.is_empty() {
            if range.end > self.stamps.len() {
                self.grow(range.end);
            }
            self.blocks[range.start >> BLOCK_SHIFT..=(range.end - 1) >> BLOCK_SHIFT].fill(self.now);
            self.stamps[range].fill(self.now);
        }
    }

    /// Mirrors a `swap_remove(i)` on a container that had `len_before` slots: the last slot
    /// moved into `i`, which is therefore rewritten.
    #[inline]
    pub(crate) fn swap_remove(&mut self, i: usize, len_before: usize) {
        if self.now != 0 {
            self.truncate(len_before - 1);
            if i < len_before - 1 {
                self.mark_enabled(i);
            }
        }
    }

    /// Mirrors a truncation of the container to `len` slots.
    #[inline]
    pub(crate) fn truncate(&mut self, len: usize) {
        self.stamps.truncate(len);
        self.blocks.truncate(len.div_ceil(1 << BLOCK_SHIFT));
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
                self.grow(i + 1);
            }
            self.stamps[i] = stamp;
            let block = &mut self.blocks[i >> BLOCK_SHIFT];
            if *block == 0 || stamp_is_newer(stamp, *block) {
                *block = stamp;
            }
        }
    }

    /// The slots written after `since`, in increasing order.
    pub(crate) fn changed_since(&self, since: u32) -> impl Iterator<Item = usize> + '_ {
        self.blocks
            .iter()
            .enumerate()
            .filter(move |(_, b)| stamp_is_newer(**b, since))
            .flat_map(move |(b, _)| {
                let start = b << BLOCK_SHIFT;
                let end = (start + (1 << BLOCK_SHIFT)).min(self.stamps.len());
                (start..end).filter(move |i| stamp_is_newer(self.stamps[*i], since))
            })
    }
}

#[cfg(all(test, feature = "serde-serialize"))]
mod test {
    use super::*;

    /// The slots `changed_since` must return: a full scan of the per-slot stamps.
    fn brute_force(stamps: &ChangeStamps, since: u32) -> Vec<usize> {
        (0..stamps.stamps.len())
            .filter(|i| stamp_is_newer(stamps.stamp(*i), since))
            .collect()
    }

    #[test]
    fn changed_since_skips_blocks_across_wraparound() {
        let mut stamps = ChangeStamps::new();
        // Start close to the wrap so the clock passes `u32::MAX` (and skips zero) midway.
        let mut now = u32::MAX - 300;
        stamps.set_now(now);
        let mut issued = Vec::new();
        let mut rng = 0x2545_f491_4f6c_dd1d_u64;
        let mut next = |bound: usize| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng % bound as u64) as usize
        };
        for _ in 0..600 {
            for _ in 0..next(6) {
                let len = stamps.stamps.len().max(1);
                match next(8) {
                    0 => stamps.mark_range(next(len)..next(len + 200)),
                    1 => stamps.truncate(next(len + 1)),
                    2 if stamps.stamps.len() > 1 => {
                        stamps.swap_remove(next(len), len);
                    }
                    3 => {
                        let old = issued.get(next(issued.len() + 1)).copied().unwrap_or(0);
                        stamps.set_stamp(next(len + 70), old);
                    }
                    _ => stamps.mark(next(len + 130)),
                }
            }
            issued.push(now);
            for since in issued.iter().rev().take(40).chain(issued.first()) {
                assert_eq!(
                    stamps.changed_since(*since).collect::<Vec<_>>(),
                    brute_force(&stamps, *since),
                    "since {since}, now {now}"
                );
            }
            now = now.wrapping_add(1).max(1);
            stamps.set_now(now);
        }
        assert!(issued.contains(&u32::MAX) && issued.contains(&1));
    }

    #[test]
    fn changed_since_skips_untouched_blocks() {
        let mut stamps = ChangeStamps::new();
        stamps.set_now(u32::MAX);
        stamps.mark_range(0..10_000);
        stamps.set_now(1);
        stamps.mark(5_000);
        stamps.mark(9_999);
        assert_eq!(
            stamps.changed_since(u32::MAX).collect::<Vec<_>>(),
            [5_000, 9_999]
        );
        // Only the two touched blocks are newer than the baseline.
        let newer = stamps
            .blocks
            .iter()
            .filter(|b| stamp_is_newer(**b, u32::MAX))
            .count();
        assert_eq!(newer, 2);
        assert_eq!(stamps.changed_since(1).count(), 0);
        assert_eq!(stamps.changed_since(u32::MAX - 1).count(), 10_000);
    }
}
