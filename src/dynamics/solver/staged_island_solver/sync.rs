//! Completion-gated stage synchronization: per-worker claim cursors with
//! stealing, straggler fast-forward, and spin-free stage publication.

use crate::alloc_prelude::*;
use core::ops::Range;
use core::sync::atomic::{AtomicUsize, Ordering};

/// The packed cursor's integer type: 64 bits wherever the target has 64-bit atomics, which
/// is every target the parallel solver can run on (`parallel` implies `std`). The small
/// no-`std` targets that lack them fall back to 32 bits — those are always single-worker,
/// so the halved stage/position range below is out of reach in practice.
#[cfg(target_has_atomic = "64")]
type CursorInt = u64;
#[cfg(target_has_atomic = "64")]
type AtomicCursor = core::sync::atomic::AtomicU64;
#[cfg(not(target_has_atomic = "64"))]
type CursorInt = u32;
#[cfg(not(target_has_atomic = "64"))]
type AtomicCursor = core::sync::atomic::AtomicU32;

/// Bit position of the stage ordinal in a packed cursor; the claim position takes the low half.
/// A step builds a fresh [`StageSync`] and every worker restarts at stage 0, so the stage half
/// only has to hold one step's stage count.
const STAGE_SHIFT: u32 = CursorInt::BITS / 2;
/// Mask of a packed cursor's position half.
const POS_MASK: CursorInt = (1 << STAGE_SHIFT) - 1;

/// An atomic cursor on its own cache line (no false sharing between per-worker claim cursors).
/// Packed `(stage_ordinal << STAGE_SHIFT) | position` so a straggler's stale claim fails its CAS
/// instead of corrupting a cursor already reset for a later stage.
#[repr(align(64))]
struct PaddedCursor(AtomicCursor);

/// Completion-gated stage sync with per-worker claim cursors: each worker drains its own static
/// sub-slice of every stage (for cache affinity), then steals. A stage advances when all its
/// **work units** completed, not when all workers arrive: a preempted worker's
/// share gets stolen and it fast-forwards on wake (stale claims fail the stage tag) — exposure
/// is one claimed batch, not a barrier residency, which keeps step times spike-free.
pub(super) struct StageSync {
    pub(super) num_workers: usize,
    /// The published current stage ordinal. Workers wait on this; claims are
    /// validated against it through the cursors' stage tags.
    published: AtomicUsize,
    /// Advance ticket, CAS'd `stage -> stage + 1` by the single worker that
    /// performs the advance (cursor + counter reset, then publish).
    advance_ticket: AtomicUsize,
    /// Work units of the published stage completed so far. The stage advances
    /// when this reaches the stage's total (every unit claimed *and* executed).
    completed: AtomicUsize,
    cursors: Vec<PaddedCursor>,
}

impl StageSync {
    pub(super) fn new(num_workers: usize) -> Self {
        Self {
            num_workers,
            published: AtomicUsize::new(0),
            advance_ticket: AtomicUsize::new(0),
            completed: AtomicUsize::new(0),
            cursors: (0..num_workers)
                .map(|_| PaddedCursor(AtomicCursor::new(0)))
                .collect(),
        }
    }

    /// Records `count` executed units of `stage` (total `stage_work`, identical across workers).
    /// The completion crossing the total advances the stage right here, so [`Self::sync`] waiters
    /// only *read* `published` — no RMWs while spinning. Sound unconditionally: an incomplete
    /// claimed batch blocks the advance, so a claimant's stage is still published when its completions land.
    pub(super) fn complete(&self, stage: usize, count: usize, stage_work: usize) {
        if count == 0 {
            return;
        }
        let prev = self.completed.fetch_add(count, Ordering::AcqRel);
        debug_assert!(prev + count <= stage_work, "stage work over-completed");
        if prev + count == stage_work {
            self.advance(stage);
        }
    }

    /// Publishes stage `stage + 1`: resets the claim state, then the publish. Stale claims
    /// can't corrupt the reset cursors (their CAS fails on the stage tag), and no one claims
    /// from `stage + 1` before observing the publish.
    fn advance(&self, stage: usize) {
        let next = stage + 1;
        // Keep the zero-work advance ticket in lockstep with the publishes so its CAS
        // (expecting the current stage) keeps working after completion-driven advances;
        // idempotent when the ticket CAS itself already stored it.
        self.advance_ticket.store(next, Ordering::Relaxed);
        for cursor in &self.cursors {
            cursor
                .0
                .store((next as CursorInt) << STAGE_SHIFT, Ordering::Relaxed);
        }
        self.completed.store(0, Ordering::Relaxed);
        self.published.store(next, Ordering::Release);
    }

    /// Ends the caller's stage `stage` (total `stage_work`); returns the next stage ordinal.
    /// Zero-work stages advance via the ticket; otherwise the crossing completion advances the
    /// stage, so the wait is a pure read of `published`. Stragglers return immediately.
    pub(super) fn sync(&self, stage: usize, stage_work: usize) -> usize {
        let next = stage + 1;

        if stage_work == 0
            && self.published.load(Ordering::Acquire) == stage
            && self
                .advance_ticket
                .compare_exchange(stage, next, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
        {
            self.advance(stage);
            return next;
        }

        let mut spins = 0u32;
        while self.published.load(Ordering::Acquire) == stage {
            core::hint::spin_loop();
            spins += 1;
            if spins > 10_000 {
                // Only reachable with >1 worker (a single worker never waits on
                // another), so `std` is always available here in practice; the
                // `alloc`-only build still needs this to compile.
                #[cfg(feature = "std")]
                std::thread::yield_now();
                spins = 0;
            }
        }
        next
    }

    /// Claims the next batch of stage-`stage` work in `range`, preferring the caller's own
    /// static sub-slice, then stealing from the others'. `None` when the range is drained —
    /// or when the machine already advanced past `stage` (straggler: this stage is done).
    pub(super) fn claim(
        &self,
        stage: usize,
        range: &Range<usize>,
        batch: usize,
        worker_id: usize,
    ) -> Option<Range<usize>> {
        let len = range.end - range.start;
        let per_worker = len.div_ceil(self.num_workers);
        let stage_tag = (stage as CursorInt) << STAGE_SHIFT;

        for k in 0..self.num_workers {
            let victim = (worker_id + k) % self.num_workers;
            let slice_start = (per_worker * victim).min(len);
            let slice_end = (per_worker * (victim + 1)).min(len);
            let slice_len = slice_end - slice_start;
            if slice_len == 0 {
                continue;
            }

            let cursor = &self.cursors[victim].0;
            let mut packed = cursor.load(Ordering::Relaxed);
            loop {
                if packed & !POS_MASK != stage_tag {
                    // The machine advanced past the claimant's stage; everything
                    // it could claim is already done.
                    return None;
                }
                let got = (packed & POS_MASK) as usize;
                if got >= slice_len {
                    // This slice is exhausted (don't grow the cursor).
                    break;
                }
                match cursor.compare_exchange_weak(
                    packed,
                    packed + batch as CursorInt,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        let start = slice_start + got;
                        let end = (start + batch).min(slice_end);
                        return Some(range.start + start..range.start + end);
                    }
                    Err(actual) => packed = actual,
                }
            }
        }

        None
    }
}
