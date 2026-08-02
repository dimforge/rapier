//! Persistent solver contact graph: per-color flat arrays of solver contacts,
//! maintained incrementally by the narrow phase over the contact lifecycle
//! (qualify/disqualify/color change), so solvers read ready color-grouped lists with
//! no per-step collect/sort. Entries are manifold *references* `(edge, manifold)`
//! dereferenced only at generate/writeback; buckets are keyed by color alone (the wide kernels
//! handle per-lane contact counts, so a 2↔1 point-count flip is not a graph event).

use crate::alloc_prelude::*;

/// The maximum solver color count (128 parallel colors + 1 overflow), matching
/// [`crate::geometry::SOLVER_COLOR_OVERFLOW`] + 1.
pub(crate) const NUM_SOLVER_COLORS: usize = 129;

/// Number of two-body buckets: one per color.
const NUM_BUCKETS: usize = NUM_SOLVER_COLORS;

/// Bucket id of the generic (multibody-involved) manifold list: solved by the scalar generic
/// constraint path, so kept entirely out of the two-body color buckets — bucket membership
/// alone classifies a manifold.
pub(crate) const GENERIC_BUCKET: u16 = NUM_BUCKETS as u16;

/// All bucket ids (including [`GENERIC_BUCKET`]) must fit [`GraphPos`]'s
/// bucket bit-field.
const _: () = assert!(NUM_BUCKETS < 1 << (32 - GraphPos::BUCKET_SHIFT));

/// Total bucket count including the generic list — the bucket-count arrays of
/// the parallel from-scratch rebuild are sized by this.
#[cfg_attr(not(feature = "parallel"), allow(dead_code))] // Parallel bulk-rebuild path.
pub(crate) const NUM_BUCKETS_WITH_GENERIC: usize = NUM_BUCKETS + 1;

/// Bucket id of a solver color.
#[inline]
pub(crate) fn bucket_id(color: u8) -> u16 {
    color as u16
}

/// A solver-active manifold: contact-graph edge + ordinal among the pair's solver manifolds.
/// Also the per-lane address the contact constraints store (resolved through the manifold
/// store at generate/writeback time), with [`ContactRef::PADDING`] marking unused SIMD lanes.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct ContactRef {
    pub edge: u32,
    pub manifold: u32,
}

impl ContactRef {
    /// Sentinel for a padding SIMD lane (no manifold).
    pub(crate) const PADDING: ContactRef = ContactRef {
        edge: u32::MAX,
        manifold: u32::MAX,
    };

    #[inline]
    pub(crate) fn is_padding(self) -> bool {
        self.edge == u32::MAX
    }
}

impl Default for ContactRef {
    fn default() -> Self {
        Self::PADDING
    }
}

/// A manifold's graph position (bucket + index within it), stored on the manifold so
/// removal/move is O(1); `NONE` = not in the graph. Layout: bucket in the high bits, local
/// index in the low `BUCKET_SHIFT` bits (must hold up to [`GENERIC_BUCKET`], asserted above).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct GraphPos(u32);

impl GraphPos {
    pub(crate) const NONE: GraphPos = GraphPos(u32::MAX);
    const BUCKET_SHIFT: u32 = 22;
    const LOCAL_MASK: u32 = (1 << Self::BUCKET_SHIFT) - 1;

    #[inline]
    pub(crate) fn new(bucket: u16, local: u32) -> Self {
        debug_assert!(local < (1 << Self::BUCKET_SHIFT));
        debug_assert!((bucket as u32) < (1 << (32 - Self::BUCKET_SHIFT)));
        let pos = GraphPos(((bucket as u32) << Self::BUCKET_SHIFT) | local);
        debug_assert!(pos.is_some());
        pos
    }

    #[inline]
    pub(crate) fn is_some(self) -> bool {
        self.0 != u32::MAX
    }

    #[inline]
    pub(crate) fn bucket(self) -> u16 {
        (self.0 >> Self::BUCKET_SHIFT) as u16
    }

    #[inline]
    pub(crate) fn local(self) -> u32 {
        self.0 & Self::LOCAL_MASK
    }
}

impl Default for GraphPos {
    fn default() -> Self {
        GraphPos::NONE
    }
}

/// The persistent, incrementally-maintained per-color contact buckets, plus
/// the generic (multibody-involved) manifold list at [`GENERIC_BUCKET`].
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SolverContactGraph {
    buckets: Vec<Vec<ContactRef>>,
}

impl SolverContactGraph {
    pub(crate) fn new() -> Self {
        Self {
            buckets: alloc::vec![Vec::new(); NUM_BUCKETS + 1],
        }
    }

    /// Clears every bucket (used when the graph must be rebuilt from scratch,
    /// e.g. after an island epoch change). The parallel build rebuilds through
    /// [`Self::resize_for_bulk_rebuild`] instead, which clears as it resizes.
    #[cfg(not(feature = "parallel"))]
    pub(crate) fn clear(&mut self) {
        if self.buckets.is_empty() {
            self.buckets.resize_with(NUM_BUCKETS + 1, Vec::new);
        }
        for b in &mut self.buckets {
            b.clear();
        }
    }

    /// Inserts a solver-active manifold into its color bucket and returns its
    /// [`GraphPos`] back-reference (to be stored on the manifold).
    #[inline]
    pub(crate) fn insert(&mut self, color: u8, contact: ContactRef) -> GraphPos {
        let bucket = bucket_id(color);
        let arr = &mut self.buckets[bucket as usize];
        let local = arr.len() as u32;
        arr.push(contact);
        GraphPos::new(bucket, local)
    }

    /// Inserts a solver-active manifold involving a multibody link into the
    /// generic list and returns its [`GraphPos`] back-reference.
    #[inline]
    pub(crate) fn insert_generic(&mut self, contact: ContactRef) -> GraphPos {
        let arr = &mut self.buckets[GENERIC_BUCKET as usize];
        let local = arr.len() as u32;
        arr.push(contact);
        GraphPos::new(GENERIC_BUCKET, local)
    }

    /// Prepares the buckets for a from-scratch parallel rebuild: clears and resizes each bucket
    /// (generic list included) to `lens[bucket]`, returning the raw base pointers the rebuild's
    /// scatter pass writes through at precomputed, disjoint offsets.
    #[cfg(feature = "parallel")]
    pub(crate) fn resize_for_bulk_rebuild(&mut self, lens: &[u32]) -> Vec<*mut ContactRef> {
        if self.buckets.is_empty() {
            self.buckets.resize_with(NUM_BUCKETS_WITH_GENERIC, Vec::new);
        }
        debug_assert_eq!(lens.len(), self.buckets.len());
        self.buckets
            .iter_mut()
            .zip(lens.iter())
            .map(|(b, len)| {
                b.clear();
                b.resize(*len as usize, ContactRef::PADDING);
                b.as_mut_ptr()
            })
            .collect()
    }

    /// Rewrites the contact-graph edge index of the entry at `pos` (the edges
    /// vec swap-remove moved its pair to a new index).
    #[inline]
    pub(crate) fn rewrite_edge(&mut self, pos: GraphPos, new_edge: u32) {
        debug_assert!(pos.is_some());
        self.buckets[pos.bucket() as usize][pos.local() as usize].edge = new_edge;
    }

    /// Removes the manifold at `pos` by swap-remove. Returns the [`ContactRef`]
    /// of the entry that was moved into the hole (whose stored [`GraphPos`] the
    /// caller must rewrite to `pos`), or `None` if the removed entry was last.
    #[inline]
    pub(crate) fn remove(&mut self, pos: GraphPos) -> Option<ContactRef> {
        debug_assert!(pos.is_some());
        let arr = &mut self.buckets[pos.bucket() as usize];
        let local = pos.local() as usize;
        let last = arr.len() - 1;
        arr.swap_remove(local);
        if local != last {
            // The previously-last entry now lives at `local`; its owner's
            // back-ref must be fixed to `pos`.
            Some(arr[local])
        } else {
            None
        }
    }

    /// Total number of solver-active manifolds across all buckets.
    // Sizes the staged solver's worker-count clamp; the single-threaded path
    // doesn't need totals.
    #[cfg_attr(not(feature = "parallel"), allow(dead_code))]
    pub(crate) fn len(&self) -> usize {
        self.buckets.iter().map(|b| b.len()).sum()
    }

    /// Iterates every non-empty two-body color bucket as `(color, &[ContactRef])`, skipping the
    /// generic list. Colors below [`crate::geometry::SOLVER_COLOR_OVERFLOW`] are body-disjoint;
    /// the overflow color's bucket may share bodies.
    pub(crate) fn buckets(&self) -> impl Iterator<Item = (u8, &[ContactRef])> {
        self.buckets[..NUM_BUCKETS]
            .iter()
            .enumerate()
            .filter_map(|(id, arr)| {
                if arr.is_empty() {
                    None
                } else {
                    Some((id as u8, arr.as_slice()))
                }
            })
    }

    /// The generic (multibody-involved) manifold list.
    pub(crate) fn generic(&self) -> &[ContactRef] {
        &self.buckets[GENERIC_BUCKET as usize]
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn cref(edge: u32) -> ContactRef {
        ContactRef { edge, manifold: 0 }
    }

    /// Insert/remove with the swap-remove + back-ref fixup must keep every
    /// live entry findable through its returned [`GraphPos`].
    #[test]
    fn insert_remove_backref_consistency() {
        let mut g = SolverContactGraph::new();
        // A dense body of contacts across a few color buckets, tracked by
        // (edge -> current GraphPos) exactly as the narrow phase would.
        let mut pos: alloc::collections::BTreeMap<u32, GraphPos> = Default::default();

        // Insert 200 contacts spread over 5 colors.
        for e in 0..200u32 {
            let color = (e % 5) as u8;
            let p = g.insert(color, cref(e));
            pos.insert(e, p);
        }
        assert_eq!(g.len(), 200);

        // Remove every third contact, applying the swap-remove back-ref fixup.
        let to_remove: alloc::vec::Vec<u32> = (0..200u32).step_by(3).collect();
        for &e in &to_remove {
            let p = pos.remove(&e).unwrap();
            if let Some(moved) = g.remove(p) {
                // The moved entry now lives where `e` was.
                *pos.get_mut(&moved.edge).unwrap() = p;
            }
        }

        // Every surviving contact must still be at its tracked position and
        // in a bucket matching its color.
        assert_eq!(g.len(), 200 - to_remove.len());
        for (color, arr) in g.buckets() {
            for (local, c) in arr.iter().enumerate() {
                assert_eq!((c.edge % 5) as u8, color);
                let tracked = pos[&c.edge];
                assert_eq!(tracked.bucket(), bucket_id(color));
                assert_eq!(tracked.local() as usize, local);
                assert!(!to_remove.contains(&c.edge));
            }
        }
    }

    /// The highest bucket ids (overflow color and the generic list) must
    /// round-trip through [`GraphPos`] and never alias low-color buckets.
    #[test]
    fn high_bucket_ids_roundtrip() {
        let mut g = SolverContactGraph::new();
        let overflow = (NUM_SOLVER_COLORS - 1) as u8; // SOLVER_COLOR_OVERFLOW
        let p = g.insert(overflow, cref(7));
        assert_eq!(p.bucket(), bucket_id(overflow));
        // Removing through the stored position must drain the overflow bucket,
        // not alias a low-color one.
        let _ = g.insert(0, cref(1));
        assert!(g.remove(p).is_none());
        let remaining: alloc::vec::Vec<_> = g.buckets().map(|(c, arr)| (c, arr.len())).collect();
        assert_eq!(remaining, alloc::vec![(0, 1)]);
        // Generic-list positions round-trip too.
        let pg = g.insert_generic(cref(9));
        assert_eq!(pg.bucket(), GENERIC_BUCKET);
        assert_eq!(g.generic().len(), 1);
        assert!(g.remove(pg).is_none());
        assert!(g.generic().is_empty());
    }

    /// Moving a contact between color buckets (a recolor) is
    /// remove-then-insert; verify the back-refs stay consistent.
    #[test]
    fn move_between_color_buckets() {
        let mut g = SolverContactGraph::new();
        let mut pos: alloc::collections::BTreeMap<u32, GraphPos> = Default::default();
        for e in 0..10u32 {
            pos.insert(e, g.insert(3, cref(e)));
        }
        // Move contact 4 from color 3 to color 7.
        let p = pos[&4];
        if let Some(moved) = g.remove(p) {
            *pos.get_mut(&moved.edge).unwrap() = p;
        }
        pos.insert(4, g.insert(7, cref(4)));

        assert_eq!(g.len(), 10);
        for (color, arr) in g.buckets() {
            for (local, c) in arr.iter().enumerate() {
                assert_eq!(pos[&c.edge].local() as usize, local);
                if c.edge == 4 {
                    assert_eq!(color, 7);
                } else {
                    assert_eq!(color, 3);
                }
            }
        }
    }
}
