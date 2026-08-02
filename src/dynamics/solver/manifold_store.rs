//! Raw view resolving [`ContactRef`]s to contact manifolds for the solvers.

use crate::data::graph::Edge;
use crate::dynamics::solver::solver_contact_graph::ContactRef;
use crate::geometry::{ContactManifold, ContactPair};

/// The contact graph's edge-array pointer and length, type-erased (the pointer is held as a
/// `usize` so the value stays `Send` and holdable across a later exclusive narrow-phase borrow)
/// until the solver scope rebuilds a [`ManifoldStore`] from it with [`ManifoldStore::from_parts`].
#[derive(Copy, Clone)]
pub(crate) struct ManifoldStoreParts {
    edges_ptr: usize,
    num_edges: usize,
}

impl ManifoldStoreParts {
    /// Erases the contact graph's live edge array into a pointer + length.
    pub(crate) fn new(edges: *mut Edge<ContactPair>, num_edges: usize) -> Self {
        Self {
            edges_ptr: edges as usize,
            num_edges,
        }
    }
}

/// A view over the narrow-phase's contact-pair storage resolving [`ContactRef`]s to manifolds
/// at generate/writeback time (replaces the per-step `Vec<&mut ContactManifold>` collection).
/// Aliasing contract: created from an exclusive narrow-phase borrow, the graph is not mutated
/// while any store exists; [`Self::get_mut`] callers must keep at most one live `&mut` per
/// `(edge, ordinal)`; concurrent resolution of one pair's ordinals only takes transient borrows
/// of the pair's manifold `Vec` *header* (never written in the solver scope).
pub(crate) struct ManifoldStore<'a> {
    edges: *mut Edge<ContactPair>,
    num_edges: usize,
    _phantom: core::marker::PhantomData<&'a mut ContactPair>,
}

// SAFETY: see the aliasing contract above — all shared state behind the raw
// pointer is either read-only during the solver scope or accessed through
// caller-guaranteed-disjoint `get_mut` calls.
unsafe impl Send for ManifoldStore<'_> {}
unsafe impl Sync for ManifoldStore<'_> {}

impl<'a> ManifoldStore<'a> {
    /// # Safety
    /// `parts` must erase the narrow-phase contact graph's live edge array,
    /// and the graph must not be mutated for `'a`.
    pub(crate) unsafe fn from_parts(parts: ManifoldStoreParts) -> Self {
        Self {
            edges: parts.edges_ptr as *mut Edge<ContactPair>,
            num_edges: parts.num_edges,
            _phantom: core::marker::PhantomData,
        }
    }

    #[inline]
    fn pair_ptr(&self, edge: u32) -> *mut ContactPair {
        // Always-on (not `debug_assert!`): a stale `ContactRef` (e.g. a graph-maintenance bug)
        // would offset past the edge array and dereference wild memory (UB). One cached-field
        // compare turns that into a defined panic and keeps the `.add()` below in-bounds.
        assert!(
            (edge as usize) < self.num_edges,
            "stale ContactRef edge index"
        );
        unsafe { &raw mut (*self.edges.add(edge as usize)).weight }
    }

    #[inline]
    fn manifold_ptr(&self, r: ContactRef) -> *mut ContactManifold {
        debug_assert!(!r.is_padding());
        let pair = self.pair_ptr(r.edge);
        // Mirrors `ContactPair::solver_manifolds` without materializing a
        // reference to the whole pair (distinct ordinals of one pair may be
        // resolved concurrently; see the aliasing contract).
        unsafe {
            let clusters: *mut _ = &raw mut (*pair).solver_clusters;
            let vec = if (*clusters).is_empty() {
                &raw mut (*pair).manifolds
            } else {
                clusters
            };
            // Always-on bound (see `pair_ptr`): a stale manifold ordinal (composite-pair hazard
            // — the pair's manifold list shrank under a graph entry still referencing a dropped
            // ordinal) would index past the list. Panic instead of the wild `.add()` deref.
            assert!(
                (r.manifold as usize) < (*vec).len(),
                "stale ContactRef manifold ordinal"
            );
            (*vec).as_mut_ptr().add(r.manifold as usize)
        }
    }

    /// Resolves a manifold for reading.
    #[inline]
    pub(crate) fn get(&self, r: ContactRef) -> &ContactManifold {
        unsafe { &*self.manifold_ptr(r) }
    }

    /// Prefetches the pair header of `r` (the first hop of [`Self::get`]'s
    /// dependent pointer chain). No-op on padding/out-of-range refs.
    #[inline]
    pub(crate) fn prefetch_pair(&self, r: ContactRef) {
        if r.is_padding() || r.edge as usize >= self.num_edges {
            return;
        }
        let pair = unsafe { &raw const (*self.edges.add(r.edge as usize)).weight };
        crate::utils::prefetch_read::<0, _>(pair);
    }

    /// Prefetches the manifold of `r` (second hop of [`Self::get`]'s dependent chain). Reads
    /// the pair header, so it is only cheap if [`Self::prefetch_pair`] was issued early enough
    /// beforehand. No-op on padding/out-of-range refs.
    #[inline]
    pub(crate) fn prefetch_manifold(&self, r: ContactRef) {
        if r.is_padding() || r.edge as usize >= self.num_edges {
            return;
        }
        let pair = self.pair_ptr(r.edge);
        unsafe {
            let clusters = &raw mut (*pair).solver_clusters;
            let vec = if (*clusters).is_empty() {
                &raw mut (*pair).manifolds
            } else {
                clusters
            };
            if (r.manifold as usize) >= (*vec).len() {
                return;
            }
            let m = (*vec).as_ptr().add(r.manifold as usize);
            // A 2D manifold (inline points + solver contacts) spans ~5 cache lines and the
            // most-read part (`data` with the inline solver contacts) sits at the TAIL of the
            // struct, so short prefetches miss exactly the bytes generation reads first.
            crate::utils::prefetch_read::<4, _>(m);
        }
    }

    /// Resolves a manifold for writing (impulse writeback).
    ///
    /// # Safety
    /// No other live reference to this `(edge, ordinal)` manifold may exist.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn get_mut(&self, r: ContactRef) -> &mut ContactManifold {
        unsafe { &mut *self.manifold_ptr(r) }
    }
}
