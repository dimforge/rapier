//! Index-addressed vectors whose writes are stamped for snapshot deltas.

use crate::alloc_prelude::*;
use crate::data::change_stamps::ChangeStamps;
use core::ops::{Deref, Index, IndexMut};

/// A `Vec` stamping every slot it hands out mutably, serialized exactly like a `Vec`.
///
/// Reads go through `Deref<Target = [T]>`; every mutation goes through a method that stamps
/// the slots it can write.
#[derive(Clone, Debug)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(Serialize, Deserialize),
    serde(transparent)
)]
pub(crate) struct StampedVec<T> {
    items: Vec<T>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) stamps: ChangeStamps,
}

impl<T> Default for StampedVec<T> {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            stamps: ChangeStamps::new(),
        }
    }
}

impl<T> Deref for StampedVec<T> {
    type Target = [T];
    #[inline]
    fn deref(&self) -> &[T] {
        &self.items
    }
}

impl<T> Index<usize> for StampedVec<T> {
    type Output = T;
    #[inline]
    fn index(&self, i: usize) -> &T {
        &self.items[i]
    }
}

impl<T> IndexMut<usize> for StampedVec<T> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut T {
        self.stamps.mark(i);
        &mut self.items[i]
    }
}

#[allow(dead_code)] // Not every container uses every operation.
impl<T> StampedVec<T> {
    pub(crate) fn len(&self) -> usize {
        self.items.len()
    }

    pub(crate) fn get_mut(&mut self, i: usize) -> Option<&mut T> {
        let item = self.items.get_mut(i)?;
        self.stamps.mark(i);
        Some(item)
    }

    pub(crate) fn push(&mut self, value: T) {
        self.stamps.mark(self.items.len());
        self.items.push(value);
    }

    pub(crate) fn pop(&mut self) -> Option<T> {
        let value = self.items.pop();
        self.stamps.truncate(self.items.len());
        value
    }

    pub(crate) fn swap_remove(&mut self, i: usize) -> T {
        let len = self.items.len();
        let value = self.items.swap_remove(i);
        self.stamps.swap_remove(i, len);
        value
    }

    pub(crate) fn truncate(&mut self, len: usize) {
        self.items.truncate(len);
        self.stamps.truncate(len);
    }

    pub(crate) fn clear(&mut self) {
        self.truncate(0);
    }

    pub(crate) fn resize(&mut self, len: usize, value: T)
    where
        T: Clone,
    {
        let old = self.items.len();
        self.items.resize(len, value);
        self.stamps.truncate(len);
        self.stamps.mark_range(old..len);
    }

    pub(crate) fn resize_with(&mut self, len: usize, f: impl FnMut() -> T) {
        let old = self.items.len();
        self.items.resize_with(len, f);
        self.stamps.truncate(len);
        self.stamps.mark_range(old..len);
    }

    /// Mutable access to every slot, all stamped.
    pub(crate) fn as_mut_slice(&mut self) -> &mut [T] {
        self.stamps.mark_range(0..self.items.len());
        &mut self.items
    }

    /// Mutable access to every slot without stamping: the caller stamps the slots it
    /// actually writes through the returned stamps.
    pub(crate) fn as_mut_slice_unstamped(&mut self) -> (&mut [T], &mut ChangeStamps) {
        (&mut self.items, &mut self.stamps)
    }

    /// Replaces the whole content, stamping every slot.
    pub(crate) fn replace(&mut self, items: Vec<T>) {
        self.items = items;
        self.stamps.truncate(0);
        self.stamps.mark_range(0..self.items.len());
    }

    pub(crate) fn as_vec(&self) -> &Vec<T> {
        &self.items
    }
}

#[cfg(feature = "serde-serialize")]
impl<T: Clone> StampedVec<T> {
    pub(crate) fn collect_delta(&self, since: Option<u32>) -> crate::data::delta::VecDelta<T> {
        crate::data::delta::VecDelta::collect(&self.items, &self.stamps, since)
    }

    pub(crate) fn apply_delta(
        &mut self,
        delta: crate::data::delta::VecDelta<T>,
        placeholder: impl FnMut() -> T,
    ) -> Result<(), crate::data::DeltaError> {
        delta.apply(&mut self.items, &mut self.stamps, placeholder)
    }
}
