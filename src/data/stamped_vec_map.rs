//! A `VecMap` whose writes are stamped for snapshot deltas.

use crate::alloc_prelude::*;
use crate::data::change_stamps::ChangeStamps;
use core::ops::{Index, IndexMut};

/// A map from small integer keys to values, laid out (and serialized) exactly like parry's
/// `VecMap`, stamping every slot it hands out mutably.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct StampedVecMap<V> {
    n: usize,
    v: Vec<Option<V>>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) stamps: ChangeStamps,
}

impl<V> Default for StampedVecMap<V> {
    fn default() -> Self {
        Self {
            n: 0,
            v: Vec::new(),
            stamps: ChangeStamps::new(),
        }
    }
}

impl<V> Index<usize> for StampedVecMap<V> {
    type Output = V;
    #[inline]
    fn index(&self, i: usize) -> &V {
        self.get(i).expect("key not present")
    }
}

impl<V> IndexMut<usize> for StampedVecMap<V> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut V {
        self.get_mut(i).expect("key not present")
    }
}

impl<V> StampedVecMap<V> {
    pub(crate) fn len(&self) -> usize {
        self.n
    }

    pub(crate) fn clear(&mut self) {
        self.n = 0;
        self.v.clear();
        self.stamps.truncate(0);
    }

    pub(crate) fn get(&self, key: usize) -> Option<&V> {
        self.v.get(key)?.as_ref()
    }

    pub(crate) fn get_mut(&mut self, key: usize) -> Option<&mut V> {
        let value = self.v.get_mut(key)?.as_mut()?;
        self.stamps.mark(key);
        Some(value)
    }

    pub(crate) fn insert(&mut self, key: usize, value: V) -> Option<V> {
        let len = self.v.len();
        if len <= key {
            self.v.extend((0..key - len + 1).map(|_| None));
            self.stamps.mark_range(len..key + 1);
        }
        self.stamps.mark(key);
        let was = self.v[key].replace(value);
        if was.is_none() {
            self.n += 1;
        }
        was
    }

    pub(crate) fn remove(&mut self, key: usize) -> Option<V> {
        let was = self.v.get_mut(key)?.take();
        if was.is_some() {
            self.stamps.mark(key);
            self.n -= 1;
        }
        was
    }

    /// The `(key, value)` pairs, by increasing key.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (usize, &V)> {
        self.v
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.as_ref().map(|v| (i, v)))
    }

}

/// The slots of a [`StampedVecMap`] written since a baseline.
#[cfg(feature = "serde-serialize")]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct VecMapDelta<V> {
    n: u32,
    v: crate::data::delta::VecDelta<Option<V>>,
}

#[cfg(feature = "serde-serialize")]
impl<V: Clone> StampedVecMap<V> {
    pub(crate) fn collect_delta(&self, since: Option<u32>) -> VecMapDelta<V> {
        VecMapDelta {
            n: self.n as u32,
            v: crate::data::delta::VecDelta::collect(&self.v, &self.stamps, since),
        }
    }

    pub(crate) fn apply_delta(
        &mut self,
        delta: VecMapDelta<V>,
    ) -> Result<(), crate::data::DeltaError> {
        delta.v.apply(&mut self.v, &mut self.stamps, || None)?;
        self.n = delta.n as usize;
        Ok(())
    }
}
