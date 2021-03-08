use super::NEXT_FREE_SENTINEL;
use crate::geometry::ColliderHandle;
use parry::bounding_volume::AABB;
use std::ops::{Index, IndexMut};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct BroadPhaseProxy {
    pub handle: ColliderHandle,
    pub aabb: AABB,
    pub next_free: u32,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct BroadPhaseProxies {
    pub elements: Vec<BroadPhaseProxy>,
    pub first_free: u32,
}

impl BroadPhaseProxies {
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
            first_free: NEXT_FREE_SENTINEL,
        }
    }

    pub fn insert(&mut self, proxy: BroadPhaseProxy) -> usize {
        if self.first_free != NEXT_FREE_SENTINEL {
            let proxy_id = self.first_free as usize;
            self.first_free = self.elements[proxy_id].next_free;
            self.elements[proxy_id] = proxy;
            proxy_id
        } else {
            self.elements.push(proxy);
            self.elements.len() - 1
        }
    }

    pub fn remove(&mut self, proxy_id: usize) {
        self.elements[proxy_id].next_free = self.first_free;
        self.first_free = proxy_id as u32;
    }

    // // FIXME: take holes into account?
    // pub fn get(&self, i: usize) -> Option<&BroadPhaseProxy> {
    //     self.elements.get(i)
    // }

    // FIXME: take holes into account?
    pub fn get_mut(&mut self, i: usize) -> Option<&mut BroadPhaseProxy> {
        self.elements.get_mut(i)
    }
}

impl Index<usize> for BroadPhaseProxies {
    type Output = BroadPhaseProxy;
    fn index(&self, i: usize) -> &BroadPhaseProxy {
        self.elements.index(i)
    }
}

impl IndexMut<usize> for BroadPhaseProxies {
    fn index_mut(&mut self, i: usize) -> &mut BroadPhaseProxy {
        self.elements.index_mut(i)
    }
}
