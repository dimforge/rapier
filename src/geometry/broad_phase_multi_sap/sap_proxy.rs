use super::NEXT_FREE_SENTINEL;
use crate::geometry::broad_phase_multi_sap::SAPRegion;
use crate::geometry::{BroadPhaseProxyIndex, ColliderHandle};
use parry::bounding_volume::Aabb;
use std::ops::{Index, IndexMut};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub enum SAPProxyData {
    Collider(ColliderHandle),
    Region(Option<Box<SAPRegion>>),
}

impl SAPProxyData {
    pub fn is_region(&self) -> bool {
        matches!(self, SAPProxyData::Region(_))
    }

    pub fn as_region(&self) -> &SAPRegion {
        match self {
            SAPProxyData::Region(r) => r.as_ref().unwrap(),
            _ => panic!("Invalid proxy type."),
        }
    }

    pub fn as_region_mut(&mut self) -> &mut SAPRegion {
        match self {
            SAPProxyData::Region(r) => r.as_mut().unwrap(),
            _ => panic!("Invalid proxy type."),
        }
    }

    pub fn take_region(&mut self) -> Option<Box<SAPRegion>> {
        match self {
            SAPProxyData::Region(r) => r.take(),
            _ => None,
        }
    }

    pub fn set_region(&mut self, region: Box<SAPRegion>) {
        *self = SAPProxyData::Region(Some(region));
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct SAPProxy {
    pub data: SAPProxyData,
    pub aabb: Aabb,
    pub next_free: BroadPhaseProxyIndex,
    // TODO: pack the layer_id and layer_depth into a single u16?
    pub layer_id: u8,
    pub layer_depth: i8,
}

impl SAPProxy {
    pub fn collider(handle: ColliderHandle, aabb: Aabb, layer_id: u8, layer_depth: i8) -> Self {
        Self {
            data: SAPProxyData::Collider(handle),
            aabb,
            next_free: NEXT_FREE_SENTINEL,
            layer_id,
            layer_depth,
        }
    }

    pub fn subregion(subregion: Box<SAPRegion>, aabb: Aabb, layer_id: u8, layer_depth: i8) -> Self {
        Self {
            data: SAPProxyData::Region(Some(subregion)),
            aabb,
            next_free: NEXT_FREE_SENTINEL,
            layer_id,
            layer_depth,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct SAPProxies {
    pub elements: Vec<SAPProxy>,
    pub first_free: BroadPhaseProxyIndex,
}

impl Default for SAPProxies {
    fn default() -> Self {
        Self::new()
    }
}

impl SAPProxies {
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
            first_free: NEXT_FREE_SENTINEL,
        }
    }

    pub fn insert(&mut self, proxy: SAPProxy) -> BroadPhaseProxyIndex {
        if self.first_free != NEXT_FREE_SENTINEL {
            let proxy_id = self.first_free;
            self.first_free = self.elements[proxy_id as usize].next_free;
            self.elements[proxy_id as usize] = proxy;
            proxy_id
        } else {
            self.elements.push(proxy);
            self.elements.len() as u32 - 1
        }
    }

    pub fn remove(&mut self, proxy_id: BroadPhaseProxyIndex) {
        let proxy = &mut self.elements[proxy_id as usize];
        proxy.next_free = self.first_free;
        self.first_free = proxy_id;
    }

    // NOTE: this must not take holes into account.
    pub fn get_mut(&mut self, i: BroadPhaseProxyIndex) -> Option<&mut SAPProxy> {
        self.elements.get_mut(i as usize)
    }
    // NOTE: this must not take holes into account.
    pub fn get(&self, i: BroadPhaseProxyIndex) -> Option<&SAPProxy> {
        self.elements.get(i as usize)
    }
}

impl Index<BroadPhaseProxyIndex> for SAPProxies {
    type Output = SAPProxy;
    fn index(&self, i: BroadPhaseProxyIndex) -> &SAPProxy {
        self.elements.index(i as usize)
    }
}

impl IndexMut<BroadPhaseProxyIndex> for SAPProxies {
    fn index_mut(&mut self, i: BroadPhaseProxyIndex) -> &mut SAPProxy {
        self.elements.index_mut(i as usize)
    }
}
