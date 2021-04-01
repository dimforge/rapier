use super::NEXT_FREE_SENTINEL;
use crate::geometry::broad_phase_multi_sap::SAPRegion;
use crate::geometry::ColliderHandle;
use parry::bounding_volume::AABB;
use std::ops::{Index, IndexMut};

pub type SAPProxyIndex = u32;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub enum SAPProxyData {
    Collider(ColliderHandle),
    Region(Option<Box<SAPRegion>>),
}

impl SAPProxyData {
    pub fn is_region(&self) -> bool {
        match self {
            SAPProxyData::Region(_) => true,
            _ => false,
        }
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
    pub aabb: AABB,
    pub next_free: SAPProxyIndex,
    // TODO: pack the layer_id and layer_depth into a single u16?
    pub layer_id: u8,
    pub layer_depth: i8,
}

impl SAPProxy {
    pub fn collider(handle: ColliderHandle, aabb: AABB, layer_id: u8, layer_depth: i8) -> Self {
        Self {
            data: SAPProxyData::Collider(handle),
            aabb,
            next_free: NEXT_FREE_SENTINEL,
            layer_id,
            layer_depth,
        }
    }

    pub fn subregion(subregion: Box<SAPRegion>, aabb: AABB, layer_id: u8, layer_depth: i8) -> Self {
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
    pub first_free: SAPProxyIndex,
}

impl SAPProxies {
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
            first_free: NEXT_FREE_SENTINEL,
        }
    }

    pub fn insert(&mut self, proxy: SAPProxy) -> SAPProxyIndex {
        let result = if self.first_free != NEXT_FREE_SENTINEL {
            let proxy_id = self.first_free;
            self.first_free = self.elements[proxy_id as usize].next_free;
            self.elements[proxy_id as usize] = proxy;
            proxy_id
        } else {
            self.elements.push(proxy);
            self.elements.len() as u32 - 1
        };

        result
    }

    pub fn remove(&mut self, proxy_id: SAPProxyIndex) {
        let proxy = &mut self.elements[proxy_id as usize];
        proxy.next_free = self.first_free;
        self.first_free = proxy_id as u32;
    }

    // NOTE: this must not take holes into account.
    pub fn get_mut(&mut self, i: SAPProxyIndex) -> Option<&mut SAPProxy> {
        self.elements.get_mut(i as usize)
    }
    // NOTE: this must not take holes into account.
    pub fn get(&self, i: SAPProxyIndex) -> Option<&SAPProxy> {
        self.elements.get(i as usize)
    }
}

impl Index<SAPProxyIndex> for SAPProxies {
    type Output = SAPProxy;
    fn index(&self, i: SAPProxyIndex) -> &SAPProxy {
        self.elements.index(i as usize)
    }
}

impl IndexMut<SAPProxyIndex> for SAPProxies {
    fn index_mut(&mut self, i: SAPProxyIndex) -> &mut SAPProxy {
        self.elements.index_mut(i as usize)
    }
}
