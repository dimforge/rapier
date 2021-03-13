use super::{SAPAxis, SAPProxies};
use crate::geometry::SAPProxyIndex;
use crate::math::DIM;
use bit_vec::BitVec;
use parry::bounding_volume::AABB;
use parry::utils::hashmap::HashMap;

pub type SAPRegionPool = Vec<Box<SAPRegion>>;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct SAPRegion {
    pub axes: [SAPAxis; DIM],
    pub existing_proxies: BitVec,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub to_insert: Vec<SAPProxyIndex>, // Workspace
    pub subregions: Vec<SAPProxyIndex>,
    pub id_in_parent_subregion: u32,
    pub update_count: u8,
    pub needs_update_after_subregion_removal: bool,
    pub proxy_count: usize,
}

impl SAPRegion {
    pub fn new(bounds: AABB) -> Self {
        let axes = [
            SAPAxis::new(bounds.mins.x, bounds.maxs.x),
            SAPAxis::new(bounds.mins.y, bounds.maxs.y),
            #[cfg(feature = "dim3")]
            SAPAxis::new(bounds.mins.z, bounds.maxs.z),
        ];
        SAPRegion {
            axes,
            existing_proxies: BitVec::new(),
            to_insert: Vec::new(),
            subregions: Vec::new(),
            id_in_parent_subregion: crate::INVALID_U32,
            update_count: 0,
            needs_update_after_subregion_removal: false,
            proxy_count: 0,
        }
    }

    pub fn recycle(bounds: AABB, mut old: Box<Self>) -> Box<Self> {
        // Correct the bounds
        for (axis, &bound) in old.axes.iter_mut().zip(bounds.mins.iter()) {
            axis.min_bound = bound;
        }
        for (axis, &bound) in old.axes.iter_mut().zip(bounds.maxs.iter()) {
            axis.max_bound = bound;
        }

        old.update_count = 0;

        // The rest of the fields should be "empty"
        assert_eq!(old.proxy_count, 0);
        assert!(old.to_insert.is_empty());
        debug_assert!(!old.existing_proxies.any());
        assert!(old.axes.iter().all(|ax| ax.endpoints.len() == 2));

        old
    }

    pub fn recycle_or_new(bounds: AABB, pool: &mut Vec<Box<Self>>) -> Box<Self> {
        if let Some(old) = pool.pop() {
            Self::recycle(bounds, old)
        } else {
            Box::new(Self::new(bounds))
        }
    }

    pub fn delete_all_region_endpoints(&mut self, proxies: &SAPProxies) {
        for axis in &mut self.axes {
            let existing_proxies = &mut self.existing_proxies;
            axis.endpoints.retain(|e| {
                if let Some(proxy) = proxies.get(e.proxy()) {
                    existing_proxies.set(e.proxy() as usize, false);
                    !proxy.data.is_subregion()
                } else {
                    true
                }
            });
        }
    }

    pub fn predelete_proxy(&mut self, _proxy_id: SAPProxyIndex) {
        // We keep the proxy_id as argument for uniformity with the "preupdate"
        // method. However we don't actually need it because the deletion will be
        // handled transparently during the next update.
        self.update_count = self.update_count.max(1);
    }

    pub fn mark_as_dirty(&mut self) {
        self.update_count = self.update_count.max(1);
    }

    pub fn register_subregion(&mut self, proxy_id: SAPProxyIndex) -> usize {
        let subregion_index = self.subregions.len();
        self.subregions.push(proxy_id);
        self.preupdate_proxy(proxy_id);
        subregion_index
    }

    pub fn preupdate_proxy(&mut self, proxy_id: SAPProxyIndex) -> bool {
        let mask_len = self.existing_proxies.len();
        if proxy_id as usize >= mask_len {
            self.existing_proxies
                .grow(proxy_id as usize + 1 - mask_len, false);
        }

        if !self.existing_proxies[proxy_id as usize] {
            self.to_insert.push(proxy_id);
            self.existing_proxies.set(proxy_id as usize, true);
            self.proxy_count += 1;
            false
        } else {
            // Here we need a second update if all proxies exit this region. In this case, we need
            // to delete the final proxy, but the region may not have AABBs overlapping it, so it
            // wouldn't get an update otherwise.
            self.update_count = 2;
            true
        }
    }

    pub fn update_after_subregion_removal(&mut self, proxies: &SAPProxies) {
        if self.needs_update_after_subregion_removal {
            for axis in &mut self.axes {
                self.proxy_count -= axis
                    .delete_deleted_proxies_and_endpoints_after_subregion_removal(
                        proxies,
                        &mut self.existing_proxies,
                    );
            }
            self.needs_update_after_subregion_removal = false;
        }
    }

    pub fn update(&mut self, proxies: &SAPProxies, reporting: &mut HashMap<(u32, u32), bool>) {
        if self.update_count > 0 {
            // Update endpoints.
            let mut deleted = 0;

            for dim in 0..DIM {
                self.axes[dim].update_endpoints(dim, proxies, reporting);
                deleted += self.axes[dim].delete_out_of_bounds_proxies(&mut self.existing_proxies);
            }

            if deleted > 0 {
                self.proxy_count -= deleted;
                for dim in 0..DIM {
                    self.axes[dim].delete_out_of_bounds_endpoints(&self.existing_proxies);
                }
            }

            self.update_count -= 1;
        }

        if !self.to_insert.is_empty() {
            // Insert new proxies.
            for dim in 1..DIM {
                self.axes[dim].batch_insert(dim, &self.to_insert, proxies, None);
            }
            self.axes[0].batch_insert(0, &self.to_insert, proxies, Some(reporting));
            self.to_insert.clear();

            // In the rare event that all proxies leave this region in the next step, we need an
            // update to remove them.
            self.update_count = 1;
        }
    }
}
