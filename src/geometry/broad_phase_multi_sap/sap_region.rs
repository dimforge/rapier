use super::{BroadPhaseProxies, SAPAxis};
use crate::math::DIM;
use bit_vec::BitVec;
use parry::bounding_volume::AABB;
use parry::utils::hashmap::HashMap;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct SAPRegion {
    pub axes: [SAPAxis; DIM],
    pub existing_proxies: BitVec,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub to_insert: Vec<usize>, // Workspace
    pub update_count: u8,
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
            update_count: 0,
            proxy_count: 0,
        }
    }

    pub fn recycle(bounds: AABB, mut old: Self) -> Self {
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

    pub fn recycle_or_new(bounds: AABB, pool: &mut Vec<Self>) -> Self {
        if let Some(old) = pool.pop() {
            Self::recycle(bounds, old)
        } else {
            Self::new(bounds)
        }
    }

    pub fn delete_all_region_endpoints(&mut self, proxies: &BroadPhaseProxies) {
        for axis in &mut self.axes {
            axis.endpoints.retain(|e| {
                if let Some(proxy) = proxies.get(e.proxy() as usize) {
                    self.existing_proxies.set(e.proxy() as usize, false);
                    !proxy.data.is_subregion()
                } else {
                    true
                }
            });
        }
    }

    pub fn predelete_proxy(&mut self, _proxy_id: usize) {
        // We keep the proxy_id as argument for uniformity with the "preupdate"
        // method. However we don't actually need it because the deletion will be
        // handled transparently during the next update.
        self.update_count = 1;
    }

    pub fn preupdate_proxy(&mut self, proxy_id: usize) -> bool {
        let mask_len = self.existing_proxies.len();
        if proxy_id >= mask_len {
            self.existing_proxies.grow(proxy_id + 1 - mask_len, false);
        }

        if !self.existing_proxies[proxy_id] {
            self.to_insert.push(proxy_id);
            self.existing_proxies.set(proxy_id, true);
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

    pub fn update(
        &mut self,
        proxies: &BroadPhaseProxies,
        reporting: &mut HashMap<(u32, u32), bool>,
    ) {
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
