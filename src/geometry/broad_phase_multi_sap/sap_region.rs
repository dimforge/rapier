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
    // Number of proxies (added to this region) that originates
    // from the layer at depth <= the depth of the layer containing
    // this region.
    subproper_proxy_count: usize,
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
            subproper_proxy_count: 0,
        }
    }

    pub fn recycle(bounds: AABB, mut old: Box<Self>) -> Box<Self> {
        // Correct the bounds
        for i in 0..DIM {
            // Make sure the axis is empty (it may still contain
            // some old endpoints from non-proper proxies.
            old.axes[i].clear();
            old.axes[i].min_bound = bounds.mins[i];
            old.axes[i].max_bound = bounds.maxs[i];
        }

        old.update_count = 0;

        if cfg!(feature = "enhanced-determinism") {
            old.existing_proxies = BitVec::new();
        } else {
            old.existing_proxies.clear();
        }

        old.id_in_parent_subregion = crate::INVALID_U32;
        old.subregions.clear();
        old.needs_update_after_subregion_removal = false;

        // The rest of the fields should be "empty"
        assert_eq!(old.subproper_proxy_count, 0);
        assert!(old.to_insert.is_empty());

        old
    }

    pub fn recycle_or_new(bounds: AABB, pool: &mut Vec<Box<Self>>) -> Box<Self> {
        if let Some(old) = pool.pop() {
            Self::recycle(bounds, old)
        } else {
            Box::new(Self::new(bounds))
        }
    }

    /// Does this region still contain endpoints of subproper proxies?
    pub fn contains_subproper_proxies(&self) -> bool {
        self.subproper_proxy_count > 0
    }

    /// If this region contains the given proxy, this will decrement this region's proxy count.
    ///
    /// Returns `true` if this region contained the proxy. Returns `false` otherwise.
    pub fn proper_proxy_moved_to_a_bigger_layer(&mut self, proxy_id: SAPProxyIndex) -> bool {
        if self.existing_proxies.get(proxy_id as usize) == Some(true) {
            // NOTE: we are just registering the fact that that proxy isn't a
            //       subproper proxy anymore. But it is still part of this region
            //       so we must not set `self.existing_proxies[proxy_id] = false`
            //       nor delete its endpoints.
            self.subproper_proxy_count -= 1;
            true
        } else {
            false
        }
    }

    /// Deletes from the axes of this region all the endpoints that point
    /// to a region.
    pub fn delete_all_region_endpoints(&mut self, proxies: &SAPProxies) {
        let mut num_deleted_subregion_endpoints = [0; DIM];

        for (i, axis) in self.axes.iter_mut().enumerate() {
            let existing_proxies = &mut self.existing_proxies;
            axis.endpoints.retain(|e| {
                // NOTE: we use `if let` instead of `unwrap` because no
                // proxy will be found for the sentinels.
                if let Some(proxy) = proxies.get(e.proxy()) {
                    if proxy.data.is_region() {
                        existing_proxies.set(e.proxy() as usize, false);
                        num_deleted_subregion_endpoints[i] += 1;
                        return false;
                    }
                }

                true
            });
        }

        // All axes should have deleted the same number of region endpoints.
        for k in 1..DIM {
            assert_eq!(
                num_deleted_subregion_endpoints[0],
                num_deleted_subregion_endpoints[k]
            );
        }

        // The number of deleted endpoints should be even because there
        // are two endpoints per proxy on each axes.
        assert_eq!(num_deleted_subregion_endpoints[0] % 2, 0);

        // All the region endpoints are subproper proxies.
        // So update the subproper proxy count accordingly.
        self.subproper_proxy_count -= num_deleted_subregion_endpoints[0] / 2;
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
        self.preupdate_proxy(proxy_id, true);
        subregion_index
    }

    pub fn preupdate_proxy(&mut self, proxy_id: SAPProxyIndex, is_subproper_proxy: bool) -> bool {
        let mask_len = self.existing_proxies.len();
        if proxy_id as usize >= mask_len {
            self.existing_proxies
                .grow(proxy_id as usize + 1 - mask_len, false);
        }

        if !self.existing_proxies[proxy_id as usize] {
            self.to_insert.push(proxy_id);
            self.existing_proxies.set(proxy_id as usize, true);

            if is_subproper_proxy {
                self.subproper_proxy_count += 1;
            }

            false
        } else {
            // Here we need a second update if all proxies exit this region. In this case, we need
            // to delete the final proxy, but the region may not have AABBs overlapping it, so it
            // wouldn't get an update otherwise.
            self.update_count = 2;
            true
        }
    }

    pub fn update_after_subregion_removal(&mut self, proxies: &SAPProxies, layer_depth: i8) {
        if self.needs_update_after_subregion_removal {
            for axis in &mut self.axes {
                self.subproper_proxy_count -= axis
                    .delete_deleted_proxies_and_endpoints_after_subregion_removal(
                        proxies,
                        &mut self.existing_proxies,
                        layer_depth,
                    );
            }
            self.needs_update_after_subregion_removal = false;
        }
    }

    pub fn update(
        &mut self,
        proxies: &SAPProxies,
        layer_depth: i8,
        reporting: &mut HashMap<(u32, u32), bool>,
    ) {
        if self.update_count > 0 {
            // Update endpoints.
            let mut total_deleted = 0;
            let mut total_deleted_subproper = 0;

            for dim in 0..DIM {
                self.axes[dim].update_endpoints(dim, proxies, reporting);
                let (num_deleted, num_deleted_subproper) = self.axes[dim]
                    .delete_out_of_bounds_proxies(proxies, &mut self.existing_proxies, layer_depth);
                total_deleted += num_deleted;
                total_deleted_subproper += num_deleted_subproper;
            }

            if total_deleted > 0 {
                self.subproper_proxy_count -= total_deleted_subproper;
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
