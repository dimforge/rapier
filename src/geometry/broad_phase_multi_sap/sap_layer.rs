use super::{SAPProxies, SAPProxy, SAPRegion, SAPRegionPool};
use crate::geometry::broad_phase_multi_sap::DELETED_AABB_VALUE;
use crate::geometry::{Collider, SAPProxyIndex, AABB};
use crate::math::{Point, Real};
use parry::utils::hashmap::{Entry, HashMap};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct SAPLayer {
    pub depth: i8,
    pub layer_id: u8,
    pub smaller_layer: Option<u8>,
    pub larger_layer: Option<u8>,
    region_width: Real,
    pub regions: HashMap<Point<i32>, SAPProxyIndex>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    regions_to_potentially_remove: Vec<Point<i32>>, // Workspace
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub created_regions: Vec<SAPProxyIndex>,
}

impl SAPLayer {
    pub fn new(
        depth: i8,
        layer_id: u8,
        smaller_layer: Option<u8>,
        larger_layer: Option<u8>,
    ) -> Self {
        Self {
            depth,
            smaller_layer,
            larger_layer,
            layer_id,
            region_width: super::region_width(depth),
            regions: HashMap::default(),
            regions_to_potentially_remove: vec![],
            created_regions: vec![],
        }
    }

    pub fn delete_all_region_endpoints(&mut self, proxies: &mut SAPProxies) {
        for region_id in self.regions.values() {
            if let Some(mut region) = proxies[*region_id].data.take_region() {
                region.delete_all_region_endpoints(proxies);
                proxies[*region_id].data.set_region(region);
            }
        }
    }

    pub fn propagate_created_regions(
        &mut self,
        larger_layer: &mut Self,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        for proxy_id in self.created_regions.drain(..) {
            larger_layer.register_subregion(proxy_id, proxies, pool)
        }
    }

    pub fn propagate_existing_regions(
        &mut self,
        larger_layer: &mut Self,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        for proxy_id in self.regions.values() {
            larger_layer.register_subregion(*proxy_id, proxies, pool)
        }
    }

    // Preupdates the proxy of a subregion.
    fn register_subregion(
        &mut self,
        proxy_id: SAPProxyIndex,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        if let Some(proxy) = proxies.get(proxy_id) {
            let region_key = super::point_key(proxy.aabb.center(), self.region_width);
            let region_id = self.ensure_region_exists(region_key, proxies, pool);
            let region = proxies[region_id].data.as_region_mut();
            let id_in_parent_subregion = region.register_subregion(proxy_id);
            proxies[proxy_id]
                .data
                .as_region_mut()
                .id_in_parent_subregion = id_in_parent_subregion as u32;
        }
    }

    fn unregister_subregion(
        &mut self,
        proxy_id: SAPProxyIndex,
        proxy_region: &SAPRegion,
        proxies: &mut SAPProxies,
    ) {
        if let Some(proxy) = proxies.get(proxy_id) {
            let id_in_parent_subregion = proxy_region.id_in_parent_subregion;
            let region_key = super::point_key(proxy.aabb.center(), self.region_width);

            if let Some(region_id) = self.regions.get(&region_key) {
                let proxy = &mut proxies[*region_id];
                let region = proxy.data.as_region_mut();
                if !region.needs_update_after_subregion_removal {
                    self.regions_to_potentially_remove.push(region_key);
                    region.needs_update_after_subregion_removal = true;
                }

                region
                    .subregions
                    .swap_remove(id_in_parent_subregion as usize); // Remove the subregion index from the subregion list.

                // Re-adjust the id_in_parent_subregion of the subregion that was swapped in place
                // of the deleted one.
                if let Some(subregion_to_update) = region
                    .subregions
                    .get(id_in_parent_subregion as usize)
                    .copied()
                {
                    proxies[subregion_to_update]
                        .data
                        .as_region_mut()
                        .id_in_parent_subregion = id_in_parent_subregion;
                }
            }
        }
    }

    pub fn ensure_region_exists(
        &mut self,
        region_key: Point<i32>,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) -> SAPProxyIndex {
        match self.regions.entry(region_key) {
            Entry::Occupied(occupied) => *occupied.get(),
            Entry::Vacant(vacant) => {
                let region_bounds = super::region_aabb(region_key, self.region_width);
                let region = SAPRegion::recycle_or_new(region_bounds, pool);
                let region_proxy = SAPProxy::subregion(region, region_bounds, self.layer_id);
                let region_proxy_id = proxies.insert(region_proxy);
                self.created_regions.push(region_proxy_id as u32);
                let _ = vacant.insert(region_proxy_id);
                region_proxy_id
            }
        }
    }

    pub fn preupdate_collider(
        &mut self,
        collider: &Collider,
        aabb: &AABB,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        let proxy_id = collider.proxy_index;
        let start = super::point_key(aabb.mins, self.region_width);
        let end = super::point_key(aabb.maxs, self.region_width);

        // Discretize the aabb.
        #[cfg(feature = "dim2")]
        let k_range = 0..1;
        #[cfg(feature = "dim3")]
        let k_range = start.z..=end.z;

        for i in start.x..=end.x {
            for j in start.y..=end.y {
                for _k in k_range.clone() {
                    #[cfg(feature = "dim2")]
                    let region_key = Point::new(i, j);
                    #[cfg(feature = "dim3")]
                    let region_key = Point::new(i, j, _k);
                    let region_id = self.ensure_region_exists(region_key, proxies, pool);
                    let region = proxies[region_id].data.as_region_mut();
                    region.preupdate_proxy(proxy_id);
                }
            }
        }
    }

    pub fn predelete_proxy(&mut self, proxies: &mut SAPProxies, proxy_index: SAPProxyIndex) {
        // Discretize the AABB to find the regions that need to be invalidated.
        let proxy_aabb = &mut proxies[proxy_index].aabb;
        let start = super::point_key(proxy_aabb.mins, self.region_width);
        let end = super::point_key(proxy_aabb.maxs, self.region_width);

        // Set the AABB of the proxy to a very large value.
        proxy_aabb.mins.coords.fill(DELETED_AABB_VALUE);
        proxy_aabb.maxs.coords.fill(DELETED_AABB_VALUE);

        #[cfg(feature = "dim2")]
        let k_range = 0..1;
        #[cfg(feature = "dim3")]
        let k_range = start.z..=end.z;

        for i in start.x..=end.x {
            for j in start.y..=end.y {
                for _k in k_range.clone() {
                    #[cfg(feature = "dim2")]
                    let key = Point::new(i, j);
                    #[cfg(feature = "dim3")]
                    let key = Point::new(i, j, _k);
                    if let Some(region_id) = self.regions.get(&key) {
                        let region = proxies[*region_id].data.as_region_mut();
                        region.predelete_proxy(proxy_index);
                    }
                }
            }
        }
    }

    pub fn update_regions(
        &mut self,
        proxies: &mut SAPProxies,
        reporting: &mut HashMap<(u32, u32), bool>,
    ) {
        // println!(
        //     "Num regions at depth {}: {}",
        //     self.depth,
        //     self.regions.len(),
        // );
        for (point, region_id) in &self.regions {
            if let Some(mut region) = proxies[*region_id].data.take_region() {
                // Update the region.
                region.update(proxies, reporting);

                // Mark all subregions as to-be-updated.
                for subregion_id in &region.subregions {
                    proxies[*subregion_id].data.as_region_mut().mark_as_dirty();
                }

                if region.proxy_count == 0 {
                    self.regions_to_potentially_remove.push(*point);
                }

                proxies[*region_id].data.set_region(region);
            }
        }
    }

    /// Complete the removals of empty regions on this layer.
    ///
    /// This method must be called in a bottom-up fashion, i.e.,
    /// by starting with the smallest layer up to the larger layer.
    /// This is needed in order to properly propagate the removal
    /// of a region (which is a subregion registered into the larger
    /// layer).
    pub fn complete_removals(
        &mut self,
        mut larger_layer: Option<&mut Self>,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        // Delete all the empty regions and store them in the region pool.
        for region_to_remove in self.regions_to_potentially_remove.drain(..) {
            if let Entry::Occupied(region_id) = self.regions.entry(region_to_remove) {
                if let Some(proxy) = proxies.get_mut(*region_id.get()) {
                    // First, we need to remove the endpoints of the deleted subregions.
                    let mut region = proxy.data.take_region().unwrap();
                    region.update_after_subregion_removal(proxies);

                    // Check if we actually can delete this region.
                    if region.proxy_count == 0 {
                        let region_id = region_id.remove();

                        // We can delete this region. So we need to tell the larger
                        // layer that one of its subregion is being deleted.
                        // The next call to `complete_removals` on the larger layer
                        // will take care of updating its own regions by taking into
                        // account this deleted subregion.
                        if let Some(larger_layer) = &mut larger_layer {
                            larger_layer.unregister_subregion(region_id, &region, proxies);
                        }

                        // Move the proxy to infinity.
                        let proxy = &mut proxies[region_id];
                        proxy.aabb.mins.coords.fill(DELETED_AABB_VALUE);
                        proxy.aabb.maxs.coords.fill(DELETED_AABB_VALUE);

                        // Mark the proxy as deleted.
                        proxies.remove(region_id);
                        pool.push(region);
                    } else {
                        proxies[*region_id.get()].data.set_region(region);
                    }
                }
            }
        }
    }
}
