use super::{SAPProxies, SAPProxy, SAPRegion, SAPRegionPool};
use crate::geometry::broad_phase_multi_sap::DELETED_AABB_VALUE;
use crate::geometry::{Aabb, BroadPhaseProxyIndex};
use crate::math::{Point, Real};
use parry::bounding_volume::BoundingVolume;
use parry::utils::hashmap::{Entry, HashMap};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct SAPLayer {
    pub depth: i8,
    pub layer_id: u8,
    pub smaller_layer: Option<u8>,
    pub larger_layer: Option<u8>,
    region_width: Real,
    pub regions: HashMap<Point<i32>, BroadPhaseProxyIndex>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    regions_to_potentially_remove: Vec<Point<i32>>, // Workspace
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub created_regions: Vec<BroadPhaseProxyIndex>,
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

    /// Deletes from all the regions of this layer, all the endpoints corresponding
    /// to subregions. Clears the arrays of subregions indices from all the regions of
    /// this layer.
    pub fn unregister_all_subregions(&mut self, proxies: &mut SAPProxies) {
        for region_id in self.regions.values() {
            // Extract the region to make the borrow-checker happy.
            let mut region = proxies[*region_id]
                .data
                .take_region()
                .expect("Should be a region proxy.");

            // Delete the endpoints.
            region.delete_all_region_endpoints(proxies);

            // Clear the subregions vec and reset the subregions parent ids.
            for subregion in region.subregions.drain(..) {
                proxies[subregion]
                    .data
                    .as_region_mut()
                    .id_in_parent_subregion = crate::INVALID_U32;
            }

            // Re set the region to make the borrow-checker happy.
            proxies[*region_id].data.set_region(region);
        }
    }

    /// Register into `larger_layer` all the region proxies of the recently-created regions
    /// contained by `self`.
    ///
    /// This method must be called in a bottom-up loop, propagating new regions from the
    /// smallest layer, up to the largest layer. That loop is done by the Phase 3 of the
    /// BroadPhaseMultiSap::update.
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

    /// Register into `larger_layer` all the region proxies of the region contained in `self`.
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

    /// Registers a subregion of this layer.
    ///
    /// The subregion proxy will be added to the region of `self` that contains
    /// that subregion center. Because the hierarchical grid cells have aligned boundaries
    /// at each depth, we have the guarantee that a given subregion will only be part of
    /// one region on its parent "larger" layer.
    fn register_subregion(
        &mut self,
        proxy_id: BroadPhaseProxyIndex,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        if let Some(proxy) = proxies.get(proxy_id) {
            let curr_id_in_parent_subregion = proxy.data.as_region().id_in_parent_subregion;

            if curr_id_in_parent_subregion == crate::INVALID_U32 {
                let region_key = super::point_key(proxy.aabb.center(), self.region_width);
                let region_id = self.ensure_region_exists(region_key, proxies, pool);
                let region = proxies[region_id].data.as_region_mut();

                let id_in_parent_subregion = region.register_subregion(proxy_id);
                proxies[proxy_id]
                    .data
                    .as_region_mut()
                    .id_in_parent_subregion = id_in_parent_subregion as u32;
            } else {
                // NOTE: all the following are just assertions to make sure the
                // region ids are correctly wired. If this piece of code causes
                // any performance problem, it can be deleted completely without
                // hesitation.
                if curr_id_in_parent_subregion != crate::INVALID_U32 {
                    let region_key = super::point_key(proxy.aabb.center(), self.region_width);
                    let region_id = self.regions.get(&region_key).unwrap();
                    let region = proxies[*region_id].data.as_region_mut();
                    assert_eq!(
                        region.subregions[curr_id_in_parent_subregion as usize],
                        proxy_id
                    );
                }
            }
        }
    }

    fn unregister_subregion(
        &mut self,
        proxy_id: BroadPhaseProxyIndex,
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

                let removed = region
                    .subregions
                    .swap_remove(id_in_parent_subregion as usize); // Remove the subregion index from the subregion list.
                assert_eq!(removed, proxy_id);

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

    /// Ensures a given region exists in this layer.
    ///
    /// If the region with the given region key does not exist yet, it is created.
    /// When a region is created, it creates a new proxy for that region, and its
    /// proxy ID is added to `self.created_region` so it can be propagated during
    /// the Phase 3 of `BroadPhaseMultiSap::update`.
    ///
    /// This returns the proxy ID of the already existing region if it existed, or
    /// of the new region if it did not exist and has been created by this method.
    pub fn ensure_region_exists(
        &mut self,
        region_key: Point<i32>,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) -> BroadPhaseProxyIndex {
        match self.regions.entry(region_key) {
            // Yay, the region already exists!
            Entry::Occupied(occupied) => *occupied.get(),
            // The region does not exist, create it.
            Entry::Vacant(vacant) => {
                let region_bounds = super::region_aabb(region_key, self.region_width);
                let region = SAPRegion::recycle_or_new(region_bounds, pool);
                // Create a new proxy for that region.
                let region_proxy =
                    SAPProxy::subregion(region, region_bounds, self.layer_id, self.depth);
                let region_proxy_id = proxies.insert(region_proxy);
                // Push this region's proxy ID to the set of created regions.
                self.created_regions.push(region_proxy_id as u32);
                // Insert the new region to this layer's region hashmap.
                let _ = vacant.insert(region_proxy_id);
                region_proxy_id
            }
        }
    }

    pub fn preupdate_collider(
        &mut self,
        proxy_id: u32,
        aabb_to_discretize: &Aabb,
        actual_aabb: Option<&Aabb>,
        proxies: &mut SAPProxies,
        pool: &mut SAPRegionPool,
    ) {
        let start = super::point_key(aabb_to_discretize.mins, self.region_width);
        let end = super::point_key(aabb_to_discretize.maxs, self.region_width);

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
                    let region_proxy = &mut proxies[region_id];
                    let region = region_proxy.data.as_region_mut();

                    // NOTE: sometimes, rounding errors will generate start/end indices
                    //       that lie outside of the actual region’s Aabb.
                    // TODO: is there a smarter, more efficient way of dealing with this?
                    if !region_proxy.aabb.intersects(aabb_to_discretize) {
                        continue;
                    }

                    if let Some(actual_aabb) = actual_aabb {
                        // NOTE: if the actual Aabb doesn't intersect the
                        //       region’s Aabb, then we need to delete the
                        //       proxy from that region because it means that
                        //       during the last update the proxy intersected
                        //       that region, but it doesn't intersect it any
                        //       more during the current update.
                        if !region_proxy.aabb.intersects(actual_aabb) {
                            region.predelete_proxy(proxy_id);
                            continue;
                        }
                    }

                    region.preupdate_proxy(proxy_id, true);
                }
            }
        }
    }

    pub fn predelete_proxy(&mut self, proxies: &mut SAPProxies, proxy_index: BroadPhaseProxyIndex) {
        // Discretize the Aabb to find the regions that need to be invalidated.
        let proxy_aabb = &mut proxies[proxy_index].aabb;
        let start = super::point_key(proxy_aabb.mins, self.region_width);
        let end = super::point_key(proxy_aabb.maxs, self.region_width);

        // Set the Aabb of the proxy to a very large value.
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
                region.update(proxies, self.depth, reporting);

                // Mark all subregions as to-be-updated.
                for subregion_id in &region.subregions {
                    proxies[*subregion_id].data.as_region_mut().mark_as_dirty();
                }

                if !region.contains_subproper_proxies() {
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
                    region.update_after_subregion_removal(proxies, self.depth);

                    // Check if we can actually delete this region.
                    if !region.contains_subproper_proxies() {
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

    pub fn proper_proxy_moved_to_bigger_layer(
        &mut self,
        proxies: &mut SAPProxies,
        proxy_id: BroadPhaseProxyIndex,
    ) {
        for (point, region_id) in &self.regions {
            let region = &mut proxies[*region_id].data.as_region_mut();
            let region_contains_proxy = region.proper_proxy_moved_to_a_bigger_layer(proxy_id);

            // If that proper proxy was the last one keeping that region
            // alive, mark the region as potentially removable.
            if region_contains_proxy && !region.contains_subproper_proxies() {
                self.regions_to_potentially_remove.push(*point);
            }
        }
    }
}
