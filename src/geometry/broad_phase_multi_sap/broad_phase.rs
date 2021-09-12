use super::{
    BroadPhasePairEvent, ColliderPair, SAPLayer, SAPProxies, SAPProxy, SAPProxyData, SAPRegionPool,
};
use crate::geometry::broad_phase_multi_sap::SAPProxyIndex;
use crate::geometry::{
    ColliderBroadPhaseData, ColliderChanges, ColliderHandle, ColliderPosition, ColliderShape,
};
use crate::math::Real;
use crate::utils::IndexMut2;
use parry::bounding_volume::BoundingVolume;
use parry::utils::hashmap::HashMap;

use crate::data::{BundleSet, ComponentSet, ComponentSetMut};

/// A broad-phase combining a Hierarchical Grid and Sweep-and-Prune.
///
/// The basic Sweep-and-Prune (SAP) algorithm has one significant flaws:
/// the interactions between far-away objects. This means that objects
/// that are very far away will still have some of their endpoints swapped
/// within the SAP data-structure. This results in poor scaling because this
/// results in lots of swapping between endpoints of AABBs that won't ever
/// actually interact.
///
/// The first optimization to address this problem is to use the Multi-SAP
/// method. This basically combines an SAP with a grid. The grid subdivides
/// the spaces into equally-sized subspaces (grid cells). Each subspace, which we call
/// a "region" contains an SAP instance (i.e. there SAP axes responsible for
/// collecting endpoints and swapping them when they move to detect interaction pairs).
/// Each AABB is inserted in all the regions it intersects.
/// This prevents the far-away problem because two objects that are far away will
/// be located on different regions. So their endpoints will never meed.
///
/// However, the Multi-SAP approach has one notable problem: the region size must
/// be chosen wisely. It could be user-defined, but that's makes it more difficult
/// to use (for the end-user). Or it can be given a fixed value. Using a fixed
/// value may result in large objects intersecting lots of regions, resulting in
/// poor performances and very high memory usage.
///
/// So a solution to that large-objects problem is the Multi-SAP approach is to
/// replace the grid by a hierarchical grid. A hierarchical grid is composed of
/// several layers. And each layer have different region sizes. For example all
/// the regions on layer 0 will have the size 1x1x1. All the regions on the layer
/// 1 will have the size 10x10x10, etc. That way, a given AABB will be inserted
/// on the layer that has regions big enough to avoid the large-object problem.
/// For example a 20x20x20 object will be inserted in the layer with region
/// of size 10x10x10, resulting in only 8 regions being intersect by the AABB.
/// (If it was inserted in the layer with regions of size 1x1x1, it would have intersected
/// 8000 regions, which is a problem performancewise.)
///
/// We call this new method the Hierarchical-SAP.
///
/// Now with the Hierarchical-SAP, we can update each layer independently from one another.
/// However, objects belonging to different layers will never be detected as intersecting that
/// way. So we need a way to do inter-layer interference detection. There is a lot ways of doing
/// this: performing inter-layer Multi-Box-Pruning passes is one example (but this is not what we do).
/// In our implementation, we do the following:
/// - The AABB bounds of each region of the layer `n` are inserted into the corresponding larger region
///   of the layer `n + 1`.
/// - When an AABB in the region of the layer `n + 1` intersects the AABB corresponding to one of the
///   regions at the smaller layer `n`, we add that AABB to that smaller region.
/// So in the end it means that a given AABB will be inserted into all the region it intersects at
/// the layer `n`. And it will also be inserted into all the regions it intersects at the smaller layers
/// (the layers `< n`), but only for the regions that already exist (so we don't have to discretize
/// our AABB into the layers `< n`). This involves a fair amount of bookkeeping unfortunately, but
/// this has the benefit of keep the overall complexity of the algorithm O(1) in the typical specially
/// coherent scenario.
///
/// From an implementation point-of-view, our hierarchical SAP is implemented with the following structures:
/// - There is one `SAPLayer` per layer of the hierarchical grid.
/// - Each `SAPLayer` contains multiple `SAPRegion` (each being a region of the grid represented by that layer).
/// - Each `SAPRegion` contains three `SAPAxis`, representing the "classical" SAP algorithm running on this region.
/// - Each `SAPAxis` maintains a sorted list of `SAPEndpoints` representing the endpoints of the AABBs intersecting
///   the bounds on the `SAPRegion` containing this `SAPAxis`.
/// - A set of `SAPProxy` are maintained separately. It contains the AABBs of all the colliders managed by this
///   broad-phase, as well as the AABBs of all the regions part of this broad-phase.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhase {
    proxies: SAPProxies,
    layers: Vec<SAPLayer>,
    smallest_layer: u8,
    largest_layer: u8,
    deleted_any: bool,
    // NOTE: we maintain this hashmap to simplify collider removal.
    //       This information is also present in the ColliderProxyId
    //       component. However if that component is removed, we need
    //       a way to access it to do some cleanup.
    //       Note that we could just remove the ColliderProxyId component
    //       altogether but that would be slow because of the need to
    //       always access this hashmap. Instead, we access this hashmap
    //       only when the collider has been added/removed.
    //       Another alternative would be to remove ColliderProxyId and
    //       just use a Coarena. But this seems like it could use too
    //       much memory.
    colliders_proxy_ids: HashMap<ColliderHandle, SAPProxyIndex>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    region_pool: SAPRegionPool, // To avoid repeated allocations.
    // We could think serializing this workspace is useless.
    // It turns out is is important to serialize at least its capacity
    // and restore this capacity when deserializing the hashmap.
    // This is because the order of future elements inserted into the
    // hashmap depends on its capacity (because the internal bucket indices
    // depend on this capacity). So not restoring this capacity may alter
    // the order at which future elements are reported. This will in turn
    // alter the order at which the pairs are registered in the narrow-phase,
    // thus altering the order of the contact manifold. In the end, this
    // alters the order of the resolution of contacts, resulting in
    // diverging simulation after restoration of a snapshot.
    #[cfg_attr(
        feature = "serde-serialize",
        serde(
            serialize_with = "parry::utils::hashmap::serialize_hashmap_capacity",
            deserialize_with = "parry::utils::hashmap::deserialize_hashmap_capacity"
        )
    )]
    reporting: HashMap<(u32, u32), bool>, // Workspace
}

impl BroadPhase {
    /// Create a new empty broad-phase.
    pub fn new() -> Self {
        BroadPhase {
            proxies: SAPProxies::new(),
            layers: Vec::new(),
            smallest_layer: 0,
            largest_layer: 0,
            region_pool: Vec::new(),
            reporting: HashMap::default(),
            colliders_proxy_ids: HashMap::default(),
            deleted_any: false,
        }
    }

    /// Maintain the broad-phase internal state by taking collider removal into account.
    ///
    /// For each colliders marked as removed, we make their containing layer mark
    /// its proxy as pre-deleted. The actual proxy removal will happen at the end
    /// of the `BroadPhase::update`.
    fn handle_removed_colliders(&mut self, removed_colliders: &[ColliderHandle]) {
        // For each removed collider, remove the corresponding proxy.
        for removed in removed_colliders {
            if let Some(proxy_id) = self.colliders_proxy_ids.get(removed).copied() {
                self.predelete_proxy(proxy_id);
            }
        }
    }

    /// Pre-deletes a proxy from this broad-phase.
    ///
    /// The removal of a proxy is a semi-lazy process. It will mark
    /// the proxy as predeleted, and will set its AABB as +infinity.
    /// After this method has been called with all the proxies to
    /// remove, the `complete_removal` method MUST be called to
    /// complete the removal of these proxies, by actually removing them
    /// from all the relevant layers/regions/axes.
    fn predelete_proxy(&mut self, proxy_index: SAPProxyIndex) {
        if proxy_index == crate::INVALID_U32 {
            // This collider has not been added to the broad-phase yet.
            return;
        }

        let proxy = &mut self.proxies[proxy_index];
        let layer = &mut self.layers[proxy.layer_id as usize];
        // Let the layer know that the proxy is being deleted.
        layer.predelete_proxy(&mut self.proxies, proxy_index);
    }

    /// Completes the removal of the deleted proxies.
    ///
    /// If `self.predelete_proxy` was called, then this `complete_removals`
    /// method must be called to complete the removals.
    ///
    /// This method will actually remove from the proxy list all the proxies
    /// marked as deletable by `self.predelete_proxy`, making their proxy
    /// handles re-usable by new proxies.
    fn complete_removals(&mut self, removed_colliders: &[ColliderHandle]) {
        // If there is no layer, there is nothing to remove.
        if self.layers.is_empty() {
            return;
        }

        // This is a bottom-up pass:
        // - Complete the removal on the layer `n`. This may cause so regions to be deleted.
        // - Continue with the layer `n + 1`. This will delete from `n + 1` all the proxies
        //   of the regions originating from `n`.
        // This bottom-up approach will propagate region removal from the smallest layer up
        // to the largest layer.
        let mut curr_layer_id = self.smallest_layer;

        loop {
            let curr_layer = &mut self.layers[curr_layer_id as usize];

            if let Some(larger_layer_id) = curr_layer.larger_layer {
                let (curr_layer, larger_layer) = self
                    .layers
                    .index_mut2(curr_layer_id as usize, larger_layer_id as usize);
                curr_layer.complete_removals(
                    Some(larger_layer),
                    &mut self.proxies,
                    &mut self.region_pool,
                );

                // NOTE: we don't care about reporting pairs.
                self.reporting.clear();
                curr_layer_id = larger_layer_id;
            } else {
                curr_layer.complete_removals(None, &mut self.proxies, &mut self.region_pool);

                // NOTE: we don't care about reporting pairs.
                self.reporting.clear();
                break;
            }
        }

        /*
         * Actually remove the colliders proxies.
         */
        for removed in removed_colliders {
            if let Some(proxy_id) = self.colliders_proxy_ids.remove(removed) {
                if proxy_id != crate::INVALID_U32 {
                    self.proxies.remove(proxy_id);
                }
            }
        }
    }

    /// Finalize the insertion of the layer identified by `layer_id`.
    ///
    /// This will:
    /// - Remove all the subregion proxies from the larger layer.
    /// - Pre-insert all the smaller layer's region proxies into this layer.
    fn finalize_layer_insertion(&mut self, layer_id: u8) {
        // Remove all the region endpoints from the larger layer.
        // They will be automatically replaced by the new layer's regions.
        if let Some(larger_layer) = self.layers[layer_id as usize].larger_layer {
            self.layers[larger_layer as usize].unregister_all_subregions(&mut self.proxies);
        }

        // Add all the regions from the smaller layer to the new layer.
        // This will result in new regions to be created in the new layer.
        // These new regions will automatically propagate to the larger layers in
        // the Phase 3 of `Self::update`.
        if let Some(smaller_layer) = self.layers[layer_id as usize].smaller_layer {
            let (smaller_layer, new_layer) = self
                .layers
                .index_mut2(smaller_layer as usize, layer_id as usize);

            smaller_layer.propagate_existing_regions(
                new_layer,
                &mut self.proxies,
                &mut self.region_pool,
            );
        }
    }

    /// Ensures that a given layer exists.
    ///
    /// If the layer does not exist then:
    /// 1. It is created and added to `self.layers`.
    /// 2. The smaller/larger layer indices are updated to order them
    ///    properly depending on their depth.
    /// 3. All the subregion proxies from the larger layer are deleted:
    ///    they will be replaced by this new layer's regions later in
    ///    the `update` function.
    /// 4. All the regions from the smaller layer are added to that new
    ///    layer.
    fn ensure_layer_exists(&mut self, new_depth: i8) -> u8 {
        // Special case: we don't have any layers yet.
        if self.layers.is_empty() {
            let layer_id = self.layers.len() as u8; // TODO: check overflow.
            self.layers
                .push(SAPLayer::new(new_depth, layer_id, None, None));
            return 0;
        }

        // Find the first layer with a depth larger or equal to new_depth.
        let mut larger_layer_id = Some(self.smallest_layer);

        while let Some(curr_layer_id) = larger_layer_id {
            if self.layers[curr_layer_id as usize].depth >= new_depth {
                break;
            }

            larger_layer_id = self.layers[curr_layer_id as usize].larger_layer;
        }

        match larger_layer_id {
            None => {
                // The layer we are currently creating is the new largest layer. So
                // we need to update `self.largest_layer` accordingly then call
                // `self.finalize_layer_insertion.
                assert_ne!(self.layers.len() as u8, u8::MAX, "Not yet implemented.");
                let new_layer_id = self.layers.len() as u8;
                self.layers[self.largest_layer as usize].larger_layer = Some(new_layer_id);
                self.layers.push(SAPLayer::new(
                    new_depth,
                    new_layer_id,
                    Some(self.largest_layer),
                    None,
                ));
                self.largest_layer = new_layer_id;
                self.finalize_layer_insertion(new_layer_id);
                new_layer_id
            }
            Some(larger_layer_id) => {
                if self.layers[larger_layer_id as usize].depth == new_depth {
                    // Found it! The layer already exists.
                    larger_layer_id
                } else {
                    // The layer does not exist yet. Create it.
                    // And we found another layer that is larger than this one.
                    // So we need to adjust the smaller/larger layer indices too
                    // keep the list sorted, and then call `self.finalize_layer_insertion`
                    // to deal with region propagation.
                    let new_layer_id = self.layers.len() as u8;
                    let smaller_layer_id = self.layers[larger_layer_id as usize].smaller_layer;
                    self.layers[larger_layer_id as usize].smaller_layer = Some(new_layer_id);

                    if let Some(smaller_layer_id) = smaller_layer_id {
                        self.layers[smaller_layer_id as usize].larger_layer = Some(new_layer_id);
                    } else {
                        self.smallest_layer = new_layer_id;
                    }

                    self.layers.push(SAPLayer::new(
                        new_depth,
                        new_layer_id,
                        smaller_layer_id,
                        Some(larger_layer_id),
                    ));
                    self.finalize_layer_insertion(new_layer_id);

                    new_layer_id
                }
            }
        }
    }

    fn handle_modified_collider(
        &mut self,
        prediction_distance: Real,
        handle: ColliderHandle,
        proxy_index: &mut u32,
        collider: (&ColliderPosition, &ColliderShape, &ColliderChanges),
    ) -> bool {
        let (co_pos, co_shape, co_changes) = collider;

        let mut aabb = co_shape
            .compute_aabb(co_pos)
            .loosened(prediction_distance / 2.0);

        aabb.mins = super::clamp_point(aabb.mins);
        aabb.maxs = super::clamp_point(aabb.maxs);
        let prev_aabb;

        let layer_id = if let Some(proxy) = self.proxies.get_mut(*proxy_index) {
            let mut layer_id = proxy.layer_id;
            prev_aabb = proxy.aabb;
            proxy.aabb = aabb;

            if co_changes.contains(ColliderChanges::SHAPE) {
                // If the shape was changed, then we need to see if this proxy should be
                // migrated to a larger layer. Indeed, if the shape was replaced by
                // a much larger shape, we need to promote the proxy to a bigger layer
                // to avoid the O(n²) discretization problem.
                let new_layer_depth = super::layer_containing_aabb(&aabb);
                if new_layer_depth > proxy.layer_depth {
                    self.layers[proxy.layer_id as usize]
                        .proper_proxy_moved_to_bigger_layer(&mut self.proxies, *proxy_index);

                    // We need to promote the proxy to the bigger layer.
                    layer_id = self.ensure_layer_exists(new_layer_depth);
                    self.proxies[*proxy_index].layer_id = layer_id;
                    self.proxies[*proxy_index].layer_depth = new_layer_depth;
                }
            }

            layer_id
        } else {
            let layer_depth = super::layer_containing_aabb(&aabb);
            let layer_id = self.ensure_layer_exists(layer_depth);

            // Create the proxy.
            let proxy = SAPProxy::collider(handle, aabb, layer_id, layer_depth);
            prev_aabb = aabb;
            *proxy_index = self.proxies.insert(proxy);
            layer_id
        };

        let layer = &mut self.layers[layer_id as usize];

        // Preupdate the collider in the layer.
        // We need to use both the prev AABB and the new AABB for this update, to
        // handle special cases where one AABB has left a region that doesn’t contain
        // any other modified AABBs.
        // If the combination of both previous and new aabbs isn’t more than 25% bigger
        // than the new AABB, we just merge them to save some computation times (to avoid
        // discretizing twice the area at their intersection. If it’s bigger than 25% then
        // we discretize both aabbs individually.
        let merged_aabbs = prev_aabb.merged(&aabb);

        if merged_aabbs.volume() > aabb.volume() * 1.25 {
            layer.preupdate_collider(
                *proxy_index,
                &aabb,
                None,
                &mut self.proxies,
                &mut self.region_pool,
            );

            layer.preupdate_collider(
                *proxy_index,
                &prev_aabb,
                Some(&aabb),
                &mut self.proxies,
                &mut self.region_pool,
            );
        } else {
            layer.preupdate_collider(
                *proxy_index,
                &merged_aabbs,
                Some(&aabb),
                &mut self.proxies,
                &mut self.region_pool,
            );
        }

        let need_region_propagation = !layer.created_regions.is_empty();

        need_region_propagation
    }

    /// Updates the broad-phase, taking into account the new collider positions.
    pub fn update<Colliders>(
        &mut self,
        prediction_distance: Real,
        colliders: &mut Colliders,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) where
        Colliders: ComponentSetMut<ColliderBroadPhaseData>
            + ComponentSet<ColliderChanges>
            + ComponentSet<ColliderPosition>
            + ComponentSet<ColliderShape>,
    {
        // Phase 1: pre-delete the collisions that have been deleted.
        self.handle_removed_colliders(removed_colliders);

        let mut need_region_propagation = false;

        // Phase 2: pre-delete the collisions that have been deleted.
        for handle in modified_colliders {
            // NOTE: we use `get` because the collider may no longer
            //       exist if it has been removed.
            let co_changes: Option<&ColliderChanges> = colliders.get(handle.0);

            if let Some(co_changes) = co_changes {
                let (co_bf_data, co_pos, co_shape): (
                    &ColliderBroadPhaseData,
                    &ColliderPosition,
                    &ColliderShape,
                ) = colliders.index_bundle(handle.0);

                if !co_changes.needs_broad_phase_update() {
                    return;
                }
                let mut new_proxy_id = co_bf_data.proxy_index;

                if self.handle_modified_collider(
                    prediction_distance,
                    *handle,
                    &mut new_proxy_id,
                    (co_pos, co_shape, co_changes),
                ) {
                    need_region_propagation = true;
                }

                if co_bf_data.proxy_index != new_proxy_id {
                    self.colliders_proxy_ids.insert(*handle, new_proxy_id);

                    // Make sure we have the new proxy index in case
                    // the collider was added for the first time.
                    colliders.set_internal(
                        handle.0,
                        ColliderBroadPhaseData {
                            proxy_index: new_proxy_id,
                        },
                    );
                }
            }
        }

        // Phase 3: bottom-up pass to propagate new regions from smaller layers to larger layers.
        if need_region_propagation {
            self.propagate_created_regions();
        }

        // Phase 4: top-down pass to propagate proxies from larger layers to smaller layers.
        self.update_layers_and_find_pairs(events);

        // Phase 5: bottom-up pass to remove proxies, and propagate region removed from smaller
        // layers to possible remove regions from larger layers that would become empty that way.
        self.complete_removals(removed_colliders);
    }

    /// Propagate regions from the smallest layers up to the larger layers.
    ///
    /// Whenever a region is created on a layer `n`, then its AABB must be
    /// added to its larger layer so we can detect when an object
    /// in a larger layer may start interacting with objects in a smaller
    /// layer.
    fn propagate_created_regions(&mut self) {
        let mut curr_layer = Some(self.smallest_layer);

        while let Some(curr_layer_id) = curr_layer {
            let layer = &mut self.layers[curr_layer_id as usize];
            let larger_layer = layer.larger_layer;

            if !layer.created_regions.is_empty() {
                if let Some(larger_layer) = larger_layer {
                    let (layer, larger_layer) = self
                        .layers
                        .index_mut2(curr_layer_id as usize, larger_layer as usize);
                    layer.propagate_created_regions(
                        larger_layer,
                        &mut self.proxies,
                        &mut self.region_pool,
                    );
                    layer.created_regions.clear();
                } else {
                    // Always clear the set of created regions, even if
                    // there is no larger layer.
                    layer.created_regions.clear();
                }
            }

            curr_layer = larger_layer;
        }
    }

    fn update_layers_and_find_pairs(&mut self, out_events: &mut Vec<BroadPhasePairEvent>) {
        if self.layers.is_empty() {
            return;
        }

        // This is a top-down operation: we start by updating the largest
        // layer, and continue down to the smallest layer.
        //
        // This must be top-down because:
        // 1. If a non-region proxy from the layer `n` interacts with one of
        //    the regions from the layer `n - 1`, it must be added to that
        //    smaller layer before that smaller layer is updated.
        // 2. If a region has been updated, then all its subregions at the
        //    layer `n - 1` must be marked as "needs-to-be-updated" too, in
        //    order to account for the fact that a big proxy moved.
        // NOTE: this 2nd point could probably be improved: instead of updating
        //       all the subregions, we could perhaps just update the subregions
        //       that crosses the boundary of the AABB of the big proxies that
        //       moved in they layer `n`.
        let mut layer_id = Some(self.largest_layer);

        while let Some(curr_layer_id) = layer_id {
            self.layers[curr_layer_id as usize]
                .update_regions(&mut self.proxies, &mut self.reporting);

            layer_id = self.layers[curr_layer_id as usize].smaller_layer;

            for ((proxy_id1, proxy_id2), colliding) in &self.reporting {
                let (proxy1, proxy2) = self
                    .proxies
                    .elements
                    .index_mut2(*proxy_id1 as usize, *proxy_id2 as usize);

                match (&mut proxy1.data, &mut proxy2.data) {
                    (SAPProxyData::Collider(handle1), SAPProxyData::Collider(handle2)) => {
                        if *colliding {
                            out_events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                                *handle1, *handle2,
                            )));
                        } else {
                            out_events.push(BroadPhasePairEvent::DeletePair(ColliderPair::new(
                                *handle1, *handle2,
                            )));
                        }
                    }
                    (SAPProxyData::Collider(_), SAPProxyData::Region(_)) => {
                        if *colliding {
                            // Add the collider to the subregion.
                            proxy2
                                .data
                                .as_region_mut()
                                .preupdate_proxy(*proxy_id1, false);
                        }
                    }
                    (SAPProxyData::Region(_), SAPProxyData::Collider(_)) => {
                        if *colliding {
                            // Add the collider to the subregion.
                            proxy1
                                .data
                                .as_region_mut()
                                .preupdate_proxy(*proxy_id2, false);
                        }
                    }
                    (SAPProxyData::Region(_), SAPProxyData::Region(_)) => {
                        // This will only happen between two adjacent subregions because
                        // they share some identical bounds. So this case does not matter.
                    }
                }
            }

            self.reporting.clear();
        }
    }
}

#[cfg(test)]
mod test {
    use crate::dynamics::{IslandManager, JointSet, RigidBodyBuilder, RigidBodySet};
    use crate::geometry::{BroadPhase, ColliderBuilder, ColliderSet};

    #[test]
    fn test_add_update_remove() {
        let mut broad_phase = BroadPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();
        let mut islands = IslandManager::new();

        let rb = RigidBodyBuilder::new_dynamic().build();
        let co = ColliderBuilder::ball(0.5).build();
        let hrb = bodies.insert(rb);
        let coh = colliders.insert_with_parent(co, hrb, &mut bodies);

        let mut events = Vec::new();
        broad_phase.update(0.0, &mut colliders, &[coh], &[], &mut events);

        bodies.remove(hrb, &mut islands, &mut colliders, &mut joints);
        broad_phase.update(0.0, &mut colliders, &[], &[coh], &mut events);

        // Create another body.
        let rb = RigidBodyBuilder::new_dynamic().build();
        let co = ColliderBuilder::ball(0.5).build();
        let hrb = bodies.insert(rb);
        let coh = colliders.insert_with_parent(co, hrb, &mut bodies);

        // Make sure the proxy handles is recycled properly.
        broad_phase.update(0.0, &mut colliders, &[coh], &[], &mut events);
    }
}
