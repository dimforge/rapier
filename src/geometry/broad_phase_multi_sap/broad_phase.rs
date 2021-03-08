use super::{
    BroadPhasePairEvent, BroadPhaseProxies, BroadPhaseProxy, BroadPhaseProxyData, ColliderPair,
    SAPLayer, SAPRegion, NEXT_FREE_SENTINEL, SENTINEL_VALUE,
};
use crate::data::pubsub::Subscription;
use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderSet, RemovedCollider};
use crate::math::Real;
use parry::bounding_volume::BoundingVolume;
use parry::utils::hashmap::HashMap;

/// A broad-phase based on multiple Sweep-and-Prune instances running of disjoint region of the 3D world.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhase {
    proxies: BroadPhaseProxies,
    layers: Vec<SAPLayer>,
    smallest_layer: u8,
    largest_layer: u8,
    removed_colliders: Option<Subscription<RemovedCollider>>,
    deleted_any: bool,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    region_pool: Vec<SAPRegion>, // To avoid repeated allocations.
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
            removed_colliders: None,
            proxies: BroadPhaseProxies::new(),
            layers: Vec::new(),
            smallest_layer: 0,
            largest_layer: 0,
            region_pool: Vec::new(),
            reporting: HashMap::default(),
            deleted_any: false,
        }
    }

    /// Maintain the broad-phase internal state by taking collider removal into account.
    pub fn maintain(&mut self, colliders: &mut ColliderSet) {
        // Ensure we already subscribed.
        if self.removed_colliders.is_none() {
            self.removed_colliders = Some(colliders.removed_colliders.subscribe());
        }

        let cursor = self.removed_colliders.take().unwrap();
        for collider in colliders.removed_colliders.read(&cursor) {
            self.remove_proxy(collider.proxy_index);
        }

        colliders.removed_colliders.ack(&cursor);
        self.removed_colliders = Some(cursor);
    }

    fn remove_proxy(&mut self, proxy_index: usize) {
        if proxy_index == crate::INVALID_USIZE {
            // This collider has not been added to the broad-phase yet.
            return;
        }

        let proxy = &mut self.proxies[proxy_index];
        let layer = &mut self.layers[proxy.layer_id as usize];
        layer.remove_proxy(proxy, proxy_index);

        // Push the proxy to infinity, but not beyond the sentinels.
        proxy.aabb.mins.coords.fill(SENTINEL_VALUE / 2.0);
        proxy.aabb.maxs.coords.fill(SENTINEL_VALUE / 2.0);
        self.proxies.remove(proxy_index);
    }

    fn finalize_layer_insertion(&mut self, layer_id: u8) {
        if let Some(larger_layer) = self.layers[layer_id as usize].larger_layer {
            // Remove all the region endpoints from the larger layer.
            // They will be automatically replaced by the new layer's region.
            self.layers[larger_layer as usize].delete_all_region_endpoints();
        }

        if let Some(smaller_layer) = self.layers[layer_id as usize].smaller_layer {
            let (smaller_layer, new_layer) = self
                .layers
                .index_mut2(smaller_layer as usize, layer_id as usize);

            // Add all the regions from the smaller layer to the new layer.
            // This will propagate to the bigger layers automatically.
            for smaller_region in smaller_layer.regions.iter() {
                let smaller_region_key = ???;
                new_layer.preupdate_proxy_in_region(smaller_region.proxy_id, smaller_region_key);
            }
        }
    }

    fn ensure_layer_exists(&mut self, new_depth: i8) -> u8 {
        // Special case: we don't have any layers yet.
        if self.layers.is_empty() {
            self.layers.push(SAPLayer::new(new_depth));
            return 0;
        }

        // Find the first layer with a depth larger or equal to new_depth.
        let mut larger_layer_id = Some(self.smallest_layer);

        while let Some(curr_layer_id) = larger_layer_id {
            if self.layers[curr_layer_id as usize].depth >= new_depth {
                break;
            }

            larger_layer_id = self.layers[layer as usize].larger_layer;
        }

        match larger_layer_id {
            None => {
                let new_layer_id = self.layers.len() as u8;
                self.layers[self.largest_layer as usize].larger_layer = Some(new_layer_id);
                self.largest_layer = new_layer_id;
                self.layers
                    .push(SAPLayer::new(new_depth, Some(self.largest_layer), None));
                self.finalize_layer_insertion(new_layer_id);
                new_layer_id
            }
            Some(larger_layer_id) => {
                if self.layers[larger_layer_id].depth == new_depth {
                    // Found it! The layer already exists.
                    larger_layer_id
                } else {
                    // The layer does not exist yet. Create it.
                    let new_layer_id = self.layers.len() as u8;
                    let smaller_layer_id = self.layers[larger_layer_id as usize].smaller_layer;
                    self.layers[larger_layer_id as usize].smaller_layer = Some(new_layer_id);

                    if let Some(smaller_layer_id) = smaller_layer_id {
                        self.layers[smaller_layer_id as usize].larger_layer = Some(new_layer_id);
                    }

                    self.layers.push(SAPLayer::new(
                        new_depth,
                        smaller_layer_id,
                        Some(larger_layer_id),
                    ));
                    self.finalize_layer_insertion(new_layer_id);

                    new_layer_id
                }
            }
        }
    }

    pub(crate) fn update_aabbs(
        &mut self,
        prediction_distance: Real,
        bodies: &RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        // First, if we have any pending removals we have
        // to deal with them now because otherwise we will
        // end up with an ABA problems when reusing proxy
        // ids.
        self.complete_removals();

        for body_handle in bodies
            .modified_inactive_set
            .iter()
            .chain(bodies.active_dynamic_set.iter())
            .chain(bodies.active_kinematic_set.iter())
        {
            for handle in &bodies[*body_handle].colliders {
                let collider = &mut colliders[*handle];
                let aabb = collider.compute_aabb().loosened(prediction_distance / 2.0);

                let layer_id = if let Some(proxy) = self.proxies.get_mut(collider.proxy_index) {
                    proxy.aabb = aabb;
                    proxy.layer_id
                } else {
                    let layer_depth = super::layer_containing_aabb(&aabb);
                    let layer_id = self.ensure_layer_exists(layer_depth);

                    // Create the proxy.
                    let proxy = BroadPhaseProxy {
                        data: BroadPhaseProxyData::Collider(*handle),
                        aabb,
                        next_free: NEXT_FREE_SENTINEL,
                        layer_id,
                    };
                    collider.proxy_index = self.proxies.insert(proxy);
                    layer_id
                };

                let layer = &mut self.layers[layer_id as usize];

                // Preupdate the collider in the layer.
                layer.preupdate_collider(collider, &aabb, &mut self.region_pool);
            }
        }
    }

    pub(crate) fn complete_removals(&mut self) {
        for layer in &mut self.layers {
            layer.complete_removals(&self.proxies, &mut self.reporting, &mut self.region_pool);
            // NOTE: we don't care about reporting pairs.
            self.reporting.clear();
        }
    }

    pub(crate) fn find_pairs(&mut self, out_events: &mut Vec<BroadPhasePairEvent>) {
        let mut layer_id = 0;

        while let Some(layer) = self.layers.get_mut(layer_id as usize) {
            layer.update_regions(&self.proxies, &mut self.reporting, &mut self.region_pool);
            layer_id = layer.prev_layer; // this MUST be done before the `for` loop because we use the prev_layer index there.

            for ((proxy_id1, proxy_id2), colliding) in &self.reporting {
                let proxy1 = &self.proxies[*proxy_id1 as usize];
                let proxy2 = &self.proxies[*proxy_id2 as usize];

                let handle1 = proxy1.handle;
                let handle2 = proxy2.handle;

                match (&handle1, &handle2) {
                    (
                        BroadPhaseProxyData::Collider(handle1),
                        BroadPhaseProxyData::Collider(handle2),
                    ) => {
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
                    (
                        BroadPhaseProxyData::Collider(_),
                        BroadPhaseProxyData::Subregion(region_key),
                    ) => {
                        if *colliding {
                            // Add the collider to the subregion.
                            let sublayer = &mut self.layers[layer_id as usize];
                            sublayer.preupdate_proxy_in_region(*proxy_id1, region_key);
                        }
                    }
                    (
                        BroadPhaseProxyData::Subregion(region_key),
                        BroadPhaseProxyData::Collider(_),
                    ) => {
                        if *colliding {
                            // Add the collider to the subregion.
                            let sublayer = &mut self.layers[layer_id as usize];
                            sublayer.preupdate_proxy_in_region(*proxy_id2, region_key);
                        }
                    }
                    (BroadPhaseProxyData::Subregion(_), BroadPhaseProxyData::Subregion(_)) => {
                        // This will only happen between two adjacent subregions because
                        // they share some of the same bounds. So this case does not matter.
                    }
                }
            }

            self.reporting.clear();
        }
    }
}

#[cfg(test)]
mod test {
    use crate::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
    use crate::geometry::{BroadPhase, ColliderBuilder, ColliderSet};

    #[test]
    fn test_add_update_remove() {
        let mut broad_phase = BroadPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();

        let rb = RigidBodyBuilder::new_dynamic().build();
        let co = ColliderBuilder::ball(0.5).build();
        let hrb = bodies.insert(rb);
        colliders.insert(co, hrb, &mut bodies);

        broad_phase.update_aabbs(0.0, &bodies, &mut colliders);

        bodies.remove(hrb, &mut colliders, &mut joints);
        broad_phase.maintain(&mut colliders);
        broad_phase.update_aabbs(0.0, &bodies, &mut colliders);

        // Create another body.
        let rb = RigidBodyBuilder::new_dynamic().build();
        let co = ColliderBuilder::ball(0.5).build();
        let hrb = bodies.insert(rb);
        colliders.insert(co, hrb, &mut bodies);

        // Make sure the proxy handles is recycled properly.
        broad_phase.update_aabbs(0.0, &bodies, &mut colliders);
    }
}
