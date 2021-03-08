use super::{
    BroadPhasePairEvent, BroadPhaseProxies, BroadPhaseProxy, ColliderPair, SAPLayer, SAPRegion,
    NEXT_FREE_SENTINEL, SENTINEL_VALUE,
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
            layers: vec![SAPLayer::new(0)],
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
            self.remove_collider(collider.proxy_index);
        }

        colliders.removed_colliders.ack(&cursor);
        self.removed_colliders = Some(cursor);
    }

    fn remove_collider(&mut self, proxy_index: usize) {
        if proxy_index == crate::INVALID_USIZE {
            // This collider has not been added to the broad-phase yet.
            return;
        }

        let proxy = &mut self.proxies[proxy_index];

        let layer = &mut self.layers[proxy.layer as usize];
        layer.remove_collider(proxy, proxy_index);

        // Push the proxy to infinity, but not beyond the sentinels.
        proxy.aabb.mins.coords.fill(SENTINEL_VALUE / 2.0);
        proxy.aabb.maxs.coords.fill(SENTINEL_VALUE / 2.0);
        self.proxies.remove(proxy_index);
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

                let layer = if let Some(proxy) = self.proxies.get_mut(collider.proxy_index) {
                    proxy.aabb = aabb;
                    proxy.layer
                } else {
                    let layer = 0; // FIXME: compute the actual layer.
                    let proxy = BroadPhaseProxy {
                        handle: *handle,
                        aabb,
                        next_free: NEXT_FREE_SENTINEL,
                        layer,
                    };
                    collider.proxy_index = self.proxies.insert(proxy);
                    layer
                };

                let layer = &mut self.layers[layer as usize];
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
        self.reporting.clear();

        for layer in &mut self.layers {
            layer.update_regions(&self.proxies, &mut self.reporting, &mut self.region_pool);
        }

        for ((proxy1, proxy2), colliding) in &self.reporting {
            let proxy1 = &self.proxies[*proxy1 as usize];
            let proxy2 = &self.proxies[*proxy2 as usize];

            let handle1 = proxy1.handle;
            let handle2 = proxy2.handle;

            if *colliding {
                out_events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                    handle1, handle2,
                )));
            } else {
                out_events.push(BroadPhasePairEvent::DeletePair(ColliderPair::new(
                    handle1, handle2,
                )));
            }
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
