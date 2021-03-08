use super::{
    BroadPhasePairEvent, BroadPhaseProxies, BroadPhaseProxy, ColliderPair, SAPRegion,
    NEXT_FREE_SENTINEL, SENTINEL_VALUE,
};
use crate::data::pubsub::Subscription;
use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderSet, RemovedCollider};
use crate::math::{Point, Real};
use parry::bounding_volume::BoundingVolume;
use parry::utils::hashmap::HashMap;

/// A broad-phase based on multiple Sweep-and-Prune instances running of disjoint region of the 3D world.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhase {
    proxies: BroadPhaseProxies,
    regions: HashMap<Point<i32>, SAPRegion>,
    removed_colliders: Option<Subscription<RemovedCollider>>,
    deleted_any: bool,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    region_pool: Vec<SAPRegion>, // To avoid repeated allocations.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    regions_to_remove: Vec<Point<i32>>, // Workspace
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
            regions: HashMap::default(),
            region_pool: Vec::new(),
            regions_to_remove: Vec::new(),
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

        // Discretize the AABB to find the regions that need to be invalidated.
        let start = super::point_key(proxy.aabb.mins);
        let end = super::point_key(proxy.aabb.maxs);

        #[cfg(feature = "dim2")]
        for i in start.x..=end.x {
            for j in start.y..=end.y {
                if let Some(region) = self.regions.get_mut(&Point::new(i, j)) {
                    region.predelete_proxy(proxy_index);
                    self.deleted_any = true;
                }
            }
        }

        #[cfg(feature = "dim3")]
        for i in start.x..=end.x {
            for j in start.y..=end.y {
                for k in start.z..=end.z {
                    if let Some(region) = self.regions.get_mut(&Point::new(i, j, k)) {
                        region.predelete_proxy(proxy_index);
                        self.deleted_any = true;
                    }
                }
            }
        }

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

                if let Some(proxy) = self.proxies.get_mut(collider.proxy_index) {
                    proxy.aabb = aabb;
                } else {
                    let proxy = BroadPhaseProxy {
                        handle: *handle,
                        aabb,
                        next_free: NEXT_FREE_SENTINEL,
                    };
                    collider.proxy_index = self.proxies.insert(proxy);
                }

                // Discretize the aabb.
                let proxy_id = collider.proxy_index;
                // let start = Point::origin();
                // let end = Point::origin();
                let start = super::point_key(aabb.mins);
                let end = super::point_key(aabb.maxs);

                let regions = &mut self.regions;
                let pool = &mut self.region_pool;

                #[cfg(feature = "dim2")]
                for i in start.x..=end.x {
                    for j in start.y..=end.y {
                        let region_key = Point::new(i, j);
                        let region_bounds = region_aabb(region_key);
                        let region = regions
                            .entry(region_key)
                            .or_insert_with(|| SAPRegion::recycle_or_new(region_bounds, pool));
                        let _ = region.preupdate_proxy(proxy_id);
                    }
                }

                #[cfg(feature = "dim3")]
                for i in start.x..=end.x {
                    for j in start.y..=end.y {
                        for k in start.z..=end.z {
                            let region_key = Point::new(i, j, k);
                            let region_bounds = super::region_aabb(region_key);
                            let region = regions
                                .entry(region_key)
                                .or_insert_with(|| SAPRegion::recycle_or_new(region_bounds, pool));
                            let _ = region.preupdate_proxy(proxy_id);
                        }
                    }
                }
            }
        }
    }

    fn update_regions(&mut self) {
        for (point, region) in &mut self.regions {
            region.update(&self.proxies, &mut self.reporting);
            if region.proxy_count == 0 {
                self.regions_to_remove.push(*point);
            }
        }

        // Remove all the empty regions and store them in the region pool
        let regions = &mut self.regions;
        self.region_pool.extend(
            self.regions_to_remove
                .drain(..)
                .map(|p| regions.remove(&p).unwrap()),
        );
    }

    pub(crate) fn complete_removals(&mut self) {
        if self.deleted_any {
            self.update_regions();

            // NOTE: we don't care about reporting pairs.
            self.reporting.clear();
            self.deleted_any = false;
        }
    }

    pub(crate) fn find_pairs(&mut self, out_events: &mut Vec<BroadPhasePairEvent>) {
        // println!("num regions: {}", self.regions.len());

        self.reporting.clear();
        self.update_regions();

        // Convert reports to broad phase events.
        // let t = instant::now();
        // let mut num_add_events = 0;
        // let mut num_delete_events = 0;

        for ((proxy1, proxy2), colliding) in &self.reporting {
            let proxy1 = &self.proxies[*proxy1 as usize];
            let proxy2 = &self.proxies[*proxy2 as usize];

            let handle1 = proxy1.handle;
            let handle2 = proxy2.handle;

            if *colliding {
                out_events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                    handle1, handle2,
                )));
                // num_add_events += 1;
            } else {
                out_events.push(BroadPhasePairEvent::DeletePair(ColliderPair::new(
                    handle1, handle2,
                )));
                // num_delete_events += 1;
            }
        }

        // println!(
        //     "Event conversion time: {}, add: {}/{}, delete: {}/{}",
        //     instant::now() - t,
        //     num_add_events,
        //     out_events.len(),
        //     num_delete_events,
        //     out_events.len()
        // );
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
