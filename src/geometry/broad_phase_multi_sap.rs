use crate::data::hashmap::HashMap;
use crate::data::pubsub::Subscription;
use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderHandle, ColliderSet, RemovedCollider};
use crate::math::{Point, Vector, DIM};
use bit_vec::BitVec;
use ncollide::bounding_volume::{BoundingVolume, AABB};
use std::cmp::Ordering;
use std::ops::{Index, IndexMut};

const NUM_SENTINELS: usize = 1;
const NEXT_FREE_SENTINEL: u32 = u32::MAX;
const SENTINEL_VALUE: f32 = f32::MAX;
const CELL_WIDTH: f32 = 20.0;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderPair {
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle,
}

impl ColliderPair {
    pub fn new(collider1: ColliderHandle, collider2: ColliderHandle) -> Self {
        ColliderPair {
            collider1,
            collider2,
        }
    }

    pub fn new_sorted(collider1: ColliderHandle, collider2: ColliderHandle) -> Self {
        if collider1.into_raw_parts().0 <= collider2.into_raw_parts().0 {
            Self::new(collider1, collider2)
        } else {
            Self::new(collider2, collider1)
        }
    }

    pub fn swap(self) -> Self {
        Self::new(self.collider2, self.collider1)
    }

    pub fn zero() -> Self {
        Self {
            collider1: ColliderHandle::from_raw_parts(0, 0),
            collider2: ColliderHandle::from_raw_parts(0, 0),
        }
    }
}

pub enum BroadPhasePairEvent {
    AddPair(ColliderPair),
    DeletePair(ColliderPair),
}

fn sort2(a: u32, b: u32) -> (u32, u32) {
    assert_ne!(a, b);

    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn point_key(point: Point<f32>) -> Point<i32> {
    (point / CELL_WIDTH).coords.map(|e| e.floor() as i32).into()
}

fn region_aabb(index: Point<i32>) -> AABB<f32> {
    let mins = index.coords.map(|i| i as f32 * CELL_WIDTH).into();
    let maxs = mins + Vector::repeat(CELL_WIDTH);
    AABB::new(mins, maxs)
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
struct Endpoint {
    value: f32,
    packed_flag_proxy: u32,
}

const START_FLAG_MASK: u32 = 0b1 << 31;
const PROXY_MASK: u32 = u32::MAX ^ START_FLAG_MASK;
const START_SENTINEL_TAG: u32 = u32::MAX;
const END_SENTINEL_TAG: u32 = u32::MAX ^ START_FLAG_MASK;

impl Endpoint {
    pub fn start_endpoint(value: f32, proxy: u32) -> Self {
        Self {
            value,
            packed_flag_proxy: proxy | START_FLAG_MASK,
        }
    }

    pub fn end_endpoint(value: f32, proxy: u32) -> Self {
        Self {
            value,
            packed_flag_proxy: proxy & PROXY_MASK,
        }
    }

    pub fn start_sentinel() -> Self {
        Self {
            value: -SENTINEL_VALUE,
            packed_flag_proxy: START_SENTINEL_TAG,
        }
    }

    pub fn end_sentinel() -> Self {
        Self {
            value: SENTINEL_VALUE,
            packed_flag_proxy: END_SENTINEL_TAG,
        }
    }

    pub fn is_sentinel(self) -> bool {
        self.packed_flag_proxy & PROXY_MASK == PROXY_MASK
    }

    pub fn proxy(self) -> u32 {
        self.packed_flag_proxy & PROXY_MASK
    }

    pub fn is_start(self) -> bool {
        (self.packed_flag_proxy & START_FLAG_MASK) != 0
    }

    pub fn is_end(self) -> bool {
        (self.packed_flag_proxy & START_FLAG_MASK) == 0
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
struct SAPAxis {
    min_bound: f32,
    max_bound: f32,
    endpoints: Vec<Endpoint>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    new_endpoints: Vec<(Endpoint, usize)>, // Workspace
}

impl SAPAxis {
    fn new(min_bound: f32, max_bound: f32) -> Self {
        assert!(min_bound <= max_bound);

        Self {
            min_bound,
            max_bound,
            endpoints: vec![Endpoint::start_sentinel(), Endpoint::end_sentinel()],
            new_endpoints: Vec::new(),
        }
    }

    fn batch_insert(
        &mut self,
        dim: usize,
        new_proxies: &[usize],
        proxies: &Proxies,
        reporting: Option<&mut HashMap<(u32, u32), bool>>,
    ) {
        if new_proxies.is_empty() {
            return;
        }

        self.new_endpoints.clear();

        for proxy_id in new_proxies {
            let proxy = &proxies[*proxy_id];
            assert!(proxy.aabb.mins[dim] <= self.max_bound);
            assert!(proxy.aabb.maxs[dim] >= self.min_bound);
            let start_endpoint = Endpoint::start_endpoint(proxy.aabb.mins[dim], *proxy_id as u32);
            let end_endpoint = Endpoint::end_endpoint(proxy.aabb.maxs[dim], *proxy_id as u32);

            self.new_endpoints.push((start_endpoint, 0));
            self.new_endpoints.push((end_endpoint, 0));
        }

        self.new_endpoints
            .sort_by(|a, b| a.0.value.partial_cmp(&b.0.value).unwrap_or(Ordering::Equal));

        let mut curr_existing_index = self.endpoints.len() - NUM_SENTINELS - 1;
        let new_num_endpoints = self.endpoints.len() + self.new_endpoints.len();
        self.endpoints
            .resize(new_num_endpoints, Endpoint::end_sentinel());
        let mut curr_shift_index = new_num_endpoints - NUM_SENTINELS - 1;

        // Sort the endpoints.
        // TODO: specialize for the case where this is the
        // first time we insert endpoints to this axis?
        for new_endpoint in self.new_endpoints.iter_mut().rev() {
            loop {
                let existing_endpoint = self.endpoints[curr_existing_index];
                if existing_endpoint.value <= new_endpoint.0.value {
                    break;
                }

                self.endpoints[curr_shift_index] = existing_endpoint;

                curr_shift_index -= 1;
                curr_existing_index -= 1;
            }

            self.endpoints[curr_shift_index] = new_endpoint.0;
            new_endpoint.1 = curr_shift_index;
            curr_shift_index -= 1;
        }

        // Report pairs using a single mbp pass on each new endpoint.
        let endpoints_wo_last_sentinel = &self.endpoints[..self.endpoints.len() - 1];
        if let Some(reporting) = reporting {
            for (endpoint, endpoint_id) in self.new_endpoints.drain(..).filter(|e| e.0.is_start()) {
                let proxy1 = &proxies[endpoint.proxy() as usize];
                let min = endpoint.value;
                let max = proxy1.aabb.maxs[dim];

                for endpoint2 in &endpoints_wo_last_sentinel[endpoint_id + 1..] {
                    if endpoint2.proxy() == endpoint.proxy() {
                        continue;
                    }

                    let proxy2 = &proxies[endpoint2.proxy() as usize];

                    // NOTE: some pairs with equal aabb.mins[dim] may end up being reported twice.
                    if (endpoint2.is_start() && endpoint2.value < max)
                        || (endpoint2.is_end() && proxy2.aabb.mins[dim] <= min)
                    {
                        // Report pair.
                        if proxy1.aabb.intersects(&proxy2.aabb) {
                            // Report pair.
                            let pair = sort2(endpoint.proxy(), endpoint2.proxy());
                            reporting.insert(pair, true);
                        }
                    }
                }
            }
        }
    }

    fn delete_out_of_bounds_proxies(&self, existing_proxies: &mut BitVec) -> usize {
        let mut deleted = 0;
        for endpoint in &self.endpoints {
            if endpoint.value < self.min_bound {
                let proxy_idx = endpoint.proxy() as usize;
                if endpoint.is_end() && existing_proxies[proxy_idx] {
                    existing_proxies.set(proxy_idx, false);
                    deleted += 1;
                }
            } else {
                break;
            }
        }

        for endpoint in self.endpoints.iter().rev() {
            if endpoint.value > self.max_bound {
                let proxy_idx = endpoint.proxy() as usize;
                if endpoint.is_start() && existing_proxies[proxy_idx] {
                    existing_proxies.set(endpoint.proxy() as usize, false);
                    deleted += 1;
                }
            } else {
                break;
            }
        }

        deleted
    }

    fn delete_out_of_bounds_endpoints(&mut self, existing_proxies: &BitVec) {
        self.endpoints
            .retain(|endpt| endpt.is_sentinel() || existing_proxies[endpt.proxy() as usize])
    }

    fn update_endpoints(
        &mut self,
        dim: usize,
        proxies: &Proxies,
        reporting: &mut HashMap<(u32, u32), bool>,
    ) {
        let last_endpoint = self.endpoints.len() - NUM_SENTINELS;
        for i in NUM_SENTINELS..last_endpoint {
            let mut endpoint_i = self.endpoints[i];
            let aabb_i = proxies[endpoint_i.proxy() as usize].aabb;

            if endpoint_i.is_start() {
                endpoint_i.value = aabb_i.mins[dim];
            } else {
                endpoint_i.value = aabb_i.maxs[dim];
            }

            let mut j = i;

            if endpoint_i.is_start() {
                while endpoint_i.value < self.endpoints[j - 1].value {
                    let endpoint_j = self.endpoints[j - 1];
                    self.endpoints[j] = endpoint_j;

                    if endpoint_j.is_end() {
                        // Report start collision.
                        if aabb_i.intersects(&proxies[endpoint_j.proxy() as usize].aabb) {
                            let pair = sort2(endpoint_i.proxy(), endpoint_j.proxy());
                            reporting.insert(pair, true);
                        }
                    }

                    j -= 1;
                }
            } else {
                while endpoint_i.value < self.endpoints[j - 1].value {
                    let endpoint_j = self.endpoints[j - 1];
                    self.endpoints[j] = endpoint_j;

                    if endpoint_j.is_start() {
                        // Report end collision.
                        if !aabb_i.intersects(&proxies[endpoint_j.proxy() as usize].aabb) {
                            let pair = sort2(endpoint_i.proxy(), endpoint_j.proxy());
                            reporting.insert(pair, false);
                        }
                    }

                    j -= 1;
                }
            }

            self.endpoints[j] = endpoint_i;
        }

        // println!(
        //     "Num start swaps: {}, end swaps: {}, dim: {}",
        //     num_start_swaps, num_end_swaps, dim
        // );
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
struct SAPRegion {
    axes: [SAPAxis; DIM],
    existing_proxies: BitVec,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    to_insert: Vec<usize>, // Workspace
    update_count: usize,
    proxy_count: usize,
}

impl SAPRegion {
    pub fn new(bounds: AABB<f32>) -> Self {
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

    pub fn recycle(bounds: AABB<f32>, mut old: Self) -> Self {
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
        for axis in old.axes.iter() {
            assert!(axis.endpoints.len() == 2); // Account for sentinels
        }

        old
    }

    pub fn recycle_or_new(bounds: AABB<f32>, pool: &mut Vec<Self>) -> Self {
        if let Some(old) = pool.pop() {
            Self::recycle(bounds, old)
        } else {
            Self::new(bounds)
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

    pub fn update(&mut self, proxies: &Proxies, reporting: &mut HashMap<(u32, u32), bool>) {
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

/// A broad-phase based on multiple Sweep-and-Prune instances running of disjoint region of the 3D world.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct BroadPhase {
    proxies: Proxies,
    regions: HashMap<Point<i32>, SAPRegion>,
    removed_colliders: Option<Subscription<RemovedCollider>>,
    deleted_any: bool,
    region_pool: Vec<SAPRegion>,
    points_to_remove: Vec<Point<i32>>, // Workspace
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
            serialize_with = "crate::data::hashmap::serialize_hashmap_capacity",
            deserialize_with = "crate::data::hashmap::deserialize_hashmap_capacity"
        )
    )]
    reporting: HashMap<(u32, u32), bool>, // Workspace
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct BroadPhaseProxy {
    handle: ColliderHandle,
    aabb: AABB<f32>,
    next_free: u32,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
struct Proxies {
    elements: Vec<BroadPhaseProxy>,
    first_free: u32,
}

impl Proxies {
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
            first_free: NEXT_FREE_SENTINEL,
        }
    }

    pub fn insert(&mut self, proxy: BroadPhaseProxy) -> usize {
        if self.first_free != NEXT_FREE_SENTINEL {
            let proxy_id = self.first_free as usize;
            self.first_free = self.elements[proxy_id].next_free;
            self.elements[proxy_id] = proxy;
            proxy_id
        } else {
            self.elements.push(proxy);
            self.elements.len() - 1
        }
    }

    pub fn remove(&mut self, proxy_id: usize) {
        self.elements[proxy_id].next_free = self.first_free;
        self.first_free = proxy_id as u32;
    }

    // // FIXME: take holes into account?
    // pub fn get(&self, i: usize) -> Option<&BroadPhaseProxy> {
    //     self.elements.get(i)
    // }

    // FIXME: take holes into account?
    pub fn get_mut(&mut self, i: usize) -> Option<&mut BroadPhaseProxy> {
        self.elements.get_mut(i)
    }
}

impl Index<usize> for Proxies {
    type Output = BroadPhaseProxy;
    fn index(&self, i: usize) -> &BroadPhaseProxy {
        self.elements.index(i)
    }
}

impl IndexMut<usize> for Proxies {
    fn index_mut(&mut self, i: usize) -> &mut BroadPhaseProxy {
        self.elements.index_mut(i)
    }
}

impl BroadPhase {
    /// Create a new empty broad-phase.
    pub fn new() -> Self {
        BroadPhase {
            removed_colliders: None,
            proxies: Proxies::new(),
            regions: HashMap::default(),
            region_pool: Vec::new(),
            points_to_remove: Vec::new(),
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

        let mut cursor = self.removed_colliders.take().unwrap();
        for collider in colliders.removed_colliders.read(&cursor) {
            self.remove_collider(collider.proxy_index);
        }

        colliders.removed_colliders.ack(&mut cursor);
        self.removed_colliders = Some(cursor);
    }

    fn remove_collider<'a>(&mut self, proxy_index: usize) {
        if proxy_index == crate::INVALID_USIZE {
            // This collider has not been added to the broad-phase yet.
            return;
        }

        let proxy = &mut self.proxies[proxy_index];

        // Push the proxy to infinity, but not beyond the sentinels.
        proxy.aabb.mins.coords.fill(SENTINEL_VALUE / 2.0);
        proxy.aabb.maxs.coords.fill(SENTINEL_VALUE / 2.0);
        // Discretize the AABB to find the regions that need to be invalidated.
        let start = point_key(proxy.aabb.mins);
        let end = point_key(proxy.aabb.maxs);

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

        self.proxies.remove(proxy_index);
    }

    pub(crate) fn update_aabbs(
        &mut self,
        prediction_distance: f32,
        bodies: &RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        // First, if we have any pending removals we have
        // to deal with them now because otherwise we will
        // end up with an ABA problems when reusing proxy
        // ids.
        self.complete_removals();

        for body_handle in bodies
            .active_dynamic_set
            .iter()
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
                let start = point_key(aabb.mins);
                let end = point_key(aabb.maxs);

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
                            let region_bounds = region_aabb(region_key);
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
                self.points_to_remove.push(*point);
            }
        }

        // Remove all the empty regions and store them in the region pool
        let regions = &mut self.regions;
        self.region_pool.extend(
            self.points_to_remove
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
    use crate::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};

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
