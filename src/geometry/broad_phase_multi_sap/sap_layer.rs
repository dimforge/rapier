use super::{BroadPhaseProxies, SAPRegion};
use crate::geometry::broad_phase_multi_sap::BroadPhaseProxy;
use crate::geometry::{Collider, AABB};
use crate::math::{Point, Real};
use parry::utils::hashmap::{Entry, HashMap};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub(crate) struct SAPLayer {
    depth: i8,
    region_width: Real,
    regions: HashMap<Point<i32>, SAPRegion>,
    deleted_any: bool,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    regions_to_remove: Vec<Point<i32>>, // Workspace
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    created_regions: Vec<Point<i32>>,
}

impl SAPLayer {
    pub fn new(depth: i8) -> Self {
        Self {
            depth,
            region_width: super::CELL_WIDTH, // FIXME
            regions: HashMap::default(),
            deleted_any: false,
            regions_to_remove: vec![],
            created_regions: vec![],
        }
    }

    pub fn insert_subregion(&mut self, sub_key: &Point<i32>) {}

    pub fn preupdate_collider(
        &mut self,
        collider: &Collider,
        aabb: &AABB,
        pool: &mut Vec<SAPRegion>,
    ) {
        let proxy_id = collider.proxy_index;
        let start = super::point_key(aabb.mins);
        let end = super::point_key(aabb.maxs);

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
                    let region_bounds = super::region_aabb(region_key);

                    let region = match self.regions.entry(region_key) {
                        Entry::Occupied(occupied) => occupied.into_mut(),
                        Entry::Vacant(vacant) => {
                            self.created_regions.push(region_key);
                            vacant.insert(SAPRegion::recycle_or_new(region_bounds, pool))
                        }
                    };
                    let _ = region.preupdate_proxy(proxy_id);
                }
            }
        }
    }

    pub fn remove_collider(&mut self, proxy: &BroadPhaseProxy, proxy_index: usize) {
        // Discretize the AABB to find the regions that need to be invalidated.
        let start = super::point_key(proxy.aabb.mins);
        let end = super::point_key(proxy.aabb.maxs);

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
                    if let Some(region) = self.regions.get_mut(&key) {
                        region.predelete_proxy(proxy_index);
                        self.deleted_any = true;
                    }
                }
            }
        }
    }

    pub fn update_regions(
        &mut self,
        proxies: &BroadPhaseProxies,
        reporting: &mut HashMap<(u32, u32), bool>,
        pool: &mut Vec<SAPRegion>,
    ) {
        for (point, region) in &mut self.regions {
            region.update(proxies, reporting);
            if region.proxy_count == 0 {
                self.regions_to_remove.push(*point);
            }
        }

        // Remove all the empty regions and store them in the region pool
        let regions = &mut self.regions;
        pool.extend(
            self.regions_to_remove
                .drain(..)
                .map(|p| regions.remove(&p).unwrap()),
        );
    }

    pub fn complete_removals(
        &mut self,
        proxies: &BroadPhaseProxies,
        reporting: &mut HashMap<(u32, u32), bool>,
        pool: &mut Vec<SAPRegion>,
    ) {
        if self.deleted_any {
            self.update_regions(proxies, reporting, pool);
            self.deleted_any = false;
        }
    }
}
