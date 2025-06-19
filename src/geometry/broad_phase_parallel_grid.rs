use crate::dynamics::RigidBodySet;
use crate::geometry::{
    Aabb, BroadPhase, BroadPhasePairEvent, ColliderHandle, ColliderPair, ColliderSet,
};
use parry::bounding_volume::BoundingVolume;
use parry::math::Real;
use rustc_hash::FxHasher;

use rayon::prelude::*;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
struct GridEntry {
    key: u64,
    proxy_id: usize,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
struct ColliderProxy {
    aabb: Aabb,
    handle: ColliderHandle,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhaseParallelGrid {
    entries: Vec<GridEntry>,
    proxies: Vec<ColliderProxy>,
    cell_width: Real,
}

impl Default for BroadPhaseParallelGrid {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl BroadPhaseParallelGrid {
    pub fn new(cell_width: Real) -> Self {
        Self {
            entries: vec![],
            proxies: vec![],
            cell_width,
        }
    }
}

impl BroadPhase for BroadPhaseParallelGrid {
    fn update(
        &mut self,
        dt: Real,
        prediction_distance: Real,
        colliders: &mut ColliderSet,
        bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        if modified_colliders.is_empty() {
            return;
        }

        // 1. Rasterize the aabbs on the grid.
        let t0 = std::time::Instant::now();
        self.entries.clear();
        self.proxies.clear();

        for (handle, co) in colliders.iter() {
            let aabb = co.compute_aabb();
            let proxy_id = self.proxies.len();
            self.proxies.push(ColliderProxy { aabb, handle });

            let imins = aabb
                .mins
                .coords
                .map(|x| (x / self.cell_width).floor() as i32);
            let imaxs = aabb
                .maxs
                .coords
                .map(|x| (x / self.cell_width).ceil() as i32);

            for ix in imins.x..imaxs.x {
                for iy in imins.y..imaxs.y {
                    for iz in imins.z..imaxs.z {
                        use std::hash::{Hash, Hasher};
                        let mut hasher = FxHasher::default();
                        hasher.write_u32(ix as u32);
                        hasher.write_u32(iy as u32);
                        hasher.write_u32(iz as u32);
                        let key = hasher.finish();
                        self.entries.push(GridEntry { key, proxy_id });
                    }
                }
            }
        }
        println!("A: {}", t0.elapsed().as_secs_f32());

        // 2. Sort.
        let t0 = std::time::Instant::now();
        self.entries.sort_unstable_by_key(|e| e.key);
        println!("B: {}", t0.elapsed().as_secs_f32());

        // 3. Group by key.
        // 4. Emit pairs.
        let t0 = std::time::Instant::now();
        let mut i = 0;
        while i < self.entries.len() {
            let curr_key = self.entries[i].key;

            while i < self.entries.len() && self.entries[i].key == curr_key {
                let proxy1 = &self.proxies[self.entries[i].proxy_id];

                for k in i + 1..self.entries.len() {
                    if self.entries[k].key != curr_key {
                        break;
                    }

                    let proxy2 = &self.proxies[self.entries[k].proxy_id];

                    if proxy1.aabb.intersects(&proxy2.aabb) {
                        events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                            proxy1.handle,
                            proxy2.handle,
                        )));
                    }
                }

                i += 1;
            }
        }
        println!("C: {}", t0.elapsed().as_secs_f32());
        println!(">>>>>> Num events: {}", events.iter().len());
    }
}
