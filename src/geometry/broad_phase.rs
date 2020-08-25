use crate::geometry::ColliderHandle;
use ncollide::bounding_volume::AABB;
#[cfg(feature = "simd-is-enabled")]
use {
    crate::geometry::WAABB,
    crate::math::{Point, SIMD_WIDTH},
    crate::utils::WVec,
    simba::simd::SimdBool as _,
};

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

pub struct WAABBHierarchyIntersections {
    curr_level_interferences: Vec<usize>,
    next_level_interferences: Vec<usize>,
}

impl WAABBHierarchyIntersections {
    pub fn new() -> Self {
        Self {
            curr_level_interferences: Vec::new(),
            next_level_interferences: Vec::new(),
        }
    }

    pub fn computed_interferences(&self) -> &[usize] {
        &self.curr_level_interferences[..]
    }

    pub(crate) fn computed_interferences_mut(&mut self) -> &mut Vec<usize> {
        &mut self.curr_level_interferences
    }
}

#[cfg(feature = "simd-is-enabled")]
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct WAABBHierarchy {
    levels: Vec<Vec<WAABB>>,
}

#[cfg(feature = "simd-is-enabled")]
impl WAABBHierarchy {
    pub fn new(aabbs: &[AABB<f32>]) -> Self {
        let mut waabbs: Vec<_> = aabbs
            .chunks_exact(SIMD_WIDTH)
            .map(|aabbs| WAABB::from(array![|ii| aabbs[ii]; SIMD_WIDTH]))
            .collect();

        if aabbs.len() % SIMD_WIDTH != 0 {
            let first_i = (aabbs.len() / SIMD_WIDTH) * SIMD_WIDTH;
            let last_i = aabbs.len() - 1;
            let last_waabb =
                WAABB::from(array![|ii| aabbs[(first_i + ii).min(last_i)]; SIMD_WIDTH]);
            waabbs.push(last_waabb);
        }

        let mut levels = vec![waabbs];

        loop {
            let last_level = levels.last().unwrap();
            let mut next_level = Vec::new();

            for chunk in last_level.chunks_exact(SIMD_WIDTH) {
                let mins = Point::from(array![|ii| chunk[ii].mins.horizontal_inf(); SIMD_WIDTH]);
                let maxs = Point::from(array![|ii| chunk[ii].maxs.horizontal_sup(); SIMD_WIDTH]);
                next_level.push(WAABB::new(mins, maxs));
            }

            // Deal with the last non-exact chunk.
            if last_level.len() % SIMD_WIDTH != 0 {
                let first_id = (last_level.len() / SIMD_WIDTH) * SIMD_WIDTH;
                let last_id = last_level.len() - 1;
                let mins = array![|ii| last_level[(first_id + ii).min(last_id)]
                    .mins
                    .horizontal_inf(); SIMD_WIDTH];
                let maxs = array![|ii| last_level[(first_id + ii).min(last_id)]
                    .maxs
                    .horizontal_sup(); SIMD_WIDTH];

                let mins = Point::from(mins);
                let maxs = Point::from(maxs);
                next_level.push(WAABB::new(mins, maxs));
            }

            if next_level.len() == 1 {
                levels.push(next_level);
                break;
            }

            levels.push(next_level);
        }

        Self { levels }
    }

    pub fn compute_interferences_with(
        &self,
        aabb: AABB<f32>,
        workspace: &mut WAABBHierarchyIntersections,
    ) {
        let waabb1 = WAABB::splat(aabb);
        workspace.next_level_interferences.clear();
        workspace.curr_level_interferences.clear();
        workspace.curr_level_interferences.push(0);

        for level in self.levels.iter().rev() {
            for i in &workspace.curr_level_interferences {
                // This `if let` handle the case when `*i` is out of bounds because
                // the initial number of aabbs was not a power of SIMD_WIDTH.
                if let Some(waabb2) = level.get(*i) {
                    // NOTE: using `intersect.bitmask()` and performing bit comparisons
                    // is much more efficient than testing if each intersect.extract(i) is true.
                    let intersect = waabb1.intersects_lanewise(waabb2);
                    let bitmask = intersect.bitmask();

                    for j in 0..SIMD_WIDTH {
                        if (bitmask & (1 << j)) != 0 {
                            workspace.next_level_interferences.push(i * SIMD_WIDTH + j)
                        }
                    }
                }
            }

            std::mem::swap(
                &mut workspace.curr_level_interferences,
                &mut workspace.next_level_interferences,
            );
            workspace.next_level_interferences.clear();
        }
    }
}

#[cfg(not(feature = "simd-is-enabled"))]
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct WAABBHierarchy {
    levels: Vec<Vec<AABB<f32>>>,
}

#[cfg(not(feature = "simd-is-enabled"))]
impl WAABBHierarchy {
    const GROUP_SIZE: usize = 4;

    pub fn new(aabbs: &[AABB<f32>]) -> Self {
        use ncollide::bounding_volume::BoundingVolume;

        let mut levels = vec![aabbs.to_vec()];

        loop {
            let last_level = levels.last().unwrap();
            let mut next_level = Vec::new();

            for chunk in last_level.chunks(Self::GROUP_SIZE) {
                let mut merged = chunk[0];
                for aabb in &chunk[1..] {
                    merged.merge(aabb)
                }

                next_level.push(merged);
            }

            if next_level.len() == 1 {
                levels.push(next_level);
                break;
            }

            levels.push(next_level);
        }

        Self { levels }
    }

    pub fn compute_interferences_with(
        &self,
        aabb1: AABB<f32>,
        workspace: &mut WAABBHierarchyIntersections,
    ) {
        use ncollide::bounding_volume::BoundingVolume;

        workspace.next_level_interferences.clear();
        workspace.curr_level_interferences.clear();
        workspace.curr_level_interferences.push(0);

        for level in self.levels[1..].iter().rev() {
            for i in &workspace.curr_level_interferences {
                for j in 0..Self::GROUP_SIZE {
                    if let Some(aabb2) = level.get(*i + j) {
                        if aabb1.intersects(aabb2) {
                            workspace
                                .next_level_interferences
                                .push((i + j) * Self::GROUP_SIZE)
                        }
                    }
                }
            }

            std::mem::swap(
                &mut workspace.curr_level_interferences,
                &mut workspace.next_level_interferences,
            );
            workspace.next_level_interferences.clear();
        }

        // Last level.
        for i in &workspace.curr_level_interferences {
            for j in 0..Self::GROUP_SIZE {
                if let Some(aabb2) = self.levels[0].get(*i + j) {
                    if aabb1.intersects(aabb2) {
                        workspace.next_level_interferences.push(i + j)
                    }
                }
            }
        }

        std::mem::swap(
            &mut workspace.curr_level_interferences,
            &mut workspace.next_level_interferences,
        );
        workspace.next_level_interferences.clear();
    }
}
