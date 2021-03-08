use crate::math::{Point, Real, Vector};
use parry::bounding_volume::AABB;

pub(crate) const NUM_SENTINELS: usize = 1;
pub(crate) const NEXT_FREE_SENTINEL: u32 = u32::MAX;
pub(crate) const SENTINEL_VALUE: Real = Real::MAX;
pub(crate) const CELL_WIDTH: Real = 20.0;

pub(crate) fn sort2(a: u32, b: u32) -> (u32, u32) {
    assert_ne!(a, b);

    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

pub(crate) fn point_key(point: Point<Real>) -> Point<i32> {
    (point / CELL_WIDTH).coords.map(|e| e.floor() as i32).into()
}

pub(crate) fn region_aabb(index: Point<i32>) -> AABB {
    let mins = index.coords.map(|i| i as Real * CELL_WIDTH).into();
    let maxs = mins + Vector::repeat(CELL_WIDTH);
    AABB::new(mins, maxs)
}
