use crate::math::{Point, Real, Vector};
use parry::bounding_volume::AABB;

pub(crate) const NUM_SENTINELS: usize = 1;
pub(crate) const NEXT_FREE_SENTINEL: u32 = u32::MAX;
pub(crate) const SENTINEL_VALUE: Real = Real::MAX;
pub(crate) const REGION_WIDTH_BASE: Real = 20.0;
pub(crate) const REGION_WIDTH_POWER_BASIS: Real = 8.0;

pub(crate) fn sort2(a: u32, b: u32) -> (u32, u32) {
    assert_ne!(a, b);

    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

pub(crate) fn point_key(point: Point<Real>, region_width: Real) -> Point<i32> {
    (point / region_width)
        .coords
        .map(|e| e.floor() as i32)
        .into()
}

pub(crate) fn region_aabb(index: Point<i32>, region_width: Real) -> AABB {
    let mins = index.coords.map(|i| i as Real * region_width).into();
    let maxs = mins + Vector::repeat(region_width);
    AABB::new(mins, maxs)
}

pub(crate) fn region_width(depth: i8) -> Real {
    REGION_WIDTH_BASE * REGION_WIDTH_POWER_BASIS.powi(depth as i32)
}

pub(crate) fn layer_containing_aabb(aabb: &AABB) -> i8 {
    let width = 2.0 * aabb.half_extents().norm();
    // TODO: handle overflows in the f32 -> i8 conversion.
    (width / REGION_WIDTH_BASE).log(REGION_WIDTH_POWER_BASIS) as i8
}
