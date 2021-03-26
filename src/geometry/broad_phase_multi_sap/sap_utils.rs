use crate::math::{Point, Real, Vector};
use parry::bounding_volume::AABB;

pub(crate) const NUM_SENTINELS: usize = 1;
pub(crate) const NEXT_FREE_SENTINEL: u32 = u32::MAX;
pub(crate) const SENTINEL_VALUE: Real = Real::MAX;
pub(crate) const DELETED_AABB_VALUE: Real = SENTINEL_VALUE / 2.0;
pub(crate) const MAX_AABB_EXTENT: Real = SENTINEL_VALUE / 4.0;
pub(crate) const REGION_WIDTH_BASE: Real = 1.0;
pub(crate) const REGION_WIDTH_POWER_BASIS: Real = 5.0;

pub(crate) fn sort2(a: u32, b: u32) -> (u32, u32) {
    assert_ne!(a, b);

    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

pub(crate) fn clamp_point(point: Point<Real>) -> Point<Real> {
    point.map(|e| na::clamp(e, -MAX_AABB_EXTENT, MAX_AABB_EXTENT))
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
    (REGION_WIDTH_BASE * REGION_WIDTH_POWER_BASIS.powi(depth as i32)).min(MAX_AABB_EXTENT)
}

/// Computes the depth of the layer the given AABB should be part of.
///
/// The idea here is that an AABB should be part of a layer which has
/// regions large enough so that one AABB doesn't crosses too many
/// regions. But the regions must also not be too large, otherwise
/// we are loosing the benefits of Multi-SAP.
///
/// If the code bellow, we select a layer such that each region can
/// contain at least a chain of 10 contiguous objects with that AABB.
pub(crate) fn layer_containing_aabb(aabb: &AABB) -> i8 {
    // Max number of elements of this size we would like one region to be able to contain.
    const NUM_ELEMENTS_PER_DIMENSION: Real = 10.0;

    let width = 2.0 * aabb.half_extents().norm() * NUM_ELEMENTS_PER_DIMENSION;
    (width / REGION_WIDTH_BASE)
        .log(REGION_WIDTH_POWER_BASIS)
        .round()
        .max(i8::MIN as Real)
        .min(i8::MAX as Real) as i8
}
