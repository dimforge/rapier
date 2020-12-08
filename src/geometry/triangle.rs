#[cfg(feature = "dim2")]
use crate::{buckler::shape::CuboidFeatureFace, geometry::Triangle, math::Vector};

#[cfg(feature = "dim2")]
pub fn support_face(_triangle: &Triangle, _local_dir: Vector<f32>) -> CuboidFeatureFace {
    unimplemented!()
}
