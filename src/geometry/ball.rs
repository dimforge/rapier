#[cfg(feature = "simd-is-enabled")]
use crate::math::{Point, SimdReal};

#[cfg(feature = "simd-is-enabled")]
#[derive(Copy, Clone, Debug)]
pub(crate) struct WBall {
    pub center: Point<SimdReal>,
    pub radius: SimdReal,
}

#[cfg(feature = "simd-is-enabled")]
impl WBall {
    pub fn new(center: Point<SimdReal>, radius: SimdReal) -> Self {
        WBall { center, radius }
    }
}
