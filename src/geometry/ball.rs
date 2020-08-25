#[cfg(feature = "simd-is-enabled")]
use crate::math::{Point, SimdFloat};

#[cfg(feature = "simd-is-enabled")]
#[derive(Copy, Clone, Debug)]
pub(crate) struct WBall {
    pub center: Point<SimdFloat>,
    pub radius: SimdFloat,
}

#[cfg(feature = "simd-is-enabled")]
impl WBall {
    pub fn new(center: Point<SimdFloat>, radius: SimdFloat) -> Self {
        WBall { center, radius }
    }
}
