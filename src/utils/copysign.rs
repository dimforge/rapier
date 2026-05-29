//! SimdSign trait for copying signs between values.

use crate::math::Real;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SimdReal;
#[cfg(not(target_arch = "spirv"))]
use na::{Scalar, Vector2, Vector3};
#[cfg(feature = "simd-is-enabled")]
use simba::simd::SimdRealField;

/// Trait to copy the sign of each component of one scalar/vector/matrix to another.
pub trait CopySign<Rhs>: Sized {
    // See SIMD implementations of copy_sign there: https://stackoverflow.com/a/57872652
    /// Copy the sign of each component of `self` to the corresponding component of `to`.
    fn copy_sign_to(self, to: Rhs) -> Rhs;
}

impl CopySign<Real> for Real {
    fn copy_sign_to(self, to: Self) -> Self {
        const MINUS_ZERO: Real = -0.0;
        let signbit = MINUS_ZERO.to_bits();
        Real::from_bits((signbit & self.to_bits()) | ((!signbit) & to.to_bits()))
    }
}

#[cfg(not(target_arch = "spirv"))]
impl<N: Scalar + Copy + CopySign<N>> CopySign<Vector2<N>> for N {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.copy_sign_to(to.x), self.copy_sign_to(to.y))
    }
}

#[cfg(not(target_arch = "spirv"))]
impl<N: Scalar + Copy + CopySign<N>> CopySign<Vector3<N>> for N {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.copy_sign_to(to.x),
            self.copy_sign_to(to.y),
            self.copy_sign_to(to.z),
        )
    }
}

#[cfg(not(target_arch = "spirv"))]
impl<N: Scalar + Copy + CopySign<N>> CopySign<Vector2<N>> for Vector2<N> {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.x.copy_sign_to(to.x), self.y.copy_sign_to(to.y))
    }
}

#[cfg(not(target_arch = "spirv"))]
impl<N: Scalar + Copy + CopySign<N>> CopySign<Vector3<N>> for Vector3<N> {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.x.copy_sign_to(to.x),
            self.y.copy_sign_to(to.y),
            self.z.copy_sign_to(to.z),
        )
    }
}

#[cfg(feature = "simd-is-enabled")]
impl CopySign<SimdReal> for SimdReal {
    fn copy_sign_to(self, to: SimdReal) -> SimdReal {
        to.simd_copysign(self)
    }
}
