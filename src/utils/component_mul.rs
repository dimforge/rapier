//! ComponentMul trait for element-wise vector operations.

use crate::math::Vector;
#[cfg(not(target_arch = "spirv"))]
use crate::utils::SimdRealCopy;
#[cfg(not(target_arch = "spirv"))]
use na::{Vector2, Vector3};

/// Extension trait for element-wise vector operations
pub trait ComponentMul: Sized {
    /// Element-wise multiplication
    fn component_mul(&self, other: &Self) -> Self;
}

impl ComponentMul for Vector {
    #[inline]
    fn component_mul(&self, other: &Self) -> Self {
        *self * *other
    }
}

// Nalgebra vector implementations for SIMD support
#[cfg(not(target_arch = "spirv"))]
impl<N: SimdRealCopy> ComponentMul for Vector2<N> {
    #[inline]
    fn component_mul(&self, other: &Self) -> Self {
        nalgebra::Vector2::component_mul(self, other)
    }
}

#[cfg(not(target_arch = "spirv"))]
impl<N: SimdRealCopy> ComponentMul for Vector3<N> {
    #[inline]
    fn component_mul(&self, other: &Self) -> Self {
        nalgebra::Vector3::component_mul(self, other)
    }
}
