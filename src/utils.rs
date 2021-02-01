//! Miscellaneous utilities.

use na::{Matrix3, Point2, Point3, Scalar, SimdRealField, Vector2, Vector3};
use num::Zero;
use simba::simd::SimdValue;

use parry::utils::SdpMatrix3;
use {
    crate::math::{Real, SimdReal},
    na::SimdPartialOrd,
    num::One,
};

pub(crate) fn inv(val: Real) -> Real {
    if val == 0.0 {
        0.0
    } else {
        1.0 / val
    }
}

pub(crate) fn simd_inv(val: SimdReal) -> SimdReal {
    let zero = SimdReal::splat(0.0);
    let is_zero = val.simd_eq(zero);
    zero.select(is_zero, SimdReal::splat(1.0) / val)
}

/// Trait to copy the sign of each component of one scalar/vector/matrix to another.
pub trait WSign<Rhs>: Sized {
    // See SIMD implementations of copy_sign there: https://stackoverflow.com/a/57872652
    /// Copy the sign of each component of `self` to the corresponding component of `to`.
    fn copy_sign_to(self, to: Rhs) -> Rhs;
}

impl WSign<Real> for Real {
    fn copy_sign_to(self, to: Self) -> Self {
        const MINUS_ZERO: Real = -0.0;
        let signbit = MINUS_ZERO.to_bits();
        Real::from_bits((signbit & self.to_bits()) | ((!signbit) & to.to_bits()))
    }
}

impl<N: Scalar + Copy + WSign<N>> WSign<Vector2<N>> for N {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.copy_sign_to(to.x), self.copy_sign_to(to.y))
    }
}

impl<N: Scalar + Copy + WSign<N>> WSign<Vector3<N>> for N {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.copy_sign_to(to.x),
            self.copy_sign_to(to.y),
            self.copy_sign_to(to.z),
        )
    }
}

impl<N: Scalar + Copy + WSign<N>> WSign<Vector2<N>> for Vector2<N> {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.x.copy_sign_to(to.x), self.y.copy_sign_to(to.y))
    }
}

impl<N: Scalar + Copy + WSign<N>> WSign<Vector3<N>> for Vector3<N> {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.x.copy_sign_to(to.x),
            self.y.copy_sign_to(to.y),
            self.z.copy_sign_to(to.z),
        )
    }
}

impl WSign<SimdReal> for SimdReal {
    fn copy_sign_to(self, to: SimdReal) -> SimdReal {
        to.simd_copysign(self)
    }
}

pub(crate) trait WComponent: Sized {
    type Element;

    fn min_component(self) -> Self::Element;
    fn max_component(self) -> Self::Element;
}

impl WComponent for Real {
    type Element = Real;

    fn min_component(self) -> Self::Element {
        self
    }
    fn max_component(self) -> Self::Element {
        self
    }
}

impl WComponent for SimdReal {
    type Element = Real;

    fn min_component(self) -> Self::Element {
        self.simd_horizontal_min()
    }
    fn max_component(self) -> Self::Element {
        self.simd_horizontal_max()
    }
}

/// Trait to compute the orthonormal basis of a vector.
pub trait WBasis: Sized {
    /// The type of the array of orthonormal vectors.
    type Basis;
    /// Computes the vectors which, when combined with `self`, form an orthonormal basis.
    fn orthonormal_basis(self) -> Self::Basis;
}

impl<N: SimdRealField> WBasis for Vector2<N> {
    type Basis = [Vector2<N>; 1];
    fn orthonormal_basis(self) -> [Vector2<N>; 1] {
        [Vector2::new(-self.y, self.x)]
    }
}

impl<N: SimdRealField + WSign<N>> WBasis for Vector3<N> {
    type Basis = [Vector3<N>; 2];
    // Robust and branchless implementation from Pixar:
    // https://graphics.pixar.com/library/OrthonormalB/paper.pdf
    fn orthonormal_basis(self) -> [Vector3<N>; 2] {
        let sign = self.z.copy_sign_to(N::one());
        let a = -N::one() / (sign + self.z);
        let b = self.x * self.y * a;

        [
            Vector3::new(
                N::one() + sign * self.x * self.x * a,
                sign * b,
                -sign * self.x,
            ),
            Vector3::new(b, sign + self.y * self.y * a, -self.y),
        ]
    }
}

pub(crate) trait WVec: Sized {
    type Element;

    fn horizontal_inf(&self) -> Self::Element;
    fn horizontal_sup(&self) -> Self::Element;
}

impl<N: Scalar + Copy + WComponent> WVec for Vector2<N>
where
    N::Element: Scalar,
{
    type Element = Vector2<N::Element>;

    fn horizontal_inf(&self) -> Self::Element {
        Vector2::new(self.x.min_component(), self.y.min_component())
    }

    fn horizontal_sup(&self) -> Self::Element {
        Vector2::new(self.x.max_component(), self.y.max_component())
    }
}

impl<N: Scalar + Copy + WComponent> WVec for Point2<N>
where
    N::Element: Scalar,
{
    type Element = Point2<N::Element>;

    fn horizontal_inf(&self) -> Self::Element {
        Point2::new(self.x.min_component(), self.y.min_component())
    }

    fn horizontal_sup(&self) -> Self::Element {
        Point2::new(self.x.max_component(), self.y.max_component())
    }
}

impl<N: Scalar + Copy + WComponent> WVec for Vector3<N>
where
    N::Element: Scalar,
{
    type Element = Vector3<N::Element>;

    fn horizontal_inf(&self) -> Self::Element {
        Vector3::new(
            self.x.min_component(),
            self.y.min_component(),
            self.z.min_component(),
        )
    }

    fn horizontal_sup(&self) -> Self::Element {
        Vector3::new(
            self.x.max_component(),
            self.y.max_component(),
            self.z.max_component(),
        )
    }
}

impl<N: Scalar + Copy + WComponent> WVec for Point3<N>
where
    N::Element: Scalar,
{
    type Element = Point3<N::Element>;

    fn horizontal_inf(&self) -> Self::Element {
        Point3::new(
            self.x.min_component(),
            self.y.min_component(),
            self.z.min_component(),
        )
    }

    fn horizontal_sup(&self) -> Self::Element {
        Point3::new(
            self.x.max_component(),
            self.y.max_component(),
            self.z.max_component(),
        )
    }
}

pub(crate) trait WCrossMatrix: Sized {
    type CrossMat;

    fn gcross_matrix(self) -> Self::CrossMat;
}

impl WCrossMatrix for Vector3<Real> {
    type CrossMat = Matrix3<Real>;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        Matrix3::new(
            0.0, -self.z, self.y,
            self.z, 0.0, -self.x,
            -self.y, self.x, 0.0,
        )
    }
}

impl WCrossMatrix for Vector2<Real> {
    type CrossMat = Vector2<Real>;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        Vector2::new(-self.y, self.x)
    }
}

pub(crate) trait WCross<Rhs>: Sized {
    type Result;
    fn gcross(&self, rhs: Rhs) -> Self::Result;
}

impl WCross<Vector3<Real>> for Vector3<Real> {
    type Result = Self;

    fn gcross(&self, rhs: Vector3<Real>) -> Self::Result {
        self.cross(&rhs)
    }
}

impl WCross<Vector2<Real>> for Vector2<Real> {
    type Result = Real;

    fn gcross(&self, rhs: Vector2<Real>) -> Self::Result {
        self.x * rhs.y - self.y * rhs.x
    }
}

impl WCross<Vector2<Real>> for Real {
    type Result = Vector2<Real>;

    fn gcross(&self, rhs: Vector2<Real>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }
}

pub(crate) trait WDot<Rhs>: Sized {
    type Result;
    fn gdot(&self, rhs: Rhs) -> Self::Result;
}

impl WDot<Vector3<Real>> for Vector3<Real> {
    type Result = Real;

    fn gdot(&self, rhs: Vector3<Real>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl WDot<Vector2<Real>> for Vector2<Real> {
    type Result = Real;

    fn gdot(&self, rhs: Vector2<Real>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl WDot<Real> for Real {
    type Result = Real;

    fn gdot(&self, rhs: Real) -> Self::Result {
        *self * rhs
    }
}

impl WCrossMatrix for Vector3<SimdReal> {
    type CrossMat = Matrix3<SimdReal>;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        Matrix3::new(
            SimdReal::zero(), -self.z, self.y,
            self.z, SimdReal::zero(), -self.x,
            -self.y, self.x, SimdReal::zero(),
        )
    }
}

impl WCrossMatrix for Vector2<SimdReal> {
    type CrossMat = Vector2<SimdReal>;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        Vector2::new(-self.y, self.x)
    }
}

impl WCross<Vector3<SimdReal>> for Vector3<SimdReal> {
    type Result = Vector3<SimdReal>;

    fn gcross(&self, rhs: Self) -> Self::Result {
        self.cross(&rhs)
    }
}

impl WCross<Vector2<SimdReal>> for SimdReal {
    type Result = Vector2<SimdReal>;

    fn gcross(&self, rhs: Vector2<SimdReal>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }
}

impl WCross<Vector2<SimdReal>> for Vector2<SimdReal> {
    type Result = SimdReal;

    fn gcross(&self, rhs: Self) -> Self::Result {
        let yx = Vector2::new(rhs.y, rhs.x);
        let prod = self.component_mul(&yx);
        prod.x - prod.y
    }
}

impl WDot<Vector3<SimdReal>> for Vector3<SimdReal> {
    type Result = SimdReal;

    fn gdot(&self, rhs: Vector3<SimdReal>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl WDot<Vector2<SimdReal>> for Vector2<SimdReal> {
    type Result = SimdReal;

    fn gdot(&self, rhs: Vector2<SimdReal>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl WDot<SimdReal> for SimdReal {
    type Result = SimdReal;

    fn gdot(&self, rhs: SimdReal) -> Self::Result {
        *self * rhs
    }
}

pub(crate) trait WAngularInertia<N> {
    type AngVector;
    type LinVector;
    type AngMatrix;
    fn inverse(&self) -> Self;
    fn transform_lin_vector(&self, pt: Self::LinVector) -> Self::LinVector;
    fn transform_vector(&self, pt: Self::AngVector) -> Self::AngVector;
    fn squared(&self) -> Self;
    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix;
    fn into_matrix(self) -> Self::AngMatrix;
}

impl WAngularInertia<Real> for Real {
    type AngVector = Real;
    type LinVector = Vector2<Real>;
    type AngMatrix = Real;

    fn inverse(&self) -> Self {
        if *self != 0.0 {
            1.0 / *self
        } else {
            0.0
        }
    }

    fn transform_lin_vector(&self, pt: Vector2<Real>) -> Vector2<Real> {
        *self * pt
    }
    fn transform_vector(&self, pt: Real) -> Real {
        *self * pt
    }

    fn squared(&self) -> Real {
        *self * *self
    }

    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix {
        mat * *self
    }

    fn into_matrix(self) -> Self::AngMatrix {
        self
    }
}

impl WAngularInertia<SimdReal> for SimdReal {
    type AngVector = SimdReal;
    type LinVector = Vector2<SimdReal>;
    type AngMatrix = SimdReal;

    fn inverse(&self) -> Self {
        let zero = <SimdReal>::zero();
        let is_zero = self.simd_eq(zero);
        (<SimdReal>::one() / *self).select(is_zero, zero)
    }

    fn transform_lin_vector(&self, pt: Vector2<SimdReal>) -> Vector2<SimdReal> {
        pt * *self
    }

    fn transform_vector(&self, pt: SimdReal) -> SimdReal {
        *self * pt
    }

    fn squared(&self) -> SimdReal {
        *self * *self
    }

    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix {
        *mat * *self
    }

    fn into_matrix(self) -> Self::AngMatrix {
        self
    }
}

impl WAngularInertia<Real> for SdpMatrix3<Real> {
    type AngVector = Vector3<Real>;
    type LinVector = Vector3<Real>;
    type AngMatrix = Matrix3<Real>;

    fn inverse(&self) -> Self {
        let minor_m12_m23 = self.m22 * self.m33 - self.m23 * self.m23;
        let minor_m11_m23 = self.m12 * self.m33 - self.m13 * self.m23;
        let minor_m11_m22 = self.m12 * self.m23 - self.m13 * self.m22;

        let determinant =
            self.m11 * minor_m12_m23 - self.m12 * minor_m11_m23 + self.m13 * minor_m11_m22;

        if determinant.is_zero() {
            Self::zero()
        } else {
            SdpMatrix3 {
                m11: minor_m12_m23 / determinant,
                m12: -minor_m11_m23 / determinant,
                m13: minor_m11_m22 / determinant,
                m22: (self.m11 * self.m33 - self.m13 * self.m13) / determinant,
                m23: (self.m13 * self.m12 - self.m23 * self.m11) / determinant,
                m33: (self.m11 * self.m22 - self.m12 * self.m12) / determinant,
            }
        }
    }

    fn squared(&self) -> Self {
        SdpMatrix3 {
            m11: self.m11 * self.m11 + self.m12 * self.m12 + self.m13 * self.m13,
            m12: self.m11 * self.m12 + self.m12 * self.m22 + self.m13 * self.m23,
            m13: self.m11 * self.m13 + self.m12 * self.m23 + self.m13 * self.m33,
            m22: self.m12 * self.m12 + self.m22 * self.m22 + self.m23 * self.m23,
            m23: self.m12 * self.m13 + self.m22 * self.m23 + self.m23 * self.m33,
            m33: self.m13 * self.m13 + self.m23 * self.m23 + self.m33 * self.m33,
        }
    }

    fn transform_lin_vector(&self, v: Vector3<Real>) -> Vector3<Real> {
        self.transform_vector(v)
    }

    fn transform_vector(&self, v: Vector3<Real>) -> Vector3<Real> {
        let x = self.m11 * v.x + self.m12 * v.y + self.m13 * v.z;
        let y = self.m12 * v.x + self.m22 * v.y + self.m23 * v.z;
        let z = self.m13 * v.x + self.m23 * v.y + self.m33 * v.z;
        Vector3::new(x, y, z)
    }

    #[rustfmt::skip]
    fn into_matrix(self) -> Matrix3<Real> {
        Matrix3::new(
            self.m11, self.m12, self.m13,
            self.m12, self.m22, self.m23,
            self.m13, self.m23, self.m33,
        )
    }

    #[rustfmt::skip]
    fn transform_matrix(&self, m: &Matrix3<Real>) -> Matrix3<Real> {
        *self * *m
    }
}

impl WAngularInertia<SimdReal> for SdpMatrix3<SimdReal> {
    type AngVector = Vector3<SimdReal>;
    type LinVector = Vector3<SimdReal>;
    type AngMatrix = Matrix3<SimdReal>;

    fn inverse(&self) -> Self {
        let minor_m12_m23 = self.m22 * self.m33 - self.m23 * self.m23;
        let minor_m11_m23 = self.m12 * self.m33 - self.m13 * self.m23;
        let minor_m11_m22 = self.m12 * self.m23 - self.m13 * self.m22;

        let determinant =
            self.m11 * minor_m12_m23 - self.m12 * minor_m11_m23 + self.m13 * minor_m11_m22;

        let zero = <SimdReal>::zero();
        let is_zero = determinant.simd_eq(zero);
        let inv_det = (<SimdReal>::one() / determinant).select(is_zero, zero);

        SdpMatrix3 {
            m11: minor_m12_m23 * inv_det,
            m12: -minor_m11_m23 * inv_det,
            m13: minor_m11_m22 * inv_det,
            m22: (self.m11 * self.m33 - self.m13 * self.m13) * inv_det,
            m23: (self.m13 * self.m12 - self.m23 * self.m11) * inv_det,
            m33: (self.m11 * self.m22 - self.m12 * self.m12) * inv_det,
        }
    }

    fn transform_lin_vector(&self, v: Vector3<SimdReal>) -> Vector3<SimdReal> {
        self.transform_vector(v)
    }

    fn transform_vector(&self, v: Vector3<SimdReal>) -> Vector3<SimdReal> {
        let x = self.m11 * v.x + self.m12 * v.y + self.m13 * v.z;
        let y = self.m12 * v.x + self.m22 * v.y + self.m23 * v.z;
        let z = self.m13 * v.x + self.m23 * v.y + self.m33 * v.z;
        Vector3::new(x, y, z)
    }

    fn squared(&self) -> Self {
        SdpMatrix3 {
            m11: self.m11 * self.m11 + self.m12 * self.m12 + self.m13 * self.m13,
            m12: self.m11 * self.m12 + self.m12 * self.m22 + self.m13 * self.m23,
            m13: self.m11 * self.m13 + self.m12 * self.m23 + self.m13 * self.m33,
            m22: self.m12 * self.m12 + self.m22 * self.m22 + self.m23 * self.m23,
            m23: self.m12 * self.m13 + self.m22 * self.m23 + self.m23 * self.m33,
            m33: self.m13 * self.m13 + self.m23 * self.m23 + self.m33 * self.m33,
        }
    }

    #[rustfmt::skip]
    fn into_matrix(self) -> Matrix3<SimdReal> {
        Matrix3::new(
            self.m11, self.m12, self.m13,
            self.m12, self.m22, self.m23,
            self.m13, self.m23, self.m33,
        )
    }

    #[rustfmt::skip]
    fn transform_matrix(&self, m: &Matrix3<SimdReal>) -> Matrix3<SimdReal> {
        let x0 = self.m11 * m.m11 + self.m12 * m.m21 + self.m13 * m.m31;
        let y0 = self.m12 * m.m11 + self.m22 * m.m21 + self.m23 * m.m31;
        let z0 = self.m13 * m.m11 + self.m23 * m.m21 + self.m33 * m.m31;

        let x1 = self.m11 * m.m12 + self.m12 * m.m22 + self.m13 * m.m32;
        let y1 = self.m12 * m.m12 + self.m22 * m.m22 + self.m23 * m.m32;
        let z1 = self.m13 * m.m12 + self.m23 * m.m22 + self.m33 * m.m32;

        let x2 = self.m11 * m.m13 + self.m12 * m.m23 + self.m13 * m.m33;
        let y2 = self.m12 * m.m13 + self.m22 * m.m23 + self.m23 * m.m33;
        let z2 = self.m13 * m.m13 + self.m23 * m.m23 + self.m33 * m.m33;

        Matrix3::new(
            x0, x1, x2,
            y0, y1, y2,
            z0, z1, z2,
        )
    }
}

// This is an RAII structure that enables flushing denormal numbers
// to zero, and automatically reseting previous flags once it is dropped.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct FlushToZeroDenormalsAreZeroFlags {
    original_flags: u32,
}

impl FlushToZeroDenormalsAreZeroFlags {
    #[cfg(not(all(
        not(feature = "enhanced-determinism"),
        any(target_arch = "x86_64", target_arch = "x86"),
        target_feature = "sse"
    )))]
    pub fn flush_denormal_to_zero() -> Self {
        Self { original_flags: 0 }
    }

    #[cfg(all(
        not(feature = "enhanced-determinism"),
        any(target_arch = "x86", target_arch = "x86_64"),
        target_feature = "sse"
    ))]
    pub fn flush_denormal_to_zero() -> Self {
        unsafe {
            #[cfg(target_arch = "x86")]
            use std::arch::x86::{_mm_getcsr, _mm_setcsr, _MM_FLUSH_ZERO_ON};
            #[cfg(target_arch = "x86_64")]
            use std::arch::x86_64::{_mm_getcsr, _mm_setcsr, _MM_FLUSH_ZERO_ON};

            // Flush denormals & underflows to zero as this as a significant impact on the solver's performances.
            // To enable this we need to set the bit 15 (given by _MM_FLUSH_ZERO_ON) and the bit 6 (for denormals-are-zero).
            // See https://software.intel.com/content/www/us/en/develop/articles/x87-and-sse-floating-point-assists-in-ia-32-flush-to-zero-ftz-and-denormals-are-zero-daz.html
            let original_flags = _mm_getcsr();
            _mm_setcsr(original_flags | _MM_FLUSH_ZERO_ON | (1 << 6));
            Self { original_flags }
        }
    }
}

#[cfg(all(
    not(feature = "enhanced-determinism"),
    any(target_arch = "x86", target_arch = "x86_64"),
    target_feature = "sse"
))]
impl Drop for FlushToZeroDenormalsAreZeroFlags {
    fn drop(&mut self) {
        #[cfg(target_arch = "x86")]
        unsafe {
            std::arch::x86::_mm_setcsr(self.original_flags)
        }
        #[cfg(target_arch = "x86_64")]
        unsafe {
            std::arch::x86_64::_mm_setcsr(self.original_flags)
        }
    }
}

pub(crate) fn select_other<T: PartialEq>(pair: (T, T), elt: T) -> T {
    if pair.0 == elt {
        pair.1
    } else {
        pair.0
    }
}
