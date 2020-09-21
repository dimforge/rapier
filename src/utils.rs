//! Miscellaneous utilities.

use crate::dynamics::RigidBodyHandle;
#[cfg(all(feature = "enhanced-determinism", feature = "serde-serialize"))]
use indexmap::IndexMap as HashMap;
use na::{Matrix2, Matrix3, Matrix3x2, Point2, Point3, Scalar, SimdRealField, Vector2, Vector3};
use num::Zero;
#[cfg(feature = "simd-is-enabled")]
use simba::simd::SimdValue;
#[cfg(all(not(feature = "enhanced-determinism"), feature = "serde-serialize"))]
use std::collections::HashMap;
use std::ops::{Add, Mul};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::simd::{SimdBool, SimdFloat},
    na::SimdPartialOrd,
    num::One,
};

// pub(crate) const SIN_10_DEGREES: f32 = 0.17364817766;
// pub(crate) const COS_10_DEGREES: f32 = 0.98480775301;
// pub(crate) const COS_45_DEGREES: f32 = 0.70710678118;
// pub(crate) const SIN_45_DEGREES: f32 = COS_45_DEGREES;
pub(crate) const COS_5_DEGREES: f32 = 0.99619469809;
#[cfg(feature = "dim2")]
pub(crate) const COS_FRAC_PI_8: f32 = 0.92387953251;
#[cfg(feature = "dim2")]
pub(crate) const SIN_FRAC_PI_8: f32 = 0.38268343236;

pub(crate) fn inv(val: f32) -> f32 {
    if val == 0.0 {
        0.0
    } else {
        1.0 / val
    }
}

/// Conditionally swaps each lanes of `a` with those of `b`.
///
/// For each `i in [0..SIMD_WIDTH[`, if `do_swap.extract(i)` is `true` then
/// `a.extract(i)` is swapped with `b.extract(i)`.
#[cfg(feature = "simd-is-enabled")]
pub fn simd_swap(do_swap: SimdBool, a: &mut SimdFloat, b: &mut SimdFloat) {
    let _a = *a;
    *a = b.select(do_swap, *a);
    *b = _a.select(do_swap, *b);
}

/// Trait to copy the sign of each component of one scalar/vector/matrix to another.
pub trait WSign<Rhs>: Sized {
    // See SIMD implementations of copy_sign there: https://stackoverflow.com/a/57872652
    /// Copy the sign of each component of `self` to the corresponding component of `to`.
    fn copy_sign_to(self, to: Rhs) -> Rhs;
}

impl WSign<f32> for f32 {
    fn copy_sign_to(self, to: Self) -> Self {
        let signbit: u32 = (-0.0f32).to_bits();
        f32::from_bits((signbit & self.to_bits()) | ((!signbit) & to.to_bits()))
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

#[cfg(feature = "simd-is-enabled")]
impl WSign<SimdFloat> for SimdFloat {
    fn copy_sign_to(self, to: SimdFloat) -> SimdFloat {
        self.simd_copysign(to)
    }
}

pub(crate) trait WComponent: Sized {
    type Element;

    fn min_component(self) -> Self::Element;
    fn max_component(self) -> Self::Element;
}

impl WComponent for f32 {
    type Element = f32;

    fn min_component(self) -> Self::Element {
        self
    }
    fn max_component(self) -> Self::Element {
        self
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WComponent for SimdFloat {
    type Element = f32;

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

impl WCrossMatrix for Vector3<f32> {
    type CrossMat = Matrix3<f32>;

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

impl WCrossMatrix for Vector2<f32> {
    type CrossMat = Vector2<f32>;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        Vector2::new(-self.y, self.x)
    }
}

pub(crate) trait WCross<Rhs>: Sized {
    type Result;
    fn gcross(&self, rhs: Rhs) -> Self::Result;
}

impl WCross<Vector3<f32>> for Vector3<f32> {
    type Result = Self;

    fn gcross(&self, rhs: Vector3<f32>) -> Self::Result {
        self.cross(&rhs)
    }
}

impl WCross<Vector2<f32>> for Vector2<f32> {
    type Result = f32;

    fn gcross(&self, rhs: Vector2<f32>) -> Self::Result {
        self.x * rhs.y - self.y * rhs.x
    }
}

impl WCross<Vector2<f32>> for f32 {
    type Result = Vector2<f32>;

    fn gcross(&self, rhs: Vector2<f32>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }
}

pub(crate) trait WDot<Rhs>: Sized {
    type Result;
    fn gdot(&self, rhs: Rhs) -> Self::Result;
}

impl WDot<Vector3<f32>> for Vector3<f32> {
    type Result = f32;

    fn gdot(&self, rhs: Vector3<f32>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl WDot<Vector2<f32>> for Vector2<f32> {
    type Result = f32;

    fn gdot(&self, rhs: Vector2<f32>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl WDot<f32> for f32 {
    type Result = f32;

    fn gdot(&self, rhs: f32) -> Self::Result {
        *self * rhs
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WCrossMatrix for Vector3<SimdFloat> {
    type CrossMat = Matrix3<SimdFloat>;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        Matrix3::new(
            SimdFloat::zero(), -self.z, self.y,
            self.z, SimdFloat::zero(), -self.x,
            -self.y, self.x, SimdFloat::zero(),
        )
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WCrossMatrix for Vector2<SimdFloat> {
    type CrossMat = Vector2<SimdFloat>;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        Vector2::new(-self.y, self.x)
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WCross<Vector3<SimdFloat>> for Vector3<SimdFloat> {
    type Result = Vector3<SimdFloat>;

    fn gcross(&self, rhs: Self) -> Self::Result {
        self.cross(&rhs)
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WCross<Vector2<SimdFloat>> for SimdFloat {
    type Result = Vector2<SimdFloat>;

    fn gcross(&self, rhs: Vector2<SimdFloat>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WCross<Vector2<SimdFloat>> for Vector2<SimdFloat> {
    type Result = SimdFloat;

    fn gcross(&self, rhs: Self) -> Self::Result {
        let yx = Vector2::new(rhs.y, rhs.x);
        let prod = self.component_mul(&yx);
        prod.x - prod.y
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WDot<Vector3<SimdFloat>> for Vector3<SimdFloat> {
    type Result = SimdFloat;

    fn gdot(&self, rhs: Vector3<SimdFloat>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WDot<Vector2<SimdFloat>> for Vector2<SimdFloat> {
    type Result = SimdFloat;

    fn gdot(&self, rhs: Vector2<SimdFloat>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WDot<SimdFloat> for SimdFloat {
    type Result = SimdFloat;

    fn gdot(&self, rhs: SimdFloat) -> Self::Result {
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

impl WAngularInertia<f32> for f32 {
    type AngVector = f32;
    type LinVector = Vector2<f32>;
    type AngMatrix = f32;

    fn inverse(&self) -> Self {
        if *self != 0.0 {
            1.0 / *self
        } else {
            0.0
        }
    }

    fn transform_lin_vector(&self, pt: Vector2<f32>) -> Vector2<f32> {
        *self * pt
    }
    fn transform_vector(&self, pt: f32) -> f32 {
        *self * pt
    }

    fn squared(&self) -> f32 {
        *self * *self
    }

    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix {
        mat * *self
    }

    fn into_matrix(self) -> Self::AngMatrix {
        self
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WAngularInertia<SimdFloat> for SimdFloat {
    type AngVector = SimdFloat;
    type LinVector = Vector2<SimdFloat>;
    type AngMatrix = SimdFloat;

    fn inverse(&self) -> Self {
        let zero = <SimdFloat>::zero();
        let is_zero = self.simd_eq(zero);
        (<SimdFloat>::one() / *self).select(is_zero, zero)
    }

    fn transform_lin_vector(&self, pt: Vector2<SimdFloat>) -> Vector2<SimdFloat> {
        pt * *self
    }

    fn transform_vector(&self, pt: SimdFloat) -> SimdFloat {
        *self * pt
    }

    fn squared(&self) -> SimdFloat {
        *self * *self
    }

    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix {
        *mat * *self
    }

    fn into_matrix(self) -> Self::AngMatrix {
        self
    }
}

/// A 2x2 symmetric-definite-positive matrix.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SdpMatrix2<N> {
    /// The component at the first row and first column of this matrix.
    pub m11: N,
    /// The component at the first row and second column of this matrix.
    pub m12: N,
    /// The component at the second row and second column of this matrix.
    pub m22: N,
}

impl<N: SimdRealField> SdpMatrix2<N> {
    /// A new SDP 2x2 matrix with the given components.
    ///
    /// Because the matrix is symmetric, only the lower off-diagonal component is required.
    pub fn new(m11: N, m12: N, m22: N) -> Self {
        Self { m11, m12, m22 }
    }

    /// Build an `SdpMatrix2` structure from a plain matrix, assuming it is SDP.
    ///
    /// No check is performed to ensure `mat` is actually SDP.
    pub fn from_sdp_matrix(mat: na::Matrix2<N>) -> Self {
        Self {
            m11: mat.m11,
            m12: mat.m12,
            m22: mat.m22,
        }
    }

    /// Create a new SDP matrix filled with zeros.
    pub fn zero() -> Self {
        Self {
            m11: N::zero(),
            m12: N::zero(),
            m22: N::zero(),
        }
    }

    /// Create a new SDP matrix with its diagonal filled with `val`, and its off-diagonal elements set to zero.
    pub fn diagonal(val: N) -> Self {
        Self {
            m11: val,
            m12: N::zero(),
            m22: val,
        }
    }

    /// Adds `val` to the diagonal components of `self`.
    pub fn add_diagonal(&mut self, elt: N) -> Self {
        Self {
            m11: self.m11 + elt,
            m12: self.m12,
            m22: self.m22 + elt,
        }
    }

    /// Compute the inverse of this SDP matrix without performing any inversibility check.
    pub fn inverse_unchecked(&self) -> Self {
        let determinant = self.m11 * self.m22 - self.m12 * self.m12;
        let m11 = self.m22 / determinant;
        let m12 = -self.m12 / determinant;
        let m22 = self.m11 / determinant;

        Self { m11, m12, m22 }
    }

    /// Convert this SDP matrix to a regular matrix representation.
    pub fn into_matrix(self) -> Matrix2<N> {
        Matrix2::new(self.m11, self.m12, self.m12, self.m22)
    }
}

impl<N: SimdRealField> Add<SdpMatrix2<N>> for SdpMatrix2<N> {
    type Output = Self;

    fn add(self, rhs: SdpMatrix2<N>) -> Self {
        Self::new(self.m11 + rhs.m11, self.m12 + rhs.m12, self.m22 + rhs.m22)
    }
}

impl<N: SimdRealField> Mul<Vector2<N>> for SdpMatrix2<N> {
    type Output = Vector2<N>;

    fn mul(self, rhs: Vector2<N>) -> Self::Output {
        Vector2::new(
            self.m11 * rhs.x + self.m12 * rhs.y,
            self.m12 * rhs.x + self.m22 * rhs.y,
        )
    }
}

/// A 3x3 symmetric-definite-positive matrix.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SdpMatrix3<N> {
    /// The component at the first row and first column of this matrix.
    pub m11: N,
    /// The component at the first row and second column of this matrix.
    pub m12: N,
    /// The component at the first row and third column of this matrix.
    pub m13: N,
    /// The component at the second row and second column of this matrix.
    pub m22: N,
    /// The component at the second row and third column of this matrix.
    pub m23: N,
    /// The component at the third row and third column of this matrix.
    pub m33: N,
}

impl<N: SimdRealField> SdpMatrix3<N> {
    /// A new SDP 3x3 matrix with the given components.
    ///
    /// Because the matrix is symmetric, only the lower off-diagonal components is required.
    pub fn new(m11: N, m12: N, m13: N, m22: N, m23: N, m33: N) -> Self {
        Self {
            m11,
            m12,
            m13,
            m22,
            m23,
            m33,
        }
    }

    /// Build an `SdpMatrix3` structure from a plain matrix, assuming it is SDP.
    ///
    /// No check is performed to ensure `mat` is actually SDP.
    pub fn from_sdp_matrix(mat: na::Matrix3<N>) -> Self {
        Self {
            m11: mat.m11,
            m12: mat.m12,
            m13: mat.m13,
            m22: mat.m22,
            m23: mat.m23,
            m33: mat.m33,
        }
    }

    /// Create a new SDP matrix filled with zeros.
    pub fn zero() -> Self {
        Self {
            m11: N::zero(),
            m12: N::zero(),
            m13: N::zero(),
            m22: N::zero(),
            m23: N::zero(),
            m33: N::zero(),
        }
    }

    /// Create a new SDP matrix with its diagonal filled with `val`, and its off-diagonal elements set to zero.
    pub fn diagonal(val: N) -> Self {
        Self {
            m11: val,
            m12: N::zero(),
            m13: N::zero(),
            m22: val,
            m23: N::zero(),
            m33: val,
        }
    }

    /// Are all components of this matrix equal to zero?
    pub fn is_zero(&self) -> bool {
        self.m11.is_zero()
            && self.m12.is_zero()
            && self.m13.is_zero()
            && self.m22.is_zero()
            && self.m23.is_zero()
            && self.m33.is_zero()
    }

    /// Compute the inverse of this SDP matrix without performing any inversibility check.
    pub fn inverse_unchecked(&self) -> Self {
        let minor_m12_m23 = self.m22 * self.m33 - self.m23 * self.m23;
        let minor_m11_m23 = self.m12 * self.m33 - self.m13 * self.m23;
        let minor_m11_m22 = self.m12 * self.m23 - self.m13 * self.m22;

        let determinant =
            self.m11 * minor_m12_m23 - self.m12 * minor_m11_m23 + self.m13 * minor_m11_m22;
        let inv_det = N::one() / determinant;

        SdpMatrix3 {
            m11: minor_m12_m23 * inv_det,
            m12: -minor_m11_m23 * inv_det,
            m13: minor_m11_m22 * inv_det,
            m22: (self.m11 * self.m33 - self.m13 * self.m13) * inv_det,
            m23: (self.m13 * self.m12 - self.m23 * self.m11) * inv_det,
            m33: (self.m11 * self.m22 - self.m12 * self.m12) * inv_det,
        }
    }

    /// Compute the quadratic form `m.transpose() * self * m`.
    pub fn quadform3x2(&self, m: &Matrix3x2<N>) -> SdpMatrix2<N> {
        let x0 = self.m11 * m.m11 + self.m12 * m.m21 + self.m13 * m.m31;
        let y0 = self.m12 * m.m11 + self.m22 * m.m21 + self.m23 * m.m31;
        let z0 = self.m13 * m.m11 + self.m23 * m.m21 + self.m33 * m.m31;

        let x1 = self.m11 * m.m12 + self.m12 * m.m22 + self.m13 * m.m32;
        let y1 = self.m12 * m.m12 + self.m22 * m.m22 + self.m23 * m.m32;
        let z1 = self.m13 * m.m12 + self.m23 * m.m22 + self.m33 * m.m32;

        let m11 = m.m11 * x0 + m.m21 * y0 + m.m31 * z0;
        let m12 = m.m11 * x1 + m.m21 * y1 + m.m31 * z1;
        let m22 = m.m12 * x1 + m.m22 * y1 + m.m32 * z1;

        SdpMatrix2 { m11, m12, m22 }
    }

    /// Compute the quadratic form `m.transpose() * self * m`.
    pub fn quadform(&self, m: &Matrix3<N>) -> Self {
        let x0 = self.m11 * m.m11 + self.m12 * m.m21 + self.m13 * m.m31;
        let y0 = self.m12 * m.m11 + self.m22 * m.m21 + self.m23 * m.m31;
        let z0 = self.m13 * m.m11 + self.m23 * m.m21 + self.m33 * m.m31;

        let x1 = self.m11 * m.m12 + self.m12 * m.m22 + self.m13 * m.m32;
        let y1 = self.m12 * m.m12 + self.m22 * m.m22 + self.m23 * m.m32;
        let z1 = self.m13 * m.m12 + self.m23 * m.m22 + self.m33 * m.m32;

        let x2 = self.m11 * m.m13 + self.m12 * m.m23 + self.m13 * m.m33;
        let y2 = self.m12 * m.m13 + self.m22 * m.m23 + self.m23 * m.m33;
        let z2 = self.m13 * m.m13 + self.m23 * m.m23 + self.m33 * m.m33;

        let m11 = m.m11 * x0 + m.m21 * y0 + m.m31 * z0;
        let m12 = m.m11 * x1 + m.m21 * y1 + m.m31 * z1;
        let m13 = m.m11 * x2 + m.m21 * y2 + m.m31 * z2;

        let m22 = m.m12 * x1 + m.m22 * y1 + m.m32 * z1;
        let m23 = m.m12 * x2 + m.m22 * y2 + m.m32 * z2;
        let m33 = m.m13 * x2 + m.m23 * y2 + m.m33 * z2;

        Self {
            m11,
            m12,
            m13,
            m22,
            m23,
            m33,
        }
    }

    /// Adds `elt` to the diagonal components of `self`.
    pub fn add_diagonal(&self, elt: N) -> Self {
        Self {
            m11: self.m11 + elt,
            m12: self.m12,
            m13: self.m13,
            m22: self.m22 + elt,
            m23: self.m23,
            m33: self.m33 + elt,
        }
    }
}

impl<N: Add<N>> Add<SdpMatrix3<N>> for SdpMatrix3<N> {
    type Output = SdpMatrix3<N::Output>;

    fn add(self, rhs: SdpMatrix3<N>) -> Self::Output {
        SdpMatrix3 {
            m11: self.m11 + rhs.m11,
            m12: self.m12 + rhs.m12,
            m13: self.m13 + rhs.m13,
            m22: self.m22 + rhs.m22,
            m23: self.m23 + rhs.m23,
            m33: self.m33 + rhs.m33,
        }
    }
}

impl<N: SimdRealField> Mul<Vector3<N>> for SdpMatrix3<N> {
    type Output = Vector3<N>;

    fn mul(self, rhs: Vector3<N>) -> Self::Output {
        let x = self.m11 * rhs.x + self.m12 * rhs.y + self.m13 * rhs.z;
        let y = self.m12 * rhs.x + self.m22 * rhs.y + self.m23 * rhs.z;
        let z = self.m13 * rhs.x + self.m23 * rhs.y + self.m33 * rhs.z;
        Vector3::new(x, y, z)
    }
}

impl<N: SimdRealField> Mul<Matrix3<N>> for SdpMatrix3<N> {
    type Output = Matrix3<N>;

    fn mul(self, rhs: Matrix3<N>) -> Self::Output {
        let x0 = self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31;
        let y0 = self.m12 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31;
        let z0 = self.m13 * rhs.m11 + self.m23 * rhs.m21 + self.m33 * rhs.m31;

        let x1 = self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32;
        let y1 = self.m12 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32;
        let z1 = self.m13 * rhs.m12 + self.m23 * rhs.m22 + self.m33 * rhs.m32;

        let x2 = self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33;
        let y2 = self.m12 * rhs.m13 + self.m22 * rhs.m23 + self.m23 * rhs.m33;
        let z2 = self.m13 * rhs.m13 + self.m23 * rhs.m23 + self.m33 * rhs.m33;

        Matrix3::new(x0, x1, x2, y0, y1, y2, z0, z1, z2)
    }
}

impl<N: SimdRealField> Mul<Matrix3x2<N>> for SdpMatrix3<N> {
    type Output = Matrix3x2<N>;

    fn mul(self, rhs: Matrix3x2<N>) -> Self::Output {
        let x0 = self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31;
        let y0 = self.m12 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31;
        let z0 = self.m13 * rhs.m11 + self.m23 * rhs.m21 + self.m33 * rhs.m31;

        let x1 = self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32;
        let y1 = self.m12 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32;
        let z1 = self.m13 * rhs.m12 + self.m23 * rhs.m22 + self.m33 * rhs.m32;

        Matrix3x2::new(x0, x1, y0, y1, z0, z1)
    }
}

impl WAngularInertia<f32> for SdpMatrix3<f32> {
    type AngVector = Vector3<f32>;
    type LinVector = Vector3<f32>;
    type AngMatrix = Matrix3<f32>;

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

    fn transform_lin_vector(&self, v: Vector3<f32>) -> Vector3<f32> {
        self.transform_vector(v)
    }

    fn transform_vector(&self, v: Vector3<f32>) -> Vector3<f32> {
        let x = self.m11 * v.x + self.m12 * v.y + self.m13 * v.z;
        let y = self.m12 * v.x + self.m22 * v.y + self.m23 * v.z;
        let z = self.m13 * v.x + self.m23 * v.y + self.m33 * v.z;
        Vector3::new(x, y, z)
    }

    #[rustfmt::skip]
    fn into_matrix(self) -> Matrix3<f32> {
        Matrix3::new(
            self.m11, self.m12, self.m13,
            self.m12, self.m22, self.m23,
            self.m13, self.m23, self.m33,
        )
    }

    #[rustfmt::skip]
    fn transform_matrix(&self, m: &Matrix3<f32>) -> Matrix3<f32> {
        *self * *m
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WAngularInertia<SimdFloat> for SdpMatrix3<SimdFloat> {
    type AngVector = Vector3<SimdFloat>;
    type LinVector = Vector3<SimdFloat>;
    type AngMatrix = Matrix3<SimdFloat>;

    fn inverse(&self) -> Self {
        let minor_m12_m23 = self.m22 * self.m33 - self.m23 * self.m23;
        let minor_m11_m23 = self.m12 * self.m33 - self.m13 * self.m23;
        let minor_m11_m22 = self.m12 * self.m23 - self.m13 * self.m22;

        let determinant =
            self.m11 * minor_m12_m23 - self.m12 * minor_m11_m23 + self.m13 * minor_m11_m22;

        let zero = <SimdFloat>::zero();
        let is_zero = determinant.simd_eq(zero);
        let inv_det = (<SimdFloat>::one() / determinant).select(is_zero, zero);

        SdpMatrix3 {
            m11: minor_m12_m23 * inv_det,
            m12: -minor_m11_m23 * inv_det,
            m13: minor_m11_m22 * inv_det,
            m22: (self.m11 * self.m33 - self.m13 * self.m13) * inv_det,
            m23: (self.m13 * self.m12 - self.m23 * self.m11) * inv_det,
            m33: (self.m11 * self.m22 - self.m12 * self.m12) * inv_det,
        }
    }

    fn transform_lin_vector(&self, v: Vector3<SimdFloat>) -> Vector3<SimdFloat> {
        self.transform_vector(v)
    }

    fn transform_vector(&self, v: Vector3<SimdFloat>) -> Vector3<SimdFloat> {
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
    fn into_matrix(self) -> Matrix3<SimdFloat> {
        Matrix3::new(
            self.m11, self.m12, self.m13,
            self.m12, self.m22, self.m23,
            self.m13, self.m23, self.m33,
        )
    }

    #[rustfmt::skip]
    fn transform_matrix(&self, m: &Matrix3<SimdFloat>) -> Matrix3<SimdFloat> {
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

impl<T> From<[SdpMatrix3<f32>; 4]> for SdpMatrix3<T>
where
    T: From<[f32; 4]>,
{
    fn from(data: [SdpMatrix3<f32>; 4]) -> Self {
        SdpMatrix3 {
            m11: T::from([data[0].m11, data[1].m11, data[2].m11, data[3].m11]),
            m12: T::from([data[0].m12, data[1].m12, data[2].m12, data[3].m12]),
            m13: T::from([data[0].m13, data[1].m13, data[2].m13, data[3].m13]),
            m22: T::from([data[0].m22, data[1].m22, data[2].m22, data[3].m22]),
            m23: T::from([data[0].m23, data[1].m23, data[2].m23, data[3].m23]),
            m33: T::from([data[0].m33, data[1].m33, data[2].m33, data[3].m33]),
        }
    }
}

#[cfg(feature = "simd-nightly")]
impl From<[SdpMatrix3<f32>; 8]> for SdpMatrix3<simba::simd::f32x8> {
    fn from(data: [SdpMatrix3<f32>; 8]) -> Self {
        SdpMatrix3 {
            m11: simba::simd::f32x8::from([
                data[0].m11,
                data[1].m11,
                data[2].m11,
                data[3].m11,
                data[4].m11,
                data[5].m11,
                data[6].m11,
                data[7].m11,
            ]),
            m12: simba::simd::f32x8::from([
                data[0].m12,
                data[1].m12,
                data[2].m12,
                data[3].m12,
                data[4].m12,
                data[5].m12,
                data[6].m12,
                data[7].m12,
            ]),
            m13: simba::simd::f32x8::from([
                data[0].m13,
                data[1].m13,
                data[2].m13,
                data[3].m13,
                data[4].m13,
                data[5].m13,
                data[6].m13,
                data[7].m13,
            ]),
            m22: simba::simd::f32x8::from([
                data[0].m22,
                data[1].m22,
                data[2].m22,
                data[3].m22,
                data[4].m22,
                data[5].m22,
                data[6].m22,
                data[7].m22,
            ]),
            m23: simba::simd::f32x8::from([
                data[0].m23,
                data[1].m23,
                data[2].m23,
                data[3].m23,
                data[4].m23,
                data[5].m23,
                data[6].m23,
                data[7].m23,
            ]),
            m33: simba::simd::f32x8::from([
                data[0].m33,
                data[1].m33,
                data[2].m33,
                data[3].m33,
                data[4].m33,
                data[5].m33,
                data[6].m33,
                data[7].m33,
            ]),
        }
    }
}

#[cfg(feature = "simd-nightly")]
impl From<[SdpMatrix3<f32>; 16]> for SdpMatrix3<simba::simd::f32x16> {
    fn from(data: [SdpMatrix3<f32>; 16]) -> Self {
        SdpMatrix3 {
            m11: simba::simd::f32x16::from([
                data[0].m11,
                data[1].m11,
                data[2].m11,
                data[3].m11,
                data[4].m11,
                data[5].m11,
                data[6].m11,
                data[7].m11,
                data[8].m11,
                data[9].m11,
                data[10].m11,
                data[11].m11,
                data[12].m11,
                data[13].m11,
                data[14].m11,
                data[15].m11,
            ]),
            m12: simba::simd::f32x16::from([
                data[0].m12,
                data[1].m12,
                data[2].m12,
                data[3].m12,
                data[4].m12,
                data[5].m12,
                data[6].m12,
                data[7].m12,
                data[8].m12,
                data[9].m12,
                data[10].m12,
                data[11].m12,
                data[12].m12,
                data[13].m12,
                data[14].m12,
                data[15].m12,
            ]),
            m13: simba::simd::f32x16::from([
                data[0].m13,
                data[1].m13,
                data[2].m13,
                data[3].m13,
                data[4].m13,
                data[5].m13,
                data[6].m13,
                data[7].m13,
                data[8].m13,
                data[9].m13,
                data[10].m13,
                data[11].m13,
                data[12].m13,
                data[13].m13,
                data[14].m13,
                data[15].m13,
            ]),
            m22: simba::simd::f32x16::from([
                data[0].m22,
                data[1].m22,
                data[2].m22,
                data[3].m22,
                data[4].m22,
                data[5].m22,
                data[6].m22,
                data[7].m22,
                data[8].m22,
                data[9].m22,
                data[10].m22,
                data[11].m22,
                data[12].m22,
                data[13].m22,
                data[14].m22,
                data[15].m22,
            ]),
            m23: simba::simd::f32x16::from([
                data[0].m23,
                data[1].m23,
                data[2].m23,
                data[3].m23,
                data[4].m23,
                data[5].m23,
                data[6].m23,
                data[7].m23,
                data[8].m23,
                data[9].m23,
                data[10].m23,
                data[11].m23,
                data[12].m23,
                data[13].m23,
                data[14].m23,
                data[15].m23,
            ]),
            m33: simba::simd::f32x16::from([
                data[0].m33,
                data[1].m33,
                data[2].m33,
                data[3].m33,
                data[4].m33,
                data[5].m33,
                data[6].m33,
                data[7].m33,
                data[8].m33,
                data[9].m33,
                data[10].m33,
                data[11].m33,
                data[12].m33,
                data[13].m33,
                data[14].m33,
                data[15].m33,
            ]),
        }
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

#[cfg(feature = "serde-serialize")]
pub(crate) fn serialize_hashmap_capacity<S: serde::Serializer, K, V, H: std::hash::BuildHasher>(
    map: &HashMap<K, V, H>,
    s: S,
) -> Result<S::Ok, S::Error> {
    s.serialize_u64(map.capacity() as u64)
}

#[cfg(feature = "serde-serialize")]
pub(crate) fn deserialize_hashmap_capacity<
    'de,
    D: serde::Deserializer<'de>,
    K,
    V,
    H: std::hash::BuildHasher + Default,
>(
    d: D,
) -> Result<HashMap<K, V, H>, D::Error> {
    struct CapacityVisitor;
    impl<'de> serde::de::Visitor<'de> for CapacityVisitor {
        type Value = u64;

        fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
            write!(formatter, "an integer between 0 and 2^64")
        }

        fn visit_u64<E: serde::de::Error>(self, val: u64) -> Result<Self::Value, E> {
            Ok(val)
        }
    }

    let capacity = d.deserialize_u64(CapacityVisitor)? as usize;
    Ok(HashMap::with_capacity_and_hasher(
        capacity,
        Default::default(),
    ))
}

/*
 * FxHasher taken from rustc_hash, except that it does not depend on the pointer size.
 */
#[cfg(feature = "enhanced-determinism")]
pub(crate) type FxHashMap32<K, V> =
    indexmap::IndexMap<K, V, std::hash::BuildHasherDefault<FxHasher32>>;

const K: u32 = 0x9e3779b9;

pub(crate) struct FxHasher32 {
    hash: u32,
}

impl Default for FxHasher32 {
    #[inline]
    fn default() -> FxHasher32 {
        FxHasher32 { hash: 0 }
    }
}

impl FxHasher32 {
    #[inline]
    fn add_to_hash(&mut self, i: u32) {
        use std::ops::BitXor;
        self.hash = self.hash.rotate_left(5).bitxor(i).wrapping_mul(K);
    }
}

impl std::hash::Hasher for FxHasher32 {
    #[inline]
    fn write(&mut self, mut bytes: &[u8]) {
        use std::convert::TryInto;
        let read_u32 = |bytes: &[u8]| u32::from_ne_bytes(bytes[..4].try_into().unwrap());
        let mut hash = FxHasher32 { hash: self.hash };
        assert!(std::mem::size_of::<u32>() <= 8);
        while bytes.len() >= std::mem::size_of::<u32>() {
            hash.add_to_hash(read_u32(bytes) as u32);
            bytes = &bytes[std::mem::size_of::<u32>()..];
        }
        if (std::mem::size_of::<u32>() > 4) && (bytes.len() >= 4) {
            hash.add_to_hash(u32::from_ne_bytes(bytes[..4].try_into().unwrap()) as u32);
            bytes = &bytes[4..];
        }
        if (std::mem::size_of::<u32>() > 2) && bytes.len() >= 2 {
            hash.add_to_hash(u16::from_ne_bytes(bytes[..2].try_into().unwrap()) as u32);
            bytes = &bytes[2..];
        }
        if (std::mem::size_of::<u32>() > 1) && bytes.len() >= 1 {
            hash.add_to_hash(bytes[0] as u32);
        }
        self.hash = hash.hash;
    }

    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u16(&mut self, i: u16) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add_to_hash(i as u32);
        self.add_to_hash((i >> 32) as u32);
    }

    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn finish(&self) -> u64 {
        self.hash as u64
    }
}

pub(crate) fn other_handle(
    pair: (RigidBodyHandle, RigidBodyHandle),
    handle: RigidBodyHandle,
) -> RigidBodyHandle {
    if pair.0 == handle {
        pair.1
    } else {
        pair.0
    }
}
