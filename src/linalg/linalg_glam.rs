use crate::utils::{
    SimdBasis, SimdCapMagnitude, SimdComponent, SimdCross, SimdCrossMatrix, SimdDot, SimdQuat,
    SimdRealCopy, SimdSign, SimdVec,
};

use glam::{Mat2, Mat3, Quat, Vec2, Vec3};
use na::{Matrix1, UnitComplex};
use parry::math::Real;

use crate::math::*;

/*
 *
 * SimdSign
 *
 */
impl SimdSign<Vec2> for Real {
    fn copy_sign_to(self, to: Vec2) -> Vec2 {
        Vec2::new(self.copy_sign_to(to.x), self.copy_sign_to(to.y))
    }
}

impl SimdSign<Vec3> for Real {
    fn copy_sign_to(self, to: Vec3) -> Vec3 {
        Vec3::new(
            self.copy_sign_to(to.x),
            self.copy_sign_to(to.y),
            self.copy_sign_to(to.z),
        )
    }
}

impl SimdSign<Vec2> for Vec2 {
    fn copy_sign_to(self, to: Vec2) -> Vec2 {
        Vec2::new(self.x.copy_sign_to(to.x), self.y.copy_sign_to(to.y))
    }
}

impl SimdSign<Vec3> for Vec3 {
    fn copy_sign_to(self, to: Vec3) -> Vec3 {
        Vec3::new(
            self.x.copy_sign_to(to.x),
            self.y.copy_sign_to(to.y),
            self.z.copy_sign_to(to.z),
        )
    }
}

impl SimdBasis for Vec2 {
    type Basis = [Vec2; 1];
    fn orthonormal_basis(self) -> [Vec2; 1] {
        [Vec2::new(-self.y, self.x)]
    }
    fn orthonormal_vector(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
}

impl SimdBasis for Vec3 {
    type Basis = [Vec3; 2];
    // Robust and branchless implementation from Pixar:
    // https://graphics.pixar.com/library/OrthonormalB/paper.pdf
    fn orthonormal_basis(self) -> [Vec3; 2] {
        let sign = self.z.copy_sign_to(1.0);
        let a = -1.0 / (sign + self.z);
        let b = self.x * self.y * a;

        [
            Vec3::new(1.0 + sign * self.x * self.x * a, sign * b, -sign * self.x),
            Vec3::new(b, sign + self.y * self.y * a, -self.y),
        ]
    }

    fn orthonormal_vector(self) -> Vec3 {
        let sign = self.z.copy_sign_to(1.0);
        let a = -1.0 / (sign + self.z);
        let b = self.x * self.y * a;
        Vec3::new(b, sign + self.y * self.y * a, -self.y)
    }
}

impl SimdCapMagnitude<Real> for Vec2 {
    fn simd_cap_magnitude(&self, max: Real) -> Self {
        use na::SimdPartialOrd;
        let n = self.length();
        let scaled = *self * (max / n);
        let use_scaled = n.simd_gt(max);
        scaled.select(use_scaled, *self)
    }
}

impl SimdVec for Vec2 {
    type Element = Vec2;

    fn horizontal_inf(&self) -> Self::Element {
        Point2::new(self.x.min_component(), self.y.min_component())
    }

    fn horizontal_sup(&self) -> Self::Element {
        Point2::new(self.x.max_component(), self.y.max_component())
    }

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        *self * *rhs
    }
}

impl SimdVec for Vec3 {
    type Element = Vec3;

    fn horizontal_inf(&self) -> Self::Element {
        Vec3::new(
            self.x.min_component(),
            self.y.min_component(),
            self.z.min_component(),
        )
    }

    fn horizontal_sup(&self) -> Self::Element {
        Vec3::new(
            self.x.max_component(),
            self.y.max_component(),
            self.z.max_component(),
        )
    }

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        self.component_mul(rhs)
    }
}

#[cfg(feature = "dim3")]
impl SimdCrossMatrix for Vec3 {
    type CrossMat = Mat3;
    type CrossMatTr = Mat3;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        Mat3::new(
            0.0, -self.z, self.y,
            self.z, 0.0, -self.x,
            -self.y, self.x, 0.0,
        )
    }

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        Mat3::new(
            0.0, self.z, -self.y,
            -self.z, 0.0, self.x,
            self.y, -self.x, 0.0,
        )
    }
}

impl SimdCrossMatrix for Vec2 {
    type CrossMat = Vec2;
    type CrossMatTr = Vec2;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        Vec2::new(-self.y, self.x)
    }
    #[inline]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        Vec2::new(-self.y, self.x)
    }
}

impl SimdCrossMatrix for Real {
    type CrossMat = Mat2;
    type CrossMatTr = Mat2;

    #[inline]
    fn gcross_matrix(self) -> Mat2 {
        Mat2::from_cols_array(&[0.0, self, -self, 0.0])
    }

    #[inline]
    fn gcross_matrix_tr(self) -> Mat2 {
        Mat2::from_cols_array(&[0.0, -self, self, 0.0])
    }
}

impl SimdCross<Vec3> for Vec3 {
    type Result = Self;

    fn gcross(&self, rhs: Vec3) -> Self::Result {
        self.cross(rhs)
    }
    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        self.cross(*rhs)
    }
}

impl SimdCross<Vec2> for Vec2 {
    type Result = Real;

    fn gcross(&self, rhs: Vec2) -> Self::Result {
        self.x * rhs.y - self.y * rhs.x
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl SimdCross<Vec2> for Real {
    type Result = Vec2;

    fn gcross(&self, rhs: Vec2) -> Self::Result {
        Vec2::new(-rhs.y * *self, rhs.x * *self)
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl SimdDot<Vec3> for Vec3 {
    type Result = Real;

    fn gdot(&self, rhs: Vec3) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl SimdDot<Vec2> for Vec2 {
    type Result = Real;

    fn gdot(&self, rhs: Vec2) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

#[cfg(feature = "dim3")]
impl SimdQuat<Real> for Quat {
    type Result = Mat3;

    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        let half = 0.5;
        let v1 = self.xyz();
        let v2 = rhs.xyz();
        let w1 = self.w;
        let w2 = rhs.w;

        // TODO: this can probably be optimized a lot by unrolling the ops.
        (v1.outer_product(&v2) + Mat3::from_diagonal_element(w1 * w2)
            - (v1 * w2 + v2 * w1).gcross_matrix()
            + v1.gcross_matrix() * v2.gcross_matrix())
            * half
    }
}

impl SimdQuat<Real> for Mat2 {
    type Result = Real;

    #[inline]
    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        let arr_self = self.as_ref();
        let arr_rhs = rhs.as_ref();
        (arr_self[1] * arr_rhs[1] + arr_self[0] * arr_rhs[0]) * 2.0
    }
}

impl SimdQuat<SimdReal> for SimdMat2 {
    type Result = SimdReal;

    #[inline]
    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        use na::SimdValue;
        (self[(1, 0)] * rhs[(1, 0)] + self[(0, 0)] * rhs[(0, 0)]) * SimdReal::splat(2.0)
    }
}
