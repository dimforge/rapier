use crate::utils::{
    SimdBasis, SimdCapMagnitude, SimdComponent, SimdCross, SimdCrossMatrix, SimdDot, SimdQuat,
    SimdSign, SimdVec,
};

use glam::{Vec2, Vec3};
use num::Zero;
use parry::math::{SimdMat3, SimdReal, SimdVec2, SimdVec3};

use {crate::math::*, na::SimdValue};

/*
 *
 * SimdSign
 *
 */
impl SimdSign<SimdVec2> for SimdReal {
    fn copy_sign_to(self, to: SimdVec2) -> SimdVec2 {
        SimdVec2::new(self.copy_sign_to(to.x), self.copy_sign_to(to.y))
    }
}

impl SimdSign<SimdVec3> for SimdReal {
    fn copy_sign_to(self, to: SimdVec3) -> SimdVec3 {
        SimdVec3::new(
            self.copy_sign_to(to.x),
            self.copy_sign_to(to.y),
            self.copy_sign_to(to.z),
        )
    }
}

impl SimdSign<SimdVec2> for SimdVec2 {
    fn copy_sign_to(self, to: SimdVec2) -> SimdVec2 {
        SimdVec2::new(self.x.copy_sign_to(to.x), self.y.copy_sign_to(to.y))
    }
}

impl SimdSign<SimdVec3> for SimdVec3 {
    fn copy_sign_to(self, to: SimdVec3) -> SimdVec3 {
        SimdVec3::new(
            self.x.copy_sign_to(to.x),
            self.y.copy_sign_to(to.y),
            self.z.copy_sign_to(to.z),
        )
    }
}

impl SimdBasis for SimdVec2 {
    type Basis = [SimdVec2; 1];
    fn orthonormal_basis(self) -> [SimdVec2; 1] {
        [SimdVec2::new(-self.y, self.x)]
    }
    fn orthonormal_vector(self) -> SimdVec2 {
        SimdVec2::new(-self.y, self.x)
    }
}

impl SimdBasis for SimdVec3 {
    type Basis = [SimdVec3; 2];
    // Robust and branchless implementation from Pixar:
    // https://graphics.pixar.com/library/OrthonormalB/paper.pdf
    fn orthonormal_basis(self) -> [SimdVec3; 2] {
        let one = SimdReal::splat(1.0);
        let sign = self.z.copy_sign_to(one);
        let a = -one / (sign + self.z);
        let b = self.x * self.y * a;

        [
            SimdVec3::new(one + sign * self.x * self.x * a, sign * b, -sign * self.x),
            SimdVec3::new(b, sign + self.y * self.y * a, -self.y),
        ]
    }

    fn orthonormal_vector(self) -> SimdVec3 {
        let one = SimdReal::splat(1.0);
        let sign = self.z.copy_sign_to(one);
        let a = -one / (sign + self.z);
        let b = self.x * self.y * a;
        SimdVec3::new(b, sign + self.y * self.y * a, -self.y)
    }
}

impl SimdCapMagnitude<SimdReal> for SimdVec2 {
    fn simd_cap_magnitude(&self, max: SimdReal) -> Self {
        use na::SimdPartialOrd;
        let n = self.length();
        let scaled = *self * (max / n);
        let use_scaled = n.simd_gt(max);
        scaled.select(use_scaled, *self)
    }
}

impl SimdVec for SimdVec2 {
    type Element = Vec2;

    fn horizontal_inf(&self) -> Self::Element {
        Vec2::new(self.x.min_component(), self.y.min_component())
    }

    fn horizontal_sup(&self) -> Self::Element {
        Vec2::new(self.x.max_component(), self.y.max_component())
    }

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        *self * *rhs
    }
}

impl SimdVec for SimdVec3 {
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

impl SimdCrossMatrix for SimdVec3 {
    type CrossMat = SimdMat3;
    type CrossMatTr = SimdMat3;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        let zero = SimdReal::splat(0.0);
        SimdMat3::new(
            zero, -self.z, self.y,
            self.z, zero, -self.x,
            -self.y, self.x, zero,
        )
    }

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        let zero = SimdReal::splat(0.0);
        SimdMat3::new(
            zero, self.z, -self.y,
            -self.z, zero, self.x,
            self.y, -self.x, zero,
        )
    }
}

impl SimdCrossMatrix for SimdVec2 {
    type CrossMat = SimdVec2;
    type CrossMatTr = SimdVec2;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        SimdVec2::new(-self.y, self.x)
    }
    #[inline]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        SimdVec2::new(-self.y, self.x)
    }
}

impl SimdCrossMatrix for SimdReal {
    type CrossMat = SimdMat2;
    type CrossMatTr = SimdMat2;

    #[inline]
    fn gcross_matrix(self) -> SimdMat2 {
        let zero = SimdReal::zero();
        SimdMat2::new(zero, -self, self, zero)
    }

    #[inline]
    fn gcross_matrix_tr(self) -> SimdMat2 {
        let zero = SimdReal::zero();
        SimdMat2::new(zero, self, -self, zero)
    }
}

impl SimdCross<SimdVec3> for SimdVec3 {
    type Result = Self;

    fn gcross(&self, rhs: SimdVec3) -> Self::Result {
        self.cross(rhs)
    }
    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        self.cross(*rhs)
    }
}

impl SimdCross<SimdVec2> for SimdVec2 {
    type Result = SimdReal;

    fn gcross(&self, rhs: SimdVec2) -> Self::Result {
        self.x * rhs.y - self.y * rhs.x
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl SimdDot<SimdVec3> for SimdVec3 {
    type Result = SimdReal;

    fn gdot(&self, rhs: SimdVec3) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl SimdDot<SimdVec2> for SimdVec2 {
    type Result = SimdReal;

    fn gdot(&self, rhs: SimdVec2) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl SimdCross<SimdVec2> for SimdReal {
    type Result = SimdVec2;

    fn gcross(&self, rhs: SimdVec2) -> Self::Result {
        SimdVec2::new(-rhs.y * *self, rhs.x * *self)
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        unreachable!()
    }
}

impl SimdQuat<SimdReal> for parry::math::SimdQuat {
    type Result = SimdMat3;

    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        let half = SimdReal::splat(0.5);
        let v1 = self.xyz();
        let v2 = rhs.xyz();
        let w1 = self.w;
        let w2 = rhs.w;

        // TODO: this can probably be optimized a lot by unrolling the ops.
        (v1.outer_product(v2) + SimdMat3::from_diagonal_element(w1 * w2)
            - (v1 * w2 + v2 * w1).gcross_matrix()
            + v1.gcross_matrix() * v2.gcross_matrix())
            * half
    }
}
