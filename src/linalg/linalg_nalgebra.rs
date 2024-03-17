use crate::utils::{
    simd_inv, SimdAngularInertia, SimdBasis, SimdCapMagnitude, SimdComponent, SimdCross,
    SimdCrossMatrix, SimdDot, SimdQuat, SimdRealCopy, SimdSign, SimdVec,
};

use na::{
    Matrix1, Matrix2, Matrix3, Point2, Point3, RowVector2, Scalar, SimdRealField, UnitComplex,
    UnitQuaternion, Vector1, Vector2, Vector3,
};
use num::Zero;
use parry::math::{Real, SimdReal};
use simba::scalar::ClosedMul;

use na::SimdPartialOrd;

impl<N: Scalar + Copy + SimdSign<N>> SimdSign<Vector2<N>> for N {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.copy_sign_to(to.x), self.copy_sign_to(to.y))
    }
}

impl<N: Scalar + Copy + SimdSign<N>> SimdSign<Vector3<N>> for N {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.copy_sign_to(to.x),
            self.copy_sign_to(to.y),
            self.copy_sign_to(to.z),
        )
    }
}

impl<N: Scalar + Copy + SimdSign<N>> SimdSign<Vector2<N>> for Vector2<N> {
    fn copy_sign_to(self, to: Vector2<N>) -> Vector2<N> {
        Vector2::new(self.x.copy_sign_to(to.x), self.y.copy_sign_to(to.y))
    }
}

impl<N: Scalar + Copy + SimdSign<N>> SimdSign<Vector3<N>> for Vector3<N> {
    fn copy_sign_to(self, to: Vector3<N>) -> Vector3<N> {
        Vector3::new(
            self.x.copy_sign_to(to.x),
            self.y.copy_sign_to(to.y),
            self.z.copy_sign_to(to.z),
        )
    }
}

impl SimdSign<SimdReal> for SimdReal {
    fn copy_sign_to(self, to: SimdReal) -> SimdReal {
        to.simd_copysign(self)
    }
}

impl SimdComponent for Real {
    type Element = Real;

    fn min_component(self) -> Self::Element {
        self
    }
    fn max_component(self) -> Self::Element {
        self
    }
}

impl SimdComponent for SimdReal {
    type Element = Real;

    fn min_component(self) -> Self::Element {
        self.simd_horizontal_min()
    }
    fn max_component(self) -> Self::Element {
        self.simd_horizontal_max()
    }
}

impl<N: SimdRealCopy> SimdBasis for Vector2<N> {
    type Basis = [Vector2<N>; 1];
    fn orthonormal_basis(self) -> [Vector2<N>; 1] {
        [Vector2::new(-self.y, self.x)]
    }
    fn orthonormal_vector(self) -> Vector2<N> {
        Vector2::new(-self.y, self.x)
    }
}

impl<N: SimdRealCopy + SimdSign<N>> SimdBasis for Vector3<N> {
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

    fn orthonormal_vector(self) -> Vector3<N> {
        let sign = self.z.copy_sign_to(N::one());
        let a = -N::one() / (sign + self.z);
        let b = self.x * self.y * a;
        Vector3::new(b, sign + self.y * self.y * a, -self.y)
    }
}

impl<N> SimdVec for Vector2<N>
where
    N: Scalar + Copy + SimdComponent + ClosedMul,
    N::Element: Scalar,
{
    type Element = Vector2<N::Element>;

    fn horizontal_inf(&self) -> Self::Element {
        Vector2::new(self.x.min_component(), self.y.min_component())
    }

    fn horizontal_sup(&self) -> Self::Element {
        Vector2::new(self.x.max_component(), self.y.max_component())
    }

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        self.component_mul(rhs)
    }
}

impl<N: SimdRealField> SimdCapMagnitude<N> for Vector2<N>
where
    N::Element: SimdRealField,
{
    fn simd_cap_magnitude(&self, limit: N) -> Self {
        self.simd_cap_magnitude(limit)
    }
}

impl<N: Scalar + Copy + SimdComponent + ClosedMul> SimdVec for Point2<N>
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

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        self.coords.component_mul(&rhs.coords).into()
    }
}

impl<N: Scalar + Copy + SimdComponent + ClosedMul> SimdVec for Vector3<N>
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

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        self.component_mul(rhs)
    }
}

impl<N: Scalar + Copy + SimdComponent + ClosedMul> SimdVec for Point3<N>
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

    fn component_mul_simd(&self, rhs: &Self) -> Self {
        self.coords.component_mul(&rhs.coords).into()
    }
}

impl<N: SimdRealCopy> SimdCrossMatrix for Vector3<N> {
    type CrossMat = Matrix3<N>;
    type CrossMatTr = Matrix3<N>;

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix(self) -> Self::CrossMat {
        Matrix3::new(
            N::zero(), -self.z, self.y,
            self.z, N::zero(), -self.x,
            -self.y, self.x, N::zero(),
        )
    }

    #[inline]
    #[rustfmt::skip]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        Matrix3::new(
            N::zero(), self.z, -self.y,
            -self.z, N::zero(), self.x,
            self.y, -self.x, N::zero(),
        )
    }
}

impl<N: SimdRealCopy> SimdCrossMatrix for Vector2<N> {
    type CrossMat = RowVector2<N>;
    type CrossMatTr = Vector2<N>;

    #[inline]
    fn gcross_matrix(self) -> Self::CrossMat {
        RowVector2::new(-self.y, self.x)
    }
    #[inline]
    fn gcross_matrix_tr(self) -> Self::CrossMatTr {
        Vector2::new(-self.y, self.x)
    }
}

#[cfg(feature = "linalg-nalgebra")]
impl SimdCrossMatrix for Real {
    type CrossMat = Matrix2<Real>;
    type CrossMatTr = Matrix2<Real>;

    #[inline]
    fn gcross_matrix(self) -> Matrix2<Real> {
        Matrix2::new(0.0, -self, self, 0.0)
    }

    #[inline]
    fn gcross_matrix_tr(self) -> Matrix2<Real> {
        Matrix2::new(0.0, self, -self, 0.0)
    }
}

#[cfg(feature = "linalg-nalgebra")]
impl SimdCrossMatrix for SimdReal {
    type CrossMat = Matrix2<SimdReal>;
    type CrossMatTr = Matrix2<SimdReal>;

    #[inline]
    fn gcross_matrix(self) -> Matrix2<SimdReal> {
        Matrix2::new(SimdReal::zero(), -self, self, SimdReal::zero())
    }

    #[inline]
    fn gcross_matrix_tr(self) -> Matrix2<SimdReal> {
        Matrix2::new(SimdReal::zero(), self, -self, SimdReal::zero())
    }
}

impl SimdCross<Vector3<Real>> for Vector3<Real> {
    type Result = Self;

    fn gcross(&self, rhs: Vector3<Real>) -> Self::Result {
        self.cross(rhs)
    }
    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        self.cross(rhs)
    }
}

impl SimdCross<Vector2<Real>> for Vector2<Real> {
    type Result = Real;

    fn gcross(&self, rhs: Vector2<Real>) -> Self::Result {
        self.x * rhs.y - self.y * rhs.x
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl SimdCross<Vector2<Real>> for Real {
    type Result = Vector2<Real>;

    fn gcross(&self, rhs: Vector2<Real>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl<N: SimdRealCopy> SimdDot<Vector3<N>> for Vector3<N> {
    type Result = N;

    fn gdot(&self, rhs: Vector3<N>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl<N: SimdRealCopy> SimdDot<Vector2<N>> for Vector2<N> {
    type Result = N;

    fn gdot(&self, rhs: Vector2<N>) -> Self::Result {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl<N: SimdRealCopy> SimdDot<Vector1<N>> for N {
    type Result = N;

    fn gdot(&self, rhs: Vector1<N>) -> Self::Result {
        *self * rhs.x
    }
}

impl<N: SimdRealCopy> SimdDot<N> for N {
    type Result = N;

    fn gdot(&self, rhs: N) -> Self::Result {
        *self * rhs
    }
}

impl<N: SimdRealCopy> SimdDot<N> for Vector1<N> {
    type Result = N;

    fn gdot(&self, rhs: N) -> Self::Result {
        self.x * rhs
    }
}

impl SimdCross<Vector3<SimdReal>> for Vector3<SimdReal> {
    type Result = Vector3<SimdReal>;

    fn gcross(&self, rhs: Self) -> Self::Result {
        self.cross(&rhs)
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        self.cross(rhs)
    }
}

impl SimdCross<Vector2<SimdReal>> for SimdReal {
    type Result = Vector2<SimdReal>;

    fn gcross(&self, rhs: Vector2<SimdReal>) -> Self::Result {
        Vector2::new(-rhs.y * *self, rhs.x * *self)
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, rhs: &Self) -> Self {
        unreachable!()
    }
}

impl SimdCross<Vector2<SimdReal>> for Vector2<SimdReal> {
    type Result = SimdReal;

    fn gcross(&self, rhs: Self) -> Self::Result {
        let yx = Vector2::new(rhs.y, rhs.x);
        let prod = self.component_mul(&yx);
        prod.x - prod.y
    }

    #[cfg(feature = "dim3")]
    fn cross_(&self, _: &Self) -> Self {
        unreachable!()
    }
}

impl<N: SimdRealCopy> SimdQuat<N> for UnitComplex<N> {
    type Result = Matrix1<N>;

    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        let two: N = N::splat(2.0);
        Matrix1::new((self.im * rhs.im + self.re * rhs.re) * two)
    }
}

impl<N: SimdRealCopy> SimdQuat<N> for UnitQuaternion<N> {
    type Result = Matrix3<N>;

    fn diff_conj1_2(&self, rhs: &Self) -> Self::Result {
        let half = N::splat(0.5);
        let v1 = self.imag();
        let v2 = rhs.imag();
        let w1 = self.w;
        let w2 = rhs.w;

        // TODO: this can probably be optimized a lot by unrolling the ops.
        (v1 * v2.transpose() + Matrix3::from_diagonal_element(w1 * w2)
            - (v1 * w2 + v2 * w1).cross_matrix()
            + v1.cross_matrix() * v2.cross_matrix())
            * half
    }
}

impl<N: SimdRealCopy> SimdAngularInertia<N> for N {
    type AngVector = N;
    type LinVector = Vector2<N>;
    type AngMatrix = N;

    fn inverse(&self) -> Self {
        simd_inv(*self)
    }

    fn transform_lin_vector(&self, pt: Vector2<N>) -> Vector2<N> {
        pt * *self
    }
    fn transform_vector(&self, pt: N) -> N {
        pt * *self
    }

    fn squared(&self) -> N {
        *self * *self
    }

    fn transform_matrix(&self, mat: &Self::AngMatrix) -> Self::AngMatrix {
        *mat * *self
    }

    fn into_matrix(self) -> Self::AngMatrix {
        self
    }
}
