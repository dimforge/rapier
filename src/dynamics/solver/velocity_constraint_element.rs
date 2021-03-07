use super::DeltaVel;
use crate::math::{AngVector, Vector, DIM};
use crate::utils::{WBasis, WDot};
use na::SimdRealField;

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraintTangentPart<N: SimdRealField> {
    pub gcross1: [AngVector<N>; DIM - 1],
    pub gcross2: [AngVector<N>; DIM - 1],
    pub rhs: [N; DIM - 1],
    #[cfg(feature = "dim2")]
    pub impulse: [N; DIM - 1],
    #[cfg(feature = "dim3")]
    pub impulse: na::Vector2<N>,
    pub r: [N; DIM - 1],
}

impl<N: SimdRealField> VelocityConstraintTangentPart<N> {
    #[cfg(not(target_arch = "wasm32"))]
    fn zero() -> Self {
        Self {
            gcross1: [na::zero(); DIM - 1],
            gcross2: [na::zero(); DIM - 1],
            rhs: [na::zero(); DIM - 1],
            #[cfg(feature = "dim2")]
            impulse: [na::zero(); DIM - 1],
            #[cfg(feature = "dim3")]
            impulse: na::zero(),
            r: [na::zero(); DIM - 1],
        }
    }

    #[inline]
    pub fn warmstart(
        &self,
        tangents1: [&Vector<N>; DIM - 1],
        im1: N,
        im2: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        AngVector<N>: WDot<AngVector<N>, Result = N>,
        N::Element: SimdRealField,
    {
        for j in 0..DIM - 1 {
            mj_lambda1.linear += tangents1[j] * (im1 * self.impulse[j]);
            mj_lambda1.angular += self.gcross1[j] * self.impulse[j];

            mj_lambda2.linear += tangents1[j] * (-im2 * self.impulse[j]);
            mj_lambda2.angular += self.gcross2[j] * self.impulse[j];
        }
    }

    #[inline]
    pub fn solve(
        &mut self,
        tangents1: [&Vector<N>; DIM - 1],
        im1: N,
        im2: N,
        limit: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        AngVector<N>: WDot<AngVector<N>, Result = N>,
        N::Element: SimdRealField,
    {
        #[cfg(feature = "dim2")]
        {
            let dimpulse = tangents1[0].dot(&mj_lambda1.linear)
                + self.gcross1[0].gdot(mj_lambda1.angular)
                - tangents1[0].dot(&mj_lambda2.linear)
                + self.gcross2[0].gdot(mj_lambda2.angular)
                + self.rhs[0];
            let new_impulse = (self.impulse[0] - self.r[0] * dimpulse).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            mj_lambda1.linear += tangents1[0] * (im1 * dlambda);
            mj_lambda1.angular += self.gcross1[0] * dlambda;

            mj_lambda2.linear += tangents1[0] * (-im2 * dlambda);
            mj_lambda2.angular += self.gcross2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let dimpulse_0 = tangents1[0].dot(&mj_lambda1.linear)
                + self.gcross1[0].gdot(mj_lambda1.angular)
                - tangents1[0].dot(&mj_lambda2.linear)
                + self.gcross2[0].gdot(mj_lambda2.angular)
                + self.rhs[0];
            let dimpulse_1 = tangents1[1].dot(&mj_lambda1.linear)
                + self.gcross1[1].gdot(mj_lambda1.angular)
                - tangents1[1].dot(&mj_lambda2.linear)
                + self.gcross2[1].gdot(mj_lambda2.angular)
                + self.rhs[1];

            let new_impulse = na::Vector2::new(
                self.impulse[0] - self.r[0] * dimpulse_0,
                self.impulse[1] - self.r[1] * dimpulse_1,
            );
            let new_impulse = new_impulse.simd_cap_magnitude(limit);
            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            mj_lambda1.linear +=
                tangents1[0] * (im1 * dlambda[0]) + tangents1[1] * (im1 * dlambda[1]);
            mj_lambda1.angular += self.gcross1[0] * dlambda[0] + self.gcross1[1] * dlambda[1];

            mj_lambda2.linear +=
                tangents1[0] * (-im2 * dlambda[0]) + tangents1[1] * (-im2 * dlambda[1]);
            mj_lambda2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraintNormalPart<N: SimdRealField> {
    pub gcross1: AngVector<N>,
    pub gcross2: AngVector<N>,
    pub rhs: N,
    pub impulse: N,
    pub r: N,
}

impl<N: SimdRealField> VelocityConstraintNormalPart<N> {
    #[cfg(not(target_arch = "wasm32"))]
    fn zero() -> Self {
        Self {
            gcross1: na::zero(),
            gcross2: na::zero(),
            rhs: na::zero(),
            impulse: na::zero(),
            r: na::zero(),
        }
    }

    #[inline]
    pub fn warmstart(
        &self,
        dir1: &Vector<N>,
        im1: N,
        im2: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        AngVector<N>: WDot<AngVector<N>, Result = N>,
    {
        mj_lambda1.linear += dir1 * (im1 * self.impulse);
        mj_lambda1.angular += self.gcross1 * self.impulse;

        mj_lambda2.linear += dir1 * (-im2 * self.impulse);
        mj_lambda2.angular += self.gcross2 * self.impulse;
    }

    #[inline]
    pub fn solve(
        &mut self,
        dir1: &Vector<N>,
        im1: N,
        im2: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        AngVector<N>: WDot<AngVector<N>, Result = N>,
    {
        let dimpulse = dir1.dot(&mj_lambda1.linear) + self.gcross1.gdot(mj_lambda1.angular)
            - dir1.dot(&mj_lambda2.linear)
            + self.gcross2.gdot(mj_lambda2.angular)
            + self.rhs;
        let new_impulse = (self.impulse - self.r * dimpulse).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        mj_lambda1.linear += dir1 * (im1 * dlambda);
        mj_lambda1.angular += self.gcross1 * dlambda;

        mj_lambda2.linear += dir1 * (-im2 * dlambda);
        mj_lambda2.angular += self.gcross2 * dlambda;
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraintElement<N: SimdRealField> {
    pub normal_part: VelocityConstraintNormalPart<N>,
    pub tangent_part: VelocityConstraintTangentPart<N>,
}

impl<N: SimdRealField> VelocityConstraintElement<N> {
    #[cfg(not(target_arch = "wasm32"))]
    pub fn zero() -> Self {
        Self {
            normal_part: VelocityConstraintNormalPart::zero(),
            tangent_part: VelocityConstraintTangentPart::zero(),
        }
    }

    #[inline]
    pub fn warmstart_group(
        elements: &[Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im1: N,
        im2: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        Vector<N>: WBasis,
        AngVector<N>: WDot<AngVector<N>, Result = N>,
        N::Element: SimdRealField,
    {
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross(&tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        for element in elements {
            element
                .tangent_part
                .warmstart(tangents1, im1, im2, mj_lambda1, mj_lambda2);
            element
                .normal_part
                .warmstart(dir1, im1, im2, mj_lambda1, mj_lambda2);
        }
    }

    #[inline]
    pub fn solve_group(
        elements: &mut [Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im1: N,
        im2: N,
        limit: N,
        mj_lambda1: &mut DeltaVel<N>,
        mj_lambda2: &mut DeltaVel<N>,
    ) where
        Vector<N>: WBasis,
        AngVector<N>: WDot<AngVector<N>, Result = N>,
        N::Element: SimdRealField,
    {
        // Solve friction.
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross(&tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        for element in elements.iter_mut() {
            let limit = limit * element.normal_part.impulse;
            let part = &mut element.tangent_part;
            part.solve(tangents1, im1, im2, limit, mj_lambda1, mj_lambda2);
        }

        // Solve penetration.
        for element in elements.iter_mut() {
            element
                .normal_part
                .solve(&dir1, im1, im2, mj_lambda1, mj_lambda2);
        }
    }
}
