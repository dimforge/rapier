use crate::dynamics::solver::SolverVel;
use crate::math::{AngVector, Vector, DIM};
use crate::utils::{SimdBasis, SimdDot, SimdRealCopy};

#[derive(Copy, Clone, Debug)]
pub(crate) struct TwoBodyConstraintTangentPart<N: SimdRealCopy> {
    pub gcross1: [AngVector<N>; DIM - 1],
    pub gcross2: [AngVector<N>; DIM - 1],
    pub rhs: [N; DIM - 1],
    pub rhs_wo_bias: [N; DIM - 1],
    #[cfg(feature = "dim2")]
    pub impulse: na::Vector1<N>,
    #[cfg(feature = "dim3")]
    pub impulse: na::Vector2<N>,
    #[cfg(feature = "dim2")]
    pub total_impulse: na::Vector1<N>,
    #[cfg(feature = "dim3")]
    pub total_impulse: na::Vector2<N>,
    #[cfg(feature = "dim2")]
    pub r: [N; 1],
    #[cfg(feature = "dim3")]
    pub r: [N; DIM],
}

impl<N: SimdRealCopy> TwoBodyConstraintTangentPart<N> {
    fn zero() -> Self {
        Self {
            gcross1: [na::zero(); DIM - 1],
            gcross2: [na::zero(); DIM - 1],
            rhs: [na::zero(); DIM - 1],
            rhs_wo_bias: [na::zero(); DIM - 1],
            impulse: na::zero(),
            total_impulse: na::zero(),
            #[cfg(feature = "dim2")]
            r: [na::zero(); 1],
            #[cfg(feature = "dim3")]
            r: [na::zero(); DIM],
        }
    }

    #[inline]
    pub fn apply_limit(
        &mut self,
        tangents1: [&Vector<N>; DIM - 1],
        im1: &Vector<N>,
        im2: &Vector<N>,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let new_impulse = self.impulse[0].simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            solver_vel1.linear += tangents1[0].component_mul(im1) * dlambda;
            solver_vel1.angular += self.gcross1[0] * dlambda;

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda;
            solver_vel2.angular += self.gcross2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let new_impulse = self.impulse;
            let new_impulse = {
                let _disable_fe_except =
                    crate::utils::DisableFloatingPointExceptionsFlags::
                    disable_floating_point_exceptions();
                new_impulse.simd_cap_magnitude(limit)
            };

            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel1.linear += tangents1[0].component_mul(im1) * dlambda[0]
                + tangents1[1].component_mul(im1) * dlambda[1];
            solver_vel1.angular += self.gcross1[0] * dlambda[0] + self.gcross1[1] * dlambda[1];

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda[0]
                + tangents1[1].component_mul(im2) * -dlambda[1];
            solver_vel2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }

    #[inline]
    pub fn solve(
        &mut self,
        tangents1: [&Vector<N>; DIM - 1],
        im1: &Vector<N>,
        im2: &Vector<N>,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let dvel = tangents1[0].dot(&solver_vel1.linear)
                + self.gcross1[0].gdot(solver_vel1.angular)
                - tangents1[0].dot(&solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let new_impulse = (self.impulse[0] - self.r[0] * dvel).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            solver_vel1.linear += tangents1[0].component_mul(im1) * dlambda;
            solver_vel1.angular += self.gcross1[0] * dlambda;

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda;
            solver_vel2.angular += self.gcross2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let dvel_0 = tangents1[0].dot(&solver_vel1.linear)
                + self.gcross1[0].gdot(solver_vel1.angular)
                - tangents1[0].dot(&solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let dvel_1 = tangents1[1].dot(&solver_vel1.linear)
                + self.gcross1[1].gdot(solver_vel1.angular)
                - tangents1[1].dot(&solver_vel2.linear)
                + self.gcross2[1].gdot(solver_vel2.angular)
                + self.rhs[1];

            let dvel_00 = dvel_0 * dvel_0;
            let dvel_11 = dvel_1 * dvel_1;
            let dvel_01 = dvel_0 * dvel_1;
            let inv_lhs = (dvel_00 + dvel_11)
                * crate::utils::simd_inv(
                    dvel_00 * self.r[0] + dvel_11 * self.r[1] + dvel_01 * self.r[2],
                );
            let delta_impulse = na::vector![inv_lhs * dvel_0, inv_lhs * dvel_1];
            let new_impulse = self.impulse - delta_impulse;
            let new_impulse = {
                let _disable_fe_except =
                        crate::utils::DisableFloatingPointExceptionsFlags::
                        disable_floating_point_exceptions();
                new_impulse.simd_cap_magnitude(limit)
            };

            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel1.linear += tangents1[0].component_mul(im1) * dlambda[0]
                + tangents1[1].component_mul(im1) * dlambda[1];
            solver_vel1.angular += self.gcross1[0] * dlambda[0] + self.gcross1[1] * dlambda[1];

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda[0]
                + tangents1[1].component_mul(im2) * -dlambda[1];
            solver_vel2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct TwoBodyConstraintNormalPart<N: SimdRealCopy> {
    pub gcross1: AngVector<N>,
    pub gcross2: AngVector<N>,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub impulse: N,
    pub total_impulse: N,
    pub r: N,
}

impl<N: SimdRealCopy> TwoBodyConstraintNormalPart<N> {
    fn zero() -> Self {
        Self {
            gcross1: na::zero(),
            gcross2: na::zero(),
            rhs: na::zero(),
            rhs_wo_bias: na::zero(),
            impulse: na::zero(),
            total_impulse: na::zero(),
            r: na::zero(),
        }
    }

    #[inline]
    pub fn solve(
        &mut self,
        cfm_factor: N,
        dir1: &Vector<N>,
        im1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        let dvel = dir1.dot(&solver_vel1.linear) + self.gcross1.gdot(solver_vel1.angular)
            - dir1.dot(&solver_vel2.linear)
            + self.gcross2.gdot(solver_vel2.angular)
            + self.rhs;
        let new_impulse = cfm_factor * (self.impulse - self.r * dvel).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel1.linear += dir1.component_mul(im1) * dlambda;
        solver_vel1.angular += self.gcross1 * dlambda;

        solver_vel2.linear += dir1.component_mul(im2) * -dlambda;
        solver_vel2.angular += self.gcross2 * dlambda;
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct TwoBodyConstraintElement<N: SimdRealCopy> {
    pub normal_part: TwoBodyConstraintNormalPart<N>,
    pub tangent_part: TwoBodyConstraintTangentPart<N>,
}

impl<N: SimdRealCopy> TwoBodyConstraintElement<N> {
    pub fn zero() -> Self {
        Self {
            normal_part: TwoBodyConstraintNormalPart::zero(),
            tangent_part: TwoBodyConstraintTangentPart::zero(),
        }
    }

    #[inline]
    pub fn solve_group(
        cfm_factor: N,
        elements: &mut [Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im1: &Vector<N>,
        im2: &Vector<N>,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
        solve_normal: bool,
        solve_friction: bool,
    ) where
        Vector<N>: SimdBasis,
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross(tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        // Solve penetration.
        if solve_normal {
            for element in elements.iter_mut() {
                element
                    .normal_part
                    .solve(cfm_factor, dir1, im1, im2, solver_vel1, solver_vel2);
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.apply_limit(tangents1, im1, im2, limit, solver_vel1, solver_vel2);
            }
        }

        // Solve friction.
        if solve_friction {
            for element in elements.iter_mut() {
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.solve(tangents1, im1, im2, limit, solver_vel1, solver_vel2);
            }
        }
    }
}
