use crate::dynamics::solver::SolverVel;
use crate::math::*;
use crate::utils::{SimdBasis, SimdCapMagnitude, SimdCross, SimdDot, SimdRealCopy, SimdVec};

#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct OneBodyConstraintTangentPart<N: SimdRealCopy> {
    pub gcross2: [N::AngVector; DIM - 1],
    pub rhs: [N; DIM - 1],
    pub rhs_wo_bias: [N; DIM - 1],
    #[cfg(feature = "dim2")]
    pub impulse: N,
    #[cfg(feature = "dim3")]
    pub impulse: N::Vector2,
    #[cfg(feature = "dim2")]
    pub total_impulse: N,
    #[cfg(feature = "dim3")]
    pub total_impulse: N::Vector2,
    #[cfg(feature = "dim2")]
    pub r: [N; 1],
    #[cfg(feature = "dim3")]
    pub r: [N; DIM],
}

impl<N: SimdRealCopy> OneBodyConstraintTangentPart<N> {
    fn zero() -> Self {
        Self::default()
    }

    #[inline]
    pub fn apply_limit(
        &mut self,
        tangents1: [&N::Vector; DIM - 1],
        im2: &N::Vector,
        limit: N,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: SimdDot<N::AngVector, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let new_impulse = self.impulse.simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel2.linear += tangents1[0].component_mul_simd(im2) * -dlambda;
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

            solver_vel2.linear += tangents1[0].component_mul_simd(im2) * -dlambda[0]
                + tangents1[1].component_mul_simd(im2) * -dlambda[1];
            solver_vel2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }

    #[inline]
    pub fn solve(
        &mut self,
        tangents1: [&N::Vector; DIM - 1],
        im2: &N::Vector,
        limit: N,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: SimdDot<N::AngVector, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let dvel = -tangents1[0].gdot(solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let new_impulse = (self.impulse - self.r[0] * dvel).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel2.linear += tangents1[0].component_mul_simd(im2) * -dlambda;
            solver_vel2.angular += self.gcross2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let dvel_0 = -tangents1[0].gdot(solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let dvel_1 = -tangents1[1].gdot(solver_vel2.linear)
                + self.gcross2[1].gdot(solver_vel2.angular)
                + self.rhs[1];

            let dvel_00 = dvel_0 * dvel_0;
            let dvel_11 = dvel_1 * dvel_1;
            let dvel_01 = dvel_0 * dvel_1;
            let inv_lhs = (dvel_00 + dvel_11)
                * crate::utils::simd_inv(
                    dvel_00 * self.r[0] + dvel_11 * self.r[1] + dvel_01 * self.r[2],
                );
            let delta_impulse = N::Vector2::from([inv_lhs * dvel_0, inv_lhs * dvel_1]);
            let new_impulse = self.impulse - delta_impulse;
            let new_impulse = {
                let _disable_fe_except =
                    crate::utils::DisableFloatingPointExceptionsFlags::
                    disable_floating_point_exceptions();
                new_impulse.simd_cap_magnitude(limit)
            };
            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel2.linear += tangents1[0].component_mul_simd(im2) * -dlambda[0]
                + tangents1[1].component_mul_simd(im2) * -dlambda[1];
            solver_vel2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct OneBodyConstraintNormalPart<N: SimdRealCopy> {
    pub gcross2: N::AngVector,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub impulse: N,
    pub total_impulse: N,
    pub r: N,
}

impl<N: SimdRealCopy> OneBodyConstraintNormalPart<N> {
    fn zero() -> Self {
        Self::default()
    }

    #[inline]
    pub fn solve(
        &mut self,
        cfm_factor: N,
        dir1: &N::Vector,
        im2: &N::Vector,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: SimdDot<N::AngVector, Result = N>,
    {
        let dvel =
            -dir1.gdot(solver_vel2.linear) + self.gcross2.gdot(solver_vel2.angular) + self.rhs;
        let new_impulse = cfm_factor * (self.impulse - self.r * dvel).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel2.linear += dir1.component_mul_simd(im2) * -dlambda;
        solver_vel2.angular += self.gcross2 * dlambda;
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct OneBodyConstraintElement<N: SimdRealCopy> {
    pub normal_part: OneBodyConstraintNormalPart<N>,
    pub tangent_part: OneBodyConstraintTangentPart<N>,
}

impl<N: SimdRealCopy> OneBodyConstraintElement<N> {
    pub fn zero() -> Self {
        Self {
            normal_part: OneBodyConstraintNormalPart::zero(),
            tangent_part: OneBodyConstraintTangentPart::zero(),
        }
    }

    #[inline]
    pub fn solve_group(
        cfm_factor: N,
        elements: &mut [Self],
        dir1: &N::Vector,
        #[cfg(feature = "dim3")] tangent1: &N::Vector,
        im2: &N::Vector,
        limit: N,
        solver_vel2: &mut SolverVel<N>,
        solve_normal: bool,
        solve_friction: bool,
    ) where
        N::Vector: SimdBasis,
        N::AngVector: SimdDot<N::AngVector, Result = N>,
    {
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross_(tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        // Solve penetration.
        if solve_normal {
            for element in elements.iter_mut() {
                element
                    .normal_part
                    .solve(cfm_factor, dir1, im2, solver_vel2);
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.apply_limit(tangents1, im2, limit, solver_vel2);
            }
        }

        // Solve friction.
        if solve_friction {
            for element in elements.iter_mut() {
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.solve(tangents1, im2, limit, solver_vel2);
            }
        }
    }
}
