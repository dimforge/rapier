use crate::dynamics::integration_parameters::BLOCK_SOLVER_ENABLED;
use crate::dynamics::solver::SolverVel;
use crate::math::{AngVector, DIM, TangentImpulse, Vector};
use crate::utils::{SimdBasis, SimdDot, SimdRealCopy};
use na::Vector2;
use simba::simd::SimdValue;

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
    pub impulse_accumulator: na::Vector1<N>,
    #[cfg(feature = "dim3")]
    pub impulse_accumulator: na::Vector2<N>,
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
            impulse_accumulator: na::zero(),
            #[cfg(feature = "dim2")]
            r: [na::zero(); 1],
            #[cfg(feature = "dim3")]
            r: [na::zero(); DIM],
        }
    }

    /// Total impulse applied across all the solver substeps.
    #[inline]
    pub fn total_impulse(&self) -> TangentImpulse<N> {
        self.impulse_accumulator + self.impulse
    }

    #[inline]
    pub fn warmstart(
        &mut self,
        tangents1: [&Vector<N>; DIM - 1],
        im1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            solver_vel1.linear += tangents1[0].component_mul(im1) * self.impulse[0];
            solver_vel1.angular += self.gcross1[0] * self.impulse[0];

            solver_vel2.linear += tangents1[0].component_mul(im2) * -self.impulse[0];
            solver_vel2.angular += self.gcross2[0] * self.impulse[0];
        }

        #[cfg(feature = "dim3")]
        {
            solver_vel1.linear += tangents1[0].component_mul(im1) * self.impulse[0]
                + tangents1[1].component_mul(im1) * self.impulse[1];
            solver_vel1.angular +=
                self.gcross1[0] * self.impulse[0] + self.gcross1[1] * self.impulse[1];

            solver_vel2.linear += tangents1[0].component_mul(im2) * -self.impulse[0]
                + tangents1[1].component_mul(im2) * -self.impulse[1];
            solver_vel2.angular +=
                self.gcross2[0] * self.impulse[0] + self.gcross2[1] * self.impulse[1];
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
    pub impulse_accumulator: N,
    pub r: N,
    // For coupled constraint pairs, even constraints store the
    // diagonal of the projected mass matrix. Odd constraints
    // store the off-diagonal element of the projected mass matrix,
    // as well as the off-diagonal element of the inverse projected mass matrix.
    pub r_mat_elts: [N; 2],
}

impl<N: SimdRealCopy> TwoBodyConstraintNormalPart<N> {
    fn zero() -> Self {
        Self {
            gcross1: na::zero(),
            gcross2: na::zero(),
            rhs: na::zero(),
            rhs_wo_bias: na::zero(),
            impulse: na::zero(),
            impulse_accumulator: na::zero(),
            r: na::zero(),
            r_mat_elts: [N::zero(); 2],
        }
    }

    /// Total impulse applied across all the solver substeps.
    #[inline]
    pub fn total_impulse(&self) -> N {
        self.impulse_accumulator + self.impulse
    }

    #[inline]
    pub fn warmstart(
        &mut self,
        dir1: &Vector<N>,
        im1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        solver_vel1.linear += dir1.component_mul(im1) * self.impulse;
        solver_vel1.angular += self.gcross1 * self.impulse;

        solver_vel2.linear += dir1.component_mul(im2) * -self.impulse;
        solver_vel2.angular += self.gcross2 * self.impulse;
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

    #[inline(always)]
    pub(crate) fn solve_mlcp_two_constraints(
        dvel: Vector2<N>,
        prev_impulse: Vector2<N>,
        r_a: N,
        r_b: N,
        [r_mat11, r_mat22]: [N; 2],
        [r_mat12, r_mat_inv12]: [N; 2],
        cfm_factor: N,
    ) -> Vector2<N> {
        let r_dvel = Vector2::new(
            r_mat11 * dvel.x + r_mat12 * dvel.y,
            r_mat12 * dvel.x + r_mat22 * dvel.y,
        );
        let new_impulse0 = prev_impulse - r_dvel;
        let new_impulse1 = Vector2::new(prev_impulse.x - r_a * dvel.x, N::zero());
        let new_impulse2 = Vector2::new(N::zero(), prev_impulse.y - r_b * dvel.y);
        let new_impulse3 = Vector2::new(N::zero(), N::zero());

        let keep0 = new_impulse0.x.simd_ge(N::zero()) & new_impulse0.y.simd_ge(N::zero());
        let keep1 = new_impulse1.x.simd_ge(N::zero())
            & (dvel.y + r_mat_inv12 * new_impulse1.x).simd_ge(N::zero());
        let keep2 = new_impulse2.y.simd_ge(N::zero())
            & (dvel.x + r_mat_inv12 * new_impulse2.y).simd_ge(N::zero());
        let keep3 = dvel.x.simd_ge(N::zero()) & dvel.y.simd_ge(N::zero());

        let selected3 = (new_impulse3 * cfm_factor).select(keep3, prev_impulse);
        let selected2 = (new_impulse2 * cfm_factor).select(keep2, selected3);
        let selected1 = (new_impulse1 * cfm_factor).select(keep1, selected2);
        (new_impulse0 * cfm_factor).select(keep0, selected1)
    }

    #[inline]
    pub fn solve_pair(
        constraint_a: &mut Self,
        constraint_b: &mut Self,
        cfm_factor: N,
        dir1: &Vector<N>,
        im1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        let dvel_lin = dir1.dot(&solver_vel1.linear) - dir1.dot(&solver_vel2.linear);
        let dvel_a = dvel_lin
            + constraint_a.gcross1.gdot(solver_vel1.angular)
            + constraint_a.gcross2.gdot(solver_vel2.angular)
            + constraint_a.rhs;
        let dvel_b = dvel_lin
            + constraint_b.gcross1.gdot(solver_vel1.angular)
            + constraint_b.gcross2.gdot(solver_vel2.angular)
            + constraint_b.rhs;

        let prev_impulse = Vector2::new(constraint_a.impulse, constraint_b.impulse);
        let new_impulse = Self::solve_mlcp_two_constraints(
            Vector2::new(dvel_a, dvel_b),
            prev_impulse,
            constraint_a.r,
            constraint_b.r,
            constraint_a.r_mat_elts,
            constraint_b.r_mat_elts,
            cfm_factor,
        );

        let dlambda = new_impulse - prev_impulse;

        constraint_a.impulse = new_impulse.x;
        constraint_b.impulse = new_impulse.y;

        solver_vel1.linear += dir1.component_mul(im1) * (dlambda.x + dlambda.y);
        solver_vel1.angular += constraint_a.gcross1 * dlambda.x + constraint_b.gcross1 * dlambda.y;
        solver_vel2.linear += dir1.component_mul(im2) * (-dlambda.x - dlambda.y);
        solver_vel2.angular += constraint_a.gcross2 * dlambda.x + constraint_b.gcross2 * dlambda.y;
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
    pub fn warmstart_group(
        elements: &mut [Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross(tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        for element in elements.iter_mut() {
            element
                .normal_part
                .warmstart(dir1, im1, im2, solver_vel1, solver_vel2);
            element
                .tangent_part
                .warmstart(tangents1, im1, im2, solver_vel1, solver_vel2);
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
        solve_restitution: bool,
        solve_friction: bool,
    ) where
        Vector<N>: SimdBasis,
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        if solve_restitution {
            if BLOCK_SOLVER_ENABLED {
                for elements in elements.chunks_exact_mut(2) {
                    let [element_a, element_b] = elements else {
                        unreachable!()
                    };

                    TwoBodyConstraintNormalPart::solve_pair(
                        &mut element_a.normal_part,
                        &mut element_b.normal_part,
                        cfm_factor,
                        dir1,
                        im1,
                        im2,
                        solver_vel1,
                        solver_vel2,
                    );
                }

                // There is one constraint left to solve if there isn’t an even number.
                if elements.len() % 2 == 1 {
                    let element = elements.last_mut().unwrap();
                    element
                        .normal_part
                        .solve(cfm_factor, dir1, im1, im2, solver_vel1, solver_vel2);
                }
            } else {
                for element in elements.iter_mut() {
                    element
                        .normal_part
                        .solve(cfm_factor, dir1, im1, im2, solver_vel1, solver_vel2);
                }
            }
        }

        if solve_friction {
            #[cfg(feature = "dim3")]
            let tangents1 = [tangent1, &dir1.cross(tangent1)];
            #[cfg(feature = "dim2")]
            let tangents1 = [&dir1.orthonormal_vector()];

            for element in elements.iter_mut() {
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.solve(tangents1, im1, im2, limit, solver_vel1, solver_vel2);
            }
        }
    }
}
