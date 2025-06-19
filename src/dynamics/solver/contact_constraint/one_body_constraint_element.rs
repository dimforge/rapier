use crate::dynamics::integration_parameters::BLOCK_SOLVER_ENABLED;
use crate::dynamics::solver::SolverVel;
use crate::dynamics::solver::contact_constraint::TwoBodyConstraintNormalPart;
use crate::math::{AngVector, DIM, TangentImpulse, Vector};
use crate::utils::{SimdBasis, SimdDot, SimdRealCopy};
use na::Vector2;

#[derive(Copy, Clone, Debug)]
pub(crate) struct OneBodyConstraintTangentPart<N: SimdRealCopy> {
    pub gcross2: [AngVector<N>; DIM - 1],
    pub rhs: [N; DIM - 1],
    pub rhs_wo_bias: [N; DIM - 1],
    pub impulse: TangentImpulse<N>,
    pub impulse_accumulator: TangentImpulse<N>,
    #[cfg(feature = "dim2")]
    pub r: [N; 1],
    #[cfg(feature = "dim3")]
    pub r: [N; DIM],
}

impl<N: SimdRealCopy> OneBodyConstraintTangentPart<N> {
    fn zero() -> Self {
        Self {
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
        im2: &Vector<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        #[cfg(feature = "dim2")]
        {
            solver_vel2.linear += tangents1[0].component_mul(im2) * -self.impulse[0];
            solver_vel2.angular += self.gcross2[0] * self.impulse[0];
        }

        #[cfg(feature = "dim3")]
        {
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
        im2: &Vector<N>,
        limit: N,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let dvel = -tangents1[0].dot(&solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let new_impulse = (self.impulse[0] - self.r[0] * dvel).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda;
            solver_vel2.angular += self.gcross2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let dvel_0 = -tangents1[0].dot(&solver_vel2.linear)
                + self.gcross2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let dvel_1 = -tangents1[1].dot(&solver_vel2.linear)
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

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda[0]
                + tangents1[1].component_mul(im2) * -dlambda[1];
            solver_vel2.angular += self.gcross2[0] * dlambda[0] + self.gcross2[1] * dlambda[1];
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct OneBodyConstraintNormalPart<N: SimdRealCopy> {
    pub gcross2: AngVector<N>,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub impulse: N,
    pub impulse_accumulator: N,
    pub r: N,
    pub r_mat_elts: [N; 2],
}

impl<N: SimdRealCopy> OneBodyConstraintNormalPart<N> {
    fn zero() -> Self {
        Self {
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
    pub fn warmstart(&mut self, dir1: &Vector<N>, im2: &Vector<N>, solver_vel2: &mut SolverVel<N>) {
        solver_vel2.linear += dir1.component_mul(im2) * -self.impulse;
        solver_vel2.angular += self.gcross2 * self.impulse;
    }

    #[inline]
    pub fn solve(
        &mut self,
        cfm_factor: N,
        dir1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        let dvel =
            -dir1.dot(&solver_vel2.linear) + self.gcross2.gdot(solver_vel2.angular) + self.rhs;
        let new_impulse = cfm_factor * (self.impulse - self.r * dvel).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel2.linear += dir1.component_mul(im2) * -dlambda;
        solver_vel2.angular += self.gcross2 * dlambda;
    }

    #[inline]
    pub fn solve_pair(
        constraint_a: &mut Self,
        constraint_b: &mut Self,
        cfm_factor: N,
        dir1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        AngVector<N>: SimdDot<AngVector<N>, Result = N>,
    {
        let dvel_a = -dir1.dot(&solver_vel2.linear)
            + constraint_a.gcross2.gdot(solver_vel2.angular)
            + constraint_a.rhs;
        let dvel_b = -dir1.dot(&solver_vel2.linear)
            + constraint_b.gcross2.gdot(solver_vel2.angular)
            + constraint_b.rhs;

        let prev_impulse = Vector2::new(constraint_a.impulse, constraint_b.impulse);
        let new_impulse = TwoBodyConstraintNormalPart::solve_mlcp_two_constraints(
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

        solver_vel2.linear += dir1.component_mul(im2) * (-dlambda.x - dlambda.y);
        solver_vel2.angular += constraint_a.gcross2 * dlambda.x + constraint_b.gcross2 * dlambda.y;
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
    pub fn warmstart_group(
        elements: &mut [Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im2: &Vector<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        #[cfg(feature = "dim3")]
        let tangents1 = [tangent1, &dir1.cross(tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&dir1.orthonormal_vector()];

        for element in elements.iter_mut() {
            element.normal_part.warmstart(dir1, im2, solver_vel2);
            element.tangent_part.warmstart(tangents1, im2, solver_vel2);
        }
    }

    #[inline]
    pub fn solve_group(
        cfm_factor: N,
        elements: &mut [Self],
        dir1: &Vector<N>,
        #[cfg(feature = "dim3")] tangent1: &Vector<N>,
        im2: &Vector<N>,
        limit: N,
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
            if BLOCK_SOLVER_ENABLED {
                for elements in elements.chunks_exact_mut(2) {
                    let [element_a, element_b] = elements else {
                        unreachable!()
                    };

                    OneBodyConstraintNormalPart::solve_pair(
                        &mut element_a.normal_part,
                        &mut element_b.normal_part,
                        cfm_factor,
                        dir1,
                        im2,
                        solver_vel2,
                    );
                }

                if elements.len() % 2 == 1 {
                    let element = elements.last_mut().unwrap();
                    element
                        .normal_part
                        .solve(cfm_factor, dir1, im2, solver_vel2);
                }
            } else {
                for element in elements.iter_mut() {
                    element
                        .normal_part
                        .solve(cfm_factor, dir1, im2, solver_vel2);
                }
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
