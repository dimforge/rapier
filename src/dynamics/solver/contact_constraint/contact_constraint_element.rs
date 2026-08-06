use crate::dynamics::solver::SolverVel;
use crate::math::{DIM, TangentImpulse};
#[cfg(feature = "dim3")]
use crate::utils::{AngularInertiaOps, CrossProduct};
use crate::utils::{ComponentMul, DotProduct, ScalarType};
#[cfg(feature = "block-solver")]
use na::Vector2;
#[cfg(feature = "block-solver")]
use simba::simd::SimdValue;

#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintTangentPart<N: ScalarType> {
    pub torque_dir1: [N::AngVector; DIM - 1],
    pub torque_dir2: [N::AngVector; DIM - 1],
    pub ii_torque_dir1: [N::AngVector; DIM - 1],
    pub ii_torque_dir2: [N::AngVector; DIM - 1],
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

impl<N: ScalarType> ContactConstraintTangentPart<N> {
    pub fn zero() -> Self {
        Self {
            torque_dir1: [Default::default(); DIM - 1],
            torque_dir2: [Default::default(); DIM - 1],
            ii_torque_dir1: [Default::default(); DIM - 1],
            ii_torque_dir2: [Default::default(); DIM - 1],
            rhs: [N::zero(); DIM - 1],
            rhs_wo_bias: [N::zero(); DIM - 1],
            impulse: na::zero(),
            impulse_accumulator: na::zero(),
            #[cfg(feature = "dim2")]
            r: [N::zero(); 1],
            #[cfg(feature = "dim3")]
            r: [N::zero(); DIM],
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
        tangents1: [&N::Vector; DIM - 1],
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            solver_vel1.linear += tangents1[0].component_mul(im1) * self.impulse[0];
            solver_vel1.angular += self.ii_torque_dir1[0] * self.impulse[0];

            solver_vel2.linear += tangents1[0].component_mul(im2) * -self.impulse[0];
            solver_vel2.angular += self.ii_torque_dir2[0] * self.impulse[0];
        }

        #[cfg(feature = "dim3")]
        {
            solver_vel1.linear += (*tangents1[0] * self.impulse[0]
                + *tangents1[1] * self.impulse[1])
                .component_mul(im1);
            solver_vel1.angular +=
                self.ii_torque_dir1[0] * self.impulse[0] + self.ii_torque_dir1[1] * self.impulse[1];

            solver_vel2.linear += (*tangents1[0] * -self.impulse[0]
                + *tangents1[1] * -self.impulse[1])
                .component_mul(im2);
            solver_vel2.angular +=
                self.ii_torque_dir2[0] * self.impulse[0] + self.ii_torque_dir2[1] * self.impulse[1];
        }
    }

    #[inline]
    pub fn solve(
        &mut self,
        tangents1: [&N::Vector; DIM - 1],
        im1: &N::Vector,
        im2: &N::Vector,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        #[cfg(feature = "dim2")]
        {
            let dvel = tangents1[0].gdot(solver_vel1.linear)
                + self.torque_dir1[0].gdot(solver_vel1.angular)
                - tangents1[0].gdot(solver_vel2.linear)
                + self.torque_dir2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let new_impulse = (self.impulse[0] - self.r[0] * dvel).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            solver_vel1.linear += tangents1[0].component_mul(im1) * dlambda;
            solver_vel1.angular += self.ii_torque_dir1[0] * dlambda;

            solver_vel2.linear += tangents1[0].component_mul(im2) * -dlambda;
            solver_vel2.angular += self.ii_torque_dir2[0] * dlambda;
        }

        #[cfg(feature = "dim3")]
        {
            let dvel_0 = tangents1[0].gdot(solver_vel1.linear)
                + self.torque_dir1[0].gdot(solver_vel1.angular)
                - tangents1[0].gdot(solver_vel2.linear)
                + self.torque_dir2[0].gdot(solver_vel2.angular)
                + self.rhs[0];
            let dvel_1 = tangents1[1].gdot(solver_vel1.linear)
                + self.torque_dir1[1].gdot(solver_vel1.angular)
                - tangents1[1].gdot(solver_vel2.linear)
                + self.torque_dir2[1].gdot(solver_vel2.angular)
                + self.rhs[1];

            // Exact coupled 2×2 central-friction solve: `Δλ = -K⁻¹·dvel` with
            // `K` the tangent effective-mass matrix. A 1D solve along `dvel` leaves an
            // orthogonal residual when `K` is anisotropic; iterating it rotates energy between
            // the tangents and pumps the friction-only mode of large stacks into ejection.
            let k11 = self.r[0];
            let k22 = self.r[1];
            let k12 = self.r[2] * N::splat(0.5);
            let inv_det = crate::utils::simd_inv(k11 * k22 - k12 * k12);
            let delta_impulse = na::vector![
                (k22 * dvel_0 - k12 * dvel_1) * inv_det,
                (k11 * dvel_1 - k12 * dvel_0) * inv_det
            ];
            let new_impulse = self.impulse - delta_impulse;
            let new_impulse = {
                let _disable_fe_except =
                        crate::utils::DisableFloatingPointExceptionsFlags::
                        disable_floating_point_exceptions();
                new_impulse.simd_cap_magnitude(limit)
            };

            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vel1.linear +=
                (*tangents1[0] * dlambda[0] + *tangents1[1] * dlambda[1]).component_mul(im1);
            solver_vel1.angular +=
                self.ii_torque_dir1[0] * dlambda[0] + self.ii_torque_dir1[1] * dlambda[1];

            solver_vel2.linear +=
                (*tangents1[0] * -dlambda[0] + *tangents1[1] * -dlambda[1]).component_mul(im2);
            solver_vel2.angular +=
                self.ii_torque_dir2[0] * dlambda[0] + self.ii_torque_dir2[1] * dlambda[1];
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintNormalPart<N: ScalarType> {
    pub torque_dir1: N::AngVector,
    pub torque_dir2: N::AngVector,
    pub ii_torque_dir1: N::AngVector,
    pub ii_torque_dir2: N::AngVector,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub impulse: N,
    pub impulse_accumulator: N,
    pub r: N,
    /// Per-point softness (see `ContactConstraintNormalPartSlim::cfm_factor`):
    /// separated (speculative) points are solved rigidly.
    pub cfm_factor: N,
    // For coupled constraint pairs, even constraints store the
    // diagonal of the projected mass matrix. Odd constraints
    // store the off-diagonal element of the projected mass matrix,
    // as well as the off-diagonal element of the inverse projected mass matrix.
    #[cfg(feature = "block-solver")]
    pub r_mat_elts: [N; 2],
}

impl<N: ScalarType> ContactConstraintNormalPart<N> {
    pub fn zero() -> Self {
        Self {
            torque_dir1: Default::default(),
            torque_dir2: Default::default(),
            ii_torque_dir1: Default::default(),
            ii_torque_dir2: Default::default(),
            rhs: N::zero(),
            rhs_wo_bias: N::zero(),
            impulse: N::zero(),
            impulse_accumulator: N::zero(),
            r: N::zero(),
            cfm_factor: N::zero(),
            #[cfg(feature = "block-solver")]
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
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        solver_vel1.linear += dir1.component_mul(im1) * self.impulse;
        solver_vel1.angular += self.ii_torque_dir1 * self.impulse;

        solver_vel2.linear += dir1.component_mul(im2) * -self.impulse;
        solver_vel2.angular += self.ii_torque_dir2 * self.impulse;
    }

    #[inline]
    pub fn solve(
        &mut self,
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let dvel = dir1.gdot(solver_vel1.linear) + self.torque_dir1.gdot(solver_vel1.angular)
            - dir1.gdot(solver_vel2.linear)
            + self.torque_dir2.gdot(solver_vel2.angular)
            + self.rhs;
        let new_impulse = self.cfm_factor * (self.impulse - self.r * dvel).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel1.linear += dir1.component_mul(im1) * dlambda;
        solver_vel1.angular += self.ii_torque_dir1 * dlambda;

        solver_vel2.linear += dir1.component_mul(im2) * -dlambda;
        solver_vel2.angular += self.ii_torque_dir2 * dlambda;
    }

    #[cfg(feature = "block-solver")]
    #[inline]
    pub(crate) fn solve_mlcp_two_constraints(
        dvel: Vector2<N>,
        degraded_dvel_a: N,
        prev_impulse: Vector2<N>,
        r_a: N,
        r_b: N,
        [k12, block_flag]: [N; 2],
        cfm_factor: Vector2<N>,
    ) -> Vector2<N> {
        let zero = N::zero();

        // Compliant 2x2 LCP (exact coupled generalization of the per-point soft step):
        // `0 <= lam PERP K' lam + b >= 0`, `k'_ii = k_ii / ms_i`, `b = dvel - K prev`
        let _disable_fe_except =
            crate::utils::DisableFloatingPointExceptionsFlags::disable_floating_point_exceptions();

        let k11 = crate::utils::simd_inv(r_a);
        let k22 = crate::utils::simd_inv(r_b);
        let b1 = dvel.x - k11 * prev_impulse.x - k12 * prev_impulse.y;
        let b2 = dvel.y - k12 * prev_impulse.x - k22 * prev_impulse.y;

        let kp11 = k11 * crate::utils::simd_inv(cfm_factor.x);
        let kp22 = k22 * crate::utils::simd_inv(cfm_factor.y);
        let det = kp11 * kp22 - k12 * k12;
        let inv_det = crate::utils::simd_inv(det);

        // Candidate 0: both points active: lam = -K'⁻¹ b.
        let new_impulse0 = Vector2::new(
            (k12 * b2 - kp22 * b1) * inv_det,
            (k12 * b1 - kp11 * b2) * inv_det,
        );
        // Candidate 1: only point a active (lam_b = 0): lam_a = -b1 / K'11.
        let cand1_x = -b1 * crate::utils::simd_inv(kp11);
        let new_impulse1 = Vector2::new(cand1_x, zero);
        // Candidate 2: only point b active (lam_a = 0): lam_b = -b2 / K'22.
        let cand2_y = -b2 * crate::utils::simd_inv(kp22);
        let new_impulse2 = Vector2::new(zero, cand2_y);
        // Candidate 3: both points inactive. (The complementarity checks of the
        // one-point candidates use the PHYSICAL velocity at the inactive point:
        // its diagonal compliance term vanishes with its zero impulse.)
        let new_impulse3 = Vector2::new(zero, zero);

        let keep0 = det.simd_gt(zero) & new_impulse0.x.simd_ge(zero) & new_impulse0.y.simd_ge(zero);
        let keep1 = cand1_x.simd_ge(zero) & (b2 + k12 * cand1_x).simd_ge(zero);
        let keep2 = cand2_y.simd_ge(zero) & (b1 + k12 * cand2_y).simd_ge(zero);
        let keep3 = b1.simd_ge(zero) & b2.simd_ge(zero);

        let selected3 = new_impulse3.select(keep3, prev_impulse);
        let selected2 = new_impulse2.select(keep2, selected3);
        let selected1 = new_impulse1.select(keep1, selected2);
        let block_result = new_impulse0.select(keep0, selected1);

        // Degraded lanes (`block_flag` = 0: manifold lacks both points, or the pair matrix
        // wasn't invertible at build time).
        // Use `degraded_dvel_a` rather than `dvel.x` so that this case matches the non-mlcp
        // solve exactly.
        // TODO: measure if using `degraded_dvel_a` instead of `dvel.x` has any performance
        //       impact.
        let degraded = Vector2::new(
            cfm_factor.x * (prev_impulse.x - r_a * degraded_dvel_a).simd_max(zero),
            zero,
        );
        let use_block = block_flag.simd_gt(N::splat(0.5));
        block_result.select(use_block, degraded)
    }

    #[cfg(feature = "block-solver")]
    #[inline]
    pub fn solve_pair(
        constraint_a: &mut Self,
        constraint_b: &mut Self,
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let dvel_lin1 = dir1.gdot(solver_vel1.linear);
        let dvel_lin2 = dir1.gdot(solver_vel2.linear);
        let dvel_lin = dvel_lin1 - dvel_lin2;
        let ang_a1 = constraint_a.torque_dir1.gdot(solver_vel1.angular);
        let ang_a2 = constraint_a.torque_dir2.gdot(solver_vel2.angular);
        let dvel_a = dvel_lin + ang_a1 + ang_a2 + constraint_a.rhs;
        let dvel_b = dvel_lin
            + constraint_b.torque_dir1.gdot(solver_vel1.angular)
            + constraint_b.torque_dir2.gdot(solver_vel2.angular)
            + constraint_b.rhs;
        // Same quantity as `dvel_a`, re-associated to match `Self::solve` bit for bit — only
        // the degraded lanes use it.
        let degraded_dvel_a = dvel_lin1 + ang_a1 - dvel_lin2 + ang_a2 + constraint_a.rhs;

        let prev_impulse = Vector2::new(constraint_a.impulse, constraint_b.impulse);
        let new_impulse = Self::solve_mlcp_two_constraints(
            Vector2::new(dvel_a, dvel_b),
            degraded_dvel_a,
            prev_impulse,
            constraint_a.r,
            constraint_b.r,
            constraint_a.r_mat_elts,
            Vector2::new(constraint_a.cfm_factor, constraint_b.cfm_factor),
        );

        let dlambda = new_impulse - prev_impulse;

        constraint_a.impulse = new_impulse.x;
        constraint_b.impulse = new_impulse.y;

        solver_vel1.linear += dir1.component_mul(im1) * (dlambda.x + dlambda.y);
        solver_vel1.angular +=
            constraint_a.ii_torque_dir1 * dlambda.x + constraint_b.ii_torque_dir1 * dlambda.y;
        solver_vel2.linear += dir1.component_mul(im2) * (-dlambda.x - dlambda.y);
        solver_vel2.angular +=
            constraint_a.ii_torque_dir2 * dlambda.x + constraint_b.ii_torque_dir2 * dlambda.y;
    }
}

/*
 * "Slim" variants for the twist-friction (Simplified) model.
 */
#[cfg(feature = "dim3")]
#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintNormalPartSlim<N: ScalarType> {
    /// Angular jacobian wrt the first body.
    pub torque_dir1: N::AngVector,
    /// Angular jacobian wrt the second body.
    pub torque_dir2: N::AngVector,
    /// Equals to `ii1 * torque_dir1`.
    pub ii_torque_dir1: N::AngVector,
    /// Equals to `ii2 * torque_dir2`.
    pub ii_torque_dir2: N::AngVector,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub impulse: N,
    pub impulse_accumulator: N,
    pub r: N,
    /// Per-point softness. Separated (speculative) points are solved RIGIDLY
    /// (`massScale = 1`, `impulseScale = 0`): perfectly inelastic touchdowns damp stack
    /// rocking; a constraint-wide cfm under-stops each touchdown and pumps tall stacks.
    pub cfm_factor: N,
    /// See [`ContactConstraintNormalPart::r_mat_elts`]: the 2×2 block-solve
    /// elements coupling this point with its pair partner.
    #[cfg(feature = "block-solver")]
    pub r_mat_elts: [N; 2],
}

#[cfg(feature = "dim3")]
impl<N: ScalarType> ContactConstraintNormalPartSlim<N>
where
    N::Vector: CrossProduct<N::Vector, Result = N::AngVector>,
{
    /// Total impulse applied across all the solver substeps.
    #[inline]
    pub fn total_impulse(&self) -> N {
        self.impulse_accumulator + self.impulse
    }

    #[inline]
    pub fn warmstart(
        &mut self,
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        solver_vel1.linear += dir1.component_mul(im1) * self.impulse;
        solver_vel1.angular += self.ii_torque_dir1 * self.impulse;

        solver_vel2.linear += dir1.component_mul(im2) * -self.impulse;
        solver_vel2.angular += self.ii_torque_dir2 * self.impulse;
    }

    #[inline]
    pub fn solve(
        &mut self,
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let dvel = dir1.gdot(solver_vel1.linear) + self.torque_dir1.gdot(solver_vel1.angular)
            - dir1.gdot(solver_vel2.linear)
            + self.torque_dir2.gdot(solver_vel2.angular)
            + self.rhs;
        let new_impulse = self.cfm_factor * (self.impulse - self.r * dvel).simd_max(N::zero());
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel1.linear += dir1.component_mul(im1) * dlambda;
        solver_vel1.angular += self.ii_torque_dir1 * dlambda;

        solver_vel2.linear += dir1.component_mul(im2) * -dlambda;
        solver_vel2.angular += self.ii_torque_dir2 * dlambda;
    }

    /// See [`ContactConstraintNormalPart::solve_pair`]: solves two coupled
    /// normal constraints as a 2×2 LCP block (jacobians recomputed from the
    /// lever arms, like [`Self::solve`]).
    #[cfg(feature = "block-solver")]
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn solve_pair(
        constraint_a: &mut Self,
        constraint_b: &mut Self,
        dir1: &N::Vector,
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let torque_dir1_a = constraint_a.torque_dir1;
        let torque_dir2_a = constraint_a.torque_dir2;
        let torque_dir1_b = constraint_b.torque_dir1;
        let torque_dir2_b = constraint_b.torque_dir2;
        let ii_torque_dir1_a = constraint_a.ii_torque_dir1;
        let ii_torque_dir2_a = constraint_a.ii_torque_dir2;
        let ii_torque_dir1_b = constraint_b.ii_torque_dir1;
        let ii_torque_dir2_b = constraint_b.ii_torque_dir2;

        let dvel_lin1 = dir1.gdot(solver_vel1.linear);
        let dvel_lin2 = dir1.gdot(solver_vel2.linear);
        let dvel_lin = dvel_lin1 - dvel_lin2;
        let ang_a1 = torque_dir1_a.gdot(solver_vel1.angular);
        let ang_a2 = torque_dir2_a.gdot(solver_vel2.angular);
        let dvel_a = dvel_lin + ang_a1 + ang_a2 + constraint_a.rhs;
        let dvel_b = dvel_lin
            + torque_dir1_b.gdot(solver_vel1.angular)
            + torque_dir2_b.gdot(solver_vel2.angular)
            + constraint_b.rhs;
        // Same quantity as `dvel_a`, re-associated to match `Self::solve` bit for bit — only
        // the degraded lanes use it.
        let degraded_dvel_a = dvel_lin1 + ang_a1 - dvel_lin2 + ang_a2 + constraint_a.rhs;

        let prev_impulse = Vector2::new(constraint_a.impulse, constraint_b.impulse);
        let new_impulse = ContactConstraintNormalPart::<N>::solve_mlcp_two_constraints(
            Vector2::new(dvel_a, dvel_b),
            degraded_dvel_a,
            prev_impulse,
            constraint_a.r,
            constraint_b.r,
            constraint_a.r_mat_elts,
            Vector2::new(constraint_a.cfm_factor, constraint_b.cfm_factor),
        );

        let dlambda = new_impulse - prev_impulse;

        constraint_a.impulse = new_impulse.x;
        constraint_b.impulse = new_impulse.y;

        solver_vel1.linear += dir1.component_mul(im1) * (dlambda.x + dlambda.y);
        solver_vel1.angular += ii_torque_dir1_a * dlambda.x + ii_torque_dir1_b * dlambda.y;
        solver_vel2.linear += dir1.component_mul(im2) * (-dlambda.x - dlambda.y);
        solver_vel2.angular += ii_torque_dir2_a * dlambda.x + ii_torque_dir2_b * dlambda.y;
    }
}

#[cfg(feature = "dim3")]
#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintTangentPartSlim<N: ScalarType> {
    /// World-space lever arm of the friction center wrt the first body's center of mass.
    pub dp1: N::Vector,
    /// World-space lever arm of the friction center wrt the second body's center of mass.
    pub dp2: N::Vector,
    /// Angular jacobians wrt the first body (one per tangent), cached at build time.
    pub torque_dir1: [N::AngVector; 2],
    /// Angular jacobians wrt the second body (one per tangent), cached at build time.
    pub torque_dir2: [N::AngVector; 2],
    /// `ii1 * torque_dir1[j]`, cached at build time.
    pub ii_torque_dir1: [N::AngVector; 2],
    /// `ii2 * torque_dir2[j]`, cached at build time.
    pub ii_torque_dir2: [N::AngVector; 2],
    pub rhs: [N; 2],
    pub rhs_wo_bias: [N; 2],
    pub impulse: na::Vector2<N>,
    pub impulse_accumulator: na::Vector2<N>,
    pub r: [N; 3],
}

#[cfg(feature = "dim3")]
impl<N: ScalarType> ContactConstraintTangentPartSlim<N>
where
    N::Vector: CrossProduct<N::Vector, Result = N::AngVector>,
{
    #[inline]
    pub fn warmstart(
        &mut self,
        tangents1: [&N::Vector; 2],
        im1: &N::Vector,
        im2: &N::Vector,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        let (ii_torque_dir1_0, ii_torque_dir1_1) = (self.ii_torque_dir1[0], self.ii_torque_dir1[1]);
        let (ii_torque_dir2_0, ii_torque_dir2_1) = (self.ii_torque_dir2[0], self.ii_torque_dir2[1]);

        solver_vel1.linear +=
            (*tangents1[0] * self.impulse[0] + *tangents1[1] * self.impulse[1]).component_mul(im1);
        solver_vel1.angular +=
            ii_torque_dir1_0 * self.impulse[0] + ii_torque_dir1_1 * self.impulse[1];

        solver_vel2.linear += (*tangents1[0] * -self.impulse[0] + *tangents1[1] * -self.impulse[1])
            .component_mul(im2);
        solver_vel2.angular +=
            ii_torque_dir2_0 * self.impulse[0] + ii_torque_dir2_1 * self.impulse[1];
    }

    #[inline]
    pub fn solve(
        &mut self,
        tangents1: [&N::Vector; 2],
        im1: &N::Vector,
        im2: &N::Vector,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let (torque_dir1_0, torque_dir1_1) = (self.torque_dir1[0], self.torque_dir1[1]);
        let (torque_dir2_0, torque_dir2_1) = (self.torque_dir2[0], self.torque_dir2[1]);
        let (ii_torque_dir1_0, ii_torque_dir1_1) = (self.ii_torque_dir1[0], self.ii_torque_dir1[1]);
        let (ii_torque_dir2_0, ii_torque_dir2_1) = (self.ii_torque_dir2[0], self.ii_torque_dir2[1]);

        let dvel_0 = tangents1[0].gdot(solver_vel1.linear)
            + torque_dir1_0.gdot(solver_vel1.angular)
            - tangents1[0].gdot(solver_vel2.linear)
            + torque_dir2_0.gdot(solver_vel2.angular)
            + self.rhs[0];
        let dvel_1 = tangents1[1].gdot(solver_vel1.linear)
            + torque_dir1_1.gdot(solver_vel1.angular)
            - tangents1[1].gdot(solver_vel2.linear)
            + torque_dir2_1.gdot(solver_vel2.angular)
            + self.rhs[1];

        // Exact coupled 2×2 solve — see `ContactConstraintTangentPart::solve`
        // for why the directional approximation must not be used here.
        let k11 = self.r[0];
        let k22 = self.r[1];
        let k12 = self.r[2] * N::splat(0.5);
        let inv_det = crate::utils::simd_inv(k11 * k22 - k12 * k12);
        let delta_impulse = na::vector![
            (k22 * dvel_0 - k12 * dvel_1) * inv_det,
            (k11 * dvel_1 - k12 * dvel_0) * inv_det
        ];
        let new_impulse = self.impulse - delta_impulse;
        let new_impulse = {
            let _disable_fe_except =
                    crate::utils::DisableFloatingPointExceptionsFlags::
                    disable_floating_point_exceptions();
            new_impulse.simd_cap_magnitude(limit)
        };

        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vel1.linear +=
            (*tangents1[0] * dlambda[0] + *tangents1[1] * dlambda[1]).component_mul(im1);
        solver_vel1.angular += ii_torque_dir1_0 * dlambda[0] + ii_torque_dir1_1 * dlambda[1];

        solver_vel2.linear +=
            (*tangents1[0] * -dlambda[0] + *tangents1[1] * -dlambda[1]).component_mul(im2);
        solver_vel2.angular += ii_torque_dir2_0 * dlambda[0] + ii_torque_dir2_1 * dlambda[1];
    }
}

#[cfg(feature = "dim3")]
#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintTwistPartSlim<N: ScalarType> {
    pub rhs: N,
    pub impulse: N,
    pub impulse_accumulator: N,
    pub r: N,
}

#[cfg(feature = "dim3")]
impl<N: ScalarType> ContactConstraintTwistPartSlim<N> {
    #[inline]
    pub fn warmstart(
        &mut self,
        twist_dir1: &N::AngVector,
        ii1: &N::AngInertia,
        ii2: &N::AngInertia,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        let ii_twist_dir1 = ii1.transform_vector(*twist_dir1);
        let ii_twist_dir2 = ii2.transform_vector(*twist_dir1);
        solver_vel1.angular += ii_twist_dir1 * self.impulse;
        solver_vel2.angular -= ii_twist_dir2 * self.impulse;
    }

    #[inline]
    pub fn solve(
        &mut self,
        twist_dir1: &N::AngVector,
        ii1: &N::AngInertia,
        ii2: &N::AngInertia,
        limit: N,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) where
        N::AngVector: DotProduct<N::AngVector, Result = N>,
    {
        let ii_twist_dir1 = ii1.transform_vector(*twist_dir1);
        let ii_twist_dir2 = ii2.transform_vector(*twist_dir1);

        let dvel = twist_dir1.gdot(solver_vel1.angular - solver_vel2.angular) + self.rhs;
        let new_impulse = (self.impulse - self.r * dvel).simd_clamp(-limit, limit);
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;
        solver_vel1.angular += ii_twist_dir1 * dlambda;
        solver_vel2.angular -= ii_twist_dir2 * dlambda;
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::dynamics::solver::SolverVel;
    use crate::math::{Real, Vector};

    #[cfg(all(feature = "dim2", feature = "block-solver"))]
    type AngVec = Real;
    #[cfg(all(feature = "dim3", feature = "block-solver"))]
    type AngVec = Vector;

    #[cfg(feature = "block-solver")]
    fn ang(x: Real) -> AngVec {
        #[cfg(feature = "dim2")]
        {
            x
        }
        #[cfg(feature = "dim3")]
        {
            Vector::new(x, 0.3 * x, -0.7 * x)
        }
    }

    #[cfg(feature = "block-solver")]
    fn normal_part_cfm(
        r: Real,
        rhs: Real,
        impulse: Real,
        torque1: Real,
        torque2: Real,
        r_mat_elts: [Real; 2],
        cfm_factor: Real,
    ) -> ContactConstraintNormalPart<Real> {
        ContactConstraintNormalPart {
            torque_dir1: ang(torque1),
            torque_dir2: ang(torque2),
            // The ii-premultiplied dirs only shape how deltas feed back into the
            // angular velocities; reuse the raw dirs for the test.
            ii_torque_dir1: ang(torque1),
            ii_torque_dir2: ang(torque2),
            rhs,
            rhs_wo_bias: rhs,
            impulse,
            impulse_accumulator: 0.0,
            r,
            cfm_factor,
            r_mat_elts,
        }
    }

    fn vels() -> (SolverVel<Real>, SolverVel<Real>) {
        let mut v1 = SolverVel::<Real>::zero();
        let mut v2 = SolverVel::<Real>::zero();
        #[cfg(feature = "dim2")]
        {
            v1.linear = Vector::new(0.3, -1.2);
            v1.angular = 0.7;
            v2.linear = Vector::new(-0.4, 0.9);
            v2.angular = -0.2;
        }
        #[cfg(feature = "dim3")]
        {
            v1.linear = Vector::new(0.3, -1.2, 0.5);
            v1.angular = Vector::new(0.7, 0.1, -0.3);
            v2.linear = Vector::new(-0.4, 0.9, -0.6);
            v2.angular = Vector::new(-0.2, 0.4, 0.6);
        }
        (v1, v2)
    }

    fn dir_im() -> (Vector, Vector, Vector) {
        #[cfg(feature = "dim2")]
        {
            (
                Vector::new(0.0, 1.0),
                Vector::new(0.5, 0.5),
                Vector::new(0.25, 0.25),
            )
        }
        #[cfg(feature = "dim3")]
        {
            (
                Vector::new(0.0, 1.0, 0.0),
                Vector::new(0.5, 0.5, 0.5),
                Vector::new(0.25, 0.25, 0.25),
            )
        }
    }

    /// The inactive-second-point encoding of the 2×2 block solve (`a.r_mat_elts = [0, 0]`,
    /// `b.{r, impulse, impulse_accumulator} = 0`) must behave exactly like the scalar solve of
    /// point `a` alone — this is what lets mixed-count lanes ride `solve_pair`.
    #[cfg(feature = "block-solver")]
    #[test]
    fn degraded_solve_pair_matches_scalar_solve() {
        let (dir1, im1, im2) = dir_im();

        // Sweep clamping regimes: rhs sign drives whether the scalar solve
        // clamps to zero; cfm != 1 exercises the soft-constraint scaling.
        for &(r_a, rhs_a, imp_a) in &[
            (0.8, -2.0, 0.5), // pushes: unclamped branch
            (0.8, 5.0, 0.1),  // separates: clamps to zero
            (0.0, -1.0, 0.0), // massless point: inert
            (1.5, -0.3, 2.0), // warm-started, mild correction
        ] {
            for &cfm in &[1.0, 0.7] {
                // Garbage-ish (finite) values for the inactive point B: its
                // zeroed masses must make them unobservable.
                let b_rhs = 42.0;
                let b_t1 = -3.0;
                let b_t2 = 9.0;

                let mut a_pair = normal_part_cfm(r_a, rhs_a, imp_a, 0.9, -0.4, [0.0, 0.0], cfm);
                let mut b_pair = normal_part_cfm(0.0, b_rhs, 0.0, b_t1, b_t2, [0.0, 0.0], cfm);
                let (mut v1_pair, mut v2_pair) = vels();
                ContactConstraintNormalPart::solve_pair(
                    &mut a_pair,
                    &mut b_pair,
                    &dir1,
                    &im1,
                    &im2,
                    &mut v1_pair,
                    &mut v2_pair,
                );

                let mut a_scalar = normal_part_cfm(r_a, rhs_a, imp_a, 0.9, -0.4, [0.0, 0.0], cfm);
                let (mut v1_scalar, mut v2_scalar) = vels();
                a_scalar.solve(&dir1, &im1, &im2, &mut v1_scalar, &mut v2_scalar);

                assert_eq!(
                    a_pair.impulse, a_scalar.impulse,
                    "impulse mismatch (r_a={r_a}, rhs_a={rhs_a}, cfm={cfm})"
                );
                assert_eq!(b_pair.impulse, 0.0);
                assert_eq!(v1_pair.linear, v1_scalar.linear);
                assert_eq!(v1_pair.angular, v1_scalar.angular);
                assert_eq!(v2_pair.linear, v2_scalar.linear);
                assert_eq!(v2_pair.angular, v2_scalar.angular);
            }
        }
    }

    /// An inactive tangent slot is `impulse = 0`, `limit = 0` and, in 3D, `r = [1, 1, 0]`
    /// (the coupled solve divides by an `r`-weighted form: a zero `r` gives `inf`, and capping
    /// an infinite vector to the zero limit is NaN). The solve must stay a finite no-op.
    #[test]
    fn inactive_tangent_slot_is_finite_noop() {
        let (_dir1, im1, im2) = dir_im();

        let mut part = ContactConstraintTangentPart::<Real>::zero();
        #[cfg(feature = "dim3")]
        {
            part.r = [1.0, 1.0, 0.0];
            part.rhs = [3.0, -2.0];
        }
        #[cfg(feature = "dim2")]
        {
            part.r = [0.0];
            part.rhs = [3.0];
        }

        #[cfg(feature = "dim2")]
        let tangents1_v = [Vector::new(1.0, 0.0)];
        #[cfg(feature = "dim3")]
        let tangents1_v = [Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&tangents1_v[0]];
        #[cfg(feature = "dim3")]
        let tangents1 = [&tangents1_v[0], &tangents1_v[1]];

        let (mut v1, mut v2) = vels();
        let (v1_before, v2_before) = vels();
        part.solve(tangents1, &im1, &im2, 0.0, &mut v1, &mut v2);

        assert_eq!(part.impulse, TangentImpulse::<Real>::zeros());
        assert_eq!(v1.linear, v1_before.linear);
        assert_eq!(v1.angular, v1_before.angular);
        assert_eq!(v2.linear, v2_before.linear);
        assert_eq!(v2.angular, v2_before.angular);
        assert!(part.impulse.iter().all(|x| x.is_finite()));
    }
}
