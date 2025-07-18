use crate::dynamics::solver::{
    OneBodyConstraintElement, OneBodyConstraintNormalPart, OneBodyConstraintTangentPart,
};
use crate::math::{DIM, Real};
use na::DVector;
#[cfg(feature = "dim2")]
use na::SimdPartialOrd;

impl OneBodyConstraintTangentPart<Real> {
    #[inline]
    pub fn generic_warmstart(
        &mut self,
        j_id2: usize,
        jacobians: &DVector<Real>,
        ndofs2: usize,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
    ) {
        #[cfg(feature = "dim2")]
        {
            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                self.impulse[0],
                &jacobians.rows(j_id2 + ndofs2, ndofs2),
                1.0,
            );
        }

        #[cfg(feature = "dim3")]
        {
            let j_step = ndofs2 * 2;
            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                self.impulse[0],
                &jacobians.rows(j_id2 + ndofs2, ndofs2),
                1.0,
            );
            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                self.impulse[1],
                &jacobians.rows(j_id2 + j_step + ndofs2, ndofs2),
                1.0,
            );
        }
    }

    #[inline]
    pub fn generic_solve(
        &mut self,
        j_id2: usize,
        jacobians: &DVector<Real>,
        ndofs2: usize,
        limit: Real,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
    ) {
        #[cfg(feature = "dim2")]
        {
            let dvel_0 = jacobians
                .rows(j_id2, ndofs2)
                .dot(&solver_vels.rows(solver_vel2, ndofs2))
                + self.rhs[0];

            let new_impulse = (self.impulse[0] - self.r[0] * dvel_0).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                dlambda,
                &jacobians.rows(j_id2 + ndofs2, ndofs2),
                1.0,
            );
        }

        #[cfg(feature = "dim3")]
        {
            let j_step = ndofs2 * 2;
            let dvel_0 = jacobians
                .rows(j_id2, ndofs2)
                .dot(&solver_vels.rows(solver_vel2, ndofs2))
                + self.rhs[0];
            let dvel_1 = jacobians
                .rows(j_id2 + j_step, ndofs2)
                .dot(&solver_vels.rows(solver_vel2, ndofs2))
                + self.rhs[1];

            let new_impulse = na::Vector2::new(
                self.impulse[0] - self.r[0] * dvel_0,
                self.impulse[1] - self.r[1] * dvel_1,
            );
            let new_impulse = new_impulse.cap_magnitude(limit);

            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                dlambda[0],
                &jacobians.rows(j_id2 + ndofs2, ndofs2),
                1.0,
            );
            solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
                dlambda[1],
                &jacobians.rows(j_id2 + j_step + ndofs2, ndofs2),
                1.0,
            );
        }
    }
}

impl OneBodyConstraintNormalPart<Real> {
    #[inline]
    pub fn generic_warmstart(
        &mut self,
        j_id2: usize,
        jacobians: &DVector<Real>,
        ndofs2: usize,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
    ) {
        solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
            self.impulse,
            &jacobians.rows(j_id2 + ndofs2, ndofs2),
            1.0,
        );
    }

    #[inline]
    pub fn generic_solve(
        &mut self,
        cfm_factor: Real,
        j_id2: usize,
        jacobians: &DVector<Real>,
        ndofs2: usize,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
    ) {
        let dvel = jacobians
            .rows(j_id2, ndofs2)
            .dot(&solver_vels.rows(solver_vel2, ndofs2))
            + self.rhs;

        let new_impulse = cfm_factor * (self.impulse - self.r * dvel).max(0.0);
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        solver_vels.rows_mut(solver_vel2, ndofs2).axpy(
            dlambda,
            &jacobians.rows(j_id2 + ndofs2, ndofs2),
            1.0,
        );
    }
}

impl OneBodyConstraintElement<Real> {
    #[inline]
    pub fn generic_warmstart_group(
        elements: &mut [Self],
        jacobians: &DVector<Real>,
        ndofs2: usize,
        // Jacobian index of the first constraint.
        j_id: usize,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
    ) {
        let j_step = ndofs2 * 2 * DIM;

        // Solve penetration.
        let mut nrm_j_id = j_id;

        for element in elements.iter_mut() {
            element.normal_part.generic_warmstart(
                nrm_j_id,
                jacobians,
                ndofs2,
                solver_vel2,
                solver_vels,
            );
            nrm_j_id += j_step;
        }

        // Solve friction.
        let mut tng_j_id = j_id + ndofs2 * 2;

        for element in elements.iter_mut() {
            let part = &mut element.tangent_part;
            part.generic_warmstart(tng_j_id, jacobians, ndofs2, solver_vel2, solver_vels);
            tng_j_id += j_step;
        }
    }

    #[inline]
    pub fn generic_solve_group(
        cfm_factor: Real,
        elements: &mut [Self],
        jacobians: &DVector<Real>,
        limit: Real,
        ndofs2: usize,
        // Jacobian index of the first constraint.
        j_id: usize,
        solver_vel2: usize,
        solver_vels: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let j_step = ndofs2 * 2 * DIM;

        // Solve penetration.
        if solve_restitution {
            let mut nrm_j_id = j_id;

            for element in elements.iter_mut() {
                element.normal_part.generic_solve(
                    cfm_factor,
                    nrm_j_id,
                    jacobians,
                    ndofs2,
                    solver_vel2,
                    solver_vels,
                );
                nrm_j_id += j_step;
            }
        }

        // Solve friction.
        if solve_friction {
            let mut tng_j_id = j_id + ndofs2 * 2;

            for element in elements.iter_mut() {
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.generic_solve(tng_j_id, jacobians, ndofs2, limit, solver_vel2, solver_vels);
                tng_j_id += j_step;
            }
        }
    }
}
