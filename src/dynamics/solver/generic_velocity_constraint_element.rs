use super::DeltaVel;
use crate::dynamics::solver::{
    VelocityConstraintElement, VelocityConstraintNormalPart, VelocityConstraintTangentPart,
};
use crate::math::{AngVector, Real, Vector, DIM};
use crate::utils::WDot;
use na::DVector;
#[cfg(feature = "dim2")]
use {crate::utils::WBasis, na::SimdPartialOrd};

pub(crate) enum GenericRhs {
    DeltaVel(DeltaVel<Real>),
    GenericId(usize),
}

// Offset between the jacobians of two consecutive constraints.
#[inline(always)]
fn j_step(ndofs1: usize, ndofs2: usize) -> usize {
    (ndofs1 + ndofs2) * 2
}

#[inline(always)]
fn j_id1(j_id: usize, _ndofs1: usize, _ndofs2: usize) -> usize {
    j_id
}

#[inline(always)]
fn j_id2(j_id: usize, ndofs1: usize, _ndofs2: usize) -> usize {
    j_id + ndofs1 * 2
}

#[inline(always)]
fn normal_j_id(j_id: usize, _ndofs1: usize, _ndofs2: usize) -> usize {
    j_id
}

#[inline(always)]
fn tangent_j_id(j_id: usize, ndofs1: usize, ndofs2: usize) -> usize {
    j_id + (ndofs1 + ndofs2) * 2
}

impl GenericRhs {
    #[inline(always)]
    fn dvel(
        &self,
        j_id: usize,
        ndofs: usize,
        jacobians: &DVector<Real>,
        dir: &Vector<Real>,
        gcross: &AngVector<Real>,
        mj_lambdas: &DVector<Real>,
    ) -> Real {
        match self {
            GenericRhs::DeltaVel(rhs) => dir.dot(&rhs.linear) + gcross.gdot(rhs.angular),
            GenericRhs::GenericId(mj_lambda) => {
                let j = jacobians.rows(j_id, ndofs);
                let rhs = mj_lambdas.rows(*mj_lambda, ndofs);
                j.dot(&rhs)
            }
        }
    }

    #[inline(always)]
    fn apply_impulse(
        &mut self,
        j_id: usize,
        ndofs: usize,
        impulse: Real,
        jacobians: &DVector<Real>,
        dir: &Vector<Real>,
        gcross: &AngVector<Real>,
        mj_lambdas: &mut DVector<Real>,
        inv_mass: &Vector<Real>,
    ) {
        match self {
            GenericRhs::DeltaVel(rhs) => {
                rhs.linear += dir.component_mul(inv_mass) * impulse;
                rhs.angular += gcross * impulse;
            }
            GenericRhs::GenericId(mj_lambda) => {
                let wj_id = j_id + ndofs;
                let wj = jacobians.rows(wj_id, ndofs);
                let mut rhs = mj_lambdas.rows_mut(*mj_lambda, ndofs);
                rhs.axpy(impulse, &wj, 1.0);
            }
        }
    }
}

impl VelocityConstraintTangentPart<Real> {
    #[inline]
    pub fn generic_solve(
        &mut self,
        j_id: usize,
        jacobians: &DVector<Real>,
        tangents1: [&Vector<Real>; DIM - 1],
        im1: &Vector<Real>,
        im2: &Vector<Real>,
        ndofs1: usize,
        ndofs2: usize,
        limit: Real,
        mj_lambda1: &mut GenericRhs,
        mj_lambda2: &mut GenericRhs,
        mj_lambdas: &mut DVector<Real>,
    ) {
        let j_id1 = j_id1(j_id, ndofs1, ndofs2);
        let j_id2 = j_id2(j_id, ndofs1, ndofs2);
        #[cfg(feature = "dim3")]
        let j_step = j_step(ndofs1, ndofs2);

        #[cfg(feature = "dim2")]
        {
            let dvel_0 = mj_lambda1.dvel(
                j_id1,
                ndofs1,
                jacobians,
                &tangents1[0],
                &self.gcross1[0],
                mj_lambdas,
            ) + mj_lambda2.dvel(
                j_id2,
                ndofs2,
                jacobians,
                &-tangents1[0],
                &self.gcross2[0],
                mj_lambdas,
            ) + self.rhs[0];

            let new_impulse = (self.impulse[0] - self.r[0] * dvel_0).simd_clamp(-limit, limit);
            let dlambda = new_impulse - self.impulse[0];
            self.impulse[0] = new_impulse;

            mj_lambda1.apply_impulse(
                j_id1,
                ndofs1,
                dlambda,
                jacobians,
                &tangents1[0],
                &self.gcross1[0],
                mj_lambdas,
                im1,
            );
            mj_lambda2.apply_impulse(
                j_id2,
                ndofs2,
                dlambda,
                jacobians,
                &-tangents1[0],
                &self.gcross2[0],
                mj_lambdas,
                im2,
            );
        }

        #[cfg(feature = "dim3")]
        {
            let dvel_0 = mj_lambda1.dvel(
                j_id1,
                ndofs1,
                jacobians,
                &tangents1[0],
                &self.gcross1[0],
                mj_lambdas,
            ) + mj_lambda2.dvel(
                j_id2,
                ndofs2,
                jacobians,
                &-tangents1[0],
                &self.gcross2[0],
                mj_lambdas,
            ) + self.rhs[0];
            let dvel_1 = mj_lambda1.dvel(
                j_id1 + j_step,
                ndofs1,
                jacobians,
                &tangents1[1],
                &self.gcross1[1],
                mj_lambdas,
            ) + mj_lambda2.dvel(
                j_id2 + j_step,
                ndofs2,
                jacobians,
                &-tangents1[1],
                &self.gcross2[1],
                mj_lambdas,
            ) + self.rhs[1];

            let new_impulse = na::Vector2::new(
                self.impulse[0] - self.r[0] * dvel_0,
                self.impulse[1] - self.r[1] * dvel_1,
            );
            let new_impulse = new_impulse.cap_magnitude(limit);

            let dlambda = new_impulse - self.impulse;
            self.impulse = new_impulse;

            mj_lambda1.apply_impulse(
                j_id1,
                ndofs1,
                dlambda[0],
                jacobians,
                &tangents1[0],
                &self.gcross1[0],
                mj_lambdas,
                im1,
            );
            mj_lambda1.apply_impulse(
                j_id1 + j_step,
                ndofs1,
                dlambda[1],
                jacobians,
                &tangents1[1],
                &self.gcross1[1],
                mj_lambdas,
                im1,
            );

            mj_lambda2.apply_impulse(
                j_id2,
                ndofs2,
                dlambda[0],
                jacobians,
                &-tangents1[0],
                &self.gcross2[0],
                mj_lambdas,
                im2,
            );
            mj_lambda2.apply_impulse(
                j_id2 + j_step,
                ndofs2,
                dlambda[1],
                jacobians,
                &-tangents1[1],
                &self.gcross2[1],
                mj_lambdas,
                im2,
            );
        }
    }
}

impl VelocityConstraintNormalPart<Real> {
    #[inline]
    pub fn generic_solve(
        &mut self,
        cfm_factor: Real,
        j_id: usize,
        jacobians: &DVector<Real>,
        dir1: &Vector<Real>,
        im1: &Vector<Real>,
        im2: &Vector<Real>,
        ndofs1: usize,
        ndofs2: usize,
        mj_lambda1: &mut GenericRhs,
        mj_lambda2: &mut GenericRhs,
        mj_lambdas: &mut DVector<Real>,
    ) {
        let j_id1 = j_id1(j_id, ndofs1, ndofs2);
        let j_id2 = j_id2(j_id, ndofs1, ndofs2);

        let dvel = mj_lambda1.dvel(j_id1, ndofs1, jacobians, &dir1, &self.gcross1, mj_lambdas)
            + mj_lambda2.dvel(j_id2, ndofs2, jacobians, &-dir1, &self.gcross2, mj_lambdas)
            + self.rhs;

        let new_impulse = cfm_factor * (self.impulse - self.r * dvel).max(0.0);
        let dlambda = new_impulse - self.impulse;
        self.impulse = new_impulse;

        mj_lambda1.apply_impulse(
            j_id1,
            ndofs1,
            dlambda,
            jacobians,
            &dir1,
            &self.gcross1,
            mj_lambdas,
            im1,
        );
        mj_lambda2.apply_impulse(
            j_id2,
            ndofs2,
            dlambda,
            jacobians,
            &-dir1,
            &self.gcross2,
            mj_lambdas,
            im2,
        );
    }
}

impl VelocityConstraintElement<Real> {
    #[inline]
    pub fn generic_solve_group(
        cfm_factor: Real,
        elements: &mut [Self],
        jacobians: &DVector<Real>,
        dir1: &Vector<Real>,
        #[cfg(feature = "dim3")] tangent1: &Vector<Real>,
        im1: &Vector<Real>,
        im2: &Vector<Real>,
        limit: Real,
        // ndofs is 0 for a non-multibody body, or a multibody with zero
        // degrees of freedom.
        ndofs1: usize,
        ndofs2: usize,
        // Jacobian index of the first constraint.
        j_id: usize,
        mj_lambda1: &mut GenericRhs,
        mj_lambda2: &mut GenericRhs,
        mj_lambdas: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let j_step = j_step(ndofs1, ndofs2) * DIM;

        // Solve penetration.
        if solve_restitution {
            let mut nrm_j_id = normal_j_id(j_id, ndofs1, ndofs2);

            for element in elements.iter_mut() {
                element.normal_part.generic_solve(
                    cfm_factor, nrm_j_id, jacobians, &dir1, im1, im2, ndofs1, ndofs2, mj_lambda1,
                    mj_lambda2, mj_lambdas,
                );
                nrm_j_id += j_step;
            }
        }

        // Solve friction.
        if solve_friction {
            #[cfg(feature = "dim3")]
            let tangents1 = [tangent1, &dir1.cross(&tangent1)];
            #[cfg(feature = "dim2")]
            let tangents1 = [&dir1.orthonormal_vector()];
            let mut tng_j_id = tangent_j_id(j_id, ndofs1, ndofs2);

            for element in elements.iter_mut() {
                let limit = limit * element.normal_part.impulse;
                let part = &mut element.tangent_part;
                part.generic_solve(
                    tng_j_id, jacobians, tangents1, im1, im2, ndofs1, ndofs2, limit, mj_lambda1,
                    mj_lambda2, mj_lambdas,
                );
                tng_j_id += j_step;
            }
        }
    }
}
