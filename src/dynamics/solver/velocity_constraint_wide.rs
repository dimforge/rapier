use super::{AnyVelocityConstraint, DeltaVel};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{
    AngVector, AngularInertia, Point, Real, SimdReal, Vector, DIM, MAX_MANIFOLD_POINTS, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WBasis, WCross, WDot};
use num::Zero;
use simba::simd::{SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
pub(crate) struct WVelocityConstraintElementPart {
    pub gcross1: AngVector<SimdReal>,
    pub gcross2: AngVector<SimdReal>,
    pub rhs: SimdReal,
    pub impulse: SimdReal,
    pub r: SimdReal,
}

impl WVelocityConstraintElementPart {
    pub fn zero() -> Self {
        Self {
            gcross1: AngVector::zero(),
            gcross2: AngVector::zero(),
            rhs: SimdReal::zero(),
            impulse: SimdReal::zero(),
            r: SimdReal::zero(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct WVelocityConstraintElement {
    pub normal_part: WVelocityConstraintElementPart,
    pub tangent_parts: [WVelocityConstraintElementPart; DIM - 1],
}

impl WVelocityConstraintElement {
    pub fn zero() -> Self {
        Self {
            normal_part: WVelocityConstraintElementPart::zero(),
            tangent_parts: [WVelocityConstraintElementPart::zero(); DIM - 1],
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct WVelocityConstraint {
    pub dir1: Vector<SimdReal>, // Non-penetration force direction for the first body.
    pub elements: [WVelocityConstraintElement; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
    pub im1: SimdReal,
    pub im2: SimdReal,
    pub limit: SimdReal,
    pub mj_lambda1: [usize; SIMD_WIDTH],
    pub mj_lambda2: [usize; SIMD_WIDTH],
    pub manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
    pub manifold_contact_id: usize,
}

impl WVelocityConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        push: bool,
    ) {
        let inv_dt = SimdReal::splat(params.inv_dt());
        let rbs1 = array![|ii| &bodies[manifolds[ii].data.body_pair.body1]; SIMD_WIDTH];
        let rbs2 = array![|ii| &bodies[manifolds[ii].data.body_pair.body2]; SIMD_WIDTH];

        let im1 = SimdReal::from(array![|ii| rbs1[ii].effective_inv_mass; SIMD_WIDTH]);
        let ii1: AngularInertia<SimdReal> = AngularInertia::from(
            array![|ii| rbs1[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );

        let linvel1 = Vector::from(array![|ii| rbs1[ii].linvel; SIMD_WIDTH]);
        let angvel1 = AngVector::<SimdReal>::from(array![|ii| rbs1[ii].angvel; SIMD_WIDTH]);

        let world_com1 = Point::from(array![|ii| rbs1[ii].world_com; SIMD_WIDTH]);

        let im2 = SimdReal::from(array![|ii| rbs2[ii].effective_inv_mass; SIMD_WIDTH]);
        let ii2: AngularInertia<SimdReal> = AngularInertia::from(
            array![|ii| rbs2[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );

        let linvel2 = Vector::from(array![|ii| rbs2[ii].linvel; SIMD_WIDTH]);
        let angvel2 = AngVector::<SimdReal>::from(array![|ii| rbs2[ii].angvel; SIMD_WIDTH]);

        let world_com2 = Point::from(array![|ii| rbs2[ii].world_com; SIMD_WIDTH]);

        let force_dir1 = -Vector::from(array![|ii| manifolds[ii].data.normal; SIMD_WIDTH]);

        let mj_lambda1 = array![|ii| rbs1[ii].active_set_offset; SIMD_WIDTH];
        let mj_lambda2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let restitution_velocity_threshold = SimdReal::splat(params.restitution_velocity_threshold);

        let warmstart_multiplier =
            SimdReal::from(array![|ii| manifolds[ii].data.warmstart_multiplier; SIMD_WIDTH]);
        let warmstart_coeff = warmstart_multiplier * SimdReal::splat(params.warmstart_coeff);
        let num_active_contacts = manifolds[0].data.num_active_contacts();

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = array![|ii|
                &manifolds[ii].data.solver_contacts[l..num_active_contacts]; SIMD_WIDTH
            ];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WVelocityConstraint {
                dir1: force_dir1,
                elements: [WVelocityConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im1,
                im2,
                limit: SimdReal::splat(0.0),
                mj_lambda1,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: l,
                num_contacts: num_points as u8,
            };

            for k in 0..num_points {
                let friction =
                    SimdReal::from(array![|ii| manifold_points[ii][k].friction; SIMD_WIDTH]);
                let restitution =
                    SimdReal::from(array![|ii| manifold_points[ii][k].restitution; SIMD_WIDTH]);
                let point = Point::from(array![|ii| manifold_points[ii][k].point; SIMD_WIDTH]);
                let dist = SimdReal::from(array![|ii| manifold_points[ii][k].dist; SIMD_WIDTH]);

                let impulse =
                    SimdReal::from(array![|ii| manifold_points[ii][k].data.impulse; SIMD_WIDTH]);

                let dp1 = point - world_com1;
                let dp2 = point - world_com2;

                let vel1 = linvel1 + angvel1.gcross(dp1);
                let vel2 = linvel2 + angvel2.gcross(dp2);

                constraint.limit = friction;

                // Normal part.
                {
                    let gcross1 = ii1.transform_vector(dp1.gcross(force_dir1));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-force_dir1));

                    let r = SimdReal::splat(1.0)
                        / (im1 + im2 + gcross1.gdot(gcross1) + gcross2.gdot(gcross2));
                    let mut rhs = (vel1 - vel2).dot(&force_dir1);
                    let use_restitution = rhs.simd_le(-restitution_velocity_threshold);
                    let rhs_with_restitution = rhs + rhs * restitution;
                    rhs = rhs_with_restitution.select(use_restitution, rhs);

                    rhs += ((dist.simd_min(SimdReal::splat(0.0))
                        * SimdReal::splat(params.velocity_erp))
                        + dist.simd_max(SimdReal::splat(0.0)))
                        * inv_dt;

                    constraint.elements[k].normal_part = WVelocityConstraintElementPart {
                        gcross1,
                        gcross2,
                        rhs,
                        impulse: impulse * warmstart_coeff,
                        r,
                    };
                }

                // tangent parts.
                let tangents1 = force_dir1.orthonormal_basis();

                for j in 0..DIM - 1 {
                    #[cfg(feature = "dim2")]
                    let impulse = SimdReal::from(
                        array![|ii| manifold_points[ii][k].data.tangent_impulse; SIMD_WIDTH],
                    );
                    #[cfg(feature = "dim3")]
                    let impulse = SimdReal::from(
                        array![|ii| manifold_points[ii][k].data.tangent_impulse[j]; SIMD_WIDTH],
                    );

                    let gcross1 = ii1.transform_vector(dp1.gcross(tangents1[j]));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-tangents1[j]));
                    let r = SimdReal::splat(1.0)
                        / (im1 + im2 + gcross1.gdot(gcross1) + gcross2.gdot(gcross2));
                    let rhs = (vel1 - vel2).dot(&tangents1[j]);

                    constraint.elements[k].tangent_parts[j] = WVelocityConstraintElementPart {
                        gcross1,
                        gcross2,
                        rhs,
                        impulse: impulse * warmstart_coeff,
                        r,
                    };
                }
            }

            if push {
                out_constraints.push(AnyVelocityConstraint::Grouped(constraint));
            } else {
                out_constraints[manifolds[0].data.constraint_index + l / MAX_MANIFOLD_POINTS] =
                    AnyVelocityConstraint::Grouped(constraint);
            }
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        for i in 0..self.num_contacts as usize {
            let elt = &self.elements[i].normal_part;
            mj_lambda1.linear += self.dir1 * (self.im1 * elt.impulse);
            mj_lambda1.angular += elt.gcross1 * elt.impulse;

            mj_lambda2.linear += self.dir1 * (-self.im2 * elt.impulse);
            mj_lambda2.angular += elt.gcross2 * elt.impulse;

            // FIXME: move this out of the for loop?
            let tangents1 = self.dir1.orthonormal_basis();

            for j in 0..DIM - 1 {
                let elt = &self.elements[i].tangent_parts[j];
                mj_lambda1.linear += tangents1[j] * (self.im1 * elt.impulse);
                mj_lambda1.angular += elt.gcross1 * elt.impulse;

                mj_lambda2.linear += tangents1[j] * (-self.im2 * elt.impulse);
                mj_lambda2.angular += elt.gcross2 * elt.impulse;
            }
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda1[ii] as usize].linear = mj_lambda1.linear.extract(ii);
            mj_lambdas[self.mj_lambda1[ii] as usize].angular = mj_lambda1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![ |ii| mj_lambdas[ self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![ |ii| mj_lambdas[ self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        // Solve friction first.
        for i in 0..self.num_contacts as usize {
            // FIXME: move this out of the for loop?
            let tangents1 = self.dir1.orthonormal_basis();
            let normal_elt = &self.elements[i].normal_part;

            for j in 0..DIM - 1 {
                let elt = &mut self.elements[i].tangent_parts[j];
                let dimpulse = tangents1[j].dot(&mj_lambda1.linear)
                    + elt.gcross1.gdot(mj_lambda1.angular)
                    - tangents1[j].dot(&mj_lambda2.linear)
                    + elt.gcross2.gdot(mj_lambda2.angular)
                    + elt.rhs;
                let limit = self.limit * normal_elt.impulse;
                let new_impulse = (elt.impulse - elt.r * dimpulse).simd_clamp(-limit, limit);
                let dlambda = new_impulse - elt.impulse;
                elt.impulse = new_impulse;

                mj_lambda1.linear += tangents1[j] * (self.im1 * dlambda);
                mj_lambda1.angular += elt.gcross1 * dlambda;
                mj_lambda2.linear += tangents1[j] * (-self.im2 * dlambda);
                mj_lambda2.angular += elt.gcross2 * dlambda;
            }
        }

        // Solve non-penetration after friction.
        for i in 0..self.num_contacts as usize {
            let elt = &mut self.elements[i].normal_part;
            let dimpulse = self.dir1.dot(&mj_lambda1.linear) + elt.gcross1.gdot(mj_lambda1.angular)
                - self.dir1.dot(&mj_lambda2.linear)
                + elt.gcross2.gdot(mj_lambda2.angular)
                + elt.rhs;
            let new_impulse = (elt.impulse - elt.r * dimpulse).simd_max(SimdReal::zero());
            let dlambda = new_impulse - elt.impulse;
            elt.impulse = new_impulse;

            mj_lambda1.linear += self.dir1 * (self.im1 * dlambda);
            mj_lambda1.angular += elt.gcross1 * dlambda;
            mj_lambda2.linear += self.dir1 * (-self.im2 * dlambda);
            mj_lambda2.angular += elt.gcross2 * dlambda;
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda1[ii] as usize].linear = mj_lambda1.linear.extract(ii);
            mj_lambdas[self.mj_lambda1[ii] as usize].angular = mj_lambda1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        for k in 0..self.num_contacts as usize {
            let impulses: [_; SIMD_WIDTH] = self.elements[k].normal_part.impulse.into();
            let tangent_impulses: [_; SIMD_WIDTH] =
                self.elements[k].tangent_parts[0].impulse.into();
            #[cfg(feature = "dim3")]
            let bitangent_impulses: [_; SIMD_WIDTH] =
                self.elements[k].tangent_parts[1].impulse.into();

            for ii in 0..SIMD_WIDTH {
                let manifold = &mut manifolds_all[self.manifold_id[ii]];
                let k_base = self.manifold_contact_id;
                let active_contacts = &mut manifold.points[..manifold.data.num_active_contacts()];
                active_contacts[k_base + k].data.impulse = impulses[ii];

                #[cfg(feature = "dim2")]
                {
                    active_contacts[k_base + k].data.tangent_impulse = tangent_impulses[ii];
                }
                #[cfg(feature = "dim3")]
                {
                    active_contacts[k_base + k].data.tangent_impulse =
                        [tangent_impulses[ii], bitangent_impulses[ii]];
                }
            }
        }
    }
}
