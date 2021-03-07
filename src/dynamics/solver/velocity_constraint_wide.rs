use super::{
    AnyVelocityConstraint, DeltaVel, VelocityConstraintElement, VelocityConstraintNormalPart,
};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{
    AngVector, AngularInertia, Point, Real, SimdReal, Vector, DIM, MAX_MANIFOLD_POINTS, SIMD_WIDTH,
};
#[cfg(feature = "dim2")]
use crate::utils::WBasis;
use crate::utils::{WAngularInertia, WCross, WDot};
use num::Zero;
use simba::simd::{SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
pub(crate) struct WVelocityConstraint {
    pub dir1: Vector<SimdReal>, // Non-penetration force direction for the first body.
    #[cfg(feature = "dim3")]
    pub tangent1: Vector<SimdReal>, // One of the friction force directions.
    #[cfg(feature = "dim3")]
    pub tangent_rot1: na::UnitComplex<SimdReal>, // Orientation of the tangent basis wrt. the reference basis.
    pub elements: [VelocityConstraintElement<SimdReal>; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
    pub im1: SimdReal,
    pub im2: SimdReal,
    pub limit: SimdReal,
    pub mj_lambda1: [usize; SIMD_WIDTH],
    pub mj_lambda2: [usize; SIMD_WIDTH],
    pub manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
    pub manifold_contact_id: [[u8; SIMD_WIDTH]; MAX_MANIFOLD_POINTS],
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
        for ii in 0..SIMD_WIDTH {
            assert_eq!(manifolds[ii].data.relative_dominance, 0);
        }

        let inv_dt = SimdReal::splat(params.inv_dt());
        let velocity_solve_fraction = SimdReal::splat(params.velocity_solve_fraction);
        let velocity_based_erp_inv_dt = SimdReal::splat(params.velocity_based_erp_inv_dt());

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

        let warmstart_multiplier =
            SimdReal::from(array![|ii| manifolds[ii].data.warmstart_multiplier; SIMD_WIDTH]);
        let warmstart_coeff = warmstart_multiplier * SimdReal::splat(params.warmstart_coeff);
        let num_active_contacts = manifolds[0].data.num_active_contacts();

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let (tangents1, tangent_rot1) =
            super::compute_tangent_contact_directions(&force_dir1, &linvel1, &linvel2);

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = array![|ii|
                &manifolds[ii].data.solver_contacts[l..num_active_contacts]; SIMD_WIDTH
            ];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WVelocityConstraint {
                dir1: force_dir1,
                #[cfg(feature = "dim3")]
                tangent1: tangents1[0],
                #[cfg(feature = "dim3")]
                tangent_rot1,
                elements: [VelocityConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im1,
                im2,
                limit: SimdReal::splat(0.0),
                mj_lambda1,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: [[0; SIMD_WIDTH]; MAX_MANIFOLD_POINTS],
                num_contacts: num_points as u8,
            };

            for k in 0..num_points {
                let friction =
                    SimdReal::from(array![|ii| manifold_points[ii][k].friction; SIMD_WIDTH]);
                let restitution =
                    SimdReal::from(array![|ii| manifold_points[ii][k].restitution; SIMD_WIDTH]);
                let is_bouncy = SimdReal::from(
                    array![|ii| manifold_points[ii][k].is_bouncy() as u32 as Real; SIMD_WIDTH],
                );
                let is_resting = SimdReal::splat(1.0) - is_bouncy;
                let point = Point::from(array![|ii| manifold_points[ii][k].point; SIMD_WIDTH]);
                let dist = SimdReal::from(array![|ii| manifold_points[ii][k].dist; SIMD_WIDTH]);
                let tangent_velocity =
                    Vector::from(array![|ii| manifold_points[ii][k].tangent_velocity; SIMD_WIDTH]);

                let impulse =
                    SimdReal::from(array![|ii| manifold_points[ii][k].data.impulse; SIMD_WIDTH]);

                let dp1 = point - world_com1;
                let dp2 = point - world_com2;

                let vel1 = linvel1 + angvel1.gcross(dp1);
                let vel2 = linvel2 + angvel2.gcross(dp2);

                constraint.limit = friction;
                constraint.manifold_contact_id[k] =
                    array![|ii| manifold_points[ii][k].contact_id; SIMD_WIDTH];

                // Normal part.
                {
                    let gcross1 = ii1.transform_vector(dp1.gcross(force_dir1));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-force_dir1));

                    let r = SimdReal::splat(1.0)
                        / (im1 + im2 + gcross1.gdot(gcross1) + gcross2.gdot(gcross2));
                    let projected_velocity = (vel1 - vel2).dot(&force_dir1);
                    let mut rhs =
                        (SimdReal::splat(1.0) + is_bouncy * restitution) * projected_velocity;
                    rhs += dist.simd_max(SimdReal::zero()) * inv_dt;
                    rhs *= is_bouncy + is_resting * velocity_solve_fraction;
                    rhs +=
                        dist.simd_min(SimdReal::zero()) * (velocity_based_erp_inv_dt * is_resting);

                    constraint.elements[k].normal_part = VelocityConstraintNormalPart {
                        gcross1,
                        gcross2,
                        rhs,
                        impulse: impulse * warmstart_coeff,
                        r,
                    };
                }

                // tangent parts.
                #[cfg(feature = "dim2")]
                let impulse = [SimdReal::from(
                    array![|ii| manifold_points[ii][k].data.tangent_impulse; SIMD_WIDTH],
                )];

                #[cfg(feature = "dim3")]
                let impulse = tangent_rot1
                    * na::Vector2::from(
                        array![|ii| manifold_points[ii][k].data.tangent_impulse; SIMD_WIDTH],
                    );

                constraint.elements[k].tangent_part.impulse = impulse;

                for j in 0..DIM - 1 {
                    let gcross1 = ii1.transform_vector(dp1.gcross(tangents1[j]));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-tangents1[j]));
                    let r = SimdReal::splat(1.0)
                        / (im1 + im2 + gcross1.gdot(gcross1) + gcross2.gdot(gcross2));
                    let rhs = (vel1 - vel2 + tangent_velocity).dot(&tangents1[j]);

                    constraint.elements[k].tangent_part.gcross1[j] = gcross1;
                    constraint.elements[k].tangent_part.gcross2[j] = gcross2;
                    constraint.elements[k].tangent_part.rhs[j] = rhs;
                    constraint.elements[k].tangent_part.r[j] = r;
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

        VelocityConstraintElement::warmstart_group(
            &self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            self.im1,
            self.im2,
            &mut mj_lambda1,
            &mut mj_lambda2,
        );

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
                array![|ii| mj_lambdas[ self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[ self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        VelocityConstraintElement::solve_group(
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            self.im1,
            self.im2,
            self.limit,
            &mut mj_lambda1,
            &mut mj_lambda2,
        );

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
            #[cfg(feature = "dim2")]
            let tangent_impulses: [_; SIMD_WIDTH] = self.elements[k].tangent_part.impulse[0].into();
            #[cfg(feature = "dim3")]
            let tangent_impulses = self
                .tangent_rot1
                .inverse_transform_vector(&self.elements[k].tangent_part.impulse);

            for ii in 0..SIMD_WIDTH {
                let manifold = &mut manifolds_all[self.manifold_id[ii]];
                let contact_id = self.manifold_contact_id[k][ii];
                let active_contact = &mut manifold.points[contact_id as usize];
                active_contact.data.impulse = impulses[ii];

                #[cfg(feature = "dim2")]
                {
                    active_contact.data.tangent_impulse = tangent_impulses[ii];
                }
                #[cfg(feature = "dim3")]
                {
                    active_contact.data.tangent_impulse = tangent_impulses.extract(ii);
                }
            }
        }
    }
}
