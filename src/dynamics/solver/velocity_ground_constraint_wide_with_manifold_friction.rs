use super::{AnyVelocityConstraint, DeltaVel, SpringRegularization};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{
    AngVector, AngularInertia, Point, Real, SimdReal, Vector, DIM, MAX_MANIFOLD_POINTS, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WBasis, WCross, WDot};
use num::Zero;
use simba::simd::{SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
struct WVelocityConstraintPart {
    gcross2: AngVector<SimdReal>,
    rhs: SimdReal,
    impulse: SimdReal,
    r: SimdReal,
}

impl WVelocityConstraintPart {
    pub fn zero() -> Self {
        Self {
            gcross2: AngVector::zero(),
            rhs: SimdReal::zero(),
            impulse: SimdReal::zero(),
            r: SimdReal::zero(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct WVelocityGroundConstraintWithManifoldFriction {
    dir1: Vector<SimdReal>, // Non-penetration force direction for the first body.
    normal_parts: [WVelocityConstraintPart; MAX_MANIFOLD_POINTS],
    tangent_parts: [WVelocityConstraintPart; DIM - 1],
    twist_part: WVelocityConstraintPart,
    twist_weights: [SimdReal; MAX_MANIFOLD_POINTS],
    num_contacts: u8,
    im2: SimdReal,
    limit: SimdReal,
    mj_lambda2: [usize; SIMD_WIDTH],
    manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
    manifold_contact_id: usize,
    impulse_scale: SimdReal,
}

impl WVelocityGroundConstraintWithManifoldFriction {
    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        push: bool,
    ) {
        let (erp, cfm, impulse_scale) =
            SpringRegularization::default().erp_cfm_impulse_scale(params.dt);

        let inv_dt = SimdReal::splat(params.inv_dt());
        let mut rbs1 = array![|ii| &bodies[manifolds[ii].data.body_pair.body1]; SIMD_WIDTH];
        let mut rbs2 = array![|ii| &bodies[manifolds[ii].data.body_pair.body2]; SIMD_WIDTH];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if !rbs2[ii].is_dynamic() {
                std::mem::swap(&mut rbs1[ii], &mut rbs2[ii]);
                flipped[ii] = true;
            }
        }

        let force_dir1 = Vector::from(
            array![|ii| if flipped[ii] { manifolds[ii].data.normal } else { -manifolds[ii].data.normal }; SIMD_WIDTH],
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

        let mj_lambda2 = array![|ii| rbs2[ii].island_offset; SIMD_WIDTH];

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

            let mut constraint = WVelocityGroundConstraintWithManifoldFriction {
                dir1: force_dir1,
                normal_parts: [WVelocityConstraintPart::zero(); MAX_MANIFOLD_POINTS],
                tangent_parts: [WVelocityConstraintPart::zero(); DIM - 1],
                twist_part: WVelocityConstraintPart::zero(),
                twist_weights: [SimdReal::splat(0.0); MAX_MANIFOLD_POINTS],
                im2,
                limit: SimdReal::splat(0.0),
                mj_lambda2,
                manifold_id,
                manifold_contact_id: l,
                num_contacts: num_points as u8,
                impulse_scale: SimdReal::splat(impulse_scale),
            };

            let mut manifold_center = Point::origin();
            let mut tangent_impulses = [SimdReal::splat(0.0), SimdReal::splat(0.0)];

            for k in 0..num_points {
                let friction =
                    SimdReal::from(array![|ii| manifold_points[ii][k].friction; SIMD_WIDTH]);
                let restitution =
                    SimdReal::from(array![|ii| manifold_points[ii][k].restitution; SIMD_WIDTH]);
                let point = Point::from(array![|ii| manifold_points[ii][k].point; SIMD_WIDTH]);

                manifold_center += point.coords / SimdReal::splat(num_points as Real);

                let dist = SimdReal::from(array![|ii| manifold_points[ii][k].dist; SIMD_WIDTH]);

                let impulse =
                    SimdReal::from(array![|ii| manifold_points[ii][k].data.impulse; SIMD_WIDTH]);
                tangent_impulses[0] += SimdReal::from(
                    array![|ii| manifold_points[ii][k].data.tangent_impulse[0]; SIMD_WIDTH],
                );
                tangent_impulses[1] += SimdReal::from(
                    array![|ii| manifold_points[ii][k].data.tangent_impulse[1]; SIMD_WIDTH],
                );

                let dp1 = point - world_com1;
                let dp2 = point - world_com2;

                let vel1 = linvel1 + angvel1.gcross(dp1);
                let vel2 = linvel2 + angvel2.gcross(dp2);

                constraint.limit = friction;

                // Normal part.
                {
                    let gcross2 = ii2.transform_vector(dp2.gcross(-force_dir1));

                    let r =
                        SimdReal::splat(1.0) / (SimdReal::splat(cfm) + im2 + gcross2.gdot(gcross2));
                    let mut rhs = (vel1 - vel2).dot(&force_dir1);
                    let use_restitution = rhs.simd_le(-restitution_velocity_threshold);
                    let rhs_with_restitution = rhs + rhs * restitution;
                    rhs = rhs_with_restitution.select(use_restitution, rhs);

                    rhs += dist.simd_min(SimdReal::splat(0.0)) * SimdReal::splat(erp)
                        + dist.simd_max(SimdReal::splat(0.0)) * inv_dt;

                    constraint.normal_parts[k] = WVelocityConstraintPart {
                        gcross2,
                        rhs,
                        impulse: impulse * warmstart_coeff,
                        r,
                    };
                }
            }

            // tangent parts.
            let tangents1 = force_dir1.orthonormal_basis();

            for j in 0..DIM - 1 {
                let dp1 = manifold_center - world_com1;
                let dp2 = manifold_center - world_com2;

                let vel1 = linvel1 + angvel1.gcross(dp1);
                let vel2 = linvel2 + angvel2.gcross(dp2);

                let gcross2 = ii2.transform_vector(dp2.gcross(-tangents1[j]));
                let r = SimdReal::splat(1.0) / (im2 + gcross2.gdot(gcross2));
                let rhs = (vel1 - vel2).dot(&tangents1[j]);

                constraint.tangent_parts[j] = WVelocityConstraintPart {
                    gcross2,
                    rhs,
                    impulse: tangent_impulses[j] * warmstart_coeff,
                    r,
                };
            }

            // Twist part.
            {
                let twist_impulse =
                    SimdReal::from(array![|ii| manifolds[ii].data.twist_impulse; SIMD_WIDTH]);

                for k in 0..num_points {
                    let point = Point::from(array![|ii| manifold_points[ii][k].point; SIMD_WIDTH]);
                    constraint.twist_weights[k] = na::distance(&point, &manifold_center);
                }

                let gcross2 = ii2.transform_vector(-force_dir1);
                let inv_r = gcross2.norm_squared();
                let rhs = (angvel1 - angvel2).gdot(force_dir1);
                constraint.twist_part = WVelocityConstraintPart {
                    gcross2,
                    rhs,
                    impulse: twist_impulse * warmstart_coeff,
                    r: crate::utils::simd_inv(inv_r),
                }
            }

            if push {
                out_constraints.push(AnyVelocityConstraint::GroupedGround2(constraint));
            } else {
                out_constraints[manifolds[0].data.constraint_index + l / MAX_MANIFOLD_POINTS] =
                    AnyVelocityConstraint::GroupedGround2(constraint);
            }
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        // Normal part.
        for i in 0..self.num_contacts as usize {
            let elt = &self.normal_parts[i];
            mj_lambda2.linear += self.dir1 * (-self.im2 * elt.impulse);
            mj_lambda2.angular += elt.gcross2 * elt.impulse;
        }

        // Tangent part.
        {
            let tangents1 = self.dir1.orthonormal_basis();

            for j in 0..DIM - 1 {
                let elt = &self.tangent_parts[j];
                mj_lambda2.linear += tangents1[j] * (-self.im2 * elt.impulse);
                mj_lambda2.angular += elt.gcross2 * elt.impulse;
            }
        }

        // Twist part.
        {
            mj_lambda2.angular += self.twist_part.gcross2 * self.twist_part.impulse;
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(
                array![ |ii| mj_lambdas[ self.mj_lambda2[ii] as usize].linear; SIMD_WIDTH],
            ),
            angular: AngVector::from(
                array![ |ii| mj_lambdas[ self.mj_lambda2[ii] as usize].angular; SIMD_WIDTH],
            ),
        };

        // Solve friction first.
        let tangents1 = self.dir1.orthonormal_basis();
        let friction_limit = self.limit
            * (self.normal_parts[0].impulse
                + self.normal_parts[1].impulse
                + self.normal_parts[2].impulse
                + self.normal_parts[3].impulse);

        for j in 0..DIM - 1 {
            let elt = &mut self.tangent_parts[j];
            let dimpulse = -tangents1[j].dot(&mj_lambda2.linear)
                + elt.gcross2.gdot(mj_lambda2.angular)
                + elt.rhs;
            let new_impulse =
                (elt.impulse - elt.r * dimpulse).simd_clamp(-friction_limit, friction_limit);
            let dlambda = new_impulse - elt.impulse;
            elt.impulse = new_impulse;

            mj_lambda2.linear += tangents1[j] * (-self.im2 * dlambda);
            mj_lambda2.angular += elt.gcross2 * dlambda;
        }

        // Solve non-penetration after friction.
        for i in 0..self.num_contacts as usize {
            let elt = &mut self.normal_parts[i];
            let dimpulse =
                -self.dir1.dot(&mj_lambda2.linear) + elt.gcross2.gdot(mj_lambda2.angular) + elt.rhs;
            let new_impulse =
                (elt.impulse * self.impulse_scale - elt.r * dimpulse).simd_max(SimdReal::zero());
            let dlambda = new_impulse - elt.impulse;
            elt.impulse = new_impulse;

            mj_lambda2.linear += self.dir1 * (-self.im2 * dlambda);
            mj_lambda2.angular += elt.gcross2 * dlambda;
        }

        {
            let twist_limit = self.limit
                * (self.normal_parts[0].impulse * self.twist_weights[0]
                    + self.normal_parts[1].impulse * self.twist_weights[1]
                    + self.normal_parts[2].impulse * self.twist_weights[2]
                    + self.normal_parts[3].impulse * self.twist_weights[3]);

            let dimpulse = self.twist_part.gcross2.gdot(mj_lambda2.angular) + self.twist_part.rhs;
            let new_impulse = (self.twist_part.impulse - self.twist_part.r * dimpulse)
                .simd_clamp(-twist_limit, twist_limit);
            let dlambda = new_impulse - self.twist_part.impulse;
            self.twist_part.impulse = new_impulse;

            mj_lambda2.angular += self.twist_part.gcross2 * dlambda;
        }

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        let denom = crate::utils::simd_inv(
            self.normal_parts[0].impulse
                + self.normal_parts[1].impulse
                + self.normal_parts[2].impulse
                + self.normal_parts[3].impulse,
        );

        for k in 0..self.num_contacts as usize {
            let normal_factor = self.normal_parts[k].impulse * denom;
            let tangent_impulses = self.tangent_parts[0].impulse * normal_factor;
            let bitangent_impulses = self.tangent_parts[1].impulse * normal_factor;

            let impulses: [_; SIMD_WIDTH] = self.normal_parts[k].impulse.into();
            let tangent_impulses: [_; SIMD_WIDTH] = tangent_impulses.into();
            let bitangent_impulses: [_; SIMD_WIDTH] = bitangent_impulses.into();

            for ii in 0..SIMD_WIDTH {
                let manifold = &mut manifolds_all[self.manifold_id[ii]];
                let k_base = self.manifold_contact_id;
                let active_contacts = &mut manifold.points[..manifold.data.num_active_contacts()];
                active_contacts[k_base + k].data.impulse = impulses[ii];
                active_contacts[k_base + k].data.tangent_impulse =
                    [tangent_impulses[ii], bitangent_impulses[ii]];
            }
        }

        let twist_impulse: [_; SIMD_WIDTH] = self.twist_part.impulse.into();

        for ii in 0..SIMD_WIDTH {
            let manifold = &mut manifolds_all[self.manifold_id[ii]];
            manifold.data.twist_impulse = twist_impulse[ii];
        }
    }
}
