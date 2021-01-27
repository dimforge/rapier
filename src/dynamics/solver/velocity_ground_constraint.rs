use super::{AnyVelocityConstraint, DeltaVel};
use crate::math::{AngVector, Vector, DIM, MAX_MANIFOLD_POINTS};
use crate::utils::{WAngularInertia, WBasis, WCross, WDot};

use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use simba::simd::SimdPartialOrd;

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityGroundConstraintElementPart {
    pub gcross2: AngVector<f32>,
    pub rhs: f32,
    pub impulse: f32,
    pub r: f32,
}

#[cfg(not(target_arch = "wasm32"))]
impl VelocityGroundConstraintElementPart {
    fn zero() -> Self {
        Self {
            gcross2: na::zero(),
            rhs: 0.0,
            impulse: 0.0,
            r: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityGroundConstraintElement {
    pub normal_part: VelocityGroundConstraintElementPart,
    pub tangent_part: [VelocityGroundConstraintElementPart; DIM - 1],
}

#[cfg(not(target_arch = "wasm32"))]
impl VelocityGroundConstraintElement {
    pub fn zero() -> Self {
        Self {
            normal_part: VelocityGroundConstraintElementPart::zero(),
            tangent_part: [VelocityGroundConstraintElementPart::zero(); DIM - 1],
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityGroundConstraint {
    pub dir1: Vector<f32>, // Non-penetration force direction for the first body.
    pub im2: f32,
    pub limit: f32,
    pub mj_lambda2: usize,
    pub manifold_id: ContactManifoldIndex,
    pub manifold_contact_id: usize,
    pub num_contacts: u8,
    pub elements: [VelocityGroundConstraintElement; MAX_MANIFOLD_POINTS],
}

impl VelocityGroundConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        push: bool,
    ) {
        let inv_dt = params.inv_dt();
        let mut rb1 = &bodies[manifold.body_pair.body1];
        let mut rb2 = &bodies[manifold.body_pair.body2];
        let flipped = !rb2.is_dynamic();
        let force_dir1;
        let coll_pos1;
        let coll_pos2;

        if flipped {
            coll_pos1 = rb2.position * manifold.delta2;
            coll_pos2 = rb1.position * manifold.delta1;
            force_dir1 = coll_pos1 * (-manifold.local_n2);
            std::mem::swap(&mut rb1, &mut rb2);
        } else {
            coll_pos1 = rb1.position * manifold.delta1;
            coll_pos2 = rb2.position * manifold.delta2;
            force_dir1 = coll_pos1 * (-manifold.local_n1);
        }

        let mj_lambda2 = rb2.active_set_offset;
        let warmstart_coeff = manifold.warmstart_multiplier * params.warmstart_coeff;
        let is_bouncing = manifold.is_bouncing();

        for (l, manifold_points) in manifold
            .active_contacts()
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            #[cfg(not(target_arch = "wasm32"))]
            let mut constraint = VelocityGroundConstraint {
                dir1: force_dir1,
                elements: [VelocityGroundConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im2: rb2.mass_properties.inv_mass,
                limit: manifold.friction,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: l * MAX_MANIFOLD_POINTS,
                num_contacts: manifold_points.len() as u8,
            };

            // TODO: this is a WIP optimization for WASM platforms.
            // For some reasons, the compiler does not inline the `Vec::push` method
            // in this method. This generates two memset and one memcpy which are both very
            // expansive.
            // This would likely be solved by some kind of "placement-push" (like emplace in C++).
            // In the mean time, a workaround is to "push" using `.resize_with` and `::uninit()` to
            // avoid spurious copying.
            // Is this optimization beneficial when targeting non-WASM platforms?
            //
            // NOTE: joints have the same problem, but it is not easy to refactor the code that way
            // for the moment.
            #[cfg(target_arch = "wasm32")]
            let constraint = if push {
                let new_len = out_constraints.len() + 1;
                unsafe {
                    out_constraints.resize_with(new_len, || {
                        AnyVelocityConstraint::NongroupedGround(
                            std::mem::MaybeUninit::uninit().assume_init(),
                        )
                    });
                }
                out_constraints
                    .last_mut()
                    .unwrap()
                    .as_nongrouped_ground_mut()
                    .unwrap()
            } else {
                unreachable!(); // We don't have parallelization on WASM yet, so this is unreachable.
            };

            #[cfg(target_arch = "wasm32")]
            {
                constraint.dir1 = force_dir1;
                constraint.im2 = rb2.mass_properties.inv_mass;
                constraint.limit = manifold.friction;
                constraint.mj_lambda2 = mj_lambda2;
                constraint.manifold_id = manifold_id;
                constraint.manifold_contact_id = l * MAX_MANIFOLD_POINTS;
                constraint.num_contacts = manifold_points.len() as u8;
            }

            for (k, manifold_point) in manifold_points.iter().enumerate() {
                let (p1, p2) = if flipped {
                    // NOTE: we already swapped rb1 and rb2
                    // so we multiply by coll_pos1/coll_pos2.
                    (
                        coll_pos1 * manifold_point.local_p2,
                        coll_pos2 * manifold_point.local_p1,
                    )
                } else {
                    (
                        coll_pos1 * manifold_point.local_p1,
                        coll_pos2 * manifold_point.local_p2,
                    )
                };
                let dp2 = p2 - rb2.world_com;
                let dp1 = p1 - rb1.world_com;
                let vel1 = rb1.linvel + rb1.angvel.gcross(dp1);
                let vel2 = rb2.linvel + rb2.angvel.gcross(dp2);

                // Normal part.
                {
                    let gcross2 = rb2
                        .world_inv_inertia_sqrt
                        .transform_vector(dp2.gcross(-force_dir1));

                    let r = 1.0 / (rb2.mass_properties.inv_mass + gcross2.gdot(gcross2));

                    let rhs = if is_bouncing {
                        (1.0 + manifold.restitution) * (vel1 - vel2).dot(&force_dir1)
                    } else {
                        (vel1 - vel2).dot(&force_dir1) + manifold_point.dist.max(0.0) * inv_dt
                    };

                    let impulse = manifold_point.impulse * warmstart_coeff;

                    constraint.elements[k].normal_part = VelocityGroundConstraintElementPart {
                        gcross2,
                        rhs,
                        impulse,
                        r,
                    };
                }

                // Tangent parts.
                {
                    let tangents1 = force_dir1.orthonormal_basis();

                    for j in 0..DIM - 1 {
                        let gcross2 = rb2
                            .world_inv_inertia_sqrt
                            .transform_vector(dp2.gcross(-tangents1[j]));
                        let r = 1.0 / (rb2.mass_properties.inv_mass + gcross2.gdot(gcross2));
                        let rhs = -vel2.dot(&tangents1[j]) + vel1.dot(&tangents1[j]);
                        #[cfg(feature = "dim2")]
                        let impulse = manifold_point.tangent_impulse * warmstart_coeff;
                        #[cfg(feature = "dim3")]
                        let impulse = manifold_point.tangent_impulse[j] * warmstart_coeff;

                        constraint.elements[k].tangent_part[j] =
                            VelocityGroundConstraintElementPart {
                                gcross2,
                                rhs,
                                impulse,
                                r,
                            };
                    }
                }
            }

            #[cfg(not(target_arch = "wasm32"))]
            if push {
                out_constraints.push(AnyVelocityConstraint::NongroupedGround(constraint));
            } else {
                out_constraints[manifold.constraint_index + l] =
                    AnyVelocityConstraint::NongroupedGround(constraint);
            }
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = DeltaVel::zero();
        let tangents1 = self.dir1.orthonormal_basis();

        for i in 0..self.num_contacts as usize {
            let elt = &self.elements[i].normal_part;
            mj_lambda2.linear += self.dir1 * (-self.im2 * elt.impulse);
            mj_lambda2.angular += elt.gcross2 * elt.impulse;

            for j in 0..DIM - 1 {
                let elt = &self.elements[i].tangent_part[j];
                mj_lambda2.linear += tangents1[j] * (-self.im2 * elt.impulse);
                mj_lambda2.angular += elt.gcross2 * elt.impulse;
            }
        }

        mj_lambdas[self.mj_lambda2 as usize].linear += mj_lambda2.linear;
        mj_lambdas[self.mj_lambda2 as usize].angular += mj_lambda2.angular;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        // Solve friction.
        let tangents1 = self.dir1.orthonormal_basis();

        for i in 0..self.num_contacts as usize {
            for j in 0..DIM - 1 {
                let normal_elt = &self.elements[i].normal_part;
                let elt = &mut self.elements[i].tangent_part[j];
                let dimpulse = -tangents1[j].dot(&mj_lambda2.linear)
                    + elt.gcross2.gdot(mj_lambda2.angular)
                    + elt.rhs;
                let limit = self.limit * normal_elt.impulse;
                let new_impulse = (elt.impulse - elt.r * dimpulse).simd_clamp(-limit, limit);
                let dlambda = new_impulse - elt.impulse;
                elt.impulse = new_impulse;

                mj_lambda2.linear += tangents1[j] * (-self.im2 * dlambda);
                mj_lambda2.angular += elt.gcross2 * dlambda;
            }
        }

        // Solve penetration.
        for i in 0..self.num_contacts as usize {
            let elt = &mut self.elements[i].normal_part;
            let dimpulse =
                -self.dir1.dot(&mj_lambda2.linear) + elt.gcross2.gdot(mj_lambda2.angular) + elt.rhs;
            let new_impulse = (elt.impulse - elt.r * dimpulse).max(0.0);
            let dlambda = new_impulse - elt.impulse;
            elt.impulse = new_impulse;

            mj_lambda2.linear += self.dir1 * (-self.im2 * dlambda);
            mj_lambda2.angular += elt.gcross2 * dlambda;
        }

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code. This is exactly the same as in the non-ground velocity constraint.
    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        let manifold = &mut manifolds_all[self.manifold_id];
        manifold.old = true;
        let k_base = self.manifold_contact_id;

        for k in 0..self.num_contacts as usize {
            let active_contacts = manifold.active_contacts_mut();
            active_contacts[k_base + k].impulse = self.elements[k].normal_part.impulse;
            #[cfg(feature = "dim2")]
            {
                active_contacts[k_base + k].tangent_impulse =
                    self.elements[k].tangent_part[0].impulse;
            }
            #[cfg(feature = "dim3")]
            {
                active_contacts[k_base + k].tangent_impulse = [
                    self.elements[k].tangent_part[0].impulse,
                    self.elements[k].tangent_part[1].impulse,
                ];
            }
        }
    }
}
