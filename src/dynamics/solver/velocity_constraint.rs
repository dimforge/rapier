use super::DeltaVel;
use crate::dynamics::solver::VelocityGroundConstraint;
#[cfg(feature = "simd-is-enabled")]
use crate::dynamics::solver::{WVelocityConstraint, WVelocityGroundConstraint};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{AngVector, Vector, DIM, MAX_MANIFOLD_POINTS};
use crate::utils::{WAngularInertia, WBasis, WCross, WDot};
use simba::simd::SimdPartialOrd;

//#[repr(align(64))]
#[derive(Copy, Clone, Debug)]
pub(crate) enum AnyVelocityConstraint {
    NongroupedGround(VelocityGroundConstraint),
    Nongrouped(VelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedGround(WVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    Grouped(WVelocityConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyVelocityConstraint {
    #[cfg(target_arch = "wasm32")]
    pub fn as_nongrouped_mut(&mut self) -> Option<&mut VelocityConstraint> {
        if let AnyVelocityConstraint::Nongrouped(c) = self {
            Some(c)
        } else {
            None
        }
    }

    #[cfg(target_arch = "wasm32")]
    pub fn as_nongrouped_ground_mut(&mut self) -> Option<&mut VelocityGroundConstraint> {
        if let AnyVelocityConstraint::NongroupedGround(c) = self {
            Some(c)
        } else {
            None
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        match self {
            AnyVelocityConstraint::NongroupedGround(c) => c.warmstart(mj_lambdas),
            AnyVelocityConstraint::Nongrouped(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::GroupedGround(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::Grouped(c) => c.warmstart(mj_lambdas),
            AnyVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        match self {
            AnyVelocityConstraint::NongroupedGround(c) => c.solve(mj_lambdas),
            AnyVelocityConstraint::Nongrouped(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::GroupedGround(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::Grouped(c) => c.solve(mj_lambdas),
            AnyVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn writeback_impulses(&self, manifold_all: &mut [&mut ContactManifold]) {
        match self {
            AnyVelocityConstraint::NongroupedGround(c) => c.writeback_impulses(manifold_all),
            AnyVelocityConstraint::Nongrouped(c) => c.writeback_impulses(manifold_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::GroupedGround(c) => c.writeback_impulses(manifold_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::Grouped(c) => c.writeback_impulses(manifold_all),
            AnyVelocityConstraint::Empty => unreachable!(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraintElementPart {
    pub gcross1: AngVector<f32>,
    pub gcross2: AngVector<f32>,
    pub rhs: f32,
    pub impulse: f32,
    pub r: f32,
}

#[cfg(not(target_arch = "wasm32"))]
impl VelocityConstraintElementPart {
    fn zero() -> Self {
        Self {
            gcross1: na::zero(),
            gcross2: na::zero(),
            rhs: 0.0,
            impulse: 0.0,
            r: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraintElement {
    pub normal_part: VelocityConstraintElementPart,
    pub tangent_part: [VelocityConstraintElementPart; DIM - 1],
}

#[cfg(not(target_arch = "wasm32"))]
impl VelocityConstraintElement {
    pub fn zero() -> Self {
        Self {
            normal_part: VelocityConstraintElementPart::zero(),
            tangent_part: [VelocityConstraintElementPart::zero(); DIM - 1],
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraint {
    pub dir1: Vector<f32>, // Non-penetration force direction for the first body.
    pub im1: f32,
    pub im2: f32,
    pub limit: f32,
    pub mj_lambda1: usize,
    pub mj_lambda2: usize,
    pub manifold_id: ContactManifoldIndex,
    pub manifold_contact_id: usize,
    pub num_contacts: u8,
    pub elements: [VelocityConstraintElement; MAX_MANIFOLD_POINTS],
}

impl VelocityConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints(manifold: &ContactManifold) -> usize {
        let rest = manifold.num_active_contacts() % MAX_MANIFOLD_POINTS != 0;
        manifold.num_active_contacts() / MAX_MANIFOLD_POINTS + rest as usize
    }

    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        push: bool,
    ) {
        let inv_dt = params.inv_dt();
        let rb1 = &bodies[manifold.body_pair.body1];
        let rb2 = &bodies[manifold.body_pair.body2];
        let mj_lambda1 = rb1.active_set_offset;
        let mj_lambda2 = rb2.active_set_offset;
        let pos_coll1 = rb1.position * manifold.delta1;
        let pos_coll2 = rb2.position * manifold.delta2;
        let force_dir1 = pos_coll1 * (-manifold.local_n1);
        let warmstart_coeff = manifold.warmstart_multiplier * params.warmstart_coeff;
        let is_bouncing = manifold.is_bouncing();

        for (l, manifold_points) in manifold
            .active_contacts()
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            #[cfg(not(target_arch = "wasm32"))]
            let mut constraint = VelocityConstraint {
                dir1: force_dir1,
                elements: [VelocityConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im1: rb1.mass_properties.inv_mass,
                im2: rb2.mass_properties.inv_mass,
                limit: manifold.friction,
                mj_lambda1,
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
                        AnyVelocityConstraint::Nongrouped(
                            std::mem::MaybeUninit::uninit().assume_init(),
                        )
                    });
                }
                out_constraints
                    .last_mut()
                    .unwrap()
                    .as_nongrouped_mut()
                    .unwrap()
            } else {
                unreachable!(); // We don't have parallelization on WASM yet, so this is unreachable.
            };

            #[cfg(target_arch = "wasm32")]
            {
                constraint.dir1 = force_dir1;
                constraint.im1 = rb1.mass_properties.inv_mass;
                constraint.im2 = rb2.mass_properties.inv_mass;
                constraint.limit = manifold.friction;
                constraint.mj_lambda1 = mj_lambda1;
                constraint.mj_lambda2 = mj_lambda2;
                constraint.manifold_id = manifold_id;
                constraint.manifold_contact_id = l * MAX_MANIFOLD_POINTS;
                constraint.num_contacts = manifold_points.len() as u8;
            }

            for (k, manifold_point) in manifold_points.iter().enumerate() {
                let dp1 = (pos_coll1 * manifold_point.local_p1) - rb1.world_com;
                let dp2 = (pos_coll2 * manifold_point.local_p2) - rb2.world_com;

                let vel1 = rb1.linvel + rb1.angvel.gcross(dp1);
                let vel2 = rb2.linvel + rb2.angvel.gcross(dp2);

                // Normal part.
                {
                    let gcross1 = rb1
                        .world_inv_inertia_sqrt
                        .transform_vector(dp1.gcross(force_dir1));
                    let gcross2 = rb2
                        .world_inv_inertia_sqrt
                        .transform_vector(dp2.gcross(-force_dir1));

                    let r = 1.0
                        / (rb1.mass_properties.inv_mass
                            + rb2.mass_properties.inv_mass
                            + gcross1.gdot(gcross1)
                            + gcross2.gdot(gcross2));

                    let rhs = if is_bouncing {
                        manifold.restitution * (vel1 - vel2).dot(&force_dir1)
                    } else {
                        (vel1 - vel2).dot(&force_dir1) + manifold_point.dist.max(0.0) * inv_dt
                    };

                    let impulse = manifold_point.impulse * warmstart_coeff;

                    constraint.elements[k].normal_part = VelocityConstraintElementPart {
                        gcross1,
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
                        let gcross1 = rb1
                            .world_inv_inertia_sqrt
                            .transform_vector(dp1.gcross(tangents1[j]));
                        let gcross2 = rb2
                            .world_inv_inertia_sqrt
                            .transform_vector(dp2.gcross(-tangents1[j]));
                        let r = 1.0
                            / (rb1.mass_properties.inv_mass
                                + rb2.mass_properties.inv_mass
                                + gcross1.gdot(gcross1)
                                + gcross2.gdot(gcross2));
                        let rhs = (vel1 - vel2).dot(&tangents1[j]);
                        #[cfg(feature = "dim2")]
                        let impulse = manifold_point.tangent_impulse * warmstart_coeff;
                        #[cfg(feature = "dim3")]
                        let impulse = manifold_point.tangent_impulse[j] * warmstart_coeff;

                        constraint.elements[k].tangent_part[j] = VelocityConstraintElementPart {
                            gcross1,
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
                out_constraints.push(AnyVelocityConstraint::Nongrouped(constraint));
            } else {
                out_constraints[manifold.constraint_index + l] =
                    AnyVelocityConstraint::Nongrouped(constraint);
            }
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = DeltaVel::zero();
        let mut mj_lambda2 = DeltaVel::zero();

        for i in 0..self.num_contacts as usize {
            let elt = &self.elements[i].normal_part;
            mj_lambda1.linear += self.dir1 * (self.im1 * elt.impulse);
            mj_lambda1.angular += elt.gcross1 * elt.impulse;

            mj_lambda2.linear += self.dir1 * (-self.im2 * elt.impulse);
            mj_lambda2.angular += elt.gcross2 * elt.impulse;

            // FIXME: move this out of the for loop?
            let tangents1 = self.dir1.orthonormal_basis();

            for j in 0..DIM - 1 {
                let elt = &self.elements[i].tangent_part[j];
                mj_lambda1.linear += tangents1[j] * (self.im1 * elt.impulse);
                mj_lambda1.angular += elt.gcross1 * elt.impulse;

                mj_lambda2.linear += tangents1[j] * (-self.im2 * elt.impulse);
                mj_lambda2.angular += elt.gcross2 * elt.impulse;
            }
        }

        mj_lambdas[self.mj_lambda1 as usize].linear += mj_lambda1.linear;
        mj_lambdas[self.mj_lambda1 as usize].angular += mj_lambda1.angular;
        mj_lambdas[self.mj_lambda2 as usize].linear += mj_lambda2.linear;
        mj_lambdas[self.mj_lambda2 as usize].angular += mj_lambda2.angular;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        // Solve friction.
        for i in 0..self.num_contacts as usize {
            let tangents1 = self.dir1.orthonormal_basis();

            for j in 0..DIM - 1 {
                let normal_elt = &self.elements[i].normal_part;
                let elt = &mut self.elements[i].tangent_part[j];
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

        // Solve non-penetration.
        for i in 0..self.num_contacts as usize {
            let elt = &mut self.elements[i].normal_part;
            let dimpulse = self.dir1.dot(&mj_lambda1.linear) + elt.gcross1.gdot(mj_lambda1.angular)
                - self.dir1.dot(&mj_lambda2.linear)
                + elt.gcross2.gdot(mj_lambda2.angular)
                + elt.rhs;
            let new_impulse = (elt.impulse - elt.r * dimpulse).max(0.0);
            let dlambda = new_impulse - elt.impulse;
            elt.impulse = new_impulse;

            mj_lambda1.linear += self.dir1 * (self.im1 * dlambda);
            mj_lambda1.angular += elt.gcross1 * dlambda;

            mj_lambda2.linear += self.dir1 * (-self.im2 * dlambda);
            mj_lambda2.angular += elt.gcross2 * dlambda;
        }

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

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
