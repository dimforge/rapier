use super::{
    AnyVelocityConstraint, DeltaVel, VelocityGroundConstraintElement,
    VelocityGroundConstraintNormalPart,
};
use crate::math::{Point, Real, Vector, DIM, MAX_MANIFOLD_POINTS};
#[cfg(feature = "dim2")]
use crate::utils::WBasis;
use crate::utils::{self, WAngularInertia, WCross, WDot};

use crate::dynamics::{IntegrationParameters, RigidBodySet, RigidBodyVelocity};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityGroundConstraint {
    pub mj_lambda2: usize,
    pub dir1: Vector<Real>, // Non-penetration force direction for the first body.
    #[cfg(feature = "dim3")]
    pub tangent1: Vector<Real>, // One of the friction force directions.
    pub im2: Vector<Real>,
    pub cfm_factor: Real,
    pub limit: Real,
    pub elements: [VelocityGroundConstraintElement<Real>; MAX_MANIFOLD_POINTS],

    pub manifold_id: ContactManifoldIndex,
    pub manifold_contact_id: [u8; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
}

impl VelocityGroundConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        insert_at: Option<usize>,
    ) {
        let cfm_factor = params.cfm_factor();
        let inv_dt = params.inv_dt();
        let erp_inv_dt = params.erp_inv_dt();

        let mut handle1 = manifold.data.rigid_body1;
        let mut handle2 = manifold.data.rigid_body2;
        let flipped = manifold.data.relative_dominance < 0;

        let (force_dir1, flipped_multiplier) = if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
            (manifold.data.normal, -1.0)
        } else {
            (-manifold.data.normal, 1.0)
        };

        let (vels1, world_com1) = if let Some(handle1) = handle1 {
            let rb1 = &bodies[handle1];
            (rb1.vels, rb1.mprops.world_com)
        } else {
            (RigidBodyVelocity::zero(), Point::origin())
        };

        let rb2 = &bodies[handle2.unwrap()];
        let vels2 = &rb2.vels;
        let mprops2 = &rb2.mprops;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 =
            super::compute_tangent_contact_directions(&force_dir1, &vels1.linvel, &vels2.linvel);

        let mj_lambda2 = rb2.ids.active_set_offset;

        for (_l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            #[cfg(not(target_arch = "wasm32"))]
            let mut constraint = VelocityGroundConstraint {
                dir1: force_dir1,
                #[cfg(feature = "dim3")]
                tangent1: tangents1[0],
                elements: [VelocityGroundConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im2: mprops2.effective_inv_mass,
                cfm_factor,
                limit: 0.0,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: [0; MAX_MANIFOLD_POINTS],
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
            // NOTE: impulse_joints have the same problem, but it is not easy to refactor the code that way
            // for the moment.
            #[cfg(target_arch = "wasm32")]
            let constraint = if insert_at.is_none() {
                let new_len = out_constraints.len() + 1;
                unsafe {
                    #[allow(invalid_value)]
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
                unreachable!(); // We don't have parallelization on WASM yet, so this is unreachable.
            };

            #[cfg(target_arch = "wasm32")]
            {
                constraint.dir1 = force_dir1;
                #[cfg(feature = "dim3")]
                {
                    constraint.tangent1 = tangents1[0];
                }
                constraint.im2 = mprops2.effective_inv_mass;
                constraint.cfm_factor = cfm_factor;
                constraint.limit = 0.0;
                constraint.mj_lambda2 = mj_lambda2;
                constraint.manifold_id = manifold_id;
                constraint.manifold_contact_id = [0; MAX_MANIFOLD_POINTS];
                constraint.num_contacts = manifold_points.len() as u8;
            }

            let mut is_fast_contact = false;

            for k in 0..manifold_points.len() {
                let manifold_point = &manifold_points[k];
                let dp2 = manifold_point.point - mprops2.world_com;
                let dp1 = manifold_point.point - world_com1;
                let vel1 = vels1.linvel + vels1.angvel.gcross(dp1);
                let vel2 = vels2.linvel + vels2.angvel.gcross(dp2);

                constraint.limit = manifold_point.friction;
                constraint.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
                {
                    let gcross2 = mprops2
                        .effective_world_inv_inertia_sqrt
                        .transform_vector(dp2.gcross(-force_dir1));

                    let projected_mass = utils::inv(
                        force_dir1.dot(&mprops2.effective_inv_mass.component_mul(&force_dir1))
                            + gcross2.gdot(gcross2),
                    );

                    let is_bouncy = manifold_point.is_bouncy() as u32 as Real;
                    let is_resting = 1.0 - is_bouncy;

                    let dvel = (vel1 - vel2).dot(&force_dir1);
                    let mut rhs_wo_bias = (1.0 + is_bouncy * manifold_point.restitution) * dvel;
                    rhs_wo_bias += manifold_point.dist.max(0.0) * inv_dt;
                    rhs_wo_bias *= is_bouncy + is_resting;
                    let rhs_bias = /* is_resting
                        * */ erp_inv_dt
                        * (manifold_point.dist + params.allowed_linear_error).clamp(-params.max_penetration_correction, 0.0);

                    let rhs = rhs_wo_bias + rhs_bias;
                    is_fast_contact =
                        is_fast_contact || (-rhs * params.dt > rb2.ccd.ccd_thickness * 0.5);

                    constraint.elements[k].normal_part = VelocityGroundConstraintNormalPart {
                        gcross2,
                        rhs,
                        rhs_wo_bias,
                        impulse: na::zero(),
                        r: projected_mass,
                    };
                }

                // Tangent parts.
                {
                    constraint.elements[k].tangent_part.impulse = na::zero();

                    for j in 0..DIM - 1 {
                        let gcross2 = mprops2
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dp2.gcross(-tangents1[j]));
                        let r = tangents1[j]
                            .dot(&mprops2.effective_inv_mass.component_mul(&tangents1[j]))
                            + gcross2.gdot(gcross2);
                        let rhs = (vel1 - vel2
                            + flipped_multiplier * manifold_point.tangent_velocity)
                            .dot(&tangents1[j]);

                        constraint.elements[k].tangent_part.gcross2[j] = gcross2;
                        constraint.elements[k].tangent_part.rhs[j] = rhs;
                        constraint.elements[k].tangent_part.r[j] = if cfg!(feature = "dim2") {
                            utils::inv(r)
                        } else {
                            r
                        };
                    }

                    #[cfg(feature = "dim3")]
                    {
                        constraint.elements[k].tangent_part.r[2] = 2.0
                            * constraint.elements[k].tangent_part.gcross2[0]
                                .gdot(constraint.elements[k].tangent_part.gcross2[1]);
                    }
                }
            }

            constraint.cfm_factor = if is_fast_contact { 1.0 } else { cfm_factor };

            #[cfg(not(target_arch = "wasm32"))]
            if let Some(at) = insert_at {
                out_constraints[at + _l] = AnyVelocityConstraint::NongroupedGround(constraint);
            } else {
                out_constraints.push(AnyVelocityConstraint::NongroupedGround(constraint));
            }
        }
    }

    pub fn solve(
        &mut self,
        mj_lambdas: &mut [DeltaVel<Real>],
        solve_normal: bool,
        solve_friction: bool,
    ) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        VelocityGroundConstraintElement::solve_group(
            self.cfm_factor,
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im2,
            self.limit,
            &mut mj_lambda2,
            solve_normal,
            solve_friction,
        );

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code. This is exactly the same as in the non-ground velocity constraint.
    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        let manifold = &mut manifolds_all[self.manifold_id];

        for k in 0..self.num_contacts as usize {
            let contact_id = self.manifold_contact_id[k];
            let active_contact = &mut manifold.points[contact_id as usize];
            active_contact.data.impulse = self.elements[k].normal_part.impulse;

            #[cfg(feature = "dim2")]
            {
                active_contact.data.tangent_impulse = self.elements[k].tangent_part.impulse[0];
            }
            #[cfg(feature = "dim3")]
            {
                active_contact.data.tangent_impulse = self.elements[k].tangent_part.impulse;
            }
        }
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.cfm_factor = 1.0;
        for elt in &mut self.elements {
            elt.normal_part.rhs = elt.normal_part.rhs_wo_bias;
        }
    }
}
