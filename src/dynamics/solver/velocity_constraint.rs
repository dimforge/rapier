use crate::dynamics::solver::{
    GenericVelocityConstraint, GenericVelocityGroundConstraint, VelocityGroundConstraint,
};
#[cfg(feature = "simd-is-enabled")]
use crate::dynamics::solver::{WVelocityConstraint, WVelocityGroundConstraint};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{Real, Vector, DIM, MAX_MANIFOLD_POINTS};
use crate::utils::{self, WAngularInertia, WBasis, WCross, WDot};
use na::DVector;

use super::{DeltaVel, VelocityConstraintElement, VelocityConstraintNormalPart};

//#[repr(align(64))]
#[derive(Copy, Clone, Debug)]
pub(crate) enum AnyVelocityConstraint {
    NongroupedGround(VelocityGroundConstraint),
    Nongrouped(VelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedGround(WVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    Grouped(WVelocityConstraint),
    NongroupedGenericGround(GenericVelocityGroundConstraint),
    NongroupedGeneric(GenericVelocityConstraint),
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

    pub fn remove_bias_from_rhs(&mut self) {
        match self {
            AnyVelocityConstraint::Nongrouped(c) => c.remove_cfm_and_bias_from_rhs(),
            AnyVelocityConstraint::NongroupedGround(c) => c.remove_cfm_and_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::Grouped(c) => c.remove_cfm_and_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::GroupedGround(c) => c.remove_cfm_and_bias_from_rhs(),
            AnyVelocityConstraint::NongroupedGeneric(c) => c.remove_cfm_and_bias_from_rhs(),
            AnyVelocityConstraint::NongroupedGenericGround(c) => c.remove_cfm_and_bias_from_rhs(),
            AnyVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        mj_lambdas: &mut [DeltaVel<Real>],
        generic_mj_lambdas: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        match self {
            AnyVelocityConstraint::NongroupedGround(c) => {
                c.solve(mj_lambdas, solve_restitution, solve_friction)
            }
            AnyVelocityConstraint::Nongrouped(c) => {
                c.solve(mj_lambdas, solve_restitution, solve_friction)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::GroupedGround(c) => {
                c.solve(mj_lambdas, solve_restitution, solve_friction)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyVelocityConstraint::Grouped(c) => {
                c.solve(mj_lambdas, solve_restitution, solve_friction)
            }
            AnyVelocityConstraint::NongroupedGeneric(c) => c.solve(
                jacobians,
                mj_lambdas,
                generic_mj_lambdas,
                solve_restitution,
                solve_friction,
            ),
            AnyVelocityConstraint::NongroupedGenericGround(c) => c.solve(
                jacobians,
                generic_mj_lambdas,
                solve_restitution,
                solve_friction,
            ),
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
            AnyVelocityConstraint::NongroupedGeneric(c) => c.writeback_impulses(manifold_all),
            AnyVelocityConstraint::NongroupedGenericGround(c) => c.writeback_impulses(manifold_all),
            AnyVelocityConstraint::Empty => unreachable!(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct VelocityConstraint {
    pub dir1: Vector<Real>, // Non-penetration force direction for the first body.
    #[cfg(feature = "dim3")]
    pub tangent1: Vector<Real>, // One of the friction force directions.
    pub im1: Vector<Real>,
    pub im2: Vector<Real>,
    pub cfm_factor: Real,
    pub limit: Real,
    pub mj_lambda1: usize,
    pub mj_lambda2: usize,
    pub manifold_id: ContactManifoldIndex,
    pub manifold_contact_id: [u8; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
    pub elements: [VelocityConstraintElement<Real>; MAX_MANIFOLD_POINTS],
}

impl VelocityConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints_and_jacobian_lines(manifold: &ContactManifold) -> (usize, usize) {
        let rest = manifold.data.solver_contacts.len() % MAX_MANIFOLD_POINTS != 0;
        (
            manifold.data.solver_contacts.len() / MAX_MANIFOLD_POINTS + rest as usize,
            manifold.data.solver_contacts.len() * DIM,
        )
    }

    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        insert_at: Option<usize>,
    ) {
        assert_eq!(manifold.data.relative_dominance, 0);

        let cfm_factor = params.cfm_factor();
        let inv_dt = params.inv_dt();
        let erp_inv_dt = params.erp_inv_dt();

        let handle1 = manifold.data.rigid_body1.unwrap();
        let handle2 = manifold.data.rigid_body2.unwrap();

        let rb1 = &bodies[handle1];
        let (vels1, mprops1) = (&rb1.vels, &rb1.mprops);
        let rb2 = &bodies[handle2];
        let (vels2, mprops2) = (&rb2.vels, &rb2.mprops);
        let ccd_thickness = rb1.ccd.ccd_thickness + rb2.ccd.ccd_thickness;

        let mj_lambda1 = rb1.ids.active_set_offset;
        let mj_lambda2 = rb2.ids.active_set_offset;
        let force_dir1 = -manifold.data.normal;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 =
            super::compute_tangent_contact_directions(&force_dir1, &vels1.linvel, &vels2.linvel);

        for (_l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let mut is_fast_contact = false;

            #[cfg(not(target_arch = "wasm32"))]
            let mut constraint = VelocityConstraint {
                dir1: force_dir1,
                #[cfg(feature = "dim3")]
                tangent1: tangents1[0],
                elements: [VelocityConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im1: mprops1.effective_inv_mass,
                im2: mprops2.effective_inv_mass,
                cfm_factor,
                limit: 0.0,
                mj_lambda1,
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
                unreachable!(); // We don't have parallelization on WASM yet, so this is unreachable.
            };

            #[cfg(target_arch = "wasm32")]
            {
                constraint.dir1 = force_dir1;
                #[cfg(feature = "dim3")]
                {
                    constraint.tangent1 = tangents1[0];
                }
                constraint.im1 = mprops1.effective_inv_mass;
                constraint.im2 = mprops2.effective_inv_mass;
                constraint.cfm_factor = cfm_factor;
                constraint.limit = 0.0;
                constraint.mj_lambda1 = mj_lambda1;
                constraint.mj_lambda2 = mj_lambda2;
                constraint.manifold_id = manifold_id;
                constraint.manifold_contact_id = [0; MAX_MANIFOLD_POINTS];
                constraint.num_contacts = manifold_points.len() as u8;
            }

            for k in 0..manifold_points.len() {
                let manifold_point = &manifold_points[k];
                let dp1 = manifold_point.point - mprops1.world_com;
                let dp2 = manifold_point.point - mprops2.world_com;

                let vel1 = vels1.linvel + vels1.angvel.gcross(dp1);
                let vel2 = vels2.linvel + vels2.angvel.gcross(dp2);

                constraint.limit = manifold_point.friction;
                constraint.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
                {
                    let gcross1 = mprops1
                        .effective_world_inv_inertia_sqrt
                        .transform_vector(dp1.gcross(force_dir1));
                    let gcross2 = mprops2
                        .effective_world_inv_inertia_sqrt
                        .transform_vector(dp2.gcross(-force_dir1));

                    let imsum = mprops1.effective_inv_mass + mprops2.effective_inv_mass;
                    let projected_mass = utils::inv(
                        force_dir1.dot(&imsum.component_mul(&force_dir1))
                            + gcross1.gdot(gcross1)
                            + gcross2.gdot(gcross2),
                    );

                    let is_bouncy = manifold_point.is_bouncy() as u32 as Real;
                    let is_resting = 1.0 - is_bouncy;

                    let mut rhs_wo_bias = (1.0 + is_bouncy * manifold_point.restitution)
                        * (vel1 - vel2).dot(&force_dir1);
                    rhs_wo_bias += manifold_point.dist.max(0.0) * inv_dt;
                    rhs_wo_bias *= is_bouncy + is_resting;
                    let rhs_bias = /* is_resting
                        * */  erp_inv_dt
                        * (manifold_point.dist + params.allowed_linear_error).clamp(-params.max_penetration_correction, 0.0);

                    let rhs = rhs_wo_bias + rhs_bias;
                    is_fast_contact = is_fast_contact || (-rhs * params.dt > ccd_thickness * 0.5);

                    constraint.elements[k].normal_part = VelocityConstraintNormalPart {
                        gcross1,
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
                        let gcross1 = mprops1
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dp1.gcross(tangents1[j]));
                        let gcross2 = mprops2
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dp2.gcross(-tangents1[j]));
                        let imsum = mprops1.effective_inv_mass + mprops2.effective_inv_mass;
                        let r = tangents1[j].dot(&imsum.component_mul(&tangents1[j]))
                            + gcross1.gdot(gcross1)
                            + gcross2.gdot(gcross2);
                        let rhs =
                            (vel1 - vel2 + manifold_point.tangent_velocity).dot(&tangents1[j]);

                        constraint.elements[k].tangent_part.gcross1[j] = gcross1;
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
                            * (constraint.elements[k].tangent_part.gcross1[0]
                                .gdot(constraint.elements[k].tangent_part.gcross1[1])
                                + constraint.elements[k].tangent_part.gcross2[0]
                                    .gdot(constraint.elements[k].tangent_part.gcross2[1]));
                    }
                }
            }

            constraint.cfm_factor = if is_fast_contact { 1.0 } else { cfm_factor };

            #[cfg(not(target_arch = "wasm32"))]
            if let Some(at) = insert_at {
                out_constraints[at + _l] = AnyVelocityConstraint::Nongrouped(constraint);
            } else {
                out_constraints.push(AnyVelocityConstraint::Nongrouped(constraint));
            }
        }
    }

    pub fn solve(
        &mut self,
        mj_lambdas: &mut [DeltaVel<Real>],
        solve_normal: bool,
        solve_friction: bool,
    ) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        VelocityConstraintElement::solve_group(
            self.cfm_factor,
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im1,
            &self.im2,
            self.limit,
            &mut mj_lambda1,
            &mut mj_lambda2,
            solve_normal,
            solve_friction,
        );

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

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

#[inline(always)]
#[cfg(feature = "dim3")]
pub(crate) fn compute_tangent_contact_directions<N>(
    force_dir1: &Vector<N>,
    linvel1: &Vector<N>,
    linvel2: &Vector<N>,
) -> [Vector<N>; DIM - 1]
where
    N: utils::WReal,
    Vector<N>: WBasis,
{
    use na::SimdValue;

    // Compute the tangent direction. Pick the direction of
    // the linear relative velocity, if it is not too small.
    // Otherwise use a fallback direction.
    let relative_linvel = linvel1 - linvel2;
    let mut tangent_relative_linvel =
        relative_linvel - force_dir1 * (force_dir1.dot(&relative_linvel));

    let tangent_linvel_norm = {
        let _disable_fe_except =
            crate::utils::DisableFloatingPointExceptionsFlags::disable_floating_point_exceptions();
        tangent_relative_linvel.normalize_mut()
    };

    const THRESHOLD: Real = 1.0e-4;
    let use_fallback = tangent_linvel_norm.simd_lt(N::splat(THRESHOLD));
    let tangent_fallback = force_dir1.orthonormal_vector();

    let tangent1 = tangent_fallback.select(use_fallback, tangent_relative_linvel);
    let bitangent1 = force_dir1.cross(&tangent1);

    [tangent1, bitangent1]
}
