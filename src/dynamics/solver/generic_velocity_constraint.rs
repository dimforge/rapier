use crate::dynamics::solver::{GenericRhs, VelocityConstraint};
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{Real, DIM, MAX_MANIFOLD_POINTS};
use crate::utils::{WAngularInertia, WCross, WDot};

use super::{
    AnyVelocityConstraint, DeltaVel, VelocityConstraintElement, VelocityConstraintNormalPart,
};
#[cfg(feature = "dim2")]
use crate::utils::WBasis;
use na::DVector;

#[derive(Copy, Clone, Debug)]
pub(crate) struct GenericVelocityConstraint {
    // We just build the generic constraint on top of the velocity constraint,
    // adding some information we can use in the generic case.
    pub velocity_constraint: VelocityConstraint,
    pub j_id: usize,
    pub ndofs1: usize,
    pub ndofs2: usize,
    pub generic_constraint_mask: u8,
}

impl GenericVelocityConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        jacobians: &mut DVector<Real>,
        jacobian_id: &mut usize,
        insert_at: Option<usize>,
    ) {
        let cfm_factor = params.cfm_factor();
        let inv_dt = params.inv_dt();
        let erp_inv_dt = params.erp_inv_dt();

        let handle1 = manifold.data.rigid_body1.unwrap();
        let handle2 = manifold.data.rigid_body2.unwrap();

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let (vels1, mprops1, type1) = (&rb1.vels, &rb1.mprops, &rb1.body_type);
        let (vels2, mprops2, type2) = (&rb2.vels, &rb2.mprops, &rb2.body_type);
        let ccd_thickness = rb1.ccd.ccd_thickness + rb2.ccd.ccd_thickness;

        let multibody1 = multibodies
            .rigid_body_link(handle1)
            .map(|m| (&multibodies[m.multibody], m.id));
        let multibody2 = multibodies
            .rigid_body_link(handle2)
            .map(|m| (&multibodies[m.multibody], m.id));
        let mj_lambda1 = multibody1
            .map(|mb| mb.0.solver_id)
            .unwrap_or(if type1.is_dynamic() {
                rb1.ids.active_set_offset
            } else {
                0
            });
        let mj_lambda2 = multibody2
            .map(|mb| mb.0.solver_id)
            .unwrap_or(if type2.is_dynamic() {
                rb2.ids.active_set_offset
            } else {
                0
            });
        let force_dir1 = -manifold.data.normal;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 =
            super::compute_tangent_contact_directions(&force_dir1, &vels1.linvel, &vels2.linvel);

        let multibodies_ndof = multibody1.map(|m| m.0.ndofs()).unwrap_or(0)
            + multibody2.map(|m| m.0.ndofs()).unwrap_or(0);
        // For each solver contact we generate DIM constraints, and each constraints appends
        // the multibodies jacobian and weighted jacobians
        let required_jacobian_len =
            *jacobian_id + manifold.data.solver_contacts.len() * multibodies_ndof * 2 * DIM;

        if jacobians.nrows() < required_jacobian_len && !cfg!(feature = "parallel") {
            jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
        }

        for (_l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let chunk_j_id = *jacobian_id;
            let mut is_fast_contact = false;
            let mut constraint = VelocityConstraint {
                dir1: force_dir1,
                #[cfg(feature = "dim3")]
                tangent1: tangents1[0],
                elements: [VelocityConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im1: if type1.is_dynamic() {
                    mprops1.effective_inv_mass
                } else {
                    na::zero()
                },
                im2: if type2.is_dynamic() {
                    mprops2.effective_inv_mass
                } else {
                    na::zero()
                },
                cfm_factor,
                limit: 0.0,
                mj_lambda1,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: [0; MAX_MANIFOLD_POINTS],
                num_contacts: manifold_points.len() as u8,
            };

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
                    let torque_dir1 = dp1.gcross(force_dir1);
                    let torque_dir2 = dp2.gcross(-force_dir1);

                    let gcross1 = if type1.is_dynamic() {
                        mprops1
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(torque_dir1)
                    } else {
                        na::zero()
                    };
                    let gcross2 = if type2.is_dynamic() {
                        mprops2
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(torque_dir2)
                    } else {
                        na::zero()
                    };

                    let inv_r1 = if let Some((mb1, link_id1)) = multibody1.as_ref() {
                        mb1.fill_jacobians(
                            *link_id1,
                            force_dir1,
                            #[cfg(feature = "dim2")]
                            na::vector!(torque_dir1),
                            #[cfg(feature = "dim3")]
                            torque_dir1,
                            jacobian_id,
                            jacobians,
                        )
                        .0
                    } else if type1.is_dynamic() {
                        force_dir1.dot(&mprops1.effective_inv_mass.component_mul(&force_dir1))
                            + gcross1.gdot(gcross1)
                    } else {
                        0.0
                    };

                    let inv_r2 = if let Some((mb2, link_id2)) = multibody2.as_ref() {
                        mb2.fill_jacobians(
                            *link_id2,
                            -force_dir1,
                            #[cfg(feature = "dim2")]
                            na::vector!(torque_dir2),
                            #[cfg(feature = "dim3")]
                            torque_dir2,
                            jacobian_id,
                            jacobians,
                        )
                        .0
                    } else if type2.is_dynamic() {
                        force_dir1.dot(&mprops2.effective_inv_mass.component_mul(&force_dir1))
                            + gcross2.gdot(gcross2)
                    } else {
                        0.0
                    };

                    let r = crate::utils::inv(inv_r1 + inv_r2);

                    let is_bouncy = manifold_point.is_bouncy() as u32 as Real;
                    let is_resting = 1.0 - is_bouncy;

                    let mut rhs_wo_bias = (1.0 + is_bouncy * manifold_point.restitution)
                        * (vel1 - vel2).dot(&force_dir1);
                    rhs_wo_bias += manifold_point.dist.max(0.0) * inv_dt;
                    rhs_wo_bias *= is_bouncy + is_resting;
                    let rhs_bias =
                        /* is_resting * */ erp_inv_dt * manifold_point.dist.clamp(-params.max_penetration_correction, 0.0);

                    let rhs = rhs_wo_bias + rhs_bias;
                    is_fast_contact = is_fast_contact || (-rhs * params.dt > ccd_thickness * 0.5);

                    constraint.elements[k].normal_part = VelocityConstraintNormalPart {
                        gcross1,
                        gcross2,
                        rhs,
                        rhs_wo_bias,
                        impulse: na::zero(),
                        r,
                    };
                }

                // Tangent parts.
                {
                    constraint.elements[k].tangent_part.impulse = na::zero();

                    for j in 0..DIM - 1 {
                        let torque_dir1 = dp1.gcross(tangents1[j]);
                        let gcross1 = if type1.is_dynamic() {
                            mprops1
                                .effective_world_inv_inertia_sqrt
                                .transform_vector(torque_dir1)
                        } else {
                            na::zero()
                        };
                        constraint.elements[k].tangent_part.gcross1[j] = gcross1;

                        let torque_dir2 = dp2.gcross(-tangents1[j]);
                        let gcross2 = if type2.is_dynamic() {
                            mprops2
                                .effective_world_inv_inertia_sqrt
                                .transform_vector(torque_dir2)
                        } else {
                            na::zero()
                        };
                        constraint.elements[k].tangent_part.gcross2[j] = gcross2;

                        let inv_r1 = if let Some((mb1, link_id1)) = multibody1.as_ref() {
                            mb1.fill_jacobians(
                                *link_id1,
                                tangents1[j],
                                #[cfg(feature = "dim2")]
                                na::vector![torque_dir1],
                                #[cfg(feature = "dim3")]
                                torque_dir1,
                                jacobian_id,
                                jacobians,
                            )
                            .0
                        } else if type1.is_dynamic() {
                            force_dir1.dot(&mprops1.effective_inv_mass.component_mul(&force_dir1))
                                + gcross1.gdot(gcross1)
                        } else {
                            0.0
                        };

                        let inv_r2 = if let Some((mb2, link_id2)) = multibody2.as_ref() {
                            mb2.fill_jacobians(
                                *link_id2,
                                -tangents1[j],
                                #[cfg(feature = "dim2")]
                                na::vector![torque_dir2],
                                #[cfg(feature = "dim3")]
                                torque_dir2,
                                jacobian_id,
                                jacobians,
                            )
                            .0
                        } else if type2.is_dynamic() {
                            force_dir1.dot(&mprops2.effective_inv_mass.component_mul(&force_dir1))
                                + gcross2.gdot(gcross2)
                        } else {
                            0.0
                        };

                        let r = crate::utils::inv(inv_r1 + inv_r2);

                        let rhs =
                            (vel1 - vel2 + manifold_point.tangent_velocity).dot(&tangents1[j]);

                        constraint.elements[k].tangent_part.rhs[j] = rhs;
                        // FIXME: in 3D, we should take into account gcross[0].dot(gcross[1])
                        // in lhs. See the corresponding code on the `velocity_constraint.rs`
                        // file.
                        constraint.elements[k].tangent_part.r[j] = r;
                    }
                }
            }

            constraint.cfm_factor = if is_fast_contact { 1.0 } else { cfm_factor };

            let ndofs1 = multibody1.map(|mb| mb.0.ndofs()).unwrap_or(0);
            let ndofs2 = multibody2.map(|mb| mb.0.ndofs()).unwrap_or(0);
            // NOTE: we use the generic constraint for non-dynamic bodies because this will
            //       reduce all ops to nothing because its ndofs will be zero.
            let generic_constraint_mask = (multibody1.is_some() as u8)
                | ((multibody2.is_some() as u8) << 1)
                | (!type1.is_dynamic() as u8)
                | ((!type2.is_dynamic() as u8) << 1);

            let constraint = GenericVelocityConstraint {
                velocity_constraint: constraint,
                j_id: chunk_j_id,
                ndofs1,
                ndofs2,
                generic_constraint_mask,
            };

            if let Some(at) = insert_at {
                out_constraints[at + _l] = AnyVelocityConstraint::NongroupedGeneric(constraint);
            } else {
                out_constraints.push(AnyVelocityConstraint::NongroupedGeneric(constraint));
            }
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
        let mut mj_lambda1 = if self.generic_constraint_mask & 0b01 == 0 {
            GenericRhs::DeltaVel(mj_lambdas[self.velocity_constraint.mj_lambda1 as usize])
        } else {
            GenericRhs::GenericId(self.velocity_constraint.mj_lambda1 as usize)
        };

        let mut mj_lambda2 = if self.generic_constraint_mask & 0b10 == 0 {
            GenericRhs::DeltaVel(mj_lambdas[self.velocity_constraint.mj_lambda2 as usize])
        } else {
            GenericRhs::GenericId(self.velocity_constraint.mj_lambda2 as usize)
        };

        let elements = &mut self.velocity_constraint.elements
            [..self.velocity_constraint.num_contacts as usize];
        VelocityConstraintElement::generic_solve_group(
            self.velocity_constraint.cfm_factor,
            elements,
            jacobians,
            &self.velocity_constraint.dir1,
            #[cfg(feature = "dim3")]
            &self.velocity_constraint.tangent1,
            &self.velocity_constraint.im1,
            &self.velocity_constraint.im2,
            self.velocity_constraint.limit,
            self.ndofs1,
            self.ndofs2,
            self.j_id,
            &mut mj_lambda1,
            &mut mj_lambda2,
            generic_mj_lambdas,
            solve_restitution,
            solve_friction,
        );

        if let GenericRhs::DeltaVel(mj_lambda1) = mj_lambda1 {
            mj_lambdas[self.velocity_constraint.mj_lambda1 as usize] = mj_lambda1;
        }

        if let GenericRhs::DeltaVel(mj_lambda2) = mj_lambda2 {
            mj_lambdas[self.velocity_constraint.mj_lambda2 as usize] = mj_lambda2;
        }
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        self.velocity_constraint.writeback_impulses(manifolds_all);
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.velocity_constraint.remove_cfm_and_bias_from_rhs();
    }
}
