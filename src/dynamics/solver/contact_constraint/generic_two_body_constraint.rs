use crate::dynamics::solver::{GenericRhs, TwoBodyConstraint};
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{DIM, MAX_MANIFOLD_POINTS, Real};
use crate::utils::{SimdAngularInertia, SimdCross, SimdDot};

use super::{TwoBodyConstraintBuilder, TwoBodyConstraintElement, TwoBodyConstraintNormalPart};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::{ContactPointInfos, SolverVel};
use crate::prelude::RigidBodyHandle;
#[cfg(feature = "dim2")]
use crate::utils::SimdBasis;
use na::DVector;

#[derive(Copy, Clone)]
pub(crate) struct GenericTwoBodyConstraintBuilder {
    handle1: RigidBodyHandle,
    handle2: RigidBodyHandle,
    ccd_thickness: Real,
    inner: TwoBodyConstraintBuilder,
}

impl GenericTwoBodyConstraintBuilder {
    pub fn invalid() -> Self {
        Self {
            handle1: RigidBodyHandle::invalid(),
            handle2: RigidBodyHandle::invalid(),
            ccd_thickness: Real::MAX,
            inner: TwoBodyConstraintBuilder::invalid(),
        }
    }

    pub fn generate(
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        out_builders: &mut [GenericTwoBodyConstraintBuilder],
        out_constraints: &mut [GenericTwoBodyConstraint],
        jacobians: &mut DVector<Real>,
        jacobian_id: &mut usize,
    ) {
        let handle1 = manifold.data.rigid_body1.unwrap();
        let handle2 = manifold.data.rigid_body2.unwrap();

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let (vels1, mprops1, type1) = (&rb1.vels, &rb1.mprops, &rb1.body_type);
        let (vels2, mprops2, type2) = (&rb2.vels, &rb2.mprops, &rb2.body_type);

        let multibody1 = multibodies
            .rigid_body_link(handle1)
            .map(|m| (&multibodies[m.multibody], m.id));
        let multibody2 = multibodies
            .rigid_body_link(handle2)
            .map(|m| (&multibodies[m.multibody], m.id));
        let solver_vel1 = multibody1
            .map(|mb| mb.0.solver_id)
            .unwrap_or(if type1.is_dynamic() {
                rb1.ids.active_set_offset
            } else {
                0
            });
        let solver_vel2 = multibody2
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

        for (l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let chunk_j_id = *jacobian_id;

            let builder = &mut out_builders[l];
            let constraint = &mut out_constraints[l];
            constraint.inner.dir1 = force_dir1;
            constraint.inner.im1 = if type1.is_dynamic() {
                mprops1.effective_inv_mass
            } else {
                na::zero()
            };
            constraint.inner.im2 = if type2.is_dynamic() {
                mprops2.effective_inv_mass
            } else {
                na::zero()
            };
            constraint.inner.solver_vel1 = solver_vel1;
            constraint.inner.solver_vel2 = solver_vel2;
            constraint.inner.manifold_id = manifold_id;
            constraint.inner.num_contacts = manifold_points.len() as u8;
            #[cfg(feature = "dim3")]
            {
                constraint.inner.tangent1 = tangents1[0];
            }

            for k in 0..manifold_points.len() {
                let manifold_point = &manifold_points[k];
                let point = manifold_point.point;
                let dp1 = point - mprops1.world_com;
                let dp2 = point - mprops2.world_com;

                let vel1 = vels1.linvel + vels1.angvel.gcross(dp1);
                let vel2 = vels2.linvel + vels2.angvel.gcross(dp2);

                constraint.inner.limit = manifold_point.friction;
                constraint.inner.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
                let normal_rhs_wo_bias;
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

                    normal_rhs_wo_bias =
                        (is_bouncy * manifold_point.restitution) * (vel1 - vel2).dot(&force_dir1);

                    constraint.inner.elements[k].normal_part = TwoBodyConstraintNormalPart {
                        gcross1,
                        gcross2,
                        rhs: na::zero(),
                        rhs_wo_bias: na::zero(),
                        impulse_accumulator: na::zero(),
                        impulse: manifold_point.warmstart_impulse,
                        r,
                        r_mat_elts: [0.0; 2],
                    };
                }

                // Tangent parts.
                {
                    constraint.inner.elements[k].tangent_part.impulse =
                        manifold_point.warmstart_tangent_impulse;

                    for j in 0..DIM - 1 {
                        let torque_dir1 = dp1.gcross(tangents1[j]);
                        let gcross1 = if type1.is_dynamic() {
                            mprops1
                                .effective_world_inv_inertia_sqrt
                                .transform_vector(torque_dir1)
                        } else {
                            na::zero()
                        };
                        constraint.inner.elements[k].tangent_part.gcross1[j] = gcross1;

                        let torque_dir2 = dp2.gcross(-tangents1[j]);
                        let gcross2 = if type2.is_dynamic() {
                            mprops2
                                .effective_world_inv_inertia_sqrt
                                .transform_vector(torque_dir2)
                        } else {
                            na::zero()
                        };
                        constraint.inner.elements[k].tangent_part.gcross2[j] = gcross2;

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
                        let rhs_wo_bias = manifold_point.tangent_velocity.dot(&tangents1[j]);

                        constraint.inner.elements[k].tangent_part.rhs_wo_bias[j] = rhs_wo_bias;
                        constraint.inner.elements[k].tangent_part.rhs[j] = rhs_wo_bias;

                        // TODO: in 3D, we should take into account gcross[0].dot(gcross[1])
                        // in lhs. See the corresponding code on the `velocity_constraint.rs`
                        // file.
                        constraint.inner.elements[k].tangent_part.r[j] = r;
                    }
                }

                // Builder.
                let infos = ContactPointInfos {
                    local_p1: rb1
                        .pos
                        .position
                        .inverse_transform_point(&manifold_point.point),
                    local_p2: rb2
                        .pos
                        .position
                        .inverse_transform_point(&manifold_point.point),
                    tangent_vel: manifold_point.tangent_velocity,
                    dist: manifold_point.dist,
                    normal_rhs_wo_bias,
                };

                builder.handle1 = handle1;
                builder.handle2 = handle2;
                builder.ccd_thickness = rb1.ccd.ccd_thickness + rb2.ccd.ccd_thickness;
                builder.inner.infos[k] = infos;
                constraint.inner.manifold_contact_id[k] = manifold_point.contact_id;
            }

            let ndofs1 = multibody1.map(|mb| mb.0.ndofs()).unwrap_or(0);
            let ndofs2 = multibody2.map(|mb| mb.0.ndofs()).unwrap_or(0);

            // NOTE: we use the generic constraint for non-dynamic bodies because this will
            //       reduce all ops to nothing because its ndofs will be zero.
            let generic_constraint_mask = (multibody1.is_some() as u8)
                | ((multibody2.is_some() as u8) << 1)
                | (!type1.is_dynamic() as u8)
                | ((!type2.is_dynamic() as u8) << 1);

            constraint.j_id = chunk_j_id;
            constraint.ndofs1 = ndofs1;
            constraint.ndofs2 = ndofs2;
            constraint.generic_constraint_mask = generic_constraint_mask;
        }
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        bodies: &[SolverBody],
        multibodies: &MultibodyJointSet,
        constraint: &mut GenericTwoBodyConstraint,
    ) {
        // We don’t update jacobians so the update is mostly identical to the non-generic velocity constraint.
        let pos1 = multibodies
            .rigid_body_link(self.handle1)
            .map(|m| &multibodies[m.multibody].link(m.id).unwrap().local_to_world)
            .unwrap_or_else(|| &bodies[constraint.inner.solver_vel1].position);
        let pos2 = multibodies
            .rigid_body_link(self.handle2)
            .map(|m| &multibodies[m.multibody].link(m.id).unwrap().local_to_world)
            .unwrap_or_else(|| &bodies[constraint.inner.solver_vel2].position);

        self.inner
            .update_with_positions(params, solved_dt, pos1, pos2, &mut constraint.inner);
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct GenericTwoBodyConstraint {
    // We just build the generic constraint on top of the velocity constraint,
    // adding some information we can use in the generic case.
    pub inner: TwoBodyConstraint,
    pub j_id: usize,
    pub ndofs1: usize,
    pub ndofs2: usize,
    pub generic_constraint_mask: u8,
}

impl GenericTwoBodyConstraint {
    pub fn invalid() -> Self {
        Self {
            inner: TwoBodyConstraint::invalid(),
            j_id: usize::MAX,
            ndofs1: usize::MAX,
            ndofs2: usize::MAX,
            generic_constraint_mask: u8::MAX,
        }
    }

    pub fn warmstart(
        &mut self,
        jacobians: &DVector<Real>,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let mut solver_vel1 = if self.generic_constraint_mask & 0b01 == 0 {
            GenericRhs::SolverVel(solver_vels[self.inner.solver_vel1])
        } else {
            GenericRhs::GenericId(self.inner.solver_vel1)
        };

        let mut solver_vel2 = if self.generic_constraint_mask & 0b10 == 0 {
            GenericRhs::SolverVel(solver_vels[self.inner.solver_vel2])
        } else {
            GenericRhs::GenericId(self.inner.solver_vel2)
        };

        let elements = &mut self.inner.elements[..self.inner.num_contacts as usize];
        TwoBodyConstraintElement::generic_warmstart_group(
            elements,
            jacobians,
            &self.inner.dir1,
            #[cfg(feature = "dim3")]
            &self.inner.tangent1,
            &self.inner.im1,
            &self.inner.im2,
            self.ndofs1,
            self.ndofs2,
            self.j_id,
            &mut solver_vel1,
            &mut solver_vel2,
            generic_solver_vels,
        );

        if let GenericRhs::SolverVel(solver_vel1) = solver_vel1 {
            solver_vels[self.inner.solver_vel1] = solver_vel1;
        }

        if let GenericRhs::SolverVel(solver_vel2) = solver_vel2 {
            solver_vels[self.inner.solver_vel2] = solver_vel2;
        }
    }

    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let mut solver_vel1 = if self.generic_constraint_mask & 0b01 == 0 {
            GenericRhs::SolverVel(solver_vels[self.inner.solver_vel1])
        } else {
            GenericRhs::GenericId(self.inner.solver_vel1)
        };

        let mut solver_vel2 = if self.generic_constraint_mask & 0b10 == 0 {
            GenericRhs::SolverVel(solver_vels[self.inner.solver_vel2])
        } else {
            GenericRhs::GenericId(self.inner.solver_vel2)
        };

        let elements = &mut self.inner.elements[..self.inner.num_contacts as usize];
        TwoBodyConstraintElement::generic_solve_group(
            self.inner.cfm_factor,
            elements,
            jacobians,
            &self.inner.dir1,
            #[cfg(feature = "dim3")]
            &self.inner.tangent1,
            &self.inner.im1,
            &self.inner.im2,
            self.inner.limit,
            self.ndofs1,
            self.ndofs2,
            self.j_id,
            &mut solver_vel1,
            &mut solver_vel2,
            generic_solver_vels,
            solve_restitution,
            solve_friction,
        );

        if let GenericRhs::SolverVel(solver_vel1) = solver_vel1 {
            solver_vels[self.inner.solver_vel1] = solver_vel1;
        }

        if let GenericRhs::SolverVel(solver_vel2) = solver_vel2 {
            solver_vels[self.inner.solver_vel2] = solver_vel2;
        }
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        self.inner.writeback_impulses(manifolds_all);
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.inner.remove_cfm_and_bias_from_rhs();
    }
}
