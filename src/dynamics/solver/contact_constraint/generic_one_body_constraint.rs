use crate::dynamics::solver::OneBodyConstraint;
use crate::dynamics::{
    IntegrationParameters, MultibodyJointSet, MultibodyLinkId, RigidBodySet, RigidBodyVelocity,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{DIM, MAX_MANIFOLD_POINTS, Point, Real};
use crate::utils::SimdCross;

use super::{OneBodyConstraintElement, OneBodyConstraintNormalPart};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::{ContactPointInfos, OneBodyConstraintBuilder};
#[cfg(feature = "dim2")]
use crate::utils::SimdBasis;
use na::DVector;

#[derive(Copy, Clone)]
pub(crate) struct GenericOneBodyConstraintBuilder {
    link2: MultibodyLinkId,
    ccd_thickness: Real,
    inner: OneBodyConstraintBuilder,
}

impl GenericOneBodyConstraintBuilder {
    pub fn invalid() -> Self {
        Self {
            link2: MultibodyLinkId::default(),
            ccd_thickness: 0.0,
            inner: OneBodyConstraintBuilder::invalid(),
        }
    }

    pub fn generate(
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        out_builders: &mut [GenericOneBodyConstraintBuilder],
        out_constraints: &mut [GenericOneBodyConstraint],
        jacobians: &mut DVector<Real>,
        jacobian_id: &mut usize,
    ) {
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

        let rb1 = handle1
            .map(|h| SolverBody::from(&bodies[h]))
            .unwrap_or_default();

        let rb2 = &bodies[handle2.unwrap()];
        let (vels2, mprops2) = (&rb2.vels, &rb2.mprops);

        let link2 = *multibodies.rigid_body_link(handle2.unwrap()).unwrap();
        let (mb2, link_id2) = (&multibodies[link2.multibody], link2.id);
        let solver_vel2 = mb2.solver_id;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 =
            super::compute_tangent_contact_directions(&force_dir1, &vels1.linvel, &vels2.linvel);

        let multibodies_ndof = mb2.ndofs();
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

            builder.inner.rb1 = rb1;
            builder.inner.vels1 = vels1;

            constraint.inner.dir1 = force_dir1;
            constraint.inner.im2 = mprops2.effective_inv_mass;
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
                let dp1 = point - world_com1;
                let dp2 = point - mprops2.world_com;

                let vel1 = vels1.linvel + vels1.angvel.gcross(dp1);
                let vel2 = vels2.linvel + vels2.angvel.gcross(dp2);

                constraint.inner.limit = manifold_point.friction;
                constraint.inner.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
                let normal_rhs_wo_bias;
                {
                    let torque_dir2 = dp2.gcross(-force_dir1);
                    let inv_r2 = mb2
                        .fill_jacobians(
                            link_id2,
                            -force_dir1,
                            #[cfg(feature = "dim2")]
                            na::vector!(torque_dir2),
                            #[cfg(feature = "dim3")]
                            torque_dir2,
                            jacobian_id,
                            jacobians,
                        )
                        .0;

                    let r = crate::utils::inv(inv_r2);

                    let is_bouncy = manifold_point.is_bouncy() as u32 as Real;

                    let proj_vel1 = vel1.dot(&force_dir1);
                    let proj_vel2 = vel2.dot(&force_dir1);
                    let dvel = proj_vel1 - proj_vel2;
                    // NOTE: we add proj_vel1 since it’s not accessible through solver_vel.
                    normal_rhs_wo_bias =
                        proj_vel1 + (is_bouncy * manifold_point.restitution) * dvel;

                    constraint.inner.elements[k].normal_part = OneBodyConstraintNormalPart {
                        gcross2: na::zero(), // Unused for generic constraints.
                        rhs: na::zero(),
                        rhs_wo_bias: na::zero(),
                        impulse: na::zero(),
                        impulse_accumulator: na::zero(),
                        r,
                        r_mat_elts: [0.0; 2],
                    };
                }

                // Tangent parts.
                {
                    constraint.inner.elements[k].tangent_part.impulse = na::zero();

                    for j in 0..DIM - 1 {
                        let torque_dir2 = dp2.gcross(-tangents1[j]);
                        let inv_r2 = mb2
                            .fill_jacobians(
                                link_id2,
                                -tangents1[j],
                                #[cfg(feature = "dim2")]
                                na::vector![torque_dir2],
                                #[cfg(feature = "dim3")]
                                torque_dir2,
                                jacobian_id,
                                jacobians,
                            )
                            .0;

                        let r = crate::utils::inv(inv_r2);

                        let rhs_wo_bias = (vel1
                            + flipped_multiplier * manifold_point.tangent_velocity)
                            .dot(&tangents1[j]);

                        constraint.inner.elements[k].tangent_part.rhs_wo_bias[j] = rhs_wo_bias;
                        constraint.inner.elements[k].tangent_part.rhs[j] = rhs_wo_bias;

                        // FIXME: in 3D, we should take into account gcross[0].dot(gcross[1])
                        // in lhs. See the corresponding code on the `velocity_constraint.rs`
                        // file.
                        constraint.inner.elements[k].tangent_part.r[j] = r;
                    }
                }

                // Builder.
                let infos = ContactPointInfos {
                    local_p1: rb1.position.inverse_transform_point(&manifold_point.point),
                    local_p2: rb2
                        .pos
                        .position
                        .inverse_transform_point(&manifold_point.point),
                    tangent_vel: manifold_point.tangent_velocity,
                    dist: manifold_point.dist,
                    normal_rhs_wo_bias,
                };

                builder.link2 = link2;
                builder.ccd_thickness = rb2.ccd.ccd_thickness;
                builder.inner.infos[k] = infos;
                constraint.inner.manifold_contact_id[k] = manifold_point.contact_id;
            }

            constraint.j_id = chunk_j_id;
            constraint.ndofs2 = mb2.ndofs();
        }
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        _solver_bodies: &[SolverBody],
        multibodies: &MultibodyJointSet,
        constraint: &mut GenericOneBodyConstraint,
    ) {
        // We don’t update jacobians so the update is mostly identical to the non-generic velocity constraint.
        let pos2 = &multibodies[self.link2.multibody]
            .link(self.link2.id)
            .unwrap()
            .local_to_world;

        self.inner
            .update_with_positions(params, solved_dt, pos2, &mut constraint.inner);
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct GenericOneBodyConstraint {
    // We just build the generic constraint on top of the velocity constraint,
    // adding some information we can use in the generic case.
    pub inner: OneBodyConstraint,
    pub j_id: usize,
    pub ndofs2: usize,
}

impl GenericOneBodyConstraint {
    pub fn invalid() -> Self {
        Self {
            inner: OneBodyConstraint::invalid(),
            j_id: usize::MAX,
            ndofs2: usize::MAX,
        }
    }

    pub fn warmstart(
        &mut self,
        jacobians: &DVector<Real>,
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let solver_vel2 = self.inner.solver_vel2;

        let elements = &mut self.inner.elements[..self.inner.num_contacts as usize];
        OneBodyConstraintElement::generic_warmstart_group(
            elements,
            jacobians,
            self.ndofs2,
            self.j_id,
            solver_vel2,
            generic_solver_vels,
        );
    }

    #[profiling::function]
    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        generic_solver_vels: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let solver_vel2 = self.inner.solver_vel2;

        let elements = &mut self.inner.elements[..self.inner.num_contacts as usize];
        OneBodyConstraintElement::generic_solve_group(
            self.inner.cfm_factor,
            elements,
            jacobians,
            self.inner.limit,
            self.ndofs2,
            self.j_id,
            solver_vel2,
            generic_solver_vels,
            solve_restitution,
            solve_friction,
        );
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        self.inner.writeback_impulses(manifolds_all);
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.inner.remove_cfm_and_bias_from_rhs();
    }
}
