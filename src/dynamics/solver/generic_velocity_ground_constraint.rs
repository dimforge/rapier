use crate::data::{BundleSet, ComponentSet};
use crate::dynamics::solver::VelocityGroundConstraint;
use crate::dynamics::{
    IntegrationParameters, MultibodyJointSet, RigidBodyIds, RigidBodyMassProps, RigidBodyType,
    RigidBodyVelocity,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{Point, Real, DIM, MAX_MANIFOLD_POINTS};
use crate::utils::WCross;

use super::{
    AnyVelocityConstraint, VelocityGroundConstraintElement, VelocityGroundConstraintNormalPart,
};
#[cfg(feature = "dim2")]
use crate::utils::WBasis;
use na::DVector;

#[derive(Copy, Clone, Debug)]
pub(crate) struct GenericVelocityGroundConstraint {
    // We just build the generic constraint on top of the velocity constraint,
    // adding some information we can use in the generic case.
    pub velocity_constraint: VelocityGroundConstraint,
    pub j_id: usize,
    pub ndofs2: usize,
}

impl GenericVelocityGroundConstraint {
    pub fn generate<Bodies>(
        params: &IntegrationParameters,
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &Bodies,
        multibodies: &MultibodyJointSet,
        out_constraints: &mut Vec<AnyVelocityConstraint>,
        jacobians: &mut DVector<Real>,
        jacobian_id: &mut usize,
        push: bool,
    ) where
        Bodies: ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyType>,
    {
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

        let (rb_vels1, world_com1) = if let Some(handle1) = handle1 {
            let (vels1, mprops1): (&RigidBodyVelocity, &RigidBodyMassProps) =
                bodies.index_bundle(handle1.0);
            (*vels1, mprops1.world_com)
        } else {
            (RigidBodyVelocity::zero(), Point::origin())
        };

        let (rb_vels2, rb_mprops2): (&RigidBodyVelocity, &RigidBodyMassProps) =
            bodies.index_bundle(handle2.unwrap().0);

        let (mb2, link_id2) = handle2
            .and_then(|h| multibodies.rigid_body_link(h))
            .map(|m| (&multibodies[m.multibody], m.id))
            .unwrap();
        let mj_lambda2 = mb2.solver_id;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = super::compute_tangent_contact_directions(
            &force_dir1,
            &rb_vels1.linvel,
            &rb_vels2.linvel,
        );

        let multibodies_ndof = mb2.ndofs();
        // For each solver contact we generate DIM constraints, and each constraints appends
        // the multibodies jacobian and weighted jacobians
        let required_jacobian_len =
            *jacobian_id + manifold.data.solver_contacts.len() * multibodies_ndof * 2 * DIM;

        if jacobians.nrows() < required_jacobian_len {
            jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
        }

        for (_l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let chunk_j_id = *jacobian_id;
            let mut constraint = VelocityGroundConstraint {
                dir1: force_dir1,
                #[cfg(feature = "dim3")]
                tangent1: tangents1[0],
                elements: [VelocityGroundConstraintElement::zero(); MAX_MANIFOLD_POINTS],
                im2: rb_mprops2.effective_inv_mass,
                limit: 0.0,
                mj_lambda2,
                manifold_id,
                manifold_contact_id: [0; MAX_MANIFOLD_POINTS],
                num_contacts: manifold_points.len() as u8,
            };

            for k in 0..manifold_points.len() {
                let manifold_point = &manifold_points[k];
                let dp1 = manifold_point.point - world_com1;
                let dp2 = manifold_point.point - rb_mprops2.world_com;

                let vel1 = rb_vels1.linvel + rb_vels1.angvel.gcross(dp1);
                let vel2 = rb_vels2.linvel + rb_vels2.angvel.gcross(dp2);

                constraint.limit = manifold_point.friction;
                constraint.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
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
                    let is_resting = 1.0 - is_bouncy;

                    let mut rhs_wo_bias = (1.0 + is_bouncy * manifold_point.restitution)
                        * (vel1 - vel2).dot(&force_dir1);
                    rhs_wo_bias += manifold_point.dist.max(0.0) * inv_dt;
                    rhs_wo_bias *= is_bouncy + is_resting;
                    let rhs_bias =
                        /* is_resting * */ erp_inv_dt * manifold_point.dist.min(0.0);

                    constraint.elements[k].normal_part = VelocityGroundConstraintNormalPart {
                        gcross2: na::zero(), // Unused for generic constraints.
                        rhs: rhs_wo_bias + rhs_bias,
                        rhs_wo_bias,
                        impulse: na::zero(),
                        r,
                    };
                }

                // Tangent parts.
                {
                    constraint.elements[k].tangent_part.impulse = na::zero();

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

                        let rhs = (vel1 - vel2
                            + flipped_multiplier * manifold_point.tangent_velocity)
                            .dot(&tangents1[j]);

                        constraint.elements[k].tangent_part.rhs[j] = rhs;

                        // FIXME: in 3D, we should take into account gcross[0].dot(gcross[1])
                        // in lhs. See the corresponding code on the `velocity_constraint.rs`
                        // file.
                        constraint.elements[k].tangent_part.r[j] = r;
                    }
                }
            }

            let constraint = GenericVelocityGroundConstraint {
                velocity_constraint: constraint,
                j_id: chunk_j_id,
                ndofs2: mb2.ndofs(),
            };

            if push {
                out_constraints.push(AnyVelocityConstraint::NongroupedGenericGround(constraint));
            } else {
                out_constraints[manifold.data.constraint_index + _l] =
                    AnyVelocityConstraint::NongroupedGenericGround(constraint);
            }
        }
    }

    pub fn solve(
        &mut self,
        cfm_factor: Real,
        jacobians: &DVector<Real>,
        generic_mj_lambdas: &mut DVector<Real>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let mj_lambda2 = self.velocity_constraint.mj_lambda2 as usize;

        let elements = &mut self.velocity_constraint.elements
            [..self.velocity_constraint.num_contacts as usize];
        VelocityGroundConstraintElement::generic_solve_group(
            cfm_factor,
            elements,
            jacobians,
            self.velocity_constraint.limit,
            self.ndofs2,
            self.j_id,
            mj_lambda2,
            generic_mj_lambdas,
            solve_restitution,
            solve_friction,
        );
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        self.velocity_constraint.writeback_impulses(manifolds_all);
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.velocity_constraint.remove_bias_from_rhs();
    }
}
