use super::{OneBodyConstraintElement, OneBodyConstraintNormalPart};
use crate::math::{DIM, MAX_MANIFOLD_POINTS, Point, Real, Vector};
#[cfg(feature = "dim2")]
use crate::utils::SimdBasis;
use crate::utils::{self, SimdAngularInertia, SimdCross, SimdDot, SimdRealCopy};
use na::Matrix2;
use parry::math::Isometry;

use crate::dynamics::integration_parameters::BLOCK_SOLVER_ENABLED;
use crate::dynamics::solver::SolverVel;
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodySet, RigidBodyVelocity};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

// TODO: move this struct somewhere else.
#[derive(Copy, Clone, Debug)]
pub struct ContactPointInfos<N: SimdRealCopy> {
    pub tangent_vel: Vector<N>,
    pub local_p1: Point<N>,
    pub local_p2: Point<N>,
    pub dist: N,
    pub normal_rhs_wo_bias: N,
}

impl<N: SimdRealCopy> Default for ContactPointInfos<N> {
    fn default() -> Self {
        Self {
            tangent_vel: Vector::zeros(),
            local_p1: Point::origin(),
            local_p2: Point::origin(),
            dist: N::zero(),
            normal_rhs_wo_bias: N::zero(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct OneBodyConstraintBuilder {
    // PERF: only store what’s necessary for the bias updates instead of the complete solver body.
    pub rb1: SolverBody,
    pub vels1: RigidBodyVelocity,
    pub infos: [ContactPointInfos<Real>; MAX_MANIFOLD_POINTS],
}

impl OneBodyConstraintBuilder {
    pub fn invalid() -> Self {
        Self {
            rb1: SolverBody::default(),
            vels1: RigidBodyVelocity::zero(),
            infos: [ContactPointInfos::default(); MAX_MANIFOLD_POINTS],
        }
    }

    pub fn generate(
        manifold_id: ContactManifoldIndex,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_builders: &mut [OneBodyConstraintBuilder],
        out_constraints: &mut [OneBodyConstraint],
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
        let vels2 = &rb2.vels;
        let mprops2 = &rb2.mprops;

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 =
            super::compute_tangent_contact_directions(&force_dir1, &vels1.linvel, &vels2.linvel);

        let solver_vel2 = rb2.ids.active_set_offset;

        for (l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let builder = &mut out_builders[l];
            let constraint = &mut out_constraints[l];

            builder.rb1 = rb1;
            builder.vels1 = vels1;

            constraint.dir1 = force_dir1;
            constraint.im2 = mprops2.effective_inv_mass;
            constraint.solver_vel2 = solver_vel2;
            constraint.manifold_id = manifold_id;
            constraint.num_contacts = manifold_points.len() as u8;
            #[cfg(feature = "dim3")]
            {
                constraint.tangent1 = tangents1[0];
            }

            for k in 0..manifold_points.len() {
                let manifold_point = &manifold_points[k];

                let dp2 = manifold_point.point - mprops2.world_com;
                let dp1 = manifold_point.point - world_com1;
                let vel1 = vels1.linvel + vels1.angvel.gcross(dp1);
                let vel2 = vels2.linvel + vels2.angvel.gcross(dp2);

                constraint.limit = manifold_point.friction;
                constraint.manifold_contact_id[k] = manifold_point.contact_id;

                // Normal part.
                let normal_rhs_wo_bias;
                {
                    let gcross2 = mprops2
                        .effective_world_inv_inertia_sqrt
                        .transform_vector(dp2.gcross(-force_dir1));

                    let projected_lin_mass =
                        force_dir1.dot(&mprops2.effective_inv_mass.component_mul(&force_dir1));
                    let projected_ang_mass = gcross2.gdot(gcross2);

                    let projected_mass = utils::inv(projected_lin_mass + projected_ang_mass);

                    let is_bouncy = manifold_point.is_bouncy() as u32 as Real;

                    let proj_vel1 = vel1.dot(&force_dir1);
                    let proj_vel2 = vel2.dot(&force_dir1);
                    let dvel = proj_vel1 - proj_vel2;
                    // NOTE: we add proj_vel1 since it’s not accessible through solver_vel.
                    normal_rhs_wo_bias =
                        proj_vel1 + (is_bouncy * manifold_point.restitution) * dvel;

                    constraint.elements[k].normal_part = OneBodyConstraintNormalPart {
                        gcross2,
                        rhs: na::zero(),
                        rhs_wo_bias: na::zero(),
                        impulse: manifold_point.warmstart_impulse,
                        impulse_accumulator: na::zero(),
                        r: projected_mass,
                        r_mat_elts: [0.0; 2],
                    };
                }

                // Tangent parts.
                {
                    constraint.elements[k].tangent_part.impulse =
                        manifold_point.warmstart_tangent_impulse;

                    for j in 0..DIM - 1 {
                        let gcross2 = mprops2
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dp2.gcross(-tangents1[j]));
                        let r = tangents1[j]
                            .dot(&mprops2.effective_inv_mass.component_mul(&tangents1[j]))
                            + gcross2.gdot(gcross2);
                        let rhs_wo_bias = (vel1
                            + flipped_multiplier * manifold_point.tangent_velocity)
                            .dot(&tangents1[j]);

                        constraint.elements[k].tangent_part.gcross2[j] = gcross2;
                        constraint.elements[k].tangent_part.rhs_wo_bias[j] = rhs_wo_bias;
                        constraint.elements[k].tangent_part.rhs[j] = rhs_wo_bias;
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

                // Builder.
                {
                    let local_p1 = rb1.position.inverse_transform_point(&manifold_point.point);
                    let local_p2 = rb2
                        .pos
                        .position
                        .inverse_transform_point(&manifold_point.point);
                    let infos = ContactPointInfos {
                        local_p1,
                        local_p2,
                        tangent_vel: flipped_multiplier * manifold_point.tangent_velocity,
                        dist: manifold_point.dist,
                        normal_rhs_wo_bias,
                    };

                    builder.infos[k] = infos;
                }
            }

            if BLOCK_SOLVER_ENABLED {
                // Coupling between consecutive pairs.
                for k in 0..manifold_points.len() / 2 {
                    let k0 = k * 2;
                    let k1 = k * 2 + 1;

                    let mut r_mat = Matrix2::zeros();
                    let r0 = constraint.elements[k0].normal_part.r;
                    let r1 = constraint.elements[k1].normal_part.r;
                    r_mat.m12 = force_dir1
                        .dot(&mprops2.effective_inv_mass.component_mul(&force_dir1))
                        + constraint.elements[k0]
                            .normal_part
                            .gcross2
                            .gdot(constraint.elements[k1].normal_part.gcross2);
                    r_mat.m21 = r_mat.m12;
                    r_mat.m11 = utils::inv(r0);
                    r_mat.m22 = utils::inv(r1);

                    if let Some(inv) = r_mat.try_inverse() {
                        constraint.elements[k0].normal_part.r_mat_elts = [inv.m11, inv.m22];
                        constraint.elements[k1].normal_part.r_mat_elts = [inv.m12, r_mat.m12];
                    } else {
                        // If inversion failed, the contacts are redundant.
                        // Ignore the one with the smallest depth (it is too late to
                        // have the constraint removed from the constraint set, so just
                        // set the mass (r) matrix elements to 0.
                        constraint.elements[k0].normal_part.r_mat_elts =
                            if manifold_points[k0].dist <= manifold_points[k1].dist {
                                [r0, 0.0]
                            } else {
                                [0.0, r1]
                            };
                        constraint.elements[k1].normal_part.r_mat_elts = [0.0; 2];
                    }
                }
            }
        }
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        bodies: &[SolverBody],
        _multibodies: &MultibodyJointSet,
        constraint: &mut OneBodyConstraint,
    ) {
        let rb2 = &bodies[constraint.solver_vel2];
        self.update_with_positions(params, solved_dt, &rb2.position, constraint)
    }

    // TODO: this code is SOOOO similar to TwoBodyConstraint::update.
    //       In fact the only differences are types and the `rb1` and ignoring its ccd thickness.
    pub fn update_with_positions(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        rb2_pos: &Isometry<Real>,
        constraint: &mut OneBodyConstraint,
    ) {
        let cfm_factor = params.contact_cfm_factor();
        let inv_dt = params.inv_dt();
        let erp_inv_dt = params.contact_erp_inv_dt();

        let all_infos = &self.infos[..constraint.num_contacts as usize];
        let all_elements = &mut constraint.elements[..constraint.num_contacts as usize];
        let rb1 = &self.rb1;
        // Integrate the velocity of the static rigid-body, if it’s kinematic.
        let new_pos1 = self
            .vels1
            .integrate(solved_dt, &rb1.position, &rb1.local_com);

        #[cfg(feature = "dim2")]
        let tangents1 = constraint.dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = [
            constraint.tangent1,
            constraint.dir1.cross(&constraint.tangent1),
        ];

        for (info, element) in all_infos.iter().zip(all_elements.iter_mut()) {
            // NOTE: the tangent velocity is equivalent to an additional movement of the first body’s surface.
            let p1 = new_pos1 * info.local_p1 + info.tangent_vel * solved_dt;
            let p2 = rb2_pos * info.local_p2;
            let dist = info.dist + (p1 - p2).dot(&constraint.dir1);

            // Normal part.
            {
                let rhs_wo_bias = info.normal_rhs_wo_bias + dist.max(0.0) * inv_dt;
                let rhs_bias = (erp_inv_dt * (dist + params.allowed_linear_error()))
                    .clamp(-params.max_corrective_velocity(), 0.0);
                let new_rhs = rhs_wo_bias + rhs_bias;

                element.normal_part.rhs_wo_bias = rhs_wo_bias;
                element.normal_part.rhs = new_rhs;
                element.normal_part.impulse_accumulator += element.normal_part.impulse;
                element.normal_part.impulse *= params.warmstart_coefficient;
            }

            // Tangent part.
            {
                element.tangent_part.impulse_accumulator += element.tangent_part.impulse;
                element.tangent_part.impulse *= params.warmstart_coefficient;

                for j in 0..DIM - 1 {
                    let bias = (p1 - p2).dot(&tangents1[j]) * inv_dt;
                    element.tangent_part.rhs[j] = element.tangent_part.rhs_wo_bias[j] + bias;
                }
            }
        }

        constraint.cfm_factor = cfm_factor;
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct OneBodyConstraint {
    pub solver_vel2: usize,
    pub dir1: Vector<Real>, // Non-penetration force direction for the first body.
    #[cfg(feature = "dim3")]
    pub tangent1: Vector<Real>, // One of the friction force directions.
    pub im2: Vector<Real>,
    pub cfm_factor: Real,
    pub limit: Real,
    pub elements: [OneBodyConstraintElement<Real>; MAX_MANIFOLD_POINTS],

    pub manifold_id: ContactManifoldIndex,
    pub manifold_contact_id: [u8; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
}

impl OneBodyConstraint {
    pub fn invalid() -> Self {
        Self {
            solver_vel2: usize::MAX,
            dir1: Vector::zeros(),
            #[cfg(feature = "dim3")]
            tangent1: Vector::zeros(),
            im2: Vector::zeros(),
            cfm_factor: 0.0,
            limit: 0.0,
            elements: [OneBodyConstraintElement::zero(); MAX_MANIFOLD_POINTS],
            manifold_id: ContactManifoldIndex::MAX,
            manifold_contact_id: [u8::MAX; MAX_MANIFOLD_POINTS],
            num_contacts: u8::MAX,
        }
    }

    pub fn warmstart(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel2 = solver_vels[self.solver_vel2];

        OneBodyConstraintElement::warmstart_group(
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im2,
            &mut solver_vel2,
        );

        solver_vels[self.solver_vel2] = solver_vel2;
    }

    pub fn solve(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        solve_normal: bool,
        solve_friction: bool,
    ) {
        let mut solver_vel2 = solver_vels[self.solver_vel2];

        OneBodyConstraintElement::solve_group(
            self.cfm_factor,
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im2,
            self.limit,
            &mut solver_vel2,
            solve_normal,
            solve_friction,
        );

        solver_vels[self.solver_vel2] = solver_vel2;
    }

    // FIXME: duplicated code. This is exactly the same as in the two-body velocity constraint.
    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        let manifold = &mut manifolds_all[self.manifold_id];

        for k in 0..self.num_contacts as usize {
            let contact_id = self.manifold_contact_id[k];
            let active_contact = &mut manifold.points[contact_id as usize];

            active_contact.data.warmstart_impulse = self.elements[k].normal_part.impulse;
            active_contact.data.warmstart_tangent_impulse = self.elements[k].tangent_part.impulse;
            active_contact.data.impulse = self.elements[k].normal_part.total_impulse();
            active_contact.data.tangent_impulse = self.elements[k].tangent_part.total_impulse();
        }
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.cfm_factor = 1.0;
        for elt in &mut self.elements {
            elt.normal_part.rhs = elt.normal_part.rhs_wo_bias;
            elt.tangent_part.rhs = elt.tangent_part.rhs_wo_bias;
        }
    }
}
