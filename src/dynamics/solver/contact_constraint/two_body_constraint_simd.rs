use super::{TwoBodyConstraintElement, TwoBodyConstraintNormalPart};
use crate::dynamics::integration_parameters::BLOCK_SOLVER_ENABLED;
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::{ContactPointInfos, SolverVel};
use crate::dynamics::{
    IntegrationParameters, MultibodyJointSet, RigidBodyIds, RigidBodyMassProps, RigidBodySet,
    RigidBodyVelocity,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{
    AngVector, AngularInertia, DIM, Isometry, MAX_MANIFOLD_POINTS, Point, Real, SIMD_WIDTH,
    SimdReal, TangentImpulse, Vector,
};
#[cfg(feature = "dim2")]
use crate::utils::SimdBasis;
use crate::utils::{self, SimdAngularInertia, SimdCross, SimdDot};
use num::Zero;
use parry::utils::SdpMatrix2;
use simba::simd::{SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
pub(crate) struct TwoBodyConstraintBuilderSimd {
    infos: [super::ContactPointInfos<SimdReal>; MAX_MANIFOLD_POINTS],
}

impl TwoBodyConstraintBuilderSimd {
    pub fn generate(
        manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out_builders: &mut [TwoBodyConstraintBuilderSimd],
        out_constraints: &mut [TwoBodyConstraintSimd],
    ) {
        for ii in 0..SIMD_WIDTH {
            assert_eq!(manifolds[ii].data.relative_dominance, 0);
        }

        let handles1 = gather![|ii| manifolds[ii].data.rigid_body1.unwrap()];
        let handles2 = gather![|ii| manifolds[ii].data.rigid_body2.unwrap()];

        let vels1: [&RigidBodyVelocity; SIMD_WIDTH] = gather![|ii| &bodies[handles1[ii]].vels];
        let vels2: [&RigidBodyVelocity; SIMD_WIDTH] = gather![|ii| &bodies[handles2[ii]].vels];
        let ids1: [&RigidBodyIds; SIMD_WIDTH] = gather![|ii| &bodies[handles1[ii]].ids];
        let ids2: [&RigidBodyIds; SIMD_WIDTH] = gather![|ii| &bodies[handles2[ii]].ids];
        let mprops1: [&RigidBodyMassProps; SIMD_WIDTH] = gather![|ii| &bodies[handles1[ii]].mprops];
        let mprops2: [&RigidBodyMassProps; SIMD_WIDTH] = gather![|ii| &bodies[handles2[ii]].mprops];

        let poss1 = Isometry::from(gather![|ii| bodies[handles1[ii]].pos.position]);
        let poss2 = Isometry::from(gather![|ii| bodies[handles2[ii]].pos.position]);

        let world_com1 = Point::from(gather![|ii| mprops1[ii].world_com]);
        let im1 = Vector::from(gather![|ii| mprops1[ii].effective_inv_mass]);
        let ii1: AngularInertia<SimdReal> =
            AngularInertia::from(gather![|ii| mprops1[ii].effective_world_inv_inertia_sqrt]);

        let linvel1 = Vector::from(gather![|ii| vels1[ii].linvel]);
        let angvel1 = AngVector::<SimdReal>::from(gather![|ii| vels1[ii].angvel]);

        let world_com2 = Point::from(gather![|ii| mprops2[ii].world_com]);
        let im2 = Vector::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2: AngularInertia<SimdReal> =
            AngularInertia::from(gather![|ii| mprops2[ii].effective_world_inv_inertia_sqrt]);

        let linvel2 = Vector::from(gather![|ii| vels2[ii].linvel]);
        let angvel2 = AngVector::<SimdReal>::from(gather![|ii| vels2[ii].angvel]);

        let force_dir1 = -Vector::from(gather![|ii| manifolds[ii].data.normal]);

        let solver_vel1 = gather![|ii| ids1[ii].active_set_offset];
        let solver_vel2 = gather![|ii| ids2[ii].active_set_offset];

        let num_active_contacts = manifolds[0].data.num_active_contacts();

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = super::compute_tangent_contact_directions(&force_dir1, &linvel1, &linvel2);

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points =
                gather![|ii| &manifolds[ii].data.solver_contacts[l..num_active_contacts]];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let constraint = &mut out_constraints[l / MAX_MANIFOLD_POINTS];
            let builder = &mut out_builders[l / MAX_MANIFOLD_POINTS];

            constraint.dir1 = force_dir1;
            constraint.im1 = im1;
            constraint.im2 = im2;
            constraint.solver_vel1 = solver_vel1;
            constraint.solver_vel2 = solver_vel2;
            constraint.manifold_id = manifold_id;
            constraint.num_contacts = num_points as u8;
            #[cfg(feature = "dim3")]
            {
                constraint.tangent1 = tangents1[0];
            }

            for k in 0..num_points {
                let friction = SimdReal::from(gather![|ii| manifold_points[ii][k].friction]);
                let restitution = SimdReal::from(gather![|ii| manifold_points[ii][k].restitution]);
                let is_bouncy = SimdReal::from(gather![
                    |ii| manifold_points[ii][k].is_bouncy() as u32 as Real
                ]);
                let warmstart_impulse =
                    SimdReal::from(gather![|ii| manifold_points[ii][k].warmstart_impulse]);
                let warmstart_tangent_impulse = TangentImpulse::from(gather![|ii| manifold_points
                    [ii][k]
                    .warmstart_tangent_impulse]);

                let dist = SimdReal::from(gather![|ii| manifold_points[ii][k].dist]);
                let point = Point::from(gather![|ii| manifold_points[ii][k].point]);

                let tangent_velocity =
                    Vector::from(gather![|ii| manifold_points[ii][k].tangent_velocity]);
                let dp1 = point - world_com1;
                let dp2 = point - world_com2;

                let vel1 = linvel1 + angvel1.gcross(dp1);
                let vel2 = linvel2 + angvel2.gcross(dp2);

                constraint.limit = friction;
                constraint.manifold_contact_id[k] = gather![|ii| manifold_points[ii][k].contact_id];

                // Normal part.
                let normal_rhs_wo_bias;
                {
                    let gcross1 = ii1.transform_vector(dp1.gcross(force_dir1));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-force_dir1));

                    let imsum = im1 + im2;
                    let projected_mass = utils::simd_inv(
                        force_dir1.dot(&imsum.component_mul(&force_dir1))
                            + gcross1.gdot(gcross1)
                            + gcross2.gdot(gcross2),
                    );

                    let projected_velocity = (vel1 - vel2).dot(&force_dir1);
                    normal_rhs_wo_bias = is_bouncy * restitution * projected_velocity;

                    constraint.elements[k].normal_part = TwoBodyConstraintNormalPart {
                        gcross1,
                        gcross2,
                        rhs: na::zero(),
                        rhs_wo_bias: na::zero(),
                        impulse: warmstart_impulse,
                        impulse_accumulator: SimdReal::splat(0.0),
                        r: projected_mass,
                        r_mat_elts: [SimdReal::zero(); 2],
                    };
                }

                // tangent parts.
                constraint.elements[k].tangent_part.impulse = warmstart_tangent_impulse;

                for j in 0..DIM - 1 {
                    let gcross1 = ii1.transform_vector(dp1.gcross(tangents1[j]));
                    let gcross2 = ii2.transform_vector(dp2.gcross(-tangents1[j]));
                    let imsum = im1 + im2;
                    let r = tangents1[j].dot(&imsum.component_mul(&tangents1[j]))
                        + gcross1.gdot(gcross1)
                        + gcross2.gdot(gcross2);
                    let rhs_wo_bias = tangent_velocity.dot(&tangents1[j]);

                    constraint.elements[k].tangent_part.gcross1[j] = gcross1;
                    constraint.elements[k].tangent_part.gcross2[j] = gcross2;
                    constraint.elements[k].tangent_part.rhs_wo_bias[j] = rhs_wo_bias;
                    constraint.elements[k].tangent_part.rhs[j] = rhs_wo_bias;
                    constraint.elements[k].tangent_part.r[j] = if cfg!(feature = "dim2") {
                        utils::simd_inv(r)
                    } else {
                        r
                    };
                }

                #[cfg(feature = "dim3")]
                {
                    constraint.elements[k].tangent_part.r[2] = SimdReal::splat(2.0)
                        * (constraint.elements[k].tangent_part.gcross1[0]
                            .gdot(constraint.elements[k].tangent_part.gcross1[1])
                            + constraint.elements[k].tangent_part.gcross2[0]
                                .gdot(constraint.elements[k].tangent_part.gcross2[1]));
                }

                // Builder.
                let infos = ContactPointInfos {
                    local_p1: poss1.inverse_transform_point(&point),
                    local_p2: poss2.inverse_transform_point(&point),
                    tangent_vel: tangent_velocity,
                    dist,
                    normal_rhs_wo_bias,
                };

                builder.infos[k] = infos;
            }

            if BLOCK_SOLVER_ENABLED {
                // Coupling between consecutive pairs.
                for k in 0..num_points / 2 {
                    let k0 = k * 2;
                    let k1 = k * 2 + 1;

                    let imsum = im1 + im2;
                    let r0 = constraint.elements[k0].normal_part.r;
                    let r1 = constraint.elements[k1].normal_part.r;

                    let mut r_mat = SdpMatrix2::zero();
                    r_mat.m12 = force_dir1.dot(&imsum.component_mul(&force_dir1))
                        + constraint.elements[k0]
                            .normal_part
                            .gcross1
                            .gdot(constraint.elements[k1].normal_part.gcross1)
                        + constraint.elements[k0]
                            .normal_part
                            .gcross2
                            .gdot(constraint.elements[k1].normal_part.gcross2);
                    r_mat.m11 = utils::simd_inv(r0);
                    r_mat.m22 = utils::simd_inv(r1);

                    let (inv, det) = {
                        let _disable_fe_except =
                            crate::utils::DisableFloatingPointExceptionsFlags::
                            disable_floating_point_exceptions();
                        r_mat.inverse_and_get_determinant_unchecked()
                    };
                    let is_invertible = det.simd_gt(SimdReal::zero());

                    // If inversion failed, the contacts are redundant.
                    // Ignore the one with the smallest depth (it is too late to
                    // have the constraint removed from the constraint set, so just
                    // set the mass (r) matrix elements to 0.
                    constraint.elements[k0].normal_part.r_mat_elts = [
                        inv.m11.select(is_invertible, r0),
                        inv.m22.select(is_invertible, SimdReal::zero()),
                    ];
                    constraint.elements[k1].normal_part.r_mat_elts = [
                        inv.m12.select(is_invertible, SimdReal::zero()),
                        r_mat.m12.select(is_invertible, SimdReal::zero()),
                    ];
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
        constraint: &mut TwoBodyConstraintSimd,
    ) {
        let cfm_factor = SimdReal::splat(params.contact_cfm_factor());
        let inv_dt = SimdReal::splat(params.inv_dt());
        let allowed_lin_err = SimdReal::splat(params.allowed_linear_error());
        let erp_inv_dt = SimdReal::splat(params.contact_erp_inv_dt());
        let max_corrective_velocity = SimdReal::splat(params.max_corrective_velocity());
        let warmstart_coeff = SimdReal::splat(params.warmstart_coefficient);

        let rb1 = gather![|ii| &bodies[constraint.solver_vel1[ii]]];
        let rb2 = gather![|ii| &bodies[constraint.solver_vel2[ii]]];

        let poss1 = Isometry::from(gather![|ii| rb1[ii].position]);
        let poss2 = Isometry::from(gather![|ii| rb2[ii].position]);

        let all_infos = &self.infos[..constraint.num_contacts as usize];
        let all_elements = &mut constraint.elements[..constraint.num_contacts as usize];

        #[cfg(feature = "dim2")]
        let tangents1 = constraint.dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = [
            constraint.tangent1,
            constraint.dir1.cross(&constraint.tangent1),
        ];

        let solved_dt = SimdReal::splat(solved_dt);

        for (info, element) in all_infos.iter().zip(all_elements.iter_mut()) {
            // NOTE: the tangent velocity is equivalent to an additional movement of the first body’s surface.
            let p1 = poss1 * info.local_p1 + info.tangent_vel * solved_dt;
            let p2 = poss2 * info.local_p2;
            let dist = info.dist + (p1 - p2).dot(&constraint.dir1);

            // Normal part.
            {
                let rhs_wo_bias =
                    info.normal_rhs_wo_bias + dist.simd_max(SimdReal::zero()) * inv_dt;
                let rhs_bias = ((dist + allowed_lin_err) * erp_inv_dt)
                    .simd_clamp(-max_corrective_velocity, SimdReal::zero());
                let new_rhs = rhs_wo_bias + rhs_bias;

                element.normal_part.rhs_wo_bias = rhs_wo_bias;
                element.normal_part.rhs = new_rhs;
                element.normal_part.impulse_accumulator += element.normal_part.impulse;
                element.normal_part.impulse *= warmstart_coeff;
            }

            // tangent parts.
            {
                element.tangent_part.impulse_accumulator += element.tangent_part.impulse;
                element.tangent_part.impulse *= warmstart_coeff;

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
pub(crate) struct TwoBodyConstraintSimd {
    pub dir1: Vector<SimdReal>, // Non-penetration force direction for the first body.
    #[cfg(feature = "dim3")]
    pub tangent1: Vector<SimdReal>, // One of the friction force directions.
    pub elements: [TwoBodyConstraintElement<SimdReal>; MAX_MANIFOLD_POINTS],
    pub num_contacts: u8,
    pub im1: Vector<SimdReal>,
    pub im2: Vector<SimdReal>,
    pub cfm_factor: SimdReal,
    pub limit: SimdReal,
    pub solver_vel1: [usize; SIMD_WIDTH],
    pub solver_vel2: [usize; SIMD_WIDTH],
    pub manifold_id: [ContactManifoldIndex; SIMD_WIDTH],
    pub manifold_contact_id: [[u8; SIMD_WIDTH]; MAX_MANIFOLD_POINTS],
}

impl TwoBodyConstraintSimd {
    pub fn warmstart(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel1 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].angular]),
        };

        let mut solver_vel2 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].angular]),
        };

        TwoBodyConstraintElement::warmstart_group(
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im1,
            &self.im2,
            &mut solver_vel1,
            &mut solver_vel2,
        );

        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel1[ii]].linear = solver_vel1.linear.extract(ii);
            solver_vels[self.solver_vel1[ii]].angular = solver_vel1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel2[ii]].linear = solver_vel2.linear.extract(ii);
            solver_vels[self.solver_vel2[ii]].angular = solver_vel2.angular.extract(ii);
        }
    }

    pub fn solve(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        solve_normal: bool,
        solve_friction: bool,
    ) {
        let mut solver_vel1 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].angular]),
        };

        let mut solver_vel2 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].angular]),
        };

        TwoBodyConstraintElement::solve_group(
            self.cfm_factor,
            &mut self.elements[..self.num_contacts as usize],
            &self.dir1,
            #[cfg(feature = "dim3")]
            &self.tangent1,
            &self.im1,
            &self.im2,
            self.limit,
            &mut solver_vel1,
            &mut solver_vel2,
            solve_normal,
            solve_friction,
        );

        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel1[ii]].linear = solver_vel1.linear.extract(ii);
            solver_vels[self.solver_vel1[ii]].angular = solver_vel1.angular.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel2[ii]].linear = solver_vel2.linear.extract(ii);
            solver_vels[self.solver_vel2[ii]].angular = solver_vel2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, manifolds_all: &mut [&mut ContactManifold]) {
        for k in 0..self.num_contacts as usize {
            let warmstart_impulses: [_; SIMD_WIDTH] = self.elements[k].normal_part.impulse.into();
            let warmstart_tangent_impulses = self.elements[k].tangent_part.impulse;
            let impulses: [_; SIMD_WIDTH] = self.elements[k].normal_part.total_impulse().into();
            let tangent_impulses = self.elements[k].tangent_part.total_impulse();

            for ii in 0..SIMD_WIDTH {
                let manifold = &mut manifolds_all[self.manifold_id[ii]];
                let contact_id = self.manifold_contact_id[k][ii];
                let active_contact = &mut manifold.points[contact_id as usize];
                active_contact.data.warmstart_impulse = warmstart_impulses[ii];
                active_contact.data.warmstart_tangent_impulse =
                    warmstart_tangent_impulses.extract(ii);
                active_contact.data.impulse = impulses[ii];
                active_contact.data.tangent_impulse = tangent_impulses.extract(ii);
            }
        }
    }

    pub fn remove_cfm_and_bias_from_rhs(&mut self) {
        self.cfm_factor = SimdReal::splat(1.0);
        for elt in &mut self.elements {
            elt.normal_part.rhs = elt.normal_part.rhs_wo_bias;
            elt.tangent_part.rhs = elt.tangent_part.rhs_wo_bias;
        }
    }
}
