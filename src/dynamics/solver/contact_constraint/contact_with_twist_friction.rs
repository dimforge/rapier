use super::{
    ContactConstraintNormalPartSlim, ContactConstraintTangentPartSlim,
    ContactConstraintTwistPartSlim,
};
#[cfg(feature = "block-solver")]
use crate::dynamics::solver::contact_constraint::BLOCK_SOLVER_MIN_CONDITION;
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::solver::solver_contact_graph::ContactRef;
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodySet};
use crate::geometry::{ContactManifold, SimdSolverContact};
use crate::math::{DIM, MAX_MANIFOLD_POINTS, Real, SIMD_WIDTH, SimdReal, TangentImpulse};
use crate::utils::{self, AngularInertiaOps, CrossProduct, DotProduct, ScalarType, SimdLength};
use num::Zero;
use simba::simd::{SimdBool, SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
pub struct TwistContactPointInfos<N: ScalarType> {
    // This is different from the Coulomb version because it doesn’t
    // have the `tangent_vel` per-contact here.
    /// See [`super::contact_with_coulomb_friction::CoulombContactPointInfos::restitution_seed`].
    pub restitution_seed: N,
    pub local_p1: N::Vector,
    pub local_p2: N::Vector,
    pub dist: N,
}

impl<N: ScalarType> Default for TwistContactPointInfos<N> {
    fn default() -> Self {
        Self {
            restitution_seed: N::zero(),
            local_p1: Default::default(),
            local_p2: Default::default(),
            dist: N::zero(),
        }
    }
}

/*
 * FIXME: this involves a lot of duplicate code wrt. the contact with coulomb friction.
 *        Find a way to refactor so we can at least share the code for the ristution part.
 */
#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactWithTwistFrictionBuilder<N: ScalarType> {
    infos: [TwistContactPointInfos<N>; MAX_MANIFOLD_POINTS],
    local_friction_center1: N::Vector,
    local_friction_center2: N::Vector,
    tangent_vel: N::Vector,
    /// The contact normal in the first body's (com-centered) local frame, so
    /// `refresh` can re-derive the world normal without touching the manifold.
    local_n1: N::Vector,
    /// The pair's restitution coefficient (needed by `refresh` to recompute the
    /// restitution rhs seed).
    restitution: N,
}

impl ContactWithTwistFrictionBuilder<SimdReal> {
    pub fn generate(
        manifold_id: [ContactRef; SIMD_WIDTH],
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        solver_bodies: &SolverBodies,
        out_builder: &mut ContactWithTwistFrictionBuilder<SimdReal>,
        out_constraint: &mut ContactWithTwistFriction<SimdReal>,
    ) {
        // The solver-body ids were stamped on the manifolds by the narrow-phase's
        // solver-graph maintenance (`u32::MAX` for world-attached sides: fixed bodies, or a
        // frontier pair's sleeping body acting as a world-attached wall), so the rigid-body
        // set is never read here.
        let _ = bodies;
        let ids1: [u32; SIMD_WIDTH] = array![|ii| if manifolds[ii].data.relative_dominance <= 0
            && !manifold_id[ii].is_padding()
        {
            manifolds[ii].data.solver_body_ids[0]
        } else {
            u32::MAX
        }];
        let ids2: [u32; SIMD_WIDTH] = array![|ii| if manifolds[ii].data.relative_dominance >= 0
            && !manifold_id[ii].is_padding()
        {
            manifolds[ii].data.solver_body_ids[1]
        } else {
            u32::MAX
        }];

        // Optional guard: validate the solver-body ids once here, before the
        // unchecked SIMD gathers below (and every per-iteration gather that
        // reuses them). See `SolverBodies::assert_ids_in_range`.
        #[cfg(feature = "solver-bounds-checks")]
        {
            solver_bodies.assert_ids_in_range(ids1);
            solver_bodies.assert_ids_in_range(ids2);
        }

        let vels1 = solver_bodies.gather_vels(ids1);
        let poses1 = solver_bodies.gather_poses(ids1);
        let vels2 = solver_bodies.gather_vels(ids2);
        let poses2 = solver_bodies.gather_poses(ids2);

        let world_com1 = poses1.translation;
        let world_com2 = poses2.translation;

        // TODO PERF: implement SIMD gather
        let force_dir1 =
            -<SimdReal as ScalarType>::Vector::from(gather![|ii| manifolds[ii].data.normal.into()]);
        // Per-lane active-contact counts (color-only buckets: lanes of a chunk
        // may disagree; see the coulomb twin for the inert-slot encoding).
        let counts: [usize; SIMD_WIDTH] = array![|ii| manifolds[ii]
            .data
            .num_active_contacts()
            .min(MAX_MANIFOLD_POINTS)];
        // Optional guard (see the coulomb twin): the unchecked point gather relies
        // on every lane's count being > 0; a zero means a stale `ContactRef`
        // resolved to a non-active manifold (solver contact graph corruption).
        #[cfg(feature = "solver-bounds-checks")]
        for (ii, &c) in counts.iter().enumerate() {
            assert!(
                c > 0,
                "solver contact chunk lane {ii} resolved to a manifold with no \
                 active contacts — solver contact graph corruption"
            );
        }
        let num_points = counts.iter().copied().max().unwrap_or(1).max(1);
        // Per-lane counts as a wide value for the unconditional `active` selects
        // (no count-uniform fast path — one branchless code path; the selects
        // pass every value through on count-uniform chunks).
        let counts_simd = SimdReal::from(array![|ii| counts[ii] as Real]);

        #[cfg(feature = "dim2")]
        let tangents1 = force_dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = super::compute_tangent_contact_directions::<SimdReal>(
            &force_dir1,
            &vels1.linear,
            &vels2.linear,
        );

        // Friction/restitution are per-manifold (see `ContactManifoldData`).
        let friction = SimdReal::from(array![|ii| manifolds[ii].data.friction]);
        let restitution = SimdReal::from(array![|ii| manifolds[ii].data.restitution]);

        let manifold_points = array![|ii| &manifolds[ii].data.solver_contacts[..counts[ii]]];

        // Per-manifold averages (friction center, warm starts, tangent
        // velocity) weigh each lane by ITS point count, with inactive slots
        // contributing nothing.
        let inv_num_points: [Real; SIMD_WIDTH] = array![|ii| 1.0 / counts[ii] as Real];

        out_constraint.dir1 = force_dir1;
        out_constraint.im1 = poses1.im;
        out_constraint.im2 = poses2.im;
        out_constraint.ii1 = poses1.ii;
        out_constraint.ii2 = poses2.ii;
        out_builder.local_n1 = poses1.rotation.inverse() * force_dir1;
        out_builder.restitution = restitution;
        out_constraint.solver_vel1 = ids1;
        out_constraint.solver_vel2 = ids2;
        out_constraint.manifold_id = manifold_id;
        out_constraint.num_contacts = num_points as u8;
        #[cfg(feature = "dim3")]
        {
            out_constraint.tangent1 = tangents1[0];
        }

        let mut friction_center = Default::default();
        let mut friction_center2: <SimdReal as ScalarType>::Vector = Default::default();
        let mut twist_warmstart = SimdReal::zero();
        let mut tangent_warmstart = Default::default();
        let mut tangent_vel: <SimdReal as ScalarType>::Vector = Default::default();
        // The reconstructed world points, reused by the twist-distance loop below.
        let mut points: [<SimdReal as ScalarType>::Vector; MAX_MANIFOLD_POINTS] =
            [Default::default(); MAX_MANIFOLD_POINTS];

        for k in 0..num_points {
            // Per-(point, lane) averaging weight: `1/count` for the lane's own points, zero
            // beyond — inactive slots must not feed the per-manifold averages, and unlike the
            // constraint fields this can't be fixed up post-accumulation, so it stays in the loop.
            let weight = SimdReal::from(array![|ii| if k < counts[ii] {
                inv_num_points[ii]
            } else {
                0.0
            }]);
            // Lanes with fewer than `k + 1` active contacts gather their last
            // point instead (finite garbage) and are neutralized by the `active`
            // selects below (zero effective mass / warm-start ⇒ exact no-op).
            let active = counts_simd.simd_gt(SimdReal::splat(k as Real));
            let ks = array![|ii| k.min(counts[ii] - 1)];
            // SAFETY: `ks[ii] < counts[ii]` by construction.
            let solver_contact =
                unsafe { SimdSolverContact::gather_unchecked(&manifold_points, ks) };

            // Warm-start impulses and contact newness read straight off the manifold points
            // (not duplicated on the solver contacts): a zero `impulse` means the contact never
            // carried a load — exactly what the emission-time is-new bit encoded.
            let cids = solver_contact.contact_indices();
            let pt_data = |ii: usize| &manifolds[ii].points[cids[ii] as usize].data;
            let warmstart_impulse = SimdReal::from(gather![|ii| pt_data(ii).warmstart_impulse]);
            #[cfg(feature = "dim2")]
            let warmstart_tangent_impulse =
                TangentImpulse::new(SimdReal::from(gather![|ii| pt_data(ii)
                    .warmstart_tangent_impulse
                    .x]));
            // The friction warm-start is stored as a world-space vector and projected onto the
            // CURRENT tangent basis: reusing raw components silently rotates the
            // friction force whenever the basis changes with a regenerated manifold's normal.
            #[cfg(feature = "dim3")]
            let warmstart_tangent_impulse = {
                let w = <SimdReal as ScalarType>::Vector::from(gather![|ii| pt_data(ii)
                    .warmstart_tangent_world
                    .into()]);
                TangentImpulse::new(w.gdot(tangents1[0]), w.gdot(tangents1[1]))
            };
            #[cfg(feature = "dim3")]
            let warmstart_twist_impulse =
                SimdReal::from(gather![|ii| pt_data(ii).warmstart_twist_impulse]);
            #[cfg(feature = "dim2")]
            let warmstart_twist_impulse = SimdReal::zero();
            let is_new = SimdReal::from(gather![|ii| (pt_data(ii).impulse == 0.0) as u32 as Real]);
            let is_bouncy = crate::geometry::is_bouncy_simd(restitution, is_new);

            // Reconstruct the world contact points and separation from the body-local anchors
            // and solver poses (a world-attached side gathers the identity pose, so its anchor
            // passes through). This replaces the narrow-phase's per-frame refresh of recycled contacts.
            let p1 = poses1.transform_point(solver_contact.anchor1);
            let p2 = poses2.transform_point(solver_contact.anchor2);
            let dist = (p1 - p2).gdot(force_dir1);

            // World-space lever arms frozen at the pair's last full narrow-phase
            // update (anchor freezing, see `ContactData::solver_dp1`),
            // NOT re-derived per step: time-invariance stabilizes large stacks.
            let dp1 =
                <SimdReal as ScalarType>::Vector::from(gather![|ii| pt_data(ii).solver_dp1.into()]);
            let dp2 =
                <SimdReal as ScalarType>::Vector::from(gather![|ii| pt_data(ii).solver_dp2.into()]);

            // Each body's frozen contact point, riding its own rigid motion.
            let point = world_com1 + dp1;
            points[k] = point;

            friction_center += point * weight;
            friction_center2 += (world_com2 + dp2) * weight;

            let vel1 = vels1.linear + vels1.angular.gcross(dp1);
            let vel2 = vels2.linear + vels2.angular.gcross(dp2);

            twist_warmstart += warmstart_twist_impulse * weight;
            tangent_warmstart += warmstart_tangent_impulse * weight;
            tangent_vel += solver_contact.tangent_velocity * weight;

            out_constraint.limit = friction;
            // `u8::MAX` marks an inactive slot: the impulse writeback skips it.
            out_constraint.manifold_contact_id[k] = array![|ii| if k < counts[ii] {
                cids[ii] as u8
            } else {
                u8::MAX
            }];

            // Normal part.
            let restitution_seed;
            {
                let torque_dir1 = dp1.gcross(force_dir1);
                let torque_dir2 = dp2.gcross(-force_dir1);
                let ii_torque_dir1 = poses1.ii.transform_vector(torque_dir1);
                let ii_torque_dir2 = poses2.ii.transform_vector(torque_dir2);

                let imsum = poses1.im + poses2.im;
                let projected_mass = utils::simd_inv(
                    force_dir1.gdot(imsum.component_mul(&force_dir1))
                        + ii_torque_dir1.gdot(torque_dir1)
                        + ii_torque_dir2.gdot(torque_dir2),
                );

                let projected_velocity = (vel1 - vel2).gdot(force_dir1);
                restitution_seed = is_bouncy * restitution * projected_velocity;

                out_constraint.normal_part[k].torque_dir1 = torque_dir1;
                out_constraint.normal_part[k].torque_dir2 = torque_dir2;
                out_constraint.normal_part[k].ii_torque_dir1 = ii_torque_dir1;
                out_constraint.normal_part[k].ii_torque_dir2 = ii_torque_dir2;
                // Inactive slots: zero warm-start impulse and effective mass ⇒
                // the scalar normal solve is an exact no-op.
                out_constraint.normal_part[k].impulse =
                    warmstart_impulse.select(active, SimdReal::zero());
                // Minus the warm-start impulse, so the first substep's `update` cancels it when
                // it folds `impulse` in: that value belongs to the previous step (see the
                // coulomb-friction `generate`).
                out_constraint.normal_part[k].impulse_accumulator =
                    -out_constraint.normal_part[k].impulse;
                out_constraint.normal_part[k].r = projected_mass.select(active, SimdReal::zero());
            }

            // Builder: substep anchors are the frozen per-body arms (fixed-anchor
            // separation tracking); `dist` is rebased so the substep
            // tracking `info.dist + (p1 - p2)·n` is a delta from build-time poses.
            out_builder.infos[k].local_p1 = poses1.inverse_transform_point(point);
            out_builder.infos[k].local_p2 = poses2.inverse_transform_point(world_com2 + dp2);
            out_builder.infos[k].dist = dist - (point - (world_com2 + dp2)).gdot(force_dir1);
            out_builder.infos[k].restitution_seed = restitution_seed;
        }

        /*
         * Tangent/twist part
         */
        out_constraint.tangent_part.impulse = tangent_warmstart;
        out_constraint.tangent_part.impulse_accumulator = -tangent_warmstart;
        // The twist part only acts on lanes with more than one point (a single point offers no
        // lever arm): zero the warm-start of single-point lanes so a lane whose count just
        // dropped to one can't kick with its stale stored twist impulse.
        out_constraint.twist_part.impulse =
            twist_warmstart.select(counts_simd.simd_gt(SimdReal::splat(1.0)), SimdReal::zero());
        out_constraint.twist_part.impulse_accumulator = -out_constraint.twist_part.impulse;

        out_builder.local_friction_center1 = poses1.inverse_transform_point(friction_center);
        out_builder.local_friction_center2 = poses2.inverse_transform_point(friction_center2);

        let dp1 = friction_center - world_com1;
        let dp2 = friction_center2 - world_com2;

        // Twist part. It has no effect when there is only one point.
        if num_points > 1 {
            let mut twist_dists = [SimdReal::zero(); MAX_MANIFOLD_POINTS];
            for (k, point) in points.iter().enumerate().take(num_points) {
                // Inactive slots contribute no twist lever arm.
                let active = counts_simd.simd_gt(SimdReal::splat(k as Real));
                twist_dists[k] = (friction_center - *point)
                    .simd_length()
                    .select(active, SimdReal::zero());
            }

            let ii_twist_dir1 = poses1.ii.transform_vector(force_dir1);
            let ii_twist_dir2 = poses2.ii.transform_vector(-force_dir1);
            out_constraint.twist_part.rhs = SimdReal::zero();
            out_constraint.twist_part.r =
                utils::simd_inv(ii_twist_dir1.gdot(force_dir1) + ii_twist_dir2.gdot(-force_dir1));
            out_constraint.twist_dists = twist_dists;
        }

        // Tangent part.
        out_constraint.tangent_part.dp1 = dp1;
        out_constraint.tangent_part.dp2 = dp2;

        let mut torque_dirs1 = [Default::default(); 2];
        let mut torque_dirs2 = [Default::default(); 2];
        let mut ii_torque_dirs1 = [Default::default(); 2];
        let mut ii_torque_dirs2 = [Default::default(); 2];

        for j in 0..2 {
            let torque_dir1 = dp1.gcross(tangents1[j]);
            let torque_dir2 = dp2.gcross(-tangents1[j]);
            let ii_torque_dir1 = poses1.ii.transform_vector(torque_dir1);
            let ii_torque_dir2 = poses2.ii.transform_vector(torque_dir2);

            let imsum = poses1.im + poses2.im;

            let r = tangents1[j].gdot(imsum.component_mul(&tangents1[j]))
                + ii_torque_dir1.gdot(torque_dir1)
                + ii_torque_dir2.gdot(torque_dir2);

            // TODO: add something similar to tangent velocity to the twist
            //       constraint for the case where the different points don’t
            //       have the same tangent vel?
            let rhs_wo_bias = tangent_vel.gdot(tangents1[j]);

            torque_dirs1[j] = torque_dir1;
            torque_dirs2[j] = torque_dir2;
            ii_torque_dirs1[j] = ii_torque_dir1;
            ii_torque_dirs2[j] = ii_torque_dir2;
            out_constraint.tangent_part.rhs_wo_bias[j] = rhs_wo_bias;
            out_constraint.tangent_part.rhs[j] = rhs_wo_bias;
            out_constraint.tangent_part.r[j] = r;
        }

        out_constraint.tangent_part.torque_dir1 = torque_dirs1;
        out_constraint.tangent_part.torque_dir2 = torque_dirs2;
        out_constraint.tangent_part.ii_torque_dir1 = ii_torque_dirs1;
        out_constraint.tangent_part.ii_torque_dir2 = ii_torque_dirs2;

        out_constraint.tangent_part.r[2] = SimdReal::splat(2.0)
            * (ii_torque_dirs1[0].gdot(torque_dirs1[1]) + ii_torque_dirs2[0].gdot(torque_dirs2[1]));

        #[cfg(feature = "block-solver")]
        {
            // Coupling between consecutive normal-point pairs (see the coulomb builder): the
            // narrow-phase orders 4-point manifolds as two diagonal pairs, so each 2×2 block
            // spans the face in both directions and captures any rocking couple exactly.
            for k in 0..num_points / 2 {
                let k0 = k * 2;
                let k1 = k * 2 + 1;
                let pair_active = counts_simd.simd_gt(SimdReal::splat(k1 as Real));

                let imsum = poses1.im + poses2.im;
                let r0 = out_constraint.normal_part[k0].r;
                let r1 = out_constraint.normal_part[k1].r;

                let torque_dir1_0 = out_constraint.normal_part[k0].torque_dir1;
                let torque_dir2_0 = out_constraint.normal_part[k0].torque_dir2;
                let torque_dir1_1 = out_constraint.normal_part[k1].torque_dir1;
                let torque_dir2_1 = out_constraint.normal_part[k1].torque_dir2;

                let k12 = force_dir1.gdot(imsum.component_mul(&force_dir1))
                    + poses1
                        .ii
                        .transform_vector(torque_dir1_0)
                        .gdot(torque_dir1_1)
                    + poses2
                        .ii
                        .transform_vector(torque_dir2_0)
                        .gdot(torque_dir2_1);
                let (k11, k22) = (utils::simd_inv(r0), utils::simd_inv(r1));
                // See the coulomb builder: near-singular pairs fall back to the sequential path.
                let is_invertible = (k11 * k22 - k12 * k12)
                    .simd_gt(SimdReal::splat(BLOCK_SOLVER_MIN_CONDITION) * k11 * k22);

                // Degenerate or partially-active lanes store `[0, 0]`:
                // `solve_pair` degrades to the scalar soft solve of point k0.
                let block = is_invertible & pair_active;
                // `k12` stays on every lane: the degraded path carries the first point's
                // impulse change over to the second.
                out_constraint.normal_part[k0].r_mat_elts =
                    [k12, SimdReal::splat(1.0).select(block, SimdReal::zero())];
                out_constraint.normal_part[k1].r_mat_elts = [SimdReal::zero(); 2];
            }
        }
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        bodies: &SolverBodies,
        _multibodies: &MultibodyJointSet,
        constraint: &mut ContactWithTwistFriction<SimdReal>,
    ) {
        // Contacts touching a fixed body (world-attached side ⇒ solver-vel id `u32::MAX`)
        // use a stiffer "static" softness. Blend per lane with a 0/1 mask.
        let lane_static = |ii: usize| -> Real {
            (constraint.solver_vel1[ii] == u32::MAX || constraint.solver_vel2[ii] == u32::MAX)
                as u32 as Real
        };
        let is_static = SimdReal::from(array![lane_static]);
        let dyn_cfm = params.contact_softness.cfm_factor(params.dt);
        let static_cfm = params.static_contact_softness.cfm_factor(params.dt);
        let dyn_erp = params.contact_softness.erp_inv_dt(params.dt);
        let static_erp = params.static_contact_softness.erp_inv_dt(params.dt);
        let cfm_factor =
            SimdReal::splat(dyn_cfm) + is_static * SimdReal::splat(static_cfm - dyn_cfm);
        let inv_dt = SimdReal::splat(params.inv_dt());
        let erp_inv_dt =
            SimdReal::splat(dyn_erp) + is_static * SimdReal::splat(static_erp - dyn_erp);
        let max_corrective_velocity = SimdReal::splat(params.max_corrective_velocity());
        let warmstart_coeff = SimdReal::splat(params.warmstart_coefficient);

        // Only the transform part of the poses is needed here: this gather does
        // half the transposition work of a full pose gather.
        let poses1 = bodies.gather_transforms(constraint.solver_vel1);
        let poses2 = bodies.gather_transforms(constraint.solver_vel2);
        let all_infos = &self.infos[..constraint.num_contacts as usize];
        let normal_parts = &mut constraint.normal_part[..constraint.num_contacts as usize];
        let tangent_part = &mut constraint.tangent_part;
        let twist_part = &mut constraint.twist_part;

        #[cfg(feature = "dim2")]
        let tangents1 = constraint.dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = [
            constraint.tangent1,
            constraint.dir1.gcross(constraint.tangent1),
        ];

        let solved_dt = SimdReal::splat(solved_dt);
        let tangent_delta = self.tangent_vel * solved_dt;

        for (info, normal_part) in all_infos.iter().zip(normal_parts.iter_mut()) {
            // NOTE: the tangent velocity is equivalent to an additional movement of the first body’s surface.
            let p1 = poses1.transform_point(info.local_p1) + tangent_delta;
            let p2 = poses2.transform_point(info.local_p2);
            let dist = info.dist + (p1 - p2).gdot(constraint.dir1);

            // Normal part.
            {
                // NOTE: `info.restitution_seed` is deliberately NOT part of the substep
                // rhs (the speculative slack would truncate the bounce there):
                // `Self::apply_restitution` applies it once, after all substeps.
                let rhs_wo_bias = dist.simd_max(SimdReal::zero()) * inv_dt;
                // No slop deadzone on the bias:
                // `allowed_linear_error` is geometric slop, not a solver deadzone.
                // A deadzone makes large piles settle deep, wedge, and creep.
                let rhs_bias =
                    (dist * erp_inv_dt).simd_clamp(-max_corrective_velocity, SimdReal::zero());
                let new_rhs = rhs_wo_bias + rhs_bias;

                normal_part.rhs_wo_bias = rhs_wo_bias;
                normal_part.rhs = new_rhs;
                // Separated (speculative) points are solved rigidly: the
                // touchdown is perfectly inelastic, which is what damps stack
                // rocking. Only penetrating points get the soft treatment.
                normal_part.cfm_factor =
                    cfm_factor.select(dist.simd_le(SimdReal::zero()), SimdReal::splat(1.0));
                // Bank the previous substep's impulse before the warm-start scaling (see the
                // coulomb-friction `update`).
                normal_part.impulse_accumulator += normal_part.impulse;
                normal_part.impulse *= warmstart_coeff;
            }
        }

        // tangent parts.
        {
            let p1 = poses1.transform_point(self.local_friction_center1) + tangent_delta;
            let p2 = poses2.transform_point(self.local_friction_center2);

            for j in 0..DIM - 1 {
                let bias = (p1 - p2).gdot(tangents1[j]) * inv_dt;
                tangent_part.rhs[j] = tangent_part.rhs_wo_bias[j] + bias;
            }
            tangent_part.impulse_accumulator += tangent_part.impulse;
            tangent_part.impulse *= warmstart_coeff;
            twist_part.impulse_accumulator += twist_part.impulse;
            twist_part.impulse *= warmstart_coeff;
        }

        constraint.cfm_factor = cfm_factor;
    }

    /// Relax-pass refresh: recompute the unbiased rhs (speculative term included)
    /// from the CURRENT solver poses, stripping softness and penetration bias. Positions
    /// integrate between the biased and unbiased passes, so the separations `update` baked are
    /// stale by one substep; enforcing them makes a lifted edge read as still touching (its
    /// returning velocity cancelled), driving the rocking mode of tall stacks instead of damping it.
    pub fn refresh_rhs_wo_bias(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        bodies: &SolverBodies,
        constraint: &mut ContactWithTwistFriction<SimdReal>,
    ) {
        let inv_dt = SimdReal::splat(params.inv_dt());
        let poses1 = bodies.gather_transforms(constraint.solver_vel1);
        let poses2 = bodies.gather_transforms(constraint.solver_vel2);
        let all_infos = &self.infos[..constraint.num_contacts as usize];
        let normal_parts = &mut constraint.normal_part[..constraint.num_contacts as usize];
        let tangent_delta = self.tangent_vel * SimdReal::splat(solved_dt);

        for (info, normal_part) in all_infos.iter().zip(normal_parts.iter_mut()) {
            let p1 = poses1.transform_point(info.local_p1) + tangent_delta;
            let p2 = poses2.transform_point(info.local_p2);
            let dist = info.dist + (p1 - p2).gdot(constraint.dir1);
            // See `update`: the restitution seed is applied by `apply_restitution`, not here.
            normal_part.rhs = dist.simd_max(SimdReal::zero()) * inv_dt;
            normal_part.cfm_factor = SimdReal::splat(1.0);
        }

        constraint.cfm_factor = SimdReal::splat(1.0);
        constraint.tangent_part.rhs = constraint.tangent_part.rhs_wo_bias;
    }

    /// Does any active point of this chunk hold a restitution seed (an approaching bouncy
    /// contact captured at prepare time)? Cheap pre-check gating [`Self::apply_restitution`].
    pub fn has_bouncy_seed(&self, num_contacts: u8) -> bool {
        let num = (num_contacts as usize).min(MAX_MANIFOLD_POINTS);
        self.infos[..num]
            .iter()
            .any(|info| info.restitution_seed.simd_lt(SimdReal::zero()).any())
    }

    /// End-of-step restitution pass (box2d-style): after all substeps, drive each bouncy
    /// point's normal velocity to its prepare-time `restitution * approach_velocity`, gated
    /// on the point having carried an impulse. See `ContactConstraintNormalPart::solve_restitution`.
    pub fn apply_restitution(
        &self,
        constraint: &mut ContactWithTwistFriction<SimdReal>,
        bodies: &mut SolverBodies,
    ) {
        if !self.has_bouncy_seed(constraint.num_contacts) {
            return;
        }

        let mut solver_vel1 = bodies.gather_vels(constraint.solver_vel1);
        let mut solver_vel2 = bodies.gather_vels(constraint.solver_vel2);

        let num = constraint.num_contacts as usize;
        for (info, normal_part) in self.infos[..num]
            .iter()
            .zip(constraint.normal_part[..num].iter_mut())
        {
            normal_part.solve_restitution(
                info.restitution_seed,
                &constraint.dir1,
                &constraint.im1,
                &constraint.im2,
                &mut solver_vel1,
                &mut solver_vel2,
            );
        }

        bodies.scatter_vels(constraint.solver_vel1, solver_vel1);
        bodies.scatter_vels(constraint.solver_vel2, solver_vel2);
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub(crate) struct ContactWithTwistFriction<N: ScalarType> {
    pub dir1: N::Vector, // Non-penetration force direction for the first body.
    pub im1: N::Vector,
    pub im2: N::Vector,
    // World inverse inertia of both bodies, used to recompute the angular
    // jacobians from the stored lever arms at each use (cheaper than streaming
    // the precomputed jacobians from memory).
    pub ii1: N::AngInertia,
    pub ii2: N::AngInertia,
    pub cfm_factor: N,
    pub limit: N,

    #[cfg(feature = "dim3")]
    pub tangent1: N::Vector, // One of the friction force directions.
    pub normal_part: [ContactConstraintNormalPartSlim<N>; MAX_MANIFOLD_POINTS],
    // The twist friction model emulates coulomb with only one tangent
    // constraint + one twist constraint per manifold.
    pub tangent_part: ContactConstraintTangentPartSlim<N>,
    // Twist constraint (angular-only) to compensate the lack of angular resistance on the tangent plane.
    pub twist_part: ContactConstraintTwistPartSlim<N>,
    // Distances between the friction center and the contact point.
    pub twist_dists: [N; MAX_MANIFOLD_POINTS],

    pub solver_vel1: [u32; SIMD_WIDTH],
    pub solver_vel2: [u32; SIMD_WIDTH],
    pub manifold_id: [ContactRef; SIMD_WIDTH],
    pub num_contacts: u8,
    pub manifold_contact_id: [[u8; SIMD_WIDTH]; MAX_MANIFOLD_POINTS],
}

impl ContactWithTwistFriction<SimdReal> {
    pub fn warmstart(&mut self, bodies: &mut SolverBodies) {
        let mut solver_vel1 = bodies.gather_vels(self.solver_vel1);
        let mut solver_vel2 = bodies.gather_vels(self.solver_vel2);

        let normal_parts = &mut self.normal_part[..self.num_contacts as usize];

        /*
         * Warmstart restitution.
         */
        for normal_part in normal_parts.iter_mut() {
            normal_part.warmstart(
                &self.dir1,
                &self.im1,
                &self.im2,
                &mut solver_vel1,
                &mut solver_vel2,
            );
        }

        /*
         * Warmstart friction.
         */
        let tangents1 = [&self.tangent1, &self.dir1.gcross(self.tangent1)];

        self.tangent_part.warmstart(
            tangents1,
            &self.im1,
            &self.im2,
            &mut solver_vel1,
            &mut solver_vel2,
        );
        // NOTE: if there is only 1 contact, the twist part has no effect (its
        //       effective mass isn't even initialized by the builder).
        if self.num_contacts > 1 {
            self.twist_part.warmstart(
                &self.dir1,
                &self.ii1,
                &self.ii2,
                &mut solver_vel1,
                &mut solver_vel2,
            );
        }

        bodies.scatter_vels(self.solver_vel1, solver_vel1);
        bodies.scatter_vels(self.solver_vel2, solver_vel2);
    }

    pub fn solve(
        &mut self,
        bodies: &mut SolverBodies,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let mut solver_vel1 = bodies.gather_vels(self.solver_vel1);
        let mut solver_vel2 = bodies.gather_vels(self.solver_vel2);

        let normal_parts = &mut self.normal_part[..self.num_contacts as usize];

        /*
         * Solve restitution.
         */
        if solve_restitution {
            #[cfg(feature = "block-solver")]
            {
                for normal_part in normal_parts.chunks_exact_mut(2) {
                    let [normal_part_a, normal_part_b] = normal_part else {
                        unreachable!()
                    };
                    ContactConstraintNormalPartSlim::solve_pair(
                        normal_part_a,
                        normal_part_b,
                        &self.dir1,
                        &self.im1,
                        &self.im2,
                        &mut solver_vel1,
                        &mut solver_vel2,
                    );
                }
                if normal_parts.len() % 2 == 1 {
                    let normal_part = normal_parts.last_mut().unwrap();
                    normal_part.solve(
                        &self.dir1,
                        &self.im1,
                        &self.im2,
                        &mut solver_vel1,
                        &mut solver_vel2,
                    );
                }
            }
            #[cfg(not(feature = "block-solver"))]
            for normal_part in normal_parts.iter_mut() {
                normal_part.solve(
                    &self.dir1,
                    &self.im1,
                    &self.im2,
                    &mut solver_vel1,
                    &mut solver_vel2,
                );
            }
        }

        /*
         * Solve friction.
         */
        if solve_friction {
            let tangents1 = [&self.tangent1, &self.dir1.gcross(self.tangent1)];

            let mut tangent_limit = SimdReal::zero();
            let mut twist_limit = SimdReal::zero();
            for (normal_part, dist) in normal_parts.iter().zip(self.twist_dists.iter()) {
                tangent_limit += normal_part.impulse;
                // The twist limit is computed as the sum of impulses multiplied by the
                // lever-arm length relative to the friction center. The rational is that
                // the further the point is from the friction center, the stronger angular
                // resistance it can offer.
                twist_limit += normal_part.impulse * *dist;
            }

            // Multiply by the friction coefficient.
            tangent_limit *= self.limit;
            twist_limit *= self.limit;

            // Twist first, then central friction: the twist solve changes the
            // angular velocities the central-friction constraint reads at its lever arms.
            // NOTE: if there is only 1 contact, the twist part has no effect.
            if self.num_contacts > 1 {
                self.twist_part.solve(
                    &self.dir1,
                    &self.ii1,
                    &self.ii2,
                    twist_limit,
                    &mut solver_vel1,
                    &mut solver_vel2,
                );
            }

            self.tangent_part.solve(
                tangents1,
                &self.im1,
                &self.im2,
                tangent_limit,
                &mut solver_vel1,
                &mut solver_vel2,
            );
        }

        bodies.scatter_vels(self.solver_vel1, solver_vel1);
        bodies.scatter_vels(self.solver_vel2, solver_vel2);
    }

    pub fn writeback_impulses(&self, manifolds_all: &ManifoldStore) {
        // The stored impulses are serialized state: canonicalize signed zeros (see
        // `utils::canonicalize_zero`) so snapshots stay cross-platform deterministic.
        let warmstart_tangent_impulses = utils::canonicalize_zero(self.tangent_part.impulse);
        // World-space friction impulse (see `ContactData::warmstart_tangent_world`).
        let tangent2 = self.dir1.gcross(self.tangent1);
        let warmstart_tangent_world = utils::canonicalize_zero(
            self.tangent1 * warmstart_tangent_impulses.x + tangent2 * warmstart_tangent_impulses.y,
        );
        let (wx, wy, wz): ([Real; SIMD_WIDTH], [Real; SIMD_WIDTH], [Real; SIMD_WIDTH]) = (
            warmstart_tangent_world.x.into(),
            warmstart_tangent_world.y.into(),
            warmstart_tangent_world.z.into(),
        );
        let warmstart_twist_impulses: [_; SIMD_WIDTH] =
            utils::canonicalize_zero(self.twist_part.impulse).into();

        for k in 0..self.num_contacts as usize {
            let warmstart_impulses: [_; SIMD_WIDTH] =
                utils::canonicalize_zero(self.normal_part[k].impulse).into();
            let impulses: [_; SIMD_WIDTH] =
                utils::canonicalize_zero(self.normal_part[k].total_impulse()).into();

            for ii in 0..SIMD_WIDTH {
                let contact_id = self.manifold_contact_id[k][ii];
                // `u8::MAX` = inactive slot (this lane's manifold has fewer
                // than `k + 1` active contacts).
                if !self.manifold_id[ii].is_padding() && contact_id != u8::MAX {
                    // SAFETY: each (edge, ordinal) lane belongs to exactly one
                    //         constraint chunk; no other live reference exists.
                    let manifold = unsafe { manifolds_all.get_mut(self.manifold_id[ii]) };
                    let active_contact = &mut manifold.points[contact_id as usize];
                    active_contact.data.warmstart_impulse = warmstart_impulses[ii];
                    active_contact.data.impulse = impulses[ii];
                    active_contact.data.warmstart_tangent_impulse =
                        warmstart_tangent_impulses.extract(ii);
                    {
                        active_contact.data.warmstart_tangent_world =
                            crate::math::Vector::new(wx[ii], wy[ii], wz[ii]);
                    }
                    {
                        active_contact.data.warmstart_twist_impulse = warmstart_twist_impulses[ii];
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::geometry::SolverContact;
    use crate::math::Vector;
    use parry::shape::PackedFeatureId;

    fn vect(x: Real, y: Real) -> Vector {
        Vector::new(x, y, 0.3 * x - 0.1 * y)
    }

    /// See the coulomb kernel's twin: a world-attached manifold with `n`
    /// distinct contacts and non-trivial warm-start data.
    fn test_manifold(n: usize, seed: Real) -> ContactManifold {
        let mut m = ContactManifold::new();
        m.data.normal = Vector::new(0.0, 1.0, 0.0);
        m.data.friction = 0.7;
        m.data.restitution = 0.0;
        m.data.relative_dominance = 0;
        m.data.solver_body_ids = [u32::MAX; 2];
        for k in 0..n {
            let kf = k as Real;
            let mut pt = parry::query::TrackedContact::<crate::geometry::ContactData>::new(
                vect(seed + kf, 0.5),
                vect(seed + kf, -0.5),
                PackedFeatureId::face(k as u32),
                PackedFeatureId::face(k as u32),
                -0.01,
            );
            pt.data.warmstart_impulse = seed + kf + 0.25;
            pt.data.warmstart_tangent_impulse[0] = seed - kf * 0.5;
            pt.data.warmstart_tangent_impulse[1] = seed * 0.5 + kf;
            pt.data.warmstart_twist_impulse = seed * 0.25 + kf;
            pt.data.impulse = 1.0;
            m.points.push(pt);
            m.data.solver_contacts.push(SolverContact {
                anchor1: vect(0.3 * kf + seed, 0.5),
                anchor2: vect(0.3 * kf + seed, -0.5),
                dist: -0.01,
                tangent_velocity: vect(0.0, 0.0) * 0.0,
                contact_id: [k as crate::geometry::ContactId],
                padding: [0.0],
            });
        }
        m
    }

    fn generate_chunk(
        manifolds: [&ContactManifold; SIMD_WIDTH],
    ) -> ContactWithTwistFriction<SimdReal> {
        let bodies = RigidBodySet::new();
        let solver_bodies = SolverBodies::default();
        let mut builder: ContactWithTwistFrictionBuilder<SimdReal> = unsafe { core::mem::zeroed() };
        let mut constraint: ContactWithTwistFriction<SimdReal> = unsafe { core::mem::zeroed() };
        let ids: [ContactRef; SIMD_WIDTH] = core::array::from_fn(|ii| ContactRef {
            edge: ii as u32,
            manifold: 0,
        });
        ContactWithTwistFrictionBuilder::generate(
            ids,
            manifolds,
            &bodies,
            &solver_bodies,
            &mut builder,
            &mut constraint,
        );
        constraint
    }

    /// Mixed-count masking property for the twist kernel: active slots match the uniform
    /// reference lanes bit-exactly (including the count-weighted friction-center aggregates),
    /// inactive slots hold the neutral fill, single-point lanes have no twist warm-start.
    #[test]
    fn mixed_count_generate_matches_uniform_lanes_and_neutral_fill() {
        if SIMD_WIDTH < 2 {
            return;
        }
        let m_full = test_manifold(MAX_MANIFOLD_POINTS, 1.0);
        let m_one = test_manifold(1, 2.0);
        let mixed: [&ContactManifold; SIMD_WIDTH] =
            core::array::from_fn(|ii| if ii % 2 == 0 { &m_full } else { &m_one });
        let cm = generate_chunk(mixed);
        let cf = generate_chunk([&m_full; SIMD_WIDTH]);
        let co = generate_chunk([&m_one; SIMD_WIDTH]);

        assert_eq!(cm.num_contacts as usize, MAX_MANIFOLD_POINTS);

        for ii in 0..SIMD_WIDTH {
            let (uni, count) = if ii % 2 == 0 {
                (&cf, MAX_MANIFOLD_POINTS)
            } else {
                (&co, 1)
            };
            for k in 0..MAX_MANIFOLD_POINTS {
                let np = &cm.normal_part[k];
                if k < count {
                    let unp = &uni.normal_part[k];
                    assert_eq!(np.r.extract(ii), unp.r.extract(ii));
                    assert_eq!(np.impulse.extract(ii), unp.impulse.extract(ii));
                    assert_eq!(
                        cm.manifold_contact_id[k][ii],
                        uni.manifold_contact_id[k][ii]
                    );
                } else {
                    assert_eq!(cm.manifold_contact_id[k][ii], u8::MAX);
                    assert_eq!(np.r.extract(ii), 0.0);
                    assert_eq!(np.impulse.extract(ii), 0.0);
                    assert_eq!(cm.twist_dists[k].extract(ii), 0.0);
                }
            }

            // The count-weighted aggregates (friction center → tangent lever
            // arms, averaged warm-starts) must not see the duplicated gathers.
            for j in 0..2 {
                assert_eq!(
                    cm.tangent_part.impulse[j].extract(ii),
                    uni.tangent_part.impulse[j].extract(ii)
                );
            }
            assert_eq!(
                cm.tangent_part.dp1.x.extract(ii),
                uni.tangent_part.dp1.x.extract(ii)
            );
            assert_eq!(
                cm.tangent_part.dp1.y.extract(ii),
                uni.tangent_part.dp1.y.extract(ii)
            );
            assert_eq!(
                cm.tangent_part.dp1.z.extract(ii),
                uni.tangent_part.dp1.z.extract(ii)
            );

            if count < 2 {
                // Single-point lanes: no twist lever arm, no twist warm-start.
                assert_eq!(cm.twist_part.impulse.extract(ii), 0.0);
            } else {
                assert_eq!(
                    cm.twist_part.impulse.extract(ii),
                    uni.twist_part.impulse.extract(ii)
                );
                for k in 0..count {
                    assert_eq!(
                        cm.twist_dists[k].extract(ii),
                        uni.twist_dists[k].extract(ii)
                    );
                }
            }
        }
    }
}
