use super::{ContactConstraintNormalPart, ContactConstraintTangentPart};
#[cfg(feature = "block-solver")]
use crate::dynamics::solver::contact_constraint::BLOCK_SOLVER_MIN_CONDITION;
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::solver::solver_contact_graph::ContactRef;
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodySet};
use crate::geometry::{ContactManifold, SimdSolverContact};
use crate::math::{DIM, MAX_MANIFOLD_POINTS, Real, SIMD_WIDTH, SimdReal, TangentImpulse};
#[cfg(feature = "dim2")]
use crate::utils::OrthonormalBasis;
use crate::utils::{self, AngularInertiaOps, CrossProduct, DotProduct, ScalarType};
use num::Zero;
use simba::simd::{SimdBool, SimdPartialOrd, SimdValue};

#[derive(Copy, Clone, Debug)]
pub struct CoulombContactPointInfos<N: ScalarType> {
    pub tangent_vel: N::Vector, // PERF: could be one float less, be shared by both contact point infos?
    /// `restitution * approach_velocity`, captured at prepare time and zeroed on
    /// non-bouncy points (see [`crate::geometry::is_bouncy_simd`]). Negative while
    /// approaching: the end-of-step pass drives the normal velocity to `-restitution_seed`.
    pub restitution_seed: N,
    pub local_p1: N::Vector,
    pub local_p2: N::Vector,
    pub dist: N,
}

impl<N: ScalarType> Default for CoulombContactPointInfos<N> {
    fn default() -> Self {
        Self {
            tangent_vel: Default::default(),
            restitution_seed: N::zero(),
            local_p1: Default::default(),
            local_p2: Default::default(),
            dist: N::zero(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactWithCoulombFrictionBuilder {
    infos: [CoulombContactPointInfos<SimdReal>; MAX_MANIFOLD_POINTS],
    /// The contact normal in the first body's (com-centered) local frame, so
    /// `refresh` can re-derive the world normal without touching the manifold.
    local_n1: <SimdReal as ScalarType>::Vector,
    /// The pair's restitution coefficient (needed by `refresh` to recompute the
    /// restitution rhs seed).
    restitution: SimdReal,
}

impl ContactWithCoulombFrictionBuilder {
    pub fn generate(
        manifold_id: [ContactRef; SIMD_WIDTH],
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        solver_bodies: &SolverBodies,
        out_builder: &mut ContactWithCoulombFrictionBuilder,
        out_constraint: &mut ContactWithCoulombFriction<SimdReal>,
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

        // Per-lane active-contact counts: buckets are keyed by color only, so lanes may
        // disagree. Inactive `(point, lane)` slots are encoded inertly below (zero impulse and
        // effective mass, degraded 2×2 elements) — exact no-ops in every sweep. Padding lanes
        // replicate lane 0's manifold, hence lane 0's count.
        let counts: [usize; SIMD_WIDTH] = array![|ii| manifolds[ii]
            .data
            .num_active_contacts()
            .min(MAX_MANIFOLD_POINTS)];
        // Optional guard: the unchecked gather below needs every lane's count > 0 (else
        // `counts[ii] - 1` underflows and `gather_unchecked` reads an empty slice — UB). The
        // graph only holds manifolds with active contacts, so zero means a stale `ContactRef`
        // (graph corruption). Off the per-iteration path; see `SolverBodies::assert_ids_in_range`.
        #[cfg(feature = "solver-bounds-checks")]
        for (ii, &c) in counts.iter().enumerate() {
            assert!(
                c > 0,
                "solver contact chunk lane {ii} resolved to a manifold with no \
                 active contacts — solver contact graph corruption"
            );
        }
        let num_points = counts.iter().copied().max().unwrap_or(1).max(1);
        // Per-lane counts as a wide value: inactive `(point, lane)` slots are neutralized by
        // unconditional lane-mask selects (`active = counts > k`) inline at each write —
        // one branchless path, no count-uniform fast path.
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

        out_constraint.dir1 = force_dir1;
        out_constraint.im1 = poses1.im;
        out_constraint.im2 = poses2.im;
        out_constraint.solver_vel1 = ids1;
        out_constraint.solver_vel2 = ids2;
        out_constraint.manifold_id = manifold_id;
        out_constraint.num_contacts = num_points as u8;
        out_builder.local_n1 = poses1.rotation.inverse() * force_dir1;
        out_builder.restitution = restitution;
        #[cfg(feature = "dim3")]
        {
            out_constraint.tangent1 = tangents1[0];
        }

        for k in 0..num_points {
            // Lanes with fewer than `k + 1` active contacts gather their last point (finite
            // garbage) and are neutralized by the `active` selects below (zero effective mass /
            // warm-start ⇒ exact no-op); on count-uniform chunks `active` is all-true.
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
            let is_new = SimdReal::from(gather![|ii| (pt_data(ii).impulse == 0.0) as u32 as Real]);
            let is_bouncy = crate::geometry::is_bouncy_simd(restitution, is_new);

            // Inactive slots must not warm-start (their gathered values belong
            // to another point of the lane's manifold).
            let warmstart_impulse = warmstart_impulse.select(active, SimdReal::zero());
            let warmstart_tangent_impulse =
                warmstart_tangent_impulse.map(|x| x.select(active, SimdReal::zero()));

            // Reconstruct the world contact points and separation from the body-local anchors
            // and solver poses (a world-attached side gathers the identity pose, so its anchor
            // passes through). This replaces the narrow-phase's per-frame refresh of recycled contacts.
            let p1 = poses1.transform_point(solver_contact.anchor1);
            let p2 = poses2.transform_point(solver_contact.anchor2);
            let dist = (p1 - p2).gdot(force_dir1);

            // Lever arms are the world-space arms frozen at the pair's last full narrow-phase
            // update (anchor freezing, `ContactData::solver_dp1`) — NOT re-derived
            // from current poses: time-invariance across recycled steps is load-bearing for
            // large-stack stability. Separations still track the bodies' actual rigid motion.
            let dp1 =
                <SimdReal as ScalarType>::Vector::from(gather![|ii| pt_data(ii).solver_dp1.into()]);
            let dp2 =
                <SimdReal as ScalarType>::Vector::from(gather![|ii| pt_data(ii).solver_dp2.into()]);

            let vel1 = vels1.linear + vels1.angular.gcross(dp1);
            let vel2 = vels2.linear + vels2.angular.gcross(dp2);

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
                out_constraint.normal_part[k].impulse = warmstart_impulse;
                // The accumulator must start at zero every step: the constraint buffers are
                // reused without zeroing (`reset_buffer_reusing`), so stale bytes survive in
                // fields `generate` doesn't write.
                out_constraint.normal_part[k].impulse_accumulator = SimdReal::zero();
                // Zero effective mass on inactive slots: the scalar normal solve
                // is then an exact no-op (impulse stays at its zeroed warm-start).
                out_constraint.normal_part[k].r = projected_mass.select(active, SimdReal::zero());
            }

            // tangent parts.
            out_constraint.tangent_part[k].impulse = warmstart_tangent_impulse;
            // See the normal part: explicit zero, the buffers are not zeroed.
            out_constraint.tangent_part[k].impulse_accumulator = na::zero();

            for j in 0..DIM - 1 {
                let torque_dir1 = dp1.gcross(tangents1[j]);
                let torque_dir2 = dp2.gcross(-tangents1[j]);
                let ii_torque_dir1 = poses1.ii.transform_vector(torque_dir1);
                let ii_torque_dir2 = poses2.ii.transform_vector(torque_dir2);

                let imsum = poses1.im + poses2.im;

                let r = tangents1[j].gdot(imsum.component_mul(&tangents1[j]))
                    + ii_torque_dir1.gdot(torque_dir1)
                    + ii_torque_dir2.gdot(torque_dir2);
                let rhs_wo_bias = solver_contact.tangent_velocity.gdot(tangents1[j]);

                out_constraint.tangent_part[k].torque_dir1[j] = torque_dir1;
                out_constraint.tangent_part[k].torque_dir2[j] = torque_dir2;
                out_constraint.tangent_part[k].ii_torque_dir1[j] = ii_torque_dir1;
                out_constraint.tangent_part[k].ii_torque_dir2[j] = ii_torque_dir2;
                out_constraint.tangent_part[k].rhs_wo_bias[j] = rhs_wo_bias;
                out_constraint.tangent_part[k].rhs[j] = rhs_wo_bias;
                // Inactive slots: 2D stores the inverse mass, so zero makes the solve a no-op;
                // 3D's coupled solve divides by an `r`-weighted form — a unit diagonal keeps it
                // finite while the zero friction limit pins the impulse to zero.
                out_constraint.tangent_part[k].r[j] = if cfg!(feature = "dim2") {
                    utils::simd_inv(r).select(active, SimdReal::zero())
                } else {
                    r.select(active, SimdReal::splat(1.0))
                };
            }

            #[cfg(feature = "dim3")]
            {
                // TODO PERF: we already applied the inverse inertia to the torque
                //            dire before. Could we reuse the value instead of retransforming?
                let r2 = SimdReal::splat(2.0)
                    * (out_constraint.tangent_part[k].ii_torque_dir1[0]
                        .gdot(out_constraint.tangent_part[k].torque_dir1[1])
                        + out_constraint.tangent_part[k].ii_torque_dir2[0]
                            .gdot(out_constraint.tangent_part[k].torque_dir2[1]));
                out_constraint.tangent_part[k].r[2] = r2.select(active, SimdReal::zero());
            }

            // Builder. The substep anchors are the frozen per-body arms (each body's frozen
            // contact point rides its own rigid motion — fixed-anchor separation
            // tracking); the base separation is reconstructed from the anchors above.
            out_builder.infos[k].local_p1 = poses1.inverse_transform_point(world_com1 + dp1);
            out_builder.infos[k].local_p2 = poses2.inverse_transform_point(world_com2 + dp2);
            out_builder.infos[k].tangent_vel = solver_contact.tangent_velocity;
            // Rebased so the per-substep tracking `info.dist + (p1 - p2)·n` is a pure delta
            // from the build-time poses (a base separation): with frozen arms, `p1 - p2`
            // is non-zero at build time on recycled pairs (drift since the freeze, already in `dist`).
            out_builder.infos[k].dist =
                dist - ((world_com1 + dp1) - (world_com2 + dp2)).gdot(force_dir1);
            out_builder.infos[k].restitution_seed = restitution_seed;
        }

        #[cfg(feature = "block-solver")]
        {
            // Coupling between consecutive pairs.
            for k in 0..num_points / 2 {
                let k0 = k * 2;
                let k1 = k * 2 + 1;
                // Lanes lacking both points of this pair (`count <= 2k+1`) must not form a
                // block: they fall back to the degraded `[r0, 0]` / `[0, 0]` elements, reducing
                // `solve_pair` to the scalar solve of point k0 (itself a no-op if k0 is inactive).
                let pair_active = counts_simd.simd_gt(SimdReal::splat(k1 as Real));

                let imsum = poses1.im + poses2.im;
                let r0 = out_constraint.normal_part[k0].r;
                let r1 = out_constraint.normal_part[k1].r;

                // TODO PERF: we already applied the inverse inertia to the torque
                //            dire before. Could we reuse the value instead of retransforming?
                let k12 = force_dir1.gdot(imsum.component_mul(&force_dir1))
                    + out_constraint.normal_part[k0]
                        .ii_torque_dir1
                        .gdot(out_constraint.normal_part[k1].torque_dir1)
                    + out_constraint.normal_part[k0]
                        .ii_torque_dir2
                        .gdot(out_constraint.normal_part[k1].torque_dir2);
                let (k11, k22) = (utils::simd_inv(r0), utils::simd_inv(r1));
                // Conditioning check on the physical K: a bare `det > 0` also admits
                // near-singular pairs, and block-solving those keeps piles jiggling.
                let is_invertible = (k11 * k22 - k12 * k12)
                    .simd_gt(SimdReal::splat(BLOCK_SOLVER_MIN_CONDITION) * k11 * k22);

                // Degenerate (redundant contacts) or partially-active lanes clear the block
                // flag: `solve_pair` then solves the two points sequentially instead.
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
        constraint: &mut ContactWithCoulombFriction<SimdReal>,
    ) {
        // Contacts touching a fixed body (world-attached side ⇒ solver-vel id `u32::MAX`)
        // use a stiffer "static" softness so bodies are held more firmly
        // against static geometry. Blend per lane with a 0/1 mask.
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
        let tangent_parts = &mut constraint.tangent_part[..constraint.num_contacts as usize];

        #[cfg(feature = "dim2")]
        let tangents1 = constraint.dir1.orthonormal_basis();
        #[cfg(feature = "dim3")]
        let tangents1 = [
            constraint.tangent1,
            constraint.dir1.gcross(constraint.tangent1),
        ];

        let solved_dt = SimdReal::splat(solved_dt);

        for ((info, normal_part), tangent_part) in all_infos
            .iter()
            .zip(normal_parts.iter_mut())
            .zip(tangent_parts.iter_mut())
        {
            // NOTE: the tangent velocity is equivalent to an additional movement of the first body’s surface.
            let p1 = poses1.transform_point(info.local_p1) + info.tangent_vel * solved_dt;
            let p2 = poses2.transform_point(info.local_p2);
            let dist = info.dist + (p1 - p2).gdot(constraint.dir1);

            // Normal part.
            {
                // NOTE: `info.restitution_seed` is deliberately NOT part of the substep
                // rhs (the speculative slack would truncate the bounce there):
                // `Self::apply_restitution` applies it once, after all substeps.
                let rhs_wo_bias = dist.simd_max(SimdReal::zero()) * inv_dt;
                // No slop deadzone on the position-correction bias;
                // `allowed_linear_error` is geometric slop, not a solver deadzone. A deadzone
                // lets every loaded interface settle to its deep edge, and the accumulated
                // penetration keeps large piles wedging and creeping instead of resting.
                let rhs_bias =
                    (dist * erp_inv_dt).simd_clamp(-max_corrective_velocity, SimdReal::zero());
                let new_rhs = rhs_wo_bias + rhs_bias;

                normal_part.rhs_wo_bias = rhs_wo_bias;
                normal_part.rhs = new_rhs;
                // Separated (speculative) points are solved rigidly (see
                // the twist-friction `update`).
                normal_part.cfm_factor =
                    cfm_factor.select(dist.simd_le(SimdReal::zero()), SimdReal::splat(1.0));
                normal_part.impulse_accumulator += normal_part.impulse;
                normal_part.impulse *= warmstart_coeff;
            }

            // tangent parts.
            {
                tangent_part.impulse_accumulator += tangent_part.impulse;
                tangent_part.impulse *= warmstart_coeff;

                for j in 0..DIM - 1 {
                    let bias = (p1 - p2).gdot(tangents1[j]) * inv_dt;
                    tangent_part.rhs[j] = tangent_part.rhs_wo_bias[j] + bias;
                }
            }
        }

        constraint.cfm_factor = cfm_factor;
    }

    /// Relax-pass refresh: recompute the unbiased rhs (speculative term included)
    /// from the CURRENT solver poses and strip softness and penetration bias. See
    /// `ContactWithTwistFrictionBuilder::refresh_rhs_wo_bias` for why stale separations are unusable.
    pub fn refresh_rhs_wo_bias(
        &self,
        params: &IntegrationParameters,
        solved_dt: Real,
        bodies: &SolverBodies,
        constraint: &mut ContactWithCoulombFriction<SimdReal>,
    ) {
        let inv_dt = SimdReal::splat(params.inv_dt());
        let poses1 = bodies.gather_transforms(constraint.solver_vel1);
        let poses2 = bodies.gather_transforms(constraint.solver_vel2);
        let all_infos = &self.infos[..constraint.num_contacts as usize];
        let normal_parts = &mut constraint.normal_part[..constraint.num_contacts as usize];
        let tangent_parts = &mut constraint.tangent_part[..constraint.num_contacts as usize];
        let solved_dt = SimdReal::splat(solved_dt);

        for ((info, normal_part), tangent_part) in all_infos
            .iter()
            .zip(normal_parts.iter_mut())
            .zip(tangent_parts.iter_mut())
        {
            let p1 = poses1.transform_point(info.local_p1) + info.tangent_vel * solved_dt;
            let p2 = poses2.transform_point(info.local_p2);
            let dist = info.dist + (p1 - p2).gdot(constraint.dir1);
            // See `update`: the restitution seed is applied by `apply_restitution`, not here.
            normal_part.rhs = dist.simd_max(SimdReal::zero()) * inv_dt;
            normal_part.cfm_factor = SimdReal::splat(1.0);
            tangent_part.rhs = tangent_part.rhs_wo_bias;
        }

        constraint.cfm_factor = SimdReal::splat(1.0);
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
        constraint: &mut ContactWithCoulombFriction<SimdReal>,
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
pub(crate) struct ContactWithCoulombFriction<N: ScalarType> {
    pub dir1: N::Vector, // Non-penetration force direction for the first body.
    pub im1: N::Vector,
    pub im2: N::Vector,
    pub cfm_factor: N,
    pub limit: N,

    #[cfg(feature = "dim3")]
    pub tangent1: N::Vector, // One of the friction force directions.
    pub normal_part: [ContactConstraintNormalPart<N>; MAX_MANIFOLD_POINTS],
    pub tangent_part: [ContactConstraintTangentPart<N>; MAX_MANIFOLD_POINTS],
    pub solver_vel1: [u32; SIMD_WIDTH],
    pub solver_vel2: [u32; SIMD_WIDTH],
    pub manifold_id: [ContactRef; SIMD_WIDTH],
    pub num_contacts: u8,
    pub manifold_contact_id: [[u8; SIMD_WIDTH]; MAX_MANIFOLD_POINTS],
}

impl ContactWithCoulombFriction<SimdReal> {
    pub fn warmstart(&mut self, bodies: &mut SolverBodies) {
        let mut solver_vel1 = bodies.gather_vels(self.solver_vel1);
        let mut solver_vel2 = bodies.gather_vels(self.solver_vel2);

        let normal_parts = &mut self.normal_part[..self.num_contacts as usize];
        let tangent_parts = &mut self.tangent_part[..self.num_contacts as usize];

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
        #[cfg(feature = "dim3")]
        let tangents1 = [&self.tangent1, &self.dir1.gcross(self.tangent1)];
        #[cfg(feature = "dim2")]
        let tangents1 = [&self.dir1.orthonormal_vector()];

        for tangent_part in tangent_parts.iter_mut() {
            tangent_part.warmstart(
                tangents1,
                &self.im1,
                &self.im2,
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
        let tangent_parts = &mut self.tangent_part[..self.num_contacts as usize];

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

                    ContactConstraintNormalPart::solve_pair(
                        normal_part_a,
                        normal_part_b,
                        &self.dir1,
                        &self.im1,
                        &self.im2,
                        &mut solver_vel1,
                        &mut solver_vel2,
                    );
                }

                // There is one constraint left to solve if there isn’t an even number.
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
            #[cfg(feature = "dim3")]
            let tangents1 = [&self.tangent1, &self.dir1.gcross(self.tangent1)];
            #[cfg(feature = "dim2")]
            let tangents1 = [&self.dir1.orthonormal_vector()];

            for (tangent_part, normal_part) in tangent_parts.iter_mut().zip(normal_parts.iter()) {
                let limit = self.limit * normal_part.impulse;
                tangent_part.solve(
                    tangents1,
                    &self.im1,
                    &self.im2,
                    limit,
                    &mut solver_vel1,
                    &mut solver_vel2,
                );
            }
        }

        bodies.scatter_vels(self.solver_vel1, solver_vel1);
        bodies.scatter_vels(self.solver_vel2, solver_vel2);
    }

    pub fn writeback_impulses(&self, manifolds_all: &ManifoldStore) {
        // World-space friction impulse basis (see
        // `ContactData::warmstart_tangent_world`).
        #[cfg(feature = "dim3")]
        let tangent2 = self.dir1.gcross(self.tangent1);
        for k in 0..self.num_contacts as usize {
            // The stored impulses are serialized state: canonicalize signed zeros (see
            // `utils::canonicalize_zero`) so snapshots stay cross-platform deterministic.
            let warmstart_impulses: [_; SIMD_WIDTH] =
                utils::canonicalize_zero(self.normal_part[k].impulse).into();
            let warmstart_tangent_impulses = utils::canonicalize_zero(self.tangent_part[k].impulse);
            #[cfg(feature = "dim3")]
            let warmstart_tangent_world = utils::canonicalize_zero(
                self.tangent1 * warmstart_tangent_impulses.x
                    + tangent2 * warmstart_tangent_impulses.y,
            );
            #[cfg(feature = "dim3")]
            let (wx, wy, wz): (
                [Real; SIMD_WIDTH],
                [Real; SIMD_WIDTH],
                [Real; SIMD_WIDTH],
            ) = (
                warmstart_tangent_world.x.into(),
                warmstart_tangent_world.y.into(),
                warmstart_tangent_world.z.into(),
            );
            let impulses: [_; SIMD_WIDTH] =
                utils::canonicalize_zero(self.normal_part[k].total_impulse()).into();
            let tangent_impulses = utils::canonicalize_zero(self.tangent_part[k].total_impulse());

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
                    active_contact.data.warmstart_tangent_impulse =
                        warmstart_tangent_impulses.extract(ii);
                    #[cfg(feature = "dim3")]
                    {
                        active_contact.data.warmstart_tangent_world =
                            crate::math::Vector::new(wx[ii], wy[ii], wz[ii]);
                    }
                    active_contact.data.impulse = impulses[ii];
                    active_contact.data.tangent_impulse = tangent_impulses.extract(ii);
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

    #[cfg(feature = "dim2")]
    fn vect(x: Real, y: Real) -> Vector {
        Vector::new(x, y)
    }
    #[cfg(feature = "dim3")]
    fn vect(x: Real, y: Real) -> Vector {
        Vector::new(x, y, 0.3 * x - 0.1 * y)
    }
    #[cfg(feature = "dim2")]
    fn up() -> Vector {
        Vector::new(0.0, 1.0)
    }
    #[cfg(feature = "dim3")]
    fn up() -> Vector {
        Vector::new(0.0, 1.0, 0.0)
    }

    /// A world-attached-on-both-sides manifold with `n` distinct contacts and non-trivial
    /// warm-start data (both solver-body ids `u32::MAX`, so gathers read identity/zero defaults
    /// — `generate` only needs manifold-side inputs to exercise the per-lane masking).
    fn test_manifold(n: usize, seed: Real) -> ContactManifold {
        let mut m = ContactManifold::new();
        m.data.normal = up();
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
            #[cfg(feature = "dim3")]
            {
                pt.data.warmstart_tangent_impulse[1] = seed * 0.5 + kf;
            }
            pt.data.impulse = 1.0; // Not "new": keeps restitution handling inert.
            m.points.push(pt);
            m.data.solver_contacts.push(SolverContact {
                anchor1: vect(0.3 * kf + seed, 0.5),
                anchor2: vect(0.3 * kf + seed, -0.5),
                dist: -0.01,
                tangent_velocity: vect(0.0, 0.0) * 0.0,
                contact_id: [k as crate::geometry::ContactId],
                #[cfg(feature = "dim3")]
                padding: [0.0],
            });
        }
        m
    }

    fn generate_chunk(
        manifolds: [&ContactManifold; SIMD_WIDTH],
    ) -> ContactWithCoulombFriction<SimdReal> {
        let bodies = RigidBodySet::new();
        let solver_bodies = SolverBodies::default();
        // Zero-init exactly like the production constraint arenas.
        let mut builder: ContactWithCoulombFrictionBuilder = unsafe { core::mem::zeroed() };
        let mut constraint: ContactWithCoulombFriction<SimdReal> = unsafe { core::mem::zeroed() };
        let ids: [ContactRef; SIMD_WIDTH] = core::array::from_fn(|ii| ContactRef {
            edge: ii as u32,
            manifold: 0,
        });
        ContactWithCoulombFrictionBuilder::generate(
            ids,
            manifolds,
            &bodies,
            &solver_bodies,
            &mut builder,
            &mut constraint,
        );
        constraint
    }

    /// The masking property the neutral-fill fixup must uphold: on a mixed-count chunk, every
    /// ACTIVE `(point, lane)` slot is bit-identical to a count-uniform chunk's, and every
    /// INACTIVE slot holds exactly the neutral values (zero warm-start/effective mass, inert
    /// tangent `r`, `u8::MAX` writeback sentinel, degraded 2×2 block elements).
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
                let tp = &cm.tangent_part[k];
                if k < count {
                    // Active slots: bit-identical to the uniform reference lane.
                    let (unp, utp) = (&uni.normal_part[k], &uni.tangent_part[k]);
                    assert_eq!(np.r.extract(ii), unp.r.extract(ii));
                    assert_eq!(np.impulse.extract(ii), unp.impulse.extract(ii));
                    assert_eq!(
                        cm.manifold_contact_id[k][ii],
                        uni.manifold_contact_id[k][ii]
                    );
                    for j in 0..DIM - 1 {
                        assert_eq!(tp.impulse[j].extract(ii), utp.impulse[j].extract(ii));
                        assert_eq!(tp.r[j].extract(ii), utp.r[j].extract(ii));
                        assert_eq!(tp.rhs[j].extract(ii), utp.rhs[j].extract(ii));
                    }
                    #[cfg(feature = "dim3")]
                    assert_eq!(tp.r[2].extract(ii), utp.r[2].extract(ii));
                } else {
                    // Inactive slots: the exact neutral fill.
                    assert_eq!(cm.manifold_contact_id[k][ii], u8::MAX);
                    assert_eq!(np.r.extract(ii), 0.0);
                    assert_eq!(np.impulse.extract(ii), 0.0);
                    for j in 0..DIM - 1 {
                        assert_eq!(tp.impulse[j].extract(ii), 0.0);
                    }
                    #[cfg(feature = "dim2")]
                    assert_eq!(tp.r[0].extract(ii), 0.0);
                    #[cfg(feature = "dim3")]
                    {
                        assert_eq!(tp.r[0].extract(ii), 1.0);
                        assert_eq!(tp.r[1].extract(ii), 1.0);
                        assert_eq!(tp.r[2].extract(ii), 0.0);
                    }
                }
            }

            #[cfg(feature = "block-solver")]
            {
                for kp in 0..MAX_MANIFOLD_POINTS / 2 {
                    let (k0, k1) = (kp * 2, kp * 2 + 1);
                    let elts0 = &cm.normal_part[k0].r_mat_elts;
                    let elts1 = &cm.normal_part[k1].r_mat_elts;
                    if k1 < count {
                        let u0 = &uni.normal_part[k0].r_mat_elts;
                        let u1 = &uni.normal_part[k1].r_mat_elts;
                        assert_eq!(elts0[0].extract(ii), u0[0].extract(ii));
                        assert_eq!(elts0[1].extract(ii), u0[1].extract(ii));
                        assert_eq!(elts1[0].extract(ii), u1[0].extract(ii));
                        assert_eq!(elts1[1].extract(ii), u1[1].extract(ii));
                    } else {
                        // Degraded pair: `[r0, 0]` / `[0, 0]` with r0 = the
                        // (post-fixup) effective mass of the pair's first point
                        // (zero when that point is inactive too).
                        assert_eq!(elts0[0].extract(ii), cm.normal_part[k0].r.extract(ii));
                        assert_eq!(elts0[1].extract(ii), 0.0);
                        assert_eq!(elts1[0].extract(ii), 0.0);
                        assert_eq!(elts1[1].extract(ii), 0.0);
                    }
                }
            }
        }
    }
}
