//! The stage machine executed by every worker: solver-body init, constraint
//! generation, the per-group substep loop, and the impulse/rigid-body
//! writeback stages.

use core::sync::atomic::Ordering;

#[cfg(feature = "dim3")]
use crate::dynamics::rigid_body::gyroscopic_corrected_angvel;
use crate::dynamics::solver::JointConstraintsSet;
use crate::dynamics::solver::contact_constraint::ContactWithCoulombFrictionBuilder;
#[cfg(feature = "dim3")]
use crate::dynamics::solver::contact_constraint::ContactWithTwistFrictionBuilder;
use crate::dynamics::solver::joint_constraint::GenericJointConstraintBuilder;
use crate::dynamics::solver::solver_body::SOLVER_BODY_ALLOW_FAST_ROTATION;
use crate::dynamics::solver::solver_contact_graph::ContactRef;
use crate::dynamics::{JointGraphEdge, RigidBodyType, RigidBodyVelocity};
use crate::geometry::ContactManifold;
use crate::math::Real;
use parry::math::SIMD_WIDTH;

use super::solve::solve_pass;
use super::{BODY_BATCH, SharedCtx, chunk_at, stage_batch};

/// Per-full-step angular rotation cap (0.25π ≈ 45°): bodies without
/// `allow_fast_rotation` get their angular velocity clamped each substep so the full-step
/// rotation stays under this (≥ 0.5π per step breaks CCD).
// The cast is a no-op in f64 mode but narrows in f32 mode, so it has to stay.
#[allow(clippy::unnecessary_cast)]
const MAX_ROTATION: Real = core::f64::consts::FRAC_PI_4 as Real;

/// The stage machine executed by every worker. See [`SharedCtx`] for the safety contract.
pub(super) unsafe fn run_worker(ctx: &SharedCtx, worker_id: usize) {
    let sync = ctx.sync;
    let num_chunks = ctx.num_chunks;
    let all_chunks = 0..num_chunks;
    let all_island_bodies = 0..ctx.island_bodies.len();
    // The caller's stage ordinal; every worker passes the same sequence of
    // stage sites, so ordinals agree by construction. See `StageSync`.
    let mut stage = 0;

    /*
     * Stage: initialize solver bodies and their velocity increments (parallel over
     * claimed body slots; each writes solver-body slot i for island body i).
     * Multibodies were initialized in the serial pre-phase.
     */
    {
        let all_fill_slots = 0..ctx.island_bodies.len();
        let stage_work = all_fill_slots.end;
        let mut done = 0;
        while let Some(claimed) = sync.claim(stage, &all_fill_slots, BODY_BATCH, worker_id) {
            let claimed_len = claimed.len();
            for i in claimed {
                let handle = ctx.island_bodies[i];
                if ctx.has_multibodies {
                    let multibodies = unsafe { &*ctx.multibodies };
                    if multibodies.rigid_body_link(handle).is_some() {
                        continue;
                    }
                }

                let bodies = unsafe { &*ctx.bodies };
                let rb = &bodies[handle];
                debug_assert_eq!(rb.ids.active_set_id, i as u32);
                let vs = unsafe { &mut *ctx.velocity_solver };
                vs.solver_bodies.copy_from(i, rb);

                // The per-substep external-force increment, baked with the
                // slot's group substep dt (groups are few; the scan is cheap).
                let slot_dt = ctx
                    .groups
                    .iter()
                    .find(|g| g.bodies.contains(&i))
                    .map(|g| g.dt)
                    .unwrap_or(ctx.groups[0].dt);
                let incr = &mut vs.solver_vels_increment[i];
                incr.angular = rb.mprops.effective_world_inv_inertia * rb.forces.torque * slot_dt;
                incr.linear = rb.forces.force * rb.mprops.effective_inv_mass * slot_dt;

                // Gyroscopic forces (3D) are applied per-substep by the increment
                // stage; record the body's diagonal local inertia here.
                #[cfg(feature = "dim3")]
                {
                    let gyro = &mut vs.solver_gyro[i];
                    // Only dynamic bodies: a kinematic body's velocity is
                    // user-prescribed and must not be altered by the solver.
                    if rb.forces.gyroscopic_forces_enabled && rb.is_dynamic() && !rb.is_sleeping() {
                        gyro.principal_inertia = rb.mprops.local_mprops.principal_inertia();
                        gyro.inv_principal_inertia = rb.mprops.local_mprops.inv_principal_inertia;
                        gyro.principal_frame = rb.mprops.local_mprops.principal_inertia_local_frame;
                        gyro.enabled = true;
                        // SAFETY: shared atomic; claimed slots make the writes disjoint.
                        unsafe { &*ctx.any_gyroscopic }.store(true, Ordering::Relaxed);
                    } else {
                        gyro.enabled = false;
                    }
                }
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, stage_work);
        stage = sync.sync(stage, stage_work);
    }

    /*
     * Stage: generate SIMD contact constraints (parallel over claimed chunk slots).
     */
    {
        let bodies = unsafe { &*ctx.bodies };
        let mut done = 0;
        let mut local_bouncy = false;
        while let Some(claimed) = sync.claim(
            stage,
            &all_chunks,
            stage_batch(all_chunks.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            let claim_end = claimed.end;
            for chunk_id in claimed {
                // Software prefetch: `store.get` walks a dependent edge → pair → manifold chain
                // per lane and the random-order misses otherwise serialize. Pair headers at
                // distance 2, manifolds at distance 1 (their address needs the header in cache).
                if chunk_id + 2 < claim_end {
                    for r in chunk_at(ctx.chunk_segments, chunk_id + 2) {
                        ctx.store.prefetch_pair(r);
                    }
                }
                if chunk_id + 1 < claim_end {
                    for r in chunk_at(ctx.chunk_segments, chunk_id + 1) {
                        ctx.store.prefetch_manifold(r);
                    }
                }
                // The chunk lanes are the (edge, ordinal) addresses the
                // constraint stores per lane (PADDING for unused lanes, whose
                // manifold gather replicates lane 0).
                let cref_ids = chunk_at(ctx.chunk_segments, chunk_id);
                let manifold_refs: [&ContactManifold; SIMD_WIDTH] = core::array::from_fn(|ii| {
                    let r = if cref_ids[ii].is_padding() {
                        cref_ids[0]
                    } else {
                        cref_ids[ii]
                    };
                    ctx.store.get(r)
                });
                // SAFETY: reads solver bodies (no other writer during this stage),
                //         writes the claimed constraint slot only.
                let solver_bodies = unsafe { &(*ctx.velocity_solver).solver_bodies };

                #[cfg(feature = "dim3")]
                if ctx.use_twist {
                    let builder = unsafe { &mut *ctx.twist_builders.add(chunk_id) };
                    let constraint = unsafe { &mut *ctx.twist_constraints.add(chunk_id) };
                    ContactWithTwistFrictionBuilder::generate(
                        cref_ids,
                        manifold_refs,
                        bodies,
                        solver_bodies,
                        builder,
                        constraint,
                    );
                    local_bouncy |= builder.has_bouncy_seed(constraint.num_contacts);
                }
                if !ctx.use_twist {
                    let builder = unsafe { &mut *ctx.coulomb_builders.add(chunk_id) };
                    let constraint = unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
                    ContactWithCoulombFrictionBuilder::generate(
                        cref_ids,
                        manifold_refs,
                        bodies,
                        solver_bodies,
                        builder,
                        constraint,
                    );
                    local_bouncy |= builder.has_bouncy_seed(constraint.num_contacts);
                }
            }
            done += claimed_len;
        }
        // Publish this worker's restitution-seed sightings BEFORE the completion flush so
        // every worker reads a settled value after the stage barrier.
        if local_bouncy {
            unsafe { &*ctx.any_bouncy }.store(true, Ordering::Relaxed);
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, num_chunks);
        stage = sync.sync(stage, num_chunks);
    }

    /*
     * Group ring: each substep solve-group runs its whole substep loop in turn on its own
     * slots/chunks/colors/builders. Groups are ordered by decreasing substep count, so
     * lower-cadence groups see higher-cadence bodies at end-of-step poses; every worker walks
     * the identical group/substep/stage sequence, keeping the `StageSync` ordinals in lockstep.
     */
    // Whether any island body has gyroscopic forces enabled. Set by the body-copy
    // stage above and published by its barrier, so every worker reads the same
    // value; gyro-free islands (the common case) skip the per-substep pass.
    #[cfg(feature = "dim3")]
    let has_gyroscopic = unsafe { &*ctx.any_gyroscopic }.load(Ordering::Relaxed);
    // Whether any contact constraint captured a restitution seed, published by the
    // constraint-generation barrier so every worker agrees. Bounce-free steps (the common
    // case) skip the end-of-step restitution stages entirely.
    let has_bouncy = unsafe { &*ctx.any_bouncy }.load(Ordering::Relaxed);
    for group in ctx.groups {
        // This group's substep parameters: identical to `base_params` except
        // for the substep dt. Everything dt-derived downstream (soft erp/cfm,
        // rhs, speculative terms, integration) recomputes from this copy.
        let mut gparams = *ctx.base_params;
        gparams.dt = group.dt;
        let params = &gparams;
        let num_substeps = group.num_substeps;
        // This group's claim domains; single-group they span everything, except `group_bodies`
        // (island slots only — the omitted padding tail slots hold zero velocities and
        // increments, so skipping them is behavior-preserving).
        let group_bodies = group.bodies.clone();
        let num_group_bodies = group_bodies.len();
        let num_joint_chunks = group.joint_chunks.len();
        let joint_builders = group.joint_builders.clone();
        // Update-stage virtual claim domain: this group's contact chunks,
        // then its SIMD joint chunks, then its scalar joint builders.
        let group_chunks_len = group.chunks.len();
        let all_updates = 0..group_chunks_len + num_joint_chunks + joint_builders.len();

        for substep_id in 0..num_substeps {
            let is_last_substep = substep_id == num_substeps - 1;
            let solved_dt = substep_id as Real * params.dt;

            /*
             * Stage: apply velocity increments (parallel over claimed body slots).
             * Worker 0 also applies the generic (multibody) increments.
             */
            {
                // +1: worker 0's generic increments.
                let stage_work = num_group_bodies + 1;
                let mut done = 0;
                while let Some(claimed) = sync.claim(stage, &group_bodies, BODY_BATCH, worker_id) {
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    let claimed_len = claimed.len();
                    for i in claimed {
                        let incr = vs.solver_vels_increment[i];
                        {
                            let vels = &mut vs.solver_bodies.vels[i];
                            vels.linear += incr.linear;
                            vels.angular += incr.angular;
                        }

                        // Per-substep gyroscopic correction (applied every substep
                        // to keep long, skinny bodies stable), using the
                        // orientation integrated by the previous substep.
                        #[cfg(feature = "dim3")]
                        if has_gyroscopic {
                            let gyro = vs.solver_gyro[i];
                            if gyro.enabled {
                                // World-space principal axes = body rotation ∘ principal frame.
                                let principal_axes =
                                    vs.solver_bodies.poses[i].rotation * gyro.principal_frame;
                                let angular = vs.solver_bodies.vels[i].angular;
                                vs.solver_bodies.vels[i].angular = gyroscopic_corrected_angvel(
                                    angular,
                                    principal_axes,
                                    gyro.principal_inertia,
                                    gyro.inv_principal_inertia,
                                    params.dt,
                                );
                            }
                        }
                    }
                    done += claimed_len;
                }
                // One completion flush per worker per stage: per-batch RMWs on the
                // shared counter measurably throttle scalar builds (4x the chunks).
                sync.complete(stage, done, stage_work);
                if worker_id == 0 {
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    let incr = core::mem::take(&mut vs.generic_solver_vels_increment);
                    vs.generic_solver_vels += &incr;
                    vs.generic_solver_vels_increment = incr;
                    sync.complete(stage, 1, stage_work);
                }
                stage = sync.sync(stage, stage_work);
            }

            /*
             * Stage: update constraints from builders (parallel; contact chunks first, then
             * scalar joint builders — each writes only its own constraint slot/rows).
             * Worker 0 also updates the generic joint and generic contact constraints.
             */
            {
                let multibodies = unsafe { &*ctx.multibodies };
                // NOTE: when warm-starting is enabled (the common case), the contact
                //       constraint updates are fused into the warmstart stage below so
                //       each constraint is only swept once per substep.
                let fused_contact_update = params.warmstart_coefficient != 0.0;
                let update_domain = if fused_contact_update {
                    group_chunks_len..all_updates.end
                } else {
                    all_updates.clone()
                };
                let stage_work = update_domain.len() + 1;
                let mut done = 0;
                while let Some(claimed) = sync.claim(
                    stage,
                    &update_domain,
                    stage_batch(update_domain.end - update_domain.start, sync.num_workers),
                    worker_id,
                ) {
                    let solver_bodies = unsafe { &(*ctx.velocity_solver).solver_bodies };
                    let claimed_len = claimed.len();
                    for idx in claimed {
                        if idx >= group_chunks_len + num_joint_chunks {
                            // SAFETY: each builder is claimed by exactly one worker per
                            // stage and only writes itself (substep-0 GS recording) and
                            // its own constraint rows.
                            let joints = unsafe { &mut *ctx.joint_constraints };
                            let builder = unsafe {
                                &mut *joints.velocity_constraints_builder.as_mut_ptr().add(
                                    joint_builders.start
                                        + (idx - group_chunks_len - num_joint_chunks),
                                )
                            };
                            let warmstart_joints = params
                                .warmstart_joints
                                .then_some(params.warmstart_coefficient);
                            builder.update(
                                params,
                                substep_id,
                                warmstart_joints,
                                solver_bodies,
                                &mut joints.velocity_constraints,
                            );
                            continue;
                        }
                        if idx >= group_chunks_len {
                            // SIMD joint chunk of this group.
                            // SAFETY: same as the scalar builders.
                            let joints = unsafe { &mut *ctx.joint_constraints };
                            let builder = unsafe {
                                &mut *joints
                                    .simd_velocity_constraints_builder
                                    .as_mut_ptr()
                                    .add(group.joint_chunks.start + (idx - group_chunks_len))
                            };
                            let warmstart_joints = params
                                .warmstart_joints
                                .then_some(params.warmstart_coefficient);
                            builder.update(
                                params,
                                substep_id,
                                warmstart_joints,
                                solver_bodies,
                                &mut joints.simd_velocity_constraints,
                            );
                            continue;
                        }

                        let chunk_id = group.chunks.start + idx;
                        #[cfg(feature = "dim3")]
                        if ctx.use_twist {
                            let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                            builder.update(params, solved_dt, solver_bodies, multibodies, unsafe {
                                &mut *ctx.twist_constraints.add(chunk_id)
                            });
                        }
                        if !ctx.use_twist {
                            let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                            builder.update(params, solved_dt, solver_bodies, multibodies, unsafe {
                                &mut *ctx.coulomb_constraints.add(chunk_id)
                            });
                        }
                    }
                    done += claimed_len;
                }
                // One completion flush per worker per stage: per-batch RMWs on the
                // shared counter measurably throttle scalar builds (4x the chunks).
                sync.complete(stage, done, stage_work);

                if worker_id == 0 {
                    // Generic (multibody) constraints exist only on the
                    // single-group path (multibody scenes never split).
                    if ctx.groups.len() == 1 {
                        let joints = unsafe { &mut *ctx.joint_constraints };
                        let solver_bodies = unsafe { &(*ctx.velocity_solver).solver_bodies };
                        let JointConstraintsSet {
                            generic_velocity_constraints_builder,
                            generic_jacobians,
                            generic_velocity_constraints,
                            ..
                        } = joints;
                        for builder in generic_velocity_constraints_builder.iter_mut() {
                            match builder {
                                GenericJointConstraintBuilder::External(b) => b.update(
                                    params,
                                    multibodies,
                                    solver_bodies,
                                    generic_jacobians,
                                    generic_velocity_constraints,
                                ),
                                GenericJointConstraintBuilder::Internal(b) => b.update(
                                    params,
                                    multibodies,
                                    generic_jacobians,
                                    generic_velocity_constraints,
                                ),
                                GenericJointConstraintBuilder::Empty => {}
                            }
                        }

                        if !fused_contact_update {
                            let contacts = unsafe { &mut *ctx.contact_constraints };
                            let solver_bodies = unsafe { &(*ctx.velocity_solver).solver_bodies };
                            for (builder, constraint) in contacts
                                .generic_velocity_constraints_builder
                                .iter()
                                .zip(contacts.generic_velocity_constraints.iter_mut())
                            {
                                builder.update(
                                    params,
                                    solved_dt,
                                    solver_bodies,
                                    multibodies,
                                    constraint,
                                );
                            }
                        }
                    }
                    sync.complete(stage, 1, stage_work);
                }
                stage = sync.sync(stage, stage_work);
            }

            /*
             * Stages: warmstart, color by color (parallel within a color: constraints of a
             * same color write disjoint solver bodies).
             */
            if params.warmstart_coefficient != 0.0 {
                let multibodies = unsafe { &*ctx.multibodies };
                for (_, color) in &ctx.color_ranges[group.colors.clone()] {
                    let mut done = 0;
                    while let Some(claimed) = sync.claim(
                        stage,
                        color,
                        stage_batch(color.end - color.start, sync.num_workers),
                        worker_id,
                    ) {
                        let claimed_len = claimed.len();
                        for chunk_id in claimed {
                            let solver_bodies =
                                unsafe { &mut (*ctx.velocity_solver).solver_bodies };
                            // Fused per-substep constraint update + warmstart: one sweep
                            // over the constraint memory instead of two.
                            #[cfg(feature = "dim3")]
                            if ctx.use_twist {
                                let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                                let constraint =
                                    unsafe { &mut *ctx.twist_constraints.add(chunk_id) };
                                builder.update(
                                    params,
                                    solved_dt,
                                    solver_bodies,
                                    multibodies,
                                    constraint,
                                );
                                constraint.warmstart(solver_bodies);
                            }
                            if !ctx.use_twist {
                                let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                                let constraint =
                                    unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
                                builder.update(
                                    params,
                                    solved_dt,
                                    solver_bodies,
                                    multibodies,
                                    constraint,
                                );
                                constraint.warmstart(solver_bodies);
                            }
                        }
                        done += claimed_len;
                    }
                    // One completion flush per worker per stage: per-batch RMWs on the
                    // shared counter measurably throttle scalar builds (4x the chunks).
                    sync.complete(stage, done, color.len());
                    stage = sync.sync(stage, color.len());
                }

                // Overflow + generic (multibody) contacts, exclusively on worker 0
                // (fused update + warmstart, like the colored chunks above).
                if worker_id == 0 {
                    for chunk_id in group.overflow.clone() {
                        let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
                        #[cfg(feature = "dim3")]
                        if ctx.use_twist {
                            let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                            let constraint = unsafe { &mut *ctx.twist_constraints.add(chunk_id) };
                            builder.update(
                                params,
                                solved_dt,
                                solver_bodies,
                                multibodies,
                                constraint,
                            );
                            constraint.warmstart(solver_bodies);
                        }
                        if !ctx.use_twist {
                            let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                            let constraint = unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
                            builder.update(
                                params,
                                solved_dt,
                                solver_bodies,
                                multibodies,
                                constraint,
                            );
                            constraint.warmstart(solver_bodies);
                        }
                    }

                    let contacts = unsafe { &mut *ctx.contact_constraints };
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    for (builder, c) in contacts
                        .generic_velocity_constraints_builder
                        .iter()
                        .zip(contacts.generic_velocity_constraints.iter_mut())
                    {
                        builder.update(params, solved_dt, &vs.solver_bodies, multibodies, c);
                        c.warmstart(
                            &contacts.generic_jacobians,
                            &mut vs.solver_bodies,
                            &mut vs.generic_solver_vels,
                        );
                    }
                    sync.complete(stage, 1, 1);
                }
                stage = sync.sync(stage, 1);
            }

            /*
             * Stages: solve with bias.
             */
            for pgs_iter in 0..params.num_internal_pgs_iterations {
                // Joint warm-starting is fused into the first biased pass: each claimed
                // joint applies its carried impulse right before being solved (sound
                // within a color; Gauss-Seidel-ordered across colors).
                let warmstart_joints = params.warmstart_joints && pgs_iter == 0;
                stage = unsafe {
                    solve_pass(
                        ctx,
                        group,
                        worker_id,
                        stage,
                        false,
                        warmstart_joints,
                        params,
                        solved_dt,
                    )
                };
            }

            /*
             * Stage: integrate positions (parallel over claimed body slots).
             * Worker 0 also integrates the multibodies (sole accessor of `bodies` and
             * `multibodies` during this stage).
             */
            {
                // Per-substep speed cap. `max_ang` ties to the *full-step* inv_dt
                // so per-step rotation stays under `MAX_ROTATION` regardless of substep count;
                // clamped in place on the solver body so the cap persists into later substeps
                // and the final velocity writeback.
                let base_params = ctx.base_params;
                let max_lin = base_params.max_linear_velocity();
                let max_ang = MAX_ROTATION * base_params.inv_dt();

                let mut done = 0;
                while let Some(claimed) = sync.claim(stage, &group_bodies, BODY_BATCH, worker_id) {
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    let claimed_len = claimed.len();
                    for i in claimed {
                        {
                            let vel = &mut vs.solver_bodies.vels[i];
                            if max_lin != Real::MAX {
                                let n = vel.linear.length();
                                if n > max_lin {
                                    vel.linear *= max_lin / n;
                                }
                            }
                            if vs.solver_bodies.flags[i] & SOLVER_BODY_ALLOW_FAST_ROTATION == 0 {
                                #[cfg(feature = "dim2")]
                                if vel.angular.abs() > max_ang {
                                    vel.angular = vel.angular.signum() * max_ang;
                                }
                                #[cfg(feature = "dim3")]
                                {
                                    let n = vel.angular.length();
                                    if n > max_ang {
                                        vel.angular *= max_ang / n;
                                    }
                                }
                            }
                        }

                        let vels = vs.solver_bodies.vels[i];
                        let pose = &mut vs.solver_bodies.poses[i];
                        let new_vels = RigidBodyVelocity {
                            linvel: vels.linear,
                            angvel: vels.angular,
                        };
                        new_vels.integrate_linearized(
                            params.dt,
                            &mut pose.translation,
                            &mut pose.rotation,
                        );
                    }
                    done += claimed_len;
                }
                // One completion flush per worker per stage: per-batch RMWs on the
                // shared counter measurably throttle scalar builds (4x the chunks).
                sync.complete(stage, done, num_group_bodies + 1);

                if worker_id == 0 {
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    let bodies = unsafe { &mut *ctx.bodies };
                    let multibodies = unsafe { &mut *ctx.multibodies };
                    vs.integrate_multibody_positions(params, is_last_substep, bodies, multibodies);
                    sync.complete(stage, 1, num_group_bodies + 1);
                }
                stage = sync.sync(stage, num_group_bodies + 1);
            }

            /*
             * Stages: solve without bias.
             */
            for _ in 0..params.num_internal_stabilization_iterations {
                stage = unsafe {
                    solve_pass(
                        ctx,
                        group,
                        worker_id,
                        stage,
                        true,
                        false,
                        params,
                        solved_dt + params.dt,
                    )
                };
            }
        }

        /*
         * Stages: end-of-step restitution, color by color (box2d-style). Applied ONCE after
         * all substeps, gated on the point having carried an impulse: inside the substep rhs
         * the speculative `dist * inv_dt` slack would truncate the bounce.
         */
        if has_bouncy {
            for (_, color) in &ctx.color_ranges[group.colors.clone()] {
                let mut done = 0;
                while let Some(claimed) = sync.claim(
                    stage,
                    color,
                    stage_batch(color.end - color.start, sync.num_workers),
                    worker_id,
                ) {
                    let claimed_len = claimed.len();
                    for chunk_id in claimed {
                        // SAFETY: constraints of a same color touch pairwise-disjoint bodies.
                        let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
                        #[cfg(feature = "dim3")]
                        if ctx.use_twist {
                            let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                            builder.apply_restitution(
                                unsafe { &mut *ctx.twist_constraints.add(chunk_id) },
                                solver_bodies,
                            );
                        }
                        if !ctx.use_twist {
                            let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                            builder.apply_restitution(
                                unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) },
                                solver_bodies,
                            );
                        }
                    }
                    done += claimed_len;
                }
                // One completion flush per worker per stage (see the warmstart stages).
                sync.complete(stage, done, color.len());
                stage = sync.sync(stage, color.len());
            }

            // Overflow + generic (multibody) contacts, exclusively on worker 0.
            if worker_id == 0 {
                for chunk_id in group.overflow.clone() {
                    let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
                    #[cfg(feature = "dim3")]
                    if ctx.use_twist {
                        let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                        builder.apply_restitution(
                            unsafe { &mut *ctx.twist_constraints.add(chunk_id) },
                            solver_bodies,
                        );
                    }
                    if !ctx.use_twist {
                        let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                        builder.apply_restitution(
                            unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) },
                            solver_bodies,
                        );
                    }
                }

                // Generic constraints exist only on the single-group path.
                if ctx.groups.len() == 1 {
                    let contacts = unsafe { &mut *ctx.contact_constraints };
                    let vs = unsafe { &mut *ctx.velocity_solver };
                    for (builder, c) in contacts
                        .generic_velocity_constraints_builder
                        .iter()
                        .zip(contacts.generic_velocity_constraints.iter_mut())
                    {
                        builder.apply_restitution(
                            c,
                            &contacts.generic_jacobians,
                            &mut vs.solver_bodies,
                            &mut vs.generic_solver_vels,
                        );
                    }
                }
                sync.complete(stage, 1, 1);
            }
            stage = sync.sync(stage, 1);
        }
    }

    /*
     * Stage: write impulses back to the manifolds (parallel; each chunk writes only its own
     * manifolds, padding lanes skipped). Worker 0 also writes back the joint and generic
     * contact impulses (disjoint from the chunk manifolds).
     */
    {
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &all_chunks,
            stage_batch(all_chunks.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            let claim_end = claimed.end;
            // Same two-distance software prefetch as the constraint-generation
            // stage: the impulse writeback walks the same dependent
            // edge → pair → manifold chain per lane.
            let manifold_ids_of = |chunk_id: usize| -> [ContactRef; SIMD_WIDTH] {
                #[cfg(feature = "dim3")]
                if ctx.use_twist {
                    return unsafe { (*ctx.twist_constraints.add(chunk_id)).manifold_id };
                }
                unsafe { (*ctx.coulomb_constraints.add(chunk_id)).manifold_id }
            };
            for chunk_id in claimed {
                if chunk_id + 2 < claim_end {
                    for r in manifold_ids_of(chunk_id + 2) {
                        ctx.store.prefetch_pair(r);
                    }
                }
                if chunk_id + 1 < claim_end {
                    for r in manifold_ids_of(chunk_id + 1) {
                        ctx.store.prefetch_manifold(r);
                    }
                }
                #[cfg(feature = "dim3")]
                if ctx.use_twist {
                    unsafe { (*ctx.twist_constraints.add(chunk_id)).writeback_impulses(ctx.store) };
                }
                if !ctx.use_twist {
                    unsafe {
                        (*ctx.coulomb_constraints.add(chunk_id)).writeback_impulses(ctx.store)
                    };
                }
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, num_chunks + 1);

        if worker_id == 0 {
            let joints_all: &mut [JointGraphEdge] =
                unsafe { core::slice::from_raw_parts_mut(ctx.joints, ctx.num_joints) };
            let joint_constraints = unsafe { &mut *ctx.joint_constraints };
            joint_constraints.writeback_impulses(joints_all);

            let contacts = unsafe { &mut *ctx.contact_constraints };
            for c in contacts.generic_velocity_constraints.iter_mut() {
                c.writeback_impulses(ctx.store);
            }
            sync.complete(stage, 1, num_chunks + 1);
        }
        stage = sync.sync(stage, num_chunks + 1);
    }

    /*
     * Stage: write velocities and poses back to the rigid-bodies (parallel over
     * claimed island bodies; each writes its own rigid-body). The multibody
     * writeback runs on worker 0 afterwards, exclusively.
     */
    {
        let base_params = ctx.base_params;
        let mut done = 0;
        while let Some(claimed) = sync.claim(stage, &all_island_bodies, BODY_BATCH, worker_id) {
            let claimed_len = claimed.len();
            for i in claimed {
                let handle = ctx.island_bodies[i];
                if ctx.has_multibodies {
                    let multibodies = unsafe { &*ctx.multibodies };
                    if multibodies.rigid_body_link(handle).is_some() {
                        continue;
                    }
                }

                let bodies = unsafe { &mut *ctx.bodies };
                let rb = bodies.index_mut_internal(handle);
                let vs = unsafe { &*ctx.velocity_solver };
                let solver_vels = &vs.solver_bodies.vels[rb.ids.active_set_id as usize];
                let solver_poses = &vs.solver_bodies.poses[rb.ids.active_set_id as usize];

                let mut new_vels = RigidBodyVelocity {
                    linvel: solver_vels.linear,
                    angvel: solver_vels.angular,
                };
                new_vels = new_vels.apply_damping(base_params.dt, &rb.damping);
                rb.vels = new_vels;

                // NOTE: if it's a position-based kinematic body, don't writeback as we want
                //       to preserve exactly the value given by the user (it might not be
                //       exactly equal to the integrated position because of rounding errors).
                if rb.body_type != RigidBodyType::KinematicPositionBased {
                    let local_com = -rb.mprops.local_mprops.local_com;
                    rb.pos.next_position = solver_poses.pose().prepend_translation(local_com);
                }

                // Capture the post-solve velocity used by the CCD sweep — for *every* dynamic
                // body (fast bodies get CCD vs fixed colliders by default), skipped entirely
                // when CCD is globally off (`max_ccd_substeps == 0`) so those users pay nothing.
                if base_params.max_ccd_substeps != 0 && rb.is_dynamic() {
                    rb.ccd_vels = rb
                        .pos
                        .interpolate_velocity(base_params.inv_dt(), rb.local_center_of_mass());

                    // Fused post-solve CCD activation (fast-body criterion on the
                    // solved motion): replaces the pipeline's serial post-solve
                    // walk over every active body when no multibody is present.
                    let moving_fast = rb.ccd.is_moving_fast_with_next_position(
                        base_params.dt,
                        &rb.ccd_vels,
                        &rb.pos,
                        rb.mprops.local_mprops.local_com,
                        rb.mprops.max_extent(),
                    );
                    rb.ccd.ccd_active = moving_fast;
                    if moving_fast {
                        // SAFETY: shared atomic; claimed slots make the flag
                        // writes disjoint.
                        unsafe { &*ctx.any_ccd_active }.store(true, Ordering::Relaxed);
                    }
                } else {
                    rb.ccd_vels = RigidBodyVelocity::zero();
                }
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, all_island_bodies.end);
        // This sync is still needed: the loop above reads `ctx.multibodies`
        // (link lookups) while the block below takes it mutably.
        stage = sync.sync(stage, all_island_bodies.end);

        if worker_id == 0 && ctx.has_multibodies {
            let vs = unsafe { &mut *ctx.velocity_solver };
            let multibodies = unsafe { &mut *ctx.multibodies };
            for link in &vs.multibody_roots {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();
                let solver_vels = vs
                    .generic_solver_vels
                    .rows(multibody.solver_id as usize, multibody.ndofs());
                multibody.velocities.copy_from(&solver_vels);
            }
        }
        // No sync after the multibody-velocity writeback: nothing follows, and
        // the caller's scope join is the final synchronization point.
        let _ = stage;
    }
}
