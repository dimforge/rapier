//! One solve pass of a substep: soft-body prepare, joint colors, worker-0 joint overflow,
//! soft-body colors/shape constraints/serial constraints, contact colors, then the worker-0 contact
//! overflow (all completion-gated stages).

use crate::dynamics::IntegrationParameters;
use crate::dynamics::solver::JointConstraintsSet;
use crate::math::Real;

use super::{SharedCtx, stage_batch};

/// One solve pass: joints (colored, then overflow + generic on worker 0), soft-body rows, then
/// contact colors (parallel); returns the updated stage ordinal. `soft_update` updates the soft
/// constraints from the poses first; `soft_warmstart` is `Some(coefficient)` to warm-start them.
pub(super) unsafe fn solve_pass(
    ctx: &SharedCtx,
    group_index: usize,
    worker_id: usize,
    mut stage: usize,
    wo_bias: bool,
    warmstart_joints: bool,
    soft_update: bool,
    soft_warmstart: Option<Real>,
    params: &IntegrationParameters,
    solved_dt: Real,
) -> usize {
    let group = &ctx.groups[group_index];
    let sync = ctx.sync;
    // Soft-body work of this group. The set is shared and immutable in shape during the solve, so
    // every worker takes the same branches (identical stage sequences).
    // Soft rows are springs solved once per substep, in the biased pass only: solving them
    // again in the relax pass (with or without their bias) leaves the two passes fighting over
    // the same impulse, and the velocity read out after the relax pass carries the difference.
    let soft_ref = unsafe { &*ctx.soft_constraints };
    let has_soft = !soft_ref.is_empty() && !wo_bias;
    // Particle attachments are joints: solved in both passes, right after the joints.
    let has_attachments =
        !soft_ref.is_empty() && !soft_ref.groups[group_index].attachments.is_empty();
    /*
     * Stage: soft-body prepare (parallel over the group's awake soft bodies): shape-matching
     * goals and global-volume gradients from the current poses.
     */
    if has_soft && soft_ref.group_needs_prepare(group_index, soft_update) {
        let awake = soft_ref.groups[group_index].awake.clone();
        let stage_work = awake.len();
        let mut done = 0;
        while let Some(claimed) = sync.claim(stage, &awake, 1, worker_id) {
            // SAFETY: each awake body is claimed by exactly one worker per stage, and the
            // prepare only writes that body's own records and its own particles' velocities
            // (soft bodies share no particle).
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            let soft = unsafe { &mut *ctx.soft_constraints };
            let claimed_len = claimed.len();
            for ai in claimed {
                soft.prepare_awake_body(ai, solver_bodies, soft_update);
            }
            done += claimed_len;
        }
        sync.complete(stage, done, stage_work);
        stage = sync.sync(stage, stage_work);
    }

    /*
     * Stage: gather the active soft-body clusters (worker 0, serial): each cluster's proxy slot is
     * updated from its particles' velocities (keeping unscattered impulses) so the joints below
     * pull on the cluster's true rigid state. Runs in both passes, like the joints.
     */
    let has_clusters = !soft_ref.is_empty() && soft_ref.group_has_clusters(group_index);
    if has_clusters {
        if worker_id == 0 {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            soft.gather_clusters(group_index, solver_bodies);
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    // Optionally, friction can be solved only in the unbiased pass
    // ("no friction when applying bias"), unless there is no unbiased pass.
    let solve_friction = wo_bias
        || ctx.base_params.friction_in_bias_pass
        || ctx.base_params.num_internal_stabilization_iterations == 0;

    // Helpers solving one scalar joint (all its rows), one SIMD joint chunk, or
    // one contact chunk.
    let solve_joint = |joint_id: usize| {
        // SAFETY: constraints of a same color (or claimed by worker 0's exclusive
        // overflow stage) touch bodies no other concurrent constraint touches.
        let vs = unsafe { &mut *ctx.velocity_solver };
        let joints = unsafe { &mut *ctx.joint_constraints };
        for row in ctx.joint_rows[joint_id].clone() {
            let c = &mut joints.velocity_constraints[row];
            if wo_bias {
                c.remove_bias_from_rhs();
            }
            if warmstart_joints {
                c.warmstart(&mut vs.solver_bodies);
            }
            c.solve(&mut vs.solver_bodies);
        }
    };
    let solve_joint_chunk = |chunk_id: usize| {
        // SAFETY: same argument as `solve_joint`; the lanes of a chunk share a color, so are
        // pairwise body-disjoint (padding lanes replicate lane 0 and recompute its exact
        // values, so their duplicated scatter is value-identical).
        let vs = unsafe { &mut *ctx.velocity_solver };
        let joints = unsafe { &mut *ctx.joint_constraints };
        for row in ctx.joint_chunk_rows[chunk_id].clone() {
            let c = &mut joints.simd_velocity_constraints[row];
            if wo_bias {
                c.remove_bias_from_rhs();
            }
            if warmstart_joints {
                c.warmstart(&mut vs.solver_bodies);
            }
            c.solve(&mut vs.solver_bodies);
        }
    };
    let solve_chunk = |chunk_id: usize| {
        // SAFETY: same argument as `solve_joint`.
        let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
        #[cfg(feature = "dim3")]
        if ctx.use_twist {
            let c = unsafe { &mut *ctx.twist_constraints.add(chunk_id) };
            if wo_bias {
                // Positions were integrated after the biased pass: update the unbiased rhs
                // from the current poses (reusing pre-integration separations
                // destabilizes stack rocking).
                let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                builder.update_rhs_wo_bias(params, solved_dt, solver_bodies, c);
            }
            c.solve(solver_bodies, true, solve_friction);
        }
        if !ctx.use_twist {
            let c = unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
            if wo_bias {
                let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                builder.update_rhs_wo_bias(params, solved_dt, solver_bodies, c);
            }
            c.solve(solver_bodies, true, solve_friction);
        }
    };

    // ALL joints (colored, overflow, generic) solve BEFORE any contact in every pass: the last
    // constraint solved on a body wins its velocity residual and contacts must win, else a joint
    // re-imposed after a heavier body's contacts lets it push through (heavy cube on a spring-hung
    // ball). Generic constraints exist only single-group; vec lengths are fixed once laid out.
    let has_generic_joints = ctx.groups.len() == 1
        && !unsafe { &*ctx.joint_constraints }
            .generic_velocity_constraints
            .is_empty();
    let has_generic_contacts = ctx.groups.len() == 1
        && !unsafe { &*ctx.contact_constraints }
            .generic_velocity_constraints
            .is_empty();

    // Joint color stages (ascending color id; parallel within a color:
    // constraints of a color touch pairwise-disjoint bodies). Only this group's
    // slices of the layout tables participate.
    for (_, joint_range) in &ctx.joint_color_ranges[group.joint_colors.clone()] {
        let virt = 0..joint_range.end - joint_range.start;
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &virt,
            stage_batch(virt.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            for idx in claimed {
                solve_joint_chunk(joint_range.start + idx);
                // Without SIMD, the joint color ranges hold scalar builders.
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage: per-batch RMWs on the
        // shared counter measurably throttle scalar builds (4x the chunks).
        sync.complete(stage, done, virt.end);
        stage = sync.sync(stage, virt.end);
    }

    // Overflow + generic joints and soft-body particle attachments, solved exclusively by
    // worker 0 (they write body velocities, so no other worker may access solver bodies
    // concurrently: they wait at the barrier). Skipped entirely when empty (the jointless case).
    if !group.joint_overflow.is_empty() || has_generic_joints || has_attachments {
        if worker_id == 0 {
            for joint_id in group.joint_overflow.clone() {
                solve_joint(joint_id);
            }

            if has_attachments {
                let soft = unsafe { &mut *ctx.soft_constraints };
                let vs = unsafe { &mut *ctx.velocity_solver };
                soft.solve_attachments(
                    group_index,
                    &mut vs.solver_bodies,
                    wo_bias,
                    soft_update,
                    soft_warmstart,
                );
            }

            if has_generic_joints {
                let joints = unsafe { &mut *ctx.joint_constraints };
                let vs = unsafe { &mut *ctx.velocity_solver };
                let JointConstraintsSet {
                    generic_jacobians,
                    generic_velocity_constraints,
                    ..
                } = joints;
                for c in generic_velocity_constraints.iter_mut() {
                    if wo_bias {
                        c.remove_bias_from_rhs();
                    }
                    c.solve(
                        generic_jacobians,
                        &mut vs.solver_bodies,
                        &mut vs.generic_solver_vels,
                    );
                }
            }
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    /*
     * Soft-body stages, after the joints and before the contacts: contacts must win a body's
     * velocity residual over the soft body's own structure (constraints strained past
     * `soft_bodies.resweep_strain` are swept again after the contacts, at the end of the pass).
     */
    if has_soft {
        stage = unsafe {
            solve_soft_constraints(
                ctx,
                group_index,
                worker_id,
                stage,
                soft_update,
                soft_warmstart,
            )
        };
    }

    // Contact color stages (ascending color id).
    for (_, chunk_range) in &ctx.color_ranges[group.colors.clone()] {
        let virt = 0..chunk_range.end - chunk_range.start;
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &virt,
            stage_batch(virt.end, sync.num_workers),
            worker_id,
        ) {
            let claimed_len = claimed.len();
            for idx in claimed {
                solve_chunk(chunk_range.start + idx);
            }
            done += claimed_len;
        }
        // One completion flush per worker per stage (see the joint stages above).
        sync.complete(stage, done, virt.end);
        stage = sync.sync(stage, virt.end);
    }

    // Soft-body surface contact stages (constraints over the surface's particles and the other body),
    // solved in both passes like every contact: one stage per color of chunks holding
    // pairwise-disjoint bodies (see `soft_contact_chunks`), then the serial tail below.
    let has_soft_contacts =
        !soft_ref.is_empty() && !soft_ref.groups[group_index].contacts.is_empty();
    if has_soft_contacts {
        let sg = &soft_ref.groups[group_index];
        for chunk_range in &soft_ref.contact_colors[sg.contact_colors.clone()] {
            let mut done = 0;
            while let Some(claimed) = sync.claim(stage, chunk_range, 1, worker_id) {
                // SAFETY: chunks of a same color touch pairwise-disjoint solver bodies.
                let soft = unsafe { &mut *ctx.soft_constraints };
                let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
                let claimed_len = claimed.len();
                for chunk in claimed {
                    soft.solve_contact_chunk(
                        chunk,
                        solver_bodies,
                        params,
                        wo_bias,
                        soft_update,
                        soft_warmstart.is_some(),
                    );
                }
                done += claimed_len;
            }
            sync.complete(stage, done, chunk_range.len());
            stage = sync.sync(stage, chunk_range.len());
        }
    }

    // Overflow + generic contacts, solved exclusively by worker 0. Unlike the
    // joint overflow stage this one is unconditional: it doubles as the pass'
    // trailing barrier so every worker leaves solve_pass in lockstep.
    if worker_id == 0 {
        for chunk_id in group.overflow.clone() {
            solve_chunk(chunk_id);
        }

        // The serial tail of the soft contacts.
        if has_soft_contacts {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            for chunk in soft.groups[group_index].contact_serial.clone() {
                soft.solve_contact_chunk(
                    chunk,
                    solver_bodies,
                    params,
                    wo_bias,
                    soft_update,
                    soft_warmstart.is_some(),
                );
            }
        }

        if has_generic_contacts {
            let contacts = unsafe { &mut *ctx.contact_constraints };
            let vs = unsafe { &mut *ctx.velocity_solver };
            let jac = &contacts.generic_jacobians;
            for c in contacts.generic_velocity_constraints.iter_mut() {
                if wo_bias {
                    c.remove_cfm_and_bias_from_rhs();
                }
                c.solve(
                    jac,
                    &mut vs.solver_bodies,
                    &mut vs.generic_solver_vels,
                    true,
                    solve_friction,
                );
            }
        }
        sync.complete(stage, 1, 1);
    }
    stage = sync.sync(stage, 1);

    if has_soft && ctx.base_params.soft_bodies.resweep_strain < Real::MAX {
        // Second sweep of the strained soft constraints after the contacts (already updated and
        // warm-started by the first sweep): a torn body's structure gets the substep's last word;
        // the rest stay with the contacts (re-solving them re-violates contacts). Worker 0, serial.
        if worker_id == 0 {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            soft.resweep_strained_constraints(group_index, solver_bodies);
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    /*
     * Stage: scatter the active clusters' proxy velocity changes (this pass's joint impulses and
     * any external impulse) onto their particles as a rigid field (worker 0, serial: overlapping
     * clusters share particles).
     */
    if has_clusters {
        if worker_id == 0 {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            soft.scatter_clusters(group_index, solver_bodies);
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    stage
}

/// The soft-body constraint stages of one pass: colored element constraints (parallel within a color),
/// shape-matching constraints (parallel: one particle each), then the serial tail on worker 0
/// (overflow-color constraints and the global-volume constraints). Returns the updated stage ordinal.
unsafe fn solve_soft_constraints(
    ctx: &SharedCtx,
    group_index: usize,
    worker_id: usize,
    mut stage: usize,
    soft_update: bool,
    soft_warmstart: Option<Real>,
) -> usize {
    let sync = ctx.sync;
    let soft_ref = unsafe { &*ctx.soft_constraints };
    let sg = soft_ref.groups[group_index].clone();
    let min_strain = ctx.base_params.soft_bodies.resweep_strain;
    for color_range in &soft_ref.color_ranges[sg.colors.clone()] {
        let virt = 0..color_range.len();
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &virt,
            stage_batch(virt.end, sync.num_workers),
            worker_id,
        ) {
            // SAFETY: rows of a same color touch pairwise-disjoint particles.
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            let claimed_len = claimed.len();
            for idx in claimed {
                soft.solve_color_constraint(
                    color_range,
                    idx,
                    solver_bodies,
                    soft_update,
                    soft_warmstart,
                    min_strain,
                );
            }
            done += claimed_len;
        }
        sync.complete(stage, done, virt.end);
        stage = sync.sync(stage, virt.end);
    }

    if !sg.shape_constraints.is_empty() && !sg.shape_serial {
        let mut done = 0;
        while let Some(claimed) = sync.claim(
            stage,
            &sg.shape_constraints,
            stage_batch(sg.shape_constraints.len(), sync.num_workers),
            worker_id,
        ) {
            // SAFETY: without per-cluster shape matching, shape constraints touch one particle each,
            // all distinct.
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            let claimed_len = claimed.len();
            for constraint_id in claimed {
            }
            done += claimed_len;
        }
        sync.complete(stage, done, sg.shape_constraints.len());
        stage = sync.sync(stage, sg.shape_constraints.len());
    } else if !sg.shape_constraints.is_empty() {
        // Per-cluster shape matching can give one particle several constraints: solved serially by
        // worker 0 (cluster shape constraints are few).
        if worker_id == 0 {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            for constraint_id in sg.shape_constraints.clone() {
            }
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }

    if sg.serial.len() > 0 || !sg.volume_rows.is_empty() {
        if worker_id == 0 {
            let soft = unsafe { &mut *ctx.soft_constraints };
            let solver_bodies = unsafe { &mut (*ctx.velocity_solver).solver_bodies };
            for idx in 0..sg.serial.len() {
                soft.solve_color_constraint(
                    &sg.serial,
                    idx,
                    solver_bodies,
                    soft_update,
                    soft_warmstart,
                    min_strain,
                );
            }
            for vi in sg.volume_constraints.clone() {
                soft.solve_volume_constraint(vi, solver_bodies, soft_warmstart);
            }
            sync.complete(stage, 1, 1);
        }
        stage = sync.sync(stage, 1);
    }
    stage
}
