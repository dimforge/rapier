//! One solve pass of a substep: joint colors, worker-0 joint overflow, contact
//! colors, then the worker-0 contact overflow (all completion-gated stages).

use crate::dynamics::IntegrationParameters;
use crate::dynamics::solver::JointConstraintsSet;
use crate::math::Real;

use super::{GroupLayout, SharedCtx, stage_batch};

/// One solve pass: joints (colored, then overflow + generic on worker 0) then
/// contact colors (parallel). Returns the caller's updated stage ordinal.
pub(super) unsafe fn solve_pass(
    ctx: &SharedCtx,
    group: &GroupLayout,
    worker_id: usize,
    mut stage: usize,
    wo_bias: bool,
    warmstart_joints: bool,
    params: &IntegrationParameters,
    solved_dt: Real,
) -> usize {
    let sync = ctx.sync;
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
                // Positions were integrated after the biased pass: refresh the unbiased rhs
                // from the current poses (reusing pre-integration separations
                // destabilizes stack rocking).
                let builder = unsafe { &*ctx.twist_builders.add(chunk_id) };
                builder.refresh_rhs_wo_bias(params, solved_dt, solver_bodies, c);
            }
            c.solve(solver_bodies, true, solve_friction);
        }
        if !ctx.use_twist {
            let c = unsafe { &mut *ctx.coulomb_constraints.add(chunk_id) };
            if wo_bias {
                let builder = unsafe { &*ctx.coulomb_builders.add(chunk_id) };
                builder.refresh_rhs_wo_bias(params, solved_dt, solver_bodies, c);
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

    // Overflow + generic joints, solved exclusively by worker 0 (they write body
    // velocities, so no other worker may access solver bodies concurrently: they
    // wait at the barrier). Skipped entirely when empty (the jointless case).
    if !group.joint_overflow.is_empty() || has_generic_joints {
        if worker_id == 0 {
            for joint_id in group.joint_overflow.clone() {
                solve_joint(joint_id);
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

    // Overflow + generic contacts, solved exclusively by worker 0. Unlike the
    // joint overflow stage this one is unconditional: it doubles as the pass'
    // trailing barrier so every worker leaves solve_pass in lockstep.
    if worker_id == 0 {
        for chunk_id in group.overflow.clone() {
            solve_chunk(chunk_id);
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
    sync.sync(stage, 1)
}
