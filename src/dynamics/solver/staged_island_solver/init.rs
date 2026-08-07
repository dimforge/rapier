//! Per-step assembly and launch: substep solve-groups, the SIMD chunk layout
//! over the persistent contact buckets, and the worker spawn/inline run.

use crate::alloc_prelude::*;
use core::sync::atomic::Ordering;

use crate::counters::Counters;
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::reset_buffer_reusing;
use crate::dynamics::solver::solver_contact_graph::{ContactRef, SolverContactGraph};
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    RigidBodySet,
};
use crate::math::Real;
use parry::math::SIMD_WIDTH;

#[cfg(feature = "dim3")]
use crate::dynamics::FrictionModel;

use super::sync::StageSync;
use super::worker::run_worker;
use super::{
    CHUNK_BATCH, ChunkSegment, GroupLayout, LAYOUT_REF_WORKERS, SharedCtx, StagedIslandSolver,
};

impl StagedIslandSolver {
    #[profiling::function]
    #[allow(clippy::too_many_arguments)]
    pub fn init_and_solve(
        &mut self,
        num_workers: usize,
        island_id: usize,
        counters: &mut Counters,
        base_params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        // The persistent per-color solver contact buckets (+ the
        // generic list), maintained incrementally by the narrow-phase — consumed
        // directly; nothing is selected or sorted per step.
        graph: &SolverContactGraph,
        store: &ManifoldStore,
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        joint_assembly_epoch: u32,
        multibodies: &mut MultibodyJointSet,
        // The narrow-phase's per-body masks of persistent contact solver colors,
        // used to color the joints in the same color space as the contacts.
        contact_color_masks: &[u128],
    ) {
        counters.solver.velocity_assembly_time.resume();
        // Substep solve-groups: each group (contiguous constraint-closed awake-body range, keyed
        // by max `additional_solver_iterations`) runs `base + extra` substeps at smaller dt —
        // real substeps buy convergence on high mass ratios, unlike the flat PGS sweeps they
        // replaced. Multibody scenes: one group at max extra (generic tables not group-major yet).
        let island_bodies = islands.island(island_id).bodies();
        let multi_group = islands.solve_groups.len() >= 2 && multibodies.iter().next().is_none();
        let max_extra = islands
            .solve_groups
            .first()
            .map(|g| g.extra_iters as usize)
            .unwrap_or(0);
        self.groups.clear();
        if multi_group {
            for group in &islands.solve_groups {
                let num_substeps = base_params.num_solver_iterations + group.extra_iters as usize;
                self.groups.push(GroupLayout {
                    bodies: group.body_range.clone(),
                    chunks: 0..0,
                    colors: 0..0,
                    overflow: 0..0,
                    joint_colors: 0..0,
                    joint_chunks: 0..0,
                    joint_builders: 0..0,
                    joint_overflow: 0..0,
                    num_substeps,
                    dt: base_params.dt / num_substeps as Real,
                });
            }
        } else {
            let num_substeps = base_params.num_solver_iterations + max_extra;
            self.groups.push(GroupLayout {
                bodies: 0..island_bodies.len(),
                chunks: 0..0,
                colors: 0..0,
                overflow: 0..0,
                joint_colors: 0..0,
                joint_chunks: 0..0,
                joint_builders: 0..0,
                joint_overflow: 0..0,
                num_substeps,
                dt: base_params.dt / num_substeps as Real,
            });
        }
        let num_solver_iterations = base_params.num_solver_iterations + max_extra;

        // `params` keeps the FIRST group's substep dt (multibody precompute uses it; multibody
        // scenes are always single-group). The worker loop re-derives each group's dt from
        // `GroupLayout`.
        let mut params = *base_params;
        params.dt /= num_solver_iterations as Real;

        /*
         * Serial pre-phase: solver buffers & multibodies, coloring, joint & generic
         * constraints. (Plain solver bodies are initialized by the workers in the
         * first parallel stage.)
         */
        self.velocity_solver.init_solver_buffers_and_multibodies(
            &params,
            island_bodies,
            bodies,
            multibodies,
        );

        // The persistent solver contact graph already holds two-body manifolds grouped by
        // (color, contact count) — colors touch pairwise-disjoint bodies, buckets slice straight
        // into SIMD chunks — and multibody manifolds apart; nothing to categorize/sort per step.
        const NUM_COLORS: usize = 129; // 128 parallel colors + the overflow color.
        {
            let set = &mut self.contact_constraints;
            // NOTE: the twist constraint/builder buffers are NOT cleared: they are
            // the persistent constraint cache, preserved (and resized) further
            // below once the chunk layout is known.
            set.generic_jacobians.fill(0.0);
            set.generic_velocity_constraints.clear();
            set.generic_velocity_constraints_builder.clear();
            set.simd_velocity_coulomb_constraints.clear();
            set.simd_velocity_coulomb_constraints_builder.clear();
        }
        // Avoid spawning more workers than there is work to distribute.
        let num_two_body = graph.len() - graph.generic().len();
        let approx_chunks = num_two_body / SIMD_WIDTH + joint_indices.len() / 4;
        let num_workers = num_workers.clamp(1, (approx_chunks / 16).max(1));

        // Joints: colored like contacts and laid out color by color (scalar
        // constraints only). Colors too small to parallelize plus the generic
        // (multibody) joints are solved by worker 0 in exclusive stages.
        self.init_joints(
            islands.active_set_epoch,
            joint_assembly_epoch,
            base_params.warmstart_joints,
            island_bodies,
            bodies,
            multibodies,
            impulse_joints,
            joint_indices,
            contact_color_masks,
        );
        // Refill the per-step group layouts from the (freshly built or reused)
        // per-group joint layout slices.
        debug_assert_eq!(self.groups.len(), self.staged_group_joint_layout.len());
        for (g, l) in self
            .groups
            .iter_mut()
            .zip(self.staged_group_joint_layout.iter())
        {
            g.joint_colors = l.colors.clone();
            g.joint_chunks = l.chunks.clone();
            g.joint_builders = l.builders.clone();
            g.joint_overflow = l.overflow.clone();
        }

        let set = &mut self.contact_constraints;

        // Build the SIMD chunk layout, group-major then color by color, straight from the
        // persistent buckets. Colors too small to distribute across workers merge into a
        // per-group serial "overflow" tail. The threshold is calibrated for
        // [`LAYOUT_REF_WORKERS`], NOT the pool size (see that constant's docs).
        let min_color_chunks = CHUNK_BATCH * LAYOUT_REF_WORKERS / 2;
        self.chunk_segments.clear();
        self.overflow_chunk_refs.clear();
        self.grouped_chunk_refs.clear();
        self.color_ranges.clear();
        let mut next_chunk = 0usize;

        // NOTE: the SIMD builders handle per-lane active-contact counts, so a color's whole
        //       bucket slices straight into chunks (no count grouping). Buckets arrive colors
        //       ascending, overflow last.
        let push_segment =
            |segments: &mut Vec<ChunkSegment>, next_chunk: &mut usize, refs: &[ContactRef]| {
                if refs.is_empty() {
                    return;
                }
                segments.push(ChunkSegment {
                    first_chunk: *next_chunk as u32,
                    refs: refs.as_ptr(),
                    len: refs.len() as u32,
                });
                *next_chunk += refs.len().div_ceil(SIMD_WIDTH);
            };

        if self.groups.len() == 1 {
            // Single group (the common case): the historical zero-copy layout,
            // slicing the persistent buckets directly.
            for (color, refs) in graph.buckets() {
                if (color as usize) < NUM_COLORS - 1
                    && refs.len().div_ceil(SIMD_WIDTH) >= min_color_chunks
                {
                    let range_start = next_chunk;
                    push_segment(&mut self.chunk_segments, &mut next_chunk, refs);
                    self.color_ranges.push((color, range_start..next_chunk));
                }
            }

            let overflow_start = next_chunk;
            // The overflow grouper's output lands in `overflow_chunk_refs`; its segments are
            // created only once the vec stops growing (they hold raw pointers into it). The
            // overflow color sorts last, so deferring them preserves the global chunk order.
            let mut overflow_grouped_len = 0usize;
            for (color, refs) in graph.buckets() {
                let is_overflow_color = (color as usize) == NUM_COLORS - 1;
                if is_overflow_color {
                    // Overflow color: uncolorable pairs / 2nd+ manifolds of a pair CAN share both
                    // bodies; linear chunking would alias a dynamic body across SIMD lanes and the
                    // last-writer-wins scatter would drop an impulse. Body-mask grouping keeps each
                    // chunk's lanes disjoint (ungroupable => 1-lane chunks); worker 0 solves serially.
                    self.overflow_scratch.clear();
                    self.overflow_scratch.extend_from_slice(refs);
                    set.interaction_groups.clear_groups();
                    set.interaction_groups.group_manifold_refs(
                        island_bodies.len(),
                        store,
                        &self.overflow_scratch,
                    );
                    self.overflow_chunk_refs
                        .extend_from_slice(&set.interaction_groups.simd_ref_interactions);
                    overflow_grouped_len = self.overflow_chunk_refs.len();
                    self.overflow_chunk_refs
                        .extend_from_slice(&set.interaction_groups.nongrouped_ref_interactions);
                } else if refs.len().div_ceil(SIMD_WIDTH) < min_color_chunks {
                    // A real color too small to parallelize: body-disjoint already.
                    push_segment(&mut self.chunk_segments, &mut next_chunk, refs);
                }
            }
            // The overflow segments: the grouped prefix is a multiple of SIMD_WIDTH
            // (whole body-disjoint chunks); each ungroupable ref is its own 1-lane
            // chunk, so it gets its own single-ref segment.
            debug_assert_eq!(overflow_grouped_len % SIMD_WIDTH, 0);
            push_segment(
                &mut self.chunk_segments,
                &mut next_chunk,
                &self.overflow_chunk_refs[..overflow_grouped_len],
            );
            for i in overflow_grouped_len..self.overflow_chunk_refs.len() {
                push_segment(
                    &mut self.chunk_segments,
                    &mut next_chunk,
                    &self.overflow_chunk_refs[i..i + 1],
                );
            }
            let g = &mut self.groups[0];
            g.chunks = 0..next_chunk;
            g.colors = 0..self.color_ranges.len();
            g.overflow = overflow_start..next_chunk;
        } else {
            // Multi-group: classify every bucket ref by group (a per-ref pass, only on this
            // gated path), then emit the layout group-major so each group's chunks are
            // contiguous in global chunk-id space. Reserve exact upper bounds up front:
            // segments hold raw pointers into these vecs, which must never reallocate below.
            self.grouped_chunk_refs.reserve(num_two_body);
            self.overflow_chunk_refs.reserve(num_two_body);
            let grouped_cap = self.grouped_chunk_refs.capacity();
            let overflow_cap = self.overflow_chunk_refs.capacity();

            // Slot -> group-index table (groups cover the island slots
            // exactly; fixed ids fall outside and are skipped).
            let num_groups = self.groups.len();
            let mut slot_group = alloc::vec![u16::MAX; island_bodies.len()];
            for (gi, g) in self.groups.iter().enumerate() {
                slot_group[g.bodies.clone()].fill(gi as u16);
            }
            // A manifold's group: max group index over its in-island solver bodies. Both sides
            // agree for dynamic-dynamic pairs (constraints never span groups by construction);
            // for kinematic-dynamic the max picks the dynamic side.
            let group_of = |r: &ContactRef| -> usize {
                let mut gi = 0u16;
                let mut found = false;
                for id in store.get(*r).data.solver_body_ids {
                    if let Some(&g) = slot_group.get(id as usize) {
                        if g != u16::MAX {
                            gi = gi.max(g);
                            found = true;
                        }
                    }
                }
                debug_assert!(found, "solver-active pair without an in-island body");
                gi as usize
            };

            // Pass A: copy each color's refs into per-(group, color)
            // contiguous runs of `grouped_chunk_refs` (overflow color kept
            // aside per group for the body-mask grouper).
            let mut runs: Vec<Vec<(u8, u32, u32)>> = alloc::vec![Vec::new(); num_groups];
            let mut overflow_by_group: Vec<Vec<ContactRef>> = alloc::vec![Vec::new(); num_groups];
            let mut split: Vec<Vec<ContactRef>> = alloc::vec![Vec::new(); num_groups];
            for (color, refs) in graph.buckets() {
                let is_overflow_color = (color as usize) == NUM_COLORS - 1;
                for scratch in &mut split {
                    scratch.clear();
                }
                for r in refs {
                    split[group_of(r)].push(*r);
                }
                for (gi, scratch) in split.iter().enumerate() {
                    if scratch.is_empty() {
                        continue;
                    }
                    if is_overflow_color {
                        overflow_by_group[gi].extend_from_slice(scratch);
                    } else {
                        let start = self.grouped_chunk_refs.len() as u32;
                        self.grouped_chunk_refs.extend_from_slice(scratch);
                        runs[gi].push((color, start, scratch.len() as u32));
                    }
                }
            }

            // Pass B: emit the layout group-major. Within a group: parallel
            // colors first (each a `color_ranges` entry), then the serial
            // tail (small colors + the group's grouped/ungroupable overflow).
            for gi in 0..num_groups {
                let chunks_start = next_chunk;
                let colors_start = self.color_ranges.len();
                for (color, start, len) in &runs[gi] {
                    let refs = &self.grouped_chunk_refs[*start as usize..(*start + *len) as usize];
                    if refs.len().div_ceil(SIMD_WIDTH) >= min_color_chunks {
                        let range_start = next_chunk;
                        push_segment(&mut self.chunk_segments, &mut next_chunk, refs);
                        self.color_ranges.push((*color, range_start..next_chunk));
                    }
                }
                let overflow_start = next_chunk;
                for (color, start, len) in &runs[gi] {
                    let _ = color;
                    let refs = &self.grouped_chunk_refs[*start as usize..(*start + *len) as usize];
                    if refs.len().div_ceil(SIMD_WIDTH) < min_color_chunks {
                        push_segment(&mut self.chunk_segments, &mut next_chunk, refs);
                    }
                }
                set.interaction_groups.clear_groups();
                set.interaction_groups.group_manifold_refs(
                    island_bodies.len(),
                    store,
                    &overflow_by_group[gi],
                );
                let grouped_start = self.overflow_chunk_refs.len();
                self.overflow_chunk_refs
                    .extend_from_slice(&set.interaction_groups.simd_ref_interactions);
                let grouped_end = self.overflow_chunk_refs.len();
                self.overflow_chunk_refs
                    .extend_from_slice(&set.interaction_groups.nongrouped_ref_interactions);
                debug_assert_eq!((grouped_end - grouped_start) % SIMD_WIDTH, 0);
                push_segment(
                    &mut self.chunk_segments,
                    &mut next_chunk,
                    &self.overflow_chunk_refs[grouped_start..grouped_end],
                );
                for i in grouped_end..self.overflow_chunk_refs.len() {
                    push_segment(
                        &mut self.chunk_segments,
                        &mut next_chunk,
                        &self.overflow_chunk_refs[i..i + 1],
                    );
                }

                let g = &mut self.groups[gi];
                g.chunks = chunks_start..next_chunk;
                g.colors = colors_start..self.color_ranges.len();
                g.overflow = overflow_start..next_chunk;
            }
            // The reserves above must have covered all growth (pointer
            // stability of the segment refs).
            debug_assert_eq!(self.grouped_chunk_refs.capacity(), grouped_cap);
            debug_assert_eq!(self.overflow_chunk_refs.capacity(), overflow_cap);
        }

        // Every SIMD chunk (parallel colors AND the serial overflow tail) needs lane-disjoint
        // solver bodies: the wide gather/scatter is last-writer-wins, so an aliased slot drops an
        // impulse. Fixed sides (`u32::MAX`, scatter-skipped) may repeat; sleeping sides hold a real
        // appended slot and must not — the coloring keeps pairs sharing any non-fixed body apart.
        #[cfg(debug_assertions)]
        for chunk in self
            .chunk_segments
            .iter()
            .flat_map(|seg| (0..seg.num_chunks()).map(|local| seg.chunk(local)))
        {
            let mut seen = [u32::MAX; SIMD_WIDTH * 2];
            let mut n = 0;
            for &id in &chunk {
                if id.is_padding() {
                    continue;
                }
                for bid in store.get(id).data.solver_body_ids {
                    if bid == u32::MAX {
                        continue;
                    }
                    // A non-dynamic body still holding an active slot (its type
                    // changed to fixed this step: kinematic-like for one step) may
                    // be shared across lanes: zero inverse mass, so every lane
                    // scatters the same unchanged velocity back.
                    if island_bodies
                        .get(bid as usize)
                        .is_some_and(|h| !bodies[*h].is_dynamic())
                    {
                        continue;
                    }
                    debug_assert!(
                        !seen[..n].contains(&bid),
                        "dynamic solver body {bid} shared by two lanes of one SIMD contact chunk"
                    );
                    seen[n] = bid;
                    n += 1;
                }
            }
        }

        let num_chunks = next_chunk;
        #[cfg(feature = "dim3")]
        let use_twist = matches!(params.friction_model, FrictionModel::Simplified);
        #[cfg(feature = "dim2")]
        let use_twist = false;

        // Constraints are regenerated from the manifolds every step (only the
        // warm-start impulses persist, on the manifolds themselves); reset the wide
        // constraint/builder buffers to one entry per chunk.
        #[cfg(feature = "dim3")]
        if use_twist {
            unsafe {
                reset_buffer_reusing(&mut set.simd_velocity_twist_constraints_builder, num_chunks);
                reset_buffer_reusing(&mut set.simd_velocity_twist_constraints, num_chunks);
            }
        }
        if !use_twist {
            unsafe {
                reset_buffer_reusing(
                    &mut set.simd_velocity_coulomb_constraints_builder,
                    num_chunks,
                );
                reset_buffer_reusing(&mut set.simd_velocity_coulomb_constraints, num_chunks);
            }
        }

        // Generic (multibody) contact constraints: serial init, solved by worker 0.
        let mut jacobian_id = 0;
        set.compute_generic_constraints(bodies, multibodies, graph, store, &mut jacobian_id);

        #[cfg(feature = "dim3")]
        self.any_gyroscopic.store(false, Ordering::Relaxed);
        self.any_ccd_active.store(false, Ordering::Relaxed);
        // Seed the restitution-pass gate with the generic constraints' contribution (built
        // serially above); the workers OR in the SIMD chunks' contribution during the
        // constraint-generation stage.
        let generic_bouncy = set
            .generic_velocity_constraints_builder
            .iter()
            .zip(set.generic_velocity_constraints.iter())
            .any(|(b, c)| b.has_bouncy_seed(c.num_contacts));
        self.any_bouncy.store(generic_bouncy, Ordering::Relaxed);

        counters.solver.velocity_assembly_time.pause();
        counters.solver.velocity_resolution_time.resume();

        /*
         * Parallel phase.
         */
        self.sync = StageSync::new(num_workers);

        let ctx = SharedCtx {
            sync: &self.sync,
            chunk_segments: &self.chunk_segments,
            num_chunks,
            groups: &self.groups,
            color_ranges: &self.color_ranges,
            joint_color_ranges: &self.joint_color_ranges,
            joint_rows: &self.joint_rows,
            joint_chunk_rows: &self.joint_chunk_rows,
            island_bodies,
            has_multibodies: !self.velocity_solver.multibody_roots.is_empty(),
            base_params,
            store,
            joints: impulse_joints.as_mut_ptr(),
            num_joints: impulse_joints.len(),
            velocity_solver: &mut self.velocity_solver as *mut _,
            joint_constraints: &mut self.joint_constraints as *mut _,
            contact_constraints: set as *mut _,
            coulomb_builders: set.simd_velocity_coulomb_constraints_builder.as_mut_ptr(),
            coulomb_constraints: set.simd_velocity_coulomb_constraints.as_mut_ptr(),
            #[cfg(feature = "dim3")]
            twist_builders: set.simd_velocity_twist_constraints_builder.as_mut_ptr(),
            #[cfg(feature = "dim3")]
            twist_constraints: set.simd_velocity_twist_constraints.as_mut_ptr(),
            use_twist,
            bodies: bodies as *mut _,
            multibodies: multibodies as *mut _,
            #[cfg(feature = "dim3")]
            any_gyroscopic: &self.any_gyroscopic as *const _,
            any_ccd_active: &self.any_ccd_active as *const _,
            any_bouncy: &self.any_bouncy as *const _,
        };

        let ctx_ref = &ctx;

        // Parallel build: spawn workers `1..num_workers` into a rayon scope, run worker 0
        // inline. Non-parallel/wasm build: `num_workers == 1`, worker 0 runs inline and
        // `StageSync` advances every stage immediately (no spawning/stealing/spinning).
        #[cfg(feature = "parallel")]
        rayon::in_place_scope(|scope| {
            for worker_id in 1..num_workers {
                scope.spawn(move |_| {
                    // SAFETY: see `SharedCtx` and the per-stage comments in `run_worker`.
                    unsafe {
                        run_worker(ctx_ref, worker_id);
                    }
                });
            }

            // SAFETY: see `SharedCtx` and the per-stage comments in `run_worker`.
            unsafe {
                run_worker(ctx_ref, 0);
            }
        });

        #[cfg(not(feature = "parallel"))]
        {
            debug_assert_eq!(num_workers, 1);
            // SAFETY: single worker, so no concurrent access to the shared context.
            unsafe {
                run_worker(ctx_ref, 0);
            }
        }

        counters.solver.velocity_resolution_time.pause();
        // NOTE: impulse and rigid-body writeback now happen as parallel stages of
        //       `run_worker`, so their time is included in the resolution counter.

        // Publish the fused post-solve CCD activation verdict. Multibody link bodies skip the
        // fused flag computation (separate writeback), so their presence invalidates it and
        // the pipeline falls back to its own pass.
        self.post_solve_ccd_active = self
            .velocity_solver
            .multibody_roots
            .is_empty()
            .then(|| self.any_ccd_active.load(Ordering::Relaxed));
    }
}
