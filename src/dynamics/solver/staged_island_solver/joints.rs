//! Staged joint assembly: joint coloring in the contacts' color space, SIMD
//! chunking by row signature, scalar/overflow builders, and the cross-step
//! joint-assembly persistence cache.

use crate::alloc_prelude::*;

use crate::dynamics::solver::categorization::categorize_joints;
use crate::dynamics::solver::contact_constraint::joint_num_constraints;
use crate::dynamics::solver::joint_constraint::JointConstraintBuilder;
use crate::dynamics::solver::joint_constraint::JointConstraintBuilderSimd;
use crate::dynamics::solver::reset_buffer;
use crate::dynamics::{
    JointGraphEdge, JointIndex, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use parry::math::SIMD_WIDTH;

use super::{GroupJointRanges, JOINT_BATCH, LAYOUT_REF_WORKERS, StagedIslandSolver};

impl StagedIslandSolver {
    /// Staged joint init: joints are colored (same-color = body-disjoint); within each parallel
    /// color, SIMD-eligible joints group by row-layout signature into SIMD chunks (padding lanes
    /// replicate lane 0). Everything else (no wide formulation, extra-iteration joints, colors
    /// too small to parallelize) becomes scalar builders solved serially by worker 0.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn init_joints(
        &mut self,
        active_set_epoch: u32,
        joint_assembly_epoch: u32,
        warmstart_joints: bool,
        island_bodies: &[RigidBodyHandle],
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        contact_color_masks: &[u128],
    ) {
        // Group boundaries can move without an epoch bump (a component split
        // keeps the partition key sequence sorted), so the group-major layout
        // is validated by comparing the body ranges explicitly.
        let groups_unchanged = self.prev_staged_group_bodies.len() == self.groups.len()
            && self
                .prev_staged_group_bodies
                .iter()
                .zip(self.groups.iter())
                .all(|(prev, g)| *prev == g.bodies);
        let reusable = self.staged_joints_valid
            && groups_unchanged
            && self.prev_staged_active_set_epoch == active_set_epoch
            && self.prev_staged_assembly_epoch == joint_assembly_epoch
            && self.prev_staged_joint_indices == joint_indices
            && self.staged_joint_colors_still_free(impulse_joints, contact_color_masks);

        if reusable {
            // Recycled coloring/chunk layout/builders: only the warm-start seeds (last step's
            // written-back impulses) go stale, and only matter with joint warm-starting on.
            if warmstart_joints {
                let joints = &mut self.joint_constraints;
                // Joint-heavy scenes refresh thousands of independent builders:
                // run in parallel (this is the dominant serial-assembly cost on
                // the rain benchmark, ~0.23ms/step of ragdoll joints).
                #[cfg(feature = "parallel")]
                {
                    use rayon::prelude::*;
                    joints
                        .velocity_constraints_builder
                        .par_iter_mut()
                        .with_min_len(64)
                        .for_each(|builder| builder.refresh_warmstart_seeds(impulse_joints));
                    joints
                        .simd_velocity_constraints_builder
                        .par_iter_mut()
                        .with_min_len(16)
                        .for_each(|builder| builder.refresh_warmstart_seeds(impulse_joints));
                }
                #[cfg(not(feature = "parallel"))]
                {
                    for builder in &mut joints.velocity_constraints_builder {
                        builder.refresh_warmstart_seeds(impulse_joints);
                    }
                    for builder in &mut joints.simd_velocity_constraints_builder {
                        builder.refresh_warmstart_seeds(impulse_joints);
                    }
                }
            }

            self.rebuild_generic_joints_staged(island_bodies, bodies, multibodies, impulse_joints);
            return;
        }

        let joints = &mut self.joint_constraints;
        joints.two_body_interactions.clear();
        joints.generic_two_body_interactions.clear();
        categorize_joints(
            multibodies,
            impulse_joints,
            joint_indices,
            &mut joints.two_body_interactions,
            &mut joints.generic_two_body_interactions,
        );

        joints.simd_velocity_constraints.clear();

        if self.groups.len() > 1 {
            // Multi-group: joints colored globally (any per-group subset of a color stays
            // body-disjoint), then chunked GROUP-major so each group's solve stages address
            // contiguous table slices exactly like the single-group layout. The reuse cache
            // stays valid because the gate compares group body ranges explicitly.
            self.joint_colors.group_interactions(
                island_bodies.len(),
                impulse_joints,
                &joints.two_body_interactions,
                contact_color_masks,
            );

            self.joint_rows.clear();
            self.joint_color_ranges.clear();
            self.joint_chunk_lanes.clear();
            self.joint_chunk_rows.clear();
            self.joint_overflow_scratch.clear();
            self.staged_group_joint_layout.clear();

            // Calibrated for LAYOUT_REF_WORKERS, NOT the pool size (see that constant's docs).
            let min_color_joints = JOINT_BATCH * LAYOUT_REF_WORKERS / 2;
            let num_groups = self.groups.len();

            // Slot -> group-index table; a joint's group is the max group index over its awake
            // bodies' slots (same rule as contacts: for kinematic-dynamic the max picks the
            // dynamic side).
            let mut slot_group = alloc::vec![0u16; island_bodies.len()];
            for (gi, g) in self.groups.iter().enumerate() {
                slot_group[g.bodies.clone()].fill(gi as u16);
            }
            let joint_group = |joint_i: JointIndex| -> usize {
                let joint = &impulse_joints[joint_i].weight;
                let mut gi = 0usize;
                for handle in [joint.body1, joint.body2] {
                    let rb = &bodies[handle];
                    if !rb.is_fixed() && !rb.is_sleeping() {
                        if let Some(&g) = slot_group.get(rb.ids.active_set_id as usize) {
                            gi = gi.max(g as usize);
                        }
                    }
                }
                gi
            };

            // Pass 1: split every color by group; parallel-eligible (group, color) subsets are
            // buffered per group, the rest go to the group's scalar overflow. Deterministic:
            // colors ascending, stable in-color order.
            let mut parallel_by_group: Vec<Vec<(u8, Vec<JointIndex>)>> =
                alloc::vec![Vec::new(); num_groups];
            let mut scalar_by_group: Vec<Vec<JointIndex>> = alloc::vec![Vec::new(); num_groups];
            let mut split: Vec<Vec<JointIndex>> = alloc::vec![Vec::new(); num_groups];
            for color_id in 0..self.joint_colors.num_groups() {
                let color = self.joint_colors.group(color_id);
                let color_bit = self.joint_colors.group_color(color_id);
                for s in &mut split {
                    s.clear();
                }
                for joint_i in color {
                    split[joint_group(*joint_i)].push(*joint_i);
                }
                for (gi, subset) in split.iter().enumerate() {
                    if subset.is_empty() {
                        continue;
                    }
                    // Color 128 is the "couldn't color" bucket: always serial.
                    if color_bit < 128 && subset.len() >= min_color_joints {
                        parallel_by_group[gi].push((color_bit, subset.clone()));
                    } else {
                        scalar_by_group[gi].extend_from_slice(subset);
                    }
                }
            }

            // Pass 2 (SIMD): each group's parallel subsets -> SIMD chunk
            // lanes, cut at row-signature boundaries; SIMD-ineligible joints
            // fall back to the group's scalar overflow. Emitted group-major.
            {
                for gi in 0..num_groups {
                    let colors_start = self.joint_color_ranges.len();
                    let chunks_start = self.joint_chunk_lanes.len();
                    for (color_bit, subset) in &parallel_by_group[gi] {
                        self.joint_sig_scratch.clear();
                        for joint_i in subset {
                            let joint = &impulse_joints[*joint_i].weight;
                            if joint.data.supports_simd_constraints() {
                                self.joint_sig_scratch
                                    .push((joint.data.simd_row_signature(), *joint_i));
                            } else {
                                scalar_by_group[gi].push(*joint_i);
                            }
                        }
                        self.joint_sig_scratch.sort_by_key(|(sig, _)| *sig);

                        let chunk_start = self.joint_chunk_lanes.len();
                        let mut run_start = 0;
                        while run_start < self.joint_sig_scratch.len() {
                            let sig = self.joint_sig_scratch[run_start].0;
                            let mut run_end = run_start + 1;
                            while run_end < self.joint_sig_scratch.len()
                                && self.joint_sig_scratch[run_end].0 == sig
                            {
                                run_end += 1;
                            }
                            for chunk in
                                self.joint_sig_scratch[run_start..run_end].chunks(SIMD_WIDTH)
                            {
                                let mut lanes = [chunk[0].1; SIMD_WIDTH];
                                for (l, (_, joint_i)) in chunk.iter().enumerate() {
                                    lanes[l] = *joint_i;
                                }
                                self.joint_chunk_lanes.push(lanes);
                            }
                            run_start = run_end;
                        }
                        if self.joint_chunk_lanes.len() > chunk_start {
                            self.joint_color_ranges
                                .push((*color_bit, chunk_start..self.joint_chunk_lanes.len()));
                        }
                    }
                    self.staged_group_joint_layout.push(GroupJointRanges {
                        colors: colors_start..self.joint_color_ranges.len(),
                        chunks: chunks_start..self.joint_chunk_lanes.len(),
                        builders: 0..0,
                        overflow: 0..0,
                    });
                }

                // Generate the wide builders and count the wide constraint rows.
                let num_joint_chunks = self.joint_chunk_lanes.len();
                unsafe {
                    reset_buffer(
                        &mut joints.simd_velocity_constraints_builder,
                        num_joint_chunks,
                    );
                }
                let mut num_wide_rows = 0;
                for (chunk_id, lanes) in self.joint_chunk_lanes.iter().enumerate() {
                    let joint_refs = array![|ii| &impulse_joints[lanes[ii]].weight];
                    let row_start = num_wide_rows;
                    JointConstraintBuilderSimd::generate(
                        joint_refs,
                        bodies,
                        *lanes,
                        &mut joints.simd_velocity_constraints_builder[chunk_id],
                        &mut num_wide_rows,
                    );
                    self.joint_chunk_rows.push(row_start..num_wide_rows);
                }
                unsafe {
                    reset_buffer(&mut joints.simd_velocity_constraints, num_wide_rows);
                }
            }

            // Pass 3: scalar builders, group-major — on non-SIMD builds each
            // group's parallel scalar colors first (recording their builder
            // ranges), then the group's worker-0 overflow.
            let num_parallel_scalar = 0usize;
            let num_scalar =
                num_parallel_scalar + scalar_by_group.iter().map(|v| v.len()).sum::<usize>();
            unsafe {
                reset_buffer(&mut joints.velocity_constraints_builder, num_scalar);
            }
            let mut num_rows = 0;
            let mut num_builders = 0;
            for gi in 0..num_groups {
                let builders_start = num_builders;
                let overflow_start = num_builders;
                for joint_i in &scalar_by_group[gi] {
                    let joint = &impulse_joints[*joint_i].weight;
                    let row_start = num_rows;
                    JointConstraintBuilder::generate(
                        joint,
                        bodies,
                        *joint_i,
                        &mut joints.velocity_constraints_builder[num_builders],
                        &mut num_rows,
                    );
                    self.joint_rows.push(row_start..num_rows);
                    num_builders += 1;
                }
                let l = &mut self.staged_group_joint_layout[gi];
                l.builders = builders_start..num_builders;
                l.overflow = overflow_start..num_builders;
            }
            unsafe {
                reset_buffer(&mut joints.velocity_constraints, num_rows);
            }
            self.joint_overflow_range = 0..0;
        } else {
            self.single_group_joint_layout(
                island_bodies,
                bodies,
                impulse_joints,
                contact_color_masks,
            );
        }

        self.staged_joints_valid = true;
        // Without SIMD the parallel colors hold scalar builders whose joint
        // indices aren't retained, so their masks can't be re-validated: keep
        // rebuilding every step (niche configuration).
        self.prev_staged_active_set_epoch = active_set_epoch;
        self.prev_staged_assembly_epoch = joint_assembly_epoch;
        self.prev_staged_joint_indices.clear();
        self.prev_staged_joint_indices
            .extend_from_slice(joint_indices);
        self.prev_staged_group_bodies.clear();
        self.prev_staged_group_bodies
            .extend(self.groups.iter().map(|g| g.bodies.clone()));

        self.rebuild_generic_joints_staged(island_bodies, bodies, multibodies, impulse_joints);
    }

    /// The historical single-group joint layout: coloring, SIMD chunking and
    /// scalar generation over the whole awake set.
    fn single_group_joint_layout(
        &mut self,
        island_bodies: &[RigidBodyHandle],
        bodies: &RigidBodySet,
        impulse_joints: &mut [JointGraphEdge],
        contact_color_masks: &[u128],
    ) {
        let joints = &mut self.joint_constraints;
        self.joint_colors.group_interactions(
            island_bodies.len(),
            impulse_joints,
            &joints.two_body_interactions,
            contact_color_masks,
        );

        self.joint_rows.clear();
        self.joint_color_ranges.clear();
        self.joint_chunk_lanes.clear();
        self.joint_chunk_rows.clear();
        self.joint_overflow_scratch.clear();

        // Calibrated for LAYOUT_REF_WORKERS, NOT the pool size (see that constant's docs).
        let min_color_joints = JOINT_BATCH * LAYOUT_REF_WORKERS / 2;

        // Pass 1 (SIMD): parallel colors -> SIMD chunk lanes, cut at row-signature boundaries
        // (stable sort keeps in-color order deterministic); ineligible joints fall back to the
        // scalar overflow. Without SIMD, parallel colors keep per-color ranges of scalar
        // builders, generated before the overflow so `joint_color_ranges` indexes contiguously.

        for color_id in 0..self.joint_colors.num_groups() {
            let color = self.joint_colors.group(color_id);
            let color_bit = self.joint_colors.group_color(color_id);
            // Color 128 is the "couldn't color" bucket: always serial overflow.
            let parallel = color_bit < 128 && color.len() >= min_color_joints;
            if !parallel {
                self.joint_overflow_scratch.extend_from_slice(color);
                continue;
            }

            {
                self.joint_sig_scratch.clear();
                for joint_i in color {
                    let joint = &impulse_joints[*joint_i].weight;
                    if joint.data.supports_simd_constraints() {
                        self.joint_sig_scratch
                            .push((joint.data.simd_row_signature(), *joint_i));
                    } else {
                        self.joint_overflow_scratch.push(*joint_i);
                    }
                }
                self.joint_sig_scratch.sort_by_key(|(sig, _)| *sig);

                let chunk_start = self.joint_chunk_lanes.len();
                let mut run_start = 0;
                while run_start < self.joint_sig_scratch.len() {
                    let sig = self.joint_sig_scratch[run_start].0;
                    let mut run_end = run_start + 1;
                    while run_end < self.joint_sig_scratch.len()
                        && self.joint_sig_scratch[run_end].0 == sig
                    {
                        run_end += 1;
                    }
                    for chunk in self.joint_sig_scratch[run_start..run_end].chunks(SIMD_WIDTH) {
                        let mut lanes = [chunk[0].1; SIMD_WIDTH];
                        for (l, (_, joint_i)) in chunk.iter().enumerate() {
                            lanes[l] = *joint_i;
                        }
                        self.joint_chunk_lanes.push(lanes);
                    }
                    run_start = run_end;
                }
                if self.joint_chunk_lanes.len() > chunk_start {
                    self.joint_color_ranges
                        .push((color_bit, chunk_start..self.joint_chunk_lanes.len()));
                }
            }
        }

        // Generate the wide builders and count the wide constraint rows.
        {
            let num_joint_chunks = self.joint_chunk_lanes.len();
            unsafe {
                reset_buffer(
                    &mut joints.simd_velocity_constraints_builder,
                    num_joint_chunks,
                );
            }
            // Row-range prefix pass: a chunk's row count is a pure function of
            // its (signature-identical) lane-0 joint, so the ranges are known
            // before any builder is generated…
            let mut num_wide_rows = 0;
            for lanes in &self.joint_chunk_lanes {
                let count = joint_num_constraints(&impulse_joints[lanes[0]].weight);
                self.joint_chunk_rows
                    .push(num_wide_rows..num_wide_rows + count);
                num_wide_rows += count;
            }
            // …which makes the builder generation embarrassingly parallel (it
            // is the dominant cost of a joint-assembly rebuild on ragdoll-heavy
            // scenes, and churn-y scenes rebuild every few steps).
            let gen_chunk = |chunk_id: usize, builder: &mut JointConstraintBuilderSimd| {
                let lanes = &self.joint_chunk_lanes[chunk_id];
                let joint_refs = array![|ii| &impulse_joints[lanes[ii]].weight];
                let mut row_cursor = self.joint_chunk_rows[chunk_id].start;
                JointConstraintBuilderSimd::generate(
                    joint_refs,
                    bodies,
                    *lanes,
                    builder,
                    &mut row_cursor,
                );
                debug_assert_eq!(row_cursor, self.joint_chunk_rows[chunk_id].end);
            };
            #[cfg(feature = "parallel")]
            {
                use rayon::prelude::*;
                joints
                    .simd_velocity_constraints_builder
                    .par_iter_mut()
                    .with_min_len(16)
                    .enumerate()
                    .for_each(|(chunk_id, builder)| gen_chunk(chunk_id, builder));
            }
            #[cfg(not(feature = "parallel"))]
            for (chunk_id, builder) in joints
                .simd_velocity_constraints_builder
                .iter_mut()
                .enumerate()
            {
                gen_chunk(chunk_id, builder);
            }
            unsafe {
                reset_buffer(&mut joints.simd_velocity_constraints, num_wide_rows);
            }
        }

        // Pass 2: scalar builders — without SIMD, first the parallel colors
        // (recording their builder ranges), then the worker-0 overflow.
        let mut num_rows = 0;
        let mut num_builders = 0;

        unsafe {
            reset_buffer(
                &mut joints.velocity_constraints_builder,
                self.joint_overflow_scratch.len(),
            );
        }

        let overflow_start = num_builders;
        for joint_i in &self.joint_overflow_scratch {
            let joint = &impulse_joints[*joint_i].weight;
            let row_start = num_rows;
            JointConstraintBuilder::generate(
                joint,
                bodies,
                *joint_i,
                &mut joints.velocity_constraints_builder[num_builders],
                &mut num_rows,
            );
            self.joint_rows.push(row_start..num_rows);
            num_builders += 1;
        }
        self.joint_overflow_range = overflow_start..num_builders;

        unsafe {
            reset_buffer(&mut joints.velocity_constraints, num_rows);
        }

        self.staged_group_joint_layout.clear();
        self.staged_group_joint_layout.push(GroupJointRanges {
            colors: 0..self.joint_color_ranges.len(),
            chunks: 0..self.joint_chunk_lanes.len(),
            builders: 0..self.joint_rows.len(),
            overflow: self.joint_overflow_range.clone(),
        });
    }

    /// `true` if every joint of the cached assembly still holds a real color that this
    /// step's contact colors leave free on both of its bodies.
    fn staged_joint_colors_still_free(
        &self,
        impulse_joints: &[JointGraphEdge],
        contact_color_masks: &[u128],
    ) -> bool {
        for joint_i in &self.prev_staged_joint_indices {
            let joint = &impulse_joints[*joint_i].weight;
            if joint.solver_color >= 128 {
                return false;
            }
            let bit = 1u128 << joint.solver_color;
            for h in [joint.body1, joint.body2] {
                if body_contact_color_mask(contact_color_masks, h) & bit != 0 {
                    return false;
                }
            }
        }
        true
    }

    /// The generic (multibody-related) joint constraints are rebuilt every
    /// step, on both the fresh and the recycled assembly paths: multibody mass
    /// matrices and jacobians change with the poses.
    fn rebuild_generic_joints_staged(
        &mut self,
        island_bodies: &[RigidBodyHandle],
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        impulse_joints: &[JointGraphEdge],
    ) {
        let joints = &mut self.joint_constraints;
        joints.generic_jacobians.fill(0.0);
        joints.generic_velocity_constraints.clear();
        joints.generic_velocity_constraints_builder.clear();
        let mut j_id = 0;
        joints.compute_generic_joint_constraints(
            island_bodies,
            bodies,
            multibodies,
            impulse_joints,
            &mut j_id,
        );
    }
}

/// The persistent-contact-color mask of a rigid-body (empty for bodies out of
/// the mask table's range).
fn body_contact_color_mask(contact_color_masks: &[u128], h: RigidBodyHandle) -> u128 {
    contact_color_masks
        .get(h.into_raw_parts().0 as usize)
        .copied()
        .unwrap_or(0)
}
