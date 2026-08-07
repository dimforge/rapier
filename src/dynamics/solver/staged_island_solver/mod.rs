//! Intra-island parallel solver: contacts are colored so same-color
//! constraints touch pairwise-disjoint bodies (SIMD-packed per color); persistent workers claim
//! batches between spin-barrier stages; joints/multibody solve in worker-0-exclusive stages.
//! Deterministic: results depend only on the coloring, not on batch distribution.

mod helpers;
mod init;
mod joints;
mod solve;
mod sync;
mod worker;

use crate::alloc_prelude::*;
use core::ops::Range;
use core::sync::atomic::AtomicBool;

use crate::dynamics::solver::contact_constraint::ContactConstraintsSet;
use crate::dynamics::solver::interaction_groups::ParallelInteractionGroups;
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::solver_contact_graph::ContactRef;
use crate::dynamics::solver::{JointConstraintsSet, VelocitySolver};
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, MultibodyJointSet, RigidBodyHandle,
    RigidBodySet,
};
use crate::math::Real;
use parry::math::SIMD_WIDTH;

use crate::dynamics::solver::contact_constraint::{
    ContactWithCoulombFriction, ContactWithCoulombFrictionBuilder,
};
#[cfg(feature = "dim3")]
use crate::dynamics::solver::contact_constraint::{
    ContactWithTwistFriction, ContactWithTwistFrictionBuilder,
};
use parry::math::SimdReal;

use self::sync::StageSync;

/// Number of work items claimed at once by a worker.
const BODY_BATCH: usize = 256;
const CHUNK_BATCH: usize = 8;
const JOINT_BATCH: usize = 16;

/// Reference worker count the parallel-vs-serial color split is calibrated for.
///
/// The split must depend on the scene alone, never on the actual pool size: demoting
/// a color to the serial overflow tail moves its solve after every parallel color,
/// so a pool-sized threshold would change the Gauss-Seidel order (and therefore the
/// results) across machines with different core counts.
const LAYOUT_REF_WORKERS: usize = 8;

/// Claim-batch size of a constraint stage (~8 claims per worker): tiny batches multiply the
/// claim/steal cross-core traffic (measurably throttles high worker counts); huge batches leave
/// stage-tail imbalance. The stealing in [`StageSync::claim`] keeps the tail at one batch.
fn stage_batch(domain_len: usize, num_workers: usize) -> usize {
    (domain_len / (num_workers * 8)).clamp(CHUNK_BATCH, 64)
}

/// One contiguous run of SIMD chunks over a slice of solver-manifold refs (a persistent color
/// bucket slice, or a piece of the overflow grouper's output). Chunks are addressed as
/// (segment, offset) with lanes resolved on demand — no per-step O(manifolds) lane-array copy.
#[derive(Copy, Clone, Debug)]
struct ChunkSegment {
    /// Global id of this segment's first chunk.
    first_chunk: u32,
    /// The refs this segment chunks. SAFETY: points into the persistent contact-graph buckets
    /// or `overflow_chunk_refs`, both stable and unmutated for the whole solver scope.
    refs: *const ContactRef,
    /// Number of refs; the segment yields `len.div_ceil(SIMD_WIDTH)` chunks,
    /// the last one PADDING-filled.
    len: u32,
}

// SAFETY: read-only view over storage that is not mutated during the solver
// scope (same contract as `ManifoldStore`).
unsafe impl Send for ChunkSegment {}
unsafe impl Sync for ChunkSegment {}

impl ChunkSegment {
    /// Only the lane-disjointness assertion walks a segment chunk by chunk.
    #[cfg(debug_assertions)]
    #[inline]
    fn num_chunks(&self) -> usize {
        (self.len as usize).div_ceil(SIMD_WIDTH)
    }

    /// The lane refs of this segment's `local`-th chunk.
    #[inline]
    fn chunk(&self, local: usize) -> [ContactRef; SIMD_WIDTH] {
        let start = local * SIMD_WIDTH;
        let end = ((local + 1) * SIMD_WIDTH).min(self.len as usize);
        debug_assert!(start < end);
        let mut ids = [ContactRef::PADDING; SIMD_WIDTH];
        for (k, i) in (start..end).enumerate() {
            // SAFETY: `i < self.len`; see the struct-level contract.
            ids[k] = unsafe { *self.refs.add(i) };
        }
        ids
    }
}

/// Resolves a global chunk id through the segment table (sorted by
/// `first_chunk`, no gaps).
#[inline]
fn chunk_at(segments: &[ChunkSegment], chunk_id: usize) -> [ContactRef; SIMD_WIDTH] {
    let seg_i = segments.partition_point(|s| s.first_chunk as usize <= chunk_id) - 1;
    let seg = &segments[seg_i];
    seg.chunk(chunk_id - seg.first_chunk as usize)
}

/// One substep solve-group: a contiguous solver-body slot range plus its slices of the
/// group-major constraint layout, solved for `num_substeps` substeps of length `dt`. With a
/// single group (the common case) the ranges span everything — identical to ungrouped layout.
#[derive(Clone, Debug)]
struct GroupLayout {
    /// Solver-body slot range (== awake-island body index range).
    bodies: Range<usize>,
    /// Global chunk-id range (this group's parallel colors + overflow tail).
    chunks: Range<usize>,
    /// Index range into `color_ranges`.
    colors: Range<usize>,
    /// This group's serial overflow chunk-id range (tail of `chunks`).
    overflow: Range<usize>,
    /// Index range into `joint_color_ranges`.
    joint_colors: Range<usize>,
    /// This group's SIMD joint chunk range (into
    /// `joint_chunk_lanes`/`joint_chunk_rows`).
    joint_chunks: Range<usize>,
    /// This group's full scalar joint-builder range (parallel scalar colors on
    /// non-SIMD builds, then the overflow tail).
    joint_builders: Range<usize>,
    /// This group's scalar overflow joint-builder range (tail of
    /// `joint_builders`).
    joint_overflow: Range<usize>,
    /// Substeps to run for this group.
    num_substeps: usize,
    /// This group's substep length (`base_dt / num_substeps`).
    dt: Real,
}

/// Per-group slices of the staged joint layout, persisted alongside the
/// joint-assembly cache (`staged_joints_valid`) so a reused layout can refill
/// the per-step [`GroupLayout`]s.
#[derive(Clone, Default)]
struct GroupJointRanges {
    /// Index range into `joint_color_ranges`.
    colors: Range<usize>,
    /// SIMD joint chunk range.
    chunks: Range<usize>,
    /// Full scalar builder range.
    builders: Range<usize>,
    /// Scalar overflow builder range (tail of `builders`).
    overflow: Range<usize>,
}

/// State shared between solver workers. Safety: the raw pointers are dereferenced by multiple
/// workers, sound only under [`run_worker`]'s stage/color discipline — slots reached only via
/// claimed indices, same-color body writes disjoint, worker 0 alone mutates joints/bodies/multibodies.
struct SharedCtx<'a> {
    sync: &'a StageSync,
    /// The SIMD chunk layout as segments over the persistent bucket slices (and
    /// the overflow grouper's output); resolve with [`chunk_at`].
    chunk_segments: &'a [ChunkSegment],
    num_chunks: usize,
    /// The substep solve-groups, group-major over every layout table below.
    /// Always at least one entry; exactly one on the ungrouped fast path.
    groups: &'a [GroupLayout],
    color_ranges: &'a [(u8, Range<usize>)],
    joint_color_ranges: &'a [(u8, Range<usize>)],
    joint_rows: &'a [Range<usize>],
    /// Constraint-row range of each SIMD joint chunk in
    /// `joint_constraints.simd_velocity_constraints`.
    joint_chunk_rows: &'a [Range<usize>],
    island_bodies: &'a [RigidBodyHandle],
    /// Sleeping bodies filled as read-only (kinematic-like) solver bodies after
    /// the island's own bodies (slot id = `island_bodies.len() + position`).
    /// Never written back.
    has_multibodies: bool,
    base_params: &'a IntegrationParameters,
    /// Raw manifold resolution: constraint generation reads manifolds through
    /// it, the impulse-writeback stage writes them back through it.
    store: &'a ManifoldStore<'a>,
    joints: *mut JointGraphEdge,
    num_joints: usize,

    velocity_solver: *mut VelocitySolver,
    joint_constraints: *mut JointConstraintsSet,
    contact_constraints: *mut ContactConstraintsSet,

    coulomb_builders: *mut ContactWithCoulombFrictionBuilder,
    coulomb_constraints: *mut ContactWithCoulombFriction<SimdReal>,
    #[cfg(feature = "dim3")]
    twist_builders: *mut ContactWithTwistFrictionBuilder<SimdReal>,
    #[cfg(feature = "dim3")]
    twist_constraints: *mut ContactWithTwistFriction<SimdReal>,
    use_twist: bool,

    bodies: *mut RigidBodySet,
    multibodies: *mut MultibodyJointSet,
    /// Set by the body-copy stage when any island body has gyroscopic forces
    /// enabled (3D); gates the per-substep gyroscopic pass. Published before
    /// the first substep barrier, read after it.
    #[cfg(feature = "dim3")]
    any_gyroscopic: *const AtomicBool,
    /// Set by the body-writeback stage when any (non-multibody) dynamic body's
    /// post-solve motion qualifies it for CCD; replaces the pipeline's serial
    /// post-solve `update_ccd_active_flags` walk over the active bodies.
    any_ccd_active: *const AtomicBool,
    /// Set by the constraint-generation stage when any contact constraint captured a
    /// restitution seed (approaching bouncy contact); gates the end-of-step restitution
    /// pass. Published before the first substep barrier, read after it.
    any_bouncy: *const AtomicBool,
}

unsafe impl Sync for SharedCtx<'_> {}

pub(crate) struct StagedIslandSolver {
    pub contact_constraints: ContactConstraintsSet,
    pub joint_constraints: JointConstraintsSet,
    pub velocity_solver: VelocitySolver,
    /// The SIMD chunk layout: segments over the persistent bucket slices (and
    /// over `overflow_chunk_refs`), in global chunk-id order.
    chunk_segments: Vec<ChunkSegment>,
    /// The overflow grouper's output, copied flat (the body-disjoint SIMD groups
    /// first — a multiple of SIMD_WIDTH — then the ungroupable refs, one 1-lane
    /// chunk each) so `chunk_segments` can point into stable storage.
    overflow_chunk_refs: Vec<ContactRef>,
    /// For each parallel color: the color id and its range of global chunk ids
    /// (and of the SIMD constraint vectors). Constraints within a range touch
    /// pairwise-disjoint bodies.
    color_ranges: Vec<(u8, Range<usize>)>,
    /// The substep solve-groups (see [`GroupLayout`]); rebuilt every step,
    /// single spanning entry on the ungrouped fast path.
    groups: Vec<GroupLayout>,
    /// Stable storage for the multi-group chunk refs: per-(group, color)
    /// contiguous copies of the persistent buckets' refs (the single-group
    /// path points straight into the buckets and never fills this).
    grouped_chunk_refs: Vec<ContactRef>,
    joint_colors: ParallelInteractionGroups,
    /// Per parallel joint color: color id and SIMD joint-chunk index range; lanes within a
    /// range touch pairwise-disjoint bodies. Joint coloring shares the contacts' color space
    /// (avoids each body's contact colors), so same-id colors merge into one solve stage.
    joint_color_ranges: Vec<(u8, Range<usize>)>,
    /// For each SIMD joint chunk, the joint (graph-edge) indices of its lanes;
    /// padding lanes replicate lane 0 (their wide solve recomputes lane-0's
    /// exact values, so the duplicated scatter is value-identical).
    joint_chunk_lanes: Vec<[JointIndex; SIMD_WIDTH]>,
    /// Constraint-row range of each SIMD joint chunk.
    joint_chunk_rows: Vec<Range<usize>>,
    /// Scratch: (row signature, joint index) of a color's SIMD-eligible joints.
    joint_sig_scratch: Vec<(u32, JointIndex)>,
    /// Scalar joints solved by worker 0: joints without a wide row formulation
    /// (motors, coupled limits), extra-solver-iterations joints, and the joints
    /// of colors too small to parallelize.
    joint_overflow_scratch: Vec<JointIndex>,
    /// Joint builders from colors too small to parallelize: solved by worker 0.
    joint_overflow_range: Range<usize>,
    /// Scratch: the overflow-color manifolds handed to the greedy body-mask
    /// grouper (snapshotted to sidestep an aliasing borrow of `contact_constraints`).
    overflow_scratch: Vec<ContactRef>,
    /// Constraint-row range of each scalar joint builder in
    /// `joint_constraints.velocity_constraints` (a joint yields several rows which
    /// must be solved by a same worker since they touch the same bodies).
    joint_rows: Vec<Range<usize>>,
    // Staged joint-assembly persistence: coloring/chunk layout/builders reused while the joint
    // list, island layout, assembly inputs AND every parallel-joint-chunk body's
    // contact color mask are unchanged (a color-`c` joint chunk must still avoid contact-color-`c` bodies).
    staged_joints_valid: bool,
    prev_staged_active_set_epoch: u32,
    prev_staged_assembly_epoch: u32,
    prev_staged_joint_indices: Vec<JointIndex>,
    /// Group body ranges the cached joint layout was built for; the reuse gate compares them
    /// explicitly because group boundaries can move without an epoch bump (a component split
    /// keeps the partition key sequence sorted).
    prev_staged_group_bodies: Vec<Range<usize>>,
    /// Per-group slices of the cached joint layout, copied into the per-step
    /// [`GroupLayout`]s whether the layout was rebuilt or reused.
    staged_group_joint_layout: Vec<GroupJointRanges>,
    /// Set by the body-copy stage when any island body has gyroscopic forces
    /// enabled (3D); gates the per-substep gyroscopic pass.
    #[cfg(feature = "dim3")]
    any_gyroscopic: AtomicBool,
    /// Set by the body-writeback stage (see `SharedCtx::any_ccd_active`).
    any_ccd_active: AtomicBool,
    /// Set by the constraint-generation stage (see `SharedCtx::any_bouncy`).
    any_bouncy: AtomicBool,
    /// `Some(any_active)` when the last solve computed post-solve CCD activation flags for
    /// EVERY dynamic body it wrote back (multibody links skip the fused computation); the
    /// pipeline falls back to its own pass otherwise.
    pub post_solve_ccd_active: Option<bool>,
    sync: StageSync,
}

impl StagedIslandSolver {
    pub fn new() -> Self {
        Self {
            contact_constraints: ContactConstraintsSet::new(),
            joint_constraints: JointConstraintsSet::new(),
            velocity_solver: VelocitySolver::new(),
            chunk_segments: Vec::new(),
            overflow_chunk_refs: Vec::new(),
            color_ranges: Vec::new(),
            groups: Vec::new(),
            grouped_chunk_refs: Vec::new(),
            overflow_scratch: Vec::new(),
            joint_colors: ParallelInteractionGroups::new(),
            joint_color_ranges: Vec::new(),
            joint_chunk_lanes: Vec::new(),
            joint_chunk_rows: Vec::new(),
            joint_sig_scratch: Vec::new(),
            joint_overflow_scratch: Vec::new(),
            joint_overflow_range: 0..0,
            joint_rows: Vec::new(),
            staged_joints_valid: false,
            prev_staged_active_set_epoch: 0,
            prev_staged_assembly_epoch: 0,
            prev_staged_joint_indices: Vec::new(),
            prev_staged_group_bodies: Vec::new(),
            staged_group_joint_layout: Vec::new(),
            #[cfg(feature = "dim3")]
            any_gyroscopic: AtomicBool::new(false),
            any_ccd_active: AtomicBool::new(false),
            any_bouncy: AtomicBool::new(false),
            post_solve_ccd_active: None,
            sync: StageSync::new(1),
        }
    }
}
