//! Runtime toggles and tuning for soft-body tangle prevention, detection, passive stand-down and
//! intersection-volume constraints. Each mechanism switches off individually and the empirical
//! constants are exposed.

use crate::math::Real;

/// What the per-point constraints of the features a volume constraint acts on do (see
/// [`SoftRecoverySettings::overlap_patch_constraints`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftPatchConstraints {
    /// Keep them as they are (they may fight the volume constraint).
    Keep,
    /// Stand them down: the volume constraint alone acts on those features.
    StandDown,
    /// Keep them but along the volume constraint's normal (the nearest grid cell's), with
    /// their separation measured along it, so they push the way the constraint does.
    AlongNormal,
}

/// Runtime configuration of the soft-body tangle detection and recovery stack. It lives on
/// [`crate::dynamics::SoftBodiesSettings::recovery`] (itself on
/// `IntegrationParameters::soft_bodies`), so it can be changed between steps.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde-serialize", serde(default))]
pub struct SoftRecoverySettings {
    // --- Prevention ---
    /// Raise the speculative contact margin from velocities authored between steps (insertion,
    /// `set_particle_velocity`), so the coming step's reach covers them and a fast fresh body
    /// does not tunnel. (default: `true`)
    pub authored_velocity_margin: bool,
    /// Give the edge-vs-edge pass a speculative reach (motion margin plus the closing-speed rule)
    /// and, in 3D, edge constraints for closed-vs-closed pairs, so bodies crossing corner-first
    /// collide; on, those constraints leave pressed 3D piles crossed. (default: `false`)
    pub edge_speculation: bool,

    // --- Detection ---
    /// Detect inverted cells (material locally inside out) each step; feeds the self stand-down.
    /// (default: `true`)
    pub inverted_cell_detection: bool,
    /// Detect surface self-crossings each step (the BVTT sweep); feeds the self stand-down and the
    /// self untangler. (default: `true`)
    pub self_crossing_detection: bool,
    /// Skip the self-crossing sweep while the surface's accumulated travel could not have bridged
    /// half a skin (crossings only form through motion). Off: sweep every step. (default: `true`)
    pub detection_motion_gating: bool,
    /// Detect boundary crossings between pairs of soft surfaces; feeds the cross-body expel-only
    /// gate, the edge-pass stand-down and the intersection-volume constraints. (default: `true`)
    pub cross_body_detection: bool,

    // --- Passive recovery (stand-down) ---
    /// Self contacts of tangled features (inverted cells, self-crossings) stand down so the
    /// elasticity resolves the tangle instead of freezing it at skin distance. (default: `true`)
    pub self_stand_down: bool,
    /// A vertex constraint touching a boundary crossing between two surfaces may only expel,
    /// never hold (a keep-apart constraint there is wrong-sided by construction).
    /// (default: `true`)
    pub cross_body_expel_gate: bool,
    /// Edge constraints touching a cross-body boundary crossing stand down (thin open bodies
    /// meet through their edge constraints, so the vertex gate alone cannot free them).
    /// (default: `true`)
    pub edge_stand_down: bool,
    /// Constraints on crossing-flagged features repel instead of standing down: the vertex is
    /// pushed to the side of the element where its one-ring centroid lies, through to skin
    /// distance; a neighborhood straddling the element still stands down. (default: `false`)
    pub crossing_repulsion: bool,
    /// Guide the crossing repulsion by the pair's volume normal (the separating direction of the
    /// two intruding patches' intersection-volume gradients, per grid cell) instead of the pierced
    /// element's plane normal. Closed pairs only; needs no volume constraint. (default: `false`)
    pub crossing_repulsion_guide: bool,
    /// Guide the self-crossing repulsion by the fold's volume normal: the vertices inside their own
    /// body's self-intersection region are pulled toward the surface they face along the region's
    /// mean normal. Closed meshes only; needs no volume constraint. (default: `false`)
    pub crossing_repulsion_self_guide: bool,

    // --- Pacing ---
    /// Material recovery pace, in length units per second (scaled by
    /// `IntegrationParameters::length_unit`): the corrective rate allowed to deep recovery,
    /// demoted intruders and untangling pulls. (default: `0.5`)
    pub recovery_pace: Real,

    // --- Intersection-volume constraints ---
    /// Intersection-volume contact (Allard et al. 2010) for closed surfaces: one coupled constraint
    /// per overlapping pair (soft-soft or soft-rigid) over both sides' intruding patches, corrected
    /// by the intersection volume; master switch of the knobs below. (default: `true`)
    pub overlap_constraints: bool,
    /// Overlap constraints against rigid colliders too (the surface vertices inside the shape;
    /// a dynamic rigid body takes the reaction, a fixed or kinematic one is a wall).
    /// (default: `true`)
    pub overlap_rigid: bool,
    /// A self-crossed mesh takes no pair constraint: part of its winding is mirrored, so its volume
    /// gradient points the wrong way there (two crossed eights exploded without this).
    /// (default: `true`)
    pub overlap_skip_self_tangled: bool,
    /// The 3D closed-closed edge constraints stand down on a pair an overlap constraint owns (inside the
    /// overlap they are wrong-sided and freeze it). (default: `true`)
    pub overlap_edge_stand_down: bool,
    /// Bound on the velocity change the coupled constraint may hand any side per step, in multiples
    /// of [`Self::recovery_pace`]; `Real::MAX` makes it hard. With
    /// [`Self::overlap_velocity_correction`] it bounds the closing rate instead. (default: `1.0`)
    pub overlap_constraint_pace: Real,
    /// What the per-point constraints (manifold constraints against rigid colliders, vertex
    /// constraints against soft surfaces) of the features inside a volume constraint's patch do,
    /// feature by feature (see [`SoftPatchConstraints`]). (default: `Keep`)
    pub overlap_patch_constraints: SoftPatchConstraints,
    /// Measure the intersection volume on the contact skins (surfaces dilated by their skin), not
    /// the geometric surfaces: a resting pair then has a hard volume constraint with speculative
    /// slack before the skins meet. Implied by [`SoftContactModel::VolumeOnly`]. (default: `false`)
    pub overlap_skin_volume: bool,
    /// The skin overlap kept at rest, as a fraction of the pair's skins (the paper's maintained
    /// interpenetration layer): the correction removes only the volume beyond it, holding the
    /// patch's average depth there; `0.0` rests the surfaces at their skins. (default: `0.0`)
    pub overlap_kept_depth: Real,
    /// Volume constraints on a body's self-overlaps between distinct surface regions: the vertices
    /// behind another part of their own surface form one patch per connected region, each coupled
    /// with the surface it faces like a pair of bodies. Closed meshes only. (default: `false`)
    pub overlap_self_regions: bool,
    /// Push along each constraint's normal instead of the volume gradients: every entry keeps its
    /// gradient's magnitude but takes the constraint's mean direction, so the whole patch
    /// separates along one axis. (default: `false`)
    pub overlap_normal_push: bool,
    /// Multi-volume grid (the paper's section 5): each pair's patch is split into a regular grid of
    /// cells along its tangent axes, each with its own constraint and multiplier, so the pressure
    /// and friction state vary across the patch. (default: `false`)
    pub overlap_multi_volume: bool,
    /// Cells per tangent axis of the multi-volume grid. (default: `3`)
    pub overlap_split: u32,
    /// Steps without progress of a pair's volume estimate before its positional correction
    /// stands down (a wedged pair jiggled forever otherwise); it re-arms when the estimate
    /// drops again. (default: `240`)
    pub overlap_patience: u32,
    /// Relative drop of the estimate that counts as progress for the patience.
    /// (default: `0.02`)
    pub overlap_progress_margin: Real,

}

impl Default for SoftRecoverySettings {
    fn default() -> Self {
        Self {
            authored_velocity_margin: true,
            edge_speculation: false,
            inverted_cell_detection: true,
            self_crossing_detection: true,
            detection_motion_gating: true,
            cross_body_detection: true,
            self_stand_down: true,
            cross_body_expel_gate: true,
            edge_stand_down: true,
            crossing_repulsion: false,
            crossing_repulsion_guide: false,
            crossing_repulsion_self_guide: false,
            recovery_pace: 0.5,
            overlap_constraints: true,
            overlap_rigid: true,
            overlap_skip_self_tangled: true,
            overlap_edge_stand_down: true,
            overlap_constraint_pace: 1.0,
            overlap_patch_constraints: SoftPatchConstraints::Keep,
            overlap_skin_volume: false,
            overlap_kept_depth: 0.0,
            overlap_normal_push: false,
            overlap_self_regions: false,
            overlap_multi_volume: true,
            overlap_split: 3,
            overlap_patience: 240,
            overlap_progress_margin: 0.02,
        }
    }
}
