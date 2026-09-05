//! Runtime toggles and tuning for the soft-body tangle detection and recovery machinery:
//! prevention, detection, passive stand-down, and the intersection-volume rows. Every
//! mechanism can be switched off individually and the empirical constants are exposed, so
//! the minimal subset that a scene needs can be found experimentally.

/// Runtime configuration of the soft-body tangle detection and recovery stack.
///
/// Lives on [`IntegrationParameters::soft_recovery`], so it can be changed between steps.
///
/// [`IntegrationParameters::soft_recovery`]: crate::dynamics::IntegrationParameters
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde-serialize", serde(default))]
pub struct SoftRecoverySettings {
    // --- Prevention ---
    /// Raise the speculative contact margin from velocities authored between steps (at
    /// insertion, or through `set_particle_velocity`), so the coming step's reach already
    /// covers them instead of lagging one step behind and letting a fast fresh body
    /// tunnel. (default: `true`)
    pub authored_velocity_margin: bool,
    /// Give the edge-vs-edge pass a speculative reach (motion margin plus the closing-speed
    /// rule) and, in 3D, emit edge rows for closed-vs-closed pairs, so two edge-leading
    /// bodies crossing corner-first (no vertex near the other's surface) collide instead of
    /// passing straight through. Off by default: the closed-vs-closed edge rows leave the
    /// pressed 3D piles crossed. (default: `false`)
    pub edge_speculation: bool,

    // --- Detection ---
    /// Detect inverted cells (material locally inside out) each step. Feeds the self
    /// stand-down. (default: `true`)
    pub inverted_cell_detection: bool,
    /// Detect surface self-crossings each step (the BVTT sweep). Feeds the self
    /// stand-down and the self untangler. (default: `true`)
    pub self_crossing_detection: bool,
    /// Skip the self-crossing sweep while the surface's accumulated travel could not have
    /// bridged half a skin (crossings only form through motion). Off: sweep every step.
    /// (default: `true`)
    pub detection_motion_gating: bool,
    /// Detect boundary crossings between pairs of soft surfaces. Feeds the cross-body
    /// expel-only gate, the edge-pass stand-down, and the intersection-volume rows.
    /// (default: `true`)
    pub cross_body_detection: bool,

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
        }
    }
}
