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

    // --- Passive recovery (stand-down) ---
    /// Self contacts of tangled features (inverted cells, self-crossings) stand down so
    /// the elasticity can resolve the tangle instead of freezing it at skin distance.
    /// (default: `true`)
    pub self_stand_down: bool,
    /// A vertex row touching a boundary crossing between two surfaces may only expel,
    /// never hold (a keep-apart row there is wrong-sided by construction).
    /// (default: `true`)
    pub cross_body_expel_gate: bool,
    /// Edge rows touching a cross-body boundary crossing stand down (thin open bodies
    /// meet through their edge rows, so the vertex gate alone cannot free them).
    /// (default: `true`)
    pub edge_stand_down: bool,
    /// Rows on crossing-flagged features repel instead of standing down: the row pushes
    /// the vertex to the side of the element where its own neighbors lie (its one-ring
    /// centroid), through to skin distance there. A shallow crossing returns where it came
    /// from, a body already past halfway completes its passage, and adjacent vertices
    /// agree; a neighborhood straddling the element (a genuine tangle) still stands down.
    /// (default: `false`)
    pub crossing_repulsion: bool,

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
        }
    }
}
