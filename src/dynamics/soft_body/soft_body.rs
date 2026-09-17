//! The `SoftBody` struct and the shared constants of the soft-body constraints.
use crate::alloc_prelude::*;
use crate::dynamics::RigidBodyHandle;
use crate::math::{DIM, Real, Vector};
use super::{
    SoftBodyCell, SoftBodyCellModel, SoftBodyEdge, SoftBodyMaterial, SoftBodyParticle,
    SoftParticleAttachment,
};
#[cfg(feature = "dim3")]
use super::SoftBodyDihedral;
#[cfg(feature = "fem")]
use super::SoftBodySolver;
#[cfg(feature = "serde-serialize")]
use super::soft_body_material::default_true;

/// Number of particles a soft-body constraint can touch (a simplex cell).
pub const SOFT_BODY_MAX_CONSTRAINT_PARTICLES: usize = DIM + 1;

/// Color id marking a soft-body element that could not be given a parallel color
/// (solved serially by the solver).
pub(crate) const SOFT_BODY_OVERFLOW_COLOR: u8 = 128;

/// A deformable body: [`SoftBodyParticle`]s linked by elastic elements ([`SoftBodyEdge`]s, bending,
/// [`SoftBodyCell`]s, volume preservation, shape matching). Its hidden root rigid body (tagged by
/// [`crate::dynamics::RigidBody::soft_body`]) must never be moved, removed or attached to joints.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBody {
    pub(crate) particles: Vec<SoftBodyParticle>,
    /// The hidden rigid body standing for this soft body in the islands and holding its
    /// colliders (invalid until the soft body is inserted in a set): the whole-body cluster's
    /// proxy, re-pointed to another live cluster's proxy if that cluster is removed.
    pub(crate) root_body: RigidBodyHandle,
    /// The clusters of this soft body (index 0: the whole-body cluster, created at insertion).
    /// Removed clusters leave a dead slot so indices stay stable.
    pub(crate) clusters: Vec<super::SoftBodyCluster>,
    /// Number of live clusters referencing each particle: a particle is removed when it drops
    /// to zero.
    pub(crate) cluster_refs: Vec<u32>,
    /// The particles attached to rigid bodies.
    pub(crate) attachments: Vec<SoftParticleAttachment>,
    /// Whether the soft body is currently asleep (mirrors the root body's state at the end of
    /// each step).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) sleeping: bool,
    /// Whether the soft body takes part in the simulation (applied to the root body at the start
    /// of the next step; cleared by the NaN quarantine).
    #[cfg_attr(feature = "serde-serialize", serde(default = "default_true"))]
    pub(crate) enabled: bool,
    /// Set by the setters that move particles between steps (positions, pinning): the colliders
    /// are updated at the start of the next step so the narrow phase sees the new geometry.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) positions_modified: bool,
    /// Set when the attachments changed since the last step: the island links are updated at
    /// the start of the next step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) attachments_modified: bool,
    pub(crate) edges: Vec<SoftBodyEdge>,
    #[cfg(feature = "dim3")]
    pub(crate) dihedrals: Vec<SoftBodyDihedral>,
    pub(crate) cells: Vec<SoftBodyCell>,
    /// The computational mesh's boundary: segments (2D) or triangles (3D), oriented outward.
    /// Used by the volume constraints, and as the geometry of the default collision mesh.
    pub(crate) boundary: Vec<[u32; DIM]>,
    /// Whether the boundary is closed (every segment vertex / triangle edge shared by exactly
    /// two elements).
    pub(crate) boundary_closed: bool,
    /// The cell owning each boundary element (`u32::MAX`: none): the tearing pass follows it,
    /// and the winding of an element whose cell is inverted no longer tells where the outside
    /// is.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) boundary_element_cells: Vec<u32>,
    pub(crate) material: SoftBodyMaterial,
    pub(crate) cell_model: SoftBodyCellModel,
    /// Which solver simulates this body's elasticity.
    #[cfg(feature = "fem")]
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) solver: SoftBodySolver,
    /// Whether the volume pieces are constrained (only set while there is at least one piece).
    pub(crate) volume_preservation: bool,
    /// The pieces of material enclosed by a closed boundary, each with its own volume constraint
    /// (rebuilt with the boundary tables).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) volume_pieces: Vec<super::SoftVolumePiece>,
    /// Multiplier of the target volume (`> 1` inflates the body).
    pub(crate) volume_factor: Real,
    /// The rest shape's center of mass at insertion (world space).
    pub(crate) rest_com: Vector,
    /// The particle spacing: the impact-adaptive substeps bound the travel per substep by it,
    /// and the collision meshes' colliders take their default contact skin from it.
    pub(crate) particle_radius: Real,
    /// Rigid-body settings applied to the particles at insertion.
    pub(crate) particle_settings: super::SoftBodyParticleSettings,
    /// Number of parallel colors used by the elements (`0..num_colors`, plus possibly the
    /// overflow color).
    pub(crate) num_colors: u8,
    /// Whether some element has the overflow color.
    pub(crate) has_overflow_color: bool,
    /// Set by the setters that change the dynamics (material, volume factor, shape matching...):
    /// the pipeline wakes the soft body up at the start of the next step so the change takes
    /// effect even if it was sleeping.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) modified: bool,
    /// Set by the solver when a cell flowed plastically by a significant amount during the last
    /// step: the body is kept awake so its creep goes on.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) plastic_flowing: bool,
    /// Set when the elements flowed plastically: the particles' rest positions are fitted to the
    /// flowed rest shapes at the end of the step, over the following steps until they settle
    /// (see `SoftBody::fit_rest_positions`).
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) rest_fit_pending: bool,
    /// The speed the sleep rule reads for this body's cluster proxies: the fastest particle at the
    /// end of the last step, or `Real::MAX` while the body flows plastically (kept awake).
    /// `RigidBodyActivation::update_energy` compares it against the proxies' activation threshold.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) sleep_speed: Real,
    /// Set when some edge or cell has a `torn` mark: the tearing pass at the end of the step
    /// removes them (see `SoftBodySet::tear`).
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) tearing_pending: bool,
    /// Bumped whenever the topology changes (a tear removed elements or duplicated particles):
    /// renderers keying their meshes on the particles and elements rebuild them on change.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) topology_version: u32,
    /// Largest normal approach speed of the rigid bodies met by the surface, for the last step
    /// and the one before (`None`: no contact constraint that step); with the particle speeds, it drives
    /// the impact-adaptive substeps (`IntegrationParameters::soft_bodies.max_extra_substeps`).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) contact_approach_speeds: [Option<Real>; 2],
    /// Total normal impulse of the surface's contact constraints over the last step, and the extra
    /// substeps currently requested from that load (see `sync_particle_positions`).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) contact_load: Real,
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) load_extra_substeps: u8,
    /// The soft body this one was split off from by a tear or a cut (see
    /// [`Self::origin`]).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) origin: Option<crate::dynamics::SoftBodyHandle>,
    /// The soft bodies split off from this one, in creation order (see [`Self::pieces`]).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) pieces: Vec<crate::dynamics::SoftBodyHandle>,
    /// User-defined data associated to this soft body.
    pub user_data: u128,
}
