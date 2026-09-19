//! The particle settings, the builder struct and its `Default` and `From` impls.

use crate::alloc_prelude::*;
use crate::dynamics::SpringCoefficients;
use crate::geometry::ColliderBuilder;
use crate::math::{DIM, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::{SoftBody, SoftBodyCellModel, SoftBodyMaterial};

/// The dynamics settings shared by every particle of a soft body.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyParticleSettings {
    /// Linear damping of the particles (air friction).
    pub linear_damping: Real,
    /// Gravity scale of the particles.
    pub gravity_scale: Real,
    /// Extra solver substeps requested for the particles (and everything they touch).
    pub additional_solver_iterations: usize,
    /// Extra internal PGS iterations per substep for the particles and everything they touch, on
    /// top of `IntegrationParameters::num_internal_pgs_iterations`; stamped on the root proxy each
    /// step (see [`crate::dynamics::RigidBody::set_additional_pgs_iterations`]) (default: `3`).
    pub additional_pgs_iterations: usize,
    /// Whether the soft body may fall asleep.
    pub can_sleep: bool,
    /// Dominance group of the soft body (see [`crate::dynamics::RigidBody::dominance_group`]).
    pub dominance_group: i8,
}

impl Default for SoftBodyParticleSettings {
    fn default() -> Self {
        Self {
            linear_damping: 0.0,
            gravity_scale: 1.0,
            additional_solver_iterations: 0,
            additional_pgs_iterations: 3,
            can_sleep: true,
            dominance_group: 0,
        }
    }
}

/// A builder for [`crate::dynamics::SoftBody`]s: particles are given in world space and elements
/// reference particles by index. The generators ([`Self::rope`], plus `cloth`/`cuboid`/`sphere`
/// in 3D and `polygon`/`disk`/`grid` in 2D) make complete bodies; the setters build any body.
#[derive(Clone, Debug)]
pub struct SoftBodyBuilder {
    /// World-space particle positions.
    pub positions: Vec<Vector>,
    /// Per-particle masses (empty = uniform `particle_mass`).
    pub masses: Vec<Real>,
    /// Uniform particle mass, used when `masses` is empty.
    pub particle_mass: Real,
    /// Indices of the pinned particles.
    pub pinned: Vec<u32>,
    /// Structural edges (particle index pairs).
    pub edges: Vec<[u32; 2]>,
    /// Bending edges (particle index pairs).
    pub bend_edges: Vec<[u32; 2]>,
    /// Edges that only resist stretching, never compression (indices into the concatenation of
    /// `edges` then `bend_edges`).
    pub tension_only_edges: Vec<u32>,
    /// Per-edge softness overrides (indices into the concatenation of `edges` then
    /// `bend_edges`); the material's softness applies to the others.
    pub edge_softness: Vec<(u32, SpringCoefficients<Real>)>,
    /// Per-edge tear-threshold multipliers (indices into the concatenation of `edges` then
    /// `bend_edges`, see [`crate::dynamics::SoftBodyEdge::tear_resistance`]); `1.0` for the others.
    pub edge_tear_resistance: Vec<(u32, Real)>,
    /// Dihedral bending constraints: shared edge then the two opposite vertices.
    #[cfg(feature = "dim3")]
    pub dihedrals: Vec<[u32; 4]>,
    /// Simplex cells (triangles in 2D, tetrahedra in 3D).
    pub cells: Vec<[u32; DIM + 1]>,
    /// Boundary elements (segments in 2D, triangles in 3D), oriented outward. Derived from the
    /// cells when empty.
    pub surface: Vec<[u32; DIM]>,
    /// The material.
    pub material: SoftBodyMaterial,
    /// The constitutive model of the cells.
    pub cell_model: SoftBodyCellModel,
    /// Which solver simulates the body's elasticity.
    #[cfg(feature = "fem")]
    pub solver: super::SoftBodySolver,
    /// Whether area/volume preservation is enabled (one constraint per piece of material enclosed
    /// by a closed surface).
    pub volume_preservation: bool,
    /// Target volume multiplier (`> 1` inflates the body).
    pub volume_factor: Real,
    /// Whether shape matching is enabled.
    pub shape_matching: bool,
    /// Whether the body's surface collides with itself.
    pub self_contacts: bool,
    /// Whether the default collision surface's shape is built with parry's `ORIENTED` flag
    /// (see [`Self::oriented`]); `None`: when the surface is closed.
    pub oriented: Option<bool>,
    /// Thickness of the particles: the radius of their ball colliders (bodies colliding through
    /// their particles), the surface collider's contact skin otherwise.
    pub particle_radius: Real,
    /// Template of the body's colliders: the deformable surface collider, or the particles'
    /// balls (their shape is replaced by a ball of `particle_radius`) for a body colliding through
    /// its particles; `None` disables collisions.
    pub collider_template: Option<ColliderBuilder>,
    /// Mesh held by the cells, if any: `(vertices, elements)` in world space, bound to the
    /// cells at build time.
    pub skin: Option<(Vec<Vector>, Vec<[u32; DIM]>)>,
    /// The segments the body collides through when it has no surface: a rope or a wire (3D
    /// only; in 2D the surface is already made of segments).
    #[cfg(feature = "dim3")]
    pub wire: Vec<[u32; 2]>,
    /// Whether the body meets the world through its skin rather than through its cells' boundary
    /// (off by default, and no effect without a skin).
    pub skin_collision: bool,
    /// Dynamics settings of the particles.
    pub particle_settings: SoftBodyParticleSettings,
    /// User data of the soft body.
    pub user_data: u128,
}

impl Default for SoftBodyBuilder {
    fn default() -> Self {
        Self::new(Vec::new())
    }
}

impl From<SoftBodyBuilder> for SoftBody {
    fn from(builder: SoftBodyBuilder) -> Self {
        builder.build()
    }
}
