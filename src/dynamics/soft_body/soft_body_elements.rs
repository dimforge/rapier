//! The element types of a soft body: particles, edges, dihedrals, cells, and the per-body solver and cell-model choices.
#[cfg(doc)]
use super::SoftBodyMaterial;
#[cfg(feature = "serde-serialize")]
use super::soft_body_material::one;
use crate::dynamics::SpringCoefficients;
use crate::math::{DIM, Matrix, Real, Rotation, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// One particle (point mass) of a soft body: plain state owned by its
/// [`crate::dynamics::SoftBody`], given its own solver body for the step when the body is awake.
/// A pinned particle (`inv_mass == 0`) is kinematic: it moves at its velocity or toward its target.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyParticle {
    /// World-space position.
    pub(crate) position: Vector,
    /// World-space velocity.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) velocity: Vector,
    /// Position in the body's rest shape, relative to the rest center of mass (the
    /// shape-matching goal shape). Follows the plastic flow of the elements (see
    /// [`Self::initial_rest_position`]).
    pub(crate) rest_position: Vector,
    /// Nominal mass (used by shape matching even for pinned particles).
    pub(crate) mass: Real,
    /// `0.0` for pinned particles.
    pub(crate) inv_mass: Real,
    /// User force applied to the particle every step until reset (`SoftBody::reset_forces`).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) force: Vector,
    /// Where a pinned particle must be at the end of the next step (`None`: it keeps moving at
    /// its velocity).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) next_position: Option<Vector>,
    /// Set once an element touching the particle tore (see [`Self::is_damaged`]).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) damaged: bool,
    /// Whether some surface element has this particle as a vertex (updated with the boundary).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) on_surface: bool,
}

impl SoftBodyParticle {
    /// The world-space position of this particle.
    pub fn position(&self) -> Vector {
        self.position
    }

    /// The world-space velocity of this particle.
    pub fn velocity(&self) -> Vector {
        self.velocity
    }

    /// The user force currently applied to this particle (see [`crate::dynamics::SoftBody::add_particle_force`]).
    pub fn force(&self) -> Vector {
        self.force
    }

    /// The position a pinned particle is moving to over the next step, if one was set with
    /// [`crate::dynamics::SoftBody::set_particle_kinematic_target`].
    pub fn kinematic_target(&self) -> Option<Vector> {
        self.next_position
    }

    /// The position of this particle in the rest shape (relative to the rest center of mass).
    /// With a plastic material the rest shape follows the elements' flow (see
    /// [`SoftBodyMaterial::plastic_yield`]); [`Self::initial_rest_position`] predates any flow.
    pub fn rest_position(&self) -> Vector {
        self.rest_position
    }

    /// The nominal mass of this particle.
    pub fn mass(&self) -> Real {
        self.mass
    }

    /// The inverse mass of this particle (`0.0` for pinned particles).
    pub fn inv_mass(&self) -> Real {
        self.inv_mass
    }

    /// Whether this particle is pinned (infinite mass, moved kinematically).
    pub fn is_pinned(&self) -> bool {
        self.inv_mass == 0.0
    }

    /// Whether an element touching this particle has torn (or [`crate::dynamics::SoftBody::set_particle_damaged`]
    /// marked it). The elements around a damaged particle lose the interior strength of
    /// [`SoftBodyMaterial::interior_strength`]: a tear runs on from where it started.
    pub fn is_damaged(&self) -> bool {
        self.damaged
    }

    /// Whether this particle is a vertex of the body's surface (a boundary segment in 2D, a
    /// boundary triangle in 3D). An element whose particles are all interior gets the material's
    /// [`SoftBodyMaterial::interior_strength`].
    pub fn is_on_surface(&self) -> bool {
        self.on_surface
    }
}

/// The role of a soft-body edge (distance constraint).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftBodyEdgeKind {
    /// A structural edge, keeping two neighboring particles at their rest distance
    /// (softness: [`SoftBodyMaterial::edge_softness`]).
    Structural,
    /// A bending edge, keeping two second-neighbor particles at their rest distance
    /// (softness: [`SoftBodyMaterial::bend_softness`]).
    Bend,
}

/// A distance constraint between two particles of a soft body.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyEdge {
    /// The two particles.
    pub vertices: [u32; 2],
    /// The distance the edge tries to maintain.
    pub rest_length: Real,
    /// Structural or bending edge.
    pub kind: SoftBodyEdgeKind,
    /// If `true`, the edge only resists stretching (a rope segment), never compression.
    pub tension_only: bool,
    /// Softness of this edge, overriding the material's (`edge_softness` / `bend_softness`)
    /// when set: anisotropic cloth (warp, weft and shear stiffness), stiffer seams...
    pub softness: Option<SpringCoefficients<Real>>,
    /// Multiplier of the material's tear thresholds for this edge (default `1.0`): a seam that
    /// holds, a perforation that gives. Usually set through [`crate::dynamics::SoftBody::set_edge_tear_resistance`]
    /// or [`crate::dynamics::SoftBody::set_cluster_tear_resistance`].
    #[cfg_attr(feature = "serde-serialize", serde(default = "one"))]
    pub tear_resistance: Real,
    /// Accumulated impulse of the last step (warm-start state).
    pub(crate) impulse: Real,
    /// Parallel solve color.
    pub(crate) color: u8,
    /// (or by [`crate::dynamics::SoftBody::tear_edge`]): removed by the tearing pass at the end of the step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) torn: bool,
    /// The load of the edge as a fraction of its tear threshold, smoothed over the material's
    /// `tear_smoothing` (see [`Self::stress`]).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) stress: Real,
}

impl SoftBodyEdge {
    /// The impulse this edge applied during the last substep, along the edge direction
    /// (positive when the edge was resisting stretching).
    pub fn impulse(&self) -> Real {
        self.impulse
    }

    /// The load this edge carries as a fraction of its tear threshold, smoothed over the
    /// material's [`SoftBodyMaterial::tear_smoothing`]: `0.0` slack, `1.0` tearing. The larger of
    /// the stretch over [`SoftBodyMaterial::tear_strain`] and the force over
    /// [`SoftBodyMaterial::tear_force`]; stays `0.0` while the material has neither.
    pub fn stress(&self) -> Real {
        self.stress
    }

}
/// A dihedral bending constraint between two triangles sharing an edge (3D only).
///
/// `vertices[0..2]` is the shared edge, `vertices[2]` and `vertices[3]` the two opposite
/// vertices.
#[cfg(feature = "dim3")]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyDihedral {
    /// The four particles.
    pub vertices: [u32; 4],
    /// The angle between the two triangle normals at rest.
    pub rest_angle: Real,
    /// Accumulated impulse of the last step (warm-start state).
    pub(crate) impulse: Real,
    /// Parallel solve color.
    pub(crate) color: u8,
}

/// Which solver simulates a soft body's elasticity (requires the `fem` cargo feature).
#[cfg(feature = "fem")]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftBodySolver {
    /// Every elastic element becomes constraints, warm-started and swept Gauss-Seidel by color once
    /// per substep with the contacts and joints. Cheap and robust, but converged only as far as the
    /// sweep count: a stiff body keeps a residual compliance, a load crosses a long body slowly.
    #[default]
    Constraints,
    /// Implicit Euler elasticity: forces and tangent stiffness from the strain-energy density,
    /// `(M + h D + h² K) Δv = b` solved over the whole body once per substep; constraints see the
    /// body through its augmented mass. Costs a factorization per step and a solve per constraint.
    Fem,
}

/// The constitutive model of a soft body's cells (triangles in 2D, tetrahedra in 3D).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftBodyCellModel {
    /// One constraint per cell keeping its signed area/volume at its rest value
    /// (softness: [`SoftBodyMaterial::volume_softness`]). Cheap; combined with the edges this
    /// gives a convincing jelly.
    #[default]
    Volume,
    /// Corotational elasticity (linear elasticity in the cell's rotation-free frame) plus an exact
    /// volume constraint, parameterized by [`SoftBodyMaterial::young_modulus`], `poisson_ratio` and
    /// `elastic_damping_ratio`. Stable at any stiffness, recovers from inverted cells.
    Corotational,
    /// Stable Neo-Hookean hyperelasticity (Smith et al. 2018): the parameters (plasticity too) and
    /// small-strain behavior of [`SoftBodyCellModel::Corotational`], but Neo-Hookean at large
    /// strains (stiffens when stretched, inverted cells pushed back through the collapse); 2x cost.
    NeoHookean,
}

/// Number of warm-start impulses stored per cell (the corotational model's strain rows plus its
/// volumetric row).
pub(crate) const CELL_IMPULSES: usize = DIM * (DIM + 1) / 2 + 1;

/// A simplex cell of a soft body: a triangle in 2D, a tetrahedron in 3D.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyCell {
    /// The cell's particles, positively oriented (positive signed area/volume at rest).
    pub vertices: [u32; DIM + 1],
    /// The signed area (2D) or volume (3D) of the cell at rest.
    pub rest_volume: Real,
    /// Inverse of the rest edge matrix `[x1 - x0, x2 - x0, (x3 - x0)]`.
    pub(crate) inv_rest_matrix: Matrix,
    /// Accumulated plastic stretch of the rest shape (material frame, unit determinant): the
    /// current rest edge matrix is `plastic_stretch * Dm₀`. Bounded by the material's
    /// `plastic_max`, which keeps flowing cells from degenerating into slivers.
    pub(crate) plastic_stretch: Matrix,
    /// Accumulated impulses of the last step (warm-start state): the volume row's in `[0]`, or
    /// the corotational rows' (strain rows in the cell frame, then volumetric).
    pub(crate) impulses: [Real; CELL_IMPULSES],
    /// Rotation of the polar decomposition of the cell's deformation gradient at the last step
    /// (warm start of the corotational rows' rotation extraction).
    pub(crate) rotation: Rotation,
    /// Multiplier of the material's Young modulus for this cell (default `1.0`): per-region
    /// stiffness, usually set through [`crate::dynamics::SoftBody::set_cluster_stiffness_scale`].
    #[cfg_attr(feature = "serde-serialize", serde(default = "one"))]
    pub stiffness_scale: Real,
    /// Multiplier of the material's tear strain for this cell (default `1.0`), see
    /// [`crate::dynamics::SoftBody::set_cell_tear_resistance`].
    #[cfg_attr(feature = "serde-serialize", serde(default = "one"))]
    pub tear_resistance: Real,
    /// Parallel solve color.
    pub(crate) color: u8,
    /// Set when the cell was strained past the material's `tear_strain` during the last step
    /// (or by [`crate::dynamics::SoftBody::tear_cell`]): removed by the tearing pass at the end of the step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) torn: bool,
    /// The tensile strain of the cell as a fraction of its tear threshold, smoothed over the
    /// material's `tear_smoothing` (see [`Self::stress`]).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub(crate) stress: Real,
}

impl SoftBodyCell {
    /// The largest tensile strain of this cell as a fraction of its tear threshold, smoothed
    /// over the material's [`SoftBodyMaterial::tear_smoothing`]: `0.0` slack, `1.0` tearing.
    /// Stays `0.0` while the material has no `tear_strain` (cells only tear on their strain).
    pub fn stress(&self) -> Real {
        self.stress
    }
}
