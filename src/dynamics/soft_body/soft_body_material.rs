//! The `SoftBodyMaterial`: per-family stiffness, damping, plasticity and tearing parameters.
use crate::dynamics::soft_body::SoftEdgePlasticFlow;
use crate::dynamics::SpringCoefficients;
use crate::math::Real;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// The stiffness and damping of every soft-body constraint family: a `natural_frequency` (Hz) and
/// `damping_ratio` per family, normalized by the constraint's effective mass (independent of mass,
/// substeps and timestep); a frequency far above the substep rate saturates instead of exploding.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodyMaterial {
    /// Softness of the structural edges.
    pub edge_softness: SpringCoefficients<Real>,
    /// Softness of the bending edges and dihedral constraints.
    pub bend_softness: SpringCoefficients<Real>,
    /// Softness of the per-cell volume constraints ([`crate::dynamics::SoftBodyCellModel::Volume`]) and of the global
    /// volume-preservation constraint.
    pub volume_softness: SpringCoefficients<Real>,
    /// Softness of the shape-matching constraints.
    pub shape_matching_softness: SpringCoefficients<Real>,
    /// Young's modulus of the elastic cells ([`crate::dynamics::SoftBodyCellModel::Corotational`]
    /// and [`crate::dynamics::SoftBodyCellModel::NeoHookean`]), in force per unit area (3D) or
    /// length (2D); their natural frequency derives from it, so a finer mesh does not get stiffer.
    pub young_modulus: Real,
    /// Poisson's ratio of the elastic cells, in `[0, 0.5)`.
    pub poisson_ratio: Real,
    /// Damping ratio of the elastic cells.
    pub elastic_damping_ratio: Real,
    /// Plastic yield of the elastic cells: the strain (Frobenius norm of the deviatoric part of
    /// `sym(RᵀF) - I`, read from stress by the constraint solver) beyond which a cell's
    /// rest shape flows toward its current shape (default: `0.0`, no plasticity).
    pub plastic_yield: Real,
    /// Rate (per second) at which the strain in excess of the yield is absorbed into the rest
    /// shape (default: `1.0`); the flow is deviatoric (volume preserving).
    pub plastic_creep: Real,
    /// Largest accumulated plastic deformation of a cell, the Frobenius norm of `P - I` with `P`
    /// its rest shape's plastic stretch (default: `1.0`). Flow past it is projected back onto the
    /// bound, so a crushed cell cannot flow toward a sliver; an inverted cell never flows.
    pub plastic_max: Real,
    /// Rate (per second) at which the particles' velocities are pulled toward the body's best-fit
    /// rigid motion, damping every deformation mode but not the whole-body motion; a few units per
    /// second settle a stiff slender body's residual bending sway (default: `0.0`, off).
    pub deformation_damping: Real,
    /// Plastic yield of the edges (structural and bending): the strain `|length / rest_length - 1|`
    /// beyond which the rest length flows toward the current length (default: `0.0`, none), at the
    /// rate [`Self::edge_plastic_creep`], up to `edge_plastic_max`; tears use the initial length.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub edge_plastic_yield: Real,
    /// Rate (per second) at which an edge's strain in excess of its yield is absorbed into its
    /// rest length (default: `1.0`); `Real::INFINITY` (or anything above the step rate) sets it at
    /// once.
    #[cfg_attr(feature = "serde-serialize", serde(default = "one"))]
    pub edge_plastic_creep: Real,
    /// Largest permanent set an edge may accumulate, as a fraction of the length it was created
    /// with (default: `1.0`): past it the edge stops yielding and stays elastic from where it
    /// flowed to (a repeatedly crushed edge cannot flow to nothing).
    #[cfg_attr(feature = "serde-serialize", serde(default = "half"))]
    pub edge_plastic_max: Real,
    /// Whether the edges take a permanent set under a squeeze, a stretch, or both (default:
    /// both).
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub edge_plastic_flow: SoftEdgePlasticFlow,
    /// Strain beyond which the elements break (default: `None`, unbreakable): a structural or
    /// cell whose largest tensile principal strain exceeds it, is torn at the end of the step
    /// (see [`crate::dynamics::SoftBody::tear_edge`] for what a tear removes). Cells of the volume model do not
    /// tear on their own, their edges do. The cell strain is the one seen by the solver, capped
    /// at `1.0`, so a cell threshold of one or more never fires.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub tear_strain: Option<Real>,
    /// [`crate::dynamics::SoftBody::tear_edge`]). Unlike the strain, the force reports the load of an edge that
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub tear_force: Option<Real>,
    /// Time constant, in seconds, over which an element's load is averaged before it is compared
    /// to its tear threshold (default: `0.0`, none). At `0.0` one step past the threshold tears;
    /// with a positive value the load is smoothed exponentially over about that long, so a brief
    /// spike (an impact, a jerk) is shrugged off and only a sustained pull tears. Applies to the
    /// strain and the force criteria alike; [`crate::dynamics::SoftBodyEdge::stress`] and [`crate::dynamics::SoftBodyCell::stress`]
    /// read the smoothed load back.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub tear_smoothing: Real,
    /// How much tougher an element deep inside the body is than one at its surface (default:
    /// `1.0`, no difference). An element whose particles are all interior (none on the surface)
    /// and undamaged needs this many times its tear threshold; elements at the surface, or next
    /// to a particle an earlier tear passed through, tear at the threshold itself. Tears then
    /// start at the surface or at existing damage and run inward, where the material is
    /// weakened, instead of anywhere a particle happens to be pulled. Multiplies the per-element
    /// [`crate::dynamics::SoftBodyEdge::tear_resistance`] / [`crate::dynamics::SoftBodyCell::tear_resistance`].
    #[cfg_attr(feature = "serde-serialize", serde(default = "one"))]
    pub interior_strength: Real,
    /// ones (see [`crate::dynamics::SoftBodyEdge::stress`]) go first, the others wait for the next step (edges
    /// projectile being caught like in a net while its rim tears one edge at a time; the
    /// default lets a puncture open at once.
    #[cfg_attr(
        feature = "serde-serialize",
        serde(default = "default_max_tears_per_step")
    )]
    pub max_tears_per_step: u32,
}

#[cfg(feature = "serde-serialize")]
fn default_max_tears_per_step() -> u32 {
    u32::MAX
}

#[cfg(feature = "serde-serialize")]
pub(super) fn default_true() -> bool {
    true
}

#[cfg(feature = "serde-serialize")]
pub(super) fn one() -> Real {
    1.0
}

impl Default for SoftBodyMaterial {
    fn default() -> Self {
        Self {
            edge_softness: SpringCoefficients::new(30.0, 1.0),
            bend_softness: SpringCoefficients::new(10.0, 1.0),
            volume_softness: SpringCoefficients::new(30.0, 1.0),
            shape_matching_softness: SpringCoefficients::new(10.0, 1.0),
            young_modulus: 1.0e4,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            plastic_yield: 0.0,
            plastic_creep: 1.0,
            plastic_max: 1.0,
            deformation_damping: 0.0,
            edge_plastic_yield: 0.0,
            edge_plastic_creep: 1.0,
            edge_plastic_max: 0.5,
            edge_plastic_flow: SoftEdgePlasticFlow::Both,
            tear_strain: None,
            tear_force: None,
            tear_smoothing: 0.0,
            interior_strength: 1.0,
            max_tears_per_step: u32::MAX,
        }
    }
}

impl SoftBodyMaterial {
    /// A material with the same softness for every spring family (edges, bending, volume, shape
    /// matching); the elastic-cell parameters keep their defaults.
    pub fn uniform(softness: SpringCoefficients<Real>) -> Self {
        Self {
            edge_softness: softness,
            bend_softness: softness,
            volume_softness: softness,
            shape_matching_softness: softness,
            ..Default::default()
        }
    }

    /// Whether some tear criterion is set (`tear_strain` or `tear_force`).
    pub fn tears(&self) -> bool {
        self.tear_strain.is_some() || self.tear_force.is_some()
    }

    /// Load of an edge as a fraction of its tear threshold: the larger of its stretch (`length`
    /// over its initial rest length `rest`) over `tear_strain` and its `force` along its direction
    /// (positive when resisting stretching) over `tear_force`; compression or no threshold: `0.0`.
    pub(crate) fn edge_tear_load(&self, length: Real, rest: Real, force: Real) -> Real {
        let mut load: Real = 0.0;
        if let Some(tear) = self.tear_strain {
            if rest > 0.0 {
                load = (length / rest - 1.0).max(0.0) / tear.max(Real::EPSILON);
            }
        }
        if let Some(tear) = self.tear_force {
            load = load.max(force.max(0.0) / tear.max(Real::EPSILON));
        }
        load
    }

    /// The load of a cell whose largest tensile strain is `strain`, as a fraction of its tear
    /// threshold (`0.0` without a `tear_strain`).
    pub(crate) fn cell_tear_load(&self, strain: Real) -> Real {
        match self.tear_strain {
            Some(tear) => strain.max(0.0) / tear.max(Real::EPSILON),
            None => 0.0,
        }
    }

    /// Blends the load `load` measured over a step of length `dt` into the smoothed load
    /// `stress`, over the `tear_smoothing` time constant (no smoothing: the load itself).
    pub(crate) fn smooth_stress(&self, stress: Real, load: Real, dt: Real) -> Real {
        if self.tear_smoothing <= 0.0 {
            load
        } else {
            let blend = 1.0 - (-dt / self.tear_smoothing).exp();
            stress + (load - stress) * blend
        }
    }

    /// The Lamé parameters `(μ, λ)` of the elastic cells.
    pub fn lame_parameters(&self) -> (Real, Real) {
        let nu = self.poisson_ratio.clamp(0.0, 0.499);
        let e = self.young_modulus.max(0.0);
        let mu = e / (2.0 * (1.0 + nu));
        let lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        (mu, lambda)
    }
}
