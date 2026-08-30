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
    /// Softness of the per-cell volume rows ([`crate::dynamics::SoftBodyCellModel::Volume`]) and of the global
    /// volume-preservation row.
    pub volume_softness: SpringCoefficients<Real>,
    /// Softness of the shape-matching rows.
    pub shape_matching_softness: SpringCoefficients<Real>,
    /// Young's modulus of the elastic cells ([`crate::dynamics::SoftBodyCellModel::Corotational`] and
    /// [`crate::dynamics::SoftBodyCellModel::NeoHookean`]), in force per unit area (3D) or per unit length (2D).
    ///
    /// Unlike the other families, the elastic cells are parameterized physically: their per-cell
    /// natural frequency is derived from the modulus, the cell's rest volume and the particle
    /// masses, so a finer mesh of the same material behaves like the coarser one (up to the
    /// discretization) instead of getting stiffer.
    pub young_modulus: Real,
    /// Poisson's ratio of the elastic cells, in `[0, 0.5)`.
    pub poisson_ratio: Real,
    /// Damping ratio of the elastic cells.
    pub elastic_damping_ratio: Real,
    /// Plastic yield of the elastic cells (corotational and Neo-Hookean models): the strain
    /// magnitude beyond which a cell's rest shape starts flowing toward its current shape
    /// (default: `0.0`, meaning no plasticity). Strain here is the Frobenius norm of the
    pub plastic_yield: Real,
    /// Rate (per second) at which the strain in excess of the yield is absorbed into the rest
    /// shape (default: `1.0`); the flow is deviatoric (volume preserving).
    pub plastic_creep: Real,
    /// Largest accumulated plastic deformation of a cell (default: `1.0`): the Frobenius norm of
    /// `P - I`, `P` the plastic stretch of its rest shape (a cell flattened to 40% of its rest
    /// thickness is at about `1.0`). Flow past it is discarded, which keeps a repeatedly crushed
    /// cell from flowing toward a sliver (and from there to inversions and instability).
    pub plastic_max: Real,
    /// Rate (per second) at which the particles' velocities are pulled toward the body's
    /// best-fit rigid motion, damping every deformation mode without slowing the body down as a
    /// whole (default: `0.0`, off).
    ///
    /// The material rows only damp their own strain rates: the residual bending of a stiff
    /// slender body (a standing letter, a cantilever) is left by the solver's finite convergence
    /// and is not damped by them, so such bodies sway for a long time. A rate of a few units per
    /// second settles them within a second or two; soft jelly-like bodies keep it at zero (it
    /// would damp their wobble too).
    pub deformation_damping: Real,
    /// Strain beyond which the elements break (default: `None`, unbreakable): a structural or
    /// cell whose largest tensile principal strain exceeds it, is torn at the end of the step
    /// (see [`crate::dynamics::SoftBody::tear_edge`] for what a tear removes). Cells of the volume model do not
    /// tear on their own, their edges do. The cell strain is the one seen by the solver, capped
    /// at `1.0`, so a cell threshold of one or more never fires.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub tear_strain: Option<Real>,
    /// [`crate::dynamics::SoftBody::tear_edge`]). Unlike the strain, the force reports the load of an edge that
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
            tear_strain: None,
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

    /// The Lamé parameters `(μ, λ)` of the elastic cells.
    pub fn lame_parameters(&self) -> (Real, Real) {
        let nu = self.poisson_ratio.clamp(0.0, 0.499);
        let e = self.young_modulus.max(0.0);
        let mu = e / (2.0 * (1.0 + nu));
        let lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        (mu, lambda)
    }
}
