//! Simulation settings shared by every soft body of a world, grouped on
//! [`IntegrationParameters::soft_bodies`](crate::dynamics::IntegrationParameters::soft_bodies).

use super::SoftRecoverySettings;
use crate::math::Real;

/// Simulation settings shared by every soft body of a world.
///
/// Lives on [`IntegrationParameters::soft_bodies`], so it can be changed between steps.
///
/// [`IntegrationParameters::soft_bodies`]: crate::dynamics::IntegrationParameters::soft_bodies
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde-serialize", serde(default))]
pub struct SoftBodiesSettings {
    /// Runtime toggles and tuning for the soft-body tangle detection and recovery stack
    /// (see [`SoftRecoverySettings`]). Every mechanism can be switched off individually.
    pub recovery: SoftRecoverySettings,
    /// Strain beyond which a soft-body constraint (an elastic cell's strain component, a distance
    /// constraint's relative error) is re-solved after the contacts inside every substep, so a
    /// light body buried under heavier ones is not torn (default: `0.75`; `Real::MAX` to disable).
    pub resweep_strain: Real,
    /// Maximum number of extra substeps a soft body requests for its island component while it
    /// is hit fast (default: `4`; `0` disables the impact-adaptive substeps): enough substeps to
    /// bound the per-substep travel of its particles and approaching rigid bodies by one radius.
    pub max_extra_substeps: usize,
    /// Factor applied to the [`IntegrationParameters`](crate::dynamics::IntegrationParameters)
    /// contact softness natural frequencies for the soft-body contacts (default: `4.0`); they run
    /// stiffer because a contact's effective mass is a few particles' worth, not the whole body's.
    pub contact_stiffening: Real,
    /// Tuning of the FEM soft-body solver (see
    /// [`SoftBodySolver::Fem`](crate::dynamics::SoftBodySolver::Fem)); only read by the soft
    /// bodies that select it.
    #[cfg(feature = "fem")]
    pub fem: SoftFemParameters,
}

impl Default for SoftBodiesSettings {
    fn default() -> Self {
        Self {
            recovery: SoftRecoverySettings::default(),
            resweep_strain: 0.75,
            max_extra_substeps: 4,
            contact_stiffening: 4.0,
            #[cfg(feature = "fem")]
            fem: SoftFemParameters::default(),
        }
    }
}

/// Tuning of the FEM soft-body solver (see
/// [`SoftBodySolver::Fem`](crate::dynamics::SoftBodySolver::Fem)), which solves a linear system
/// per substep by conjugate gradient and factorizes it once per step (see `max_dense_dofs`).
#[cfg(feature = "fem")]
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftFemParameters {
    /// Relative residual at which the conjugate gradient stops (default: `1.0e-5`).
    pub linear_tolerance: Real,
    /// Hard cap on the conjugate-gradient iterations, whatever the residual (default: `256`).
    ///
    /// A very stiff, finely meshed body can need several hundred iterations to converge; the
    /// truncated step it gets instead is under-relaxed (safe), only slower to settle.
    pub max_linear_iterations: usize,
    ///
}
#[cfg(feature = "fem")]
impl Default for SoftFemParameters {
    fn default() -> Self {
        Self {
            linear_tolerance: 1.0e-5,
            max_linear_iterations: 20,
        }
    }
}
