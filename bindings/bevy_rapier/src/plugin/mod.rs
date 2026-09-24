#[cfg(feature = "dim3")]
pub use self::configuration::FrictionModel;
pub use self::configuration::{
    BroadPhaseOptimizationStrategy, IntegrationParameters, RapierConfiguration, SimulationMode,
    TimestepMode,
};
pub use self::context::{
    systemparams::{RapierContext, RapierContextMut, ReadRapierContext, WriteRapierContext},
    DefaultRapierContext, RapierContextEntityLink, SimulationToRenderTime,
};
pub use self::diagnostics::RapierDiagnosticsPlugin;
pub use self::plugin::{
    NoUserData, PhysicsSet, RapierBevyComponentApply, RapierContextInitialization,
    RapierPhysicsPlugin, RapierTransformPropagateSet,
};
pub use narrow_phase::{ContactManifoldView, ContactPairView, ContactView, SolverContactView};

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub mod systems;

pub mod configuration;
pub mod context;
mod diagnostics;
mod narrow_phase;
#[allow(clippy::module_inception)]
mod plugin;
#[cfg(test)]
mod soft_body_tests;
#[cfg(test)]
mod tests;
