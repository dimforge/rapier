//! Components used to configure a simulation run by rapier, these are not modified by bevy_rapier.

use bevy::{
    prelude::{Component, Resource},
    reflect::Reflect,
};

use crate::math::{Real, Vect};

#[cfg(doc)]
use crate::prelude::TransformInterpolation;

/// The friction model selected by [`IntegrationParameters::friction_model`].
#[cfg(feature = "dim3")]
pub use rapier::dynamics::FrictionModel;
/// The low-level parameters of a context's simulation (see
/// [`RapierContextSimulation::integration_parameters`](crate::plugin::context::RapierContextSimulation::integration_parameters)
/// and [`RapierContextInitialization`](crate::plugin::RapierContextInitialization)).
pub use rapier::dynamics::IntegrationParameters;

/// The different ways of adjusting the timestep length each frame.
#[derive(Copy, Clone, Debug, PartialEq, Resource)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum TimestepMode {
    /// Use a fixed timestep: the physics simulation will be advanced by the fixed value
    /// `dt` seconds at each Bevy tick by performing `substeps` of length `dt / substeps`.
    Fixed {
        /// The physics simulation will be advanced by this total amount at each Bevy tick.
        dt: f32,
        /// This number of substeps of length `dt / substeps` will be performed at each Bevy tick.
        substeps: usize,
    },
    /// Use a variable timestep: the physics simulation will be advanced by the variable value
    /// `min(max_dt, Time::delta_seconds() * time_scale)` seconds at each Bevy tick. If
    /// `time_scale > 1.0` then the simulation will appear to run faster than real-time whereas
    /// `time_scale < 1.0` makes the simulation run in slow-motion. No step is performed on ticks
    /// where this is zero (e.g. the first tick, or `time_scale == 0.0`).
    Variable {
        /// Maximum amount of time the physics simulation may be advanced at each Bevy tick.
        max_dt: f32,
        /// Multiplier controlling if the physics simulation should advance faster (> 1.0),
        /// at the same speed (= 1.0) or slower (< 1.0) than the real time.
        time_scale: f32,
        /// The number of substeps that will be performed at each tick.
        substeps: usize,
    },
    /// Use a fixed timestep equal to `IntegrationParameters::dt`, but don't step if the
    /// physics simulation advanced by a time greater than the real-world elapsed time multiplied by `time_scale`.
    /// Rigid-bodies with a component [`TransformInterpolation`] attached will use interpolation to
    /// estimate the rigid-bodies position in-between steps.
    Interpolated {
        /// The physics simulation will be advanced by this total amount at each Bevy tick, unless
        /// the physics simulation time is ahead of a the real time.
        dt: f32,
        /// Multiplier controlling if the physics simulation should advance faster (> 1.0),
        /// at the same speed (= 1.0) or slower (< 1.0) than the real time.
        time_scale: f32,
        /// The number of substeps that will be performed whenever the physics simulation is advanced.
        substeps: usize,
    },
}

impl Default for TimestepMode {
    fn default() -> Self {
        TimestepMode::Variable {
            max_dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 1,
        }
    }
}

/// Selects what a simulation step of a Rapier context does.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SimulationMode {
    /// Run the full physics pipeline: collision detection, constraints resolution and
    /// integration of the rigid-bodies' motion.
    #[default]
    Full,
    /// Only run collision detection with Rapier's `CollisionPipeline`.
    ///
    /// Collision events, contact pairs, intersections, physics hooks and scene queries keep
    /// working, but the dynamics are not simulated: no forces, joints or contact responses are
    /// applied and rigid-bodies only move when their `Transform` is modified (kinematic
    /// position-based bodies are teleported to their next kinematic position). Contact force
    /// events are never emitted in this mode.
    CollisionOnly,
}

/// Strategy used to keep the broad-phase BVH efficient as colliders move.
///
/// Mirrors Rapier’s [`BvhOptimizationStrategy`](rapier::geometry::BvhOptimizationStrategy).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum BroadPhaseOptimizationStrategy {
    /// Different sub-trees of the BVH are optimized at each step.
    #[default]
    SubtreeOptimizer,
    /// Disables incremental BVH optimization (discouraged, meant for debugging).
    None,
}

impl From<BroadPhaseOptimizationStrategy> for rapier::geometry::BvhOptimizationStrategy {
    fn from(value: BroadPhaseOptimizationStrategy) -> Self {
        match value {
            BroadPhaseOptimizationStrategy::SubtreeOptimizer => Self::SubtreeOptimizer,
            BroadPhaseOptimizationStrategy::None => Self::None,
        }
    }
}

impl From<rapier::geometry::BvhOptimizationStrategy> for BroadPhaseOptimizationStrategy {
    fn from(value: rapier::geometry::BvhOptimizationStrategy) -> Self {
        match value {
            rapier::geometry::BvhOptimizationStrategy::SubtreeOptimizer => Self::SubtreeOptimizer,
            rapier::geometry::BvhOptimizationStrategy::None => Self::None,
        }
    }
}

#[derive(Component, Copy, Clone, Debug, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A component for specifying configuration information for the physics simulation
pub struct RapierConfiguration {
    /// Specifying the gravity of the physics simulation.
    pub gravity: Vect,
    /// Specifies if the physics simulation is active and update the physics world.
    pub physics_pipeline_active: bool,
    /// Specifies the number of subdivisions along each axes a shape should be subdivided
    /// if its scaled representation cannot be represented with the same shape type.
    ///
    /// For example, a ball subject to a non-uniform scaling cannot be represented as a ball
    /// (it’s an ellipsoid). Thus, in order to be compatible with Rapier, the shape is automatically
    /// discretized into a convex polyhedron, using `scaled_shape_subdivision` as the number of subdivisions
    /// along each spherical coordinates angle.
    pub scaled_shape_subdivision: u32,
    /// Specifies if backend sync should always accept transform changes, which may be from the writeback stage.
    pub force_update_from_transform_changes: bool,
    /// Whether this context simulates the full dynamics, or only detects collisions.
    pub simulation_mode: SimulationMode,
    /// The number of worker threads of a thread pool dedicated to this context’s simulation.
    ///
    /// If `None` (the default), the simulation runs on the thread pool of the calling thread,
    /// usually Rayon’s global pool. Changes are applied before the next simulation step. This
    /// is ignored unless the `parallel` feature is enabled (and `unsync-callbacks` is not).
    pub num_threads: Option<usize>,
}

impl RapierConfiguration {
    /// Configures rapier with the specified length unit.
    ///
    /// See the documentation of [`IntegrationParameters::length_unit`] for additional details
    /// on that argument.
    ///
    /// The default gravity is automatically scaled by that length unit.
    pub fn new(length_unit: Real) -> Self {
        Self {
            gravity: Vect::Y * -9.81 * length_unit,
            physics_pipeline_active: true,
            scaled_shape_subdivision: 10,
            force_update_from_transform_changes: false,
            simulation_mode: SimulationMode::Full,
            num_threads: None,
        }
    }
}
