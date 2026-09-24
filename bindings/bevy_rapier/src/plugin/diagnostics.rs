//! Integration of the simulation statistics with Bevy’s diagnostics.

use bevy::diagnostic::{
    Diagnostic, DiagnosticPath, Diagnostics, DiagnosticsStore, RegisterDiagnostic,
};
use bevy::ecs::entity::EntityHashSet;
use bevy::ecs::{intern::Interned, schedule::ScheduleLabel};
use bevy::prelude::*;

use crate::plugin::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};
use crate::plugin::PhysicsSet;

/// Plugin registering Bevy [`Diagnostic`]s measuring the simulation of every Rapier context.
///
/// The measurements are taken after each physics update that executed at least one simulation
/// step, from [`RapierContextSimulation::step_stats`] (which relies on Rapier’s counters) and
/// from the current state of the contexts. They can be displayed, for example, with Bevy’s
/// `LogDiagnosticsPlugin`.
///
/// The diagnostics at the paths of the associated constants (e.g. [`Self::STEP_TIME`]) are sums
/// over all the contexts (counts and timings alike), so they measure the whole physics workload
/// of the app. Enable [`Self::per_context`] to also measure each context separately at the paths
/// returned by [`Self::context_path`], registered when a context is first measured.
///
/// Only [`Self::STEP_TIME`] and the counts are measured by default: the other timings rely on
/// Rapier’s internal profiler and are only registered if the `profiler` feature is enabled. This
/// plugin enables the [`PhysicsPipeline::counters`](rapier::pipeline::PhysicsPipeline::counters)
/// of every context.
pub struct RapierDiagnosticsPlugin {
    schedule: Interned<dyn ScheduleLabel>,
    /// The maximum number of measurements kept in the history of each diagnostic.
    pub max_history_length: usize,
    /// Whether each context is also measured separately, at the paths returned by
    /// [`Self::context_path`] (default: `false`).
    pub per_context: bool,
}

impl Default for RapierDiagnosticsPlugin {
    fn default() -> Self {
        Self {
            schedule: PostUpdate.intern(),
            max_history_length: bevy::diagnostic::DEFAULT_MAX_HISTORY_LENGTH,
            per_context: false,
        }
    }
}

impl RapierDiagnosticsPlugin {
    /// Total wall-clock time spent running the simulation steps of a physics update, in
    /// milliseconds.
    pub const STEP_TIME: DiagnosticPath = DiagnosticPath::const_new("rapier/step_time");
    /// Number of simulation steps (including substeps) executed by a physics update.
    pub const STEPS: DiagnosticPath = DiagnosticPath::const_new("rapier/steps");
    /// Time spent on collision detection, in milliseconds (requires the `profiler` feature).
    pub const COLLISION_DETECTION_TIME: DiagnosticPath =
        DiagnosticPath::const_new("rapier/collision_detection_time");
    /// Time spent in the broad-phase, in milliseconds (requires the `profiler` feature).
    pub const BROAD_PHASE_TIME: DiagnosticPath =
        DiagnosticPath::const_new("rapier/broad_phase_time");
    /// Time spent in the narrow-phase, in milliseconds (requires the `profiler` feature).
    pub const NARROW_PHASE_TIME: DiagnosticPath =
        DiagnosticPath::const_new("rapier/narrow_phase_time");
    /// Time spent building the simulation islands, in milliseconds (requires the `profiler`
    /// feature).
    pub const ISLAND_CONSTRUCTION_TIME: DiagnosticPath =
        DiagnosticPath::const_new("rapier/island_construction_time");
    /// Time spent in the constraints solver, in milliseconds (requires the `profiler` feature).
    pub const SOLVER_TIME: DiagnosticPath = DiagnosticPath::const_new("rapier/solver_time");
    /// Time spent on Continuous Collision Detection, in milliseconds (requires the `profiler`
    /// feature).
    pub const CCD_TIME: DiagnosticPath = DiagnosticPath::const_new("rapier/ccd_time");
    /// Time spent updating the rigid-bodies’ positions and velocities, in milliseconds (requires
    /// the `profiler` feature).
    pub const UPDATE_TIME: DiagnosticPath = DiagnosticPath::const_new("rapier/update_time");
    /// Number of CCD substeps executed by a physics update (zero when the CCD didn't need to act).
    pub const CCD_SUBSTEPS: DiagnosticPath = DiagnosticPath::const_new("rapier/ccd_substeps");
    /// Number of rigid-bodies.
    pub const RIGID_BODIES: DiagnosticPath = DiagnosticPath::const_new("rapier/rigid_bodies");
    /// Number of awake (active) dynamic and kinematic rigid-bodies.
    pub const ACTIVE_BODIES: DiagnosticPath = DiagnosticPath::const_new("rapier/active_bodies");
    /// Number of colliders.
    pub const COLLIDERS: DiagnosticPath = DiagnosticPath::const_new("rapier/colliders");
    /// Number of impulse joints.
    pub const IMPULSE_JOINTS: DiagnosticPath = DiagnosticPath::const_new("rapier/impulse_joints");
    /// Number of multibody joints.
    pub const MULTIBODY_JOINTS: DiagnosticPath =
        DiagnosticPath::const_new("rapier/multibody_joints");
    /// Number of contact pairs tracked by the narrow-phase (touching or not) after the last step.
    pub const CONTACT_PAIRS: DiagnosticPath = DiagnosticPath::const_new("rapier/contact_pairs");
    /// Number of contact pairs with at least one active contact.
    pub const ACTIVE_CONTACT_PAIRS: DiagnosticPath =
        DiagnosticPath::const_new("rapier/active_contact_pairs");
    /// Number of contact points (soft-body contacts included) handed to the constraints solver
    /// by a physics update.
    pub const CONTACT_CONSTRAINTS: DiagnosticPath =
        DiagnosticPath::const_new("rapier/contact_constraints");
    /// Number of contact manifolds and impulse joints handed to the constraints solver by a
    /// physics update. The contacts and joints of sleeping bodies aren't counted.
    pub const SOLVER_CONSTRAINTS: DiagnosticPath =
        DiagnosticPath::const_new("rapier/solver_constraints");
    /// Number of intersecting sensor pairs.
    pub const INTERSECTIONS: DiagnosticPath = DiagnosticPath::const_new("rapier/intersections");

    /// The diagnostics measured in every build, with their unit suffix.
    const ALWAYS_MEASURED: [(DiagnosticPath, &'static str); 13] = [
        (Self::STEP_TIME, "ms"),
        (Self::STEPS, ""),
        (Self::CCD_SUBSTEPS, ""),
        (Self::RIGID_BODIES, ""),
        (Self::ACTIVE_BODIES, ""),
        (Self::COLLIDERS, ""),
        (Self::IMPULSE_JOINTS, ""),
        (Self::MULTIBODY_JOINTS, ""),
        (Self::CONTACT_PAIRS, ""),
        (Self::ACTIVE_CONTACT_PAIRS, ""),
        (Self::CONTACT_CONSTRAINTS, ""),
        (Self::SOLVER_CONSTRAINTS, ""),
        (Self::INTERSECTIONS, ""),
    ];

    /// The timings relying on Rapier’s internal profiler.
    #[cfg(feature = "profiler")]
    const PROFILER_TIMINGS: [DiagnosticPath; 7] = [
        Self::COLLISION_DETECTION_TIME,
        Self::BROAD_PHASE_TIME,
        Self::NARROW_PHASE_TIME,
        Self::ISLAND_CONSTRUCTION_TIME,
        Self::SOLVER_TIME,
        Self::CCD_TIME,
        Self::UPDATE_TIME,
    ];

    /// Measures the diagnostics in the given schedule (default: [`PostUpdate`]).
    ///
    /// This should be the schedule the physics runs in (see
    /// [`RapierPhysicsPlugin::in_schedule`](crate::plugin::RapierPhysicsPlugin::in_schedule)).
    pub fn in_schedule(mut self, schedule: impl ScheduleLabel) -> Self {
        self.schedule = schedule.intern();
        self
    }

    /// Also measures each context separately (see [`Self::per_context`]).
    pub fn with_per_context_diagnostics(mut self, per_context: bool) -> Self {
        self.per_context = per_context;
        self
    }

    /// The path of the diagnostic measuring `path` (one of the associated constants, e.g.
    /// [`Self::STEP_TIME`]) for the context `context` only, e.g. `rapier/12v0/step_time`.
    ///
    /// These diagnostics are only measured if [`Self::per_context`] is enabled.
    pub fn context_path(path: &DiagnosticPath, context: Entity) -> DiagnosticPath {
        let name = path.as_str();
        let name = name.strip_prefix("rapier/").unwrap_or(name);
        DiagnosticPath::new(format!("rapier/{context}/{name}"))
    }

    /// The path and unit suffix of every diagnostic measured by this plugin.
    fn measured_paths() -> impl Iterator<Item = (DiagnosticPath, &'static str)> {
        let always = Self::ALWAYS_MEASURED.into_iter();
        #[cfg(feature = "profiler")]
        let always = always.chain(Self::PROFILER_TIMINGS.into_iter().map(|path| (path, "ms")));
        always
    }

    /// The measurements of one context, or `None` if it didn’t execute any step during the last
    /// physics update.
    fn measure_context(
        simulation: &mut Mut<RapierContextSimulation>,
        colliders: &RapierContextColliders,
        bodies: &RapierRigidBodySet,
        joints: &RapierContextJoints,
    ) -> Option<Vec<(DiagnosticPath, f64)>> {
        if !simulation.pipeline.counters.enabled() {
            simulation
                .bypass_change_detection()
                .pipeline
                .counters
                .enable();
        }

        let stats = *simulation.step_stats();
        if stats.num_steps == 0 {
            return None;
        }

        #[allow(unused_mut)] // Only mutated with the `profiler` feature.
        let mut measurements = vec![
            (Self::STEP_TIME, stats.step_time_ms),
            (Self::STEPS, stats.num_steps as f64),
            (Self::CCD_SUBSTEPS, stats.ccd_substeps as f64),
            (Self::RIGID_BODIES, bodies.bodies.len() as f64),
            (
                Self::ACTIVE_BODIES,
                simulation.islands.active_bodies().count() as f64,
            ),
            (Self::COLLIDERS, colliders.colliders.len() as f64),
            (Self::IMPULSE_JOINTS, joints.impulse_joints.len() as f64),
            (
                Self::MULTIBODY_JOINTS,
                joints.multibody_joints.iter().count() as f64,
            ),
            (Self::CONTACT_PAIRS, stats.num_contact_pairs as f64),
            (
                Self::ACTIVE_CONTACT_PAIRS,
                simulation
                    .narrow_phase
                    .contact_pairs()
                    .filter(|pair| pair.has_any_active_contact())
                    .count() as f64,
            ),
            (Self::CONTACT_CONSTRAINTS, stats.num_solver_contacts as f64),
            (
                Self::SOLVER_CONSTRAINTS,
                stats.num_solver_constraints as f64,
            ),
            (
                Self::INTERSECTIONS,
                simulation
                    .narrow_phase
                    .intersection_pairs()
                    .filter(|(_, _, intersecting)| *intersecting)
                    .count() as f64,
            ),
        ];
        #[cfg(feature = "profiler")]
        measurements.extend([
            (
                Self::COLLISION_DETECTION_TIME,
                stats.collision_detection_time_ms,
            ),
            (Self::BROAD_PHASE_TIME, stats.broad_phase_time_ms),
            (Self::NARROW_PHASE_TIME, stats.narrow_phase_time_ms),
            (
                Self::ISLAND_CONSTRUCTION_TIME,
                stats.island_construction_time_ms,
            ),
            (Self::SOLVER_TIME, stats.solver_time_ms),
            (Self::CCD_TIME, stats.ccd_time_ms),
            (Self::UPDATE_TIME, stats.update_time_ms),
        ]);
        Some(measurements)
    }

    /// Updates the measurements of the diagnostics registered by this plugin.
    ///
    /// If `per_context` is set, the per-context diagnostics of the contexts not in `registered`
    /// are registered (their first measurement is then taken on the next update).
    fn diagnostic_system(
        per_context: bool,
        max_history_length: usize,
        commands: &mut Commands,
        diagnostics: &mut Diagnostics,
        contexts: &mut Query<(
            Entity,
            &mut RapierContextSimulation,
            &RapierContextColliders,
            &RapierRigidBodySet,
            &RapierContextJoints,
        )>,
        registered: &mut EntityHashSet,
    ) {
        let mut totals: Option<Vec<(DiagnosticPath, f64)>> = None;

        for (context, mut simulation, colliders, bodies, joints) in contexts.iter_mut() {
            let Some(measurements) =
                Self::measure_context(&mut simulation, colliders, bodies, joints)
            else {
                continue;
            };

            if per_context {
                if registered.insert(context) {
                    commands.queue(move |world: &mut World| {
                        let mut store = world.resource_mut::<DiagnosticsStore>();
                        for (path, suffix) in Self::measured_paths() {
                            store.add(
                                Diagnostic::new(Self::context_path(&path, context))
                                    .with_suffix(suffix)
                                    .with_max_history_length(max_history_length),
                            );
                        }
                    });
                }
                for (path, value) in &measurements {
                    diagnostics.add_measurement(&Self::context_path(path, context), || *value);
                }
            }

            match &mut totals {
                Some(totals) => {
                    for ((_, total), (_, value)) in totals.iter_mut().zip(&measurements) {
                        *total += value;
                    }
                }
                None => totals = Some(measurements),
            }
        }

        for (path, total) in totals.into_iter().flatten() {
            diagnostics.add_measurement(&path, || total);
        }
    }
}

impl Plugin for RapierDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        for (path, suffix) in Self::measured_paths() {
            app.register_diagnostic(
                Diagnostic::new(path)
                    .with_suffix(suffix)
                    .with_max_history_length(self.max_history_length),
            );
        }

        let per_context = self.per_context;
        let max_history_length = self.max_history_length;
        app.add_systems(
            self.schedule,
            (move |mut commands: Commands,
                   mut diagnostics: Diagnostics,
                   mut contexts: Query<(
                Entity,
                &mut RapierContextSimulation,
                &RapierContextColliders,
                &RapierRigidBodySet,
                &RapierContextJoints,
            )>,
                   mut registered: Local<EntityHashSet>| {
                Self::diagnostic_system(
                    per_context,
                    max_history_length,
                    &mut commands,
                    &mut diagnostics,
                    &mut contexts,
                    &mut registered,
                )
            })
            .after(PhysicsSet::Writeback),
        );
    }
}
