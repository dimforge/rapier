//! Run Harness to minimize boilerplate required to get a simulation running
use crate::dynamics::{CCDSolver, IntegrationParameters, IslandManager, JointSet, RigidBodySet};
use crate::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use crate::math::Vector;
use crate::physics::{PhysicsEvents, PhysicsState};
use crate::pipeline::{ChannelEventCollector, PhysicsHooks, PhysicsPipeline, QueryPipeline};
use parry::math::Real;
use plugin::HarnessPlugin;

/// Harness Plugin
pub mod plugin;

/// Thread state of the simulation, if running parallel
#[cfg(feature = "parallel")]
pub struct ThreadState {
    /// Rayon's [ThreadPool]
    pub thread_pool: crate::rayon::ThreadPool,
    /// The number of threads we will run with
    pub num_threads: usize,
}

#[cfg(feature = "parallel")]
/// Thread state when running in parallel
impl ThreadState {
    /// Construct a new, default [ThreadState]
    pub fn new() -> Self {
        let num_threads = num_cpus::get_physical();

        let thread_pool = crate::rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
            .unwrap();

        Self {
            thread_pool: thread_pool,
            num_threads,
        }
    }
}

#[derive(Clone)]
/// Run state of the simulation (timestep and time fields)
pub struct RunState {
    /// The current timestep_id (i.e. frame)
    pub timestep_id: usize,
    /// The current simulation time
    pub time: f32,
}

impl RunState {
    /// Construct a new, default `RunState`
    pub fn new() -> Self {
        Self {
            timestep_id: 0,
            time: 0.0,
        }
    }
}

/// The run harness that manages the physics state, physics events, and stepping
pub struct Harness {
    /// The [PhysicsState] at this point in time
    pub physics: PhysicsState,
    /// The [RunState] at this point in time
    pub run_state: RunState,
    #[cfg(feature = "parallel")]
    /// The [ThreadState] of the simulation
    pub thread_state: ThreadState,
    /// The [PhysicsEvents] for the simulation
    pub events: PhysicsEvents,
    event_handler: ChannelEventCollector,
    max_steps: usize,
    callbacks: Callbacks,
    plugins: Vec<Box<dyn HarnessPlugin>>,
}

type Callbacks = Vec<Box<dyn FnMut(&mut PhysicsState, &PhysicsEvents, &RunState)>>;

#[allow(dead_code)]
impl Harness {
    /// Create a new, empty `Harness`, with sane defaults
    pub fn new_empty() -> Self {
        let contact_channel = crossbeam::channel::unbounded();
        let proximity_channel = crossbeam::channel::unbounded();
        let event_handler = ChannelEventCollector::new(proximity_channel.0, contact_channel.0);
        let events = PhysicsEvents {
            contact_events: contact_channel.1,
            intersection_events: proximity_channel.1,
        };
        let physics = PhysicsState::new();
        let state = RunState::new();

        Self {
            physics,
            max_steps: 1000,
            callbacks: Vec::new(),
            plugins: Vec::new(),
            events,
            event_handler,
            run_state: state,
            #[cfg(feature = "parallel")]
            thread_state: ThreadState::new(),
        }
    }
    /// Create a new `Harness` with the bodies, colliders and joints
    pub fn new(bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) -> Self {
        let mut res = Self::new_empty();
        res.set_world(bodies, colliders, joints);
        res
    }

    /// Set the number of timesteps this simulation will run for
    pub fn set_max_steps(&mut self, max_steps: usize) {
        self.max_steps = max_steps
    }

    /// Get the `IntegrationParamaters` for the simulation
    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.physics.integration_parameters
    }

    /// Clear the callbacks `Vec`
    pub fn clear_callbacks(&mut self) {
        self.callbacks.clear();
    }

    /// Get the `PhysicsState` at this point in the simulation
    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.physics
    }

    /// Setup world, with bodies, colliders, and joints
    pub fn set_world(&mut self, bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) {
        self.set_world_with_params(bodies, colliders, joints, Vector::y() * -9.81, ())
    }

    /// Setup world, with bodies, colliders, and joints, gravity and `PhysicsHooks`
    pub fn set_world_with_params(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        joints: JointSet,
        gravity: Vector<Real>,
        hooks: impl PhysicsHooks<RigidBodySet, ColliderSet> + 'static,
    ) {
        // println!("Num bodies: {}", bodies.len());
        // println!("Num joints: {}", joints.len());
        self.physics.gravity = gravity;
        self.physics.bodies = bodies;
        self.physics.colliders = colliders;
        self.physics.joints = joints;
        self.physics.hooks = Box::new(hooks);

        self.physics.islands = IslandManager::new();
        self.physics.broad_phase = BroadPhase::new();
        self.physics.narrow_phase = NarrowPhase::new();
        self.run_state.timestep_id = 0;
        self.physics.ccd_solver = CCDSolver::new();
        self.physics.query_pipeline = QueryPipeline::new();
        self.physics.pipeline = PhysicsPipeline::new();
        self.physics.pipeline.counters.enable();
    }

    /// Add a plugin
    pub fn add_plugin(&mut self, plugin: impl HarnessPlugin + 'static) {
        self.plugins.push(Box::new(plugin));
    }

    /// Add a callback to be called at each step of the simulation
    pub fn add_callback<F: FnMut(&mut PhysicsState, &PhysicsEvents, &RunState) + 'static>(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    /// Step the simulation
    pub fn step(&mut self) {
        #[cfg(feature = "parallel")]
        {
            let physics = &mut self.physics;
            let event_handler = &self.event_handler;
            self.thread_state.thread_pool.install(|| {
                physics.pipeline.step(
                    &physics.gravity,
                    &physics.integration_parameters,
                    &mut physics.islands,
                    &mut physics.broad_phase,
                    &mut physics.narrow_phase,
                    &mut physics.bodies,
                    &mut physics.colliders,
                    &mut physics.joints,
                    &mut physics.ccd_solver,
                    &*physics.hooks,
                    event_handler,
                );
            });
        }

        #[cfg(not(feature = "parallel"))]
        self.physics.pipeline.step(
            &self.physics.gravity,
            &self.physics.integration_parameters,
            &mut self.physics.islands,
            &mut self.physics.broad_phase,
            &mut self.physics.narrow_phase,
            &mut self.physics.bodies,
            &mut self.physics.colliders,
            &mut self.physics.joints,
            &mut self.physics.ccd_solver,
            &*self.physics.hooks,
            &self.event_handler,
        );

        self.physics.query_pipeline.update(
            &self.physics.islands,
            &self.physics.bodies,
            &self.physics.colliders,
        );

        for plugin in &mut self.plugins {
            plugin.step(&mut self.physics, &self.run_state)
        }

        for f in &mut self.callbacks {
            f(&mut self.physics, &self.events, &self.run_state);
        }

        for plugin in &mut self.plugins {
            plugin.run_callbacks(&mut self.physics, &self.events, &self.run_state)
        }

        self.events.poll_all();

        self.run_state.time += self.physics.integration_parameters.dt as f32;
        self.run_state.timestep_id += 1;
    }

    /// Run the simulation
    // FIXME: it would be good if this was pausable, not sure the best way to go about this
    pub fn run(&mut self) {
        for _ in 0..self.max_steps {
            self.step();
        }
    }
}
