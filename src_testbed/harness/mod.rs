#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

use crate::{
    physics::{PhysicsEvents, PhysicsState},
    TestbedGraphics,
};
use plugin::HarnessPlugin;
use rapier::geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase};
use rapier::math::{Real, Vector};
use rapier::pipeline::{ChannelEventCollector, PhysicsHooks, PhysicsPipeline, QueryPipeline};
use rapier::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodySet,
    },
    prelude::PhysicsContext,
};

pub mod plugin;

pub struct RunState {
    #[cfg(feature = "parallel")]
    pub thread_pool: rapier::rayon::ThreadPool,
    #[cfg(feature = "parallel")]
    num_threads: usize,
    pub timestep_id: usize,
    pub time: f32,
}

impl Default for RunState {
    fn default() -> Self {
        Self::new()
    }
}

impl RunState {
    #[cfg(feature = "parallel")]
    pub fn new() -> Self {
        let num_threads = num_cpus::get_physical();

        let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
            .unwrap();

        Self {
            thread_pool,
            num_threads,
            timestep_id: 0,
            time: 0.0,
        }
    }

    #[cfg(not(feature = "parallel"))]
    pub fn new() -> Self {
        Self {
            timestep_id: 0,
            time: 0.0,
        }
    }

    #[cfg(feature = "parallel")]
    pub fn num_threads(&self) -> usize {
        self.num_threads
    }

    #[cfg(not(feature = "parallel"))]
    pub fn num_threads(&self) -> usize {
        1
    }

    #[cfg(feature = "parallel")]
    pub fn set_num_threads(&mut self, num_threads: usize) {
        if self.num_threads != num_threads {
            self.thread_pool = rapier::rayon::ThreadPoolBuilder::new()
                .num_threads(num_threads)
                .build()
                .unwrap();
            self.num_threads = num_threads;
        }
    }
}

pub struct Harness {
    pub physics: PhysicsState,
    max_steps: usize,
    callbacks: Callbacks,
    plugins: Vec<Box<dyn HarnessPlugin>>,
    events: PhysicsEvents,
    event_handler: ChannelEventCollector,
    pub state: RunState,
}

type Callbacks =
    Vec<Box<dyn FnMut(Option<&mut TestbedGraphics>, &mut PhysicsState, &PhysicsEvents, &RunState)>>;

#[allow(dead_code)]
impl Harness {
    pub fn new_empty() -> Self {
        let collision_event_channel = crossbeam::channel::unbounded();
        let contact_force_event_channel = crossbeam::channel::unbounded();
        let event_handler =
            ChannelEventCollector::new(collision_event_channel.0, contact_force_event_channel.0);
        let events = PhysicsEvents {
            collision_events: collision_event_channel.1,
            contact_force_events: contact_force_event_channel.1,
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
            state,
        }
    }

    pub fn new(
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
    ) -> Self {
        let mut res = Self::new_empty();
        res.set_world(bodies, colliders, impulse_joints, multibody_joints);
        res
    }

    pub fn set_max_steps(&mut self, max_steps: usize) {
        self.max_steps = max_steps
    }

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.physics.context.integration_parameters
    }

    pub fn clear_callbacks(&mut self) {
        self.callbacks.clear();
    }

    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.physics
    }

    pub fn set_world(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
    ) {
        self.set_world_with_params(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            Vector::y() * -9.81,
            (),
        )
    }

    pub fn set_world_with_params(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
        gravity: Vector<Real>,
        hooks: impl PhysicsHooks + 'static,
    ) {
        // println!("Num bodies: {}", bodies.len());
        // println!("Num impulse_joints: {}", impulse_joints.len());
        self.physics = PhysicsState {
            context: PhysicsContext {
                gravity,
                integration_parameters: IntegrationParameters::default(),
                physics_pipeline: PhysicsPipeline::new(),
                island_manager: IslandManager::new(),
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                ccd_solver: CCDSolver::new(),
                query_pipeline: Some(QueryPipeline::new()),
            },
            hooks: Box::new(hooks),
        };
        self.physics.context.physics_pipeline.counters.enable();
        self.state.timestep_id = 0;
        self.state.time = 0.0;
    }

    pub fn add_plugin(&mut self, plugin: impl HarnessPlugin + 'static) {
        self.plugins.push(Box::new(plugin));
    }

    pub fn add_callback<
        F: FnMut(Option<&mut TestbedGraphics>, &mut PhysicsState, &PhysicsEvents, &RunState) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn step(&mut self) {
        self.step_with_graphics(None);
    }

    #[profiling::function]
    pub fn step_with_graphics(&mut self, mut graphics: Option<&mut TestbedGraphics>) {
        #[cfg(feature = "parallel")]
        {
            let physics = &mut self.physics;
            let event_handler = &self.event_handler;
            self.state.thread_pool.install(|| {
                physics.context.step(&*physics.hooks, event_handler);
            });
        }

        #[cfg(not(feature = "parallel"))]
        self.physics
            .context
            .step(&*self.physics.hooks, &self.event_handler);

        for plugin in &mut self.plugins {
            plugin.step(&mut self.physics, &self.state)
        }

        for f in &mut self.callbacks {
            f(
                graphics.as_deref_mut(),
                &mut self.physics,
                &self.events,
                &self.state,
            );
        }

        for plugin in &mut self.plugins {
            plugin.run_callbacks(&mut self.physics, &self.events, &self.state)
        }

        self.events.poll_all();

        self.state.time += self.physics.context.integration_parameters.dt as f32;
        self.state.timestep_id += 1;
    }

    pub fn run(&mut self) {
        for _ in 0..self.max_steps {
            self.step();
        }
    }
}
