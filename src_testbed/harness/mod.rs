use crate::{
    physics::{PhysicsEvents, PhysicsState},
    GraphicsManager,
};
use kiss3d::window::Window;
use plugin::HarnessPlugin;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier::math::Vector;
use rapier::pipeline::{ChannelEventCollector, PhysicsPipeline, QueryPipeline};

pub mod plugin;

pub struct RunState {
    #[cfg(feature = "parallel")]
    pub thread_pool: rapier::rayon::ThreadPool,
    #[cfg(feature = "parallel")]
    pub num_threads: usize,
    pub timestep_id: usize,
    pub time: f32,
}

impl RunState {
    pub fn new() -> Self {
        #[cfg(feature = "parallel")]
        let num_threads = num_cpus::get_physical();

        #[cfg(feature = "parallel")]
        let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
            .unwrap();

        Self {
            #[cfg(feature = "parallel")]
            thread_pool: thread_pool,
            #[cfg(feature = "parallel")]
            num_threads,
            timestep_id: 0,
            time: 0.0,
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

type Callbacks = Vec<
    Box<
        dyn FnMut(
            Option<&mut Window>,
            Option<&mut GraphicsManager>,
            &mut PhysicsState,
            &PhysicsEvents,
            &RunState,
        ),
    >,
>;

#[allow(dead_code)]
impl Harness {
    pub fn new_empty() -> Self {
        #[cfg(feature = "parallel")]
        let num_threads = num_cpus::get_physical();

        #[cfg(feature = "parallel")]
        let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
            .unwrap();

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
            state,
        }
    }

    pub fn new(bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) -> Self {
        let mut res = Self::new_empty();
        res.set_world(bodies, colliders, joints);
        res
    }

    pub fn set_max_steps(&mut self, max_steps: usize) {
        self.max_steps = max_steps
    }

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.physics.integration_parameters
    }

    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.physics
    }

    pub fn set_world(&mut self, bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) {
        self.set_world_with_gravity(bodies, colliders, joints, Vector::y() * -9.81)
    }

    pub fn set_world_with_gravity(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        joints: JointSet,
        gravity: Vector<f32>,
    ) {
        // println!("Num bodies: {}", bodies.len());
        // println!("Num joints: {}", joints.len());
        self.physics.gravity = gravity;
        self.physics.bodies = bodies;
        self.physics.colliders = colliders;
        self.physics.joints = joints;
        self.physics.broad_phase = BroadPhase::new();
        self.physics.narrow_phase = NarrowPhase::new();
        self.state.timestep_id = 0;
        self.physics.query_pipeline = QueryPipeline::new();
        self.physics.pipeline = PhysicsPipeline::new();
        self.physics.pipeline.counters.enable();
    }

    pub fn add_plugin(&mut self, plugin: impl HarnessPlugin + 'static) {
        self.plugins.push(Box::new(plugin));
    }

    pub fn add_callback<
        F: FnMut(
                Option<&mut Window>,
                Option<&mut GraphicsManager>,
                &mut PhysicsState,
                &PhysicsEvents,
                &RunState,
            ) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn step(&mut self) {
        self.step_with_graphics(None, None);
    }

    pub fn step_with_graphics(
        &mut self,
        window: Option<&mut Window>,
        graphics: Option<&mut GraphicsManager>,
    ) {
        #[cfg(feature = "parallel")]
        {
            let physics = &mut self.physics;
            let event_handler = &self.event_handler;
            self.state.thread_pool.install(|| {
                physics.pipeline.step(
                    &physics.gravity,
                    &physics.integration_parameters,
                    &mut physics.broad_phase,
                    &mut physics.narrow_phase,
                    &mut physics.bodies,
                    &mut physics.colliders,
                    &mut physics.joints,
                    None,
                    None,
                    event_handler,
                );
            });
        }

        #[cfg(not(feature = "parallel"))]
        self.physics.pipeline.step(
            &self.physics.gravity,
            &self.physics.integration_parameters,
            &mut self.physics.broad_phase,
            &mut self.physics.narrow_phase,
            &mut self.physics.bodies,
            &mut self.physics.colliders,
            &mut self.physics.joints,
            None,
            None,
            &self.event_handler,
        );

        self.physics
            .query_pipeline
            .update(&self.physics.bodies, &self.physics.colliders);

        for plugin in &mut self.plugins {
            plugin.step(&mut self.physics, &self.state)
        }

        // FIXME: This assumes either window & graphics are Some, or they are all None
        // this is required as we cannot pass Option<&mut Window> & Option<&mut GraphicsManager directly in a loop
        // there must be a better way of doing this?
        match (window, graphics) {
            (Some(window), Some(graphics)) => {
                for f in &mut self.callbacks {
                    f(
                        Some(window),
                        Some(graphics),
                        &mut self.physics,
                        &self.events,
                        &self.state,
                    );
                }
            }
            _ => {
                for f in &mut self.callbacks {
                    f(None, None, &mut self.physics, &self.events, &self.state);
                }
            }
        }

        for plugin in &mut self.plugins {
            plugin.run_callbacks(&mut self.physics, &self.events, &self.state)
        }

        self.events.poll_all();

        self.state.time += self.physics.integration_parameters.dt;
        self.state.timestep_id += 1;
    }

    pub fn run(&mut self) {
        for _ in 0..self.max_steps {
            self.step();
        }
    }
}
