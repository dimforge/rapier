use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;

use crate::engine::GraphicsManager;
#[cfg(feature = "fluids")]
use crate::objects::FluidRenderingMode;
use crate::ui::TestbedUi;
use crossbeam::channel::Receiver;
use kiss3d::camera::Camera;
use kiss3d::event::Event;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3, Vector3};
use rapier::dynamics::{
    ActivationStatus, IntegrationParameters, JointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{BroadPhase, ColliderSet, ContactEvent, NarrowPhase, ProximityEvent, Ray};
use rapier::math::Vector;
use rapier::pipeline::{ChannelEventCollector, PhysicsPipeline, QueryPipeline};
#[cfg(feature = "fluids")]
use salva::{coupling::ColliderCouplingSet, object::FluidHandle, LiquidWorld};

#[cfg(all(feature = "dim2", feature = "other-backends"))]
use crate::box2d_backend::Box2dWorld;
#[cfg(feature = "other-backends")]
use crate::nphysics_backend::NPhysicsWorld;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
use crate::physx_backend::PhysxWorld;

const RAPIER_BACKEND: usize = 0;
const NPHYSICS_BACKEND: usize = 1;
#[cfg(feature = "dim2")]
const BOX2D_BACKEND: usize = 2;
pub(crate) const PHYSX_BACKEND_PATCH_FRICTION: usize = 2;
pub(crate) const PHYSX_BACKEND_TWO_FRICTION_DIR: usize = 3;

#[derive(PartialEq)]
pub enum RunMode {
    Running,
    Stop,
    Step,
    Quit,
}

#[cfg(not(feature = "log"))]
fn usage(exe_name: &str) {
    println!("Usage: {} [OPTION] ", exe_name);
    println!();
    println!("Options:");
    println!("    --help  - prints this help message and exits.");
    println!("    --pause - do not start the simulation right away.");
}

#[cfg(feature = "log")]
fn usage(exe_name: &str) {
    info!("Usage: {} [OPTION] ", exe_name);
    info!("");
    info!("Options:");
    info!("    --help  - prints this help message and exits.");
    info!("    --pause - do not start the simulation right away.");
}

bitflags! {
    #[derive(Default)]
    pub struct TestbedStateFlags: u32 {
        const NONE = 0;
        const SLEEP = 1 << 0;
        const SUB_STEPPING = 1 << 1;
        const SHAPES = 1 << 2;
        const JOINTS = 1 << 3;
        const AABBS = 1 << 4;
        const CONTACT_POINTS = 1 << 5;
        const CONTACT_NORMALS = 1 << 6;
        const CENTER_OF_MASSES = 1 << 7;
        const WIREFRAME = 1 << 8;
        const STATISTICS = 1 << 9;
        const PROFILE = 1 << 10;
        const DEBUG = 1 << 11;
    }
}

bitflags! {
    pub struct TestbedActionFlags: u32 {
        const RESET_WORLD_GRAPHICS = 1 << 0;
        const EXAMPLE_CHANGED = 1 << 1;
        const RESTART = 1 << 2;
        const BACKEND_CHANGED = 1 << 3;
        const TAKE_SNAPSHOT = 1 << 4;
        const RESTORE_SNAPSHOT = 1 << 5;
    }
}

pub struct PhysicsSnapshot {
    timestep_id: usize,
    broad_phase: Vec<u8>,
    narrow_phase: Vec<u8>,
    bodies: Vec<u8>,
    colliders: Vec<u8>,
    joints: Vec<u8>,
}

impl PhysicsSnapshot {
    fn new(
        timestep_id: usize,
        broad_phase: &BroadPhase,
        narrow_phase: &NarrowPhase,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        joints: &JointSet,
    ) -> bincode::Result<Self> {
        Ok(Self {
            timestep_id,
            broad_phase: bincode::serialize(broad_phase)?,
            narrow_phase: bincode::serialize(narrow_phase)?,
            bodies: bincode::serialize(bodies)?,
            colliders: bincode::serialize(colliders)?,
            joints: bincode::serialize(joints)?,
        })
    }

    fn restore(
        &self,
    ) -> bincode::Result<(
        usize,
        BroadPhase,
        NarrowPhase,
        RigidBodySet,
        ColliderSet,
        JointSet,
    )> {
        Ok((
            self.timestep_id,
            bincode::deserialize(&self.broad_phase)?,
            bincode::deserialize(&self.narrow_phase)?,
            bincode::deserialize(&self.bodies)?,
            bincode::deserialize(&self.colliders)?,
            bincode::deserialize(&self.joints)?,
        ))
    }

    fn print_snapshot_len(&self) {
        let total = self.broad_phase.len()
            + self.narrow_phase.len()
            + self.bodies.len()
            + self.colliders.len()
            + self.joints.len();
        println!("Snapshot length: {}B", total);
        println!("|_ broad_phase: {}B", self.broad_phase.len());
        println!("|_ narrow_phase: {}B", self.narrow_phase.len());
        println!("|_ bodies: {}B", self.bodies.len());
        println!("|_ colliders: {}B", self.colliders.len());
        println!("|_ joints: {}B", self.joints.len());
    }
}

pub struct PhysicsEvents {
    pub contact_events: Receiver<ContactEvent>,
    pub proximity_events: Receiver<ProximityEvent>,
}

impl PhysicsEvents {
    fn poll_all(&self) {
        while let Ok(_) = self.contact_events.try_recv() {}
        while let Ok(_) = self.proximity_events.try_recv() {}
    }
}

pub struct PhysicsState {
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub joints: JointSet,
    pub pipeline: PhysicsPipeline,
    pub query_pipeline: QueryPipeline,
}

impl PhysicsState {
    fn new() -> Self {
        Self {
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            joints: JointSet::new(),
            pipeline: PhysicsPipeline::new(),
            query_pipeline: QueryPipeline::new(),
        }
    }
}

pub struct TestbedState {
    pub running: RunMode,
    pub draw_colls: bool,
    pub highlighted_body: Option<RigidBodyHandle>,
    //    pub grabbed_object: Option<DefaultBodyPartHandle>,
    //    pub grabbed_object_constraint: Option<DefaultJointConstraintHandle>,
    pub grabbed_object_plane: (Point3<f32>, Vector3<f32>),
    pub can_grab_behind_ground: bool,
    pub drawing_ray: Option<Point2<f32>>,
    pub prev_flags: TestbedStateFlags,
    pub flags: TestbedStateFlags,
    pub action_flags: TestbedActionFlags,
    pub backend_names: Vec<&'static str>,
    pub example_names: Vec<&'static str>,
    pub selected_example: usize,
    pub selected_backend: usize,
    pub physx_use_two_friction_directions: bool,
    pub num_threads: usize,
    pub snapshot: Option<PhysicsSnapshot>,
    #[cfg(feature = "parallel")]
    pub thread_pool: rapier::rayon::ThreadPool,
    pub timestep_id: usize,
}

#[cfg(feature = "fluids")]
struct FluidsState {
    world: LiquidWorld<f32>,
    coupling: ColliderCouplingSet<f32, RigidBodyHandle>,
}

pub struct Testbed {
    builders: Vec<(&'static str, fn(&mut Testbed))>,
    #[cfg(feature = "fluids")]
    fluids: Option<FluidsState>,
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    physics: PhysicsState,
    graphics: GraphicsManager,
    nsteps: usize,
    camera_locked: bool, // Used so that the camera can remain the same before and after we change backend or press the restart button.
    callbacks: Callbacks,
    #[cfg(feature = "fluids")]
    callbacks_fluids: CallbacksFluids,
    time: f32,
    hide_counters: bool,
    //    persistant_contacts: HashMap<ContactId, bool>,
    font: Rc<Font>,
    cursor_pos: Point2<f32>,
    events: PhysicsEvents,
    event_handler: ChannelEventCollector,
    ui: Option<TestbedUi>,
    state: TestbedState,
    #[cfg(all(feature = "dim2", feature = "other-backends"))]
    box2d: Option<Box2dWorld>,
    #[cfg(all(feature = "dim3", feature = "other-backends"))]
    physx: Option<PhysxWorld>,
    #[cfg(feature = "other-backends")]
    nphysics: Option<NPhysicsWorld>,
}

type Callbacks =
    Vec<Box<dyn FnMut(&mut Window, &mut PhysicsState, &PhysicsEvents, &mut GraphicsManager, f32)>>;

#[cfg(feature = "fluids")]
type CallbacksFluids = Vec<
    Box<
        dyn FnMut(
            &mut Window,
            &mut LiquidWorld<f32>,
            &mut ColliderCouplingSet<f32, RigidBodyHandle>,
            &mut PhysicsState,
            &mut GraphicsManager,
            f32,
        ),
    >,
>;

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let flags = TestbedStateFlags::SLEEP;
        let ui = None;

        #[allow(unused_mut)]
        let mut backend_names = vec!["rapier"];
        #[cfg(feature = "other-backends")]
        backend_names.push("nphysics");
        #[cfg(all(feature = "dim2", feature = "other-backends"))]
        backend_names.push("box2d");
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (patch friction)");
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (two friction dir)");

        #[cfg(feature = "parallel")]
        let num_threads = num_cpus::get_physical();
        #[cfg(not(feature = "parallel"))]
        let num_threads = 1;

        #[cfg(feature = "parallel")]
        let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
            .unwrap();

        let state = TestbedState {
            running: RunMode::Running,
            draw_colls: false,
            highlighted_body: None,
            //            grabbed_object: None,
            //            grabbed_object_constraint: None,
            grabbed_object_plane: (Point3::origin(), na::zero()),
            can_grab_behind_ground: false,
            drawing_ray: None,
            snapshot: None,
            prev_flags: flags,
            flags,
            action_flags: TestbedActionFlags::empty(),
            backend_names,
            example_names: Vec::new(),
            selected_example: 0,
            timestep_id: 0,
            selected_backend: RAPIER_BACKEND,
            physx_use_two_friction_directions: true,
            num_threads,
            #[cfg(feature = "parallel")]
            thread_pool,
        };

        let gravity = Vector::y() * -9.81;
        let integration_parameters = IntegrationParameters::default();
        let contact_channel = crossbeam::channel::unbounded();
        let proximity_channel = crossbeam::channel::unbounded();
        let event_handler = ChannelEventCollector::new(proximity_channel.0, contact_channel.0);
        let events = PhysicsEvents {
            contact_events: contact_channel.1,
            proximity_events: proximity_channel.1,
        };
        let physics = PhysicsState::new();

        Testbed {
            builders: Vec::new(),
            #[cfg(feature = "fluids")]
            fluids: None,
            gravity,
            integration_parameters,
            physics,
            callbacks: Vec::new(),
            #[cfg(feature = "fluids")]
            callbacks_fluids: Vec::new(),
            graphics,
            nsteps: 1,
            camera_locked: false,
            time: 0.0,
            hide_counters: true,
            //            persistant_contacts: HashMap::new(),
            font: Font::default(),
            cursor_pos: Point2::new(0.0f32, 0.0),
            ui,
            event_handler,
            events,
            state,
            #[cfg(all(feature = "dim2", feature = "other-backends"))]
            box2d: None,
            #[cfg(all(feature = "dim3", feature = "other-backends"))]
            physx: None,
            #[cfg(feature = "other-backends")]
            nphysics: None,
        }
    }

    pub fn new(bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) -> Self {
        let mut res = Self::new_empty();
        res.set_world(bodies, colliders, joints);
        res
    }

    pub fn from_builders(default: usize, builders: Vec<(&'static str, fn(&mut Self))>) -> Self {
        let mut res = Testbed::new_empty();
        res.state
            .action_flags
            .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        res.state.selected_example = default;
        res.set_builders(builders);
        res
    }

    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.nsteps = nsteps
    }

    pub fn allow_grabbing_behind_ground(&mut self, allow: bool) {
        self.state.can_grab_behind_ground = allow;
    }

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(&mut self, bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) {
        println!("Num bodies: {}", bodies.len());
        println!("Num joints: {}", joints.len());
        self.physics.bodies = bodies;
        self.physics.colliders = colliders;
        self.physics.joints = joints;
        self.physics.broad_phase = BroadPhase::new();
        self.physics.narrow_phase = NarrowPhase::new();
        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
        self.time = 0.0;
        self.state.timestep_id = 0;
        self.state.highlighted_body = None;
        self.physics.query_pipeline = QueryPipeline::new();
        self.physics.pipeline = PhysicsPipeline::new();
        self.physics.pipeline.counters.enable();

        #[cfg(all(feature = "dim2", feature = "other-backends"))]
        {
            if self.state.selected_backend == BOX2D_BACKEND {
                self.box2d = Some(Box2dWorld::from_rapier(
                    self.gravity,
                    &self.physics.bodies,
                    &self.physics.colliders,
                    &self.physics.joints,
                ));
            }
        }

        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        {
            if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
            {
                self.physx = Some(PhysxWorld::from_rapier(
                    self.gravity,
                    &self.integration_parameters,
                    &self.physics.bodies,
                    &self.physics.colliders,
                    &self.physics.joints,
                    self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR,
                    self.state.num_threads,
                ));
            }
        }

        #[cfg(feature = "other-backends")]
        {
            if self.state.selected_backend == NPHYSICS_BACKEND {
                self.nphysics = Some(NPhysicsWorld::from_rapier(
                    self.gravity,
                    &self.physics.bodies,
                    &self.physics.colliders,
                    &self.physics.joints,
                ));
            }
        }
    }

    #[cfg(feature = "fluids")]
    pub fn set_liquid_world(
        &mut self,
        mut liquid_world: LiquidWorld<f32>,
        coupling: ColliderCouplingSet<f32, RigidBodyHandle>,
    ) {
        liquid_world.counters.enable();
        self.fluids = Some(FluidsState {
            world: liquid_world,
            coupling,
        });
    }

    pub fn set_builders(&mut self, builders: Vec<(&'static str, fn(&mut Self))>) {
        self.state.example_names = builders.iter().map(|e| e.0).collect();
        self.builders = builders
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        if !self.camera_locked {
            self.graphics.look_at(at, zoom);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        if !self.camera_locked {
            self.graphics.look_at(eye, at);
        }
    }

    pub fn set_body_color(&mut self, body: RigidBodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
    }

    #[cfg(feature = "fluids")]
    pub fn set_fluid_color(&mut self, fluid: FluidHandle, color: Point3<f32>) {
        self.graphics.set_fluid_color(fluid, color);
    }

    pub fn set_body_wireframe(&mut self, body: RigidBodyHandle, wireframe_enabled: bool) {
        self.graphics.set_body_wireframe(body, wireframe_enabled);
    }

    #[cfg(feature = "fluids")]
    pub fn set_fluid_rendering_mode(&mut self, mode: FluidRenderingMode) {
        self.graphics.set_fluid_rendering_mode(mode)
    }

    #[cfg(feature = "fluids")]
    pub fn enable_boundary_particles_rendering(&mut self, enabled: bool) {
        self.graphics.enable_boundary_particles_rendering(enabled)
    }

    //    pub fn world(&self) -> &Box<WorldOwner> {
    //        &self.world
    //    }

    pub fn graphics_mut(&mut self) -> &mut GraphicsManager {
        &mut self.graphics
    }

    #[cfg(feature = "dim3")]
    pub fn set_up_axis(&mut self, up_axis: Vector3<f32>) {
        self.graphics.set_up_axis(up_axis);
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices = m.faces().read().unwrap().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.into_iter() {
                flat_indices.push(i.x as usize);
                flat_indices.push(i.y as usize);
                flat_indices.push(i.z as usize);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    fn clear(&mut self, window: &mut Window) {
        self.callbacks.clear();
        #[cfg(feature = "fluids")]
        self.callbacks_fluids.clear();
        //        self.persistant_contacts.clear();
        //        self.state.grabbed_object = None;
        //        self.state.grabbed_object_constraint = None;
        self.state.can_grab_behind_ground = false;
        self.graphics.clear(window);
    }

    pub fn add_callback<
        F: FnMut(&mut Window, &mut PhysicsState, &PhysicsEvents, &mut GraphicsManager, f32) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    #[cfg(feature = "fluids")]
    pub fn add_callback_with_fluids<
        F: FnMut(
                &mut Window,
                &mut LiquidWorld<f32>,
                &mut ColliderCouplingSet<f32, RigidBodyHandle>,
                &mut PhysicsState,
                &mut GraphicsManager,
                f32,
            ) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks_fluids.push(Box::new(callback));
    }

    pub fn run(mut self) {
        let mut args = env::args();
        let mut benchmark_mode = false;

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                } else if &arg[..] == "--pause" {
                    self.state.running = RunMode::Stop;
                } else if &arg[..] == "--bench" {
                    benchmark_mode = true;
                }
            }
        }

        if benchmark_mode {
            use std::fs::File;
            use std::io::{BufWriter, Write};
            // Don't enter the main loop. We will just step the simulation here.
            let mut results = Vec::new();
            let builders = mem::replace(&mut self.builders, Vec::new());
            let backend_names = self.state.backend_names.clone();

            for builder in builders {
                results.clear();
                println!("Running benchmark for {}", builder.0);
                const NUM_ITERS: usize = 1000;

                for (backend_id, backend) in backend_names.iter().enumerate() {
                    println!("|_ using backend {}", backend);
                    self.state.selected_backend = backend_id;
                    if cfg!(feature = "dim3")
                        && (backend_id == PHYSX_BACKEND_PATCH_FRICTION
                            || backend_id == PHYSX_BACKEND_TWO_FRICTION_DIR)
                    {
                        self.integration_parameters.max_velocity_iterations = 1;
                        self.integration_parameters.max_position_iterations = 4;
                    } else {
                        self.integration_parameters.max_velocity_iterations = 4;
                        self.integration_parameters.max_position_iterations = 1;
                    }
                    // Init world.
                    (builder.1)(&mut self);
                    // Run the simulation.
                    let mut timings = Vec::new();
                    for k in 0..=NUM_ITERS {
                        {
                            // FIXME: code duplicated from self.step()
                            if self.state.selected_backend == RAPIER_BACKEND {
                                #[cfg(feature = "parallel")]
                                {
                                    let gravity = &self.gravity;
                                    let params = &self.integration_parameters;
                                    let physics = &mut self.physics;
                                    let event_handler = &self.event_handler;
                                    self.state.thread_pool.install(|| {
                                        physics.pipeline.step(
                                            gravity,
                                            params,
                                            &mut physics.broad_phase,
                                            &mut physics.narrow_phase,
                                            &mut physics.bodies,
                                            &mut physics.colliders,
                                            &mut physics.joints,
                                            event_handler,
                                        );
                                    });
                                }

                                #[cfg(not(feature = "parallel"))]
                                self.physics.pipeline.step(
                                    &self.gravity,
                                    &self.integration_parameters,
                                    &mut self.physics.broad_phase,
                                    &mut self.physics.narrow_phase,
                                    &mut self.physics.bodies,
                                    &mut self.physics.colliders,
                                    &mut self.physics.joints,
                                    &self.event_handler,
                                );

                                self.physics
                                    .query_pipeline
                                    .update(&self.physics.bodies, &self.physics.colliders);

                                #[cfg(feature = "fluids")]
                                {
                                    fluids_time = instant::now();
                                    if let Some(fluids) = &mut self.fluids {
                                        let dt = self.world.timestep();
                                        let gravity = &self.world.gravity;
                                        fluids.world.step_with_coupling(
                                            dt,
                                            gravity,
                                            &mut fluids.coupling.as_manager_mut(
                                                &self.physics.colliders,
                                                &mut self.physics.bodies,
                                            ),
                                        );
                                    }

                                    fluids_time = instant::now() - fluids_time;
                                }
                            }

                            #[cfg(all(feature = "dim2", feature = "other-backends"))]
                            {
                                if self.state.selected_backend == BOX2D_BACKEND {
                                    self.box2d.as_mut().unwrap().step(
                                        &mut self.physics.pipeline.counters,
                                        &self.integration_parameters,
                                    );
                                    self.box2d.as_mut().unwrap().sync(
                                        &mut self.physics.bodies,
                                        &mut self.physics.colliders,
                                    );
                                }
                            }

                            #[cfg(all(feature = "dim3", feature = "other-backends"))]
                            {
                                if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                                    || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                                {
                                    //                        println!("Step");
                                    self.physx.as_mut().unwrap().step(
                                        &mut self.physics.pipeline.counters,
                                        &self.integration_parameters,
                                    );
                                    self.physx.as_mut().unwrap().sync(
                                        &mut self.physics.bodies,
                                        &mut self.physics.colliders,
                                    );
                                }
                            }

                            #[cfg(feature = "other-backends")]
                            {
                                if self.state.selected_backend == NPHYSICS_BACKEND {
                                    self.nphysics.as_mut().unwrap().step(
                                        &mut self.physics.pipeline.counters,
                                        &self.integration_parameters,
                                    );
                                    self.nphysics.as_mut().unwrap().sync(
                                        &mut self.physics.bodies,
                                        &mut self.physics.colliders,
                                    );
                                }
                            }
                        }

                        // Skip the first update.
                        if k > 0 {
                            timings.push(self.physics.pipeline.counters.step_time.time());
                        }
                    }
                    results.push(timings);
                }

                // Write the result as a csv file.
                use inflector::Inflector;
                let filename = format!("{}.csv", builder.0.to_camel_case());
                let mut file = BufWriter::new(File::create(filename).unwrap());

                write!(file, "{}", backend_names[0]).unwrap();
                for backend in &backend_names[1..] {
                    write!(file, ",{}", backend).unwrap();
                }
                writeln!(file).unwrap();

                for i in 0..results[0].len() {
                    write!(file, "{}", results[0][i]).unwrap();
                    for result in &results[1..] {
                        write!(file, ",{}", result[i]).unwrap();
                    }
                    writeln!(file).unwrap();
                }
            }
        } else {
            #[cfg(feature = "dim3")]
            let mut window = Window::new("rapier: 3d demo");
            #[cfg(feature = "dim2")]
            let mut window = Window::new("rapier: 2d demo");
            window.set_background_color(0.85, 0.85, 0.85);
            window.set_framerate_limit(Some(60));
            window.set_light(Light::StickToCamera);
            self.ui = Some(TestbedUi::new(&mut window));
            window.render_loop(self);
        }
    }

    fn handle_common_event<'b>(&mut self, event: Event<'b>) -> Event<'b> {
        match event.value {
            WindowEvent::Key(Key::T, Action::Release, _) => {
                if self.state.running == RunMode::Stop {
                    self.state.running = RunMode::Running;
                } else {
                    self.state.running = RunMode::Stop;
                }
            }
            WindowEvent::Key(Key::S, Action::Release, _) => self.state.running = RunMode::Step,
            WindowEvent::Key(Key::R, Action::Release, _) => self
                .state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true),
            WindowEvent::Key(Key::C, Action::Release, _) => {
                // Delete 1 collider of 10% of the remaining dynamic bodies.
                let mut colliders: Vec<_> = self
                    .physics
                    .bodies
                    .iter()
                    .filter(|e| e.1.is_dynamic())
                    .filter(|e| !e.1.colliders().is_empty())
                    .map(|e| e.1.colliders().to_vec())
                    .collect();
                colliders.sort_by_key(|co| -(co.len() as isize));

                let num_to_delete = (colliders.len() / 10).max(1);
                for to_delete in &colliders[..num_to_delete] {
                    self.physics.pipeline.remove_collider(
                        to_delete[0],
                        &mut self.physics.broad_phase,
                        &mut self.physics.narrow_phase,
                        &mut self.physics.bodies,
                        &mut self.physics.colliders,
                    );
                }
            }
            WindowEvent::Key(Key::D, Action::Release, _) => {
                // Delete 10% of the remaining dynamic bodies.
                let dynamic_bodies: Vec<_> = self
                    .physics
                    .bodies
                    .iter()
                    .filter(|e| !e.1.is_static())
                    .map(|e| e.0)
                    .collect();
                let num_to_delete = (dynamic_bodies.len() / 10).max(1);
                for to_delete in &dynamic_bodies[..num_to_delete] {
                    self.physics.pipeline.remove_rigid_body(
                        *to_delete,
                        &mut self.physics.broad_phase,
                        &mut self.physics.narrow_phase,
                        &mut self.physics.bodies,
                        &mut self.physics.colliders,
                        &mut self.physics.joints,
                    );
                }
            }
            WindowEvent::CursorPos(x, y, _) => {
                self.cursor_pos.x = x as f32;
                self.cursor_pos.y = y as f32;
            }
            _ => {}
        }

        event
    }

    #[cfg(feature = "dim2")]
    fn handle_special_event(&mut self, window: &mut Window, _event: Event) {
        if window.is_conrod_ui_capturing_mouse() {
            return;
        }

        /*
        match event.value {
            WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                let all_groups = &CollisionGroups::new();
                for b in self.geometrical_world.interferences_with_point(
                    &self.colliders,
                    &self.cursor_pos,
                    all_groups,
                ) {
                    if !b.1.query_type().is_proximity_query()
                        && Some(b.1.body()) != self.ground_handle
                    {
                        if let ColliderAnchor::OnBodyPart { body_part, .. } = b.1.anchor() {
                            self.state.grabbed_object = Some(*body_part);
                        } else {
                            continue;
                        }
                    }
                }

                if modifier.contains(Modifiers::Shift) {
                    if let Some(body_part) = self.state.grabbed_object {
                        if Some(body_part.0) != self.ground_handle {
                            self.graphics.remove_body_nodes(window, body_part.0);
                            self.bodies.remove(body_part.0);
                        }
                    }

                    self.state.grabbed_object = None;
                } else if modifier.contains(Modifiers::Alt) {
                    self.state.drawing_ray = Some(self.cursor_pos);
                } else if !modifier.contains(Modifiers::Control) {
                    if let Some(body) = self.state.grabbed_object {
                        if let Some(joint) = self.state.grabbed_object_constraint {
                            let _ = self.constraints.remove(joint);
                        }

                        let body_pos = self
                            .bodies
                            .get(body.0)
                            .unwrap()
                            .part(body.1)
                            .unwrap()
                            .position();
                        let attach1 = self.cursor_pos;
                        let attach2 = body_pos.inverse() * attach1;

                        if let Some(ground) = self.ground_handle {
                            let joint = MouseConstraint::new(
                                BodyPartHandle(ground, 0),
                                body,
                                attach1,
                                attach2,
                                1.0,
                            );
                            self.state.grabbed_object_constraint =
                                Some(self.constraints.insert(joint));
                        }

                        for node in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                            node.select()
                        }
                    }

                    event.inhibited = true;
                } else {
                    self.state.grabbed_object = None;
                }
            }
            WindowEvent::MouseButton(MouseButton::Button1, Action::Release, _) => {
                if let Some(body) = self.state.grabbed_object {
                    for n in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                        n.unselect()
                    }
                }

                if let Some(joint) = self.state.grabbed_object_constraint {
                    let _ = self.constraints.remove(joint);
                }

                if let Some(start) = self.state.drawing_ray {
                    self.graphics
                        .add_ray(Ray::new(start, self.cursor_pos - start));
                }

                self.state.drawing_ray = None;
                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            WindowEvent::CursorPos(x, y, modifiers) => {
                self.cursor_pos.x = x as f32;
                self.cursor_pos.y = y as f32;

                self.cursor_pos = self
                    .graphics
                    .camera()
                    .unproject(&self.cursor_pos, &na::convert(window.size()));

                let attach2 = self.cursor_pos;
                if self.state.grabbed_object.is_some() {
                    if let Some(constraint) = self
                        .state
                        .grabbed_object_constraint
                        .and_then(|joint| self.constraints.get_mut(joint))
                        .and_then(|joint| {
                            joint.downcast_mut::<MouseConstraint<f32, RigidBodyHandle>>()
                        })
                    {
                        constraint.set_anchor_1(attach2);
                    }
                }

                event.inhibited =
                    modifiers.contains(Modifiers::Control) || modifiers.contains(Modifiers::Shift);
            }
            _ => {}
        }
        */
    }

    #[cfg(feature = "dim3")]
    fn handle_special_event(&mut self, window: &mut Window, _event: Event) {
        if window.is_conrod_ui_capturing_mouse() {
            return;
        }

        /*
        match event.value {
            WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                if modifier.contains(Modifiers::Alt) {
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);
                    self.graphics.add_ray(ray);

                    event.inhibited = true;
                } else if modifier.contains(Modifiers::Shift) {
                    // XXX: huge and ugly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (_, b, inter) in self.geometrical_world.interferences_with_ray(
                        &self.colliders,
                        &ray,
                        std::f32::MAX,
                        &all_groups,
                    ) {
                        if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                            mintoi = inter.toi;

                            let subshape = b.shape().subshape_containing_feature(inter.feature);
                            minb = Some(b.body_part(subshape));
                        }
                    }

                    if let Some(body_part) = minb {
                        if modifier.contains(Modifiers::Control) {
                            if Some(body_part.0) != self.ground_handle {
                                self.graphics.remove_body_nodes(window, body_part.0);
                                self.bodies.remove(body_part.0);
                            }
                        } else {
                            self.bodies
                                .get_mut(body_part.0)
                                .unwrap()
                                .apply_force_at_point(
                                    body_part.1,
                                    &(ray.dir.normalize() * 0.01),
                                    &ray.point_at(mintoi),
                                    ForceType::Impulse,
                                    true,
                                );
                        }
                    }

                    event.inhibited = true;
                } else if !modifier.contains(Modifiers::Control) {
                    match self.state.grabbed_object {
                        Some(body) => {
                            for n in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                                n.unselect()
                            }
                        }
                        None => {}
                    }

                    // XXX: huge and uggly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (_, b, inter) in self.geometrical_world.interferences_with_ray(
                        &self.colliders,
                        &ray,
                        std::f32::MAX,
                        &all_groups,
                    ) {
                        if ((Some(b.body()) != self.ground_handle)
                            || self.state.can_grab_behind_ground)
                            && !b.query_type().is_proximity_query()
                            && inter.toi < mintoi
                        {
                            mintoi = inter.toi;

                            let subshape = b.shape().subshape_containing_feature(inter.feature);
                            minb = Some(b.body_part(subshape));
                        }
                    }

                    if let Some(body_part_handle) = minb {
                        if self
                            .bodies
                            .get(body_part_handle.0)
                            .unwrap()
                            .status_dependent_ndofs()
                            != 0
                        {
                            self.state.grabbed_object = minb;
                            for n in self
                                .graphics
                                .body_nodes_mut(body_part_handle.0)
                                .unwrap()
                                .iter_mut()
                            {
                                if let Some(joint) = self.state.grabbed_object_constraint {
                                    let constraint = self.constraints.remove(joint).unwrap();
                                    let (b1, b2) = constraint.anchors();
                                    self.bodies.get_mut(b1.0).unwrap().activate();
                                    self.bodies.get_mut(b2.0).unwrap().activate();
                                }

                                let attach1 = ray.origin + ray.dir * mintoi;
                                let attach2 = {
                                    let body = self.bodies.get_mut(body_part_handle.0).unwrap();
                                    body.activate();
                                    let part = body.part(body_part_handle.1).unwrap();
                                    body.material_point_at_world_point(part, &attach1)
                                };

                                if let Some(ground_handle) = self.ground_handle {
                                    let constraint = MouseConstraint::new(
                                        BodyPartHandle(ground_handle, 0),
                                        body_part_handle,
                                        attach1,
                                        attach2,
                                        1.0,
                                    );
                                    self.state.grabbed_object_plane = (attach1, -ray.dir);
                                    self.state.grabbed_object_constraint =
                                        Some(self.constraints.insert(constraint));
                                }

                                n.select()
                            }
                        }
                    }

                    event.inhibited = true;
                }
            }
            WindowEvent::MouseButton(MouseButton::Button1, Action::Release, _) => {
                if let Some(body_part) = self.state.grabbed_object {
                    for n in self
                        .graphics
                        .body_nodes_mut(body_part.0)
                        .unwrap()
                        .iter_mut()
                    {
                        n.unselect()
                    }
                }

                if let Some(joint) = self.state.grabbed_object_constraint {
                    let constraint = self.constraints.remove(joint).unwrap();
                    let (b1, b2) = constraint.anchors();
                    self.bodies.get_mut(b1.0).unwrap().activate();
                    self.bodies.get_mut(b2.0).unwrap().activate();
                }

                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            _ => {}
        }
        */
    }

    #[cfg(feature = "dim2")]
    fn highlight_hovered_body(&mut self, window: &Window) {
        // Do nothing for now.
    }

    #[cfg(feature = "dim3")]
    fn highlight_hovered_body(&mut self, window: &Window) {
        if let Some(highlighted_body) = self.state.highlighted_body {
            if let Some(nodes) = self.graphics.body_nodes_mut(highlighted_body) {
                for node in nodes {
                    node.unselect()
                }
            }
        }

        let size = window.size();
        let (pos, dir) = self
            .graphics
            .camera()
            .unproject(&self.cursor_pos, &na::convert(size));
        let ray = Ray::new(pos, dir);
        let hit = self
            .physics
            .query_pipeline
            .cast_ray(&self.physics.colliders, &ray, f32::MAX);

        if let Some((_, collider, _)) = hit {
            if self.physics.bodies[collider.parent()].is_dynamic() {
                self.state.highlighted_body = Some(collider.parent());
                for node in self.graphics.body_nodes_mut(collider.parent()).unwrap() {
                    node.select()
                }
            }
        }
    }
}

type CameraEffects<'a> = (
    Option<&'a mut dyn Camera>,
    Option<&'a mut dyn PlanarCamera>,
    Option<&'a mut dyn PostProcessingEffect>,
);

impl State for Testbed {
    fn cameras_and_effect(&mut self) -> CameraEffects<'_> {
        #[cfg(feature = "dim2")]
        let result = (
            None,
            Some(self.graphics.camera_mut() as &mut dyn PlanarCamera),
            None,
        );
        #[cfg(feature = "dim3")]
        let result = (
            Some(self.graphics.camera_mut() as &mut dyn Camera),
            None,
            None,
        );
        result
    }

    fn step(&mut self, window: &mut Window) {
        if let Some(ui) = &mut self.ui {
            ui.update(window, &mut self.integration_parameters, &mut self.state);
        }

        // Handle UI actions.
        {
            let backend_changed = self
                .state
                .action_flags
                .contains(TestbedActionFlags::BACKEND_CHANGED);
            if backend_changed {
                // Marking the example as changed will make the simulation
                // restart with the selected backend.
                self.state
                    .action_flags
                    .set(TestbedActionFlags::BACKEND_CHANGED, false);
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
                self.camera_locked = true;
            }

            let restarted = self
                .state
                .action_flags
                .contains(TestbedActionFlags::RESTART);
            if restarted {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESTART, false);
                self.camera_locked = true;
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
            }

            let example_changed = self
                .state
                .action_flags
                .contains(TestbedActionFlags::EXAMPLE_CHANGED);
            if example_changed {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, false);
                self.clear(window);
                self.builders[self.state.selected_example].1(self);
                self.camera_locked = false;
            }

            if self
                .state
                .action_flags
                .contains(TestbedActionFlags::TAKE_SNAPSHOT)
            {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::TAKE_SNAPSHOT, false);
                self.state.snapshot = PhysicsSnapshot::new(
                    self.state.timestep_id,
                    &self.physics.broad_phase,
                    &self.physics.narrow_phase,
                    &self.physics.bodies,
                    &self.physics.colliders,
                    &self.physics.joints,
                )
                .ok();

                if let Some(snap) = &self.state.snapshot {
                    snap.print_snapshot_len();
                }
            }

            if self
                .state
                .action_flags
                .contains(TestbedActionFlags::RESTORE_SNAPSHOT)
            {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESTORE_SNAPSHOT, false);
                if let Some(snapshot) = &self.state.snapshot {
                    if let Ok(w) = snapshot.restore() {
                        self.clear(window);
                        self.graphics.clear(window);
                        self.physics.broad_phase = w.1;
                        self.physics.narrow_phase = w.2;
                        self.set_world(w.3, w.4, w.5);
                        self.state.timestep_id = w.0;
                    }
                }
            }

            if self
                .state
                .action_flags
                .contains(TestbedActionFlags::RESET_WORLD_GRAPHICS)
            {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, false);
                for (handle, _) in self.physics.bodies.iter() {
                    self.graphics.add(
                        window,
                        handle,
                        &self.physics.bodies,
                        &self.physics.colliders,
                    );
                }

                #[cfg(feature = "fluids")]
                {
                    if let Some(fluids) = &self.fluids {
                        let radius = fluids.world.particle_radius();

                        for (handle, fluid) in fluids.world.fluids().iter() {
                            self.graphics.add_fluid(window, handle, fluid, radius);
                        }

                        for (handle, boundary) in fluids.world.boundaries().iter() {
                            self.graphics.add_boundary(window, handle, boundary, radius);
                        }
                    }
                }
            }

            if example_changed
                || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
                    != self.state.flags.contains(TestbedStateFlags::WIREFRAME)
            {
                self.graphics.toggle_wireframe_mode(
                    &self.physics.colliders,
                    self.state.flags.contains(TestbedStateFlags::WIREFRAME),
                )
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SLEEP)
                != self.state.flags.contains(TestbedStateFlags::SLEEP)
            {
                if self.state.flags.contains(TestbedStateFlags::SLEEP) {
                    for (_, mut body) in self.physics.bodies.iter_mut() {
                        body.activation.threshold = ActivationStatus::default_threshold();
                    }
                } else {
                    for (_, mut body) in self.physics.bodies.iter_mut() {
                        body.wake_up(true);
                        body.activation.threshold = -1.0;
                    }
                }
            }

            if self
                .state
                .prev_flags
                .contains(TestbedStateFlags::SUB_STEPPING)
                != self.state.flags.contains(TestbedStateFlags::SUB_STEPPING)
            {
                self.integration_parameters.return_after_ccd_substep =
                    self.state.flags.contains(TestbedStateFlags::SUB_STEPPING);
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SHAPES)
                != self.state.flags.contains(TestbedStateFlags::SHAPES)
            {
                unimplemented!()
            }

            if self.state.prev_flags.contains(TestbedStateFlags::JOINTS)
                != self.state.flags.contains(TestbedStateFlags::JOINTS)
            {
                unimplemented!()
            }

            if example_changed
                || self.state.prev_flags.contains(TestbedStateFlags::AABBS)
                    != self.state.flags.contains(TestbedStateFlags::AABBS)
            {
                if self.state.flags.contains(TestbedStateFlags::AABBS) {
                    //                    self.graphics
                    //                        .show_aabbs(&self.geometrical_world, &self.colliders, window)
                } else {
                    //                    self.graphics.hide_aabbs(window)
                }
            }

            if self
                .state
                .prev_flags
                .contains(TestbedStateFlags::CENTER_OF_MASSES)
                != self
                    .state
                    .flags
                    .contains(TestbedStateFlags::CENTER_OF_MASSES)
            {
                unimplemented!()
            }
        }

        self.state.prev_flags = self.state.flags;

        for event in window.events().iter() {
            let event = self.handle_common_event(event);
            self.handle_special_event(window, event);
        }

        #[cfg(feature = "fluids")]
        let mut fluids_time = 0.0;

        if self.state.running != RunMode::Stop {
            for _ in 0..self.nsteps {
                self.state.timestep_id += 1;
                if self.state.selected_backend == RAPIER_BACKEND {
                    #[cfg(feature = "parallel")]
                    {
                        let gravity = &self.gravity;
                        let params = &self.integration_parameters;
                        let physics = &mut self.physics;
                        let event_handler = &self.event_handler;
                        self.state.thread_pool.install(|| {
                            physics.pipeline.step(
                                gravity,
                                params,
                                &mut physics.broad_phase,
                                &mut physics.narrow_phase,
                                &mut physics.bodies,
                                &mut physics.colliders,
                                &mut physics.joints,
                                event_handler,
                            );
                        });
                    }

                    #[cfg(not(feature = "parallel"))]
                    self.physics.pipeline.step(
                        &self.gravity,
                        &self.integration_parameters,
                        &mut self.physics.broad_phase,
                        &mut self.physics.narrow_phase,
                        &mut self.physics.bodies,
                        &mut self.physics.colliders,
                        &mut self.physics.joints,
                        &self.event_handler,
                    );

                    self.physics
                        .query_pipeline
                        .update(&self.physics.bodies, &self.physics.colliders);

                    #[cfg(feature = "fluids")]
                    {
                        fluids_time = instant::now();
                        if let Some(fluids) = &mut self.fluids {
                            let dt = self.world.timestep();
                            let gravity = &self.world.gravity;
                            fluids.world.step_with_coupling(
                                dt,
                                gravity,
                                &mut fluids.coupling.as_manager_mut(
                                    &self.physics.colliders,
                                    &mut self.physics.bodies,
                                ),
                            );
                        }

                        fluids_time = instant::now() - fluids_time;
                    }
                }

                #[cfg(all(feature = "dim2", feature = "other-backends"))]
                {
                    if self.state.selected_backend == BOX2D_BACKEND {
                        self.box2d.as_mut().unwrap().step(
                            &mut self.physics.pipeline.counters,
                            &self.integration_parameters,
                        );
                        self.box2d
                            .as_mut()
                            .unwrap()
                            .sync(&mut self.physics.bodies, &mut self.physics.colliders);
                    }
                }

                #[cfg(all(feature = "dim3", feature = "other-backends"))]
                {
                    if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                        || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                    {
                        //                        println!("Step");
                        self.physx.as_mut().unwrap().step(
                            &mut self.physics.pipeline.counters,
                            &self.integration_parameters,
                        );
                        self.physx
                            .as_mut()
                            .unwrap()
                            .sync(&mut self.physics.bodies, &mut self.physics.colliders);
                    }
                }

                #[cfg(feature = "other-backends")]
                {
                    if self.state.selected_backend == NPHYSICS_BACKEND {
                        self.nphysics.as_mut().unwrap().step(
                            &mut self.physics.pipeline.counters,
                            &self.integration_parameters,
                        );
                        self.nphysics
                            .as_mut()
                            .unwrap()
                            .sync(&mut self.physics.bodies, &mut self.physics.colliders);
                    }
                }

                for f in &mut self.callbacks {
                    f(
                        window,
                        &mut self.physics,
                        &self.events,
                        &mut self.graphics,
                        self.time,
                    )
                }

                #[cfg(feature = "fluids")]
                {
                    if let Some(fluid_state) = &mut self.fluids {
                        for f in &mut self.callbacks_fluids {
                            f(
                                window,
                                &mut fluid_state.world,
                                &mut fluid_state.coupling,
                                &mut self.physics,
                                &mut self.graphics,
                                self.time,
                            )
                        }
                    }
                }

                self.events.poll_all();

                //                if true {
                //                    // !self.hide_counters {
                //                    #[cfg(not(feature = "log"))]
                //                    println!("{}", self.world.counters);
                //                    #[cfg(feature = "log")]
                //                    debug!("{}", self.world.counters);
                //                }
                self.time += self.integration_parameters.dt();
            }
        }

        self.highlight_hovered_body(window);
        self.graphics
            .draw(&self.physics.bodies, &self.physics.colliders, window);

        #[cfg(feature = "fluids")]
        {
            if let Some(fluids) = &self.fluids {
                self.graphics.draw_fluids(&fluids.world)
            }
        }

        if self.state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
            draw_contacts(window, &self.physics.narrow_phase, &self.physics.colliders);
        }

        if self.state.running == RunMode::Step {
            self.state.running = RunMode::Stop;
        }

        if self.state.running == RunMode::Quit {
            window.close()
        }

        let color = Point3::new(0.0, 0.0, 0.0);
        let counters = self.physics.pipeline.counters;
        let mut profile = String::new();

        if self.state.flags.contains(TestbedStateFlags::PROFILE) {
            profile = format!(
                r#"{}
Total: {:.2}ms
Collision detection: {:.2}ms
|_ Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
Island computation: {:.2}ms
Solver: {:.2}ms
|_ Velocity assembly: {:.2}ms
   Velocity resolution: {:.2}ms
   Velocity integration: {:.2}ms
   Position assembly: {:.2}ms
   Position resolution: {:.2}ms
CCD: {:.2}ms
|_ # of substeps: {}
   TOI computation: {:.2}ms
   Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
   Solver: {:.2}ms"#,
                profile,
                counters.step_time(),
                counters.collision_detection_time(),
                counters.broad_phase_time(),
                counters.narrow_phase_time(),
                counters.island_construction_time(),
                counters.solver_time(),
                counters.solver.velocity_assembly_time.time(),
                counters.velocity_resolution_time(),
                counters.solver.velocity_update_time.time(),
                counters.solver.position_assembly_time.time(),
                counters.position_resolution_time(),
                counters.ccd_time(),
                counters.ccd.num_substeps,
                counters.ccd.toi_computation_time.time(),
                counters.ccd.broad_phase_time.time(),
                counters.ccd.narrow_phase_time.time(),
                counters.ccd.solver_time.time(),
            );

            #[cfg(feature = "fluids")]
            {
                profile = format!(
                    r#"{}
Fluids: {:.2}ms
                    "#,
                    profile, fluids_time,
                )
            }
        }

        if self.state.flags.contains(TestbedStateFlags::DEBUG) {
            let bf = bincode::serialize(&self.physics.broad_phase).unwrap();
            let nf = bincode::serialize(&self.physics.narrow_phase).unwrap();
            let bs = bincode::serialize(&self.physics.bodies).unwrap();
            let cs = bincode::serialize(&self.physics.colliders).unwrap();
            let js = bincode::serialize(&self.physics.joints).unwrap();
            let hash_bf = md5::compute(&bf);
            let hash_nf = md5::compute(&nf);
            let hash_bodies = md5::compute(&bs);
            let hash_colliders = md5::compute(&cs);
            let hash_joints = md5::compute(&js);
            profile = format!(
                r#"{}
Hashes at frame: {}
|_ Broad phase [{:.1}KB]: {:?}
|_ Narrow phase [{:.1}KB]: {:?}
|_ Bodies [{:.1}KB]: {:?}
|_ Colliders [{:.1}KB]: {:?}
|_ Joints [{:.1}KB]: {:?}"#,
                profile,
                self.state.timestep_id,
                bf.len() as f32 / 1000.0,
                hash_bf,
                nf.len() as f32 / 1000.0,
                hash_nf,
                bs.len() as f32 / 1000.0,
                hash_bodies,
                cs.len() as f32 / 1000.0,
                hash_colliders,
                js.len() as f32 / 1000.0,
                hash_joints
            );
        }

        window.draw_text(&profile, &Point2::origin(), 45.0, &self.font, &color);
    }
}

fn draw_contacts(window: &mut Window, nf: &NarrowPhase, colliders: &ColliderSet) {
    for (_, _, pair) in nf.contact_graph().interaction_pairs() {
        for manifold in &pair.manifolds {
            for pt in manifold.all_contacts() {
                let color = if pt.dist > 0.0 {
                    Point3::new(0.0, 0.0, 1.0)
                } else {
                    Point3::new(1.0, 0.0, 0.0)
                };
                let pos1 = colliders[manifold.pair.collider1].position();
                let pos2 = colliders[manifold.pair.collider2].position();
                let start = pos1 * pt.local_p1;
                let end = pos2 * pt.local_p2;
                let n = pos1 * manifold.local_n1;

                use crate::engine::GraphicsWindow;
                window.draw_graphics_line(&start, &(start + n * 0.4), &Point3::new(0.5, 1.0, 0.5));
                window.draw_graphics_line(&start, &end, &color);
            }
        }
    }
}
