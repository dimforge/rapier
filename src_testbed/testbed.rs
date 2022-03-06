use std::env;
use std::mem;

use bevy::prelude::*;

use crate::physics::{PhysicsEvents, PhysicsSnapshot, PhysicsState};
use crate::plugin::TestbedPlugin;
use crate::ui;
use crate::{graphics::GraphicsManager, harness::RunState};

use na::{self, Point2, Point3, Vector3};
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, MultibodyJointSet, RigidBodyActivation,
    RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{ColliderHandle, ColliderSet, NarrowPhase};
#[cfg(feature = "dim3")]
use rapier::geometry::{InteractionGroups, Ray};
use rapier::math::{Real, Vector};
use rapier::pipeline::PhysicsHooks;

#[cfg(all(feature = "dim2", feature = "other-backends"))]
use crate::box2d_backend::Box2dWorld;
use crate::harness::Harness;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
use crate::physx_backend::PhysxWorld;
use bevy::pbr::wireframe::WireframePlugin;
use bevy::render::camera::Camera;
use bevy::render::options::{WgpuFeatures, WgpuOptions};
use bevy_egui::EguiContext;

#[cfg(feature = "dim2")]
use crate::camera2d::{OrbitCamera, OrbitCameraPlugin};
#[cfg(feature = "dim3")]
use crate::camera3d::{OrbitCamera, OrbitCameraPlugin};
use crate::graphics::BevyMaterial;
// use bevy::render::render_resource::RenderPipelineDescriptor;

const RAPIER_BACKEND: usize = 0;
#[cfg(all(feature = "dim2", feature = "other-backends"))]
const BOX2D_BACKEND: usize = 1;
pub(crate) const PHYSX_BACKEND_PATCH_FRICTION: usize = 1;
pub(crate) const PHYSX_BACKEND_TWO_FRICTION_DIR: usize = 2;

#[derive(PartialEq)]
pub enum RunMode {
    Running,
    Stop,
    Step,
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
    pub snapshot: Option<PhysicsSnapshot>,
    nsteps: usize,
    camera_locked: bool, // Used so that the camera can remain the same before and after we change backend or press the restart button.
}

struct SceneBuilders(Vec<(&'static str, fn(&mut Testbed))>);

#[cfg(feature = "other-backends")]
struct OtherBackends {
    #[cfg(feature = "dim2")]
    box2d: Option<Box2dWorld>,
    #[cfg(feature = "dim3")]
    physx: Option<PhysxWorld>,
}
struct Plugins(Vec<Box<dyn TestbedPlugin>>);

pub struct TestbedGraphics<'a, 'b, 'c, 'd, 'e, 'f> {
    graphics: &'a mut GraphicsManager,
    commands: &'a mut Commands<'d, 'e>,
    meshes: &'a mut Assets<Mesh>,
    materials: &'a mut Assets<BevyMaterial>,
    components: &'a mut Query<'b, 'f, (&'c mut Transform,)>,
    camera: &'a mut OrbitCamera,
}

pub struct Testbed<'a, 'b, 'c, 'd, 'e, 'f> {
    graphics: Option<TestbedGraphics<'a, 'b, 'c, 'd, 'e, 'f>>,
    harness: &'a mut Harness,
    state: &'a mut TestbedState,
    #[cfg(feature = "other-backends")]
    other_backends: &'a mut OtherBackends,
    plugins: &'a mut Plugins,
}

pub struct TestbedApp {
    builders: SceneBuilders,
    graphics: GraphicsManager,
    state: TestbedState,
    harness: Harness,
    #[cfg(feature = "other-backends")]
    other_backends: OtherBackends,
    plugins: Plugins,
}

impl TestbedApp {
    pub fn new_empty() -> Self {
        let graphics = GraphicsManager::new();
        let flags = TestbedStateFlags::SLEEP;

        #[allow(unused_mut)]
        let mut backend_names = vec!["rapier"];
        #[cfg(all(feature = "dim2", feature = "other-backends"))]
        backend_names.push("box2d");
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (patch friction)");
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (two friction dir)");

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
            selected_backend: RAPIER_BACKEND,
            physx_use_two_friction_directions: true,
            nsteps: 1,
            camera_locked: false,
        };

        let harness = Harness::new_empty();
        #[cfg(feature = "other-backends")]
        let other_backends = OtherBackends {
            #[cfg(feature = "dim2")]
            box2d: None,
            #[cfg(feature = "dim3")]
            physx: None,
        };

        TestbedApp {
            builders: SceneBuilders(Vec::new()),
            plugins: Plugins(Vec::new()),
            graphics,
            state,
            harness,
            #[cfg(feature = "other-backends")]
            other_backends,
        }
    }

    pub fn from_builders(default: usize, builders: Vec<(&'static str, fn(&mut Testbed))>) -> Self {
        let mut res = TestbedApp::new_empty();
        res.state
            .action_flags
            .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        res.state.selected_example = default;
        res.set_builders(builders);
        res
    }

    pub fn set_builders(&mut self, builders: Vec<(&'static str, fn(&mut Testbed))>) {
        self.state.example_names = builders.iter().map(|e| e.0).collect();
        self.builders = SceneBuilders(builders)
    }

    pub fn run(self) {
        self.run_with_init(|_| {})
    }

    pub fn run_with_init(mut self, mut init: impl FnMut(&mut App)) {
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

        // TODO: move this to dedicated benchmarking code
        if benchmark_mode {
            use std::fs::File;
            use std::io::{BufWriter, Write};
            // Don't enter the main loop. We will just step the simulation here.
            let mut results = Vec::new();
            let builders = mem::replace(&mut self.builders.0, Vec::new());
            let backend_names = self.state.backend_names.clone();

            for builder in builders {
                results.clear();
                println!("Running benchmark for {}", builder.0);
                const NUM_ITERS: usize = 1000;

                for (backend_id, backend) in backend_names.iter().enumerate() {
                    println!("|_ using backend {}", backend);
                    self.state.selected_backend = backend_id;
                    self.harness
                        .physics
                        .integration_parameters
                        .max_velocity_iterations = 4;
                    self.harness
                        .physics
                        .integration_parameters
                        .max_stabilization_iterations = 1;

                    // Init world.
                    let mut testbed = Testbed {
                        graphics: None,
                        state: &mut self.state,
                        harness: &mut self.harness,
                        #[cfg(feature = "other-backends")]
                        other_backends: &mut self.other_backends,
                        plugins: &mut self.plugins,
                    };
                    (builder.1)(&mut testbed);
                    // Run the simulation.
                    let mut timings = Vec::new();
                    for k in 0..=NUM_ITERS {
                        {
                            if self.state.selected_backend == RAPIER_BACKEND {
                                self.harness.step();
                            }

                            #[cfg(all(feature = "dim2", feature = "other-backends"))]
                            {
                                if self.state.selected_backend == BOX2D_BACKEND {
                                    self.other_backends.box2d.as_mut().unwrap().step(
                                        &mut self.harness.physics.pipeline.counters,
                                        &self.harness.physics.integration_parameters,
                                    );
                                    self.other_backends.box2d.as_mut().unwrap().sync(
                                        &mut self.harness.physics.bodies,
                                        &mut self.harness.physics.colliders,
                                    );
                                }
                            }

                            #[cfg(all(feature = "dim3", feature = "other-backends"))]
                            {
                                if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                                    || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                                {
                                    //                        println!("Step");
                                    self.other_backends.physx.as_mut().unwrap().step(
                                        &mut self.harness.physics.pipeline.counters,
                                        &self.harness.physics.integration_parameters,
                                    );
                                    self.other_backends.physx.as_mut().unwrap().sync(
                                        &mut self.harness.physics.bodies,
                                        &mut self.harness.physics.colliders,
                                    );
                                }
                            }
                        }

                        // Skip the first update.
                        if k > 0 {
                            timings.push(self.harness.physics.pipeline.counters.step_time.time());
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
            let title = if cfg!(feature = "dim2") {
                "Rapier: 2D demos".to_string()
            } else {
                "Rapier: 3D demos".to_string()
            };

            let mut app = App::new();

            app.insert_resource(WindowDescriptor {
                title,
                vsync: true,
                ..Default::default()
            })
            .insert_resource(ClearColor(Color::rgb(0.15, 0.15, 0.15)))
            .insert_resource(Msaa { samples: 4 })
            .insert_resource(WgpuOptions {
                // Required for wireframes.
                features: WgpuFeatures::POLYGON_MODE_LINE,
                ..Default::default()
            })
            .insert_resource(AmbientLight {
                brightness: 0.3,
                ..Default::default()
            })
            .add_plugins(DefaultPlugins)
            .add_plugin(OrbitCameraPlugin)
            .add_plugin(WireframePlugin)
            .add_plugin(bevy_egui::EguiPlugin);

            #[cfg(target_arch = "wasm32")]
            app.add_plugin(bevy_webgl2::WebGL2Plugin);

            #[cfg(feature = "other-backends")]
            app.insert_non_send_resource(self.other_backends);

            app.add_startup_system(setup_graphics_environment.system())
                .insert_non_send_resource(self.graphics)
                .insert_resource(self.state)
                .insert_non_send_resource(self.harness)
                .insert_resource(self.builders)
                .insert_non_send_resource(self.plugins)
                .add_stage_before(CoreStage::Update, "physics", SystemStage::single_threaded())
                .add_system_to_stage("physics", update_testbed.system())
                .add_system(egui_focus.system());
            init(&mut app);
            app.run();
        }
    }
}

impl<'a, 'b, 'c, 'd, 'e, 'f> TestbedGraphics<'a, 'b, 'c, 'd, 'e, 'f> {
    pub fn set_body_color(&mut self, body: RigidBodyHandle, color: [f32; 3]) {
        self.graphics
            .set_body_color(&mut self.materials, body, color);
    }

    pub fn add_body(
        &mut self,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        self.graphics.add_body_colliders(
            &mut *self.commands,
            &mut *self.meshes,
            &mut *self.materials,
            &mut *self.components,
            handle,
            bodies,
            colliders,
        )
    }

    pub fn remove_collider(&mut self, handle: ColliderHandle, colliders: &ColliderSet) {
        if let Some(parent_handle) = colliders.get(handle).map(|c| c.parent()) {
            self.graphics
                .remove_collider_nodes(&mut *self.commands, parent_handle, handle)
        }
    }

    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.graphics.remove_body_nodes(&mut *self.commands, handle)
    }

    pub fn add_collider(&mut self, handle: ColliderHandle, colliders: &ColliderSet) {
        self.graphics.add_collider(
            &mut *self.commands,
            &mut *self.meshes,
            &mut *self.materials,
            handle,
            colliders,
        )
    }
}

impl<'a, 'b, 'c, 'd, 'e, 'f> Testbed<'a, 'b, 'c, 'd, 'e, 'f> {
    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.state.nsteps = nsteps
    }

    pub fn allow_grabbing_behind_ground(&mut self, allow: bool) {
        self.state.can_grab_behind_ground = allow;
    }

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.harness.physics.integration_parameters
    }

    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.harness.physics
    }

    pub fn harness_mut(&mut self) -> &mut Harness {
        &mut self.harness
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
        hooks: impl PhysicsHooks<RigidBodySet, ColliderSet> + 'static,
    ) {
        self.harness.set_world_with_params(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            gravity,
            hooks,
        );

        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);

        self.state.highlighted_body = None;

        #[cfg(all(feature = "dim2", feature = "other-backends"))]
        {
            if self.state.selected_backend == BOX2D_BACKEND {
                self.other_backends.box2d = Some(Box2dWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.impulse_joints,
                ));
            }
        }

        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        {
            if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
            {
                self.other_backends.physx = Some(PhysxWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.integration_parameters,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.impulse_joints,
                    &self.harness.physics.multibody_joints,
                    self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR,
                    self.harness.state.num_threads,
                ));
            }
        }
    }

    pub fn set_graphics_shift(&mut self, shift: Vector<Real>) {
        if !self.state.camera_locked {
            if let Some(graphics) = &mut self.graphics {
                graphics.graphics.gfx_shift = shift;
            }
        }
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        if !self.state.camera_locked {
            if let Some(graphics) = &mut self.graphics {
                graphics.camera.center.x = at.x;
                graphics.camera.center.y = at.y;
                graphics.camera.zoom = zoom;
            }
        }
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        if !self.state.camera_locked {
            if let Some(graphics) = &mut self.graphics {
                graphics.camera.center.x = at.x;
                graphics.camera.center.y = at.y;
                graphics.camera.center.z = at.z;

                let view_dir = eye - at;
                graphics.camera.distance = view_dir.norm();

                if graphics.camera.distance > 0.0 {
                    graphics.camera.y = (view_dir.y / graphics.camera.distance).acos();
                    graphics.camera.x =
                        (-view_dir.z).atan2(view_dir.x) - std::f32::consts::FRAC_PI_2;
                }
            }
        }
    }

    pub fn set_initial_body_color(&mut self, body: RigidBodyHandle, color: [f32; 3]) {
        if let Some(graphics) = &mut self.graphics {
            graphics.graphics.set_initial_body_color(body, color);
        }
    }

    pub fn set_initial_collider_color(&mut self, collider: ColliderHandle, color: [f32; 3]) {
        if let Some(graphics) = &mut self.graphics {
            graphics
                .graphics
                .set_initial_collider_color(collider, color);
        }
    }

    pub fn set_body_wireframe(&mut self, body: RigidBodyHandle, wireframe_enabled: bool) {
        if let Some(graphics) = &mut self.graphics {
            graphics
                .graphics
                .set_body_wireframe(body, wireframe_enabled);
        }
    }

    //    pub fn world(&self) -> &Box<WorldOwner> {
    //        &self.world
    //    }

    pub fn add_callback<
        F: FnMut(Option<&mut TestbedGraphics>, &mut PhysicsState, &PhysicsEvents, &RunState) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.harness.add_callback(callback);
    }

    pub fn add_plugin(&mut self, mut plugin: impl TestbedPlugin + 'static) {
        plugin.init_plugin();
        self.plugins.0.push(Box::new(plugin));
    }

    fn handle_common_events(&mut self, events: &Input<KeyCode>) {
        for key in events.get_just_released() {
            match *key {
                KeyCode::T => {
                    if self.state.running == RunMode::Stop {
                        self.state.running = RunMode::Running;
                    } else {
                        self.state.running = RunMode::Stop;
                    }
                }
                KeyCode::S => self.state.running = RunMode::Step,
                KeyCode::R => self
                    .state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true),
                KeyCode::C => {
                    // Delete 1 collider of 10% of the remaining dynamic bodies.
                    let mut colliders: Vec<_> = self
                        .harness
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
                        if let Some(graphics) = self.graphics.as_mut() {
                            graphics.remove_collider(to_delete[0], &self.harness.physics.colliders);
                        }
                        self.harness.physics.colliders.remove(
                            to_delete[0],
                            &mut self.harness.physics.islands,
                            &mut self.harness.physics.bodies,
                            true,
                        );
                    }
                }
                KeyCode::D => {
                    // Delete 10% of the remaining dynamic bodies.
                    let dynamic_bodies: Vec<_> = self
                        .harness
                        .physics
                        .bodies
                        .iter()
                        .filter(|e| !e.1.is_static())
                        .map(|e| e.0)
                        .collect();
                    let num_to_delete = (dynamic_bodies.len() / 10).max(1);
                    for to_delete in &dynamic_bodies[..num_to_delete] {
                        if let Some(graphics) = self.graphics.as_mut() {
                            graphics.remove_body(*to_delete);
                        }
                        self.harness.physics.bodies.remove(
                            *to_delete,
                            &mut self.harness.physics.islands,
                            &mut self.harness.physics.colliders,
                            &mut self.harness.physics.impulse_joints,
                            &mut self.harness.physics.multibody_joints,
                        );
                    }
                }
                KeyCode::J => {
                    // Delete 10% of the remaining impulse_joints.
                    let impulse_joints: Vec<_> = self
                        .harness
                        .physics
                        .impulse_joints
                        .iter()
                        .map(|e| e.0)
                        .collect();
                    let num_to_delete = (impulse_joints.len() / 10).max(1);
                    for to_delete in &impulse_joints[..num_to_delete] {
                        self.harness.physics.impulse_joints.remove(
                            *to_delete,
                            &mut self.harness.physics.islands,
                            &mut self.harness.physics.bodies,
                            true,
                        );
                    }
                }
                KeyCode::A => {
                    // Delete 10% of the remaining multibody_joints.
                    let multibody_joints: Vec<_> = self
                        .harness
                        .physics
                        .multibody_joints
                        .iter()
                        .map(|e| e.0)
                        .collect();
                    let num_to_delete = (multibody_joints.len() / 10).max(1);
                    for to_delete in &multibody_joints[..num_to_delete] {
                        self.harness.physics.multibody_joints.remove(
                            *to_delete,
                            &mut self.harness.physics.islands,
                            &mut self.harness.physics.bodies,
                            true,
                        );
                    }
                }
                KeyCode::M => {
                    // Delete one remaining multibody.
                    let to_delete = self
                        .harness
                        .physics
                        .multibody_joints
                        .iter()
                        .next()
                        .map(|a| a.2.rigid_body_handle());
                    if let Some(to_delete) = to_delete {
                        self.harness
                            .physics
                            .multibody_joints
                            .remove_multibody_articulations(
                                to_delete,
                                &mut self.harness.physics.islands,
                                &mut self.harness.physics.bodies,
                                true,
                            );
                    }
                }
                _ => {}
            }
        }
    }

    // #[cfg(feature = "dim2")]
    // fn handle_special_event(&mut self) {}
    //
    // #[cfg(feature = "dim3")]
    // fn handle_special_event(&mut self) {
    //     use rapier::dynamics::RigidBodyBuilder;
    //     use rapier::geometry::ColliderBuilder;
    //
    //     if window.is_conrod_ui_capturing_mouse() {
    //         return;
    //     }
    //
    //     match event.value {
    //         WindowEvent::Key(Key::Space, Action::Release, _) => {
    //             let cam_pos = self.graphics.camera().view_transform().inverse();
    //             let forward = cam_pos * -Vector::z();
    //             let vel = forward * 1000.0;
    //
    //             let bodies = &mut self.harness.physics.bodies;
    //             let colliders = &mut self.harness.physics.colliders;
    //
    //             let collider = ColliderBuilder::cuboid(4.0, 2.0, 0.4).density(20.0).build();
    //             // let collider = ColliderBuilder::ball(2.0).density(1.0).build();
    //             let body = RigidBodyBuilder::new_dynamic()
    //                 .position(cam_pos)
    //                 .linvel(vel.x, vel.y, vel.z)
    //                 .ccd_enabled(true)
    //                 .build();
    //
    //             let body_handle = bodies.insert(body);
    //             colliders.insert(collider, body_handle, bodies);
    //             self.graphics.add(window, body_handle, bodies, colliders);
    //         }
    //         _ => {}
    //     }
    // }
}

fn draw_contacts(_nf: &NarrowPhase, _colliders: &ColliderSet) {
    // use rapier::math::Isometry;
    //
    // for pair in nf.contact_pairs() {
    //     for manifold in &pair.manifolds {
    //         /*
    //         for contact in &manifold.data.solver_contacts {
    //             let p = contact.point;
    //             let n = manifold.data.normal;
    //
    //             use crate::engine::GraphicsWindow;
    //             window.draw_graphics_line(&p, &(p + n * 0.4), &point![0.5, 1.0, 0.5]);
    //         }
    //         */
    //         for pt in manifold.contacts() {
    //             let color = if pt.dist > 0.0 {
    //                 point![0.0, 0.0, 1.0]
    //             } else {
    //                 point![1.0, 0.0, 0.0]
    //             };
    //             let pos1 = colliders[pair.pair.collider1].position();
    //             let pos2 = colliders[pair.pair.collider2].position();
    //             let start =
    //                 pos1 * manifold.subshape_pos1.unwrap_or(Isometry::identity()) * pt.local_p1;
    //             let end =
    //                 pos2 * manifold.subshape_pos2.unwrap_or(Isometry::identity()) * pt.local_p2;
    //             let n = pos1
    //                 * manifold.subshape_pos1.unwrap_or(Isometry::identity())
    //                 * manifold.local_n1;
    //
    //             // window.draw_graphics_line(&start, &(start + n * 0.4), &point![0.5, 1.0, 0.5]);
    //             // window.draw_graphics_line(&start, &end, &color);
    //         }
    //     }
    // }
}

#[cfg(feature = "dim3")]
fn setup_graphics_environment(mut commands: Commands) {
    const HALF_SIZE: f32 = 100.0;

    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            // Configure the projection to better fit the scene
            shadow_projection: OrthographicProjection {
                left: -HALF_SIZE,
                right: HALF_SIZE,
                bottom: -HALF_SIZE,
                top: HALF_SIZE,
                near: -10.0 * HALF_SIZE,
                far: 100.0 * HALF_SIZE,
                ..Default::default()
            },
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform {
            translation: Vec3::new(10.0, 2.0, 10.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_4),
            ..Default::default()
        },
        ..Default::default()
    });

    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_matrix(Mat4::face_toward(
                Vec3::new(-30.0, 30.0, 100.0),
                Vec3::new(0.0, 10.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )),
            ..Default::default()
        })
        .insert(OrbitCamera {
            rotate_sensitivity: 0.05,
            ..OrbitCamera::default()
        });
}

#[cfg(feature = "dim2")]
fn setup_graphics_environment(mut commands: Commands) {
    // commands.insert_resource(AmbientLight {
    //     brightness: 0.3,
    //     ..Default::default()
    // });
    // commands.spawn_bundle(LightBundle {
    //     transform: Transform::from_translation(Vec3::new(0.0, 0.0, 2000.0)),
    //     light: Light {
    //         intensity: 100_000_000.0,
    //         range: 6000.0,
    //         ..Default::default()
    //     },
    //     ..Default::default()
    // });
    commands
        .spawn_bundle(OrthographicCameraBundle {
            transform: Transform {
                translation: Vec3::new(0.0, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::new(0.01, 0.01, 1.0),
            },
            ..OrthographicCameraBundle::new_2d()
        })
        .insert(OrbitCamera {
            zoom: 100.0,
            pan_sensitivity: 0.02,
            ..OrbitCamera::default()
        });
}

fn egui_focus(ui_context: Res<EguiContext>, mut cameras: Query<&mut OrbitCamera>) {
    let mut camera_enabled = true;
    if ui_context.ctx().wants_pointer_input() {
        camera_enabled = false;
    }
    for mut camera in cameras.iter_mut() {
        camera.enabled = camera_enabled;
    }
}

fn update_testbed(
    mut commands: Commands,
    windows: Res<Windows>,
    // mut pipelines: ResMut<Assets<RenderPipelineDescriptor>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<BevyMaterial>>,
    builders: NonSendMut<SceneBuilders>,
    mut graphics: NonSendMut<GraphicsManager>,
    mut state: ResMut<TestbedState>,
    mut harness: NonSendMut<Harness>,
    #[cfg(feature = "other-backends")] mut other_backends: NonSendMut<OtherBackends>,
    mut plugins: NonSendMut<Plugins>,
    ui_context: Res<EguiContext>,
    mut gfx_components: Query<(&mut Transform,)>,
    mut cameras: Query<(&Camera, &GlobalTransform, &mut OrbitCamera)>,
    keys: Res<Input<KeyCode>>,
) {
    let meshes = &mut *meshes;
    let materials = &mut *materials;
    let prev_example = state.selected_example;

    // Handle inputs
    {
        let graphics_context = TestbedGraphics {
            graphics: &mut *graphics,
            commands: &mut commands,
            meshes: &mut *meshes,
            materials: &mut *materials,
            components: &mut gfx_components,
            camera: &mut cameras.iter_mut().next().unwrap().2,
        };

        let mut testbed = Testbed {
            graphics: Some(graphics_context),
            state: &mut *state,
            harness: &mut *harness,
            #[cfg(feature = "other-backends")]
            other_backends: &mut *other_backends,
            plugins: &mut *plugins,
        };

        testbed.handle_common_events(&*keys);
    }

    // Update UI
    {
        let harness = &mut *harness;
        ui::update_ui(&ui_context, &mut state, harness);

        for plugin in &mut plugins.0 {
            plugin.update_ui(
                &ui_context,
                harness,
                &mut graphics,
                &mut commands,
                &mut *meshes,
                &mut *materials,
                &mut gfx_components,
            );
        }
    }

    // Handle UI actions.
    {
        let backend_changed = state
            .action_flags
            .contains(TestbedActionFlags::BACKEND_CHANGED);
        if backend_changed {
            // Marking the example as changed will make the simulation
            // restart with the selected backend.
            state
                .action_flags
                .set(TestbedActionFlags::BACKEND_CHANGED, false);
            state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
            state.camera_locked = true;
        }

        let restarted = state.action_flags.contains(TestbedActionFlags::RESTART);
        if restarted {
            state.action_flags.set(TestbedActionFlags::RESTART, false);
            state.camera_locked = true;
            state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        }

        let example_changed = state
            .action_flags
            .contains(TestbedActionFlags::EXAMPLE_CHANGED);
        if example_changed {
            state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, false);
            clear(&mut commands, &mut state, &mut graphics, &mut plugins);
            harness.clear_callbacks();
            for plugin in (*plugins).0.iter_mut() {
                plugin.clear_graphics(&mut graphics, &mut commands);
            }
            (*plugins).0.clear();

            if state.selected_example != prev_example {
                harness.physics.integration_parameters = IntegrationParameters::default();
            }

            let selected_example = state.selected_example;
            let graphics = &mut *graphics;
            let meshes = &mut *meshes;

            let graphics_context = TestbedGraphics {
                graphics: &mut *graphics,
                commands: &mut commands,
                meshes: &mut *meshes,
                materials: &mut *materials,
                components: &mut gfx_components,
                camera: &mut cameras.iter_mut().next().unwrap().2,
            };

            let mut testbed = Testbed {
                graphics: Some(graphics_context),
                state: &mut *state,
                harness: &mut *harness,
                #[cfg(feature = "other-backends")]
                other_backends: &mut *other_backends,
                plugins: &mut *plugins,
            };

            builders.0[selected_example].1(&mut testbed);

            state.camera_locked = false;
        }

        if state
            .action_flags
            .contains(TestbedActionFlags::TAKE_SNAPSHOT)
        {
            state
                .action_flags
                .set(TestbedActionFlags::TAKE_SNAPSHOT, false);
            state.snapshot = PhysicsSnapshot::new(
                harness.state.timestep_id,
                &harness.physics.broad_phase,
                &harness.physics.narrow_phase,
                &harness.physics.bodies,
                &harness.physics.colliders,
                &harness.physics.impulse_joints,
            )
            .ok();

            if let Some(snap) = &state.snapshot {
                snap.print_snapshot_len();
            }
        }

        if state
            .action_flags
            .contains(TestbedActionFlags::RESTORE_SNAPSHOT)
        {
            state
                .action_flags
                .set(TestbedActionFlags::RESTORE_SNAPSHOT, false);
            if let Some(snapshot) = &state.snapshot {
                if let Ok(w) = snapshot.restore() {
                    clear(&mut commands, &mut state, &mut graphics, &mut plugins);

                    for plugin in &mut plugins.0 {
                        plugin.clear_graphics(&mut graphics, &mut commands);
                    }

                    // set_world(w.3, w.4, w.5);
                    harness.physics.broad_phase = w.1;
                    harness.physics.narrow_phase = w.2;
                    harness.state.timestep_id = w.0;
                }
            }
        }

        if state
            .action_flags
            .contains(TestbedActionFlags::RESET_WORLD_GRAPHICS)
        {
            state
                .action_flags
                .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, false);
            for (handle, _) in harness.physics.bodies.iter() {
                graphics.add_body_colliders(
                    &mut commands,
                    meshes,
                    materials,
                    &mut gfx_components,
                    handle,
                    &harness.physics.bodies,
                    &harness.physics.colliders,
                );
            }

            for (handle, _) in harness.physics.colliders.iter() {
                graphics.add_collider(
                    &mut commands,
                    meshes,
                    materials,
                    handle,
                    &harness.physics.colliders,
                );
            }

            for plugin in &mut plugins.0 {
                plugin.init_graphics(
                    &mut graphics,
                    &mut commands,
                    meshes,
                    materials,
                    &mut gfx_components,
                    &mut harness,
                );
            }
        }

        if example_changed
            || state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
                != state.flags.contains(TestbedStateFlags::WIREFRAME)
        {
            // graphics.toggle_wireframe_mode(
            //     &harness.physics.colliders,
            //     state.flags.contains(TestbedStateFlags::WIREFRAME),
            // )
        }

        if state.prev_flags.contains(TestbedStateFlags::SLEEP)
            != state.flags.contains(TestbedStateFlags::SLEEP)
        {
            if state.flags.contains(TestbedStateFlags::SLEEP) {
                for (_, body) in harness.physics.bodies.iter_mut() {
                    body.activation_mut().linear_threshold =
                        RigidBodyActivation::default_linear_threshold();
                    body.activation_mut().angular_threshold =
                        RigidBodyActivation::default_angular_threshold();
                }
            } else {
                for (_, body) in harness.physics.bodies.iter_mut() {
                    body.wake_up(true);
                    body.activation_mut().linear_threshold = -1.0;
                }
            }
        }
    }

    state.prev_flags = state.flags;

    // for event in window.events().iter() {
    //     let event = handle_common_event(event);
    //     handle_special_event(window, event);
    // }

    if state.running != RunMode::Stop {
        for _ in 0..state.nsteps {
            if state.selected_backend == RAPIER_BACKEND {
                let graphics = &mut graphics;

                let mut testbed_graphics = TestbedGraphics {
                    graphics: &mut *graphics,
                    commands: &mut commands,
                    meshes: &mut *meshes,
                    materials: &mut *materials,
                    components: &mut gfx_components,
                    camera: &mut cameras.iter_mut().next().unwrap().2,
                };
                harness.step_with_graphics(Some(&mut testbed_graphics));

                for plugin in &mut plugins.0 {
                    plugin.step(&mut harness.physics)
                }
            }

            #[cfg(all(feature = "dim2", feature = "other-backends"))]
            {
                if state.selected_backend == BOX2D_BACKEND {
                    let harness = &mut *harness;
                    other_backends.box2d.as_mut().unwrap().step(
                        &mut harness.physics.pipeline.counters,
                        &harness.physics.integration_parameters,
                    );
                    other_backends
                        .box2d
                        .as_mut()
                        .unwrap()
                        .sync(&mut harness.physics.bodies, &mut harness.physics.colliders);
                }
            }

            #[cfg(all(feature = "dim3", feature = "other-backends"))]
            {
                if state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                    || state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                {
                    //                        println!("Step");
                    let harness = &mut *harness;
                    other_backends.physx.as_mut().unwrap().step(
                        &mut harness.physics.pipeline.counters,
                        &harness.physics.integration_parameters,
                    );
                    other_backends
                        .physx
                        .as_mut()
                        .unwrap()
                        .sync(&mut harness.physics.bodies, &mut harness.physics.colliders);
                }
            }

            for plugin in &mut plugins.0 {
                plugin.run_callbacks(&mut harness);
            }
        }
    }

    if let Some(window) = windows.get_primary() {
        for (camera, camera_pos, _) in cameras.iter_mut() {
            highlight_hovered_body(
                &mut *materials,
                &mut *graphics,
                &mut *state,
                &harness.physics,
                window,
                camera,
                camera_pos,
            );
        }
    }

    graphics.draw(
        &harness.physics.bodies,
        &harness.physics.colliders,
        &mut gfx_components,
        &mut *materials,
    );

    for plugin in &mut plugins.0 {
        plugin.draw(
            &mut graphics,
            &mut commands,
            meshes,
            materials,
            &mut gfx_components,
            &mut harness,
        );
    }

    if state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
        draw_contacts(&harness.physics.narrow_phase, &harness.physics.colliders);
    }

    if state.running == RunMode::Step {
        state.running = RunMode::Stop;
    }
}

fn clear(
    commands: &mut Commands,
    state: &mut TestbedState,
    graphics: &mut GraphicsManager,
    plugins: &mut Plugins,
) {
    state.can_grab_behind_ground = false;
    graphics.clear(commands);

    for mut plugin in plugins.0.drain(..) {
        plugin.clear_graphics(graphics, commands);
    }
}

#[cfg(feature = "dim2")]
fn highlight_hovered_body(
    _materials: &mut Assets<BevyMaterial>,
    _graphics_manager: &mut GraphicsManager,
    _testbed_state: &mut TestbedState,
    _physics: &PhysicsState,
    _window: &Window,
    _camera: &Camera,
    _camera_transform: &GlobalTransform,
) {
    // Do nothing for now.
}

#[cfg(feature = "dim3")]
fn highlight_hovered_body(
    materials: &mut Assets<BevyMaterial>,
    graphics_manager: &mut GraphicsManager,
    testbed_state: &mut TestbedState,
    physics: &PhysicsState,
    window: &Window,
    camera: &Camera,
    camera_transform: &GlobalTransform,
) {
    if let Some(highlighted_body) = testbed_state.highlighted_body {
        if let Some(nodes) = graphics_manager.body_nodes_mut(highlighted_body) {
            for node in nodes {
                node.unselect(materials)
            }
        }
    }

    if let Some(cursor) = window.cursor_position() {
        let ndc_cursor = (cursor / Vec2::new(window.width(), window.height()) * 2.0) - Vec2::ONE;
        let ndc_to_world = camera_transform.compute_matrix() * camera.projection_matrix.inverse();
        let ray_pt1 = ndc_to_world.project_point3(Vec3::new(ndc_cursor.x, ndc_cursor.y, -1.0));
        let ray_pt2 = ndc_to_world.project_point3(Vec3::new(ndc_cursor.x, ndc_cursor.y, 1.0));
        let ray_dir = ray_pt2 - ray_pt1;
        let ray_origin = Point3::new(ray_pt1.x as Real, ray_pt1.y as Real, ray_pt1.z as Real);
        let ray_dir = Vector3::new(ray_dir.x as Real, ray_dir.y as Real, ray_dir.z as Real);

        let ray = Ray::new(ray_origin, ray_dir);
        let hit = physics.query_pipeline.cast_ray(
            &physics.colliders,
            &ray,
            Real::MAX,
            true,
            InteractionGroups::all(),
            None,
        );

        if let Some((handle, _)) = hit {
            let collider = &physics.colliders[handle];

            if let Some(parent_handle) = collider.parent() {
                if physics.bodies[parent_handle].is_dynamic() {
                    testbed_state.highlighted_body = Some(parent_handle);
                    for node in graphics_manager.body_nodes_mut(parent_handle).unwrap() {
                        node.select(materials)
                    }
                }
            }
        }
    }
}
