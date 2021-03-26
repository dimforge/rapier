use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;

use crate::physics::{PhysicsEvents, PhysicsSnapshot, PhysicsState};
use crate::plugin::TestbedPlugin;
use crate::ui::TestbedUi;
use crate::{engine::GraphicsManager, harness::RunState};

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
use rapier::geometry::{ColliderHandle, ColliderSet, NarrowPhase};
#[cfg(feature = "dim3")]
use rapier::geometry::{InteractionGroups, Ray};
use rapier::math::{Isometry, Vector};
use rapier::pipeline::PhysicsHooks;

#[cfg(all(feature = "dim2", feature = "other-backends"))]
use crate::box2d_backend::Box2dWorld;
use crate::harness::Harness;
#[cfg(feature = "other-backends")]
use crate::nphysics_backend::NPhysicsWorld;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
use crate::physx_backend::PhysxWorld;

const RAPIER_BACKEND: usize = 0;
#[cfg(feature = "other-backends")]
const NPHYSICS_BACKEND: usize = 1;
#[cfg(all(feature = "dim2", feature = "other-backends"))]
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
}

pub struct Testbed {
    builders: Vec<(&'static str, fn(&mut Testbed))>,
    graphics: GraphicsManager,
    nsteps: usize,
    camera_locked: bool, // Used so that the camera can remain the same before and after we change backend or press the restart button.
    plugins: Vec<Box<dyn TestbedPlugin>>,
    hide_counters: bool,
    //    persistant_contacts: HashMap<ContactId, bool>,
    font: Rc<Font>,
    cursor_pos: Point2<f32>,
    ui: Option<TestbedUi>,
    state: TestbedState,
    harness: Harness,
    #[cfg(all(feature = "dim2", feature = "other-backends"))]
    box2d: Option<Box2dWorld>,
    #[cfg(all(feature = "dim3", feature = "other-backends"))]
    physx: Option<PhysxWorld>,
    #[cfg(feature = "other-backends")]
    nphysics: Option<NPhysicsWorld>,
}

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
        };

        let harness = Harness::new_empty();

        Testbed {
            builders: Vec::new(),
            plugins: Vec::new(),
            graphics,
            nsteps: 1,
            camera_locked: false,
            hide_counters: true,
            //            persistant_contacts: HashMap::new(),
            font: Font::default(),
            cursor_pos: Point2::new(0.0f32, 0.0),
            ui,
            state,
            harness,
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
        res.harness.set_world(bodies, colliders, joints);
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

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.harness.physics.integration_parameters
    }

    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.harness.physics
    }

    pub fn harness_mut(&mut self) -> &mut Harness {
        &mut self.harness
    }

    pub fn set_world(&mut self, bodies: RigidBodySet, colliders: ColliderSet, joints: JointSet) {
        self.set_world_with_params(bodies, colliders, joints, Vector::y() * -9.81, ())
    }

    pub fn set_world_with_params(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        joints: JointSet,
        gravity: Vector<f32>,
        hooks: impl PhysicsHooks + 'static,
    ) {
        self.harness
            .set_world_with_params(bodies, colliders, joints, gravity, hooks);

        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);

        self.state.highlighted_body = None;

        #[cfg(all(feature = "dim2", feature = "other-backends"))]
        {
            if self.state.selected_backend == BOX2D_BACKEND {
                self.box2d = Some(Box2dWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.joints,
                ));
            }
        }

        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        {
            if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
            {
                self.physx = Some(PhysxWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.integration_parameters,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.joints,
                    self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR,
                    self.harness.state.num_threads,
                ));
            }
        }

        #[cfg(feature = "other-backends")]
        {
            if self.state.selected_backend == NPHYSICS_BACKEND {
                self.nphysics = Some(NPhysicsWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.joints,
                ));
            }
        }
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

    pub fn set_collider_initial_color(&mut self, collider: ColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_initial_color(collider, color);
    }

    pub fn set_body_wireframe(&mut self, body: RigidBodyHandle, wireframe_enabled: bool) {
        self.graphics.set_body_wireframe(body, wireframe_enabled);
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
        //        self.persistant_contacts.clear();
        //        self.state.grabbed_object = None;
        //        self.state.grabbed_object_constraint = None;
        self.state.can_grab_behind_ground = false;
        self.graphics.clear(window);

        for plugin in &mut self.plugins {
            plugin.clear_graphics(window);
        }

        self.plugins.clear();
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
        self.harness.add_callback(callback);
    }

    pub fn add_plugin(&mut self, plugin: impl TestbedPlugin + 'static) {
        self.plugins.push(Box::new(plugin));
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

        // TODO: move this to dedicated benchmarking code
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
                        self.harness
                            .physics
                            .integration_parameters
                            .max_velocity_iterations = 1;
                        self.harness
                            .physics
                            .integration_parameters
                            .max_position_iterations = 4;
                    } else {
                        self.harness
                            .physics
                            .integration_parameters
                            .max_velocity_iterations = 4;
                        self.harness
                            .physics
                            .integration_parameters
                            .max_position_iterations = 1;
                    }
                    // Init world.
                    (builder.1)(&mut self);
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
                                    self.box2d.as_mut().unwrap().step(
                                        &mut self.harness.physics.pipeline.counters,
                                        &self.harness.physics.integration_parameters,
                                    );
                                    self.box2d.as_mut().unwrap().sync(
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
                                    self.physx.as_mut().unwrap().step(
                                        &mut self.harness.physics.pipeline.counters,
                                        &self.harness.physics.integration_parameters,
                                    );
                                    self.physx.as_mut().unwrap().sync(
                                        &mut self.harness.physics.bodies,
                                        &mut self.harness.physics.colliders,
                                    );
                                }
                            }

                            #[cfg(feature = "other-backends")]
                            {
                                if self.state.selected_backend == NPHYSICS_BACKEND {
                                    self.nphysics.as_mut().unwrap().step(
                                        &mut self.harness.physics.pipeline.counters,
                                        &self.harness.physics.integration_parameters,
                                    );
                                    self.nphysics.as_mut().unwrap().sync(
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
                    self.harness.physics.colliders.remove(
                        to_delete[0],
                        &mut self.harness.physics.bodies,
                        true,
                    );
                }
            }
            WindowEvent::Key(Key::D, Action::Release, _) => {
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
                    self.harness.physics.bodies.remove(
                        *to_delete,
                        &mut self.harness.physics.colliders,
                        &mut self.harness.physics.joints,
                    );
                }
            }
            WindowEvent::Key(Key::J, Action::Release, _) => {
                // Delete 10% of the remaining joints.
                let joints: Vec<_> = self.harness.physics.joints.iter().map(|e| e.0).collect();
                let num_to_delete = (joints.len() / 10).max(1);
                for to_delete in &joints[..num_to_delete] {
                    self.harness.physics.joints.remove(
                        *to_delete,
                        &mut self.harness.physics.bodies,
                        true,
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
                        let attach2 = body_pos.inv_mul(&attach1);

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
    fn handle_special_event(&mut self, window: &mut Window, event: Event) {
        use rapier::dynamics::RigidBodyBuilder;
        use rapier::geometry::ColliderBuilder;

        if window.is_conrod_ui_capturing_mouse() {
            return;
        }

        match event.value {
            WindowEvent::Key(Key::Space, Action::Release, _) => {
                let cam_pos = self.graphics.camera().view_transform().inverse();
                let forward = cam_pos * -Vector::z();
                let vel = forward * 1000.0;

                let bodies = &mut self.harness.physics.bodies;
                let colliders = &mut self.harness.physics.colliders;

                let collider = ColliderBuilder::cuboid(4.0, 2.0, 0.4).density(20.0).build();
                // let collider = ColliderBuilder::ball(2.0).density(1.0).build();
                let body = RigidBodyBuilder::new_dynamic()
                    .position(cam_pos)
                    .linvel(vel.x, vel.y, vel.z)
                    .ccd_enabled(true)
                    .build();

                let body_handle = bodies.insert(body);
                colliders.insert(collider, body_handle, bodies);
                self.graphics.add(window, body_handle, bodies, colliders);
            }
            _ => {}
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
    fn highlight_hovered_body(&mut self, _window: &Window) {
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
        let physics = &self.harness.physics;
        let hit = physics.query_pipeline.cast_ray(
            &physics.colliders,
            &ray,
            f32::MAX,
            true,
            InteractionGroups::all(),
        );

        if let Some((handle, _)) = hit {
            let collider = &physics.colliders[handle];
            if physics.bodies[collider.parent()].is_dynamic() {
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
        let prev_example = self.state.selected_example;

        if let Some(ui) = &mut self.ui {
            ui.update(
                window,
                &mut self.harness.physics.integration_parameters,
                &mut self.state,
                &mut self.harness.state,
            );
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
                self.harness.clear_callbacks();

                if self.state.selected_example != prev_example {
                    self.harness.physics.integration_parameters = IntegrationParameters::default();

                    if cfg!(feature = "dim3")
                        && (self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                            || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR)
                    {
                        std::mem::swap(
                            &mut self
                                .harness
                                .physics
                                .integration_parameters
                                .max_velocity_iterations,
                            &mut self
                                .harness
                                .physics
                                .integration_parameters
                                .max_position_iterations,
                        )
                    }
                }

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
                    self.harness.state.timestep_id,
                    &self.harness.physics.broad_phase,
                    &self.harness.physics.narrow_phase,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.joints,
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

                        for plugin in &mut self.plugins {
                            plugin.clear_graphics(window);
                        }

                        self.set_world(w.3, w.4, w.5);
                        self.harness.physics.broad_phase = w.1;
                        self.harness.physics.narrow_phase = w.2;
                        self.harness.state.timestep_id = w.0;
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
                for (handle, _) in self.harness.physics.bodies.iter() {
                    self.graphics.add(
                        window,
                        handle,
                        &self.harness.physics.bodies,
                        &self.harness.physics.colliders,
                    );
                }

                for plugin in &mut self.plugins {
                    let graphics = &mut self.graphics;
                    plugin.init_graphics(window, &mut || graphics.next_color());
                }
            }

            if example_changed
                || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
                    != self.state.flags.contains(TestbedStateFlags::WIREFRAME)
            {
                self.graphics.toggle_wireframe_mode(
                    &self.harness.physics.colliders,
                    self.state.flags.contains(TestbedStateFlags::WIREFRAME),
                )
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SLEEP)
                != self.state.flags.contains(TestbedStateFlags::SLEEP)
            {
                if self.state.flags.contains(TestbedStateFlags::SLEEP) {
                    for (_, mut body) in self.harness.physics.bodies.iter_mut() {
                        body.activation.threshold = ActivationStatus::default_threshold();
                    }
                } else {
                    for (_, mut body) in self.harness.physics.bodies.iter_mut() {
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
                self.harness
                    .physics
                    .integration_parameters
                    .return_after_ccd_substep =
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

        if self.state.running != RunMode::Stop {
            for _ in 0..self.nsteps {
                if self.state.selected_backend == RAPIER_BACKEND {
                    let graphics = &mut self.graphics;
                    self.harness
                        .step_with_graphics(Some(window), Some(graphics));

                    for plugin in &mut self.plugins {
                        plugin.step(&mut self.harness.physics)
                    }
                }

                #[cfg(all(feature = "dim2", feature = "other-backends"))]
                {
                    if self.state.selected_backend == BOX2D_BACKEND {
                        self.box2d.as_mut().unwrap().step(
                            &mut self.harness.physics.pipeline.counters,
                            &self.harness.physics.integration_parameters,
                        );
                        self.box2d.as_mut().unwrap().sync(
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
                        self.physx.as_mut().unwrap().step(
                            &mut self.harness.physics.pipeline.counters,
                            &self.harness.physics.integration_parameters,
                        );
                        self.physx.as_mut().unwrap().sync(
                            &mut self.harness.physics.bodies,
                            &mut self.harness.physics.colliders,
                        );
                    }
                }

                #[cfg(feature = "other-backends")]
                {
                    if self.state.selected_backend == NPHYSICS_BACKEND {
                        self.nphysics.as_mut().unwrap().step(
                            &mut self.harness.physics.pipeline.counters,
                            &self.harness.physics.integration_parameters,
                        );
                        self.nphysics.as_mut().unwrap().sync(
                            &mut self.harness.physics.bodies,
                            &mut self.harness.physics.colliders,
                        );
                    }
                }

                for plugin in &mut self.plugins {
                    plugin.run_callbacks(window, &mut self.harness.physics, &self.harness.state);
                }

                //                if true {
                //                    // !self.hide_counters {
                //                    #[cfg(not(feature = "log"))]
                //                    println!("{}", self.world.counters);
                //                    #[cfg(feature = "log")]
                //                    debug!("{}", self.world.counters);
                //                }
            }
        }

        self.highlight_hovered_body(window);

        let physics = &self.harness.physics;
        self.graphics
            .draw(&physics.bodies, &physics.colliders, window);

        for plugin in &mut self.plugins {
            plugin.draw();
        }

        if self.state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
            draw_contacts(window, &physics.narrow_phase, &physics.colliders);
        }

        if self.state.running == RunMode::Step {
            self.state.running = RunMode::Stop;
        }

        if self.state.running == RunMode::Quit {
            window.close()
        }

        let color = Point3::new(0.0, 0.0, 0.0);
        let counters = physics.pipeline.counters;
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

            for plugin in &self.plugins {
                let plugin_profile = plugin.profiling_string();
                profile = format!("{}\n{}", profile, plugin_profile,)
            }
        }

        if self.state.flags.contains(TestbedStateFlags::DEBUG) {
            let t = instant::now();
            let physics = &self.harness.physics;
            // let t = instant::now();
            let bf = bincode::serialize(&physics.broad_phase).unwrap();
            // println!("bf: {}", instant::now() - t);
            // let t = instant::now();
            let nf = bincode::serialize(&physics.narrow_phase).unwrap();
            // println!("nf: {}", instant::now() - t);
            // let t = instant::now();
            let bs = bincode::serialize(&physics.bodies).unwrap();
            // println!("bs: {}", instant::now() - t);
            // let t = instant::now();
            let cs = bincode::serialize(&physics.colliders).unwrap();
            // println!("cs: {}", instant::now() - t);
            // let t = instant::now();
            let js = bincode::serialize(&physics.joints).unwrap();
            // println!("js: {}", instant::now() - t);
            let serialization_time = instant::now() - t;
            let hash_bf = md5::compute(&bf);
            let hash_nf = md5::compute(&nf);
            let hash_bodies = md5::compute(&bs);
            let hash_colliders = md5::compute(&cs);
            let hash_joints = md5::compute(&js);
            profile = format!(
                r#"{}
Serialization time: {:.2}ms
Hashes at frame: {}
|_ Broad phase [{:.1}KB]: {:?}
|_ Narrow phase [{:.1}KB]: {:?}
|_ Bodies [{:.1}KB]: {:?}
|_ Colliders [{:.1}KB]: {:?}
|_ Joints [{:.1}KB]: {:?}"#,
                profile,
                serialization_time,
                self.harness.state.timestep_id,
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
    for pair in nf.contact_pairs() {
        for manifold in &pair.manifolds {
            for contact in &manifold.data.solver_contacts {
                let color = if contact.dist > 0.0 {
                    Point3::new(0.0, 0.0, 1.0)
                } else {
                    Point3::new(1.0, 0.0, 0.0)
                };

                let p = contact.point;
                let n = manifold.data.normal;

                use crate::engine::GraphicsWindow;
                window.draw_graphics_line(&p, &(p + n * 0.4), &Point3::new(0.5, 1.0, 0.5));
            }
            /*
            for pt in manifold.contacts() {
                let color = if pt.dist > 0.0 {
                    Point3::new(0.0, 0.0, 1.0)
                } else {
                    Point3::new(1.0, 0.0, 0.0)
                };
                let pos1 = colliders[pair.pair.collider1].position();
                let pos2 = colliders[pair.pair.collider2].position();
                let start =
                    pos1 * manifold.subshape_pos1.unwrap_or(Isometry::identity()) * pt.local_p1;
                let end =
                    pos2 * manifold.subshape_pos2.unwrap_or(Isometry::identity()) * pt.local_p2;
                let n = pos1
                    * manifold.subshape_pos1.unwrap_or(Isometry::identity())
                    * manifold.local_n1;

                use crate::engine::GraphicsWindow;
                window.draw_graphics_line(&start, &(start + n * 0.4), &Point3::new(0.5, 1.0, 0.5));
                window.draw_graphics_line(&start, &end, &color);
            }
             */
        }
    }
}
