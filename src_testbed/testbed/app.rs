//! TestbedApp - the main application runner.

use crate::Camera;
use crate::debug_render::{DebugRenderPipelineResource, debug_render_scene};
use crate::graphics::GraphicsManager;
use crate::harness::Harness;
use crate::mouse::SceneMouse;
use crate::save::SerializableTestbedState;
use crate::testbed::hover::highlight_hovered_body;
use crate::ui;
use kiss3d::color::Color;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::window::Window;
use rapier::dynamics::RigidBodyActivation;
use std::mem;

use super::Plugins;
use super::graphics_context::TestbedGraphics;
use super::keys::KeysState;
use super::state::{RAPIER_BACKEND, RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags};
use super::testbed::{SimulationBuilders, Testbed};

#[cfg(feature = "other-backends")]
use super::OtherBackends;

#[cfg(all(feature = "dim3", feature = "other-backends"))]
use super::state::{PHYSX_BACKEND_PATCH_FRICTION, PHYSX_BACKEND_TWO_FRICTION_DIR};

/// The main testbed application
pub struct TestbedApp {
    builders: SimulationBuilders,
    graphics: GraphicsManager,
    state: TestbedState,
    harness: Harness,
    #[cfg(feature = "other-backends")]
    other_backends: OtherBackends,
    plugins: Plugins,
}

impl TestbedApp {
    pub fn save_file_path() -> String {
        format!("testbed_state_{}.autosave.json", env!("CARGO_CRATE_NAME"))
    }

    pub fn new_empty() -> Self {
        let graphics = GraphicsManager::new();
        let state = TestbedState::default();
        let harness = Harness::new_empty();
        #[cfg(feature = "other-backends")]
        let other_backends = OtherBackends {
            #[cfg(feature = "dim3")]
            physx: None,
        };

        TestbedApp {
            builders: Vec::new(),
            plugins: Plugins(Vec::new()),
            graphics,
            state,
            harness,
            #[cfg(feature = "other-backends")]
            other_backends,
        }
    }

    pub fn from_builders(builders: SimulationBuilders) -> Self {
        let mut res = TestbedApp::new_empty();
        res.set_builders(builders);
        res
    }

    pub fn set_builders(&mut self, builders: SimulationBuilders) {
        use super::state::ExampleEntry;
        use indexmap::IndexSet;

        // Collect unique groups in order of first appearance
        let mut groups: IndexSet<&'static str> = IndexSet::new();
        for example in &builders {
            groups.insert(example.group);
        }

        // Build the display order: group by group, preserving original order within each group
        let mut examples = Vec::new();
        for group in &groups {
            for (builder_index, example) in builders.iter().enumerate() {
                if example.group == *group {
                    examples.push(ExampleEntry {
                        name: example.name,
                        group: example.group,
                        builder_index,
                    });
                }
            }
        }

        self.state.example_groups = groups.into_iter().collect();
        self.state.examples = examples;
        self.builders = builders;
    }

    pub async fn run(self) {
        self.run_with_init(|_| {}).await
    }

    pub async fn run_with_init(mut self, init: impl FnMut(&mut Testbed)) {
        #[cfg(feature = "profiler_ui")]
        profiling::puffin::set_scopes_on(true);

        // Check for benchmark mode
        let args: Vec<String> = std::env::args().collect();
        if args.iter().any(|a| a == "--bench") {
            self.run_benchmark();
            return;
        }

        self.run_async(init).await
    }

    fn run_benchmark(&mut self) {
        use std::fs::File;
        use std::io::{BufWriter, Write};

        let num_bench_iters = 1000u32;
        let builders = mem::take(&mut self.builders);
        let backend_names = self.state.backend_names.clone();

        for builder in &builders {
            let mut results = Vec::new();
            println!("Running benchmark for {}", builder.name);

            for (backend_id, backend) in backend_names.iter().enumerate() {
                println!("|_ using backend {backend}");
                self.state.selected_backend = backend_id;
                self.harness = Harness::new_empty();

                let mut testbed = Testbed {
                    graphics: None,
                    state: &mut self.state,
                    harness: &mut self.harness,
                    #[cfg(feature = "other-backends")]
                    other_backends: &mut self.other_backends,
                    plugins: &mut self.plugins,
                };
                (builder.builder)(&mut testbed);

                let mut timings = Vec::new();
                for k in 0..num_bench_iters {
                    if self.state.selected_backend == RAPIER_BACKEND {
                        self.harness.step();
                    }

                    #[cfg(all(feature = "dim3", feature = "other-backends"))]
                    {
                        if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                            || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                        {
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

                    if k > 0 {
                        timings.push(self.harness.physics.pipeline.counters.step_time.time_ms());
                    }
                }
                results.push(timings);
            }

            use inflector::Inflector;
            let filename = format!("{}.csv", builder.name.to_camel_case());
            let mut file = BufWriter::new(File::create(filename).unwrap());

            write!(file, "{}", backend_names[0]).unwrap();
            for backend in &backend_names[1..] {
                write!(file, ",{backend}").unwrap();
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
    }

    async fn run_async(mut self, mut init: impl FnMut(&mut Testbed)) {
        let title = if cfg!(feature = "dim2") {
            "Rapier: 2D demos"
        } else {
            "Rapier: 3D demos"
        };

        let mut window = Window::new_with_size(title, 1280, 720).await;
        window.set_background_color(Color::new(245.0 / 255.0, 245.0 / 255.0, 236.0 / 255.0, 1.0));

        let mut debug_render = DebugRenderPipelineResource::default();
        let mut camera = Camera::default();
        let mut scene_mouse = SceneMouse::new();
        let mut keys = KeysState::default();

        // User init
        let testbed_gfx = TestbedGraphics {
            graphics: &mut self.graphics,
            window: &mut window,
            camera: &mut camera,
            mouse: &mut scene_mouse,
            keys: &mut keys,
            settings: None,
        };

        let mut testbed = Testbed {
            graphics: Some(testbed_gfx),
            state: &mut self.state,
            harness: &mut self.harness,
            #[cfg(feature = "other-backends")]
            other_backends: &mut self.other_backends,
            plugins: &mut self.plugins,
        };

        init(&mut testbed);

        // Main render loop
        #[cfg(feature = "dim3")]
        while window
            .render_3d(self.graphics.scene_mut(), &mut camera)
            .await
        {
            self.run_frame(
                &mut window,
                &mut debug_render,
                &mut camera,
                &mut scene_mouse,
                &mut keys,
            );
        }

        #[cfg(feature = "dim2")]
        while window
            .render_2d(self.graphics.scene_mut(), &mut camera)
            .await
        {
            self.run_frame(
                &mut window,
                &mut debug_render,
                &mut camera,
                &mut scene_mouse,
                &mut keys,
            );
        }
    }

    fn run_frame(
        &mut self,
        window: &mut Window,
        debug_render: &mut DebugRenderPipelineResource,
        camera: &mut Camera,
        scene_mouse: &mut SceneMouse,
        keys: &mut KeysState,
    ) {
        profiling::finish_frame!();

        // Handle input events
        self.handle_events(window, keys);

        // Handle the vehicle controller if there is one.
        #[cfg(feature = "dim3")]
        {
            self.update_vehicle_controller(keys);
        }

        // Update mouse state
        let cursor_pos = window.cursor_pos();
        scene_mouse.update_from_window(cursor_pos, window.size().into(), camera);

        // Handle action flags
        self.handle_action_flags(window, camera, scene_mouse, keys);

        // Handle sleep settings
        self.handle_sleep_settings();

        // Run simulation
        if self.state.running != RunMode::Stop {
            for _ in 0..self.state.nsteps {
                if self.state.selected_backend == RAPIER_BACKEND {
                    let mut testbed_gfx = TestbedGraphics {
                        graphics: &mut self.graphics,
                        window,
                        camera,
                        mouse: scene_mouse,
                        keys,
                        settings: Some(&mut self.state.example_settings),
                    };
                    self.harness.step_with_graphics(Some(&mut testbed_gfx));

                    for plugin in &mut self.plugins.0 {
                        plugin.step(&mut self.harness.physics);
                    }
                }

                #[cfg(all(feature = "dim3", feature = "other-backends"))]
                {
                    if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                        || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
                    {
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

                for plugin in &mut self.plugins.0 {
                    plugin.run_callbacks(&mut self.harness);
                }
            }

            if self.state.running == RunMode::Step {
                self.state.running = RunMode::Stop;
            }
        }

        // Autosave state.
        #[cfg(not(target_arch = "wasm32"))]
        {
            let new_save_data = self.state.save_data(*camera);
            if self.state.prev_save_data != new_save_data {
                // Save the data in a file.
                let data = serde_json::to_string_pretty(&new_save_data).unwrap();
                if let Err(e) = std::fs::write(Self::save_file_path(), &data) {
                    eprintln!("Failed to write autosave file: {}", e);
                }
                self.state.prev_save_data = new_save_data;
            }
        }

        highlight_hovered_body(&mut self.graphics, scene_mouse, &self.harness.physics);

        // Update graphics
        self.graphics.draw(
            self.state.flags,
            &self.harness.physics.bodies,
            &self.harness.physics.colliders,
        );

        // Draw debug render
        debug_render_scene(window, debug_render, &self.harness);

        // Draw UI
        window.draw_ui(|ctx| {
            ui::update_ui(ctx, &mut self.state, &mut self.harness, debug_render);
        });

        self.state.prev_flags = self.state.flags;
    }

    fn handle_events(&mut self, window: &mut Window, keys: &mut KeysState) {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(key, Action::Press, _) => {
                    // Track pressed keys
                    if !keys.pressed_keys.contains(&key) {
                        keys.pressed_keys.push(key);
                    }
                    // Update modifier states
                    match key {
                        Key::LShift | Key::RShift => keys.shift = true,
                        Key::LControl | Key::RControl => keys.ctrl = true,
                        Key::LAlt | Key::RAlt => keys.alt = true,
                        _ => {}
                    }
                }
                WindowEvent::Key(key, Action::Release, _) => {
                    // Remove from pressed keys
                    keys.pressed_keys.retain(|k| *k != key);
                    // Handle special keys
                    match key {
                        Key::T => {
                            if self.state.running == RunMode::Stop {
                                self.state.running = RunMode::Running;
                            } else {
                                self.state.running = RunMode::Stop;
                            }
                        }
                        Key::S => {
                            self.state.running = RunMode::Step;
                        }
                        Key::R => {
                            self.state
                                .action_flags
                                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
                        }
                        Key::LShift | Key::RShift => keys.shift = false,
                        Key::LControl | Key::RControl => keys.ctrl = false,
                        Key::LAlt | Key::RAlt => keys.alt = false,
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }

    #[cfg(feature = "dim3")]
    fn update_vehicle_controller(&mut self, keys: &mut KeysState) {
        use rapier::prelude::QueryFilter;

        if self.state.running == RunMode::Stop {
            return;
        }

        if let Some(vehicle) = &mut self.state.vehicle_controller {
            let mut engine_force = 0.0;
            let mut steering_angle = 0.0;

            println!("Pressed: {:?}", keys);
            if keys.pressed(Key::Right) {
                steering_angle += -0.7;
            }
            if keys.pressed(Key::Left) {
                steering_angle += 0.7;
            }
            if keys.pressed(Key::Up) {
                engine_force += 30.0;
            }
            if keys.pressed(Key::Down) {
                engine_force += -30.0;
            }

            let wheels = vehicle.wheels_mut();
            wheels[0].engine_force = engine_force;
            wheels[0].steering = steering_angle;
            wheels[1].engine_force = engine_force;
            wheels[1].steering = steering_angle;

            let query_pipeline = self.harness.physics.broad_phase.as_query_pipeline_mut(
                self.harness.physics.narrow_phase.query_dispatcher(),
                &mut self.harness.physics.bodies,
                &mut self.harness.physics.colliders,
                QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle.chassis),
            );

            vehicle.update_vehicle(
                self.harness.physics.integration_parameters.dt,
                query_pipeline,
            );
        }
    }

    fn handle_action_flags(
        &mut self,
        window: &mut Window,
        camera: &mut Camera,
        scene_mouse: &mut SceneMouse,
        keys: &mut KeysState,
    ) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let app_started = self
                .state
                .action_flags
                .contains(TestbedActionFlags::APP_STARTED);

            if app_started {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::APP_STARTED, false);
                if let Some(saved_state) = std::fs::read(Self::save_file_path())
                    .ok()
                    .and_then(|data| serde_json::from_slice::<SerializableTestbedState>(&data).ok())
                {
                    self.state.apply_saved_data(saved_state, camera);
                    self.state.camera_locked = true;
                }
            }
        }

        let backend_changed = self
            .state
            .action_flags
            .contains(TestbedActionFlags::BACKEND_CHANGED);
        if backend_changed {
            self.state
                .action_flags
                .set(TestbedActionFlags::BACKEND_CHANGED, false);
            self.state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
            self.state.camera_locked = true;
        }

        let restarted = self
            .state
            .action_flags
            .contains(TestbedActionFlags::RESTART);
        if restarted {
            self.state
                .action_flags
                .set(TestbedActionFlags::RESTART, false);
            self.state.camera_locked = true;
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

            if !self.state.camera_locked {
                *camera = Camera::default();
            }

            if !restarted {
                self.state.example_settings.clear();
            }

            // Clamp selected_display_index to valid range
            let max_index = self.state.examples.len().saturating_sub(1);
            self.state.selected_display_index = self.state.selected_display_index.min(max_index);

            if !self.builders.is_empty() {
                let builder_index = self.state.selected_builder_index();
                let builder = self.builders[builder_index].builder;
                let testbed_gfx = TestbedGraphics {
                    graphics: &mut self.graphics,
                    window,
                    camera,
                    mouse: scene_mouse,
                    keys,
                    settings: None,
                };

                let mut testbed = Testbed {
                    graphics: Some(testbed_gfx),
                    state: &mut self.state,
                    harness: &mut self.harness,
                    #[cfg(feature = "other-backends")]
                    other_backends: &mut self.other_backends,
                    plugins: &mut self.plugins,
                };
                builder(&mut testbed);
            }

            self.state.camera_locked = false;
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
                self.graphics.add_body_colliders(
                    window,
                    handle,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                );
            }

            for (handle, co) in self.harness.physics.colliders.iter() {
                if co.parent().is_none() {
                    self.graphics
                        .add_collider(window, handle, &self.harness.physics.colliders);
                }
            }
        }

        if self
            .state
            .action_flags
            .contains(TestbedActionFlags::TAKE_SNAPSHOT)
        {
            self.state
                .action_flags
                .set(TestbedActionFlags::TAKE_SNAPSHOT, false);
            self.state.snapshot = Some(self.harness.physics.snapshot());
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
                self.harness.physics.restore_snapshot(snapshot.clone());
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
            }
        }

        if example_changed
            || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
                != self.state.flags.contains(TestbedStateFlags::WIREFRAME)
        {
            self.graphics.toggle_wireframe_mode(
                &self.harness.physics.colliders,
                self.state.flags.contains(TestbedStateFlags::WIREFRAME),
            );
        }
    }

    fn handle_sleep_settings(&mut self) {
        if self.state.prev_flags.contains(TestbedStateFlags::SLEEP)
            != self.state.flags.contains(TestbedStateFlags::SLEEP)
        {
            if self.state.flags.contains(TestbedStateFlags::SLEEP) {
                for (_, body) in self.harness.physics.bodies.iter_mut() {
                    body.activation_mut().normalized_linear_threshold =
                        RigidBodyActivation::default_normalized_linear_threshold();
                    body.activation_mut().angular_threshold =
                        RigidBodyActivation::default_angular_threshold();
                }
            } else {
                for (_, body) in self.harness.physics.bodies.iter_mut() {
                    body.wake_up(true);
                    body.activation_mut().normalized_linear_threshold = -1.0;
                }
            }
        }
    }

    fn clear(&mut self, window: &mut Window) {
        self.state.can_grab_behind_ground = false;
        self.graphics.clear();

        for mut plugin in self.plugins.0.drain(..) {
            plugin.clear_graphics(&mut self.graphics, window);
        }
    }
}
