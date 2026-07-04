//! TestbedViewer — the example-owned-loop testbed.
//!
//! The viewer owns the window, graphics, cameras, input and UI. The *example*
//! owns the [`PhysicsWorld`] and drives the loop:
//!
//! ```ignore
//! pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
//!     let mut world = PhysicsWorld::new();
//!     /* build scene */
//!     viewer.set_world(&mut world);
//!     viewer.look_at(eye, at);
//!     while viewer.render_frame(&mut world).await {
//!         if viewer.simulating() {
//!             world.step();
//!         }
//!     }
//!     Ok(())
//! }
//! ```
//!
//! The example calls `world.step()` (or `world.step_with_events(&hooks, &events)`
//! with its own [`ChannelEventCollector`]) directly — the viewer is not involved
//! in stepping or event collection.

#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

use kiss3d::color::Color;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::window::Window;
use rapier::dynamics::{RigidBodyActivation, RigidBodyHandle};
use rapier::geometry::{ColliderHandle, SharedShape};
use rapier::pipeline::PhysicsWorld;

#[cfg(feature = "dim3")]
use glamx::Vec3;

use crate::Camera;
use crate::debug_render::{DebugRenderPipelineResource, debug_render_scene};
use crate::graphics::{GraphicsManager, RenderMaterial};
use crate::mouse::SceneMouse;
use crate::physics::{restore_world, snapshot_world};
use crate::settings::ExampleSettings;
use crate::testbed::hover::highlight_hovered_body;
use crate::testbed::keys::KeysState;
use crate::testbed::state::{
    ExampleEntry, RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags, Transition,
};
use crate::ui;

/// The example-owned-loop testbed viewer.
///
/// The viewer renders the world and drives the UI, but does **not** step the
/// simulation or collect events — the example does that itself via
/// [`PhysicsWorld::step`] / [`PhysicsWorld::step_with_events`].
pub struct TestbedViewer {
    window: Window,
    graphics: GraphicsManager,
    camera: Camera,
    scene_mouse: SceneMouse,
    keys: KeysState,
    debug_render: DebugRenderPipelineResource,
    state: TestbedState,
}

fn save_file_path() -> String {
    format!("testbed_state_{}.autosave.json", env!("CARGO_CRATE_NAME"))
}

impl TestbedViewer {
    /// Creates the viewer window and registers the list of examples shown in the
    /// UI (used by the outer demo runner to dispatch the selected example).
    pub async fn new(examples: Vec<ExampleEntry>) -> Self {
        // Install a default logger so warnings emitted by the examples (e.g. the
        // MJCF loader complaining about missing meshes) actually print.
        let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
            .try_init();

        #[cfg(feature = "profiler_ui")]
        profiling::puffin::set_scopes_on(true);

        let title = if cfg!(feature = "dim2") {
            "Rapier: 2D demos"
        } else {
            "Rapier: 3D demos"
        };

        let mut window = Window::new_with_size(title, 1280, 720).await;
        window.set_background_color(Color::new(245.0 / 255.0, 245.0 / 255.0, 236.0 / 255.0, 1.0));
        window.set_ambient(0.1);

        let mut camera = Camera::default();

        let mut state = TestbedState::default();
        state.set_examples(examples);

        // Restore the autosaved UI state + camera (selected example, flags, ...).
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(saved) = std::fs::read(save_file_path())
                .ok()
                .and_then(|data| serde_json::from_slice(&data).ok())
            {
                state.apply_saved_data(saved, &mut camera);
                // Keep the restored camera: the first example's `look_at` must
                // not override it.
                state.camera_locked = true;
            }
        }
        state.action_flags.remove(TestbedActionFlags::APP_STARTED);

        Self {
            window,
            graphics: GraphicsManager::new(),
            camera,
            scene_mouse: SceneMouse::new(),
            keys: KeysState::default(),
            debug_render: DebugRenderPipelineResource::default(),
            state,
        }
    }

    // ───────────────────────────── loop driving ─────────────────────────────

    /// Renders one frame (presents the scene built last frame, processes input
    /// and UI). Returns `false` when the example's loop should end — either the
    /// window was closed or a [`Transition`] (example switch / quit) was
    /// requested from the UI.
    pub async fn render_frame(&mut self, world: &mut PhysicsWorld) -> bool {
        profiling::finish_frame!();

        #[cfg(feature = "dim3")]
        let keep_open = self
            .window
            .render_3d(self.graphics.scene_mut(), &mut self.camera)
            .await;
        #[cfg(feature = "dim2")]
        let keep_open = self
            .window
            .render_2d(self.graphics.scene_mut(), &mut self.camera)
            .await;

        if !keep_open {
            self.state.transition = Some(Transition::Quit);
            return false;
        }

        self.handle_events();

        let cursor_pos = self.window.cursor_pos();
        self.scene_mouse
            .update_from_window(cursor_pos, self.window.size().into(), &self.camera);

        self.handle_action_flags(world);
        self.handle_sleep_settings(world);
        self.autosave();

        highlight_hovered_body(&mut self.graphics, &self.scene_mouse, world);
        self.graphics
            .draw(self.state.flags, &world.bodies, &world.colliders);
        debug_render_scene(&mut self.window, &mut self.debug_render, world);

        // Disjoint field borrows: the closure captures `state`/`debug` while
        // `self.window` is the receiver.
        let state = &mut self.state;
        let debug = &mut self.debug_render;
        self.window
            .draw_ui(|ctx| ui::update_ui(ctx, state, world, debug));

        self.state.prev_flags = self.state.flags;
        self.state.transition.is_none()
    }

    /// Whether the simulation should advance this frame, honoring run/pause/step.
    /// A pending single-step is consumed (returns `true` once, then pauses).
    pub fn simulating(&mut self) -> bool {
        match self.state.running {
            RunMode::Stop => false,
            RunMode::Running => true,
            RunMode::Step => {
                self.state.running = RunMode::Stop;
                true
            }
        }
    }

    // ───────────────────────── scene registration ───────────────────────────

    /// Registers render nodes for the world the example just built, configures
    /// the selected broad-phase, and enables the profiling counters.
    ///
    /// Render-node creation is deferred to the next frame so example code can
    /// still set initial body/collider colors after calling this.
    pub fn set_world(&mut self, world: &mut PhysicsWorld) {
        world.broad_phase = self.state.broad_phase_type.init_broad_phase();
        world.physics_pipeline.counters.enable();
        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
    }

    /// Clears the scene and resets per-example viewer state. Called by the outer
    /// demo runner between examples. A switch caused by a restart / backend /
    /// solver-parameter change preserves the camera and the user's setting
    /// edits; selecting a different example resets both.
    pub fn clear_scene(&mut self) {
        self.graphics.clear();
        self.state.transition = None;
        self.state.snapshot = None;
        self.state.action_flags = TestbedActionFlags::empty();

        if self.state.preserve_settings_on_switch {
            self.state.camera_locked = true;
        } else {
            self.camera = Camera::default();
            self.state.camera_locked = false;
            self.state.example_settings.clear();
        }
        self.state.preserve_settings_on_switch = false;
    }

    // ──────────────────────────── camera / scene ────────────────────────────

    pub fn allow_grabbing_behind_ground(&mut self, allow: bool) {
        self.state.can_grab_behind_ground = allow;
    }

    pub fn set_graphics_shift(&mut self, shift: rapier::math::Vector) {
        if !self.state.camera_locked {
            self.graphics.gfx_shift = shift;
        }
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: glamx::Vec2, zoom: f32) {
        if !self.state.camera_locked {
            self.camera.set_at(at);
            self.camera.set_zoom(zoom);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Vec3, at: Vec3) {
        if !self.state.camera_locked {
            self.camera.look_at(eye, at);
        }
    }

    /// Sets the world-up direction the orbit camera uses (call before
    /// [`Self::look_at`]). Keeps the gravity slider aligned with "down".
    #[cfg(feature = "dim3")]
    pub fn set_up_axis(&mut self, up_axis: Vec3) {
        #[allow(clippy::useless_conversion)]
        {
            self.state.up_axis = up_axis.normalize().into();
        }
        if !self.state.camera_locked {
            self.camera.set_up_axis(up_axis);
        }
    }

    /// Camera orientation as a unit quaternion (3D only).
    #[cfg(feature = "dim3")]
    pub fn camera_rotation(&self) -> na::UnitQuaternion<f32> {
        let rot_x = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), self.camera.at().x);
        let rot_y =
            na::UnitQuaternion::from_axis_angle(&(-na::Vector3::x_axis()), self.camera.at().y);
        rot_x * rot_y
    }

    /// Camera forward direction (3D only).
    #[cfg(feature = "dim3")]
    pub fn camera_fwd_dir(&self) -> na::Vector3<f32> {
        self.camera_rotation() * na::Vector3::z()
    }

    /// Recenters the camera so the whole scene fills the viewport (deferred to
    /// the next frame, when collider AABBs are available).
    pub fn request_frame_all(&mut self) {
        self.state
            .action_flags
            .set(TestbedActionFlags::FRAME_SCENE, true);
    }

    // ─────────────────────────── render helpers ─────────────────────────────

    pub fn set_initial_body_color(&mut self, body: RigidBodyHandle, color: Color) {
        self.graphics.set_initial_body_color(body, color);
    }

    pub fn set_initial_collider_color(&mut self, collider: ColliderHandle, color: Color) {
        self.graphics.set_initial_collider_color(collider, color);
    }

    pub fn set_body_wireframe(&mut self, body: RigidBodyHandle, wireframe_enabled: bool) {
        self.graphics.set_body_wireframe(body, wireframe_enabled);
    }

    /// Attaches a render-only mesh to `body` (does not participate in physics).
    #[allow(clippy::too_many_arguments)]
    pub fn add_body_render_mesh(
        &mut self,
        body: RigidBodyHandle,
        shape: &SharedShape,
        local_pose: rapier::math::Pose,
        color: Color,
        uvs: Option<&[[f32; 2]]>,
        normals: Option<&[[f32; 3]]>,
        texture: Option<&std::path::Path>,
        material: Option<RenderMaterial>,
    ) {
        self.graphics.add_body_render_mesh(
            &mut self.window,
            body,
            shape,
            local_pose,
            color,
            uvs,
            normals,
            texture,
            material,
        );
    }

    pub fn set_colliders_visible(&mut self, visible: bool) {
        self.graphics.set_colliders_visible(visible);
    }

    pub fn set_body_render_meshes_visible(&mut self, visible: bool) {
        self.graphics.set_body_render_meshes_visible(visible);
    }

    // ──────────────────────── runtime graphics (in-loop) ─────────────────────
    // Examples that add/remove bodies or change colors while the simulation runs
    // (formerly inside an `add_callback` closure) call these from their loop.

    /// Sets a body's render color. With `tmp_color`, the color is transient (it is
    /// reset to the body's base color next frame — used for hover/transient cues);
    /// otherwise it becomes the body's persistent color.
    pub fn set_body_color(&mut self, body: RigidBodyHandle, color: Color, tmp_color: bool) {
        self.graphics.set_body_color(body, color, tmp_color);
    }

    /// Creates render nodes for a body inserted into `world` after [`Self::set_world`].
    pub fn add_body(&mut self, handle: RigidBodyHandle, world: &PhysicsWorld) {
        self.graphics
            .add_body_colliders(&mut self.window, handle, &world.bodies, &world.colliders);
    }

    /// Removes the render nodes of a body removed from the world.
    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.graphics.remove_body_nodes(handle);
    }

    /// Creates render nodes for a parentless collider inserted after [`Self::set_world`].
    pub fn add_collider(&mut self, handle: ColliderHandle, world: &PhysicsWorld) {
        self.graphics
            .add_collider(&mut self.window, handle, &world.colliders);
    }

    /// Removes the render nodes of a collider removed from the world.
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.graphics.remove_collider_nodes(handle);
    }

    /// Refreshes a collider's render nodes after its shape was modified in place.
    pub fn update_collider(&mut self, handle: ColliderHandle, world: &PhysicsWorld) {
        self.graphics.remove_collider_nodes(handle);
        self.graphics
            .add_collider(&mut self.window, handle, &world.colliders);
    }

    // ───────────────────────────── input / events ───────────────────────────

    pub fn keys(&self) -> &KeysState {
        &self.keys
    }

    pub fn mouse(&self) -> &SceneMouse {
        &self.scene_mouse
    }

    pub fn egui_context(&self) -> &egui::Context {
        self.window.egui_context()
    }

    pub fn egui_context_mut(&mut self) -> &mut egui::Context {
        self.window.egui_context_mut()
    }

    // ───────────────────────────── registry / loop ──────────────────────────

    /// Display index of the example the UI currently has selected, clamped to a
    /// valid range (a stale autosave may carry an index past the current list).
    pub fn selected(&self) -> usize {
        self.state
            .selected_display_index
            .min(self.state.examples.len().saturating_sub(1))
    }

    /// Name of the example the UI currently has selected (for dispatch).
    pub fn selected_name(&self) -> Option<&'static str> {
        self.state
            .examples
            .get(self.state.selected_display_index)
            .map(|e| e.name)
    }

    /// Whether the user requested the whole testbed to quit.
    pub fn quitting(&self) -> bool {
        matches!(self.state.transition, Some(Transition::Quit))
    }

    /// Read-only access to the underlying example settings (for examples that
    /// read live-tunable values declared via [`Self::example_settings_mut`]).
    pub fn example_settings(&self) -> &ExampleSettings {
        &self.state.example_settings
    }

    pub fn example_settings_mut(&mut self) -> &mut ExampleSettings {
        &mut self.state.example_settings
    }

    // ───────────────────────────── internals ────────────────────────────────

    fn handle_events(&mut self) {
        // Collect first so we can mutate `self` freely inside the loop.
        let events: Vec<WindowEvent> = self.window.events().iter().map(|e| e.value).collect();

        for value in events {
            match value {
                WindowEvent::Key(key, Action::Press, _) => {
                    if !self.keys.pressed_keys.contains(&key) {
                        self.keys.pressed_keys.push(key);
                    }
                    match key {
                        Key::LShift | Key::RShift => self.keys.shift = true,
                        Key::LControl | Key::RControl => self.keys.ctrl = true,
                        Key::LAlt | Key::RAlt => self.keys.alt = true,
                        _ => {}
                    }
                }
                WindowEvent::Key(key, Action::Release, _) => {
                    self.keys.pressed_keys.retain(|k| *k != key);
                    match key {
                        Key::T => {
                            self.state.running = if self.state.running == RunMode::Stop {
                                RunMode::Running
                            } else {
                                RunMode::Stop
                            };
                        }
                        Key::S => self.state.running = RunMode::Step,
                        Key::R => self.request_restart(),
                        Key::LShift | Key::RShift => self.keys.shift = false,
                        Key::LControl | Key::RControl => self.keys.ctrl = false,
                        Key::LAlt | Key::RAlt => self.keys.alt = false,
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }

    fn request_restart(&mut self) {
        self.state.preserve_settings_on_switch = true;
        self.state.transition = Some(Transition::Switch);
    }

    fn handle_action_flags(&mut self, world: &mut PhysicsWorld) {
        if self
            .state
            .action_flags
            .contains(TestbedActionFlags::TAKE_SNAPSHOT)
        {
            self.state
                .action_flags
                .set(TestbedActionFlags::TAKE_SNAPSHOT, false);
            self.state.snapshot = Some(snapshot_world(world, 0));
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
                restore_world(world, snapshot);
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
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
            for (handle, _) in world.bodies.iter() {
                self.graphics.add_body_colliders(
                    &mut self.window,
                    handle,
                    &world.bodies,
                    &world.colliders,
                );
            }
            for (handle, co) in world.colliders.iter() {
                if co.parent().is_none() {
                    self.graphics
                        .add_collider(&mut self.window, handle, &world.colliders);
                }
            }

            // Honor the current wireframe flag on freshly created nodes.
            if self.state.flags.contains(TestbedStateFlags::WIREFRAME) {
                self.graphics.toggle_wireframe_mode(&world.colliders, true);
            }
        }

        if self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
            != self.state.flags.contains(TestbedStateFlags::WIREFRAME)
        {
            self.graphics.toggle_wireframe_mode(
                &world.colliders,
                self.state.flags.contains(TestbedStateFlags::WIREFRAME),
            );
        }

        if self
            .state
            .action_flags
            .contains(TestbedActionFlags::FRAME_SCENE)
        {
            self.state
                .action_flags
                .set(TestbedActionFlags::FRAME_SCENE, false);
            frame_scene(
                &world.colliders,
                &world.bodies,
                &self.graphics,
                &mut self.camera,
            );
        }
    }

    fn handle_sleep_settings(&mut self, world: &mut PhysicsWorld) {
        if self.state.prev_flags.contains(TestbedStateFlags::SLEEP)
            != self.state.flags.contains(TestbedStateFlags::SLEEP)
        {
            if self.state.flags.contains(TestbedStateFlags::SLEEP) {
                for (_, body) in world.bodies.iter_mut() {
                    body.activation_mut().normalized_linear_threshold =
                        RigidBodyActivation::default_normalized_linear_threshold();
                    body.activation_mut().angular_threshold =
                        RigidBodyActivation::default_angular_threshold();
                }
            } else {
                for (_, body) in world.bodies.iter_mut() {
                    body.wake_up(true);
                    body.activation_mut().normalized_linear_threshold = -1.0;
                }
            }
        }
    }

    fn autosave(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let new_save_data = self.state.save_data(self.camera);
            if self.state.prev_save_data != new_save_data {
                let data = serde_json::to_string_pretty(&new_save_data).unwrap();
                if let Err(e) = std::fs::write(save_file_path(), &data) {
                    eprintln!("Failed to write autosave file: {e}");
                }
                self.state.prev_save_data = new_save_data;
            }
        }
    }
}

/// Recenter the camera so the AABB of all the collider and visual-mesh shapes in
/// the scene fills the viewport ("frame all" / "best view").
fn frame_scene(
    colliders: &rapier::geometry::ColliderSet,
    bodies: &rapier::dynamics::RigidBodySet,
    graphics: &GraphicsManager,
    camera: &mut Camera,
) {
    use rapier::parry::bounding_volume::BoundingVolume;
    use rapier::parry::math::Real;

    fn is_finite_aabb(a: &rapier::parry::bounding_volume::Aabb) -> bool {
        if !a.mins.x.is_finite()
            || !a.mins.y.is_finite()
            || !a.maxs.x.is_finite()
            || !a.maxs.y.is_finite()
        {
            return false;
        }
        #[cfg(feature = "dim3")]
        if !a.mins.z.is_finite() || !a.maxs.z.is_finite() {
            return false;
        }
        true
    }

    let mut aabb: Option<rapier::parry::bounding_volume::Aabb> = None;
    let mut merge = |a: rapier::parry::bounding_volume::Aabb| {
        if !is_finite_aabb(&a) {
            return;
        }
        aabb = Some(match aabb {
            None => a,
            Some(prev) => prev.merged(&a),
        });
    };
    for (_, co) in colliders.iter() {
        merge(co.compute_aabb());
    }
    for vm in graphics.body_attached_nodes() {
        let Some(rb) = bodies.get(vm.body) else {
            continue;
        };
        let world_pose = *rb.position() * vm.delta;
        merge(vm.shape.compute_aabb(&world_pose));
    }
    let Some(aabb) = aabb else {
        return;
    };

    let center = aabb.center();
    let extents = aabb.extents();

    #[cfg(feature = "dim3")]
    {
        let half = extents * 0.5 as Real;
        let radius = (half.x * half.x + half.y * half.y + half.z * half.z)
            .sqrt()
            .max(0.5);
        let fov = camera.fov();
        let dist = radius / (fov as Real * 0.5).sin().max(1e-3) * 1.1;

        let kiss_center =
            kiss3d::glamx::Vec3::new(center.x as f32, center.y as f32, center.z as f32);
        let dir = kiss3d::glamx::Vec3::new(1.0, 1.0, 1.0).normalize();
        let eye = kiss_center + dir * dist as f32;
        camera.look_at(eye, kiss_center);
    }

    #[cfg(feature = "dim2")]
    {
        let kiss_center = kiss3d::glamx::Vec2::new(center.x as f32, center.y as f32);
        let max_extent = extents.x.max(extents.y).max(0.5);
        let zoom = (600.0 as Real / max_extent / 1.2) as f32;
        camera.look_at(kiss_center, zoom);
    }
}
