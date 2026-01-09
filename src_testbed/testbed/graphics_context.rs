//! Graphics context for a frame.

use kiss3d::color::Color;
use kiss3d::window::Window;
use rapier::dynamics::RigidBodyHandle;
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderHandle, ColliderSet};

use crate::Camera;
use crate::graphics::GraphicsManager;
use crate::mouse::SceneMouse;
use crate::settings::ExampleSettings;

use super::keys::KeysState;

/// Context for graphics operations during a frame
pub struct TestbedGraphics<'a> {
    pub graphics: &'a mut GraphicsManager,
    pub window: &'a mut Window,
    pub camera: &'a mut Camera,
    pub mouse: &'a SceneMouse,
    pub keys: &'a KeysState,
    pub settings: Option<&'a mut ExampleSettings>,
}

impl<'a> TestbedGraphics<'a> {
    pub fn set_body_color(&mut self, body: RigidBodyHandle, color: Color, tmp_color: bool) {
        self.graphics.set_body_color(body, color, tmp_color);
    }

    pub fn add_body(
        &mut self,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        self.graphics
            .add_body_colliders(self.window, handle, bodies, colliders);
    }

    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.graphics.remove_body_nodes(handle);
    }

    pub fn add_collider(&mut self, handle: ColliderHandle, colliders: &ColliderSet) {
        self.graphics.add_collider(self.window, handle, colliders);
    }

    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.graphics.remove_collider_nodes(handle);
    }

    pub fn keys(&self) -> &KeysState {
        self.keys
    }

    pub fn mouse(&self) -> &SceneMouse {
        self.mouse
    }

    /// Update collider graphics after shape modification
    pub fn update_collider(&mut self, handle: ColliderHandle, colliders: &ColliderSet) {
        // Remove and re-add the collider to update its graphics
        self.graphics.remove_collider_nodes(handle);
        self.graphics.add_collider(self.window, handle, colliders);
    }

    /// Get the camera rotation as a unit quaternion (3D only)
    #[cfg(feature = "dim3")]
    pub fn camera_rotation(&self) -> na::UnitQuaternion<f32> {
        // Calculate rotation from orbit camera angles
        let rot_x = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), self.camera.at().x);
        let rot_y =
            na::UnitQuaternion::from_axis_angle(&(-na::Vector3::x_axis()), self.camera.at().y);
        rot_x * rot_y
    }

    /// Get the camera forward direction (3D only)
    #[cfg(feature = "dim3")]
    pub fn camera_fwd_dir(&self) -> na::Vector3<f32> {
        let rot = self.camera_rotation();
        rot * na::Vector3::z()
    }

    /// Get mutable access to the egui context for custom UI
    pub fn egui_context(&self) -> &egui::Context {
        self.window.egui_context()
    }

    /// Get mutable access to the egui context for custom UI
    pub fn egui_context_mut(&mut self) -> &mut egui::Context {
        self.window.egui_context_mut()
    }
}
