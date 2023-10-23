// NOTE: this is inspired from the `bevy-orbit-controls` projects but
//       with some modifications like Panning, and 2D support.
//       Most of these modifications have been contributed upstream.
use bevy::input::mouse::MouseMotion;
use bevy::input::mouse::MouseScrollUnit::{Line, Pixel};
use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;
use bevy::render::camera::Camera;

const LINE_TO_PIXEL_RATIO: f32 = 0.1;

#[derive(Component)]
pub struct OrbitCamera {
    pub zoom: f32,
    pub center: Vec3,
    pub pan_sensitivity: f32,
    pub zoom_sensitivity: f32,
    pub pan_button: MouseButton,
    pub enabled: bool,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        OrbitCamera {
            zoom: 100.0,
            center: Vec3::ZERO,
            pan_sensitivity: 1.0,
            zoom_sensitivity: 0.8,
            pan_button: MouseButton::Right,
            enabled: true,
        }
    }
}

// Adapted from the 3D orbit camera from bevy-orbit-controls
pub struct OrbitCameraPlugin;
impl OrbitCameraPlugin {
    fn update_transform_system(
        mut query: Query<(&OrbitCamera, &mut Transform), (Changed<OrbitCamera>, With<Camera>)>,
    ) {
        for (camera, mut transform) in query.iter_mut() {
            transform.translation = camera.center;
            transform.scale = Vec3::new(1.0 / camera.zoom, 1.0 / camera.zoom, 1.0);
        }
    }

    fn mouse_motion_system(
        _time: Res<Time>,
        mut mouse_motion_events: EventReader<MouseMotion>,
        mouse_button_input: Res<Input<MouseButton>>,
        mut query: Query<(&mut OrbitCamera, &mut Transform, &mut Camera)>,
    ) {
        let mut delta = Vec2::ZERO;
        for event in mouse_motion_events.iter() {
            delta += event.delta;
        }
        for (mut camera, _, _) in query.iter_mut() {
            if !camera.enabled {
                continue;
            }

            if mouse_button_input.pressed(camera.pan_button) {
                let delta = delta * camera.pan_sensitivity;
                camera.center += Vec3::new(-delta.x, delta.y, 0.0);
            }
        }
    }

    fn zoom_system(
        mut mouse_wheel_events: EventReader<MouseWheel>,
        mut query: Query<&mut OrbitCamera, With<Camera>>,
    ) {
        let mut total = 0.0;
        for event in mouse_wheel_events.iter() {
            total -= event.y
                * match event.unit {
                    Line => 1.0,
                    Pixel => LINE_TO_PIXEL_RATIO,
                };
        }
        for mut camera in query.iter_mut() {
            if camera.enabled {
                camera.zoom *= camera.zoom_sensitivity.powf(total);
            }
        }
    }
}
impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, Self::mouse_motion_system)
            .add_systems(Update, Self::zoom_system)
            .add_systems(Update, Self::update_transform_system);
    }
}
