// NOTE: this is mostly taken from the `iMplode-nZ/bevy-orbit-controls` projects but
//       with some modifications like Panning, and 2D support.
//       Most of these modifications have been contributed upstream.

use bevy::input::mouse::MouseMotion;
use bevy::input::mouse::MouseScrollUnit::{Line, Pixel};
use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;
use bevy::render::camera::Camera;
use std::ops::RangeInclusive;

const LINE_TO_PIXEL_RATIO: f32 = 0.1;

#[derive(Component)]
pub struct OrbitCamera {
    pub x: f32,
    pub y: f32,
    pub pitch_range: RangeInclusive<f32>,
    pub distance: f32,
    pub center: Vec3,
    pub rotate_sensitivity: f32,
    pub pan_sensitivity: f32,
    pub zoom_sensitivity: f32,
    pub rotate_button: MouseButton,
    pub pan_button: MouseButton,
    pub enabled: bool,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        OrbitCamera {
            x: 0.0,
            y: std::f32::consts::FRAC_PI_2,
            pitch_range: 0.01..=3.13,
            distance: 5.0,
            center: Vec3::ZERO,
            rotate_sensitivity: 1.0,
            pan_sensitivity: 1.0,
            zoom_sensitivity: 0.8,
            rotate_button: MouseButton::Left,
            pan_button: MouseButton::Right,
            enabled: true,
        }
    }
}

pub struct OrbitCameraPlugin;
impl OrbitCameraPlugin {
    #[allow(clippy::type_complexity)]
    fn update_transform_system(
        mut query: Query<(&OrbitCamera, &mut Transform), (Changed<OrbitCamera>, With<Camera>)>,
    ) {
        for (camera, mut transform) in query.iter_mut() {
            let rot = Quat::from_axis_angle(Vec3::Y, camera.x)
                * Quat::from_axis_angle(-Vec3::X, camera.y);
            transform.translation = (rot * Vec3::Y) * camera.distance + camera.center;
            transform.look_at(camera.center, Vec3::Y);
        }
    }

    fn mouse_motion_system(
        time: Res<Time>,
        mut mouse_motion_events: EventReader<MouseMotion>,
        mouse_button_input: Res<ButtonInput<MouseButton>>,
        mut query: Query<(&mut OrbitCamera, &mut Transform, &mut Camera)>,
    ) {
        let mut delta = Vec2::ZERO;
        for event in mouse_motion_events.read() {
            delta += event.delta;
        }
        for (mut camera, transform, _) in query.iter_mut() {
            if !camera.enabled {
                continue;
            }

            if mouse_button_input.pressed(camera.rotate_button) {
                camera.x -= delta.x * camera.rotate_sensitivity * time.delta_secs();
                camera.y -= delta.y * camera.rotate_sensitivity * time.delta_secs();
                camera.y = camera
                    .y
                    .max(*camera.pitch_range.start())
                    .min(*camera.pitch_range.end());
            }

            if mouse_button_input.pressed(camera.pan_button) {
                let right_dir = transform.rotation * -Vec3::X;
                let up_dir = transform.rotation * Vec3::Y;
                let pan_vector = (delta.x * right_dir + delta.y * up_dir)
                    * camera.pan_sensitivity
                    * time.delta_secs();
                camera.center += pan_vector;
            }
        }
    }

    fn zoom_system(
        mut mouse_wheel_events: EventReader<MouseWheel>,
        mut query: Query<&mut OrbitCamera, With<Camera>>,
    ) {
        let mut total = 0.0;
        for event in mouse_wheel_events.read() {
            total += event.y
                * match event.unit {
                    Line => 1.0,
                    Pixel => LINE_TO_PIXEL_RATIO,
                };
        }
        for mut camera in query.iter_mut() {
            if camera.enabled {
                camera.distance *= camera.zoom_sensitivity.powf(total);
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
