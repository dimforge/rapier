use crate::math::Point;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

#[derive(Component)]
pub struct MainCamera;

#[derive(Default, Copy, Clone, Debug, Resource)]
pub struct SceneMouse {
    #[cfg(feature = "dim2")]
    pub point: Option<Point<f32>>,
    #[cfg(feature = "dim3")]
    pub ray: Option<(Point<f32>, crate::math::Vector<f32>)>,
}

pub fn track_mouse_state(
    mut scene_mouse: ResMut<SceneMouse>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&GlobalTransform, &Camera), With<MainCamera>>,
) {
    if let Ok(window) = windows.get_single() {
        for (camera_transform, camera) in camera.iter() {
            if let Some(cursor) = window.cursor_position() {
                let ndc_cursor = ((cursor / Vec2::new(window.width(), window.height()) * 2.0)
                    - Vec2::ONE)
                    * Vec2::new(1.0, -1.0);
                let ndc_to_world =
                    camera_transform.compute_matrix() * camera.projection_matrix().inverse();
                let ray_pt1 =
                    ndc_to_world.project_point3(Vec3::new(ndc_cursor.x, ndc_cursor.y, -1.0));

                #[cfg(feature = "dim2")]
                {
                    scene_mouse.point = Some(Point::new(ray_pt1.x, ray_pt1.y));
                }
                #[cfg(feature = "dim3")]
                {
                    let ray_pt2 =
                        ndc_to_world.project_point3(Vec3::new(ndc_cursor.x, ndc_cursor.y, 1.0));
                    let ray_dir = ray_pt2 - ray_pt1;
                    scene_mouse.ray = Some((na::Vector3::from(ray_pt1).into(), ray_dir.into()));
                }
            }
        }
    }
}
