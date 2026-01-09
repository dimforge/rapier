use glamx::Vec2;

#[derive(Default, Copy, Clone, Debug)]
pub struct SceneMouse {
    #[cfg(feature = "dim2")]
    pub point: Option<Vec2>,
    #[cfg(feature = "dim3")]
    pub ray: Option<(glamx::Vec3, glamx::Vec3)>,
}

impl SceneMouse {
    pub fn new() -> Self {
        Self::default()
    }

    #[cfg(feature = "dim2")]
    pub fn update_from_window(
        &mut self,
        cursor_pos: Option<(f64, f64)>,
        window_size: (u32, u32),
        camera: &kiss3d::camera::PanZoomCamera2d,
    ) {
        use kiss3d::camera::Camera2d;

        if let Some((x, y)) = cursor_pos {
            // Convert cursor position to world coordinates using the camera
            let cursor = Vec2::new(x as f32, y as f32);
            let window_size = Vec2::new(window_size.0 as f32, window_size.1 as f32);
            let world_pos = camera.unproject(cursor, window_size);
            self.point = Some(world_pos);
        } else {
            self.point = None;
        }
    }

    #[cfg(feature = "dim3")]
    pub fn update_from_window(
        &mut self,
        cursor_pos: Option<(f64, f64)>,
        window_size: (u32, u32),
        camera: &crate::Camera,
    ) {
        use kiss3d::camera::Camera3d;

        if let Some((x, y)) = cursor_pos {
            // Convert cursor position to world coordinates using the camera
            let cursor = Vec2::new(x as f32, y as f32);
            let window_size = Vec2::new(window_size.0 as f32, window_size.1 as f32);
            self.ray = Some(camera.unproject(cursor, window_size));
        } else {
            self.ray = None;
        }
    }
}
