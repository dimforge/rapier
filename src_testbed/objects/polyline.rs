use kiss3d::window::Window;
use na::Point3;
use rapier::geometry::{ColliderHandle, ColliderSet};
use rapier::math::{Isometry, Point};

pub struct Polyline {
    color: Point3<f32>,
    base_color: Point3<f32>,
    vertices: Vec<Point<f32>>,
    indices: Vec<[u32; 2]>,
    collider: ColliderHandle,
    pos: Isometry<f32>,
}

impl Polyline {
    pub fn new(
        collider: ColliderHandle,
        vertices: Vec<Point<f32>>,
        indices: Vec<[u32; 2]>,
        color: Point3<f32>,
    ) -> Polyline {
        Polyline {
            color,
            pos: Isometry::identity(),
            base_color: color,
            vertices,
            indices,
            collider,
        }
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, colliders: &ColliderSet) {
        self.pos = colliders
            .get(self.collider)
            .map(|c| *c.position())
            .unwrap_or(Isometry::identity());
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for idx in &self.indices {
            let p1 = self.pos * self.vertices[idx[0] as usize];
            let p2 = self.pos * self.vertices[idx[1] as usize];

            #[cfg(feature = "dim2")]
            window.draw_planar_line(&p1, &p2, &self.color);
            #[cfg(feature = "dim3")]
            window.draw_line(&p1, &p2, &self.color);
        }
    }
}
