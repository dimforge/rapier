use kiss3d::window::Window;
use na::{Isometry2, Point2, Point3};
use ncollide2d::shape;
use nphysics2d::object::{ColliderAnchor, DefaultColliderHandle, DefaultColliderSet};

pub struct Polyline {
    color: Point3<f32>,
    base_color: Point3<f32>,
    vertices: Vec<Point2<f32>>,
    indices: Vec<Point2<usize>>,
    collider: DefaultColliderHandle,
    pos: Isometry2<f32>,
}

impl Polyline {
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        _: Isometry2<f32>,
        vertices: Vec<Point2<f32>>,
        indices: Vec<Point2<usize>>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> Polyline {
        let mut res = Polyline {
            color,
            pos: Isometry2::identity(),
            base_color: color,
            vertices,
            indices,
            collider,
        };

        res.update(colliders);
        res
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

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        // Update if some deformation occurred.
        // FIXME: don't update if it did not move.
        if let Some(c) = colliders.get(self.collider) {
            self.pos = *c.position();
            if let ColliderAnchor::OnDeformableBody { .. } = c.anchor() {
                let shape = c.shape().as_shape::<shape::Polyline<f32>>().unwrap();
                self.vertices = shape.points().to_vec();
                self.indices.clear();

                for e in shape.edges() {
                    self.indices.push(e.indices);
                }
            }
        }
    }

    pub fn object(&self) -> DefaultColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for idx in &self.indices {
            let p1 = self.pos * self.vertices[idx.x];
            let p2 = self.pos * self.vertices[idx.y];
            window.draw_planar_line(&p1, &p2, &self.color)
        }
    }
}
