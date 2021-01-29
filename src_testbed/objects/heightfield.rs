use kiss3d::window::Window;
use na::{self, Point3};
use parry::shape;
use rapier::geometry::{ColliderHandle, ColliderSet};
#[cfg(feature = "dim2")]
use rapier::math::Point;
#[cfg(feature = "dim3")]
use {
    crate::objects::node::{self, GraphicsNode},
    kiss3d::resource::Mesh,
    rapier::math::Vector,
    std::cell::RefCell,
};

pub struct HeightField {
    color: Point3<f32>,
    base_color: Point3<f32>,
    #[cfg(feature = "dim2")]
    vertices: Vec<Point<f32>>,
    #[cfg(feature = "dim3")]
    gfx: GraphicsNode,
    collider: ColliderHandle,
}

impl HeightField {
    #[cfg(feature = "dim2")]
    pub fn new(
        collider: ColliderHandle,
        heightfield: &shape::HeightField,
        color: Point3<f32>,
        _: &mut Window,
    ) -> HeightField {
        let mut vertices = Vec::new();

        for seg in heightfield.segments() {
            vertices.push(seg.a);
            vertices.push(seg.b);
        }

        HeightField {
            color,
            base_color: color,
            vertices,
            collider,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn new(
        collider: ColliderHandle,
        heightfield: &shape::HeightField,
        color: Point3<f32>,
        window: &mut Window,
    ) -> HeightField {
        use std::rc::Rc;

        let (vertices, indices) = heightfield.to_trimesh();
        let indices = indices
            .into_iter()
            .map(|idx| Point3::new(idx[0] as u16, idx[1] as u16, idx[2] as u16))
            .collect();
        let mesh = Mesh::new(vertices, indices, None, None, false);

        let mut res = HeightField {
            color,
            base_color: color,
            gfx: window.add_mesh(Rc::new(RefCell::new(mesh)), Vector::repeat(1.0)),
            collider: collider,
        };

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);

        res
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        #[cfg(feature = "dim3")]
        {
            self.gfx.set_color(color.x, color.y, color.z);
        }
        self.color = color;
        self.base_color = color;
    }

    #[cfg(feature = "dim3")]
    pub fn update(&mut self, colliders: &ColliderSet) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.collider,
            &self.color,
            &na::Isometry::identity(),
        );
    }

    #[cfg(feature = "dim2")]
    pub fn update(&mut self, _colliders: &ColliderSet) {}

    #[cfg(feature = "dim3")]
    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    #[cfg(feature = "dim3")]
    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    #[cfg(feature = "dim2")]
    pub fn draw(&mut self, window: &mut Window) {
        for vtx in self.vertices.chunks(2) {
            window.draw_planar_line(&vtx[0], &vtx[1], &self.color)
        }
    }
}
