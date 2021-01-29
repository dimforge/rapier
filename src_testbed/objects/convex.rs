#![allow(warnings)] // TODO: remove this.

#[cfg(feature = "dim2")]
use crate::math::Vector;
use crate::math::{Isometry, Point};
use crate::objects::node::{self, GraphicsNode};
use kiss3d::window::Window;
use na::Point3;
use rapier::geometry::{ColliderHandle, ColliderSet};

pub struct Convex {
    color: Point3<f32>,
    base_color: Point3<f32>,
    gfx: GraphicsNode,
    body: ColliderHandle,
    delta: Isometry<f32>,
}

impl Convex {
    pub fn new(
        body: ColliderHandle,
        delta: Isometry<f32>,
        vertices: Vec<Point<f32>>,
        #[cfg(feature = "dim3")] indices: Vec<[u32; 3]>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Convex {
        #[cfg(feature = "dim2")]
        let node = window.add_convex_polygon(vertices, Vector::from_element(1.0));
        #[cfg(feature = "dim3")]
        let node = {
            use std::cell::RefCell;
            use std::rc::Rc;
            let mut mesh_vertices = Vec::new();
            let mut mesh_indices = Vec::new();
            for idx in indices {
                let i = mesh_vertices.len() as u16;
                mesh_vertices.push(vertices[idx[0] as usize]);
                mesh_vertices.push(vertices[idx[1] as usize]);
                mesh_vertices.push(vertices[idx[2] as usize]);
                mesh_indices.push(Point3::new(i, i + 1, i + 2));
            }

            let mesh = kiss3d::resource::Mesh::new(mesh_vertices, mesh_indices, None, None, false);
            window.add_mesh(Rc::new(RefCell::new(mesh)), na::Vector3::from_element(1.0))
        };

        let mut res = Convex {
            color,
            base_color: color,
            gfx: node,
            body,
            delta,
        };

        // res.gfx.set_texture_from_file(&Path::new("media/kitten.png"), "kitten");
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
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, colliders: &ColliderSet) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.body,
            &self.color,
            &self.delta,
        );
    }

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderHandle {
        self.body
    }
}
