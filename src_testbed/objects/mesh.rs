use crate::objects::node::{self, GraphicsNode};
use kiss3d::window;
use na::Point3;
use rapier::geometry::{ColliderHandle, ColliderSet};
use rapier::math::{Isometry, Point};
use std::cell::RefCell;
use std::rc::Rc;

pub struct Mesh {
    color: Point3<f32>,
    base_color: Point3<f32>,
    gfx: GraphicsNode,
    collider: ColliderHandle,
}

impl Mesh {
    pub fn new(
        collider: ColliderHandle,
        vertices: Vec<Point<f32>>,
        indices: Vec<[u32; 3]>,
        color: Point3<f32>,
        window: &mut window::Window,
    ) -> Mesh {
        let vs = vertices;
        let is = indices
            .into_iter()
            .map(|idx| Point3::new(idx[0] as u16, idx[1] as u16, idx[2] as u16))
            .collect();

        let mesh;
        let gfx;

        #[cfg(feature = "dim2")]
        {
            mesh = kiss3d::resource::PlanarMesh::new(vs, is, None, false);
            gfx = window.add_planar_mesh(
                Rc::new(RefCell::new(mesh)),
                crate::math::Vector::from_element(1.0),
            );
        }

        #[cfg(feature = "dim3")]
        {
            mesh = kiss3d::resource::Mesh::new(vs, is, None, None, false);
            gfx = window.add_mesh(Rc::new(RefCell::new(mesh)), na::Vector3::from_element(1.0));
        }

        let mut res = Mesh {
            color,
            base_color: color,
            gfx,
            collider,
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
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, colliders: &ColliderSet) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.collider,
            &self.color,
            &Isometry::identity(),
        );

        // // Update if some deformation occurred.
        // // FIXME: don't update if it did not move.
        // if let Some(c) = colliders.get(self.collider) {
        //     if let ColliderAnchor::OnDeformableBody { .. } = c.anchor() {
        //         let shape = c.shape().as_shape::<TriMesh<f32>>().unwrap();
        //         let vtx = shape.points();
        //
        //         self.gfx.modify_vertices(&mut |vertices| {
        //             for (v, new_v) in vertices.iter_mut().zip(vtx.iter()) {
        //                 *v = *new_v
        //             }
        //         });
        //         self.gfx.recompute_normals();
        //     }
        // }
    }

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }
}
