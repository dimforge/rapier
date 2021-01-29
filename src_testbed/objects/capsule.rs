use crate::objects::node::{self, GraphicsNode};
use kiss3d::window;
use na::Point3;
use rapier::geometry::{self, ColliderHandle, ColliderSet};
use rapier::math::Isometry;

pub struct Capsule {
    color: Point3<f32>,
    base_color: Point3<f32>,
    gfx: GraphicsNode,
    collider: ColliderHandle,
    delta: Isometry<f32>,
}

impl Capsule {
    pub fn new(
        collider: ColliderHandle,
        delta: Isometry<f32>,
        capsule: &geometry::Capsule,
        color: Point3<f32>,
        window: &mut window::Window,
    ) -> Capsule {
        let r = capsule.radius;
        let h = capsule.half_height() * 2.0;
        #[cfg(feature = "dim2")]
        let node = window.add_planar_capsule(r, h);
        #[cfg(feature = "dim3")]
        let node = window.add_capsule(r, h);

        let mut res = Capsule {
            color,
            base_color: color,
            gfx: node,
            collider,
            delta: delta * capsule.transform_wrt_y(),
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn update(&mut self, colliders: &ColliderSet) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.collider,
            &self.color,
            &self.delta,
        );
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
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
