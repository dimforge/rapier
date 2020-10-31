#[cfg(feature = "dim3")]
use kiss3d::camera::ArcBall as Camera;
#[cfg(feature = "dim2")]
use kiss3d::planar_camera::Sidescroll as Camera;
use kiss3d::window::Window;

use na::Point3;

use crate::math::Point;
use crate::objects::ball::Ball;
use crate::objects::box_node::Box as BoxNode;
use crate::objects::heightfield::HeightField;
use crate::objects::node::{GraphicsNode, Node};
use rapier::dynamics::{RigidBodyHandle, RigidBodySet};
use rapier::geometry::{Collider, ColliderHandle, ColliderSet};
//use crate::objects::capsule::Capsule;
//use crate::objects::convex::Convex;
//#[cfg(feature = "dim3")]
//use crate::objects::mesh::Mesh;
//use crate::objects::plane::Plane;
//#[cfg(feature = "dim2")]
//use crate::objects::polyline::Polyline;
use crate::objects::capsule::Capsule;
#[cfg(feature = "dim3")]
use crate::objects::cone::Cone;
#[cfg(feature = "dim3")]
use crate::objects::cylinder::Cylinder;
use crate::objects::mesh::Mesh;
use rand::{Rng, SeedableRng};
use rand_pcg::Pcg32;
use std::collections::HashMap;

pub trait GraphicsWindow {
    fn remove_graphics_node(&mut self, node: &mut GraphicsNode);
    fn draw_graphics_line(&mut self, p1: &Point<f32>, p2: &Point<f32>, color: &Point3<f32>);
}

impl GraphicsWindow for Window {
    fn remove_graphics_node(&mut self, node: &mut GraphicsNode) {
        #[cfg(feature = "dim2")]
        self.remove_planar_node(node);
        #[cfg(feature = "dim3")]
        self.remove_node(node);
    }

    fn draw_graphics_line(&mut self, p1: &Point<f32>, p2: &Point<f32>, color: &Point3<f32>) {
        #[cfg(feature = "dim2")]
        self.draw_planar_line(p1, p2, color);
        #[cfg(feature = "dim3")]
        self.draw_line(p1, p2, color);
    }
}

pub struct GraphicsManager {
    rand: Pcg32,
    b2sn: HashMap<RigidBodyHandle, Vec<Node>>,
    b2color: HashMap<RigidBodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    b2wireframe: HashMap<RigidBodyHandle, bool>,
    ground_color: Point3<f32>,
    camera: Camera,
    ground_handle: Option<RigidBodyHandle>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let mut camera;

        #[cfg(feature = "dim3")]
        {
            camera = Camera::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
            camera.set_rotate_modifiers(Some(kiss3d::event::Modifiers::Control));
        }

        #[cfg(feature = "dim2")]
        {
            camera = Camera::new();
            camera.set_zoom(50.0);
        }

        GraphicsManager {
            camera,
            rand: Pcg32::seed_from_u64(0),
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            ground_color: Point3::new(0.5, 0.5, 0.5),
            b2wireframe: HashMap::new(),
            ground_handle: None,
        }
    }

    pub fn set_ground_handle(&mut self, handle: Option<RigidBodyHandle>) {
        self.ground_handle = handle
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.b2sn.values_mut() {
            for sn in sns.iter_mut() {
                if let Some(node) = sn.scene_node_mut() {
                    window.remove_graphics_node(node);
                }
            }
        }

        self.b2sn.clear();
        self.c2color.clear();
        self.b2color.clear();
        self.b2wireframe.clear();
        self.rand = Pcg32::seed_from_u64(0);
    }

    pub fn remove_body_nodes(&mut self, window: &mut Window, body: RigidBodyHandle) {
        if let Some(sns) = self.b2sn.get_mut(&body) {
            for sn in sns.iter_mut() {
                if let Some(node) = sn.scene_node_mut() {
                    window.remove_graphics_node(node);
                }
            }
        }

        self.b2sn.remove(&body);
    }

    pub fn set_body_color(&mut self, b: RigidBodyHandle, color: Point3<f32>) {
        self.b2color.insert(b, color);

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_collider_initial_color(&mut self, c: ColliderHandle, color: Point3<f32>) {
        self.c2color.insert(c, color);
    }

    pub fn set_body_wireframe(&mut self, b: RigidBodyHandle, enabled: bool) {
        self.b2wireframe.insert(b, enabled);

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut().filter_map(|n| n.scene_node_mut()) {
                if enabled {
                    n.set_surface_rendering_activation(true);
                    n.set_lines_width(1.0);
                } else {
                    n.set_surface_rendering_activation(false);
                    n.set_lines_width(1.0);
                }
            }
        }
    }

    pub fn toggle_wireframe_mode(&mut self, colliders: &ColliderSet, enabled: bool) {
        for n in self.b2sn.values_mut().flat_map(|val| val.iter_mut()) {
            let force_wireframe = if let Some(collider) = colliders.get(n.collider()) {
                collider.is_sensor()
                    || self
                        .b2wireframe
                        .get(&collider.parent())
                        .cloned()
                        .unwrap_or(false)
            } else {
                false
            };

            if let Some(node) = n.scene_node_mut() {
                if force_wireframe || enabled {
                    node.set_lines_width(1.0);
                    node.set_surface_rendering_activation(false);
                } else {
                    node.set_lines_width(0.0);
                    node.set_surface_rendering_activation(true);
                }
            }
        }
    }

    pub fn next_color(&mut self) -> Point3<f32> {
        Self::gen_color(&mut self.rand)
    }

    fn gen_color(rng: &mut Pcg32) -> Point3<f32> {
        let mut color: Point3<f32> = rng.gen();
        color *= 1.5;
        color.x = color.x.min(1.0);
        color.y = color.y.min(1.0);
        color.z = color.z.min(1.0);
        color
    }

    fn alloc_color(&mut self, handle: RigidBodyHandle, is_static: bool) -> Point3<f32> {
        let mut color = self.ground_color;

        if !is_static {
            match self.b2color.get(&handle).cloned() {
                Some(c) => color = c,
                None => color = Self::gen_color(&mut self.rand),
            }
        }

        self.set_body_color(handle, color);

        color
    }

    pub fn add(
        &mut self,
        window: &mut Window,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        let body = bodies.get(handle).unwrap();

        let color = self
            .b2color
            .get(&handle)
            .cloned()
            .unwrap_or_else(|| self.alloc_color(handle, !body.is_dynamic()));

        self.add_with_color(window, handle, bodies, colliders, color)
    }

    pub fn add_with_color(
        &mut self,
        window: &mut Window,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        color: Point3<f32>,
    ) {
        //        let body = bodies.get(handle).unwrap();
        let mut new_nodes = Vec::new();

        for collider_handle in bodies[handle].colliders() {
            let color = self.c2color.get(collider_handle).copied().unwrap_or(color);
            let collider = &colliders[*collider_handle];
            self.add_collider(window, *collider_handle, collider, color, &mut new_nodes);
        }

        new_nodes.iter_mut().for_each(|n| n.update(colliders));

        for node in new_nodes.iter_mut().filter_map(|n| n.scene_node_mut()) {
            if self.b2wireframe.get(&handle).cloned() == Some(true) {
                node.set_lines_width(1.0);
                node.set_surface_rendering_activation(false);
            } else {
                node.set_lines_width(0.0);
                node.set_surface_rendering_activation(true);
            }
        }

        let nodes = self.b2sn.entry(handle).or_insert_with(Vec::new);
        nodes.append(&mut new_nodes);
    }

    fn add_collider(
        &mut self,
        window: &mut Window,
        handle: ColliderHandle,
        collider: &Collider,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let shape = collider.shape();

        if let Some(ball) = shape.as_ball() {
            out.push(Node::Ball(Ball::new(handle, ball.radius, color, window)))
        }

        // Shape::Polygon(poly) => out.push(Node::Convex(Convex::new(
        //     handle,
        //     poly.vertices().to_vec(),
        //     color,
        //     window,
        // ))),

        if let Some(cuboid) = shape.as_cuboid() {
            out.push(Node::Box(BoxNode::new(
                handle,
                cuboid.half_extents,
                color,
                window,
            )))
        }

        if let Some(capsule) = shape.as_capsule() {
            out.push(Node::Capsule(Capsule::new(handle, capsule, color, window)))
        }

        if let Some(triangle) = shape.as_triangle() {
            out.push(Node::Mesh(Mesh::new(
                handle,
                vec![triangle.a, triangle.b, triangle.c],
                vec![Point3::new(0, 1, 2)],
                color,
                window,
            )))
        }

        if let Some(trimesh) = shape.as_trimesh() {
            out.push(Node::Mesh(Mesh::new(
                handle,
                trimesh.vertices().to_vec(),
                trimesh
                    .indices()
                    .iter()
                    .map(|idx| na::convert(*idx))
                    .collect(),
                color,
                window,
            )))
        }

        if let Some(heightfield) = shape.as_heightfield() {
            out.push(Node::HeightField(HeightField::new(
                handle,
                heightfield,
                color,
                window,
            )))
        }

        #[cfg(feature = "dim3")]
        if let Some(cylinder) = shape
            .as_cylinder()
            .or(shape.as_round_cylinder().map(|r| &r.cylinder))
        {
            out.push(Node::Cylinder(Cylinder::new(
                handle,
                cylinder.half_height,
                cylinder.radius,
                color,
                window,
            )))
        }

        #[cfg(feature = "dim3")]
        if let Some(cone) = shape.as_cone() {
            out.push(Node::Cone(Cone::new(
                handle,
                cone.half_height,
                cone.radius,
                color,
                window,
            )))
        }
    }

    /*
    fn add_plane(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        shape: &shape::Plane<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let pos = colliders.get(object).unwrap().position();
        let position = Point::from(pos.translation.vector);
        let normal = pos * shape.normal();

        out.push(Node::Plane(Plane::new(
            object, colliders, &position, &normal, color, window,
        )))
    }

    #[cfg(feature = "dim2")]
    fn add_polyline(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Polyline<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = shape.points().to_vec();
        let indices = shape.edges().iter().map(|e| e.indices).collect();

        out.push(Node::Polyline(Polyline::new(
            object, colliders, delta, vertices, indices, color, window,
        )))
    }

    #[cfg(feature = "dim3")]
    fn add_mesh(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &TriMesh<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let points = shape.points();
        let faces = shape.faces();

        let is = faces
            .iter()
            .map(|f| Point3::new(f.indices.x as u32, f.indices.y as u32, f.indices.z as u32))
            .collect();

        out.push(Node::Mesh(Mesh::new(
            object,
            colliders,
            delta,
            points.to_vec(),
            is,
            color,
            window,
        )))
    }

    fn add_heightfield(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::HeightField(HeightField::new(
            object,
            colliders,
            delta,
            heightfield,
            color,
            window,
        )))
    }

    fn add_capsule(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Capsule<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();
        out.push(Node::Capsule(Capsule::new(
            object,
            colliders,
            delta,
            shape.radius() + margin,
            shape.height(),
            color,
            window,
        )))
    }

    fn add_ball(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Ball<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();
        out.push(Node::Ball(Ball::new(
            object,
            colliders,
            delta,
            shape.radius() + margin,
            color,
            window,
        )))
    }

    fn add_box(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &Cuboid<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();

        out.push(Node::Box(Box::new(
            object,
            colliders,
            delta,
            shape.half_extents() + Vector::repeat(margin),
            color,
            window,
        )))
    }

    #[cfg(feature = "dim2")]
    fn add_convex(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &ConvexPolygon<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let points = shape.points();

        out.push(Node::Convex(Convex::new(
            object,
            colliders,
            delta,
            points.to_vec(),
            color,
            window,
        )))
    }

    #[cfg(feature = "dim3")]
    fn add_convex(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &ConvexHull<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let mut chull = transformation::convex_hull(shape.points());
        chull.replicate_vertices();
        chull.recompute_normals();

        out.push(Node::Convex(Convex::new(
            object, colliders, delta, &chull, color, window,
        )))
    }
    */

    pub fn draw(&mut self, _bodies: &RigidBodySet, colliders: &ColliderSet, window: &mut Window) {
        // use kiss3d::camera::Camera;
        // println!(
        //     "camera eye {:?}, at: {:?}",
        //     self.camera.eye(),
        //     self.camera.at()
        // );
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                /*
                if let Some(co) = colliders.get(n.collider()) {
                    let bo = &bodies[co.parent()];

                    if bo.is_dynamic() {
                        if bo.is_sleeping() {
                            n.set_color(Point3::new(1.0, 0.0, 0.0));
                        } else {
                            n.set_color(Point3::new(0.0, 1.0, 0.0));
                        }
                    }
                }
                 */

                n.update(colliders);
                n.draw(window);
            }
        }
    }

    // pub fn draw_positions(&mut self, window: &mut Window, rbs: &RigidBodies<f32>) {
    //     for (_, ns) in self.b2sn.iter_mut() {
    //         for n in ns.iter_mut() {
    //             let object = n.object();
    //             let rb = rbs.get(object).expect("Rigid body not found.");

    //             // if let WorldObjectBorrowed::RigidBody(rb) = object {
    //                 let t      = rb.position();
    //                 let center = rb.center_of_mass();

    //                 let rotmat = t.rotation.to_rotation_matrix().unwrap();
    //                 let x = rotmat.column(0) * 0.25f32;
    //                 let y = rotmat.column(1) * 0.25f32;
    //                 let z = rotmat.column(2) * 0.25f32;

    //                 window.draw_line(center, &(*center + x), &Point3::new(1.0, 0.0, 0.0));
    //                 window.draw_line(center, &(*center + y), &Point3::new(0.0, 1.0, 0.0));
    //                 window.draw_line(center, &(*center + z), &Point3::new(0.0, 0.0, 1.0));
    //             // }
    //         }
    //     }
    // }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    pub fn camera_mut(&mut self) -> &mut Camera {
        &mut self.camera
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point<f32>, at: Point<f32>) {
        self.camera.look_at(eye, at);
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point<f32>, zoom: f32) {
        self.camera.look_at(at, zoom);
    }

    pub fn body_nodes(&self, handle: RigidBodyHandle) -> Option<&Vec<Node>> {
        self.b2sn.get(&handle)
    }

    pub fn body_nodes_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut Vec<Node>> {
        self.b2sn.get_mut(&handle)
    }

    pub fn nodes(&self) -> impl Iterator<Item = &Node> {
        self.b2sn.values().flat_map(|val| val.iter())
    }

    pub fn nodes_mut(&mut self) -> impl Iterator<Item = &mut Node> {
        self.b2sn.values_mut().flat_map(|val| val.iter_mut())
    }

    #[cfg(feature = "dim3")]
    pub fn set_up_axis(&mut self, up_axis: na::Vector3<f32>) {
        self.camera.set_up_axis(up_axis);
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
