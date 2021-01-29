use crate::objects::ball::Ball;
use crate::objects::box_node::Box;
use crate::objects::capsule::Capsule;
use crate::objects::convex::Convex;
use crate::objects::heightfield::HeightField;
use crate::objects::mesh::Mesh;
//use crate::objects::plane::Plane;
use crate::objects::polyline::Polyline;
use kiss3d::window::Window;
use na::Point3;

use crate::objects::cone::Cone;
use crate::objects::cylinder::Cylinder;
use rapier::geometry::{ColliderHandle, ColliderSet};
use rapier::math::Isometry;

#[cfg(feature = "dim2")]
pub type GraphicsNode = kiss3d::scene::PlanarSceneNode;
#[cfg(feature = "dim3")]
pub type GraphicsNode = kiss3d::scene::SceneNode;

pub enum Node {
    //    Plane(Plane),
    Ball(Ball),
    Box(Box),
    HeightField(HeightField),
    Capsule(Capsule),
    Polyline(Polyline),
    Mesh(Mesh),
    Convex(Convex),
    Cylinder(Cylinder),
    Cone(Cone),
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            //            Node::Plane(ref mut n) => n.select(),
            Node::Ball(ref mut n) => n.select(),
            Node::Box(ref mut n) => n.select(),
            Node::Capsule(ref mut n) => n.select(),
            Node::HeightField(ref mut n) => n.select(),
            Node::Polyline(ref mut n) => n.select(),
            Node::Mesh(ref mut n) => n.select(),
            Node::Convex(ref mut n) => n.select(),
            Node::Cylinder(ref mut n) => n.select(),
            Node::Cone(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            //            Node::Plane(ref mut n) => n.unselect(),
            Node::Ball(ref mut n) => n.unselect(),
            Node::Box(ref mut n) => n.unselect(),
            Node::Capsule(ref mut n) => n.unselect(),
            Node::HeightField(ref mut n) => n.unselect(),
            Node::Polyline(ref mut n) => n.unselect(),
            Node::Mesh(ref mut n) => n.unselect(),
            Node::Convex(ref mut n) => n.unselect(),
            Node::Cylinder(ref mut n) => n.unselect(),
            Node::Cone(ref mut n) => n.unselect(),
        }
    }

    pub fn update(&mut self, colliders: &ColliderSet) {
        match *self {
            //            Node::Plane(ref mut n) => n.update(colliders),
            Node::Ball(ref mut n) => n.update(colliders),
            Node::Box(ref mut n) => n.update(colliders),
            Node::Capsule(ref mut n) => n.update(colliders),
            Node::HeightField(ref mut n) => n.update(colliders),
            Node::Polyline(ref mut n) => n.update(colliders),
            Node::Mesh(ref mut n) => n.update(colliders),
            Node::Convex(ref mut n) => n.update(colliders),
            Node::Cylinder(ref mut n) => n.update(colliders),
            Node::Cone(ref mut n) => n.update(colliders),
        }
    }

    #[cfg(feature = "dim2")]
    pub fn draw(&mut self, window: &mut Window) {
        match *self {
            Node::Polyline(ref mut n) => n.draw(window),
            Node::HeightField(ref mut n) => n.draw(window),
            //            Node::Plane(ref mut n) => n.draw(_window),
            _ => {}
        }
    }

    #[cfg(feature = "dim3")]
    pub fn draw(&mut self, _: &mut Window) {}

    pub fn scene_node(&self) -> Option<&GraphicsNode> {
        match *self {
            //            #[cfg(feature = "dim3")]
            //            Node::Plane(ref n) => Some(n.scene_node()),
            Node::Ball(ref n) => Some(n.scene_node()),
            Node::Box(ref n) => Some(n.scene_node()),
            Node::Capsule(ref n) => Some(n.scene_node()),
            #[cfg(feature = "dim3")]
            Node::HeightField(ref n) => Some(n.scene_node()),
            Node::Mesh(ref n) => Some(n.scene_node()),
            Node::Convex(ref n) => Some(n.scene_node()),
            Node::Cylinder(ref n) => Some(n.scene_node()),
            Node::Cone(ref n) => Some(n.scene_node()),
            Node::Polyline(_) => None,
            #[cfg(feature = "dim2")]
            Node::HeightField(_) => None,
        }
    }

    pub fn scene_node_mut(&mut self) -> Option<&mut GraphicsNode> {
        match *self {
            //            #[cfg(feature = "dim3")]
            //            Node::Plane(ref mut n) => Some(n.scene_node_mut()),
            Node::Ball(ref mut n) => Some(n.scene_node_mut()),
            Node::Box(ref mut n) => Some(n.scene_node_mut()),
            Node::Capsule(ref mut n) => Some(n.scene_node_mut()),
            #[cfg(feature = "dim3")]
            Node::HeightField(ref mut n) => Some(n.scene_node_mut()),
            Node::Mesh(ref mut n) => Some(n.scene_node_mut()),
            Node::Convex(ref mut n) => Some(n.scene_node_mut()),
            Node::Cylinder(ref mut n) => Some(n.scene_node_mut()),
            Node::Cone(ref mut n) => Some(n.scene_node_mut()),
            Node::Polyline(_) => None,
            #[cfg(feature = "dim2")]
            Node::HeightField(_) => None,
        }
    }

    pub fn collider(&self) -> ColliderHandle {
        match *self {
            //            Node::Plane(ref n) => n.object(),
            Node::Ball(ref n) => n.object(),
            Node::Box(ref n) => n.object(),
            Node::Capsule(ref n) => n.object(),
            Node::HeightField(ref n) => n.object(),
            Node::Polyline(ref n) => n.object(),
            Node::Mesh(ref n) => n.object(),
            Node::Convex(ref n) => n.object(),
            Node::Cylinder(ref n) => n.object(),
            Node::Cone(ref n) => n.object(),
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        match *self {
            //            Node::Plane(ref mut n) => n.set_color(color),
            Node::Ball(ref mut n) => n.set_color(color),
            Node::Box(ref mut n) => n.set_color(color),
            Node::Capsule(ref mut n) => n.set_color(color),
            Node::HeightField(ref mut n) => n.set_color(color),
            Node::Polyline(ref mut n) => n.set_color(color),
            Node::Mesh(ref mut n) => n.set_color(color),
            Node::Convex(ref mut n) => n.set_color(color),
            Node::Cylinder(ref mut n) => n.set_color(color),
            Node::Cone(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node(
    node: &mut GraphicsNode,
    colliders: &ColliderSet,
    handle: ColliderHandle,
    color: &Point3<f32>,
    delta: &Isometry<f32>,
) {
    if let Some(co) = colliders.get(handle) {
        node.set_local_transformation(co.position() * delta);
        node.set_color(color.x, color.y, color.z);
    } else {
        node.set_visible(false);
        node.unlink();
    }
}
