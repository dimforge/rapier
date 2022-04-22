use crate::dynamics::{
    ImpulseJoint, ImpulseJointHandle, Multibody, MultibodyLink, RigidBody, RigidBodyHandle,
};
use crate::geometry::Collider;
use crate::math::{Isometry, Point, Real, Vector};
use crate::prelude::{ColliderHandle, MultibodyJointHandle};
use na::Scale;

#[derive(Copy, Clone)]
pub enum DebugRenderObject<'a> {
    RigidBody(RigidBodyHandle, &'a RigidBody),
    Collider(ColliderHandle, &'a Collider),
    ImpulseJoint(ImpulseJointHandle, &'a ImpulseJoint),
    MultibodyJoint(MultibodyJointHandle, &'a Multibody, &'a MultibodyLink),
    Other,
}

pub trait DebugRenderBackend {
    fn draw_line(
        &mut self,
        object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    );

    fn draw_polyline(
        &mut self,
        object: DebugRenderObject,
        vertices: &[Point<Real>],
        indices: &[[u32; 2]],
        transform: &Isometry<Real>,
        scale: &Vector<Real>,
        color: [f32; 4],
    ) {
        for idx in indices {
            let a = transform * (Scale::from(*scale) * vertices[idx[0] as usize]);
            let b = transform * (Scale::from(*scale) * vertices[idx[1] as usize]);
            self.draw_line(object, a, b, color);
        }
    }

    fn draw_line_strip(
        &mut self,
        object: DebugRenderObject,
        vertices: &[Point<Real>],
        transform: &Isometry<Real>,
        scale: &Vector<Real>,
        color: [f32; 4],
        closed: bool,
    ) {
        for vtx in vertices.windows(2) {
            let a = transform * (Scale::from(*scale) * vtx[0]);
            let b = transform * (Scale::from(*scale) * vtx[1]);
            self.draw_line(object, a, b, color);
        }

        if closed {
            if vertices.len() > 2 {
                let a = transform * (Scale::from(*scale) * vertices[0]);
                let b = transform * (Scale::from(*scale) * vertices.last().unwrap());
                self.draw_line(object, a, b, color);
            }
        }
    }
}
