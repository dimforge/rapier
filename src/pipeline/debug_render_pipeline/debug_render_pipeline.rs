use super::{outlines, DebugRenderBackend};
use crate::dynamics::{
    GenericJoint, ImpulseJointSet, MultibodyJointSet, RigidBodySet, RigidBodyType,
};
use crate::geometry::{Ball, ColliderSet, Cuboid, Shape, TypedShape};
#[cfg(feature = "dim3")]
use crate::geometry::{Cone, Cylinder};
use crate::math::{Isometry, Point, Real, Vector, DIM};
use crate::pipeline::debug_render_pipeline::debug_render_backend::DebugRenderObject;
use crate::pipeline::debug_render_pipeline::DebugRenderStyle;
use crate::utils::WBasis;
use std::any::TypeId;
use std::collections::HashMap;

bitflags::bitflags! {
    pub struct DebugRenderMode: u32 {
        const COLLIDER_SHAPES = 1 << 0;
        const RIGID_BODY_AXES = 1 << 1;
        const MULTIBODY_JOINTS = 1 << 2;
        const IMPULSE_JOINTS = 1 << 3;
    }
}

pub struct DebugRenderPipeline {
    #[cfg(feature = "dim2")]
    instances: HashMap<TypeId, Vec<Point<Real>>>,
    #[cfg(feature = "dim3")]
    instances: HashMap<TypeId, (Vec<Point<Real>>, Vec<[u32; 2]>)>,
    pub style: DebugRenderStyle,
    pub mode: DebugRenderMode,
}

impl Default for DebugRenderPipeline {
    fn default() -> Self {
        Self::render_all(DebugRenderStyle::default())
    }
}

impl DebugRenderPipeline {
    pub fn new(style: DebugRenderStyle, mode: DebugRenderMode) -> Self {
        Self {
            instances: outlines::instances(style.subdivisions),
            style,
            mode,
        }
    }

    pub fn render_all(style: DebugRenderStyle) -> Self {
        Self::new(style, DebugRenderMode::all())
    }

    pub fn render(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
    ) {
        self.render_bodies(backend, bodies);
        self.render_colliders(backend, bodies, colliders);
        self.render_joints(backend, bodies, impulse_joints, multibody_joints);
    }

    pub fn render_joints(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
    ) {
        let mut render_joint = |body1,
                                body2,
                                data: &GenericJoint,
                                mut anchor_color: [f32; 4],
                                mut separation_color: [f32; 4],
                                object| {
            if let (Some(rb1), Some(rb2)) = (bodies.get(body1), bodies.get(body2)) {
                let coeff = if (rb1.is_fixed() || rb1.is_sleeping())
                    && (rb2.is_fixed() || rb2.is_sleeping())
                {
                    self.style.sleep_color_multiplier
                } else {
                    [1.0; 4]
                };

                let frame1 = rb1.position() * data.local_frame1;
                let frame2 = rb2.position() * data.local_frame2;

                let a = *rb1.translation();
                let b = frame1.translation.vector;
                let c = frame2.translation.vector;
                let d = *rb2.translation();

                for k in 0..4 {
                    anchor_color[k] *= coeff[k];
                    separation_color[k] *= coeff[k];
                }

                backend.draw_line(object, a.into(), b.into(), anchor_color);
                backend.draw_line(object, b.into(), c.into(), separation_color);
                backend.draw_line(object, c.into(), d.into(), anchor_color);
            }
        };

        if self.mode.contains(DebugRenderMode::IMPULSE_JOINTS) {
            for (handle, joint) in impulse_joints.iter() {
                let anc_color = self.style.impulse_joint_anchor_color;
                let sep_color = self.style.impulse_joint_separation_color;
                let object = DebugRenderObject::ImpulseJoint(handle, joint);
                render_joint(
                    joint.body1,
                    joint.body2,
                    &joint.data,
                    anc_color,
                    sep_color,
                    object,
                );
            }
        }

        if self.mode.contains(DebugRenderMode::MULTIBODY_JOINTS) {
            for (handle, multibody, link) in multibody_joints.iter() {
                let anc_color = self.style.multibody_joint_anchor_color;
                let sep_color = self.style.multibody_joint_separation_color;
                let parent = multibody.link(link.parent_id().unwrap()).unwrap();
                let object = DebugRenderObject::MultibodyJoint(handle, multibody, link);
                render_joint(
                    parent.rigid_body_handle(),
                    link.rigid_body_handle(),
                    &link.joint.data,
                    anc_color,
                    sep_color,
                    object,
                );
            }
        }
    }

    pub fn render_bodies(&mut self, backend: &mut impl DebugRenderBackend, bodies: &RigidBodySet) {
        for (handle, rb) in bodies.iter() {
            let object = DebugRenderObject::RigidBody(handle, rb);

            if self.style.rigid_body_axes_length != 0.0
                && self.mode.contains(DebugRenderMode::RIGID_BODY_AXES)
            {
                let basis = rb.rotation().to_rotation_matrix().into_inner();
                let coeff = if rb.is_sleeping() {
                    self.style.sleep_color_multiplier
                } else {
                    [1.0; 4]
                };
                let colors = [
                    [0.0 * coeff[0], 1.0 * coeff[1], 0.25 * coeff[2], coeff[3]],
                    [120.0 * coeff[0], 1.0 * coeff[1], 0.1 * coeff[2], coeff[3]],
                    [240.0 * coeff[0], 1.0 * coeff[1], 0.2 * coeff[2], coeff[3]],
                ];
                let com = rb.mprops.world_com;

                for k in 0..DIM {
                    let axis = basis.column(k) * self.style.rigid_body_axes_length;
                    backend.draw_line(object, com, com + axis, colors[k]);
                }
            }
        }
    }

    pub fn render_colliders(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        if self.mode.contains(DebugRenderMode::COLLIDER_SHAPES) {
            for (h, co) in colliders.iter() {
                let object = DebugRenderObject::Collider(h, co);
                let color = if let Some(parent) = co.parent().and_then(|p| bodies.get(p)) {
                    let coeff = if parent.is_sleeping() {
                        self.style.sleep_color_multiplier
                    } else {
                        [1.0; 4]
                    };
                    let c = match parent.body_type {
                        RigidBodyType::Fixed => self.style.collider_fixed_color,
                        RigidBodyType::Dynamic => self.style.collider_dynamic_color,
                        RigidBodyType::KinematicPositionBased
                        | RigidBodyType::KinematicVelocityBased => {
                            self.style.collider_kinematic_color
                        }
                    };

                    [
                        c[0] * coeff[0],
                        c[1] * coeff[1],
                        c[2] * coeff[2],
                        c[3] * coeff[3],
                    ]
                } else {
                    self.style.collider_parentless_color
                };

                self.render_shape(object, backend, co.shape(), co.position(), color)
            }
        }
    }

    #[cfg(feature = "dim2")]
    fn render_shape(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Isometry<Real>,
        color: [f32; 4],
    ) {
        match shape.as_typed_shape() {
            TypedShape::Ball(s) => {
                let vtx = &self.instances[&TypeId::of::<Ball>()];
                backend.draw_line_strip(
                    object,
                    vtx,
                    pos,
                    &Vector::repeat(s.radius * 2.0),
                    color,
                    true,
                )
            }
            TypedShape::Cuboid(s) => {
                let vtx = &self.instances[&TypeId::of::<Cuboid>()];
                backend.draw_line_strip(object, vtx, pos, &(s.half_extents * 2.0), color, true)
            }
            TypedShape::Capsule(s) => {
                let vtx = s.to_polyline(self.style.subdivisions);
                backend.draw_line_strip(object, &vtx, pos, &Vector::repeat(1.0), color, true)
            }
            TypedShape::Segment(s) => backend.draw_line_strip(
                object,
                &[s.a, s.b],
                pos,
                &Vector::repeat(1.0),
                color,
                false,
            ),
            TypedShape::Triangle(s) => backend.draw_line_strip(
                object,
                &[s.a, s.b, s.c],
                pos,
                &Vector::repeat(1.0),
                color,
                true,
            ),
            TypedShape::TriMesh(s) => {
                for tri in s.triangles() {
                    self.render_shape(object, backend, &tri, pos, color)
                }
            }
            TypedShape::Polyline(s) => backend.draw_polyline(
                object,
                s.vertices(),
                s.indices(),
                pos,
                &Vector::repeat(1.0),
                color,
            ),
            TypedShape::HalfSpace(s) => {
                let basis = s.normal.orthonormal_basis()[0];
                let a = Point::from(basis) * 10_000.0;
                let b = Point::from(basis) * -10_000.0;
                backend.draw_line_strip(object, &[a, b], pos, &Vector::repeat(1.0), color, false)
            }
            TypedShape::HeightField(s) => {
                for seg in s.segments() {
                    self.render_shape(object, backend, &seg, pos, color)
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    self.render_shape(object, backend, &**shape, &(pos * sub_pos), color)
                }
            }
            TypedShape::ConvexPolygon(s) => {
                backend.draw_line_strip(object, s.points(), pos, &Vector::repeat(1.0), color, true)
            }
            /*
             * Round shapes.
             */
            TypedShape::RoundCuboid(s) => {
                let vtx = s.to_polyline(self.style.border_subdivisions);
                backend.draw_line_strip(object, &vtx, pos, &Vector::repeat(1.0), color, true)
            }
            TypedShape::RoundTriangle(s) => {
                // TODO: take roundness into account.
                self.render_shape(object, backend, &s.inner_shape, pos, color)
            }
            // TypedShape::RoundTriMesh(s) => self.render_shape(backend, &s.inner_shape, pos, color),
            // TypedShape::RoundHeightField(s) => {
            //     self.render_shape(backend, &s.inner_shape, pos, color)
            // }
            TypedShape::RoundConvexPolygon(s) => {
                let vtx = s.to_polyline(self.style.border_subdivisions);
                backend.draw_line_strip(object, &vtx, pos, &Vector::repeat(1.0), color, true)
            }
            TypedShape::Custom(_) => {}
        }
    }

    #[cfg(feature = "dim3")]
    fn render_shape(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Isometry<Real>,
        color: [f32; 4],
    ) {
        match shape.as_typed_shape() {
            TypedShape::Ball(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Ball>()];
                backend.draw_polyline(
                    object,
                    vtx,
                    idx,
                    pos,
                    &Vector::repeat(s.radius * 2.0),
                    color,
                )
            }
            TypedShape::Cuboid(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Cuboid>()];
                backend.draw_polyline(object, vtx, idx, pos, &(s.half_extents * 2.0), color)
            }
            TypedShape::Capsule(s) => {
                let (vtx, idx) = s.to_outline(self.style.subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, &Vector::repeat(1.0), color)
            }
            TypedShape::Segment(s) => backend.draw_polyline(
                object,
                &[s.a, s.b],
                &[[0, 1]],
                pos,
                &Vector::repeat(1.0),
                color,
            ),
            TypedShape::Triangle(s) => backend.draw_line_strip(
                object,
                &[s.a, s.b, s.c],
                pos,
                &Vector::repeat(1.0),
                color,
                true,
            ),
            TypedShape::TriMesh(s) => {
                for tri in s.triangles() {
                    self.render_shape(object, backend, &tri, pos, color)
                }
            }
            TypedShape::Polyline(s) => backend.draw_polyline(
                object,
                s.vertices(),
                s.indices(),
                pos,
                &Vector::repeat(1.0),
                color,
            ),
            TypedShape::HalfSpace(s) => {
                let basis = s.normal.orthonormal_basis();
                let a = Point::from(basis[0]) * 10_000.0;
                let b = Point::from(basis[0]) * -10_000.0;
                let c = Point::from(basis[1]) * 10_000.0;
                let d = Point::from(basis[1]) * -10_000.0;
                backend.draw_polyline(
                    object,
                    &[a, b, c, d],
                    &[[0, 1], [2, 3]],
                    pos,
                    &Vector::repeat(1.0),
                    color,
                )
            }
            TypedShape::HeightField(s) => {
                for tri in s.triangles() {
                    self.render_shape(object, backend, &tri, pos, color)
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    self.render_shape(object, backend, &**shape, &(pos * sub_pos), color)
                }
            }
            TypedShape::ConvexPolyhedron(s) => {
                let indices: Vec<_> = s
                    .edges()
                    .iter()
                    .map(|e| [e.vertices.x, e.vertices.y])
                    .collect();
                backend.draw_polyline(
                    object,
                    s.points(),
                    &indices,
                    pos,
                    &Vector::repeat(1.0),
                    color,
                )
            }
            TypedShape::Cylinder(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Cylinder>()];
                backend.draw_polyline(
                    object,
                    vtx,
                    idx,
                    pos,
                    &(Vector::new(s.radius, s.half_height, s.radius) * 2.0),
                    color,
                )
            }
            TypedShape::Cone(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Cone>()];
                backend.draw_polyline(
                    object,
                    vtx,
                    idx,
                    pos,
                    &(Vector::new(s.radius, s.half_height, s.radius) * 2.0),
                    color,
                )
            }
            /*
             * Round shapes.
             */
            TypedShape::RoundCuboid(s) => {
                let (vtx, idx) = s.to_outline(self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, &Vector::repeat(1.0), color)
            }
            TypedShape::RoundTriangle(s) => {
                // TODO: take roundness into account.
                self.render_shape(object, backend, &s.inner_shape, pos, color)
            }
            // TypedShape::RoundTriMesh(s) => self.render_shape(object, backend, &s.inner_shape, pos, color),
            // TypedShape::RoundHeightField(s) => {
            //     self.render_shape(object, backend, &s.inner_shape, pos, color)
            // }
            TypedShape::RoundCylinder(s) => {
                let (vtx, idx) =
                    s.to_outline(self.style.subdivisions, self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, &Vector::repeat(1.0), color)
            }
            TypedShape::RoundCone(s) => {
                let (vtx, idx) =
                    s.to_outline(self.style.subdivisions, self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, &Vector::repeat(1.0), color)
            }
            TypedShape::RoundConvexPolyhedron(s) => {
                let (vtx, idx) = s.to_outline(self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, &Vector::repeat(1.0), color)
            }
            TypedShape::Custom(_) => {}
        }
    }
}
