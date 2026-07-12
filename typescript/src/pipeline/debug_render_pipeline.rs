use crate::dynamics::{RawImpulseJointSet, RawMultibodyJointSet, RawRigidBodySet};
use crate::geometry::{RawColliderSet, RawNarrowPhase};
use js_sys::Float32Array;
use palette::convert::IntoColorUnclamped;
use palette::rgb::Rgba;
use palette::Hsla;
use rapier::dynamics::{RigidBody, RigidBodySet};
use rapier::geometry::ColliderSet;
use rapier::math::{Point, Real};
use rapier::pipeline::{DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
use rapier::prelude::{QueryFilter, QueryFilterFlags};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawDebugRenderPipeline {
    pub(crate) raw: DebugRenderPipeline,
    vertices: Vec<f32>,
    colors: Vec<f32>,
}

#[wasm_bindgen]
impl RawDebugRenderPipeline {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawDebugRenderPipeline {
            raw: DebugRenderPipeline::default(),
            vertices: vec![],
            colors: vec![],
        }
    }

    pub fn vertices(&self) -> Float32Array {
        let output = Float32Array::new_with_length(self.vertices.len() as u32);
        output.copy_from(&self.vertices);
        output
    }

    pub fn colors(&self) -> Float32Array {
        let output = Float32Array::new_with_length(self.colors.len() as u32);
        output.copy_from(&self.colors);
        output
    }

    pub fn render(
        &mut self,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        impulse_joints: &RawImpulseJointSet,
        multibody_joints: &RawMultibodyJointSet,
        narrow_phase: &RawNarrowPhase,
        filter_flags: u32,
        filter_predicate: &js_sys::Function,
    ) {
        self.vertices.clear();
        self.colors.clear();

        crate::utils::with_filter(filter_predicate, |predicate| {
            let mut backend = CopyToBuffersBackend {
                filter: QueryFilter {
                    flags: QueryFilterFlags::from_bits(filter_flags)
                        .unwrap_or(QueryFilterFlags::empty()),
                    groups: None,
                    exclude_collider: None,
                    exclude_rigid_body: None,
                    predicate,
                },
                bodies: &bodies.0,
                colliders: &colliders.0,
                vertices: &mut self.vertices,
                colors: &mut self.colors,
            };

            self.raw.render(
                &mut backend,
                &bodies.0,
                &colliders.0,
                &impulse_joints.0,
                &multibody_joints.0,
                &narrow_phase.0,
            )
        })
    }
}

struct CopyToBuffersBackend<'a> {
    filter: QueryFilter<'a>,
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    vertices: &'a mut Vec<f32>,
    colors: &'a mut Vec<f32>,
}

impl<'a> DebugRenderBackend for CopyToBuffersBackend<'a> {
    fn filter_object(&self, object: DebugRenderObject) -> bool {
        let test_rigid_body = |rb: &RigidBody| {
            rb.colliders().iter().all(|handle| {
                let Some(co) = self.colliders.get(*handle) else {
                    return false;
                };
                self.filter.test(self.bodies, *handle, co)
            })
        };

        match object {
            DebugRenderObject::Collider(handle, co)
            | DebugRenderObject::ColliderAabb(handle, co, _) => {
                self.filter.test(self.bodies, handle, co)
            }
            DebugRenderObject::ContactPair(pair, co1, co2) => {
                self.filter.test(self.bodies, pair.collider1, co1)
                    && self.filter.test(self.bodies, pair.collider2, co2)
            }
            DebugRenderObject::ImpulseJoint(_, joint) => {
                let Some(rb1) = self.bodies.get(joint.body1) else {
                    return false;
                };
                let Some(rb2) = self.bodies.get(joint.body2) else {
                    return false;
                };
                test_rigid_body(rb1) && test_rigid_body(rb2)
            }
            DebugRenderObject::MultibodyJoint(_, _, link) => {
                let Some(rb) = self.bodies.get(link.rigid_body_handle()) else {
                    return false;
                };
                test_rigid_body(rb)
            }
            DebugRenderObject::RigidBody(_, rb) => test_rigid_body(rb),
        }
    }

    /// Draws a colored line.
    ///
    /// Note that this method can be called multiple time for the same `object`.
    fn draw_line(
        &mut self,
        _object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        self.vertices.extend_from_slice(a.coords.as_slice());
        self.vertices.extend_from_slice(b.coords.as_slice());

        // Convert to RGB which will be easier to handle in JS.
        let hsl = Hsla::new(color[0], color[1], color[2], color[3]);
        let rgb: Rgba = hsl.into_color_unclamped();
        self.colors.extend_from_slice(&[
            rgb.red, rgb.green, rgb.blue, rgb.alpha, rgb.red, rgb.green, rgb.blue, rgb.alpha,
        ]);
    }
}
