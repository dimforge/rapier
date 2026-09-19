use super::{DebugColor, DebugRenderBackend, outlines};
use crate::alloc_prelude::*;
use crate::dynamics::{
    GenericJoint, ImpulseJointSet, MultibodyJointSet, RigidBody, RigidBodySet, RigidBodyType,
    SoftBodyEdgeKind, SoftBodySet,
};
use crate::geometry::{Ball, ColliderSet, Cuboid, NarrowPhase, Shape, TypedShape};
#[cfg(feature = "dim3")]
use crate::geometry::{Cone, Cylinder};
#[cfg(feature = "dim2")]
use crate::geometry::{ConvexPolygon, RoundShape};
use crate::math::{DIM, Matrix, Pose, Vector};
use crate::pipeline::debug_render_pipeline::DebugRenderStyle;
use crate::pipeline::debug_render_pipeline::debug_render_backend::DebugRenderObject;
use crate::utils::OrthonormalBasis;
use core::any::TypeId;
use parry::utils::PoseOpt;
use parry::utils::hashmap::HashMap;

bitflags::bitflags! {
    /// Flags indicating what part of the physics engine should be rendered
    /// by the debug-renderer.
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    pub struct DebugRenderMode: u32 {
        /// If this flag is set, the collider shapes will be rendered.
        const COLLIDER_SHAPES = 1 << 0;
        /// If this flag is set, the local coordinate axes of rigid-bodies will be rendered.
        const RIGID_BODY_AXES = 1 << 1;
        /// If this flag is set, the multibody joints will be rendered.
        const MULTIBODY_JOINTS = 1 << 2;
        /// If this flag is set, the impulse joints will be rendered.
        const IMPULSE_JOINTS = 1 << 3;
        /// If this flag is set, all the joints will be rendered.
        const JOINTS = Self::MULTIBODY_JOINTS.bits() | Self::IMPULSE_JOINTS.bits();
        /// If this flag is set, the solver contacts will be rendered.
        const SOLVER_CONTACTS = 1 << 4;
        /// If this flag is set, the geometric contacts will be rendered.
        const CONTACTS = 1 << 5;
        /// If this flag is set, the Aabbs of colliders will be rendered.
        const COLLIDER_AABBS = 1 << 6;
        /// If this flag is set, the soft bodies' elements (structural edges and cell edges) and
        /// their soft-vs-soft contacts (vertex-vs-surface and edge-vs-edge) will be rendered.
        const SOFT_BODIES = 1 << 7;
        /// If this flag is set, the pseudo-normals of the triangle-meshes (3D) and polylines (2D)
        /// that have them will be rendered.
        const PSEUDO_NORMALS = 1 << 8;
        /// If this flag is set, the soft bodies' volume constraints (the intersection-volume
        /// constraints) will be rendered: each constraint's normal at its patch center, and the
        /// volume gradient at every particle it acts on.
        const SOFT_VOLUME_CONTACTS = 1 << 9;
        /// With [`Self::SOFT_BODIES`], colors elements by load, not `soft_body_element_color`:
        /// `soft_body_slack_color` unloaded to `soft_body_loaded_color` at the tear threshold (the
        /// smoothed element `stress`); without a threshold the load is the stretch, full at 50%.
        const SOFT_BODY_STRESS = 1 << 10;
    }
}

impl Default for DebugRenderMode {
    fn default() -> Self {
        Self::COLLIDER_SHAPES | Self::JOINTS | Self::RIGID_BODY_AXES
    }
}

#[cfg(feature = "dim2")]
type InstancesMap = HashMap<TypeId, Vec<Vector>>;
#[cfg(feature = "dim3")]
type InstancesMap = HashMap<TypeId, (Vec<Vector>, Vec<[u32; 2]>)>;

/// Pipeline responsible for rendering the state of the physics engine for debugging purpose.
pub struct DebugRenderPipeline {
    #[cfg(feature = "dim2")]
    instances: InstancesMap,
    #[cfg(feature = "dim3")]
    instances: InstancesMap,
    /// The style used to compute the line colors for each element
    /// to render.
    pub style: DebugRenderStyle,
    /// Flags controlling what part of the physics engine need to
    /// be rendered.
    pub mode: DebugRenderMode,
    /// The soft-body edges to draw for the body being rendered, with their load (a cage edge is
    /// shared by every cell around it: the largest). Kept here to be reused from frame to frame.
    drawn_edges: HashMap<[u32; 2], f32>,
}

impl Default for DebugRenderPipeline {
    fn default() -> Self {
        Self::new(DebugRenderStyle::default(), DebugRenderMode::default())
    }
}

impl DebugRenderPipeline {
    /// Creates a new debug-render pipeline from a given style and flags.
    pub fn new(style: DebugRenderStyle, mode: DebugRenderMode) -> Self {
        Self {
            instances: outlines::instances(style.subdivisions),
            style,
            mode,
            drawn_edges: HashMap::default(),
        }
    }

    /// The color multiplier for one body's attached entities: disabled, asleep, sleep-eligible,
    /// or plain awake. `eligible_tint` opts out for entities whose hue already conveys meaning.
    fn body_color_multiplier(
        &self,
        rb: &RigidBody,
        co_enabled: bool,
        eligible_tint: bool,
    ) -> DebugColor {
        if !rb.is_enabled() || !co_enabled {
            self.style.disabled_color_multiplier
        } else if rb.is_sleeping() {
            self.style.sleep_color_multiplier
        } else if eligible_tint && !rb.is_fixed() && rb.activation().is_eligible_for_sleep() {
            self.style.sleep_eligible_color_multiplier
        } else {
            [1.0; 4]
        }
    }

    /// Creates a new debug-render pipeline that renders everything
    /// it can from the physics state.
    pub fn render_all(style: DebugRenderStyle) -> Self {
        Self::new(style, DebugRenderMode::all())
    }

    /// Render the scene.
    #[profiling::function]
    pub fn render(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        narrow_phase: &NarrowPhase,
        soft_bodies: &SoftBodySet,
    ) {
        self.render_rigid_bodies(backend, bodies);
        self.render_colliders(backend, bodies, colliders);
        self.render_joints(backend, bodies, impulse_joints, multibody_joints);
        self.render_contacts(backend, colliders, narrow_phase);
        self.render_soft_bodies(backend, soft_bodies);
    }

    /// Render the soft bodies' elements (structural edges and cell edges as a wireframe, from
    /// the particles' positions at the end of the last step) and their soft-vs-soft contacts
    /// (vertex-vs-surface and edge-vs-edge).
    #[profiling::function]
    #[allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.
    pub fn render_soft_bodies<B: DebugRenderBackend>(
        &mut self,
        backend: &mut B,
        soft_bodies: &SoftBodySet,
    ) {
        if self.mode.contains(DebugRenderMode::SOFT_VOLUME_CONTACTS) {
            self.render_soft_volume_contacts(backend, soft_bodies);
        }
        if !self.mode.contains(DebugRenderMode::SOFT_BODIES) {
            return;
        }
        for (handle, sb) in soft_bodies.iter() {
            let object = DebugRenderObject::SoftBody(handle, sb);
            if !backend.filter_object(object) {
                continue;
            }
            let element_color = self.style.soft_body_element_color;
            let by_stress = self.mode.contains(DebugRenderMode::SOFT_BODY_STRESS);
            // The load an element is drawn with: its smoothed tear load when the material tears,
            // its stretch (full at 50%) otherwise, so the picture stays meaningful for a body
            // that never tears.
            let tears = sb.material().tears();
            let stretch = |a: u32, b: u32| -> f32 {
                let (pa, pb) = (&sb.particles()[a as usize], &sb.particles()[b as usize]);
                let rest = (pa.rest_position() - pb.rest_position()).length();
                let len = (pa.position() - pb.position()).length();
                if rest > 0.0 {
                    ((len / rest - 1.0).abs() / 0.5) as f32
                } else {
                    0.0
                }
            };
            // The cage is drawn edge by edge rather than cell by cell (an edge is shared by every
            // cell around it, and duplicates only thicken the picture); each edge keeps the largest
            // load of the elements sharing it.
            self.drawn_edges.clear();
            let record = |drawn: &mut HashMap<[u32; 2], f32>, e: [u32; 2], load: f32| {
                let key = [e[0].min(e[1]), e[0].max(e[1])];
                let slot = drawn.entry(key).or_insert(0.0);
                *slot = slot.max(load);
            };
            for e in sb.edges() {
                if e.kind == SoftBodyEdgeKind::Structural {
                    let load = match (by_stress, tears) {
                        (false, _) => 0.0,
                        (true, true) => e.stress() as f32,
                        (true, false) => stretch(e.vertices[0], e.vertices[1]),
                    };
                    record(&mut self.drawn_edges, e.vertices, load);
                }
            }
            for c in sb.cells() {
                for a in 0..DIM + 1 {
                    for b in a + 1..DIM + 1 {
                        let edge = [c.vertices[a], c.vertices[b]];
                        let load = match (by_stress, tears) {
                            (false, _) => 0.0,
                            (true, true) => c.stress() as f32,
                            (true, false) => stretch(edge[0], edge[1]),
                        };
                        record(&mut self.drawn_edges, edge, load);
                    }
                }
            }
            // Sorted: the backend sees the same lines in the same order every frame.
            let mut edges: Vec<([u32; 2], f32)> =
                self.drawn_edges.iter().map(|(k, v)| (*k, *v)).collect();
            edges.sort_unstable_by_key(|e| e.0);
            for (key, load) in edges {
                let color = if by_stress {
                    let t = load.clamp(0.0, 1.0);
                    let (slack, loaded) = (
                        self.style.soft_body_slack_color,
                        self.style.soft_body_loaded_color,
                    );
                    core::array::from_fn(|k| slack[k] + (loaded[k] - slack[k]) * t)
                } else {
                    element_color
                };
                backend.draw_line(
                    object,
                    sb.particle_position(key[0] as usize),
                    sb.particle_position(key[1] as usize),
                    color,
                );
            }
            // Same reading as the rigid contacts: the depth segment joins the two witness
            // points, and the normal starts on the surface side. A contact sitting exactly on
            // the surface has no direction to show, so it gets the segment alone.
            let depth_color = self.style.contact_depth_color;
            let normal_color = self.style.contact_normal_color;
            let normal_length = self.style.contact_normal_length;
            for (a, b) in sb
                .edge_contact_segments(soft_bodies)
                .chain(sb.vertex_contact_segments(soft_bodies))
            {
                backend.draw_line(object, a, b, depth_color);
                if let Some(n) = (a - b).try_normalize() {
                    backend.draw_line(object, b, b + n * normal_length, normal_color);
                }
            }
        }
    }

    /// Renders the soft bodies' volume constraints (`SoftVolumeContact`): each one's normal at its
    /// patch center (owner body toward the other side) and the volume gradient at every particle it
    /// acts on, scaled so the largest has the normal's length (the push goes the opposite way).
    #[profiling::function]
    pub fn render_soft_volume_contacts<B: DebugRenderBackend>(
        &mut self,
        backend: &mut B,
        soft_bodies: &SoftBodySet,
    ) {
        let normal_color = self.style.volume_contact_normal_color;
        let gradient_color = self.style.volume_gradient_color;
        let length = self.style.contact_normal_length;
        for (handle, sb) in soft_bodies.iter() {
            let object = DebugRenderObject::SoftBody(handle, sb);
            if !backend.filter_object(object) {
                continue;
            }
            for c in sb.volume_contacts() {
                backend.draw_line(object, c.center, c.center + c.normal * length, normal_color);
                let g_max = c
                    .gradients
                    .iter()
                    .map(|(_, g)| g.length())
                    .fold(0.0, crate::math::Real::max);
                if g_max <= 0.0 {
                    continue;
                }
                for (p, g) in &c.gradients {
                    backend.draw_line(object, *p, *p + *g * (length / g_max), gradient_color);
                }
            }
        }
    }

    /// Render contact.
    #[profiling::function]
    pub fn render_contacts(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
    ) {
        if self.mode.contains(DebugRenderMode::CONTACTS) {
            for pair in narrow_phase.contact_pairs() {
                if let (Some(co1), Some(co2)) =
                    (colliders.get(pair.collider1), colliders.get(pair.collider2))
                {
                    let object = DebugRenderObject::ContactPair(pair, co1, co2);

                    if backend.filter_object(object) {
                        for manifold in pair.manifolds() {
                            for contact in manifold.contacts() {
                                let world_subshape_pos1 =
                                    manifold.subshape_pos1().prepend_to(co1.position());
                                backend.draw_line(
                                    object,
                                    world_subshape_pos1 * contact.local_p1,
                                    manifold.subshape_pos2().prepend_to(co2.position())
                                        * contact.local_p2,
                                    self.style.contact_depth_color,
                                );
                                backend.draw_line(
                                    object,
                                    world_subshape_pos1 * contact.local_p1,
                                    world_subshape_pos1
                                        * (contact.local_p1
                                            + manifold.local_n1 * self.style.contact_normal_length),
                                    self.style.contact_normal_color,
                                );
                            }
                        }
                    }
                }
            }
        }

        if self.mode.contains(DebugRenderMode::SOLVER_CONTACTS) {
            for pair in narrow_phase.contact_pairs() {
                if let (Some(co1), Some(co2)) =
                    (colliders.get(pair.collider1), colliders.get(pair.collider2))
                {
                    let object = DebugRenderObject::ContactPair(pair, co1, co2);

                    if backend.filter_object(object) {
                        for manifold in pair.manifolds() {
                            let world_pos1 = manifold.subshape_pos1().prepend_to(co1.position());
                            let world_pos2 = manifold.subshape_pos2().prepend_to(co2.position());
                            for contact in &manifold.data.solver_contacts {
                                // Solver contacts store body-local anchors; without
                                // the rigid-body set at hand, resolve the world
                                // point through the matching manifold point (equal
                                // up to the contact-skin shift and hook edits).
                                let cid = (contact.contact_id[0]
                                    & !crate::geometry::NEW_CONTACT_BIT)
                                    as usize;
                                let Some(pt) = manifold.points.get(cid) else {
                                    continue;
                                };
                                let point =
                                    (world_pos1 * pt.local_p1).midpoint(world_pos2 * pt.local_p2);
                                backend.draw_line(
                                    object,
                                    point,
                                    point + manifold.data.normal * self.style.contact_normal_length,
                                    self.style.contact_normal_color,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    /// Render only the joints from the scene.
    #[profiling::function]
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
                                mut anchor_color: DebugColor,
                                mut separation_color: DebugColor,
                                object| {
            if !backend.filter_object(object) {
                return;
            }

            if let (Some(rb1), Some(rb2)) = (bodies.get(body1), bodies.get(body2)) {
                let settled = |rb: &RigidBody| rb.is_fixed() || rb.is_sleeping();
                let eligible =
                    |rb: &RigidBody| settled(rb) || rb.activation().is_eligible_for_sleep();
                let coeff = if !data.is_enabled() || !rb1.is_enabled() || !rb2.is_enabled() {
                    self.style.disabled_color_multiplier
                } else if settled(rb1) && settled(rb2) {
                    self.style.sleep_color_multiplier
                } else if eligible(rb1) && eligible(rb2) {
                    self.style.sleep_eligible_color_multiplier
                } else {
                    [1.0; 4]
                };

                let frame1 = rb1.position() * data.local_frame1;
                let frame2 = rb2.position() * data.local_frame2;

                let a = rb1.translation();
                let b = frame1.translation;
                let c = frame2.translation;
                let d = rb2.translation();

                for k in 0..4 {
                    anchor_color[k] *= coeff[k];
                    separation_color[k] *= coeff[k];
                }

                backend.draw_line(object, a, b, anchor_color);
                backend.draw_line(object, b, c, separation_color);
                backend.draw_line(object, c, d, anchor_color);
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
            for (handle, _, multibody, link) in multibody_joints.iter() {
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

    /// Render only the rigid-bodies from the scene.
    #[profiling::function]
    pub fn render_rigid_bodies(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
    ) {
        for (handle, rb) in bodies.iter() {
            let object = DebugRenderObject::RigidBody(handle, rb);

            if self.style.rigid_body_axes_length != 0.0
                && self.mode.contains(DebugRenderMode::RIGID_BODY_AXES)
                && backend.filter_object(object)
            {
                #[cfg(feature = "dim2")]
                let basis = Matrix::from_angle(rb.rotation().angle());
                #[cfg(feature = "dim3")]
                let basis = Matrix::from_quat(*rb.rotation());
                let coeff = self.body_color_multiplier(rb, true, false);
                let colors = [
                    [0.0 * coeff[0], 1.0 * coeff[1], 0.25 * coeff[2], coeff[3]],
                    [120.0 * coeff[0], 1.0 * coeff[1], 0.1 * coeff[2], coeff[3]],
                    [240.0 * coeff[0], 1.0 * coeff[1], 0.2 * coeff[2], coeff[3]],
                ];

                let com = rb.position() * rb.mprops.local_mprops.local_com;

                for k in 0..DIM {
                    let axis = basis.col(k) * self.style.rigid_body_axes_length;
                    backend.draw_line(object, com, com + axis, colors[k]);
                }
            }
        }
    }

    /// Render only the colliders from the scene.
    #[profiling::function]
    pub fn render_colliders(
        &mut self,
        backend: &mut impl DebugRenderBackend,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        if self.mode.contains(DebugRenderMode::COLLIDER_SHAPES) {
            for (h, co) in colliders.iter() {
                let object = DebugRenderObject::Collider(h, co);

                if backend.filter_object(object) {
                    let color = if let Some(parent) = co.parent().and_then(|p| bodies.get(p)) {
                        let coeff = self.body_color_multiplier(parent, co.is_enabled(), true);
                        let c = match parent.body_type {
                            RigidBodyType::Fixed => self.style.collider_fixed_color,
                            RigidBodyType::Dynamic | RigidBodyType::SoftFrame => {
                                self.style.collider_dynamic_color
                            }
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
                    } else if !co.is_enabled() {
                        self.style.disabled_color_multiplier
                    } else {
                        self.style.collider_parentless_color
                    };

                    self.render_shape(object, backend, co.shape(), co.position(), color)
                }
            }
        }

        if self.mode.contains(DebugRenderMode::COLLIDER_AABBS) {
            for (h, co) in colliders.iter() {
                let aabb = co.compute_aabb();
                let cuboid = Cuboid::new(aabb.half_extents());
                let object = DebugRenderObject::ColliderAabb(h, co, &aabb);

                if backend.filter_object(object) {
                    self.render_shape(
                        object,
                        backend,
                        &cuboid,
                        &Pose::from_translation(aabb.center()),
                        self.style.collider_aabb_color,
                    );
                }
            }
        }

        if self.mode.contains(DebugRenderMode::PSEUDO_NORMALS) {
            for (h, co) in colliders.iter() {
                let object = DebugRenderObject::Collider(h, co);

                if backend.filter_object(object) {
                    self.render_shape_pseudo_normals(object, backend, co.shape(), co.position());
                }
            }
        }
    }

    /// Renders the pseudo-normals of the polylines reachable from `shape`, if computed (polyline
    /// `ORIENTED`), each starting at its vertex.
    #[cfg(feature = "dim2")]
    #[profiling::function]
    fn render_shape_pseudo_normals(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Pose,
    ) {
        let len = self.style.pseudo_normal_length;

        match shape.as_typed_shape() {
            TypedShape::Polyline(s) => {
                let Some(pseudo_normals) = s.pseudo_normals() else {
                    return;
                };

                for (vtx, n) in s.vertices().iter().zip(pseudo_normals) {
                    let Some(n) = n.try_normalize() else {
                        continue;
                    };
                    let a = *pos * *vtx;
                    backend.draw_line(
                        object,
                        a,
                        a + pos.rotation * n * len,
                        self.style.vertex_pseudo_normal_color,
                    );
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    self.render_shape_pseudo_normals(object, backend, &**shape, &(pos * sub_pos))
                }
            }
            _ => {}
        }
    }

    /// Renders the pseudo-normals of the triangle meshes reachable from `shape`, if computed (mesh
    /// `ORIENTED` or `FIX_INTERNAL_EDGES`), each from its feature: a vertex, or an edge's midpoint.
    #[cfg(feature = "dim3")]
    #[profiling::function]
    fn render_shape_pseudo_normals(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Pose,
    ) {
        let len = self.style.pseudo_normal_length;

        match shape.as_typed_shape() {
            TypedShape::TriMesh(s) => {
                let Some(pseudo_normals) = s.pseudo_normals() else {
                    return;
                };
                let vertices = s.vertices();

                for (vtx, n) in vertices.iter().zip(&pseudo_normals.vertices_pseudo_normal) {
                    let Some(n) = n.try_normalize() else {
                        continue;
                    };
                    let a = *pos * *vtx;
                    backend.draw_line(
                        object,
                        a,
                        a + pos.rotation * n * len,
                        self.style.vertex_pseudo_normal_color,
                    );
                }

                // The per-triangle pseudo-normals are stored in the edge order [ab, bc, ca], as
                // built by `TriMesh::compute_pseudo_normals`.
                for (idx, normals) in s.indices().iter().zip(&pseudo_normals.edges_pseudo_normal) {
                    for (k, n) in normals.iter().enumerate() {
                        let Some(n) = n.try_normalize() else {
                            continue;
                        };
                        let v0 = vertices[idx[k] as usize];
                        let v1 = vertices[idx[(k + 1) % 3] as usize];
                        let a = *pos * ((v0 + v1) * 0.5);
                        backend.draw_line(
                            object,
                            a,
                            a + pos.rotation * n * len,
                            self.style.edge_pseudo_normal_color,
                        );
                    }
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    self.render_shape_pseudo_normals(object, backend, &**shape, &(pos * sub_pos))
                }
            }
            _ => {}
        }
    }

    #[cfg(feature = "dim2")]
    #[profiling::function]
    fn render_shape(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Pose,
        color: DebugColor,
    ) {
        match shape.as_typed_shape() {
            TypedShape::Ball(s) => {
                let vtx = &self.instances[&TypeId::of::<Ball>()];
                backend.draw_line_strip(
                    object,
                    vtx,
                    pos,
                    Vector::splat(s.radius * 2.0),
                    color,
                    true,
                );
                // Draw a radius line to visualize rotation
                backend.draw_line(
                    object,
                    pos * Vector::new(s.radius * 0.2, 0.0),
                    pos * Vector::new(s.radius * 0.8, 0.0),
                    color,
                )
            }
            TypedShape::Cuboid(s) => {
                let vtx = &self.instances[&TypeId::of::<Cuboid>()];
                backend.draw_line_strip(object, vtx, pos, s.half_extents * 2.0, color, true)
            }
            TypedShape::Capsule(s) => {
                let vtx = s.to_polyline(self.style.subdivisions);
                backend.draw_line_strip(object, &vtx, pos, Vector::splat(1.0), color, true)
            }
            TypedShape::Segment(s) => {
                backend.draw_line_strip(object, &[s.a, s.b], pos, Vector::splat(1.0), color, false)
            }
            TypedShape::Triangle(s) => backend.draw_line_strip(
                object,
                &[s.a, s.b, s.c],
                pos,
                Vector::splat(1.0),
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
                Vector::splat(1.0),
                color,
            ),
            TypedShape::HalfSpace(s) => {
                let basis = s.normal.orthonormal_basis()[0];
                let a = Vector::from(basis) * 10_000.0;
                let b = Vector::from(basis) * -10_000.0;
                backend.draw_line_strip(object, &[a, b], pos, Vector::splat(1.0), color, false)
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
                backend.draw_line_strip(object, s.points(), pos, Vector::splat(1.0), color, true)
            }
            /*
             * Round shapes.
             */
            TypedShape::RoundCuboid(s) => {
                let vtx = s.to_polyline(self.style.border_subdivisions);
                backend.draw_line_strip(object, &vtx, pos, Vector::splat(1.0), color, true)
            }
            TypedShape::RoundTriangle(s) => {
                // Parry doesn’t implement `to_polyline` for `RoundTriangle`, so convert it
                // to a round convex polygon which discretizes to the same rounded boundary.
                let tri = &s.inner_shape;
                if let Some(inner_shape) = ConvexPolygon::from_convex_hull(&[tri.a, tri.b, tri.c]) {
                    let poly = RoundShape {
                        inner_shape,
                        border_radius: s.border_radius,
                    };
                    let vtx = poly.to_polyline(self.style.border_subdivisions);
                    backend.draw_line_strip(object, &vtx, pos, Vector::splat(1.0), color, true)
                } else {
                    // Degenerate triangle: render the flat inner shape instead.
                    self.render_shape(object, backend, &s.inner_shape, pos, color)
                }
            }
            // TypedShape::RoundTriMesh(s) => self.render_shape(backend, &s.inner_shape, pos, color),
            // TypedShape::RoundHeightField(s) => {
            //     self.render_shape(backend, &s.inner_shape, pos, color)
            // }
            TypedShape::RoundConvexPolygon(s) => {
                let vtx = s.to_polyline(self.style.border_subdivisions);
                backend.draw_line_strip(object, &vtx, pos, Vector::splat(1.0), color, true)
            }
            TypedShape::Voxels(s) => {
                let (vtx, idx) = s.to_polyline();
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::Custom(_) => {}
        }
    }

    #[cfg(feature = "dim3")]
    #[profiling::function]
    fn render_shape(
        &mut self,
        object: DebugRenderObject,
        backend: &mut impl DebugRenderBackend,
        shape: &dyn Shape,
        pos: &Pose,
        color: DebugColor,
    ) {
        match shape.as_typed_shape() {
            TypedShape::Ball(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Ball>()];
                backend.draw_polyline(object, vtx, idx, pos, Vector::splat(s.radius * 2.0), color)
            }
            TypedShape::Cuboid(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Cuboid>()];
                backend.draw_polyline(object, vtx, idx, pos, s.half_extents * 2.0, color)
            }
            TypedShape::Capsule(s) => {
                let (vtx, idx) = s.to_outline(self.style.subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::Segment(s) => backend.draw_polyline(
                object,
                &[s.a, s.b],
                &[[0, 1]],
                pos,
                Vector::splat(1.0),
                color,
            ),
            TypedShape::Triangle(s) => backend.draw_line_strip(
                object,
                &[s.a, s.b, s.c],
                pos,
                Vector::splat(1.0),
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
                Vector::splat(1.0),
                color,
            ),
            TypedShape::HalfSpace(s) => {
                let basis = s.normal.orthonormal_basis();
                let a = Vector::from(basis[0]) * 10_000.0;
                let b = Vector::from(basis[0]) * -10_000.0;
                let c = Vector::from(basis[1]) * 10_000.0;
                let d = Vector::from(basis[1]) * -10_000.0;
                backend.draw_polyline(
                    object,
                    &[a, b, c, d],
                    &[[0, 1], [2, 3]],
                    pos,
                    Vector::splat(1.0),
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
                    .map(|e| [e.vertices[0], e.vertices[1]])
                    .collect();
                backend.draw_polyline(object, s.points(), &indices, pos, Vector::splat(1.0), color)
            }
            TypedShape::Cylinder(s) => {
                let (vtx, idx) = &self.instances[&TypeId::of::<Cylinder>()];
                backend.draw_polyline(
                    object,
                    vtx,
                    idx,
                    pos,
                    Vector::new(s.radius, s.half_height, s.radius) * 2.0,
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
                    Vector::new(s.radius, s.half_height, s.radius) * 2.0,
                    color,
                )
            }
            /*
             * Round shapes.
             */
            TypedShape::RoundCuboid(s) => {
                let (vtx, idx) = s.to_outline(self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::RoundTriangle(s) => {
                // Parry doesn’t implement `to_outline` for `RoundTriangle` in 3D
                // (the impl is currently disabled upstream), so fall back to
                // rendering the flat inner triangle.
                self.render_shape(object, backend, &s.inner_shape, pos, color)
            }
            // TypedShape::RoundTriMesh(s) => self.render_shape(object, backend, &s.inner_shape, pos, color),
            // TypedShape::RoundHeightField(s) => {
            //     self.render_shape(object, backend, &s.inner_shape, pos, color)
            // }
            TypedShape::RoundCylinder(s) => {
                let (vtx, idx) =
                    s.to_outline(self.style.subdivisions, self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::RoundCone(s) => {
                let (vtx, idx) =
                    s.to_outline(self.style.subdivisions, self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::RoundConvexPolyhedron(s) => {
                let (vtx, idx) = s.to_outline(self.style.border_subdivisions);
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::Voxels(s) => {
                let (vtx, idx) = s.to_outline();
                backend.draw_polyline(object, &vtx, &idx, pos, Vector::splat(1.0), color)
            }
            TypedShape::Custom(_) => {}
        }
    }
}
