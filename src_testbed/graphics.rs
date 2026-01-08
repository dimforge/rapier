#![allow(clippy::unnecessary_cast, clippy::useless_conversion)] // Casts/conversions are needed for switching between f32/f64.

use crate::testbed::TestbedStateFlags;
use kiss3d::prelude::*;
use rand_pcg::Pcg32;
use rapier::dynamics::{RigidBodyHandle, RigidBodySet};
use rapier::geometry::{ColliderHandle, ColliderSet, Shape, ShapeType};
use std::collections::HashMap;

#[cfg(feature = "dim2")]
pub use kiss3d::prelude::SceneNode2d as SceneNode;
#[cfg(feature = "dim3")]
pub use kiss3d::prelude::SceneNode3d as SceneNode;

/// Template key identifies a shape type that can be instanced together
#[derive(Hash, Eq, PartialEq, Clone, Debug)]
pub enum ShapeTemplateType {
    Ball,
    Cuboid,
    // Capsule,
    #[cfg(feature = "dim3")]
    Cylinder,
    #[cfg(feature = "dim3")]
    Cone,
}

/// Info about an instanced collider
#[derive(Clone)]
pub struct InstancedCollider {
    pub collider: ColliderHandle,
    pub body: Option<RigidBodyHandle>,
    pub delta: rapier::math::Pose,
    pub half_extents: rapier::math::Vector,
    pub color: Color,
    pub tmp_color: Option<Color>,
}

/// A template node that can render multiple instances
pub struct ShapeTemplate {
    pub node: SceneNode,
    pub colliders: Vec<InstancedCollider>,
}

/// Non-instanced node for complex shapes
pub struct IndividualNode {
    pub node: SceneNode,
    pub collider: ColliderHandle,
    pub delta: rapier::math::Pose,
    pub color: Color,
    pub tmp_color: Option<Color>,
}

/// Location of a collider in the graphics system
#[derive(Clone, Debug)]
pub enum NodeLocation {
    Instanced {
        template: ShapeTemplateType,
        index: usize,
    },
    Individual {
        index: usize,
    },
}

pub struct GraphicsManager {
    scene: SceneNode,
    rand: Pcg32,
    /// Template nodes for instanced primitives
    templates: HashMap<ShapeTemplateType, ShapeTemplate>,
    /// Individual nodes for complex shapes (trimesh, heightfield, etc.)
    individual_nodes: Vec<IndividualNode>,
    /// Map from collider to its render nodes
    c2nodes: HashMap<ColliderHandle, NodeLocation>,
    /// Colliders attached to a particular body, used to identify the
    /// body’s nodes even if it has been removed from the RigidBodySet.
    b2colliders: HashMap<RigidBodyHandle, Vec<ColliderHandle>>,
    /// Colors per body
    b2color: HashMap<RigidBodyHandle, Color>,
    /// Colors per collider (overrides body color)
    c2color: HashMap<ColliderHandle, Color>,
    /// Wireframe mode per body
    b2wireframe: HashMap<RigidBodyHandle, bool>,
    /// Default color for fixed/ground bodies
    ground_color: Color,
    /// Graphics offset
    pub gfx_shift: rapier::math::Vector,
    /// Global wireframe mode
    wireframe_mode: bool,
}

const GROUND_COLOR: Color = Color {
    r: 0.58,
    g: 0.54,
    b: 0.50,
    a: 1.0,
};

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        GraphicsManager {
            scene: Default::default(),
            rand: Pcg32::new(0, 1),
            templates: HashMap::new(),
            individual_nodes: Vec::new(),
            c2nodes: HashMap::new(),
            b2colliders: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            b2wireframe: HashMap::new(),
            ground_color: GROUND_COLOR,
            gfx_shift: rapier::math::Vector::ZERO,
            wireframe_mode: false,
        }
    }

    pub fn clear(&mut self) {
        self.scene = SceneNode::empty();
        #[cfg(feature = "dim3")]
        {
            // Setup lights.
            let mut light = self
                .scene
                .add_light(Light::directional(Vec3::new(-1.0, -1.0, -1.0)));
            light.set_position(Vec3::new(100.0, 100.0, 100.0));

            let mut light = self
                .scene
                .add_light(Light::point(10000.0).with_intensity(1.0));
            light.set_position(Vec3::new(-100.0, 100.0, -100.0));
        }

        self.templates.clear();
        self.individual_nodes.clear();
        self.c2nodes.clear();
        self.b2colliders.clear();
        self.c2color.clear();
        self.b2color.clear();
        self.b2wireframe.clear();
        self.rand = Pcg32::new(0, 1);
    }

    pub fn scene(&self) -> &SceneNode {
        &self.scene
    }

    pub fn scene_mut(&mut self) -> &mut SceneNode {
        &mut self.scene
    }

    /// Get or create a template node for a shape type
    #[cfg(feature = "dim3")]
    fn get_or_create_template(&mut self, template_type: ShapeTemplateType) -> &mut ShapeTemplate {
        self.templates
            .entry(template_type.clone())
            .or_insert_with(|| {
                // Create a unit-sized template node
                let node = match template_type {
                    ShapeTemplateType::Ball => self.scene.add_sphere_with_subdiv(1.0, 20, 20),
                    ShapeTemplateType::Cuboid => self.scene.add_cube(2.0, 2.0, 2.0),
                    // ShapeTemplateType::Capsule => self.scene.add_capsule(1.0, 2.0),
                    ShapeTemplateType::Cylinder => self.scene.add_cylinder(1.0, 2.0),
                    ShapeTemplateType::Cone => self.scene.add_cone(1.0, 2.0),
                };
                ShapeTemplate {
                    node,
                    colliders: Vec::new(),
                }
            })
    }

    #[cfg(feature = "dim2")]
    fn get_or_create_template(&mut self, template_type: ShapeTemplateType) -> &mut ShapeTemplate {
        self.templates
            .entry(template_type.clone())
            .or_insert_with(|| {
                // Create a unit-sized template node
                let node = match template_type {
                    ShapeTemplateType::Ball => self.scene.add_circle(1.0),
                    ShapeTemplateType::Cuboid => self.scene.add_rectangle(2.0, 2.0),
                    // ShapeTemplateType::Capsule => {
                    //     // Use proper 2D capsule with unit radius and unit half-height
                    //     window.add_capsule_2d(1.0, 2.0)
                    // }
                };
                ShapeTemplate {
                    node,
                    colliders: Vec::new(),
                }
            })
    }

    /// Determine the template type for a shape, if it can be instanced
    fn shape_template_type(shape: &dyn Shape) -> Option<ShapeTemplateType> {
        match shape.shape_type() {
            ShapeType::Ball => Some(ShapeTemplateType::Ball),
            ShapeType::Cuboid | ShapeType::RoundCuboid => Some(ShapeTemplateType::Cuboid),
            // ShapeType::Capsule => Some(ShapeTemplateType::Capsule),
            #[cfg(feature = "dim3")]
            ShapeType::Cylinder | ShapeType::RoundCylinder => Some(ShapeTemplateType::Cylinder),
            #[cfg(feature = "dim3")]
            ShapeType::Cone | ShapeType::RoundCone => Some(ShapeTemplateType::Cone),
            _ => None,
        }
    }

    /// Get half-extents for a shape (used for scaling instances)
    #[cfg(feature = "dim3")]
    fn shape_half_extents(shape: &dyn Shape) -> rapier::math::Vector {
        match shape.shape_type() {
            ShapeType::Ball => {
                let r = shape.as_ball().unwrap().radius;
                rapier::math::Vector::new(r, r, r)
            }
            ShapeType::Cuboid => shape.as_cuboid().unwrap().half_extents,
            ShapeType::RoundCuboid => shape.as_round_cuboid().unwrap().inner_shape.half_extents,
            ShapeType::Capsule => {
                let c = shape.as_capsule().unwrap();
                rapier::math::Vector::new(c.radius, c.half_height() + c.radius, c.radius)
            }
            ShapeType::Cylinder => {
                let c = shape.as_cylinder().unwrap();
                rapier::math::Vector::new(c.radius, c.half_height, c.radius)
            }
            ShapeType::RoundCylinder => {
                let c = &shape.as_round_cylinder().unwrap().inner_shape;
                rapier::math::Vector::new(c.radius, c.half_height, c.radius)
            }
            ShapeType::Cone => {
                let c = shape.as_cone().unwrap();
                rapier::math::Vector::new(c.radius, c.half_height, c.radius)
            }
            ShapeType::RoundCone => {
                let c = &shape.as_round_cone().unwrap().inner_shape;
                rapier::math::Vector::new(c.radius, c.half_height, c.radius)
            }
            _ => rapier::math::Vector::new(1.0, 1.0, 1.0),
        }
    }

    #[cfg(feature = "dim2")]
    fn shape_half_extents(shape: &dyn Shape) -> rapier::math::Vector {
        match shape.shape_type() {
            ShapeType::Ball => {
                let r = shape.as_ball().unwrap().radius;
                rapier::math::Vector::new(r, r)
            }
            ShapeType::Cuboid => shape.as_cuboid().unwrap().half_extents,
            ShapeType::RoundCuboid => shape.as_round_cuboid().unwrap().inner_shape.half_extents,
            ShapeType::Capsule => {
                let c = shape.as_capsule().unwrap();
                rapier::math::Vector::new(c.radius, c.half_height() + c.radius)
            }
            _ => rapier::math::Vector::new(1.0, 1.0),
        }
    }

    fn remove_node(&mut self, location: NodeLocation) {
        match location {
            NodeLocation::Instanced { template, index } => {
                if let Some(tmpl) = self.templates.get_mut(&template) {
                    // Remove by swapping with last (O(1) removal)
                    if index < tmpl.colliders.len() {
                        tmpl.colliders.swap_remove(index);
                        // Update the location of the swapped element
                        if index < tmpl.colliders.len() {
                            let swapped_collider = tmpl.colliders[index].collider;
                            self.c2nodes.insert(
                                swapped_collider,
                                NodeLocation::Instanced {
                                    template: template.clone(),
                                    index,
                                },
                            );
                        }
                    }
                }
            }
            NodeLocation::Individual { index } => {
                if index < self.individual_nodes.len() {
                    self.individual_nodes[index].node.detach();
                    self.individual_nodes.swap_remove(index);
                    // Update the location of the swapped element
                    if index < self.individual_nodes.len() {
                        let swapped_collider = self.individual_nodes[index].collider;
                        self.c2nodes
                            .insert(swapped_collider, NodeLocation::Individual { index });
                    }
                }
            }
        }
    }

    pub fn remove_collider_nodes(&mut self, collider: ColliderHandle) {
        if let Some(location) = self.c2nodes.remove(&collider) {
            self.remove_node(location);
        }
    }

    pub fn remove_body_nodes(&mut self, body: RigidBodyHandle) {
        if let Some(colliders) = self.b2colliders.get(&body).cloned() {
            for collider in colliders {
                self.remove_collider_nodes(collider);
            }
        }
    }

    pub fn set_body_color(&mut self, b: RigidBodyHandle, color: Color, tmp_color: bool) {
        if !tmp_color {
            self.b2color.insert(b, color);
        }

        if let Some(colls) = self.b2colliders.get(&b) {
            for co in colls.iter() {
                match &self.c2nodes[co] {
                    NodeLocation::Individual { index } => {
                        if tmp_color {
                            self.individual_nodes[*index].tmp_color = Some(color);
                        } else {
                            self.individual_nodes[*index].color = color;
                        }
                    }
                    NodeLocation::Instanced { template, index } => {
                        let instances = &mut self.templates.get_mut(template).unwrap();
                        if tmp_color {
                            instances.colliders[*index].tmp_color = Some(color);
                        } else {
                            instances.colliders[*index].color = color;
                        }
                    }
                }
            }
        }
    }

    pub fn set_initial_body_color(&mut self, b: RigidBodyHandle, color: Color) {
        self.b2color.insert(b, color);
    }

    pub fn set_initial_collider_color(&mut self, c: ColliderHandle, color: Color) {
        self.c2color.insert(c, color);
    }

    pub fn set_body_wireframe(&mut self, b: RigidBodyHandle, enabled: bool) {
        self.b2wireframe.insert(b, enabled);
    }

    pub fn toggle_wireframe_mode(&mut self, _colliders: &ColliderSet, enabled: bool) {
        self.wireframe_mode = enabled;

        // // Update template nodes
        // for tmpl in self.templates.values_mut() {
        //     if enabled {
        //         tmpl.node.set_lines_width(1.0, false);
        //         tmpl.node.set_surface_rendering_activation(false);
        //     } else {
        //         tmpl.node.set_lines_width(0.0, false);
        //         tmpl.node.set_surface_rendering_activation(true);
        //     }
        // }
        //
        // // Update individual nodes
        // for node in &mut self.individual_nodes {
        //     if enabled {
        //         node.node.set_lines_width(1.0, false);
        //         node.node.set_surface_rendering_activation(false);
        //     } else {
        //         node.node.set_lines_width(0.0, false);
        //         node.node.set_surface_rendering_activation(true);
        //     }
        // }
    }

    pub fn next_color(&mut self) -> Rgba<f32> {
        Self::gen_color(&mut self.rand)
    }

    fn gen_color(rng: &mut Pcg32) -> Rgba<f32> {
        use rand::Rng;
        let [r, g, b]: [f32; 3] = rng.random();
        [r, g, b, 1.0].into()
    }

    fn alloc_color(&mut self, handle: RigidBodyHandle, is_fixed: bool) -> Color {
        let mut color = self.ground_color;

        if !is_fixed {
            match self.b2color.get(&handle).cloned() {
                Some(c) => color = c,
                None => color = Self::gen_color(&mut self.rand),
            }
        }

        self.b2color.insert(handle, color);
        color
    }

    pub fn add_body_colliders(
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

        self.add_body_colliders_with_color(window, handle, bodies, colliders, color);
    }

    pub fn add_body_colliders_with_color(
        &mut self,
        window: &mut Window,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        color: Color,
    ) {
        for collider_handle in bodies[handle].colliders() {
            let color = self.c2color.get(collider_handle).copied().unwrap_or(color);
            let collider = &colliders[*collider_handle];
            self.add_shape(
                window,
                *collider_handle,
                Some(handle),
                collider.shape(),
                collider.is_sensor(),
                rapier::math::Pose::IDENTITY,
                color,
            );
        }
    }

    pub fn add_collider(
        &mut self,
        window: &mut Window,
        handle: ColliderHandle,
        colliders: &ColliderSet,
    ) {
        let collider = &colliders[handle];
        let collider_parent = collider.parent();
        let color = collider_parent
            .and_then(|p| self.b2color.get(&p).copied())
            .unwrap_or(self.ground_color);
        let color = self.c2color.get(&handle).copied().unwrap_or(color);
        self.add_shape(
            window,
            handle,
            collider_parent,
            collider.shape(),
            collider.is_sensor(),
            rapier::math::Pose::IDENTITY,
            color,
        );
    }

    pub fn add_shape(
        &mut self,
        _window: &mut Window,
        handle: ColliderHandle,
        body: Option<RigidBodyHandle>,
        shape: &dyn Shape,
        sensor: bool,
        delta: rapier::math::Pose,
        color: Color,
    ) {
        if let Some(body) = body {
            let colls = self.b2colliders.entry(body).or_default();
            if !colls.contains(&handle) {
                colls.push(handle);
            }
        }

        // Handle compound shapes recursively
        if let Some(compound) = shape.as_compound() {
            for (shape_pos, sub_shape) in compound.shapes() {
                self.add_shape(
                    _window,
                    handle,
                    body,
                    &**sub_shape,
                    sensor,
                    *shape_pos * delta,
                    color,
                );
            }
            return;
        }

        let opacity = if sensor { 0.5 } else { 1.0 };

        // Try to use instancing for primitive shapes
        if let Some(template_type) = Self::shape_template_type(shape) {
            let half_extents = Self::shape_half_extents(shape);
            let template = self.get_or_create_template(template_type.clone());
            let index = template.colliders.len();
            template.colliders.push(InstancedCollider {
                collider: handle,
                body,
                delta,
                half_extents,
                color: color.with_alpha(opacity),
                tmp_color: None,
            });
            self.c2nodes.insert(
                handle,
                NodeLocation::Instanced {
                    template: template_type,
                    index,
                },
            );
        } else {
            // Create individual node for complex shapes
            if let Some(node) = Self::create_individual_node(&mut self.scene, shape, color, sensor)
            {
                let index = self.individual_nodes.len();
                self.individual_nodes.push(IndividualNode {
                    node,
                    collider: handle,
                    delta,
                    color: color.with_alpha(opacity),
                    tmp_color: None,
                });
                self.c2nodes
                    .insert(handle, NodeLocation::Individual { index });
            }
        }
    }

    #[cfg(feature = "dim3")]
    fn create_individual_node(
        scene: &mut SceneNode,
        shape: &dyn Shape,
        color: Color,
        sensor: bool,
    ) -> Option<SceneNode3d> {
        use kiss3d::procedural::{IndexBuffer, RenderMesh};

        fn to_render_mesh(trimesh: &rapier::geometry::TriMesh) -> RenderMesh {
            let vtx = trimesh
                .vertices()
                .iter()
                .map(|pt| Vec3::new(pt.x as f32, pt.y as f32, pt.z as f32))
                .collect();
            let idx = trimesh.indices().to_vec();
            let mut mesh = RenderMesh::new(vtx, None, None, Some(IndexBuffer::Unified(idx)));
            mesh.replicate_vertices();
            mesh.recompute_normals();
            mesh
        }

        let mut node = match shape.shape_type() {
            ShapeType::TriMesh => {
                let trimesh = shape.as_trimesh().unwrap();
                Some(scene.add_render_mesh(to_render_mesh(trimesh), Vec3::ONE))
            }
            ShapeType::HeightField => {
                let heightfield = shape.as_heightfield().unwrap();
                let (vertices, indices) = heightfield.to_trimesh();
                let trimesh = rapier::geometry::TriMesh::new(vertices, indices).unwrap();
                Some(scene.add_render_mesh(to_render_mesh(&trimesh), Vec3::ONE))
            }
            ShapeType::ConvexPolyhedron | ShapeType::RoundConvexPolyhedron => {
                let poly = shape
                    .as_convex_polyhedron()
                    .or_else(|| shape.as_round_convex_polyhedron().map(|rp| &rp.inner_shape));
                poly.map(|p| {
                    let (vertices, indices) = p.to_trimesh();
                    let trimesh = rapier::geometry::TriMesh::new(vertices, indices).unwrap();
                    scene.add_render_mesh(to_render_mesh(&trimesh), Vec3::ONE)
                })
            }
            ShapeType::Capsule => {
                let caps = shape.as_capsule().unwrap();
                let pose = caps.canonical_transform();
                let mut parent = scene.add_group();
                let mut node =
                    parent.add_capsule(caps.radius as f32, caps.half_height() as f32 * 2.0);
                node.set_pose(pose.into());
                Some(parent)
            }
            ShapeType::Triangle => {
                let tri = shape.as_triangle().unwrap();
                let vertices = vec![tri.a, tri.b, tri.c];
                let indices = vec![[0u32, 1, 2], [0u32, 2, 1]];
                let trimesh = rapier::geometry::TriMesh::new(vertices, indices).unwrap();
                Some(scene.add_render_mesh(to_render_mesh(&trimesh), Vec3::ONE))
            }
            ShapeType::HalfSpace => {
                let mut parent = scene.add_group();
                parent.rotate(Quat::from_axis_angle(Vec3::X, -std::f32::consts::FRAC_PI_2));
                let node = parent.add_quad(2000.0, 2000.0, 1, 1);
                Some(node)
            }
            ShapeType::Voxels => {
                let voxels = shape.as_voxels().unwrap();
                let (vertices, indices) = voxels.to_trimesh();
                let trimesh = rapier::geometry::TriMesh::new(vertices, indices).unwrap();
                Some(scene.add_render_mesh(to_render_mesh(&trimesh), Vec3::ONE))
            }
            _ => None,
        };

        if let Some(ref mut n) = node {
            n.set_color(color);
            if sensor {
                n.set_surface_rendering_activation(false);
                n.set_lines_width(1.0, false);
            }
        }

        node
    }

    #[cfg(feature = "dim2")]
    fn create_individual_node(
        scene: &mut SceneNode,
        shape: &dyn Shape,
        color: Color,
        _sensor: bool,
    ) -> Option<SceneNode2d> {
        let mut node = match shape.shape_type() {
            ShapeType::Triangle => {
                let tri = shape.as_triangle().unwrap();
                let vertices: Vec<Vec2> = vec![
                    Vec2::new(tri.a.x as f32, tri.a.y as f32),
                    Vec2::new(tri.b.x as f32, tri.b.y as f32),
                    Vec2::new(tri.c.x as f32, tri.c.y as f32),
                ];
                Some(scene.add_convex_polygon(vertices, Vec2::ONE))
            }
            ShapeType::TriMesh => {
                let trimesh = shape.as_trimesh().unwrap();
                let vertices: Vec<Vec2> = trimesh
                    .vertices()
                    .iter()
                    .map(|pt| Vec2::new(pt.x as f32, pt.y as f32))
                    .collect();
                let render_mesh = GpuMesh2d::new(vertices, trimesh.indices().to_vec(), None, false);
                Some(scene.add_mesh(Rc::new(RefCell::new(render_mesh)), Vec2::ONE))
            }
            ShapeType::ConvexPolygon | ShapeType::RoundConvexPolygon => {
                let poly = shape
                    .as_convex_polygon()
                    .or_else(|| shape.as_round_convex_polygon().map(|rp| &rp.inner_shape));
                poly.map(|p| {
                    let vertices: Vec<Vec2> = p
                        .points()
                        .iter()
                        .map(|pt| Vec2::new(pt.x as f32, pt.y as f32))
                        .collect();
                    scene.add_convex_polygon(vertices, Vec2::ONE)
                })
            }
            ShapeType::Capsule => {
                let caps = shape.as_capsule().unwrap();
                let pose = caps.canonical_transform();
                let mut parent = scene.add_group();
                let mut node =
                    parent.add_capsule(caps.radius as f32, caps.half_height() as f32 * 2.0);
                node.set_pose(pose.into());
                Some(parent)
            }
            ShapeType::Segment => {
                // Render segment as a thin quad with some thickness
                let seg = shape.as_segment().unwrap();
                let vertices = vec![
                    Vec2::new(seg.a.x as f32, seg.a.y as f32),
                    Vec2::new(seg.b.x as f32, seg.b.y as f32),
                ];
                Some(scene.add_polyline(vertices, None, 0.1))
            }
            ShapeType::Polyline => {
                // Render polyline as connected thin quads
                let polyline = shape.as_polyline().unwrap();
                let vertices: Vec<Vec2> = polyline
                    .vertices()
                    .iter()
                    .map(|pt| Vec2::new(pt.x as f32, pt.y as f32))
                    .collect();
                Some(
                    scene
                        .add_polyline(vertices, Some(polyline.indices().to_vec()), 0.2)
                        .set_lines_color(Some(GROUND_COLOR)),
                )
            }
            ShapeType::HeightField => {
                let hf = shape.as_heightfield().unwrap();
                let (polyline_verts, indices) = hf.to_polyline();
                let vertices: Vec<Vec2> = polyline_verts
                    .iter()
                    .map(|pt| Vec2::new(pt.x as f32, pt.y as f32))
                    .collect();
                Some(
                    scene
                        .add_polyline(vertices, Some(indices), 0.2)
                        .set_lines_color(Some(GROUND_COLOR)),
                )
            }
            ShapeType::Voxels => {
                let voxels = shape.as_voxels().unwrap();

                let mut vtx = vec![];
                let mut idx = vec![];
                let sz = voxels.voxel_size() / 2.0;
                for vox in voxels.voxels() {
                    if !vox.state.is_empty() {
                        let bid = vtx.len() as u32;
                        let center = Vec2::new(vox.center.x as f32, vox.center.y as f32);
                        vtx.push(center + Vec2::new(sz.x as f32, sz.y as f32));
                        vtx.push(center + Vec2::new(-sz.x as f32, sz.y as f32));
                        vtx.push(center + Vec2::new(-sz.x as f32, -sz.y as f32));
                        vtx.push(center + Vec2::new(sz.x as f32, -sz.y as f32));
                        idx.push([bid, bid + 1, bid + 2]);
                        idx.push([bid + 2, bid + 3, bid]);
                    }
                }

                let mesh = GpuMesh2d::new(vtx, idx, None, false);
                Some(scene.add_mesh(Rc::new(RefCell::new(mesh)), Vec2::ONE))
            }
            _ => None,
        };

        if let Some(ref mut n) = node {
            n.set_color(color);
        }

        node
    }

    #[cfg(feature = "dim3")]
    pub fn draw(
        &mut self,
        flags: TestbedStateFlags,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        let draw_surfaces = flags.contains(TestbedStateFlags::DRAW_SURFACES);

        // Update instance data for all templates
        for (template_type, template) in &mut self.templates {
            template
                .node
                .set_visible(draw_surfaces && !template.colliders.is_empty());

            if template.colliders.is_empty() {
                continue;
            }

            let instances: Vec<InstanceData3d> = template
                .colliders
                .iter_mut()
                .filter_map(|ic| {
                    let co = colliders.get(ic.collider)?;
                    let co_pos = *co.position() * ic.delta;

                    // Get actual color (might have been updated)
                    let bcolor = ic
                        .body
                        .and_then(|b| self.b2color.get(&b).copied())
                        .unwrap_or(ic.color);
                    let ccolor = self
                        .c2color
                        .get(&ic.collider)
                        .copied()
                        .unwrap_or(bcolor)
                        .with_alpha(ic.color.a);
                    let color = ic.tmp_color.take().unwrap_or(ccolor);

                    // Build position
                    let pos = Vec3::new(
                        (co_pos.translation.x + self.gfx_shift.x) as f32,
                        (co_pos.translation.y + self.gfx_shift.y) as f32,
                        (co_pos.translation.z + self.gfx_shift.z) as f32,
                    );

                    // Build deformation matrix (rotation * scale)
                    let rot = Quat::from_xyzw(
                        co_pos.rotation.x as f32,
                        co_pos.rotation.y as f32,
                        co_pos.rotation.z as f32,
                        co_pos.rotation.w as f32,
                    );
                    let rot_mat = Mat3::from_quat(rot);

                    // Scale based on shape type
                    let scale = match template_type {
                        ShapeTemplateType::Ball => {
                            // Ball template is unit radius, scale uniformly
                            Vec3::splat(ic.half_extents.x as f32)
                        }
                        ShapeTemplateType::Cuboid => {
                            // Cuboid template is 2x2x2, scale by half_extents
                            Vec3::new(
                                ic.half_extents.x as f32,
                                ic.half_extents.y as f32,
                                ic.half_extents.z as f32,
                            )
                        }
                        // ShapeTemplateType::Capsule => {
                        //     // Capsule template has radius 1, half_height 1
                        //     // half_extents = (radius, half_height + radius, radius)
                        //     let radius = ic.half_extents.x as f32;
                        //     let half_height = ic.half_extents.y as f32 - radius;
                        //     Vec3::new(radius, (half_height + radius).max(radius), radius)
                        // }
                        ShapeTemplateType::Cylinder => {
                            // Cylinder template has radius 1, half_height 1
                            Vec3::new(
                                ic.half_extents.x as f32,
                                ic.half_extents.y as f32,
                                ic.half_extents.z as f32,
                            )
                        }
                        ShapeTemplateType::Cone => {
                            // Cone template has radius 1, half_height 1
                            Vec3::new(
                                ic.half_extents.x as f32,
                                ic.half_extents.y as f32,
                                ic.half_extents.z as f32,
                            )
                        }
                    };

                    let scale_mat = Mat3::from_diagonal(scale);
                    let deformation = rot_mat * scale_mat;

                    Some(InstanceData3d {
                        position: pos,
                        deformation,
                        color,
                        lines_color: None,
                        lines_width: None,
                        points_color: None,
                        points_size: None,
                    })
                })
                .collect();

            template.node.set_instances(&instances);
        }

        // Update individual nodes
        for node in &mut self.individual_nodes {
            node.node.set_visible(draw_surfaces);

            if let Some(co) = colliders.get(node.collider) {
                let co_pos = *co.position() * node.delta;
                node.node
                    .set_pose(co_pos.append_translation(self.gfx_shift).into());
                node.node
                    .set_color(node.tmp_color.take().unwrap_or(node.color));
            }
        }
    }

    #[cfg(feature = "dim2")]
    pub fn draw(
        &mut self,
        flags: TestbedStateFlags,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        let draw_surfaces = flags.contains(TestbedStateFlags::DRAW_SURFACES);

        // Update instance data for all templates
        for (template_type, template) in &mut self.templates {
            template
                .node
                .set_visible(draw_surfaces && !template.colliders.is_empty());

            if template.colliders.is_empty() {
                continue;
            }

            let instances: Vec<InstanceData2d> = template
                .colliders
                .iter_mut()
                .filter_map(|ic| {
                    let co = colliders.get(ic.collider)?;
                    let co_pos = *co.position() * ic.delta;

                    // Get actual color (might have been updated)
                    let bcolor = ic
                        .body
                        .and_then(|b| self.b2color.get(&b).copied())
                        .unwrap_or(ic.color);
                    let ccolor = self
                        .c2color
                        .get(&ic.collider)
                        .copied()
                        .unwrap_or(bcolor)
                        .with_alpha(ic.color.a);
                    let color = ic.tmp_color.take().unwrap_or(ccolor);

                    // Build position
                    let pos = co_pos.translation + self.gfx_shift;
                    let pos = Vec2::new(pos.x as f32, pos.y as f32);

                    // Build deformation matrix (rotation * scale)
                    let rot_mat = co_pos.rotation.to_mat();
                    let rot_mat = Mat2::from_cols_array(&[
                        rot_mat.x_axis.x as f32,
                        rot_mat.x_axis.y as f32,
                        rot_mat.y_axis.x as f32,
                        rot_mat.y_axis.y as f32,
                    ]);

                    // Scale based on shape type
                    let scale = match template_type {
                        ShapeTemplateType::Ball => {
                            // Ball template is unit radius, scale uniformly
                            Vec2::splat(ic.half_extents.x as f32)
                        }
                        ShapeTemplateType::Cuboid => {
                            // Cuboid template is 2x2, scale by half_extents
                            Vec2::new(ic.half_extents.x as f32, ic.half_extents.y as f32)
                        }
                    };

                    let scale_mat = Mat2::from_diagonal(scale);
                    let deformation = rot_mat * scale_mat;

                    Some(InstanceData2d {
                        position: pos,
                        deformation,
                        color: color.into(),
                        lines_color: None,
                        lines_width: None,
                        points_color: None,
                        points_size: None,
                    })
                })
                .collect();

            template.node.set_instances(&instances);
        }

        // Update individual nodes
        for node in &mut self.individual_nodes {
            node.node.set_visible(draw_surfaces);

            if let Some(co) = colliders.get(node.collider) {
                let co_pos = *co.position() * node.delta;
                node.node.set_position(Vec2::new(
                    (co_pos.translation.x + self.gfx_shift.x) as f32,
                    (co_pos.translation.y + self.gfx_shift.y) as f32,
                ));
                node.node.set_rotation(co_pos.rotation.angle() as f32);
                node.node
                    .set_color(node.tmp_color.take().unwrap_or(node.color));
            }
        }
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
