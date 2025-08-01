use bevy::prelude::*;

use na::{Point3, Point4, point};

use crate::objects::node::EntityWithGraphics;
use rapier::dynamics::{RigidBodyHandle, RigidBodySet};
use rapier::geometry::{ColliderHandle, ColliderSet, Shape, ShapeType};
use rapier::math::{Isometry, Real, Vector};
//use crate::objects::capsule::Capsule;
//#[cfg(feature = "dim3")]
//use crate::objects::mesh::Mesh;
//use crate::objects::plane::Plane;
//#[cfg(feature = "dim2")]
//use crate::objects::polyline::Polyline;
// use crate::objects::mesh::Mesh;
use crate::testbed::TestbedStateFlags;
use rand_pcg::Pcg32;
use std::collections::HashMap;

#[cfg(feature = "dim2")]
pub type BevyMaterial = bevy_sprite::ColorMaterial;
#[cfg(feature = "dim3")]
pub type BevyMaterial = StandardMaterial;
#[cfg(feature = "dim2")]
pub type BevyMaterialComponent = MeshMaterial2d<BevyMaterial>;
#[cfg(feature = "dim3")]
pub type BevyMaterialComponent = MeshMaterial3d<BevyMaterial>;

#[derive(Clone, Default)]
pub struct InstancedMaterials {
    cache: HashMap<Point4<usize>, Handle<BevyMaterial>>,
}

impl InstancedMaterials {
    pub fn insert(
        &mut self,
        materials: &mut Assets<BevyMaterial>,
        color: Point3<f32>,
        opacity: f32,
    ) -> Handle<BevyMaterial> {
        let key = color
            .coords
            .push(opacity)
            .map(|c| (c * 255.0) as usize)
            .into();
        let bevy_color = Color::from(Srgba::new(color.x, color.y, color.z, opacity));

        #[cfg(feature = "dim2")]
        let material = bevy_sprite::ColorMaterial {
            color: bevy_color,
            texture: None,
            ..default()
        };
        #[cfg(feature = "dim3")]
        let material = StandardMaterial {
            metallic: 0.5,
            perceptual_roughness: 0.5,
            double_sided: true, // TODO: this doesn't do anything?
            ..StandardMaterial::from(bevy_color)
        };

        self.cache
            .entry(key)
            .or_insert_with(|| materials.add(material))
            .clone_weak()
    }

    pub fn get(&self, color: &Point3<f32>, opacity: f32) -> Option<Handle<BevyMaterial>> {
        let key = color
            .coords
            .push(opacity)
            .map(|c| (c * 255.0) as usize)
            .into();
        self.cache.get(&key).map(|h| h.clone_weak())
    }

    pub fn clear(&mut self) {
        self.cache.clear();
    }
}

pub const SELECTED_OBJECT_COLOR: Point3<f32> = point![1.0, 0.0, 0.0];

pub struct GraphicsManager {
    rand: Pcg32,
    b2sn: HashMap<RigidBodyHandle, Vec<EntityWithGraphics>>,
    b2color: HashMap<RigidBodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    b2wireframe: HashMap<RigidBodyHandle, bool>,
    ground_color: Point3<f32>,
    prefab_meshes: HashMap<ShapeType, Handle<Mesh>>,
    instanced_materials: InstancedMaterials,
    pub gfx_shift: Vector<Real>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        GraphicsManager {
            rand: Pcg32::new(0, 1),
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            ground_color: point![0.5, 0.5, 0.5],
            b2wireframe: HashMap::new(),
            prefab_meshes: HashMap::new(),
            instanced_materials: Default::default(),
            gfx_shift: Vector::zeros(),
        }
    }

    pub fn selection_material(&self) -> Handle<BevyMaterial> {
        self.instanced_materials
            .get(&SELECTED_OBJECT_COLOR, 1.0)
            .unwrap()
            .clone_weak()
    }

    pub fn clear(&mut self, commands: &mut Commands) {
        for sns in self.b2sn.values_mut() {
            for sn in sns.iter_mut() {
                commands.entity(sn.entity).despawn()
            }
        }

        self.instanced_materials.clear();
        self.b2sn.clear();
        self.c2color.clear();
        self.b2color.clear();
        self.b2wireframe.clear();
        self.rand = Pcg32::new(0, 1);
    }

    pub fn remove_collider_nodes(
        &mut self,
        commands: &mut Commands,
        body: Option<RigidBodyHandle>,
        collider: ColliderHandle,
    ) {
        let body = body.unwrap_or(RigidBodyHandle::invalid());
        if let Some(sns) = self.b2sn.get_mut(&body) {
            sns.retain(|sn| {
                if let Some(sn_c) = sn.collider
                    && sn_c == collider
                {
                    commands.entity(sn.entity).despawn();
                    return false;
                }

                true
            });
        }
    }

    pub fn remove_body_nodes(&mut self, commands: &mut Commands, body: RigidBodyHandle) {
        if let Some(sns) = self.b2sn.get_mut(&body) {
            for sn in sns.iter_mut() {
                commands.entity(sn.entity).despawn();
            }
        }

        self.b2sn.remove(&body);
    }

    pub fn set_body_color(
        &mut self,
        materials: &mut Assets<BevyMaterial>,
        material_handles: &mut Query<&mut BevyMaterialComponent>,
        b: RigidBodyHandle,
        color: [f32; 3],
    ) {
        self.b2color.insert(b, color.into());

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut() {
                n.set_color(materials, &mut self.instanced_materials, color.into());

                if let Ok(mut mat) = material_handles.get_mut(n.entity) {
                    mat.0 = n.material.clone_weak();
                }
            }
        }
    }

    pub fn set_initial_body_color(&mut self, b: RigidBodyHandle, color: [f32; 3]) {
        self.b2color.insert(b, color.into());
    }

    pub fn set_initial_collider_color(&mut self, c: ColliderHandle, color: [f32; 3]) {
        self.c2color.insert(c, color.into());
    }

    pub fn set_body_wireframe(&mut self, b: RigidBodyHandle, enabled: bool) {
        self.b2wireframe.insert(b, enabled);

        if let Some(_ns) = self.b2sn.get_mut(&b) {
            // for n in ns.iter_mut().filter_map(|n| n.scene_node_mut()) {
            //     if enabled {
            //         n.set_surface_rendering_activation(true);
            //         n.set_lines_width(1.0);
            //     } else {
            //         n.set_surface_rendering_activation(false);
            //         n.set_lines_width(1.0);
            //     }
            // }
        }
    }

    pub fn toggle_wireframe_mode(&mut self, _colliders: &ColliderSet, _enabled: bool) {
        for _n in self.b2sn.values_mut().flat_map(|val| val.iter_mut()) {
            // let _force_wireframe = if let Some(collider) = colliders.get(n.collider) {
            //     collider.is_sensor()
            //         || self
            //             .b2wireframe
            //             .get(&collider.parent())
            //             .cloned()
            //             .unwrap_or(false)
            // } else {
            //     false
            // };

            // if let Some(node) = n.scene_node_mut() {
            //     if force_wireframe || enabled {
            //         node.set_lines_width(1.0);
            //         node.set_surface_rendering_activation(false);
            //     } else {
            //         node.set_lines_width(0.0);
            //         node.set_surface_rendering_activation(true);
            //     }
            // }
        }
    }

    pub fn next_color(&mut self) -> Point3<f32> {
        Self::gen_color(&mut self.rand)
    }

    fn gen_color(rng: &mut Pcg32) -> Point3<f32> {
        use rand::Rng;
        let mut color: Point3<f32> = rng.random();

        // Quantize the colors a bit to get some amount of auto-instancing from bevy.
        color.x = (color.x * 5.0).round() / 5.0;
        color.y = (color.y * 5.0).round() / 5.0;
        color.z = (color.z * 5.0).round() / 5.0;
        color
    }

    fn alloc_color(&mut self, handle: RigidBodyHandle, is_fixed: bool) -> Point3<f32> {
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
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        components: &mut Query<&mut Transform>,
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

        let _ = self.add_body_colliders_with_color(
            commands, meshes, materials, components, handle, bodies, colliders, color,
        );
    }

    pub fn add_body_colliders_with_color(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        components: &mut Query<&mut Transform>,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        color: Point3<f32>,
    ) -> Vec<EntityWithGraphics> {
        let mut new_nodes = Vec::new();

        for collider_handle in bodies[handle].colliders() {
            let color = self.c2color.get(collider_handle).copied().unwrap_or(color);
            let collider = &colliders[*collider_handle];
            self.add_shape(
                commands,
                meshes,
                materials,
                Some(*collider_handle),
                collider.shape(),
                collider.is_sensor(),
                collider.position(),
                &Isometry::identity(),
                color,
                &mut new_nodes,
            );
        }

        new_nodes
            .iter_mut()
            .for_each(|n| n.update(colliders, components, &self.gfx_shift));

        // for node in new_nodes.iter_mut().filter_map(|n| n.scene_node_mut()) {
        //     if self.b2wireframe.get(&handle).cloned() == Some(true) {
        //         node.set_lines_width(1.0);
        //         node.set_surface_rendering_activation(false);
        //     } else {
        //         node.set_lines_width(0.0);
        //         node.set_surface_rendering_activation(true);
        //     }
        // }

        let nodes = self.b2sn.entry(handle).or_default();

        nodes.append(&mut new_nodes.clone());

        new_nodes
    }

    pub fn add_collider(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        handle: ColliderHandle,
        colliders: &ColliderSet,
    ) {
        let collider = &colliders[handle];
        let collider_parent = collider.parent().unwrap_or(RigidBodyHandle::invalid());
        let color = self
            .b2color
            .get(&collider_parent)
            .copied()
            .unwrap_or(self.ground_color);
        let color = self.c2color.get(&handle).copied().unwrap_or(color);
        let mut nodes = std::mem::take(self.b2sn.entry(collider_parent).or_default());
        self.add_shape(
            commands,
            meshes,
            materials,
            Some(handle),
            collider.shape(),
            collider.is_sensor(),
            collider.position(),
            &Isometry::identity(),
            color,
            &mut nodes,
        );
        self.b2sn.insert(collider_parent, nodes);
    }

    pub fn add_shape(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        handle: Option<ColliderHandle>,
        shape: &dyn Shape,
        sensor: bool,
        pos: &Isometry<Real>,
        delta: &Isometry<Real>,
        color: Point3<f32>,
        out: &mut Vec<EntityWithGraphics>,
    ) {
        if let Some(compound) = shape.as_compound() {
            for (shape_pos, shape) in compound.shapes() {
                self.add_shape(
                    commands,
                    meshes,
                    materials,
                    handle,
                    &**shape,
                    sensor,
                    pos,
                    &(shape_pos * delta),
                    color,
                    out,
                )
            }
        } else {
            if self.prefab_meshes.is_empty() {
                EntityWithGraphics::gen_prefab_meshes(&mut self.prefab_meshes, meshes);
            }

            let node = EntityWithGraphics::spawn(
                commands,
                meshes,
                materials,
                &self.prefab_meshes,
                &mut self.instanced_materials,
                shape,
                handle,
                *pos,
                *delta,
                color,
                sensor,
            );
            out.push(node);
        }
    }

    pub fn draw(
        &mut self,
        flags: TestbedStateFlags,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        components: &mut Query<&mut Transform>,
        visibilities: &mut Query<&mut Visibility>,
        _materials: &mut Assets<BevyMaterial>,
    ) {
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                // if let Some(bo) = n
                //     .collider
                //     .and_then(|h| bodies.get(colliders.get(h)?.parent()?))
                // {
                //     if bo.activation().time_since_can_sleep
                //         >= RigidBodyActivation::default_time_until_sleep()
                //     {
                //         n.set_color(materials, point![1.0, 0.0, 0.0]);
                //     }
                //     /* else if bo.activation().energy < bo.activation().threshold {
                //         n.set_color(materials, point![0.0, 0.0, 1.0]);
                //     } */
                //     else {
                //         n.set_color(materials, point![0.0, 1.0, 0.0]);
                //     }
                // }

                if let Ok(mut vis) = visibilities.get_mut(n.entity) {
                    if flags.contains(TestbedStateFlags::DRAW_SURFACES) {
                        *vis = Visibility::Inherited;
                    } else {
                        *vis = Visibility::Hidden;
                    }
                }

                n.update(colliders, components, &self.gfx_shift);
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

    //                 window.draw_line(center, &(*center + x), &point![1.0, 0.0, 0.0]);
    //                 window.draw_line(center, &(*center + y), &point![0.0, 1.0, 0.0]);
    //                 window.draw_line(center, &(*center + z), &point![0.0, 0.0, 1.0]);
    //             // }
    //         }
    //     }
    // }

    pub fn body_nodes(&self, handle: RigidBodyHandle) -> Option<&Vec<EntityWithGraphics>> {
        self.b2sn.get(&handle)
    }

    pub fn body_nodes_mut(
        &mut self,
        handle: RigidBodyHandle,
    ) -> Option<&mut Vec<EntityWithGraphics>> {
        self.b2sn.get_mut(&handle)
    }

    pub fn nodes(&self) -> impl Iterator<Item = &EntityWithGraphics> {
        self.b2sn.values().flat_map(|val| val.iter())
    }

    pub fn nodes_mut(&mut self) -> impl Iterator<Item = &mut EntityWithGraphics> {
        self.b2sn.values_mut().flat_map(|val| val.iter_mut())
    }

    pub fn prefab_meshes(&self) -> &HashMap<ShapeType, Handle<Mesh>> {
        &self.prefab_meshes
    }

    pub fn prefab_meshes_mut(&mut self) -> &mut HashMap<ShapeType, Handle<Mesh>> {
        &mut self.prefab_meshes
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
